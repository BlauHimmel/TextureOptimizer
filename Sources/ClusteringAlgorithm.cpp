#pragma warning(disable:4244 4267)
#include "ClusteringAlgorithm.hpp"

#include <igl/edge_topology.h>
#include <tbb/parallel_for.h>

#include <queue>
#include <utility>
#include <algorithm>

DualNodeType::DualNodeType(TriMesh * pMesh, const std::unordered_map<std::pair<int, int>, double, PairHasher> & intersectLength,
    const std::vector<double> & faceAreas, const std::vector<double> & edgeLengths, int facet,
    Eigen::Matrix3d A, const Eigen::Vector3d & b, double c) : A(std::move(A)), b(b), c(c)
{
    faces = std::unordered_set<int>();
    faces.insert(facet);
    pAttachedMesh = pMesh;
    pFaceAreas = &faceAreas;
    pEdgeLengths = &edgeLengths;
    pIntersectLength = &intersectLength;

    /*area = 0.0;
    for (int faceIdx : faces)
    {
        area += faceAreas[faceIdx];
    }

    perimeter = 0.0;
    std::set<int> boundaryEdges;
    std::unordered_map<int, int> edgeCount;

    for (int faceIdx : faces)
    {
        const auto fh = TriMesh::FaceHandle(faceIdx);
        for (const auto eh : pMesh->fe_range(fh))
        {
            edgeCount[eh.idx()]++;
            if (edgeCount[eh.idx()] == 1)
            {
                boundaryEdges.insert(eh.idx());
            }
            else
            {
                boundaryEdges.erase(eh.idx());
            }
        }
    }

    for (int edgeIdx : boundaryEdges)
    {
        perimeter += edgeLengths[edgeIdx];
    }

   irregularity = (perimeter * perimeter) / (4.0 * M_PI * area);*/
}

DualNodeType DualNodeType::operator+(const DualNodeType & rhs) const
{
    DualNodeType res;
    res.faces = faces;
    for (int face : rhs.faces)
    {
        res.faces.insert(face);
    }
    res.A = A + rhs.A;
    res.b = b + rhs.b;
    res.c = c + rhs.c;
    /*res.area = area + rhs.area;
    res.perimeter = perimeter + rhs.perimeter;
    res.perimeter -= 2.0 * (*pIntersectLength).find(std::make_pair(id, rhs.id))->second;
    res.irregularity = (perimeter * perimeter) / (4.0 * M_PI * area);*/

    res.pAttachedMesh = pAttachedMesh;
    res.pFaceAreas = pFaceAreas;
    res.pEdgeLengths = pEdgeLengths;
    res.pIntersectLength = pIntersectLength;

    return res;
}

std::vector<std::vector<int>> ClusteringAlgorithm::Cluster(TriMesh * pMesh, size_t clusterNum, double threshold)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    F.resize(pMesh->n_faces(), 3);
    for (int i = 0; i < pMesh->n_faces(); i++)
    {
        const auto fh = TriMesh::FaceHandle(i);

        auto fvIter = pMesh->fv_range(fh).begin();
        auto fvIterEnd = pMesh->fv_range(fh).end();

        const auto vh0 = *fvIter;
        *fvIter++;
        const auto vh1 = *fvIter;
        *fvIter++;
        const auto vh2 = *fvIter;
        *fvIter++;

        F(i, 0) = vh0.idx();
        F(i, 1) = vh1.idx();
        F(i, 2) = vh2.idx();
    }

    V.resize(pMesh->n_vertices(), 3);
    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        V(i, 0) = point[0];
        V(i, 1) = point[1];
        V(i, 2) = point[2];
    }

    std::vector<double> faceAreas;
    for (int i = 0; i < pMesh->n_faces(); i++)
    {
        const auto fh = TriMesh::FaceHandle(i);
        faceAreas.push_back(static_cast<double>(pMesh->calc_face_area(fh)));
    }

    std::vector<double> edgeLengths;
    for (int i = 0; i < pMesh->n_edges(); i++)
    {
        const auto eh = TriMesh::EdgeHandle(i);
        edgeLengths.push_back(static_cast<double>(pMesh->calc_edge_length(eh)));
    }

    std::unordered_map<std::pair<int, int>, double, PairHasher> intersectLength;
    for (const auto eh : pMesh->edges())
    {
        if (!eh.is_boundary())
        {
            const auto f0 = eh.h0().face();
            const auto f1 = eh.h1().face();
            double len = pMesh->calc_edge_length(eh);
            intersectLength[std::make_pair(f0.idx(), f1.idx())] = len;
            intersectLength[std::make_pair(f1.idx(), f0.idx())] = len;
        }
    }

    std::unordered_set<int> valid;
    std::vector<DualNodeType> dualNodes(F.rows());

    auto ComputeDualNodeKernel = [&](const tbb::blocked_range<int> & range)
    {
        for (int i = range.begin(); i != range.end(); i++)
        {
            int i0 = F(i, 0), i1 = F(i, 1), i2 = F(i, 2);
            DualNodeType dn0(pMesh, intersectLength, faceAreas, edgeLengths, i, V.row(i0).transpose().eval() * V.row(i0), V.row(i0).transpose(), 1);
            DualNodeType dn1(pMesh, intersectLength, faceAreas, edgeLengths, i, V.row(i1).transpose().eval() * V.row(i1), V.row(i1).transpose(), 1);
            DualNodeType dn2(pMesh, intersectLength, faceAreas, edgeLengths, i, V.row(i2).transpose().eval() * V.row(i2), V.row(i2).transpose(), 1);
            dualNodes[i] = dn0 + dn1 + dn2;
            dualNodes[i].id = i;
        }
    };

    tbb::blocked_range<int> computeDualNodeRange(0, static_cast<int>(F.rows()));
    tbb::parallel_for(computeDualNodeRange, ComputeDualNodeKernel);

    for (int i = 0; i < F.rows(); ++i)
    {
        valid.insert(i);
    }

    std::vector<std::pair<int, int>> dualEdges;
    GetDualEdges(V, F, dualEdges);

    std::vector<std::vector<int>> dualAdjacencyList(dualNodes.size());
    std::priority_queue<std::pair<double, int>> pq;
    std::vector<double> costs(dualEdges.size());

    auto ComputeDualEdgeCostKernel = [&](tbb::blocked_range<int> & range)
    {
        for (int i = range.begin(); i != range.end(); i++)
        {
            const auto & p = dualEdges[i];
            const DualNodeType & n1 = dualNodes[p.first];
            const DualNodeType & n2 = dualNodes[p.second];
            costs[i] = EvaluateCost(n1, n2, intersectLength);
        }
    };

    tbb::blocked_range<int> computeDualEdgeCostRange(0, static_cast<int>(dualEdges.size()));
    tbb::parallel_for(computeDualEdgeCostRange, ComputeDualEdgeCostKernel);

    for (int i = 0; i < dualEdges.size(); ++i)
    {
        const auto & p = dualEdges[i];
        pq.push({ -costs[i], i });
        dualAdjacencyList[p.first].push_back(p.second);
        dualAdjacencyList[p.second].push_back(p.first);
    }

    while (!pq.empty())
    {
        double cost = -pq.top().first;
        if (cost > threshold) { break; }
        if (valid.size() <= clusterNum) { break; }
        auto p = dualEdges[pq.top().second];
        pq.pop();
        int ndID0 = p.first, ndID1 = p.second;
        if (!valid.count(ndID0) || !valid.count(ndID1)) { continue; }

        auto newID = static_cast<int>(dualNodes.size());
        dualNodes.emplace_back(dualNodes[ndID0] + dualNodes[ndID1]);
        dualNodes[newID].id = newID;

        std::vector<int> newNeighborhood;
        for (int neighbor : dualAdjacencyList[ndID0])
        {
            if (valid.count(neighbor) && neighbor != ndID1)
            {
                newNeighborhood.push_back(neighbor);
                dualAdjacencyList[neighbor].push_back(newID);

                double newCost = EvaluateCost(dualNodes[ndID0], dualNodes[neighbor], intersectLength);
                pq.push({ -newCost, static_cast<int>(dualEdges.size()) });
                dualEdges.emplace_back(neighbor, newID);

                /*double oldIntersectLength = intersectLength[std::make_pair(neighbor, ndID0)];
                intersectLength[std::make_pair(neighbor, newID)] += oldIntersectLength;
                intersectLength[std::make_pair(newID, neighbor)] += oldIntersectLength;
                intersectLength.erase(std::make_pair(neighbor, ndID0));
                intersectLength.erase(std::make_pair(ndID0, neighbor));*/
            }
        }

        for (int neighbor : dualAdjacencyList[ndID1])
        {
            if (valid.count(neighbor) && neighbor != ndID0)
            {
                newNeighborhood.push_back(neighbor);
                dualAdjacencyList[neighbor].push_back(newID);

                double newCost = EvaluateCost(dualNodes[ndID1], dualNodes[neighbor], intersectLength);
                pq.push({ -newCost, static_cast<int>(dualEdges.size()) });
                dualEdges.emplace_back(neighbor, newID);

                /*double oldIntersectLength = intersectLength[std::make_pair(neighbor, ndID1)];
                intersectLength[std::make_pair(neighbor, newID)] += oldIntersectLength;
                intersectLength[std::make_pair(newID, neighbor)] += oldIntersectLength;
                intersectLength.erase(std::make_pair(neighbor, ndID1));
                intersectLength.erase(std::make_pair(ndID1, neighbor));*/
            }
        }

        /*intersectLength.erase(std::make_pair(ndID0, ndID1));
        intersectLength.erase(std::make_pair(ndID1, ndID0));*/

        dualAdjacencyList.push_back(newNeighborhood);

        valid.insert(newID);
        valid.erase(ndID0), valid.erase(ndID1);
        dualAdjacencyList[ndID0].clear();
        dualAdjacencyList[ndID1].clear();
    }

    std::vector<std::vector<int>> res;
    for (auto ndID : valid)
    {
        std::vector<int> patch;
        for (auto face : dualNodes[ndID].faces)
        {
            patch.push_back(face);
        }
        res.push_back(patch);
    }
    return res;
}

void ClusteringAlgorithm::GetDualEdges(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, std::vector<std::pair<int, int>> & dualEdges)
{
    Eigen::MatrixXi EV, FE, EF;
    igl::edge_topology(V, F, EV, FE, EF);

    dualEdges.clear();
    for (int i = 0; i < EF.rows(); ++i)
    {
        int v0 = EF(i, 0), v1 = EF(i, 1);
        if (v0 == -1 || v1 == -1) { continue; }
        dualEdges.emplace_back(v0, v1);
    }
}

double ClusteringAlgorithm::EvaluateCost(const DualNodeType & dn1, const DualNodeType & dn2, const std::unordered_map<std::pair<int, int>, double, PairHasher> & intersectLength)
{
    Eigen::Matrix3d A = dn1.A + dn2.A;
    Eigen::Vector3d b = dn1.b + dn2.b;
    double c = dn1.c + dn2.c;

    Eigen::Matrix3d Z = A - b * b.transpose().eval() / c;
    Eigen::EigenSolver<Eigen::Matrix3d> eigenSolver(Z);
    std::vector<double> eigenValues(3);
    std::vector<Eigen::Vector3d> eigenVectors(3);

    for (int i = 0; i < 3; i++)
    {
        eigenValues[i] = eigenSolver.eigenvalues()[i].real();
        Eigen::Vector3cd vcd = eigenSolver.eigenvectors().col(i);
        for (int j = 0; j < 3; ++j)
        {
            eigenVectors[i](j) = vcd(j).real();
        }
        eigenVectors[i].normalize();
    }

    for (int i = 0; i < 3; ++i)
    {
        for (int j = i + 1; j < 3; ++j)
        {
            if (eigenValues[i] > eigenValues[j])
            {
                std::swap(eigenValues[i], eigenValues[j]);
                std::swap(eigenVectors[i], eigenVectors[j]);
            }
        }
    }

    Eigen::Vector3d n = eigenVectors[0];
    double d = -n.transpose().dot(b) / c;
    double energyFit = ((n.transpose().eval() * A).dot(n) + 2 * d * n.transpose().eval().dot(b)) / c + d * d;

    /*double area = dn1.area + dn2.area;
    double perimeter = dn1.perimeter + dn2.perimeter;
    perimeter -= 2.0 * intersectLength.find(std::make_pair(dn1.id, dn2.id))->second;
    double irregularity = (perimeter * perimeter) / (4.0 * M_PI * area);
    double irregularity1 = dn1.irregularity;
    double irregularity2 = dn2.irregularity;

    double energyShape = (irregularity - std::max(irregularity1, irregularity2)) / irregularity;*/

    double energy = energyFit/* + energyShape * 0.5*/;

    return energy;
}