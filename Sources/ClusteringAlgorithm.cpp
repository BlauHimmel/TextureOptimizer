#pragma warning(disable:4244 4267)
#include "ClusteringAlgorithm.hpp"

#include <igl/edge_topology.h>

#include <queue>
#include <utility>

DualNodeType::DualNodeType(int facet, Eigen::Matrix3d A, const Eigen::Vector3d & b, double c) : A(std::move(A)), b(b), c(c)
{
    faces = std::unordered_set<int>();
    faces.insert(facet);
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

    std::vector<DualNodeType> dualNodes;
    std::unordered_set<int> valid;
    for (int i = 0; i < F.rows(); ++i)
    {
        int i0 = F(i, 0), i1 = F(i, 1), i2 = F(i, 2);
        DualNodeType dn0(i, V.row(i0).transpose().eval() * V.row(i0), V.row(i0).transpose(), 1);
        DualNodeType dn1(i, V.row(i1).transpose().eval() * V.row(i1), V.row(i1).transpose(), 1);
        DualNodeType dn2(i, V.row(i2).transpose().eval() * V.row(i2), V.row(i2).transpose(), 1);
        dualNodes.emplace_back(dn0 + dn1 + dn2);
        valid.insert(i);
    }

    std::vector<std::pair<int, int>> dualEdges;
    GetDualEdges(V, F, dualEdges);

    std::vector<std::vector<int>> dualAdjacencyList(dualNodes.size());
    std::priority_queue<std::pair<double, int>> pq;
    for (int i = 0; i < dualEdges.size(); ++i)
    {
        const auto & p = dualEdges[i];
        const DualNodeType & n1 = dualNodes[p.first];
        const DualNodeType & n2 = dualNodes[p.second];
        double cost = EvaluateCost(n1, n2);
        pq.push({ -cost, i });

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

        std::vector<int> newNeighborhood;
        for (int neighbor : dualAdjacencyList[ndID0])
        {
            if (valid.count(neighbor) && neighbor != ndID1)
            {
                newNeighborhood.push_back(neighbor);
                dualAdjacencyList[neighbor].push_back(newID);

                double newCost = EvaluateCost(dualNodes[ndID0], dualNodes[neighbor]);
                pq.push({ -newCost, static_cast<int>(dualEdges.size()) });
                dualEdges.emplace_back(neighbor, newID);
            }
        }

        for (int neighbor : dualAdjacencyList[ndID1])
        {
            if (valid.count(neighbor) && neighbor != ndID0)
            {
                newNeighborhood.push_back(neighbor);
                dualAdjacencyList[neighbor].push_back(newID);

                double newCost = EvaluateCost(dualNodes[ndID1], dualNodes[neighbor]);
                pq.push({ -newCost, static_cast<int>(dualEdges.size()) });
                dualEdges.emplace_back(neighbor, newID);
            }
        }

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

double ClusteringAlgorithm::EvaluateCost(const DualNodeType & dn1, const DualNodeType & dn2)
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

    return ((n.transpose().eval() * A).dot(n) + 2 * d * n.transpose().eval().dot(b)) / c + d * d;
}