#include "MeshUtils.hpp"

#include <mutex>
#include <thread>

#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include <progress_bar.h>
#include <SLIMWrapper.hpp>
#include <EBPWrapper.h>
#include <SMI04Wrapper.h>

#include "CutToDiskImpl.hpp"
#include "Logger.hpp"

std::vector<std::vector<OpenMesh::VertexHandle>> MeshComponentVertices(const TriMesh * pMesh)
{
    TriMesh mesh(*pMesh);
    std::vector<std::vector<OpenMesh::VertexHandle>> componentVertices;

    OpenMesh::VPropHandleT<bool> visitedVProp;
    mesh.add_property(visitedVProp);
    for (auto vh : mesh.vertices())
    {
        mesh.property(visitedVProp, vh) = false;
    }

    auto vertexIter = mesh.vertices_begin();

    while (true)
    {
        /* find an unvisited vertex */
        bool bFound = false;
        OpenMesh::VertexHandle foundVH;
        for (auto iter = vertexIter; iter != mesh.vertices_end(); ++iter)
        {
            if (!mesh.property(visitedVProp, *iter))
            {
                bFound = true;
                foundVH = *iter;
                mesh.property(visitedVProp, *iter) = true;
                vertexIter = iter;
                break;
            }
        }

        /* if not found, then finished */
        if (!bFound) { break; }

        std::vector<OpenMesh::VertexHandle> handles;
        std::vector<OpenMesh::VertexHandle> component;
        handles.push_back(foundVH);

        /* grow from found vertex */
        while (!handles.empty())
        {
            auto currentVH = handles.back();
            handles.pop_back();
            component.push_back(currentVH);

            for (auto iter = mesh.vv_iter(currentVH); iter.is_valid(); ++iter)
            {
                if (!mesh.property(visitedVProp, *iter))
                {
                    mesh.property(visitedVProp, *iter) = true;
                    handles.push_back(*iter);
                }
            }
        }

        componentVertices.push_back(component);
    }

    return componentVertices;
}

std::vector<size_t> MeshComponentVertexNums(const TriMesh * pMesh)
{
    auto compVertices = MeshComponentVertices(pMesh);
    std::vector<size_t> vertexNums;
    vertexNums.reserve(compVertices.size());
    for (const auto & comp : compVertices)
    {
        vertexNums.push_back(comp.size());
    }
    return vertexNums;
}

size_t MeshComponentCount(const TriMesh * pMesh)
{
    return MeshComponentVertices(pMesh).size();
}

std::vector<std::vector<OpenMesh::VertexHandle>> MeshBoundaryVertices(const TriMesh * pMesh)
{
    TriMesh mesh(*pMesh);
    std::vector<std::vector<OpenMesh::VertexHandle>> boundaryVertices;

    OpenMesh::HPropHandleT<bool> visitedHProp;
    mesh.add_property(visitedHProp);
    for (auto heh : mesh.halfedges())
    {
        mesh.property(visitedHProp, heh) = false;
    }

    for (auto heh : mesh.halfedges())
    {
        if (mesh.property(visitedHProp, heh)) { continue; }
        if (!mesh.is_boundary(heh)) { continue; }

        std::vector<OpenMesh::VertexHandle> boundary;
        boundary.push_back(mesh.from_vertex_handle(heh));
        mesh.property(visitedHProp, heh) = true;

        auto nextHeh = mesh.next_halfedge_handle(heh);
        while (mesh.is_boundary(nextHeh) && !mesh.property(visitedHProp, nextHeh))
        {
            boundary.push_back(mesh.from_vertex_handle(nextHeh));
            mesh.property(visitedHProp, nextHeh) = true;
            nextHeh = mesh.next_halfedge_handle(nextHeh);
        }
        boundaryVertices.push_back(boundary);
    }
    mesh.remove_property(visitedHProp);

    return boundaryVertices;
}

std::vector<size_t> MeshBoundaryLengths(const TriMesh * pMesh)
{
    auto boundaryVertices = MeshBoundaryVertices(pMesh);
    std::vector<size_t> boundaryLengths;
    boundaryLengths.reserve(boundaryVertices.size());
    for (const auto & boundary : boundaryVertices)
    {
        boundaryLengths.push_back(boundary.size());
    }
    return boundaryLengths;
}

std::vector<Real> MeshBoundaryDistances(const TriMesh * pMesh)
{
    auto boundaryVertices = MeshBoundaryVertices(pMesh);
    std::vector<Real> boundaryDistances;
    boundaryDistances.reserve(boundaryVertices.size());
    for (const auto & boundary : boundaryVertices)
    {
        Real distance = 0.0;
        for (size_t i = 0; i < boundary.size(); i++)
        {
            const auto & v0 = pMesh->point(boundary[i]);
            const auto & v1 = pMesh->point(boundary[(i + 1) % boundary.size()]);
            distance += (v1 - v0).norm();
        }
        boundaryDistances.push_back(distance);
    }
    return boundaryDistances;
}

size_t MeshBoundaryCount(const TriMesh * pMesh)
{
    return MeshBoundaryLengths(pMesh).size();
}

void RemoveSmallComponent(TriMesh * pMesh)
{
    auto componentVHs = MeshComponentVertices(pMesh);

    Logger::Info("%zd unique components found!", componentVHs.size());
    for (size_t i = 0; i < componentVHs.size(); i++)
    {
        Logger::Info("Component %zd: %zd vertices.", i, componentVHs[i].size());
    }

    if (componentVHs.size() <= 1) { return; }

    size_t maxComponentIdx(-1);
    size_t maxComponentVertexNum(0);
    for (size_t i = 0; i < componentVHs.size(); i++)
    {
        if (componentVHs[i].size() > maxComponentVertexNum)
        {
            maxComponentVertexNum = componentVHs[i].size();
            maxComponentIdx = i;
        }
    }

    pMesh->request_face_status();
    pMesh->request_edge_status();
    pMesh->request_vertex_status();

    for (size_t i = 0; i < componentVHs.size(); i++)
    {
        if (i == maxComponentIdx) { continue; }
        for (auto componentVH : componentVHs[i])
        {
            pMesh->delete_vertex(componentVH);
        }
    }

    pMesh->garbage_collection();
    pMesh->release_face_status();
    pMesh->release_edge_status();
    pMesh->release_vertex_status();

    Logger::Info("Component %zd (%zd vertices) was reserved, others were removed.", maxComponentIdx, maxComponentVertexNum);
}

uint32_t RepairNonManifold(TriMesh * pMesh)
{
    if (pMesh == nullptr) { return 0; }

    std::vector<OpenMesh::VertexHandle> nonManifoldVHs;
    for (auto vh : pMesh->vertices())
    {
        if (!pMesh->is_manifold(vh))
        {
            nonManifoldVHs.push_back(vh);
        }
    }

    Logger::Info("%zd non-manifold vertices found!", nonManifoldVHs.size());

    if (nonManifoldVHs.empty()) { return 0; }

    uint32_t newVertexNum = 0;

    OpenMesh::FPropHandleT<size_t> componentFProp;
    pMesh->add_property(componentFProp);
    pMesh->request_face_status();
    pMesh->request_edge_status();
    pMesh->request_vertex_status();

    for (auto vh : pMesh->vertices())
    {
        /* unmark all faces around the vertices */
        for (auto fh : pMesh->vf_range(vh))
        {
            pMesh->property(componentFProp, fh) = 0;
        }

        size_t componentCount = 1;

        /* search and isolate new components, then shared vertices will be duplicated */
        for (auto fh : pMesh->vf_range(vh))
        {
            /* find the first face in the component */
            std::vector<OpenMesh::FaceHandle> checkedNeighbour;
            if (pMesh->property(componentFProp, fh) == 0)
            {
                pMesh->property(componentFProp, fh) = componentCount;
                checkedNeighbour.push_back(fh);
            }

            /* if a reference face was found, a new component exists and
               a new vertex is required (except for the 1st component) */
            OpenMesh::VertexHandle newVH;
            if (componentCount > 1 && !checkedNeighbour.empty())
            {
                TriMesh::Point pt = pMesh->point(vh);
                newVH = pMesh->add_vertex(pt);
                newVertexNum++;
            }

            /* check all adjacent faces of the reference */
            while (!checkedNeighbour.empty())
            {
                OpenMesh::FaceHandle currFH = checkedNeighbour.back();
                checkedNeighbour.pop_back();

                /* get all neighbour faces of the current face */
                std::vector<OpenMesh::VertexHandle> adjVHs;
                for (auto adjVH : pMesh->fv_range(currFH))
                {
                    adjVHs.push_back(adjVH);
                    if (adjVH != vh)
                    {
                        /* find the next neighbour face over edge <adjVH-vh> */
                        OpenMesh::FaceHandle neighbourFH;
                        for (auto neighbourFHIter = pMesh->vf_begin(vh);
                             neighbourFHIter.is_valid() && !neighbourFH.is_valid();
                             ++neighbourFHIter)
                        {
                            if (*neighbourFHIter != currFH)
                            {
                                for (auto neighbourVHIter = pMesh->fv_begin(*neighbourFHIter);
                                     neighbourVHIter.is_valid() && !neighbourFH.is_valid();
                                     ++neighbourVHIter)
                                {
                                    if (*neighbourVHIter == adjVH)
                                    {
                                        neighbourFH = *neighbourFHIter;
                                    }
                                }
                            }
                        }

                        /* if such face was found, it is in the same component as the reference face */
                        if (neighbourFH.is_valid() && pMesh->property(componentFProp, neighbourFH) == 0)
                        {
                            pMesh->property(componentFProp, neighbourFH) = componentCount;
                            checkedNeighbour.push_back(neighbourFH);
                        }
                    }
                }

                /* if one face was not found in the 1st component, then it is a new component */
                if (componentCount > 1 && newVH.is_valid())
                {
                    std::replace(adjVHs.begin(), adjVHs.end(), OpenMesh::VertexHandle(vh), newVH);
                    pMesh->delete_face(currFH, false);
                    pMesh->add_face(adjVHs);
                }
            }

            /* all faces which belong to 'vh' and inside the same component found
               the next face will be in a new component */
            componentCount++;
        }
    }

    pMesh->garbage_collection();
    pMesh->remove_property(componentFProp);
    pMesh->release_face_status();
    pMesh->release_edge_status();
    pMesh->release_vertex_status();

    Logger::Info("Non-manifold fixed, %u new vertices created!", newVertexNum);
    return newVertexNum;
}

bool CutToDisk(TriMesh * pMesh, const Real * pTotalArea, size_t seedFaceNum)
{
    size_t faceIdxIncrement = pMesh->n_faces() / seedFaceNum;

    std::vector<size_t> seedIdxs;
    for (size_t seedIdx = 0; seedIdx < pMesh->n_faces(); seedIdx += faceIdxIncrement)
    {
        seedIdxs.push_back(seedIdx);
    }

    bmc::progress_bar progressBar(0, seedIdxs.size(), 80, false, true);
    std::mutex mutex;

    Real totalArea = 0.0;
    if (pTotalArea == nullptr)
    {
        for (const auto fh : pMesh->faces())
        {
            totalArea += pMesh->calc_face_area(fh);
        }
    }
    else
    {
        totalArea = *pTotalArea;
    }

    auto KernelFunc = [&seedIdxs, &pMesh, &mutex, &progressBar, totalArea](tbb::blocked_range<size_t> & range, std::pair<size_t, Real> value) -> std::pair<size_t, Real>
    {
        for (size_t i = range.begin(); i != range.end(); i++)
        {
            mutex.lock();
            progressBar++;
            mutex.unlock();

            Real boundaryDistances = CutToDiskSeamLengthImpl(pMesh, seedIdxs[i]);
            Real priority = std::pow(boundaryDistances, 2.0) / (4.0 * M_PI * totalArea);

            if (priority < value.second)
            {
                value.first = seedIdxs[i];
                value.second = priority;
            }
        }

        return value;
    };

    auto ReduceFunc = [](std::pair<size_t, Real> a, std::pair<size_t, Real> b) -> std::pair<size_t, Real>
    {
        return a.second < b.second ? a : b;
    };

    bool bStopped = false;
    std::thread progressThread([](bmc::progress_bar & progress, bool & stopped)
    {
        while (!stopped)
        {
            std::cout << progress;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }, std::ref(progressBar), std::ref(bStopped));

    tbb::blocked_range<size_t> range(0, seedIdxs.size());
    auto result = tbb::parallel_reduce(range, std::make_pair(size_t(-1), std::numeric_limits<Real>::max()), KernelFunc, ReduceFunc);
    bStopped = true;
    progressThread.join();
    std::cout << progressBar << std::endl;

    if (result.first == size_t(-1) || result.second == std::numeric_limits<Real>::max())
    {
        return false;
    }

    CutToDiskImpl(pMesh, result.first);
    return true;
}

int32_t GetMeshEuler(const TriMesh * pMesh)
{
    return static_cast<int32_t>(pMesh->n_vertices()) - static_cast<int32_t>(pMesh->n_edges()) + static_cast<int32_t>(pMesh->n_faces());
}

int32_t GetMeshGenus(const TriMesh * pMesh)
{
    int32_t euler = GetMeshEuler(pMesh);
    return (2 - euler - static_cast<int32_t>(MeshBoundaryCount(pMesh))) / 2;
}

bool MeshUVTransfer(TriMesh * pMesh, const TriMesh * pUVMesh)
{
    /* Try best to ensure Mesh and UVMesh have the same topology. */
    if (pMesh->n_vertices() != pUVMesh->n_vertices() ||
        pMesh->n_edges() != pUVMesh->n_edges() ||
        pMesh->n_faces() != pUVMesh->n_faces() ||
        pMesh->n_halfedges() != pUVMesh->n_halfedges())
    {
        Logger::Error("When transfer UV, input meshes must have same topology.");
        return false;
    }

    auto KernelFunc = [&](tbb::blocked_range<size_t> & range)
    {
        for (size_t i = range.begin(); i != range.end(); i++)
        {
            const auto heh = TriMesh::HalfedgeHandle(int(i));
            const auto vh = pUVMesh->to_vertex_handle(heh);
            const auto & point = pUVMesh->point(vh);
            TriMesh::TexCoord2D uv { point[0], point[1] };
            pMesh->set_texcoord2D(heh, uv);
        }
    };

    tbb::blocked_range<size_t> range(0, pMesh->n_halfedges());
    tbb::parallel_for(range, KernelFunc);

    return true;
}

void SLIMProxy(TriMesh * pMesh, Real * pWidth, Real * pHeith)
{
    int faceNumber = static_cast<int>(pMesh->n_faces());
    int vertexNumber =  static_cast<int>(pMesh->n_vertices());

    /* facet(i, j) : index of the j-th vertex at i-th triangle */
    std::vector<std::vector<size_t>> facets(faceNumber, std::vector<size_t>(3));

    for (int i = 0; i < faceNumber; i++)
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

        facets[i][0] = vh0.idx();
        facets[i][1] = vh1.idx();
        facets[i][2] = vh2.idx();
    }

    /* vertices(i, j) : value of the j-th component of position at i-th vertex */
    std::vector<std::vector<double>> vertices(vertexNumber, std::vector<double>(3));

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        vertices[i][0] = point[0];
        vertices[i][1] = point[1];
        vertices[i][2] = point[2];
    }

    const int ITERATIONS = 20;

    bmc::progress_bar progressBar(0, ITERATIONS, 80, false, true);

    bool bStopped = false;
    int currIterations = 0;
    std::thread progressThread([](bmc::progress_bar & progress, bool & stopped, int & current)
    {
        while (!stopped)
        {
            progress.set(current);
            std::cout << progress;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }, std::ref(progressBar), std::ref(bStopped), std::ref(currIterations));

    auto uv = SLIM(vertices, facets, ITERATIONS, &currIterations);

    bStopped = true;
    progressThread.join();
    progressBar.set(ITERATIONS);
    std::cout << progressBar << std::endl;

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle(i);
        auto & point = pMesh->point(vh);
        point[0] = uv[i][0];
        point[1] = uv[i][1];
        point[2] = 0.0;
    }

    Real xMin =  std::numeric_limits<Real>::max();
    Real yMin =  std::numeric_limits<Real>::max();
    Real xMax = -std::numeric_limits<Real>::max();
    Real yMax = -std::numeric_limits<Real>::max();

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        xMin = std::min(xMin, point[0]);
        xMax = std::max(xMax, point[0]);

        yMin = std::min(yMin, point[1]);
        yMax = std::max(yMax, point[1]);
    }

    Real invDelta = 1.0 / std::max(xMax - xMin, yMax - yMin);
    Real invDeltaX = 1.0 / (xMax - xMin);
    Real invDeltaY = 1.0 / (yMax - yMin);

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        auto & point = pMesh->point(vh);
        point[0] = (point[0] - xMin) * invDeltaX;
        point[1] = (point[1] - yMin) * invDeltaY;
    }

    if (pWidth != nullptr)
    {
        *pWidth = (xMax - xMin) * invDelta;
    }

    if (pHeith != nullptr)
    {
        *pHeith = (yMax - yMin) * invDelta;
    }
}

void EBPProxy(TriMesh * pMesh, Real * pWidth, Real * pHeith)
{
    int faceNumber = static_cast<int>(pMesh->n_faces());
    int vertexNumber =  static_cast<int>(pMesh->n_vertices());

    /* facet(i, j) : index of the j-th vertex at i-th triangle */
    std::vector<std::vector<size_t>> facets(faceNumber, std::vector<size_t>(3));

    for (int i = 0; i < faceNumber; i++)
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

        facets[i][0] = vh0.idx();
        facets[i][1] = vh1.idx();
        facets[i][2] = vh2.idx();
    }

    /* vertices(i, j) : value of the j-th component of position at i-th vertex */
    std::vector<std::vector<double>> vertices(vertexNumber, std::vector<double>(3));

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        vertices[i][0] = point[0];
        vertices[i][1] = point[1];
        vertices[i][2] = point[2];
    }

    auto uv = EBP(vertices, facets);

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle(i);
        auto & point = pMesh->point(vh);
        point[0] = uv[i][0];
        point[1] = uv[i][1];
        point[2] = 0.0;
    }

    Real xMin =  std::numeric_limits<Real>::max();
    Real yMin =  std::numeric_limits<Real>::max();
    Real xMax = -std::numeric_limits<Real>::max();
    Real yMax = -std::numeric_limits<Real>::max();

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        xMin = std::min(xMin, point[0]);
        xMax = std::max(xMax, point[0]);

        yMin = std::min(yMin, point[1]);
        yMax = std::max(yMax, point[1]);
    }

    Real invDelta = 1.0 / std::max(xMax - xMin, yMax - yMin);
    Real invDeltaX = 1.0 / (xMax - xMin);
    Real invDeltaY = 1.0 / (yMax - yMin);

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        auto & point = pMesh->point(vh);
        point[0] = (point[0] - xMin) * invDeltaX;
        point[1] = (point[1] - yMin) * invDeltaY;
    }

    if (pWidth != nullptr)
    {
        *pWidth = (xMax - xMin) * invDelta;
    }

    if (pHeith != nullptr)
    {
        *pHeith = (yMax - yMin) * invDelta;
    }
}

void SMI04Proxy(TriMesh * pMesh, Real * pWidth, Real * pHeith, Real * pSTL2error)
{
    int faceNumber = static_cast<int>(pMesh->n_faces());
    int vertexNumber =  static_cast<int>(pMesh->n_vertices());

    /* facet(i, j) : index of the j-th vertex at i-th triangle */
    std::vector<std::vector<size_t>> facets(faceNumber, std::vector<size_t>(3));

    for (int i = 0; i < faceNumber; i++)
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

        facets[i][0] = vh0.idx();
        facets[i][1] = vh1.idx();
        facets[i][2] = vh2.idx();
    }

    /* vertices(i, j) : value of the j-th component of position at i-th vertex */
    std::vector<std::vector<double>> vertices(vertexNumber, std::vector<double>(3));

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        vertices[i][0] = point[0];
        vertices[i][1] = point[1];
        vertices[i][2] = point[2];
    }

    SMI04Configuration config;
    config.iteNum = static_cast<int>(static_cast<double>(vertices.size()) / 20000.0 * 2000.0);

    auto uv = SMI04(vertices, facets, &config, pSTL2error);

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle(i);
        auto & point = pMesh->point(vh);
        point[0] = uv[i][0];
        point[1] = uv[i][1];
        point[2] = 0.0;
    }

    Real xMin =  std::numeric_limits<Real>::max();
    Real yMin =  std::numeric_limits<Real>::max();
    Real xMax = -std::numeric_limits<Real>::max();
    Real yMax = -std::numeric_limits<Real>::max();

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        xMin = std::min(xMin, point[0]);
        xMax = std::max(xMax, point[0]);

        yMin = std::min(yMin, point[1]);
        yMax = std::max(yMax, point[1]);
    }

    Real invDelta = 1.0 / std::max(xMax - xMin, yMax - yMin);
    Real invDeltaX = 1.0 / (xMax - xMin);
    Real invDeltaY = 1.0 / (yMax - yMin);

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        auto & point = pMesh->point(vh);
        point[0] = (point[0] - xMin) * invDeltaX;
        point[1] = (point[1] - yMin) * invDeltaY;
    }

    if (pWidth != nullptr)
    {
        *pWidth = (xMax - xMin) * invDelta;
    }

    if (pHeith != nullptr)
    {
        *pHeith = (yMax - yMin) * invDelta;
    }
}

bool TextureIntegrate(const TriMesh * pOriginMesh, const TriMesh * pNewMesh, Texture * pTexture, uint32_t width, uint32_t height, uint32_t expandRadius)
{
    if (pOriginMesh->n_vertices() != pNewMesh->n_vertices() ||
        pOriginMesh->n_edges() != pNewMesh->n_edges() ||
        pOriginMesh->n_faces() != pNewMesh->n_faces() ||
        pOriginMesh->n_halfedges() != pNewMesh->n_halfedges())
    {
        Logger::Error("When integrate texture, input meshes must have same topology.");
        return false;
    }

    uint32_t targetWidth = (width == uint32_t(-1) ? pTexture->Width() : width);
    uint32_t targetHeight = (height == uint32_t(-1) ? pTexture->Height() : height);
    uint32_t targetChannels = pTexture->Channels();

    Texture integratedTexture;
    integratedTexture.Create(targetWidth, targetHeight, targetChannels);

    /* AABB of triangle v0, v1, v2 in image, v* should be texcoord in [0, 1]^2 */
    auto GetTriangleAABBInImageFunc = [](const Vector2 & v0, const Vector2 & v1, const Vector2 & v2, uint32_t width,
        uint32_t height) -> std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> /* AABB[xMin, yMin, xMax, yMax] */
    {
        Real xMin =  std::numeric_limits<Real>::max();
        Real yMin =  std::numeric_limits<Real>::max();
        Real xMax = -std::numeric_limits<Real>::max();
        Real yMax = -std::numeric_limits<Real>::max();

        xMin = std::min(xMin, v0[0]); xMax = std::max(xMax, v0[0]);
        xMin = std::min(xMin, v1[0]); xMax = std::max(xMax, v1[0]);
        xMin = std::min(xMin, v2[0]); xMax = std::max(xMax, v2[0]);

        yMin = std::min(yMin, v0[1]); yMax = std::max(yMax, v0[1]);
        yMin = std::min(yMin, v1[1]); yMax = std::max(yMax, v1[1]);
        yMin = std::min(yMin, v2[1]); yMax = std::max(yMax, v2[1]);

        uint32_t widthLimit = width - 1;
        uint32_t heightLimit = height - 1;

        auto xMinInt = static_cast<uint32_t>(std::floor(xMin * widthLimit));
        auto yMinInt = static_cast<uint32_t>(std::floor(yMin * heightLimit));
        auto xMaxInt = static_cast<uint32_t>(std::ceil(xMax * widthLimit));
        auto yMaxInt = static_cast<uint32_t>(std::ceil(yMax * heightLimit));

        xMinInt = std::clamp(xMinInt, 0u, widthLimit);
        yMinInt = std::clamp(yMinInt, 0u, heightLimit);
        xMaxInt = std::clamp(xMaxInt, 0u, widthLimit);
        yMaxInt = std::clamp(yMaxInt, 0u, heightLimit);

        return std::make_tuple<>(xMinInt, yMinInt, xMaxInt, yMaxInt);
    };

    /* Whether v is inside triangle v0, v1, v2 */
    auto IsInsideTriangleFunc = [](const Vector2 & v, const Vector2 & v0, const Vector2 & v1, const Vector2 & v2) -> bool
    {
        Real signOfTriangle = (v1[0] - v0[0]) * (v2[1] - v0[1]) - (v1[1] - v0[1]) * (v2[0] - v0[0]);
        Real signOfAB = (v1[0] - v0[0]) * (v[1] - v0[1]) - (v1[1] - v0[1]) * (v[0] - v0[0]);
        Real signOfCA = (v0[0] - v2[0]) * (v[1] - v2[1]) - (v0[1] - v2[1]) * (v[0] - v2[0]);
        Real signOfBC = (v2[0] - v1[0]) * (v[1] - v2[1]) - (v2[1] - v1[1]) * (v[0] - v2[0]);

        bool d1 = (signOfAB * signOfTriangle >= 0);
        bool d2 = (signOfCA * signOfTriangle >= 0);
        bool d3 = (signOfBC * signOfTriangle >= 0);

        return d1 && d2 && d3;
    };

    /* affine matrix from triangle va0, va1, va2 to triangle vb0, vb1, vb2 */
    auto GetTransformFunc = [](const Vector2 & va0, const Vector2 & va1, const Vector2 & va2,
        const Vector2 & vb0, const Vector2 & vb1, const Vector2 & vb2) -> Matrix3
    {
        Matrix3 A;
        A(0, 0) = va0[0]; A(0, 1) = va0[1]; A(0, 2) = 1;
        A(1, 0) = va1[0]; A(1, 1) = va1[1]; A(1, 2) = 1;
        A(2, 0) = va2[0]; A(2, 1) = va2[1]; A(2, 2) = 1;

        Matrix3 B;
        B(0, 0) = vb0[0]; B(0, 1) = vb0[1]; B(0, 2) = 1;
        B(1, 0) = vb1[0]; B(1, 1) = vb1[1]; B(1, 2) = 1;
        B(2, 0) = vb2[0]; B(2, 1) = vb2[1]; B(2, 2) = 1;

        /* Ax=B */
        return A.inverse() * B;
    };

    /* apply affine matrix t (by left multiply) to v */
    auto ApplyTransformFunc = [](const Vector2 & v, const Matrix3 & t) -> Vector2
    {
        Eigen::Matrix<Real, 1, 3> vec(v[0], v[1], 1);
        vec *= t;
        return Vector2(vec[0], vec[1]);
    };

    uint8_t BLANK = 0, COPIED = 1, EXPANDED = 2;
    std::vector<std::vector<uint8_t>> flag(targetHeight, std::vector<uint8_t>(targetWidth, BLANK));

    auto CopyKernel = [&pOriginMesh, &pNewMesh, &pTexture, &GetTransformFunc, &GetTriangleAABBInImageFunc, &IsInsideTriangleFunc,
        &ApplyTransformFunc, &targetWidth, &targetHeight, &targetChannels, &integratedTexture, &flag, COPIED](tbb::blocked_range<int> & range)
    {
        for (int i = range.begin(); i != range.end(); i++)
        {
            const auto fh = TriMesh::FaceHandle(i);

            auto heha0 = pOriginMesh->halfedge_handle(fh);
            auto heha1 = pOriginMesh->next_halfedge_handle(heha0);
            auto heha2 = pOriginMesh->next_halfedge_handle(heha1);

            const auto & va0 = pOriginMesh->texcoord2D(heha0);
            const auto & va1 = pOriginMesh->texcoord2D(heha1);
            const auto & va2 = pOriginMesh->texcoord2D(heha2);

            auto hehb0 = pNewMesh->halfedge_handle(fh);
            auto hehb1 = pNewMesh->next_halfedge_handle(hehb0);
            auto hehb2 = pNewMesh->next_halfedge_handle(hehb1);

            const auto & vb0 = pNewMesh->texcoord2D(hehb0);
            const auto & vb1 = pNewMesh->texcoord2D(hehb1);
            const auto & vb2 = pNewMesh->texcoord2D(hehb2);

            auto t = GetTransformFunc(vb0, vb1, vb2, va0, va1, va2);

            auto aabb = GetTriangleAABBInImageFunc(vb0, vb1, vb2, targetWidth, targetHeight);

            uint32_t yMin = std::get<1>(aabb);
            uint32_t yMax = std::get<3>(aabb);
            uint32_t xMin = std::get<0>(aabb);
            uint32_t xMax = std::get<2>(aabb);

            for (uint32_t y = yMin; y <= yMax; y++)
            {
                for (uint32_t x = xMin; x <= xMax; x++)
                {
                    Vector2 pos(static_cast<Real>(x) / static_cast<Real>(targetWidth - 1), static_cast<Real>(y) / static_cast<Real>(targetHeight - 1));
                    if (IsInsideTriangleFunc(pos, vb0, vb1, vb2))
                    {
                        Vector2 originPos = ApplyTransformFunc(pos, t);

                        uint32_t srcWidthLimit = pTexture->Width() - 1;
                        uint32_t srcHeightLimit = pTexture->Height() - 1;

                        auto x0 = static_cast<uint32_t>(std::floor(originPos[0] * srcWidthLimit));
                        auto y0 = static_cast<uint32_t>(std::floor(originPos[1] * srcHeightLimit));
                        auto x1 = x0 + 1;
                        auto y1 = y0 + 1;

                        x0 = std::clamp(x0, 0u, srcWidthLimit);
                        y0 = std::clamp(y0, 0u, srcHeightLimit);
                        x1 = std::clamp(x1, 0u, srcWidthLimit);
                        y1 = std::clamp(y1, 0u, srcHeightLimit);

                        uint8_t * pX0Y0 = pTexture->At(x0, srcHeightLimit - y0);
                        uint8_t * pX0Y1 = pTexture->At(x0, srcHeightLimit - y1);
                        uint8_t * pX1Y0 = pTexture->At(x1, srcHeightLimit - y0);
                        uint8_t * pX1Y1 = pTexture->At(x1, srcHeightLimit - y1);

                        uint8_t * pDstPixel = integratedTexture.At(x, targetHeight - 1 - y);

                        for (uint32_t c = 0; c < targetChannels; c++)
                        {
                            uint32_t sum = static_cast<uint32_t>(pX0Y0[c]) +
                                           static_cast<uint32_t>(pX0Y1[c]) +
                                           static_cast<uint32_t>(pX1Y0[c]) +
                                           static_cast<uint32_t>(pX1Y1[c]);

                            pDstPixel[c] = static_cast<uint8_t>(sum / 4);
                        }

                        flag[y][x] = COPIED;
                    }
                }
            }
        }
    };

    tbb::blocked_range<int> CopyRange(0, static_cast<int>(pOriginMesh->n_faces()));
    tbb::parallel_for(CopyRange, CopyKernel);

    auto ExpandKernel = [&targetWidth, &targetHeight, &targetChannels, &expandRadius, &flag, &integratedTexture, BLANK,
        COPIED, EXPANDED](tbb::blocked_range<uint32_t> & range)
    {
        for (uint32_t y = range.begin(); y != range.end(); y++)
        {
            for (uint32_t x = 0; x < targetWidth; x++)
            {
                if (flag[y][x] == BLANK)
                {
                    uint32_t n = 0;
                    std::vector<uint32_t> sums(targetChannels, 0u);

                    uint32_t heightLimit = targetHeight - 1u;
                    uint32_t widthLimit = targetWidth - 1u;

                    uint32_t yMin = std::max(y - expandRadius, 0u);
                    uint32_t xMin = std::max(x - expandRadius, 0u);
                    uint32_t yMax = std::min(y + expandRadius, heightLimit);
                    uint32_t xMax = std::min(x + expandRadius, widthLimit);

                    for (uint32_t y0 = yMin; y0 <= yMax; y0++)
                    {
                        for (uint32_t x0 = xMin; x0 <= xMax; x0++)
                        {
                            if (flag[y0][x0] == COPIED)
                            {
                                n++;
                                uint8_t * pPixel = integratedTexture.At(x0, heightLimit - y0);
                                for (uint32_t c = 0; c < targetChannels; c++)
                                {
                                    sums[c] += static_cast<uint32_t>(pPixel[c]);
                                }
                            }
                        }
                    }

                    if (n > 0)
                    {
                        uint8_t * pPixel = integratedTexture.At(x, heightLimit - y);
                        for (uint32_t c = 0; c < targetChannels; c++)
                        {
                            pPixel[c] = static_cast<uint8_t>(sums[c] / n);
                        }
                        flag[y][x] = EXPANDED;
                    }
                }
            }
        }
    };

    tbb::blocked_range<uint32_t> ExpandRange(0u, targetHeight);
    tbb::parallel_for(ExpandRange, ExpandKernel);

    integratedTexture.CopyTo(pTexture);

    return true;
}

bool IsDiskTopology(const TriMesh * pMesh)
{
    int32_t genus = GetMeshGenus(pMesh);
    int32_t euler = GetMeshEuler(pMesh);
    size_t comp = MeshComponentCount(pMesh);
    size_t boundary = MeshBoundaryCount(pMesh);
    return genus == 0 && euler == 1 && comp == 1 && boundary == 1;
}

double ComputePackingEfficiency(const TriMesh * pMesh)
{
    Real totalAtlasArea = 1.0;
    Real atlasArea = 0.0;

    for (const auto & fh : pMesh->faces())
    {
        std::vector<TriMesh::Point> v;

        for (const auto heh : pMesh->fh_range(fh))
        {
            const auto & uv = pMesh->texcoord2D(heh);
            v.emplace_back( uv[0], uv[1], 0.0f );
        }

        const auto l1 = v[1] - v[0];
        const auto l2 = v[2] - v[0];

        atlasArea += l1.cross(l2).norm() * 0.5;
    }

    double PE = atlasArea / totalAtlasArea;
    return PE;
}