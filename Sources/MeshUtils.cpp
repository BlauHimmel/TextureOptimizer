#include "MeshUtils.hpp"

#include <mutex>

#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include <progress_bar.h>

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

std::vector<std::vector<OpenMesh::VertexHandle>> MeshBoundaryVertices(const TriMesh *pMesh)
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

void RepairNonManifold(TriMesh * pMesh)
{
    if (pMesh == nullptr) { return; }

    std::vector<OpenMesh::VertexHandle> nonManifoldVHs;
    for (auto vh : pMesh->vertices())
    {
        if (!pMesh->is_manifold(vh))
        {
            nonManifoldVHs.push_back(vh);
        }
    }

    Logger::Info("%zd non-manifold vertices found!", nonManifoldVHs.size());

    if (nonManifoldVHs.empty()) { return; }

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
}

bool CutToDisk(TriMesh * pMesh)
{
    const size_t SEED_FACE_NUM = 100;
    size_t faceIdxIncrement = pMesh->n_faces() / SEED_FACE_NUM;

    std::vector<size_t> seedIdxs;
    for (size_t seedIdx = 0; seedIdx < pMesh->n_faces(); seedIdx += faceIdxIncrement)
    {
        seedIdxs.push_back(seedIdx);
    }

    bmc::progress_bar progressBar(0, seedIdxs.size(), 80, false, true);
    std::mutex mutex;

    auto KernelFunc = [&seedIdxs, &pMesh, &mutex, &progressBar](tbb::blocked_range<size_t> & range, std::pair<size_t, Real> value) -> std::pair<size_t, Real>
    {
        for (size_t i = range.begin(); i != range.end(); i++)
        {
            mutex.lock();
            progressBar++;
            mutex.unlock();

            TriMesh testMesh(*pMesh);
            CutToDiskImpl(&testMesh, seedIdxs[i]);

            auto boundaryDistances = MeshBoundaryDistances(&testMesh);
            if (boundaryDistances.size() != 1) { continue; }

            if (boundaryDistances[0] < value.second)
            {
                value.first = seedIdxs[i];
                value.second = boundaryDistances[0];
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

size_t GetMeshEuler(const TriMesh * pMesh)
{
    return pMesh->n_vertices() - pMesh->n_edges() + pMesh->n_faces();
}

size_t GetMeshGenus(const TriMesh * pMesh)
{
    size_t euler = GetMeshEuler(pMesh);
    return (2 - euler - MeshBoundaryCount(pMesh)) / 2;
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
        for (size_t i = range.begin(); i < range.end(); i++)
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
