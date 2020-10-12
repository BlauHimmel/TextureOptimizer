#include "TextureOptimizer.hpp"

#include <algorithm>

#include "Logger.hpp"
#include "CutToDiskImpl.hpp"

bool TextureOptimizer::Run()
{
    if (m_pMesh == nullptr || m_pTexture == nullptr || !m_pTexture->IsValid())
    {
        return false;
    }

    RepairNonManifold(m_pMesh);
    CutToDisk(m_pMesh);
    return true;
}

std::vector<std::vector<OpenMesh::VertexHandle>> TextureOptimizer::MeshComponentVertices(const TriMesh * pMesh)
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

std::vector<size_t> TextureOptimizer::MeshComponentVertexNums(const TriMesh * pMesh)
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

size_t TextureOptimizer::MeshComponentCount(const TriMesh * pMesh)
{
    return MeshComponentVertices(pMesh).size();
}

std::vector<std::vector<OpenMesh::VertexHandle>> TextureOptimizer::MeshBoundaryVertices(const TriMesh *pMesh)
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

std::vector<size_t> TextureOptimizer::MeshBoundaryLengths(const TriMesh * pMesh)
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

size_t TextureOptimizer::MeshBoundaryCount(const TriMesh * pMesh)
{
    return MeshBoundaryLengths(pMesh).size();
}

void TextureOptimizer::RemoveSmallComponent(TriMesh *pMesh)
{

}

void TextureOptimizer::RepairNonManifold(TriMesh * pMesh)
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

    Logger::Info("%zd non-manifold vertices founld!", nonManifoldVHs.size());

    if (nonManifoldVHs.empty()) { return; }

    uint32_t newVertexNum = 0;

    OpenMesh::FPropHandleT<size_t> componentFProp;
    pMesh->add_property(componentFProp);

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

    pMesh->remove_property(componentFProp);
    pMesh->garbage_collection();
    Logger::Info("Non-manifold fixed, %u new vertices created!", newVertexNum);
}

void TextureOptimizer::CutToDisk(TriMesh * pMesh)
{
    CutToDiskImpl(pMesh);
}
