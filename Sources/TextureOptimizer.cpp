#include "TextureOptimizer.hpp"

bool TextureOptimizer::Run()
{
    if (m_pMesh == nullptr || m_pTexture == nullptr || !m_pTexture->IsValid())
    {
        return false;
    }

    for (auto vh : m_pMesh->vertices())
    {
        if (!m_pMesh->is_manifold(vh))
        {
            std::cout << "Vertex [" << vh.idx() << "] is non-manifold!" << std::endl;
        }
    }

    auto comps = MeshComponentVertexNums(m_pMesh);
    auto bouns = MeshBoundaryLengths(m_pMesh);

    return true;
}

std::vector<size_t> TextureOptimizer::MeshComponentVertexNums(const TriMesh *pMesh)
{
    TriMesh mesh(*pMesh);
    std::vector<size_t> vertexNums;

    OpenMesh::VPropHandleT<bool> visitedVProp;
    mesh.add_property(visitedVProp);
    for (auto vh : mesh.vertices())
    {
        mesh.property(visitedVProp, vh) = false;
    }

    auto vertexIter = mesh.vertices_begin();

    while (true)
    {
        // find an unvisited vertex
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

        // if not found, then finished
        if (!bFound) { break; }

        std::vector<OpenMesh::VertexHandle> handles;
        size_t vertexNum = 0;
        handles.push_back(foundVH);

        // grow from found vertex
        while (!handles.empty())
        {
            auto currentVH = handles.back();
            handles.pop_back();
            vertexNum++;

            for (auto iter = mesh.vv_iter(currentVH); iter.is_valid(); ++iter)
            {
                if (!mesh.property(visitedVProp, *iter))
                {
                    mesh.property(visitedVProp, *iter) = true;
                    handles.push_back(*iter);
                }
            }
        }

        vertexNums.push_back(vertexNum);
    }

    return vertexNums;
}

size_t TextureOptimizer::MeshComponentCount(const TriMesh * pMesh)
{
    return MeshComponentVertexNums(pMesh).size();
}

std::vector<size_t> TextureOptimizer::MeshBoundaryLengths(const TriMesh *pMesh)
{
    TriMesh mesh(*pMesh);
    std::vector<size_t> boundaryLengths;

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

        mesh.property(visitedHProp, heh) = true;
        auto nextHeh = mesh.next_halfedge_handle(heh);
        int length = 1;

        while (mesh.is_boundary(heh) && !mesh.property(visitedHProp, nextHeh))
        {
            mesh.property(visitedHProp, nextHeh) = true;
            nextHeh = mesh.next_halfedge_handle(nextHeh);
            length++;
        }
        boundaryLengths.push_back(length);
    }

    return boundaryLengths;
}

size_t TextureOptimizer::MeshBoundaryCount(const TriMesh *pMesh)
{
    return MeshBoundaryLengths(pMesh).size();
}
