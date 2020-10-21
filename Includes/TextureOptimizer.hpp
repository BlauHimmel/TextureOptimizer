#ifndef TEXTURE_OPTIMIZER_HPP
#define TEXTURE_OPTIMIZER_HPP

#include "Declaration.hpp"
#include "Texture.hpp"

class TextureOptimizer
{
public:
    TextureOptimizer(TriMesh * pMesh, Texture * pTexture) : m_pMesh(pMesh), m_pTexture(pTexture) { }
    bool Run(uint32_t width, uint32_t height);

    static std::vector<std::vector<OpenMesh::VertexHandle>> MeshComponentVertices(const TriMesh * pMesh);
    static std::vector<size_t> MeshComponentVertexNums(const TriMesh * pMesh);
    static size_t MeshComponentCount(const TriMesh * pMesh);

    static std::vector<std::vector<OpenMesh::VertexHandle>> MeshBoundaryVertices(const TriMesh * pMesh);
    static std::vector<size_t> MeshBoundaryLengths(const TriMesh * pMesh);
    static size_t MeshBoundaryCount(const TriMesh * pMesh);

    static void RemoveSmallComponent(TriMesh * pMesh);
    static void RepairNonManifold(TriMesh * pMesh);
    static bool CutToDisk(TriMesh * pMesh);

    static size_t GetMeshEuler(const TriMesh * pMesh);
    static size_t GetMeshGenus(const TriMesh * pMesh);

    static bool MeshUVTransfer(TriMesh * pMesh, const TriMesh * pUVMesh);

    static bool TextureIntegrate(const TriMesh * pOriginMesh, const TriMesh * pNewMesh, Texture * pTexture, uint32_t width, uint32_t height);
    static void SLIMProxy(TriMesh * pMesh);

private:
    TriMesh * m_pMesh;
    Texture * m_pTexture;
};

#endif //TEXTURE_OPTIMIZER_HPP
