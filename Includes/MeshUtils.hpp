#ifndef MESH_UTILS_HPP
#define MESH_UTILS_HPP

#include "Declaration.hpp"
#include "Texture.hpp"

enum class EParameterization
{
    EBarycentric = 0,
    ESLIM = 1,
    EEBP = 2,
    ESMI04 = 3
};

std::vector<std::vector<OpenMesh::VertexHandle>> MeshComponentVertices(const TriMesh * pMesh);
std::vector<size_t> MeshComponentVertexNums(const TriMesh * pMesh);
size_t MeshComponentCount(const TriMesh * pMesh);

std::vector<std::vector<OpenMesh::VertexHandle>> MeshBoundaryVertices(const TriMesh * pMesh);
std::vector<size_t> MeshBoundaryLengths(const TriMesh * pMesh);
std::vector<Real> MeshBoundaryDistances(const TriMesh * pMesh);
size_t MeshBoundaryCount(const TriMesh * pMesh);

void RemoveSmallComponent(TriMesh * pMesh);
void RepairNonManifold(TriMesh * pMesh);
bool CutToDisk(TriMesh * pMesh, const Real * pTotalArea = nullptr, size_t seedFaceNum = 100);

int32_t GetMeshEuler(const TriMesh * pMesh);
int32_t GetMeshGenus(const TriMesh * pMesh);

bool MeshUVTransfer(TriMesh * pMesh, const TriMesh * pUVMesh);

void SLIMProxy(TriMesh * pMesh, Real * pWidth = nullptr, Real * pHeith = nullptr);
void EBPProxy(TriMesh * pMesh, Real * pWidth = nullptr, Real * pHeith = nullptr);
void SMI04Proxy(TriMesh * pMesh, Real * pWidth = nullptr, Real * pHeith = nullptr, Real * pSTL2error = nullptr);

bool TextureIntegrate(const TriMesh * pOriginMesh, const TriMesh * pNewMesh, Texture * pTexture, uint32_t width, uint32_t height, uint32_t expandRadius = 4);

bool IsDiskTopology(const TriMesh * pMesh);

#endif //MESH_UTILS_HPP
