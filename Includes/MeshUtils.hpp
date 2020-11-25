#ifndef MESH_UTILS_HPP
#define MESH_UTILS_HPP

#include "Declaration.hpp"

enum class EParameterization
{
    EBarycentric = 0,
    ESLIM = 1,
    EEBP = 2
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
bool CutToDisk(TriMesh * pMesh);

int32_t GetMeshEuler(const TriMesh * pMesh);
int32_t GetMeshGenus(const TriMesh * pMesh);

bool MeshUVTransfer(TriMesh * pMesh, const TriMesh * pUVMesh);

void SLIMProxy(TriMesh * pMesh);
void EBPProxy(TriMesh * pMesh);

#endif //MESH_UTILS_HPP
