#ifndef ATLAS_MERGER_HPP
#define ATLAS_MERGER_HPP

#include "Declaration.hpp"

#include <memory>
#include <unordered_map>
#include <set>

struct AtlasInfo
{
    std::unordered_map<size_t, std::set<int>> atlasVertices;
    std::unordered_map<size_t, std::set<int>> atlasEdges;
    std::unordered_map<size_t, std::set<int>> atlasHalfedges;
    std::unordered_map<size_t, std::set<int>> atlasFaces;
    size_t atlasNum = 0;
};

class AtlasMerger
{
public:
    explicit AtlasMerger(TriMesh * pMesh) : m_pMesh(pMesh) { }
    bool Run();

    static std::unique_ptr<AtlasInfo> MarkAtlasIndex(TriMesh * pMesh);
    static size_t MergeAtlas(TriMesh * pMesh, AtlasInfo * pAtlasInfo);

private:
    static void MergeAtlasInfoAndFProperty(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, size_t source, size_t target);
    static void RemapAtlasInfoKey(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, const std::set<size_t> & atlasIdxSet);

private:
    TriMesh * m_pMesh;
};

#endif //ATLAS_MERGER_HPP
