#ifndef ATLAS_MERGER_HPP
#define ATLAS_MERGER_HPP

#include "Declaration.hpp"
#include "Texture.hpp"
#include "MeshUtils.hpp"

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
    explicit AtlasMerger(TriMesh * pMesh, Texture * pTexture) : m_pMesh(pMesh), m_pTexture(pTexture) { }
    bool Run(uint32_t width, uint32_t height, EParameterization parameterization, const std::string & meshOutput, const std::string & textureOutput);

    static std::unique_ptr<AtlasInfo> MarkAtlasIndex(TriMesh * pMesh);
    static size_t MergeAtlas(TriMesh * pMesh, AtlasInfo * pAtlasInfo);
    static std::unique_ptr<AtlasInfo> MeshClustering(TriMesh * pMesh);
    static std::unique_ptr<TriMesh[]> CreateSubMeshFromAtlas(const TriMesh * pMesh, AtlasInfo * pAtlasInfo);

private:
    static void MergeAtlasInfoAndFProperty(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, size_t source, size_t target);
    static void RemapAtlasInfoKey(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, const std::set<size_t> & atlasIdxSet);
    static size_t IntersectNum(const std::set<int> & edges0, const std::set<int> & edges1);
    static bool IsIntersect(const std::set<int> & edges0, const std::set<int> & edges1);
    static void SortAtlasSize(std::vector<std::pair<size_t, size_t>/*<atlasIdx, faceNum>*/> & atlasSizes);
    static int32_t GetAtlasGenus(AtlasInfo * pAtlasInfo, size_t atlasIdx);
    static bool TextureIntegrate(const TriMesh * pOriginSubMesh, const TriMesh * pNewSubMesh, Texture * pTexture, uint32_t width, uint32_t height);

private:
    TriMesh * m_pMesh;
    Texture * m_pTexture;
};

#endif //ATLAS_MERGER_HPP
