#include "AtlasMerger.hpp"

#include <unordered_set>
#include <queue>
#include <random>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"
#include "MeshUtils.hpp"

const char * MESH_FPROP_ATLAS_INDEX = "atlasIdx";

struct TexCoord2DHasher
{
    size_t operator()(const TriMesh::TexCoord2D & uv) const
    {
        return std::hash<Real>()(uv[0]) ^ std::hash<Real>()(uv[1]);
    }
};

bool AtlasMerger::Run()
{
    if (m_pMesh == nullptr)
    {
        return false;
    }

    Logger::Info("Removing small components of the mesh...");
    RemoveSmallComponent(m_pMesh);
    Logger::Info("<RemoveSmallComponent> Completed!");

    Logger::Info("Repairing non-manifold vertices of the mesh...");
    RepairNonManifold(m_pMesh);
    Logger::Info("<RepairNonManifold> Completed!");

    Logger::Info("Marking the atlas info...");
    std::unique_ptr<AtlasInfo> atlasInfo = MarkAtlasIndex(m_pMesh);
    if (atlasInfo == nullptr)
    {
        Logger::Error("Error occurred when marking the atlas info.");
        return false;
    }
    Logger::Info("<MarkAtlasIndex> Completed!");

    Logger::Info("Merging the atlas...");
    size_t atlasNum = MergeAtlas(m_pMesh, atlasInfo.get());
    if (atlasNum == size_t(-1))
    {
        Logger::Error("Error occurred when merging the atlas.");
        return false;
    }
    Logger::Info("<MergeAtlas> Completed! Now, there are %zd atlas.", atlasNum);

    return true;
}

std::unique_ptr<AtlasInfo> AtlasMerger::MarkAtlasIndex(TriMesh * pMesh)
{
    if (pMesh == nullptr) { return nullptr; }

    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    pMesh->add_property(atlasIdxFProp, MESH_FPROP_ATLAS_INDEX);

    for (auto fh : pMesh->faces())
    {
        pMesh->property(atlasIdxFProp, fh) = size_t(-1);
    }

    std::unique_ptr<AtlasInfo> atlasInfo(new AtlasInfo);

    while (true)
    {
        /* find an unhandled face as a seed face */
        int seedFaceIdx = -1;
        for (auto fh : pMesh->faces())
        {
            if (pMesh->property(atlasIdxFProp, fh) == size_t(-1))
            {
                seedFaceIdx = fh.idx();
                break;
            }
        }

        /* all faces have been handled, break */
        if (seedFaceIdx == -1) { break; }

        std::queue<OpenMesh::FaceHandle> faceQueue;
        std::unordered_set<TriMesh::TexCoord2D, TexCoord2DHasher> atlasUVSet;

        {
            auto fh = OpenMesh::FaceHandle(seedFaceIdx);
            faceQueue.push(fh);
            for (auto heh : pMesh->fh_range(fh))
            {
                const auto & uv = pMesh->texcoord2D(heh);
                atlasUVSet.insert(uv);
            }
        }

        while (!faceQueue.empty())
        {
            auto fh = faceQueue.front();
            faceQueue.pop();

            /* check whether current face has been handled before (since it might be the adjacent face (xyz domain) of several face) */
            if (pMesh->property(atlasIdxFProp, fh) != size_t(-1))
            {
                continue;
            }

            /* check whether is face is isolated (uv domain) */
            int existUVCount = 0;
            for (auto heh : pMesh->fh_range(fh))
            {
                const auto & uv = pMesh->texcoord2D(heh);
                if (atlasUVSet.count(uv) != 0)
                {
                    existUVCount++;
                }
            }

            if (existUVCount <= 1) { continue; }

            /* update face property & atlas info */
            size_t atlasIdx = atlasInfo->atlasNum;
            pMesh->property(atlasIdxFProp, fh) = atlasIdx;

            atlasInfo->atlasFaces[atlasIdx].insert(fh.idx());
            for (auto vh : pMesh->fv_range(fh))
            {
                atlasInfo->atlasVertices[atlasIdx].insert(vh.idx());
            }
            for (auto eh : pMesh->fe_range(fh))
            {
                atlasInfo->atlasEdges[atlasIdx].insert(eh.idx());
            }
            for (auto heh : pMesh->fh_range(fh))
            {
                atlasInfo->atlasHalfedges[atlasIdx].insert(heh.idx());
            }

            /* store the uv coordinate of current face */
            for (auto heh : pMesh->fh_range(fh))
            {
                const auto & uv = pMesh->texcoord2D(heh);
                atlasUVSet.insert(uv);
            }

            /* insert the adjacent faces (xyz domain) that have not been handled before into the queue */
            for (auto adjFH : pMesh->ff_range(fh))
            {
                if (pMesh->property(atlasIdxFProp, adjFH) == size_t(-1))
                {
                    faceQueue.push(adjFH);
                }
            }
        }

        atlasInfo->atlasNum++;
    }

    Logger::Info("%d atlas found.", atlasInfo->atlasNum);

    /* visualization used for debug */
    DisplayAtlas(pMesh, atlasInfo->atlasNum, MESH_FPROP_ATLAS_INDEX, "Atlas - Before Merge");

    return atlasInfo;
}

size_t AtlasMerger::MergeAtlas(TriMesh * pMesh, AtlasInfo * pAtlasInfo)
{
    if (pMesh == nullptr || pAtlasInfo == nullptr)
    {
        return size_t(-1);
    }

    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    if (!pMesh->get_property_handle(atlasIdxFProp, MESH_FPROP_ATLAS_INDEX))
    {
        Logger::Error("To merge atlas, mesh must have face property \"atlasIdx\".");
        return size_t(-1);
    }

    std::set<size_t> atlasIdxSet;
    for (size_t i = 0; i < pAtlasInfo->atlasNum; i++) { atlasIdxSet.insert(i); }

    const size_t MERGE_STAGE_1_CONNECT_THRESHOLD = 3;
    const size_t MERGE_STAGE_2_CONNECT_THRESHOLD = 1;
    const size_t FINAL_ATLAS_NUM_THRESHOLD = 9;
    const Real SMALL_ATLAS_FACTOR = 0.2;

    /* merge disk-like atlas together */
    while (true)
    {
        std::vector<std::pair<size_t, size_t>/*<atlasIdx, faceNum>*/> atlasSizes;
        atlasSizes.reserve(pAtlasInfo->atlasNum);

        /* compute handle number of each atlas and record disk-like atlas */
        for (size_t atlasIdx : atlasIdxSet)
        {
            if (GetAtlasGenus(pAtlasInfo, atlasIdx) == 0)
            {
                size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
                atlasSizes.emplace_back(atlasIdx, faceNum);
            }
        }

        /* exit loop if there is no disk-like atlas */
        if (atlasSizes.empty()) { break; }

        /* sort all disk-like atlas according the face number */
        SortAtlasSize(atlasSizes);

        size_t mergedFaceNum = 0;

        for (size_t i = 0; i < atlasSizes.size(); i++)
        {
            /* continue if this atlas has been merged before */
            size_t mergedAtlasIdx0 = atlasSizes[i].first;
            if (mergedAtlasIdx0 == size_t(-1)) { continue; }

            for (size_t j = i + 1; j < atlasSizes.size(); j++)
            {
                /* continue if this atlas has been merged before */
                size_t mergedAtlasIdx1 = atlasSizes[j].first;
                if (mergedAtlasIdx1 == size_t(-1)) { continue; }

                std::set<int> & atlasEdges0 = pAtlasInfo->atlasEdges[mergedAtlasIdx0];
                std::set<int> & atlasEdges1 = pAtlasInfo->atlasEdges[mergedAtlasIdx1];

                /* if two atlas contains same edges, them it can be merged tother */
                if (IntersectNum(atlasEdges0, atlasEdges1) <= MERGE_STAGE_1_CONNECT_THRESHOLD) { continue; }

                /* update atlas info and mesh property */
                MergeAtlasInfoAndFProperty(pAtlasInfo, pMesh, atlasIdxFProp, mergedAtlasIdx1, mergedAtlasIdx0);
                mergedFaceNum++;

                /* remove the atlas that has been merged */
                atlasSizes[j].first = size_t(-1);
                atlasSizes[j].second = size_t(-1);
                atlasIdxSet.erase(mergedAtlasIdx1);
            }
        }

        if (mergedFaceNum == 0) { break; }
    }

    Logger::Info("Merge stage 1 complete, %zd atlas left now.", pAtlasInfo->atlasNum);

    /* merge non-disk-like atlas and its adjacent disk-like atlas */
    while (true)
    {
        std::vector<std::pair<size_t, size_t>/*<atlasIdx, faceNum>*/> nonDiskAtlasSizes, diskAtlasSizes;
        nonDiskAtlasSizes.reserve(pAtlasInfo->atlasNum);
        diskAtlasSizes.reserve(pAtlasInfo->atlasNum);

        /* compute handle number of each atlas and record disk-like atlas */
        for (size_t atlasIdx : atlasIdxSet)
        {
            size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
            if (GetAtlasGenus(pAtlasInfo, atlasIdx) == 0)
            {
                diskAtlasSizes.emplace_back(atlasIdx, faceNum);
            }
            else
            {
                nonDiskAtlasSizes.emplace_back(atlasIdx, faceNum);
            }
        }

        /* exit loop if there is no disk-like atlas */
        if (diskAtlasSizes.empty()) { break; }

        /* sort all disk-like and non-disk-like atlas according the face number */
        SortAtlasSize(diskAtlasSizes);
        SortAtlasSize(nonDiskAtlasSizes);

        size_t mergedFaceNum = 0;

        for (size_t i = 0; i < nonDiskAtlasSizes.size() && mergedFaceNum < 1; i++)
        {
            size_t mergedAtlasIdx0 = nonDiskAtlasSizes[i].first;

            for (size_t j = 0; j < diskAtlasSizes.size() && mergedFaceNum < j; j++)
            {
                /* continue if this atlas has been merged before */
                size_t mergedAtlasIdx1 = diskAtlasSizes[j].first;
                if (mergedAtlasIdx1 == size_t(-1)) { continue; }

                std::set<int> & atlasEdges0 = pAtlasInfo->atlasEdges[mergedAtlasIdx0];
                std::set<int> & atlasEdges1 = pAtlasInfo->atlasEdges[mergedAtlasIdx1];

                /* if two atlas contains same edges, them it can be merged tother */
                if (IntersectNum(atlasEdges0, atlasEdges1) <= MERGE_STAGE_2_CONNECT_THRESHOLD) { continue; }

                /* update atlas info and mesh property */
                MergeAtlasInfoAndFProperty(pAtlasInfo, pMesh, atlasIdxFProp, mergedAtlasIdx1, mergedAtlasIdx0);
                mergedFaceNum++;

                /* remove the atlas that has been merged */
                diskAtlasSizes[j].first = size_t(-1);
                diskAtlasSizes[j].second = size_t(-1);
                atlasIdxSet.erase(mergedAtlasIdx1);
            }
        }

        if (mergedFaceNum == 0) { break; }
    }

    Logger::Info("Merge stage 2 complete, %zd atlas left now.", pAtlasInfo->atlasNum);

    /* merge small atlas together to further reduce the atlas num */
    while (true)
    {
        std::vector<std::pair<size_t, size_t>/*<atlasIdx, faceNum>*/> atlasSizes;
        atlasSizes.reserve(pAtlasInfo->atlasNum);

        /* record atlas */
        for (size_t atlasIdx : atlasIdxSet)
        {
            size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
            atlasSizes.emplace_back(atlasIdx, faceNum);
        }

        /* exit loop if there is no atlas */
        if (atlasSizes.empty()) { break; }

        /* sort all atlas according the face number */
        SortAtlasSize(atlasSizes);

        size_t mergedFaceNum = 0;

        for (size_t i = 0; i < atlasSizes.size() && mergedFaceNum < 1; i++)
        {
            /* continue if this atlas has been merged before */
            size_t mergedAtlasIdx0 = atlasSizes[i].first;
            if (mergedAtlasIdx0 == size_t(-1)) { continue; }

            for (size_t j = i + 1; j < atlasSizes.size() && mergedFaceNum < 1; j++)
            {
                /* continue if this atlas has been merged before */
                size_t mergedAtlasIdx1 = atlasSizes[j].first;
                if (mergedAtlasIdx1 == size_t(-1)) { continue; }

                std::set<int> & atlasEdges0 = pAtlasInfo->atlasEdges[mergedAtlasIdx0];
                std::set<int> & atlasEdges1 = pAtlasInfo->atlasEdges[mergedAtlasIdx1];

                /* if two atlas contains same edges, them it can be merged tother */
                if (!IsIntersect(atlasEdges0, atlasEdges1)) { continue; }

                /* update atlas info and mesh property */
                MergeAtlasInfoAndFProperty(pAtlasInfo, pMesh, atlasIdxFProp, mergedAtlasIdx1, mergedAtlasIdx0);
                mergedFaceNum++;

                /* remove the atlas that has been merged */
                atlasSizes[j].first = size_t(-1);
                atlasSizes[j].second = size_t(-1);
                atlasIdxSet.erase(mergedAtlasIdx1);
            }
        }

        if (pAtlasInfo->atlasNum <= FINAL_ATLAS_NUM_THRESHOLD || mergedFaceNum == 0)
        {
            break;
        }
    }

    Logger::Info("Merge stage 3 complete, %zd atlas left now.", pAtlasInfo->atlasNum);

    /* if there exist a small atlas, merge it even if atlas <= FINAL_ATLAS_NUM_THRESHOLD */
    std::vector<std::pair<size_t, size_t>/*<atlasIdx, faceNum>*/> smallAtlasSizes;
    size_t avgAtlasSize = 0;
    for (size_t atlasIdx : atlasIdxSet)
    {
        size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
        avgAtlasSize += faceNum;
    }
    avgAtlasSize /= pAtlasInfo->atlasNum;

    for (size_t atlasIdx : atlasIdxSet)
    {
        size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
        if (faceNum <= avgAtlasSize * SMALL_ATLAS_FACTOR)
        {
            smallAtlasSizes.emplace_back(atlasIdx, faceNum);
            Logger::Info("Small atlas found : [%zd] : %zd faces.", atlasIdx, faceNum);
        }
    }

    if (!smallAtlasSizes.empty())
    {
        while (true)
        {
            std::vector<std::pair<size_t, size_t>/*<atlasIdx, faceNum>*/> atlasSizes;
            atlasSizes.reserve(pAtlasInfo->atlasNum);

            /* record atlas (that are not in 'smallAtlasSizes') */
            for (size_t atlasIdx : atlasIdxSet)
            {
                size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
                std::pair<size_t, size_t> smallAtlasSize = { atlasIdx, faceNum };
                if (std::find(smallAtlasSizes.begin(), smallAtlasSizes.end(), smallAtlasSize) == smallAtlasSizes.end())
                {
                    atlasSizes.push_back(smallAtlasSize);
                }
            }

            /* exit loop if there is no atlas */
            if (atlasSizes.empty()) { break; }

            /* sort all atlas according the face number */
            SortAtlasSize(smallAtlasSizes);
            SortAtlasSize(atlasSizes);

            size_t mergedFaceNum = 0;

            for (size_t i = 0; i < atlasSizes.size() && mergedFaceNum < 1; i++)
            {
                /* continue if this atlas has been merged before */
                size_t mergedAtlasIdx0 = atlasSizes[i].first;
                if (mergedAtlasIdx0 == size_t(-1)) { continue; }

                for (size_t j = 0; j < smallAtlasSizes.size() && mergedFaceNum < 1; j++)
                {
                    /* continue if this atlas has been merged before */
                    size_t mergedAtlasIdx1 = smallAtlasSizes[j].first;
                    if (mergedAtlasIdx1 == size_t(-1)) { continue; }

                    std::set<int> & atlasEdges0 = pAtlasInfo->atlasEdges[mergedAtlasIdx0];
                    std::set<int> & atlasEdges1 = pAtlasInfo->atlasEdges[mergedAtlasIdx1];

                    /* if two atlas contains same edges, them it can be merged tother */
                    if (!IsIntersect(atlasEdges0, atlasEdges1)) { continue; }

                    /* update atlas info and mesh property */
                    MergeAtlasInfoAndFProperty(pAtlasInfo, pMesh, atlasIdxFProp, mergedAtlasIdx1, mergedAtlasIdx0);
                    mergedFaceNum++;

                    /* remove the atlas that has been merged */
                    smallAtlasSizes[j].first = size_t(-1);
                    smallAtlasSizes[j].second = size_t(-1);
                    atlasIdxSet.erase(mergedAtlasIdx1);
                }
            }

            if (mergedFaceNum == 0) { break; }
        }

        Logger::Info("Try to merge small atlas into its adjacent atlas.");
    }

    for (size_t atlasIdx : atlasIdxSet)
    {
        size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
        Logger::Info("Atals [%zd]: %zd faces.", atlasIdx, faceNum);
    }

    /* remap the atlas index into a continuous sequence */
    RemapAtlasInfoKey(pAtlasInfo, pMesh, atlasIdxFProp, atlasIdxSet);

    /* visualization used for debug */
    DisplayAtlas(pMesh, pAtlasInfo->atlasNum, MESH_FPROP_ATLAS_INDEX, "Atlas - After Merge");

    return pAtlasInfo->atlasNum;
}

std::unique_ptr<TriMesh[]> AtlasMerger::AtlasOptimize(TriMesh * pMesh, AtlasInfo * pAtlasInfo)
{
    return nullptr;
}

void AtlasMerger::MergeAtlasInfoAndFProperty(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, size_t source, size_t target)
{
    if (pAtlasInfo == nullptr)
    {
        return;
    }

    std::set<int> & atlasEdges1 = pAtlasInfo->atlasEdges[source];
    pAtlasInfo->atlasEdges[target].insert(atlasEdges1.begin(), atlasEdges1.end());
    pAtlasInfo->atlasEdges.erase(source);

    std::set<int> & atlasFaces1 = pAtlasInfo->atlasFaces[source];
    pAtlasInfo->atlasFaces[target].insert(atlasFaces1.begin(), atlasFaces1.end());
    for (int face1 : atlasFaces1)
    {
        pMesh->property(fprop, OpenMesh::FaceHandle(face1)) = target;
    }
    pAtlasInfo->atlasFaces.erase(source);

    std::set<int> & atlasVertices1 = pAtlasInfo->atlasVertices[source];
    pAtlasInfo->atlasVertices[target].insert(atlasVertices1.begin(), atlasVertices1.end());
    pAtlasInfo->atlasVertices.erase(source);

    std::set<int> & atlasHalfedges1 = pAtlasInfo->atlasHalfedges[source];
    pAtlasInfo->atlasHalfedges[target].insert(atlasHalfedges1.begin(), atlasHalfedges1.end());
    pAtlasInfo->atlasHalfedges.erase(source);

    pAtlasInfo->atlasNum--;
}

void AtlasMerger::RemapAtlasInfoKey(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, const std::set<size_t> & atlasIdxSet)
{
    std::unordered_map<size_t, size_t> atlasIdxRemap;
    size_t remappedIdx = 0;
    for (size_t atlasIdx : atlasIdxSet)
    {
        atlasIdxRemap[atlasIdx] = remappedIdx;
        remappedIdx++;
    }

    for (auto fh : pMesh->faces())
    {
        size_t & oldAtlasIdx = pMesh->property(fprop, fh);
        size_t newAtlasIdx = atlasIdxRemap[oldAtlasIdx];
        oldAtlasIdx = newAtlasIdx;
    }

    std::unordered_map<size_t, std::set<int>> newAtlasVertices;
    for (auto & kv : pAtlasInfo->atlasVertices)
    {
        size_t oldAtlasIdx = kv.first;
        size_t newAtlasIdx = atlasIdxRemap[oldAtlasIdx];
        newAtlasVertices[newAtlasIdx] = std::move(kv.second);
    }
    pAtlasInfo->atlasVertices = std::move(newAtlasVertices);

    std::unordered_map<size_t, std::set<int>> newAtlasEdges;
    for (auto & kv : pAtlasInfo->atlasEdges)
    {
        size_t oldAtlasIdx = kv.first;
        size_t newAtlasIdx = atlasIdxRemap[oldAtlasIdx];
        newAtlasEdges[newAtlasIdx] = std::move(kv.second);
    }
    pAtlasInfo->atlasEdges = std::move(newAtlasEdges);

    std::unordered_map<size_t, std::set<int>> newAtlasFaces;
    for (auto & kv : pAtlasInfo->atlasFaces)
    {
        size_t oldAtlasIdx = kv.first;
        size_t newAtlasIdx = atlasIdxRemap[oldAtlasIdx];
        newAtlasFaces[newAtlasIdx] = std::move(kv.second);
    }
    pAtlasInfo->atlasFaces = std::move(newAtlasFaces);

    std::unordered_map<size_t, std::set<int>> newAtlasHalfedges;
    for (auto & kv : pAtlasInfo->atlasHalfedges)
    {
        size_t oldAtlasIdx = kv.first;
        size_t newAtlasIdx = atlasIdxRemap[oldAtlasIdx];
        newAtlasHalfedges[newAtlasIdx] = std::move(kv.second);
    }
    pAtlasInfo->atlasHalfedges = std::move(newAtlasHalfedges);
}

size_t AtlasMerger::IntersectNum(const std::set<int> & edges0, const std::set<int> & edges1)
{
    size_t intersectLength = 0;
    for (int edge0 : edges0)
    {
        if (edges1.count(edge0) > 0)
        {
            intersectLength++;
        }
    }
    return intersectLength;
}

bool AtlasMerger::IsIntersect(const std::set<int> & edges0, const std::set<int> & edges1)
{
    bool bConnected = false;
    for (int edge0 : edges0)
    {
        if (edges1.count(edge0) > 0)
        {
            bConnected = true;
            break;
        }
    }
    return bConnected;
}

void AtlasMerger::SortAtlasSize(std::vector<std::pair<size_t, size_t>> & atlasSizes)
{
    std::sort(atlasSizes.begin(), atlasSizes.end(),
    [](const std::pair<size_t, size_t> & lhs, std::pair<size_t, size_t> & rhs)
    {
        return lhs.second < rhs.second;
    });
}

int32_t AtlasMerger::GetAtlasGenus(AtlasInfo * pAtlasInfo, size_t atlasIdx)
{
    size_t vertexNum = pAtlasInfo->atlasVertices[atlasIdx].size();
    size_t edgeNum = pAtlasInfo->atlasEdges[atlasIdx].size();
    size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
    int32_t euler = static_cast<int32_t>(vertexNum) - static_cast<int32_t>(edgeNum) + static_cast<int32_t>(faceNum);
    int32_t boundaryCount = 1;
    int32_t genus = (2 - euler - boundaryCount) / 2;
    return genus;
}
