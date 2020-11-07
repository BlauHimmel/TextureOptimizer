#include "AtlasMerger.hpp"

#include <unordered_set>
#include <queue>
#include <random>

#include "IglDebugViewer.hpp"

struct TexCoord2DHasher
{
    size_t operator()(const TriMesh::TexCoord2D & uv) const
    {
        return std::hash<Real>()(uv[0]) ^ std::hash<Real>()(uv[1]);
    }
};

std::unique_ptr<AtlasInfo> AtlasMerger::MarkAtlasIndex(TriMesh * pMesh)
{
    if (pMesh == nullptr) { return nullptr; }

    const std::string MESH_FPROP_ATLAS_INDEX = "atlasIdx";

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

    /* visualization used for debug */
    DisplayAtlas(pMesh, atlasInfo->atlasNum, MESH_FPROP_ATLAS_INDEX);

    return atlasInfo;
}


size_t AtlasMerger::MergeAtlas(TriMesh * pMesh, AtlasInfo * pAtlasInfo)
{
    if (pMesh == nullptr || pAtlasInfo == nullptr)
    {
        return size_t(-1);
    }



    return 0;
}