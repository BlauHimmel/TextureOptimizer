#include "AtlasMerger.hpp"

#include <unordered_set>
#include <queue>
#include <random>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"
#include "ClusteringAlgorithm.hpp"
#include "BarycentricParameterization.hpp"

const char * MESH_FPROP_ATLAS_INDEX = "atlasIdx";

struct TexCoord2DHasher
{
    size_t operator()(const TriMesh::TexCoord2D & uv) const
    {
        return std::hash<Real>()(uv[0]) ^ std::hash<Real>()(uv[1]);
    }
};

bool AtlasMerger::Run(uint32_t width, uint32_t height, EParameterization parameterization, const std::string & meshOutput, const std::string & textureOutput)
{
    if (m_pMesh == nullptr || m_pTexture == nullptr || !m_pTexture->IsValid())
    {
        return false;
    }

    Logger::Info("Removing small components of the mesh...");
    RemoveSmallComponent(m_pMesh);
    Logger::Info("<RemoveSmallComponent> Completed!");

    Logger::Info("Repairing non-manifold vertices of the mesh...");
    RepairNonManifold(m_pMesh);
    Logger::Info("<RepairNonManifold> Completed!");

/*
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
*/

    Logger::Info("Clustering the mesh...");
    std::unique_ptr<AtlasInfo> atlasInfo = MeshClustering(m_pMesh);
    if (atlasInfo == nullptr)
    {
        Logger::Error("Error occurred when clustering the mesh.");
        return false;
    }
    Logger::Info("<MeshClustering> Completed!");

    Logger::Info("Creating submeshes from atlas...");
    std::unique_ptr<TriMesh[]> subMeshes = CreateSubMeshFromAtlas(m_pMesh, atlasInfo.get());
    if (subMeshes == nullptr)
    {
        Logger::Error("Error occurred when creating submeshes.");
        return false;
    }
    Logger::Info("<CreateSubMeshFromAtlas> Completed!");


    for (size_t i = 0; i < atlasInfo->atlasNum; i++)
    {
        Logger::Info("Processing submesh %zd...", i);

        std::unique_ptr<Texture> subTexture = m_pTexture->Copy();

        std::string submeshOutput = meshOutput;
        std::string subtextureOutput = textureOutput;

        submeshOutput.insert( submeshOutput.find(".opt"), "_" + std::to_string(i));
        subtextureOutput.insert( subtextureOutput.find(".opt"), "_" + std::to_string(i));

        Logger::Info("[%zd] Removing small components of the submesh...", i);
        RemoveSmallComponent(&subMeshes[i]);
        Logger::Info("[%zd] <RemoveSmallComponent> Completed!", i);

        Logger::Info("[%zd] Repairing non-manifold vertices of the submesh...", i);
        RepairNonManifold(&subMeshes[i]);
        Logger::Info("[%zd] <RepairNonManifold> Completed!", i);

        Logger::Info("[%zd] Cutting the submesh...", i);
        if (!CutToDisk(&subMeshes[i]) || GetMeshGenus(&subMeshes[i]) != 0)
        {
            Logger::Error("[%zd] Cannot cut the submesh into a topological disk.", i);
            continue;
        }
        Logger::Info("[%zd] <CutToDisk> Completed!", i);

        Logger::Info("[%zd] Parameterizing the submesh...", i);
        TriMesh uvMesh(subMeshes[i]);
        TriMesh originMesh(subMeshes[i]);
        if (parameterization == EParameterization::EBarycentric)
        {
            BarycentricMapping(uvMesh, EBoundary::Square, EBoundaryFixWeight::Length, ECoefficient::DiscreteHarmonic);
        }
        else if (parameterization == EParameterization::ESLIM)
        {
            SLIMProxy(&uvMesh);
        }
        else if (parameterization == EParameterization::EEBP)
        {
            EBPProxy(&uvMesh);
        }
        else
        {
            Logger::Error("[%zd] Unkonwn parameterization method.", i);
            continue;
        }
        Logger::Info("[%zd] <MeshParameterization> Completed!", i);

        Logger::Info("[%zd] Applying paramterization result...", i);
        if (!MeshUVTransfer(&subMeshes[i], &uvMesh))
        {
            Logger::Error("[%zd] Error occurred when applying parameterizing result.", i);
            continue;
        }
        Logger::Info("[%zd] <MeshUVTransfer> Completed!", i);

        Logger::Info("[%zd] Integrating the subtexture...", i);
        if (!TextureIntegrate(&originMesh, &subMeshes[i], subTexture.get(), width, height))
        {
            Logger::Error("[%zd] Error occurred when integrating the subtexture.", i);
            continue;
        }
        Logger::Info("[%zd] <TextureIntegrate> Completed!", i);

        OpenMesh::IO::Options wOpts;
        wOpts += OpenMesh::IO::Options::FaceTexCoord;

        Logger::Info("[%zd] Writing submesh...", i);
        if (!OpenMesh::IO::write_mesh(subMeshes[i], submeshOutput, wOpts))
        {
            Logger::Error("[%zd] Could not write submesh: \"%s\"!", i, submeshOutput.c_str());
            continue;
        }
        Logger::Info("[%zd] Successfully write submesh: \"%s\"!", i, submeshOutput.c_str());

        Logger::Info("[%zd] Writing subtexture...", i);
        if (!subTexture->Write(subtextureOutput))
        {
            Logger::Error("[%zd] Could not write subtexture: \"%s\"!", i, subtextureOutput.c_str());
            continue;
        }
        Logger::Info("[%zd] Successfully write subtexture: \"%s\"!", i, subtextureOutput.c_str());

        Logger::Info("Submesh %zd completed!", i);
    }

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

    const size_t MERGE_STAGE_1_CONNECT_THRESHOLD = 6;
    const size_t MERGE_STAGE_2_CONNECT_THRESHOLD = 3;
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
                if (IntersectNum(atlasEdges0, atlasEdges1) < MERGE_STAGE_1_CONNECT_THRESHOLD) { continue; }

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

            for (size_t j = 0; j < diskAtlasSizes.size() && mergedFaceNum < 1; j++)
            {
                /* continue if this atlas has been merged before */
                size_t mergedAtlasIdx1 = diskAtlasSizes[j].first;
                if (mergedAtlasIdx1 == size_t(-1)) { continue; }

                std::set<int> & atlasEdges0 = pAtlasInfo->atlasEdges[mergedAtlasIdx0];
                std::set<int> & atlasEdges1 = pAtlasInfo->atlasEdges[mergedAtlasIdx1];

                /* if two atlas contains same edges, them it can be merged tother */
                if (IntersectNum(atlasEdges0, atlasEdges1) < MERGE_STAGE_2_CONNECT_THRESHOLD) { continue; }

                /* update atlas info and mesh property */
                MergeAtlasInfoAndFProperty(pAtlasInfo, pMesh, atlasIdxFProp, mergedAtlasIdx1, mergedAtlasIdx0);
                mergedFaceNum++;

                /* remove the atlas that has been merged */
                diskAtlasSizes[j].first = size_t(-1);
                diskAtlasSizes[j].second = size_t(-1);
                atlasIdxSet.erase(mergedAtlasIdx1);
            }
        }

        if (pAtlasInfo->atlasNum <= FINAL_ATLAS_NUM_THRESHOLD || mergedFaceNum == 0)
        {
            break;
        }
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

std::unique_ptr<AtlasInfo> AtlasMerger::MeshClustering(TriMesh * pMesh)
{
    ClusteringAlgorithm cluster;
    std::vector<std::vector<int>> clusters = cluster.Cluster(pMesh, 16);

    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    pMesh->add_property(atlasIdxFProp, MESH_FPROP_ATLAS_INDEX);

    std::unique_ptr<AtlasInfo> atlasInfo(new AtlasInfo);
    atlasInfo->atlasNum = clusters.size();

    for (size_t i = 0; i < clusters.size(); i++)
    {
        for (size_t j = 0; j < clusters[i].size(); j++)
        {
            const auto fh = OpenMesh::FaceHandle(clusters[i][j]);
            pMesh->property(atlasIdxFProp, fh) = i;

            atlasInfo->atlasFaces[i].insert(fh.idx());
            for (auto vh : pMesh->fv_range(fh))
            {
                atlasInfo->atlasVertices[i].insert(vh.idx());
            }
            for (auto eh : pMesh->fe_range(fh))
            {
                atlasInfo->atlasEdges[i].insert(eh.idx());
            }
            for (auto heh : pMesh->fh_range(fh))
            {
                atlasInfo->atlasHalfedges[i].insert(heh.idx());
            }
        }
    }

    /* visualization used for debug */
    DisplayAtlas(pMesh, atlasInfo->atlasNum, MESH_FPROP_ATLAS_INDEX, "Atlas - After Clustering");

    return atlasInfo;
}

std::unique_ptr<TriMesh[]> AtlasMerger::CreateSubMeshFromAtlas(const TriMesh * pMesh, AtlasInfo * pAtlasInfo)
{
    std::unique_ptr<TriMesh[]> subMeshes(new TriMesh[pAtlasInfo->atlasNum]);
    for (size_t i = 0; i < pAtlasInfo->atlasNum; i++)
    {
        std::unordered_map<int, int> vertexMap;
        std::vector<OpenMesh::VertexHandle> vhs;

        subMeshes[i].request_halfedge_texcoords2D();

        for (int vertex : pAtlasInfo->atlasVertices[i])
        {
            vertexMap[vertex] = static_cast<int>(vhs.size());

            const auto vh = OpenMesh::VertexHandle(vertex);
            const auto & point = pMesh->point(vh);
            const auto newVH = subMeshes[i].add_vertex(point);
            vhs.push_back(newVH);
        }

        for (int face : pAtlasInfo->atlasFaces[i])
        {
            const auto fh = OpenMesh::FaceHandle(face);
            std::vector<OpenMesh::VertexHandle> faceVHs;
            std::unordered_map<int, TriMesh::TexCoord2D> vhUVMap;

            for (const auto heh : pMesh->fh_range(fh))
            {
                const auto vh = pMesh->to_vertex_handle(heh);
                const auto & uv = pMesh->texcoord2D(heh);

                int remappedIdx = vertexMap[vh.idx()];
                vhUVMap[vhs[remappedIdx].idx()] = uv;
                faceVHs.push_back(vhs[remappedIdx]);
            }

            const auto newFH = subMeshes[i].add_face(faceVHs);

            for (const auto heh : subMeshes[i].fh_range(newFH))
            {
                const auto vh = subMeshes[i].to_vertex_handle(heh);
                subMeshes[i].set_texcoord2D(heh, vhUVMap[vh.idx()]);
            }
        }
    }

    return subMeshes;
}

void AtlasMerger::MergeAtlasInfoAndFProperty(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, size_t source, size_t target)
{
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

bool AtlasMerger::TextureIntegrate(const TriMesh * pOriginSubMesh, const TriMesh * pNewSubMesh, Texture * pTexture, uint32_t width, uint32_t height)
{
    if (pOriginSubMesh->n_vertices() != pNewSubMesh->n_vertices() ||
        pOriginSubMesh->n_edges() != pNewSubMesh->n_edges() ||
        pOriginSubMesh->n_faces() != pNewSubMesh->n_faces() ||
        pOriginSubMesh->n_halfedges() != pNewSubMesh->n_halfedges())
    {
        Logger::Error("When integrate texture, input meshes must have same topology.");
        return false;
    }

    uint32_t targetWidth = (width == uint32_t(-1) ? pTexture->Width() : width);
    uint32_t targetHeight = (height == uint32_t(-1) ? pTexture->Height() : height);
    uint32_t targetChannels = pTexture->Channels();

    Texture integratedTexture;
    integratedTexture.Create(targetWidth, targetHeight, targetChannels);

    /* AABB of triangle v0, v1, v2 in image, v* should be texcoord in [0, 1]^2 */
    auto GetTriangleAABBInImageFunc = [](const Vector2 & v0, const Vector2 & v1, const Vector2 & v2, uint32_t width,
        uint32_t height) -> std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> /* AABB[xMin, yMin, xMax, yMax] */
    {
        Real xMin =  std::numeric_limits<Real>::max();
        Real yMin =  std::numeric_limits<Real>::max();
        Real xMax = -std::numeric_limits<Real>::max();
        Real yMax = -std::numeric_limits<Real>::max();

        xMin = std::min(xMin, v0[0]); xMax = std::max(xMax, v0[0]);
        xMin = std::min(xMin, v1[0]); xMax = std::max(xMax, v1[0]);
        xMin = std::min(xMin, v2[0]); xMax = std::max(xMax, v2[0]);

        yMin = std::min(yMin, v0[1]); yMax = std::max(yMax, v0[1]);
        yMin = std::min(yMin, v1[1]); yMax = std::max(yMax, v1[1]);
        yMin = std::min(yMin, v2[1]); yMax = std::max(yMax, v2[1]);

        auto xMinInt = static_cast<uint32_t>(std::floor(xMin * width));
        auto yMinInt = static_cast<uint32_t>(std::floor(yMin * height));
        auto xMaxInt = static_cast<uint32_t>(std::ceil(xMax * width));
        auto yMaxInt = static_cast<uint32_t>(std::ceil(yMax * height));

        xMinInt = std::clamp(xMinInt, 0u, width - 1);
        yMinInt = std::clamp(yMinInt, 0u, height - 1);
        xMaxInt = std::clamp(xMaxInt, 0u, width - 1);
        yMaxInt = std::clamp(yMaxInt, 0u, height - 1);

        return std::make_tuple<>(xMinInt, yMinInt, xMaxInt, yMaxInt);
    };

    /* Whether v is inside triangle v0, v1, v2 */
    auto IsInsideTriangleFunc = [](const Vector2 & v, const Vector2 & v0, const Vector2 & v1, const Vector2 & v2) -> bool
    {
        Real signOfTriangle = (v1[0] - v0[0]) * (v2[1] - v0[1]) - (v1[1] - v0[1]) * (v2[0] - v0[0]);
        Real signOfAB = (v1[0] - v0[0]) * (v[1] - v0[1]) - (v1[1] - v0[1]) * (v[0] - v0[0]);
        Real signOfCA = (v0[0] - v2[0]) * (v[1] - v2[1]) - (v0[1] - v2[1]) * (v[0] - v2[0]);
        Real signOfBC = (v2[0] - v1[0]) * (v[1] - v2[1]) - (v2[1] - v1[1]) * (v[0] - v2[0]);

        bool d1 = (signOfAB * signOfTriangle >= 0);
        bool d2 = (signOfCA * signOfTriangle >= 0);
        bool d3 = (signOfBC * signOfTriangle >= 0);

        return d1 && d2 && d3;
    };

    /* affine matrix from triangle va0, va1, va2 to triangle vb0, vb1, vb2 */
    auto GetTransformFunc = [](const Vector2 & va0, const Vector2 & va1, const Vector2 & va2,
        const Vector2 & vb0, const Vector2 & vb1, const Vector2 & vb2) -> Matrix3
    {
        Matrix3 A;
        A(0, 0) = va0[0]; A(0, 1) = va0[1]; A(0, 2) = 1;
        A(1, 0) = va1[0]; A(1, 1) = va1[1]; A(1, 2) = 1;
        A(2, 0) = va2[0]; A(2, 1) = va2[1]; A(2, 2) = 1;

        Matrix3 B;
        B(0, 0) = vb0[0]; B(0, 1) = vb0[1]; B(0, 2) = 1;
        B(1, 0) = vb1[0]; B(1, 1) = vb1[1]; B(1, 2) = 1;
        B(2, 0) = vb2[0]; B(2, 1) = vb2[1]; B(2, 2) = 1;

        /* Ax=B */
        return A.inverse() * B;
    };

    /* apply affine matrix t (by left multiply) to v */
    auto ApplyTransformFunc = [](const Vector2 & v, const Matrix3 & t) -> Vector2
    {
        Eigen::Matrix<Real, 1, 3> vec(v[0], v[1], 1);
        vec *= t;
        return Vector2(vec[0], vec[1]);
    };

    const uint8_t BLANK = 0, COPIED = 1, EXPANDED = 2;
    std::vector<std::vector<uint8_t>> flag(targetHeight, std::vector<uint8_t>(targetWidth, BLANK));

    for (int i = 0; i < pOriginSubMesh->n_faces(); i++)
    {
        const auto fh = TriMesh::FaceHandle(i);

        auto heha0 = pOriginSubMesh->halfedge_handle(fh);
        auto heha1 = pOriginSubMesh->next_halfedge_handle(heha0);
        auto heha2 = pOriginSubMesh->next_halfedge_handle(heha1);

        const auto & va0 = pOriginSubMesh->texcoord2D(heha0);
        const auto & va1 = pOriginSubMesh->texcoord2D(heha1);
        const auto & va2 = pOriginSubMesh->texcoord2D(heha2);

        auto hehb0 = pNewSubMesh->halfedge_handle(TriMesh::FaceHandle(i));
        auto hehb1 = pNewSubMesh->next_halfedge_handle(hehb0);
        auto hehb2 = pNewSubMesh->next_halfedge_handle(hehb1);

        const auto & vb0 = pNewSubMesh->texcoord2D(hehb0);
        const auto & vb1 = pNewSubMesh->texcoord2D(hehb1);
        const auto & vb2 = pNewSubMesh->texcoord2D(hehb2);

        auto t = GetTransformFunc(vb0, vb1, vb2, va0, va1, va2);

        auto aabb = GetTriangleAABBInImageFunc(vb0, vb1, vb2, targetWidth, targetHeight);

        uint32_t yMin = std::get<1>(aabb);
        uint32_t yMax = std::get<3>(aabb);
        uint32_t xMin = std::get<0>(aabb);
        uint32_t xMax = std::get<2>(aabb);

        for (uint32_t y = yMin; y <= yMax; y++)
        {
            for (uint32_t x = xMin; x <= xMax; x++)
            {
                Vector2 pos(static_cast<Real>(x) / static_cast<Real>(targetWidth), static_cast<Real>(y) / static_cast<Real>(targetHeight));
                if (IsInsideTriangleFunc(pos, vb0, vb1, vb2))
                {
                    Vector2 originPos = ApplyTransformFunc(pos, t);

                    auto x0 = static_cast<uint32_t>(std::floor(originPos[0] * pTexture->Width()));
                    auto y0 = static_cast<uint32_t>(std::floor(originPos[1] * pTexture->Height()));
                    auto x1 = x0 + 1;
                    auto y1 = y0 + 1;

                    x0 = std::clamp(x0, 0u, pTexture->Width() - 1);
                    y0 = std::clamp(y0, 0u, pTexture->Height() - 1);
                    x1 = std::clamp(x1, 0u, pTexture->Width() - 1);
                    y1 = std::clamp(y1, 0u, pTexture->Height() - 1);

                    for (uint32_t c = 0; c < targetChannels; c++)
                    {
                        uint32_t sum = static_cast<uint32_t>(pTexture->At(x0, pTexture->Height() - 1 - y0, c)) +
                                       static_cast<uint32_t>(pTexture->At(x0, pTexture->Height() - 1 - y1, c)) +
                                       static_cast<uint32_t>(pTexture->At(x1, pTexture->Height() - 1 - y0, c)) +
                                       static_cast<uint32_t>(pTexture->At(x1, pTexture->Height() - 1 - y1, c));

                        integratedTexture.At(x, targetHeight - 1 - y, c) = static_cast<uint8_t>(sum / 4);
                    }

                    flag[y][x] = COPIED;
                }
            }
        }
    }

    for (uint32_t y = 0; y < targetWidth; y++)
    {
        for (uint32_t x = 0; x < targetHeight; x++)
        {
            if (flag[y][x] == BLANK)
            {
                uint32_t n = 0;
                std::vector<uint32_t> sums(targetChannels, 0u);

                for (uint32_t y0 = std::max(y - 1, 0u); y0 <= y + 1; y0++)
                {
                    for (uint32_t x0 = std::max(x - 1, 0u); x0 <= x + 1; x0++)
                    {
                        if (x0 < targetWidth && y0 < targetHeight && flag[y0][x0] == COPIED)
                        {
                            n++;
                            for (uint32_t c = 0; c < targetChannels; c++)
                            {
                                sums[c] += static_cast<uint32_t>(integratedTexture.At(x0, y0, c));
                            }
                        }
                    }
                }

                if (n > 0)
                {
                    for (uint32_t c = 0; c < targetChannels; c++)
                    {
                        integratedTexture.At(x, y, c) = static_cast<uint8_t>(sums[c] / n);
                    }
                    flag[y][x] = EXPANDED;
                }
            }
        }
    }

    for (uint32_t y = 0; y < targetWidth; y++)
    {
        for (uint32_t x = 0; x < targetHeight; x++)
        {
            for (uint32_t c = 0; c < targetChannels; c++)
            {
                pTexture->At(x, y, c) = integratedTexture.At(x, y, c);
            }
        }
    }

    return true;
}