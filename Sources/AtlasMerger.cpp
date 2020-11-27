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

    std::unique_ptr<Texture[]> subTextures(new Texture[atlasInfo->atlasNum]);

    for (size_t i = 0; i < atlasInfo->atlasNum; i++)
    {
        Logger::Info("Processing submesh %zd...", i);

        m_pTexture->CopyTo(&subTextures[i]);

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
        if (!TextureIntegrate(&originMesh, &subMeshes[i], &subTextures[i], width, height))
        {
            Logger::Error("[%zd] Error occurred when integrating the subtexture.", i);
            continue;
        }
        Logger::Info("[%zd] <TextureIntegrate> Completed!", i);

        Logger::Info("Submesh %zd completed!", i);
    }

    std::string mergedMeshOutput = meshOutput;
    std::string mergedTextureOutput = textureOutput;

    mergedMeshOutput.insert(mergedMeshOutput.find(".opt"), "_Merged");
    mergedTextureOutput.insert(mergedTextureOutput.find(".opt"), "_Merged");

    uint32_t mergedTextureWidth = (width == uint32_t(-1) ? m_pTexture->Width() : width);
    uint32_t mergedTextureHeight = (height == uint32_t(-1) ? m_pTexture->Height() : height);
    uint32_t mergedTextureChannels = m_pTexture->Channels();

    for (size_t i = 0; i < atlasInfo->atlasNum; i++)
    {
        std::string submeshOutput = meshOutput;
        std::string subtextureOutput = textureOutput;

        submeshOutput.insert( submeshOutput.find(".opt"), "_" + std::to_string(i));
        subtextureOutput.insert( subtextureOutput.find(".opt"), "_" + std::to_string(i));

        OpenMesh::IO::Options wOpts;
        wOpts += OpenMesh::IO::Options::FaceTexCoord;

        Logger::Info("Writing submesh [%zd]...", i);
        if (!OpenMesh::IO::write_mesh(subMeshes[i], submeshOutput, wOpts))
        {
            Logger::Error("Could not write submesh [%zd]: \"%s\"!", i, submeshOutput.c_str());
            continue;
        }
        Logger::Info("Successfully write submesh [%zd]: \"%s\"!", i, submeshOutput.c_str());

        Logger::Info("[%zd] Writing subtexture...", i);
        if (!subTextures[i].Write(subtextureOutput))
        {
            Logger::Error("Could not write subtexture [%zd]: \"%s\"!", i, subtextureOutput.c_str());
            continue;
        }
        Logger::Info("Successfully write subtexture [%zd]: \"%s\"!", i, subtextureOutput.c_str());
    }

    Logger::Info("Merging submeshes and textures...");
    if (!MergeResultAndWrite(subMeshes.get(), subTextures.get(), atlasInfo->atlasNum,
        mergedTextureWidth, mergedTextureHeight, mergedTextureChannels, mergedMeshOutput, mergedTextureOutput))
    {
        Logger::Error("Error occurred when merging submeshes and textures.");
        return false;
    }
    Logger::Info(R"(Successfully write merged mesh: "%s" and merged texture: "%s"!)", mergedMeshOutput.c_str(), mergedTextureOutput.c_str());
    Logger::Info("<MergeResultAndWrite> Completed!");

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
    RemapAtlasInfoKeyInSequence(pAtlasInfo, pMesh, atlasIdxFProp, atlasIdxSet);

    /* visualization used for debug */
    DisplayAtlas(pMesh, pAtlasInfo->atlasNum, MESH_FPROP_ATLAS_INDEX, "Atlas - After Merge");

    return pAtlasInfo->atlasNum;
}

std::unique_ptr<AtlasInfo> AtlasMerger::MeshClustering(TriMesh * pMesh)
{
    if (pMesh == nullptr)
    {
        return nullptr;
    }

    std::vector<std::vector<int>> clusters = ClusteringAlgorithm::Cluster(pMesh, 4 * 4);

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
    if (pMesh == nullptr || pAtlasInfo == nullptr)
    {
        return nullptr;
    }

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

bool AtlasMerger::MergeResultAndWrite(const TriMesh * pMeshes, const Texture * pTextures, size_t clusterNum,
    uint32_t mergedTextureWidth, uint32_t mergedTextureHeight, uint32_t mergedTextureChannels,
    const std::string & meshOutput, const std::string & textureOutput)
{
    if (pMeshes == nullptr || pTextures == nullptr || clusterNum == 0)
    {
        return false;
    }

    for (size_t i = 0; i < clusterNum; i++)
    {
        if (pTextures[i].Channels() != mergedTextureChannels)
        {
            Logger::Error("Channels of all subtexture should be equal to the channels of the merged texture.");
            return false;
        }
    }

    auto sqrtClusterNum = static_cast<uint32_t>(std::sqrt(clusterNum));
    uint32_t atlasStride = sqrtClusterNum + (clusterNum % sqrtClusterNum == 0 ? 0 : 1);

    uint32_t mergedTextureStrideX = mergedTextureWidth / atlasStride;
    uint32_t mergedTextureStrideY = mergedTextureHeight / atlasStride;

    TriMesh mergedMesh;
    mergedMesh.request_halfedge_texcoords2D();

    Texture mergedTexture;
    mergedTexture.Create(mergedTextureWidth, mergedTextureHeight, mergedTextureChannels);

    std::unordered_map<int, std::unordered_map<int, int>> subMeshVertexMap;
    std::vector<OpenMesh::VertexHandle> vhs;
    for (int i = 0; i < clusterNum; i++)
    {
        for (const auto vh : pMeshes[i].vertices())
        {
            subMeshVertexMap[i][vh.idx()] = static_cast<int>(vhs.size());

            const auto & point = pMeshes[i].point(vh);
            const auto newVH = mergedMesh.add_vertex(point);
            vhs.push_back(newVH);
        }
    }

    for (int i = 0; i < clusterNum; i++)
    {
        for (const auto fh : pMeshes[i].faces())
        {
            std::vector<OpenMesh::VertexHandle> faceVHs;
            std::unordered_map<int, TriMesh::TexCoord2D> vhUVMap;

            for (const auto heh : pMeshes[i].fh_range(fh))
            {
                const auto vh = pMeshes[i].to_vertex_handle(heh);
                const auto & uv = pMeshes[i].texcoord2D(heh);

                int remappedIdx = subMeshVertexMap[i][vh.idx()];
                vhUVMap[vhs[remappedIdx].idx()] = uv;
                faceVHs.push_back(vhs[remappedIdx]);
            }

            const auto newFH = mergedMesh.add_face(faceVHs);

            for (const auto heh : mergedMesh.fh_range(newFH))
            {
                const auto vh = mergedMesh.to_vertex_handle(heh);
                auto uv = vhUVMap[vh.idx()];
                uv[0] = (uv[0] + static_cast<Real>(i % atlasStride)) / static_cast<Real>(atlasStride);
                uv[1] = (uv[1] + static_cast<Real>(static_cast<int>(i / atlasStride))) / static_cast<Real>(atlasStride);
                mergedMesh.set_texcoord2D(heh, uv);
            }
        }
    }

    for (const auto heh : mergedMesh.halfedges())
    {
        const auto & uv = mergedMesh.texcoord2D(heh);
        if (mergedMesh.is_boundary(heh))
        {
            mergedMesh.set_texcoord2D(heh, { 0.0, 0.0 });
        }
    }

    for (uint32_t i = 0; i < clusterNum; i++)
    {
        uint32_t baseX = mergedTextureStrideX * (i % atlasStride);
        uint32_t baseY = mergedTextureStrideY * (atlasStride - 1 - i / atlasStride);

        for (uint32_t y = 0; y < mergedTextureStrideY; y++)
        {
            Real normalizedY = static_cast<Real>(y) / static_cast<Real>(mergedTextureStrideY - 1);
            for (uint32_t x = 0; x < mergedTextureStrideX; x++)
            {
                Real normalizedX = static_cast<Real>(x) / static_cast<Real>(mergedTextureStrideX - 1);
                auto srcPixel = pTextures[i].Sample(normalizedX, normalizedY);
                uint8_t * pDstPixel = mergedTexture.At(baseX + x, baseY + y);

                for (uint32_t c = 0; c < mergedTexture.Channels(); c++)
                {
                    pDstPixel[c] = srcPixel[c];
                }
            }
        }
    }

    OpenMesh::IO::Options wOpts;
    wOpts += OpenMesh::IO::Options::FaceTexCoord;

    bool bSuccessMesh = OpenMesh::IO::write_mesh(mergedMesh, meshOutput, wOpts);
    bool bSuccessTexture = mergedTexture.Write(textureOutput);

    return bSuccessMesh && bSuccessTexture;
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

void AtlasMerger::RemapAtlasInfoKeyInSequence(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, const std::set<size_t> & atlasIdxSet)
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