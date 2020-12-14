#include "TextureOptimizerV2.hpp"

#include <unordered_set>
#include <queue>
#include <random>
#include <filesystem>

#include <finders_interface.h>
#include <igl/triangle/triangulate.h>
#include <simple_svg.hpp>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"
#include "ClusteringAlgorithm.hpp"
#include "BarycentricParameterization.hpp"
#include "OMT.hpp"
#include "MeshMeasure.hpp"

const char * MESH_FPROP_ATLAS_INDEX = "atlasIdx";

bool TextureOptimizerV2::Run(uint32_t width, uint32_t height, EParameterization parameterization, double clusterThreshold, double omtThreshold,
    const std::string & meshOutput, const std::string & textureOutput)
{
    if (m_pMesh == nullptr || m_pTexture == nullptr || !m_pTexture->IsValid())
    {
        return false;
    }

    Logger::Info("Removing small components of the mesh...");
    RemoveSmallComponent(m_pMesh);
    Logger::Info("<RemoveSmallComponent> Completed!");

    Logger::Info("Repairing non-manifold vertices of the mesh...");
    if (RepairNonManifold(m_pMesh) > 0)
    {
        RemoveSmallComponent(m_pMesh);
    }
    Logger::Info("<RepairNonManifold> Completed!");

    Logger::Info("Clustering the mesh...");
    std::unique_ptr<AtlasInfo> atlasInfo = MeshClustering(m_pMesh, clusterThreshold);
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
    std::vector<Real> subMeshAreas(atlasInfo->atlasNum, 0);
    std::vector<Real> sizeCoeffs(atlasInfo->atlasNum, 0);
    Real maxSubMeshArea = 0.0;
    std::vector<Real> uvWidths(atlasInfo->atlasNum, 1);
    std::vector<Real> uvHeights(atlasInfo->atlasNum, 1);

    for (size_t i = 0; i < atlasInfo->atlasNum; i++)
    {
        Real totalArea = 0.0;
        for (const auto fh : subMeshes[i].faces())
        {
            totalArea += subMeshes[i].calc_face_area(fh);
        }
        subMeshAreas[i] = totalArea;
        maxSubMeshArea = std::max(maxSubMeshArea, totalArea);
    }

    for (size_t i = 0; i < atlasInfo->atlasNum; i++)
    {
        sizeCoeffs[i] = std::sqrt(subMeshAreas[i] / maxSubMeshArea);
    }

    for (size_t i = 0; i < atlasInfo->atlasNum; i++)
    {
        Logger::Info("Processing submesh %zd (%zd vertices, %zd faces)...", i, subMeshes[i].n_vertices(), subMeshes[i].n_faces());

        m_pTexture->CopyTo(&subTextures[i]);

        Logger::Info("Removing small components of the submesh [%zd]...", i);
        RemoveSmallComponent(&subMeshes[i]);
        Logger::Info("<RemoveSmallComponent> Completed [%zd]!", i);

        Logger::Info("Repairing non-manifold vertices of the submesh [%zd]...", i);
        RepairNonManifold(&subMeshes[i]);
        Logger::Info("<RepairNonManifold> Completed [%zd]!", i);

        if (!IsDiskTopology(&subMeshes[i]))
        {
            Logger::Info("Cutting the submesh [%zd]...", i);
            if (!CutToDisk(&subMeshes[i], &subMeshAreas[i]) || GetMeshGenus(&subMeshes[i]) != 0 || GetMeshEuler(&subMeshes[i]) != 1)
            {
                Logger::Error("Cannot cut the submesh [%zd] into a topological disk.", i);
                continue;
            }
            Logger::Info("<CutToDisk> Completed [%zd]!", i);
        }
        else
        {
            Logger::Info("Since submesh [%zd] is already a topology disk, <CutToDisk> is passed.", i);
        }

        Logger::Info("Parameterizing the submesh [%zd]...", i);
        TriMesh uvMesh(subMeshes[i]);
        TriMesh originMesh(subMeshes[i]);
        if (parameterization == EParameterization::EBarycentric)
        {
            BarycentricMapping(uvMesh, EBoundary::Square, EBoundaryFixWeight::Length, ECoefficient::DiscreteHarmonic);
        }
        else if (parameterization == EParameterization::ESLIM)
        {
            SLIMProxy(&uvMesh, &uvWidths[i], &uvHeights[i]);
        }
        else if (parameterization == EParameterization::EEBP)
        {
            EBPProxy(&uvMesh, &uvWidths[i], &uvHeights[i]);
        }
        else if (parameterization == EParameterization::ESMI04)
        {
            double stl2error = 0.0;
            SMI04Proxy(&uvMesh, &uvWidths[i], &uvHeights[i], &stl2error);

            if (stl2error > 10.0)
            {
                Logger::Warn("SMI04 parameterization for submesh [%zd] cause high distortion. Try to use EBP instead.", i);
                uvMesh = subMeshes[i];
                EBPProxy(&uvMesh, &uvWidths[i], &uvHeights[i]);
            }
        }
        else
        {
            Logger::Error("Unkonwn parameterization method [%zd].", i);
            continue;
        }
        Logger::Info("<MeshParameterization> Completed [%zd]!", i);

        Logger::Info("Applying paramterization result [%zd]...", i);
        if (!MeshUVTransfer(&subMeshes[i], &uvMesh))
        {
            Logger::Error("Error occurred when applying parameterizing result [%zd].", i);
            continue;
        }
        Logger::Info("<MeshUVTransfer> Completed [%zd]!", i);

        Logger::Info("Integrating the subtexture [%zd]...", i);

        auto integratedWidth = static_cast<uint32_t>((width == uint32_t(-1) ? subTextures[i].Width() : width) * sizeCoeffs[i] * uvWidths[i]);
        auto integratedHeight = static_cast<uint32_t>((height == uint32_t(-1) ? subTextures[i].Height() : height) * sizeCoeffs[i] * uvHeights[i]);

        if (!TextureIntegrate(&originMesh, &subMeshes[i], &subTextures[i], integratedWidth, integratedHeight))
        {
            Logger::Error("Error occurred when integrating the subtexture [%zd].", i);
            continue;
        }
        Logger::Info("<TextureIntegrate> Completed [%zd]!", i);

        Logger::Info("Submesh %zd completed!", i);
    }

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

        Logger::Info("Writing subtexture [%zd]...", i);
        if (!subTextures[i].Write(subtextureOutput))
        {
            Logger::Error("Could not write subtexture [%zd]: \"%s\"!", i, subtextureOutput.c_str());
            continue;
        }
        Logger::Info("Successfully write subtexture [%zd]: \"%s\"!", i, subtextureOutput.c_str());
    }

    std::string mergedMeshOutput = meshOutput;
    std::string mergedTextureOutput = textureOutput;

    mergedMeshOutput.insert(mergedMeshOutput.find(".opt"), "_Merged");
    mergedTextureOutput.insert(mergedTextureOutput.find(".opt"), "_Merged");

    auto sqrtAtlasNum = static_cast<uint32_t>(std::sqrt(atlasInfo->atlasNum));
    uint32_t mergedTextureWidth = (width == uint32_t(-1) ? m_pTexture->Width() : width) * sqrtAtlasNum;
    uint32_t mergedTextureHeight = (height == uint32_t(-1) ? m_pTexture->Height() : height) * sqrtAtlasNum;
    uint32_t mergedTextureChannels = m_pTexture->Channels();

    TriMesh mergedMesh;
    Texture mergedTexture;

    Logger::Info("Merging submeshes and textures...");
    if (!MergeResultAndWriteV2(subMeshes.get(), subTextures.get(), atlasInfo->atlasNum, mergedTextureWidth, mergedTextureHeight,
        mergedTextureChannels, mergedMeshOutput, mergedTextureOutput, &mergedMesh, &mergedTexture))
    {
        Logger::Error("Error occurred when merging submeshes and textures.");
        return false;
    }
    Logger::Info(R"(Successfully write merged mesh: "%s" and merged texture: "%s"!)", mergedMeshOutput.c_str(), mergedTextureOutput.c_str());
    Logger::Info("<MergeResultAndWrite> Completed!");

    std::string optMergedMeshOutput = meshOutput;
    std::string optMergedTextureOutput = textureOutput;

    optMergedMeshOutput.insert(optMergedMeshOutput.find(".opt"), "_OptMerged");
    optMergedTextureOutput.insert(optMergedTextureOutput.find(".opt"), "_OptMerged");

    if (parameterization != EParameterization::ESMI04 && parameterization != EParameterization::EBarycentric)
    {
        Logger::Info("Further optimizing the merged mesh and the merged textures...");
        if (!FurtherOptimizeAndWrite(&mergedMesh, &mergedTexture, atlasInfo->atlasNum, omtThreshold, optMergedMeshOutput, optMergedTextureOutput))
        {
            Logger::Error("Error occurred when further optimizing the merged mesh and the merged textures.");
            return false;
        }
        Logger::Info(R"(Successfully write optimized merged mesh: "%s" and optimized merged texture: "%s"!)", optMergedMeshOutput.c_str(), optMergedTextureOutput.c_str());
        Logger::Info("<FurtherOptimize> Completed!");
    }

    return true;
}

struct TexCoord2DHasher
{
    size_t operator()(const TriMesh::TexCoord2D & uv) const
    {
        return std::hash<Real>()(uv[0]) ^ std::hash<Real>()(uv[1]);
    }
};

[[maybe_unused]] std::unique_ptr<AtlasInfo> TextureOptimizerV2::MarkAtlasIndex(TriMesh * pMesh)
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

[[maybe_unused]] size_t TextureOptimizerV2::MergeAtlas(TriMesh * pMesh, AtlasInfo * pAtlasInfo)
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

std::unique_ptr<AtlasInfo> TextureOptimizerV2::MeshClustering(TriMesh * pMesh, double clusterThreshold)
{
    if (pMesh == nullptr)
    {
        return nullptr;
    }

    std::vector<std::vector<int>> clusters = ClusteringAlgorithm::Cluster(pMesh, 3 * 3, clusterThreshold);

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

    Logger::Info("Cluster count = %zd", atlasInfo->atlasNum);

    /* visualization used for debug */
    DisplayAtlas(pMesh, atlasInfo->atlasNum, MESH_FPROP_ATLAS_INDEX, "Atlas - After Clustering");

    return atlasInfo;
}

std::unique_ptr<TriMesh[]> TextureOptimizerV2::CreateSubMeshFromAtlas(const TriMesh * pMesh, AtlasInfo * pAtlasInfo)
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

[[maybe_unused]] bool TextureOptimizerV2::MergeResultAndWrite(const TriMesh * pMeshes, const Texture * pTextures, const Real * pSizeCoeffs,
    const Real * uvWidths, const Real * uvHeights, size_t clusterNum, uint32_t mergedTextureWidth, uint32_t mergedTextureHeight,
    uint32_t mergedTextureChannels, const std::string & meshOutput, const std::string & textureOutput,
    TriMesh * pMergedMesh, Texture * pMergedTexture)
{
    if (pMeshes == nullptr || pTextures == nullptr || pSizeCoeffs == nullptr || uvWidths == nullptr || uvHeights == nullptr || clusterNum == 0)
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
                uv[0] = (uv[0] * pSizeCoeffs[i] * uvWidths[i] + static_cast<Real>(i % atlasStride)) / static_cast<Real>(atlasStride);
                uv[1] = (uv[1] * pSizeCoeffs[i] * uvHeights[i] + static_cast<Real>(static_cast<int>(i / atlasStride))) / static_cast<Real>(atlasStride);
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

        auto actualStrideX = static_cast<uint32_t>(mergedTextureStrideX * pSizeCoeffs[i] * uvWidths[i]);
        auto actualStrideY = static_cast<uint32_t>(mergedTextureStrideY * pSizeCoeffs[i] * uvHeights[i]);

        for (uint32_t y = 0; y < actualStrideY; y++)
        {
            Real normalizedY = static_cast<Real>(y) / static_cast<Real>(actualStrideY - 1);
            uint32_t offsetY = mergedTextureStrideY - actualStrideY;
            for (uint32_t x = 0; x < actualStrideX; x++)
            {
                Real normalizedX = static_cast<Real>(x) / static_cast<Real>(actualStrideX - 1);
                auto srcPixel = pTextures[i].Sample(normalizedX, normalizedY);
                uint8_t * pDstPixel = mergedTexture.At(baseX + x, baseY + y + offsetY);

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

    if (bSuccessMesh && bSuccessTexture)
    {
        pMergedMesh->assign(mergedMesh, true);
        mergedTexture.CopyTo(pMergedTexture);
    }

    return bSuccessMesh && bSuccessTexture;
}

bool TextureOptimizerV2::MergeResultAndWriteV2(const TriMesh * pMeshes, const Texture * pTextures,
    size_t clusterNum, uint32_t mergedTextureWidth, uint32_t mergedTextureHeight,
    uint32_t mergedTextureChannels, const std::string & meshOutput, const std::string & textureOutput,
    TriMesh * pMergedMesh, Texture * pMergedTexture)
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

    const auto bAllowFlip = false;
    const auto runtimeFlippingMode = rectpack2D::flipping_option::DISABLED;
    using TSpaces = rectpack2D::empty_spaces<bAllowFlip, rectpack2D::default_empty_spaces>;
    using TRect = rectpack2D::output_rect_t<TSpaces>;

    auto ReportSuccessFunc = [](TRect &)
    {
        return rectpack2D::callback_result::CONTINUE_PACKING;
    };
    auto ReportFailFunc = [](TRect &)
    {
        return rectpack2D::callback_result::ABORT_PACKING;
    };

    const auto maxSide = static_cast<int>(std::max(mergedTextureWidth, mergedTextureHeight));
    const auto discardStep = 1;

    std::vector<TRect> rectangles;
    for (int i = 0; i < clusterNum; i++)
    {
        auto w = static_cast<int>(pTextures[i].Width());
        auto h = static_cast<int>(pTextures[i].Height());
        rectangles.emplace_back(rectpack2D::rect_xywh(0, 0, w, h));
    }

    const auto resultSize = rectpack2D::find_best_packing<TSpaces>(
        rectangles,
        rectpack2D::make_finder_input(
            maxSide,
            discardStep,
            ReportSuccessFunc,
            ReportFailFunc,
            runtimeFlippingMode
        )
    );

    std::vector<Real> packBaseX(clusterNum, 0);
    std::vector<Real> packBaseY(clusterNum, 0);
    std::vector<Real> packWidth(clusterNum, 0);
    std::vector<Real> packHeight(clusterNum, 0);

    for (int i = 0; i < clusterNum; i++)
    {
        packBaseX[i] = static_cast<Real>(rectangles[i].x) / static_cast<Real>(resultSize.w);
        packBaseY[i] = static_cast<Real>(rectangles[i].y) / static_cast<Real>(resultSize.h);
        packWidth[i] = static_cast<Real>(rectangles[i].w) / static_cast<Real>(resultSize.w);
        packHeight[i] = static_cast<Real>(rectangles[i].h) / static_cast<Real>(resultSize.h);
    }

    mergedTextureWidth = static_cast<uint32_t>(resultSize.w);
    mergedTextureHeight = static_cast<uint32_t>(resultSize.h);

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
                uv[0] = (uv[0] * packWidth[i]) + packBaseX[i];
                uv[1] = (uv[1] * packHeight[i]) + packBaseY[i];
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
        auto actualStrideX = static_cast<uint32_t>(rectangles[i].w);
        auto actualStrideY = static_cast<uint32_t>(rectangles[i].h);

        auto baseX = static_cast<uint32_t>(rectangles[i].x);
        auto baseY = static_cast<uint32_t>(rectangles[i].y);

        for (uint32_t y = 0; y < actualStrideY; y++)
        {
            Real normalizedY = 1.0 - static_cast<Real>(y) / static_cast<Real>(actualStrideY - 1);
            for (uint32_t x = 0; x < actualStrideX; x++)
            {
                Real normalizedX = static_cast<Real>(x) / static_cast<Real>(actualStrideX - 1);
                auto srcPixel = pTextures[i].Sample(normalizedX, normalizedY);
                uint8_t * pDstPixel = mergedTexture.At(baseX + x, mergedTextureHeight - 1 - (baseY + y));

                for (uint32_t c = 0; c < mergedTexture.Channels(); c++)
                {
                    pDstPixel[c] = srcPixel[c];
                }
            }
        }
    }

    std::filesystem::path textureOutputPath(textureOutput);
    std::string atlasSVGOutput = textureOutputPath.replace_extension(".svg").string();

    OpenMesh::IO::Options wOpts;
    wOpts += OpenMesh::IO::Options::FaceTexCoord;

    bool bSuccessMesh = OpenMesh::IO::write_mesh(mergedMesh, meshOutput, wOpts);
    bool bSuccessTexture = mergedTexture.Write(textureOutput);
    bool bSuccessAtlasSVG = WriteAtlasSVG(&mergedMesh, clusterNum, 1000, atlasSVGOutput);

    if (bSuccessMesh && bSuccessTexture && bSuccessAtlasSVG)
    {
        pMergedMesh->assign(mergedMesh, true);
        mergedTexture.CopyTo(pMergedTexture);
    }

    return bSuccessMesh && bSuccessTexture && bSuccessAtlasSVG;
}

struct PointHasher
{
    size_t operator()(const TriMesh::Point & pt) const
    {
        return std::hash<Real>()(pt[0]) ^ std::hash<Real>()(pt[1]) ^ std::hash<Real>()(pt[2]);
    }
};

bool TextureOptimizerV2::FurtherOptimizeAndWrite(const TriMesh * pMergedMesh, const Texture * pMergedTexture, size_t clusterNum,
    double omtThreshold, const std::string & meshOutput, const std::string & textureOutput)
{
    if (pMergedMesh == nullptr || pMergedTexture == nullptr)
    {
        return false;
    }

    /* Construct scaffold for the optimized uv parameterization mesh */

    TriMesh uvMesh;
    std::vector<OpenMesh::VertexHandle> vhs;
    vhs.reserve(pMergedMesh->n_vertices());

    for (const auto vh : pMergedMesh->vertices())
    {
        TriMesh::TexCoord2D uv;
        for (const auto heh : pMergedMesh->vih_range(vh))
        {
            if (!pMergedMesh->is_boundary(heh))
            {
                uv = pMergedMesh->texcoord2D(heh);
                break;
            }
        }

        for (const auto heh : pMergedMesh->vih_range(vh))
        {
            if (!pMergedMesh->is_boundary(heh))
            {
                if (pMergedMesh->texcoord2D(heh) != uv)
                {
                    Logger::Error("Multiple different texcoords assigned to single vertex!");
                    return false;
                }
            }
        }

        const auto newVH = uvMesh.add_vertex({uv[0], uv[1], 0.0});
        vhs.push_back(newVH);
    }

    for (const auto fh : pMergedMesh->faces())
    {
        std::vector<OpenMesh::VertexHandle> faceVHs;
        for (const auto vh :pMergedMesh->fv_range(fh))
        {
            faceVHs.push_back(vhs[vh.idx()]);
        }
        uvMesh.add_face(faceVHs);
    }

    const size_t boundarySubdivisionTimes = 50;
    const Real boundarySubdivisionDelta = 1.0 / static_cast<Real>(boundarySubdivisionTimes);

    auto boundaries = MeshBoundaryVertices(&uvMesh);
    size_t totalBoundaryVertexNum = 4 * boundarySubdivisionTimes;
    for (const auto & boundary : boundaries)
    {
        totalBoundaryVertexNum += boundary.size();
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    Eigen::MatrixXd H;
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;

    V.resize(totalBoundaryVertexNum,2);
    E.resize(totalBoundaryVertexNum,2);

    std::vector<std::vector<Real>> holeVertices;

    size_t idxBase = 0;

    for (const auto & boundary : boundaries)
    {
        for (size_t j = 0; j < boundary.size(); j++)
        {
            const auto & pt = uvMesh.point(boundary[j]);

            V(idxBase + j, 0) = pt[0];
            V(idxBase + j, 1) = pt[1];

            E(idxBase + j, 0) = idxBase + j;
            E(idxBase + j, 1) = idxBase + ((j + 1) % boundary.size());
        }

        std::set<int> holeVertexSet;

        for (const auto & boundarVH : boundary)
        {
            for (const auto vh : uvMesh.vv_range(boundarVH))
            {
                if (!uvMesh.is_boundary(vh) && holeVertexSet.count(vh.idx()) == 0)
                {
                    const auto & pt = uvMesh.point(vh);
                    holeVertices.push_back({ pt[0], pt[1] });
                    holeVertexSet.insert(vh.idx());
                }
            }
        }

        idxBase += boundary.size();
    }

    Real boundaryX = 0.0, boundaryY = 0.0;
    for (size_t i = 0; i < boundarySubdivisionTimes; i++)
    {
        V(idxBase + i, 0) =  boundaryX * 1.05 - 0.025;
        V(idxBase + i, 1) =  boundaryY * 1.05 - 0.025;
        E(idxBase + i, 0) = idxBase + i;
        E(idxBase + i, 1) = idxBase + i + 1;
        boundaryX += boundarySubdivisionDelta;
    }

    for (size_t i = boundarySubdivisionTimes; i < 2 * boundarySubdivisionTimes; i++)
    {
        V(idxBase + i, 0) =  boundaryX * 1.05 - 0.025;
        V(idxBase + i, 1) =  boundaryY * 1.05 - 0.025;
        E(idxBase + i, 0) = idxBase + i;
        E(idxBase + i, 1) = idxBase + i + 1;
        boundaryY += boundarySubdivisionDelta;
    }

    for (size_t i = 2 * boundarySubdivisionTimes; i < 3 * boundarySubdivisionTimes; i++)
    {
        V(idxBase + i, 0) =  boundaryX * 1.05 - 0.025;
        V(idxBase + i, 1) =  boundaryY * 1.05 - 0.025;
        E(idxBase + i, 0) = idxBase + i;
        E(idxBase + i, 1) = idxBase + i + 1;
        boundaryX -= boundarySubdivisionDelta;
    }

    for (size_t i = 3 * boundarySubdivisionTimes; i < 4 * boundarySubdivisionTimes; i++)
    {
        V(idxBase + i, 0) =  boundaryX * 1.05 - 0.025;
        V(idxBase + i, 1) =  boundaryY * 1.05 - 0.025;
        E(idxBase + i, 0) = idxBase + i;
        E(idxBase + i, 1) = idxBase + ((i + 1) % (4 * boundarySubdivisionTimes));
        boundaryY -= boundarySubdivisionDelta;
    }

    H.resize(holeVertices.size(), 2);
    for (size_t i = 0; i < holeVertices.size(); i++)
    {
        H(i, 0) = holeVertices[i][0];
        H(i, 1) = holeVertices[i][1];
    }

    igl::triangle::triangulate(V, E, H,"a0.001q15YYQ",V2,F2);

    TriMesh scaffoldUVMesh;
    scaffoldUVMesh.request_vertex_texcoords2D();
    vhs.clear();
    std::unordered_map<TriMesh::Point, int, PointHasher> boundaryVertexIdxMap;
    std::vector<int> faceTypes(uvMesh.n_faces() + F2.rows(), -1); /* 0 for scaffold, 1 for mesh */

    for (const auto vh : uvMesh.vertices())
    {
        const auto & pt = uvMesh.point(vh);
        const auto newVH = scaffoldUVMesh.add_vertex(pt);
        if (uvMesh.is_boundary(vh))
        {
            boundaryVertexIdxMap[pt] = vhs.size();
        }
        vhs.push_back(newVH);
    }

    for (const auto fh : uvMesh.faces())
    {
        std::vector<OpenMesh::VertexHandle> faceVHs;
        for (const auto vh :uvMesh.fv_range(fh))
        {
            faceVHs.push_back(vhs[vh.idx()]);
        }
        const auto newFH = scaffoldUVMesh.add_face(faceVHs);
        faceTypes[newFH.idx()] = 1;
    }

    std::vector<OpenMesh::VertexHandle> scaffoldVHs;
    for (Eigen::Index i = 0; i < V2.rows(); i++)
    {
        const TriMesh::Point pt = { V2(i, 0), V2(i, 1), 0.0 };
        if (boundaryVertexIdxMap.count(pt) == 0)
        {
            const auto newVH = scaffoldUVMesh.add_vertex(pt);
            scaffoldVHs.push_back(newVH);
        }
        else
        {
            scaffoldVHs.push_back(TriMesh::InvalidVertexHandle);
        }
    }

    for (Eigen::Index i = 0; i < F2.rows(); i++)
    {
        std::vector<OpenMesh::VertexHandle> faceVHs;
        for (Eigen::Index j = 0; j < 3; j++)
        {
            int idx = F2(i, j);
            const TriMesh::Point pt = { V2(idx, 0), V2(idx, 1), 0.0 };
            if (boundaryVertexIdxMap.count(pt) == 0)
            {
                faceVHs.push_back(scaffoldVHs[idx]);
            }
            else
            {
                faceVHs.push_back(vhs[boundaryVertexIdxMap[pt]]);
            }
        }
        const auto newFH = scaffoldUVMesh.add_face(faceVHs);
        faceTypes[newFH.idx()] = 0;
    }

    for (const auto vh : scaffoldUVMesh.vertices())
    {
        const auto & pt = scaffoldUVMesh.point(vh);
        scaffoldUVMesh.set_texcoord2D(vh, { pt[0], pt[1] });
    }

    {
        /* visualization used for debug */
        Eigen::MatrixXi FF;
        FF.resize(scaffoldUVMesh.n_faces(), 3);

        for (int i = 0; i < scaffoldUVMesh.n_faces(); i++)
        {
            const auto fh = TriMesh::FaceHandle(i);

            auto fvIter = scaffoldUVMesh.fv_range(fh).begin();
            auto fvIterEnd = scaffoldUVMesh.fv_range(fh).end();

            const auto vh0 = *fvIter;
            *fvIter++;
            const auto vh1 = *fvIter;
            *fvIter++;
            const auto vh2 = *fvIter;
            *fvIter++;

            FF(i, 0) = vh0.idx();
            FF(i, 1) = vh1.idx();
            FF(i, 2) = vh2.idx();
        }

        Eigen::MatrixXd VV;
        VV.resize(scaffoldUVMesh.n_vertices(), 3);

        for (int i = 0; i < scaffoldUVMesh.n_vertices(); i++)
        {
            const auto vh = TriMesh::VertexHandle (i);
            const auto & point = scaffoldUVMesh.point(vh);

            VV(i, 0) = point[0];
            VV(i, 1) = point[1];
            VV(i, 2) = point[2];
        }

        Eigen::MatrixXd CC;
        CC.resize(FF.rows(), 3);
        for (size_t i = 0; i < CC.rows(); i++)
        {
            if (i < uvMesh.n_faces())
            {
                CC(i, 0) = 1.0;
                CC(i, 1) = 1.0;
                CC(i, 2) = 0.0;
            }
            else
            {
                CC(i, 0) = 1.0;
                CC(i, 1) = 1.0;
                CC(i, 2) = 1.0;
            }
        }

        DisplayVFC(VV, FF, CC, "Scaffold UV Mesh");
        DisplayVF(V2, F2, "Scaffold");
    }

    /* Use OMT to optimize the scaffold uv mesh. */

    std::vector<float> targetAreas;
    targetAreas.resize(scaffoldUVMesh.n_vertices(), 0.0f);
    const double scaffoldAreaShrinkFactor = 0.015;

    Real totalMeshArea = 0.0f;
    for (const auto & fh : scaffoldUVMesh.faces())
    {
        TriMesh::Point points[3];
        size_t Index = 0;

        for (const auto & vh : scaffoldUVMesh.fv_range(fh))
        {
            points[Index++] = scaffoldUVMesh.point(vh);
        }

        const auto & va = points[0];
        const auto & vb = points[1];
        const auto & vc = points[2];

        auto lba = va - vb;
        auto lbc = vc - vb;

        Real area = lba.cross(lbc).norm() * 0.5;

        if (fh.idx() < uvMesh.n_faces())
        {
            totalMeshArea += area;
        }
        else
        {
            totalMeshArea += area * scaffoldAreaShrinkFactor;
        }
    }

    for (const auto & vh : scaffoldUVMesh.vertices())
    {
        Real oneRingMeshArea = 0.0;
        for (const auto & fh : scaffoldUVMesh.vf_range(vh))
        {
            TriMesh::Point points[3];
            size_t Index = 0;

            for (const auto & fhvh : scaffoldUVMesh.fv_range(fh))
            {
                points[Index++] = scaffoldUVMesh.point(fhvh);
            }

            const auto & va = points[0];
            const auto & vb = points[1];
            const auto & vc = points[2];

            auto lba = va - vb;
            auto lbc = vc - vb;

            Real area = lba.cross(lbc).norm() * 0.5;

            if (fh.idx() < uvMesh.n_faces())
            {
                oneRingMeshArea += area / 3.0;
            }
            else
            {
                oneRingMeshArea += area / 3.0 * scaffoldAreaShrinkFactor;
            }
        }

        const Real totalTargetArea = 4.0;
        targetAreas[vh.idx()] = static_cast<float>(oneRingMeshArea / totalMeshArea * totalTargetArea);
    }

    OMT2D omt(&scaffoldUVMesh, targetAreas);
    if (!omt.Initialize())
    {
        Logger::Error("Failed to initialize OMT2D.");
        return false;
    }
    omt.Run(omtThreshold);

    scaffoldUVMesh.request_face_status();
    scaffoldUVMesh.request_edge_status();
    scaffoldUVMesh.request_vertex_status();

    for (const auto fh : scaffoldUVMesh.faces())
    {
        if (fh.idx() >= uvMesh.n_faces())
        {
            scaffoldUVMesh.delete_face(fh);
        }
    }

    scaffoldUVMesh.garbage_collection();
    scaffoldUVMesh.release_face_status();
    scaffoldUVMesh.release_edge_status();
    scaffoldUVMesh.release_vertex_status();

    TriMesh furtherOptMesh(*pMergedMesh);
    for (const auto fh : furtherOptMesh.faces())
    {
        for (const auto heh : furtherOptMesh.fh_range(fh))
        {
            const auto vh = furtherOptMesh.to_vertex_handle(heh);
            const auto & uv = scaffoldUVMesh.texcoord2D(vh);
            furtherOptMesh.set_texcoord2D(heh, uv);
        }
    }

    Texture furtherOptTexture;
    pMergedTexture->CopyTo(&furtherOptTexture);
    if (!TextureIntegrate(pMergedMesh, &furtherOptMesh, &furtherOptTexture, pMergedTexture->Width(), pMergedTexture->Height()))
    {
        return false;
    }

    std::string meshAreaDistortionStatsOutput = std::filesystem::path(meshOutput).replace_extension(".area.txt").string();
    std::string meshAreaDistortionFigureOutput = std::filesystem::path(meshOutput).replace_extension(".area.svg").string();
    std::string meshAngleDistortionStatsOutput = std::filesystem::path(meshOutput).replace_extension(".angle.txt").string();
    std::string meshAngleDistortionFigureOutput = std::filesystem::path(meshOutput).replace_extension(".angle.svg").string();
    std::string meshSymDirichletDistortionStatsOutput = std::filesystem::path(meshOutput).replace_extension(".symdirichlet.txt").string();
    std::string meshSymDirichletDistortionFigureOutput = std::filesystem::path(meshOutput).replace_extension(".symdirichlet.svg").string();
    std::string meshEnergyStatsOutput = std::filesystem::path(meshOutput).replace_extension(".energy.txt").string();

    MeshMeasure::AreaDistortion(&furtherOptMesh, meshAreaDistortionStatsOutput, meshAreaDistortionFigureOutput);
    MeshMeasure::AngleDistortion(&furtherOptMesh, meshAngleDistortionStatsOutput, meshAngleDistortionFigureOutput);
    MeshMeasure::SymmetricDirichletDistortion(&furtherOptMesh, meshSymDirichletDistortionStatsOutput, meshSymDirichletDistortionFigureOutput);
    MeshMeasure::EnergyStats(&furtherOptMesh, clusterNum, meshEnergyStatsOutput);

    std::filesystem::path textureOutputPath(textureOutput);
    std::string atlasSVGOutput = textureOutputPath.replace_extension(".svg").string();

    OpenMesh::IO::Options wOpts;
    wOpts += OpenMesh::IO::Options::FaceTexCoord;
    bool bSuccessMesh = OpenMesh::IO::write_mesh(furtherOptMesh, meshOutput, wOpts);
    bool bSuccessTexture = furtherOptTexture.Write(textureOutput);
    bool bSuccessAtlasSVG = WriteAtlasSVG(&furtherOptMesh, clusterNum, 1000, atlasSVGOutput);

    return bSuccessMesh && bSuccessTexture && bSuccessAtlasSVG;
}

void TextureOptimizerV2::MergeAtlasInfoAndFProperty(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, size_t source, size_t target)
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

[[maybe_unused]] void TextureOptimizerV2::RemapAtlasInfoKeyInSequence(AtlasInfo * pAtlasInfo, TriMesh * pMesh, OpenMesh::FPropHandleT<size_t> & fprop, const std::set<size_t> & atlasIdxSet)
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

[[maybe_unused]] size_t TextureOptimizerV2::IntersectNum(const std::set<int> & edges0, const std::set<int> & edges1)
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

bool TextureOptimizerV2::IsIntersect(const std::set<int> & edges0, const std::set<int> & edges1)
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

[[maybe_unused]] void TextureOptimizerV2::SortAtlasSize(std::vector<std::pair<size_t, size_t>> & atlasSizes)
{
    std::sort(atlasSizes.begin(), atlasSizes.end(),
    [](const std::pair<size_t, size_t> & lhs, std::pair<size_t, size_t> & rhs)
    {
        return lhs.second < rhs.second;
    });
}

[[maybe_unused]] int32_t TextureOptimizerV2::GetAtlasGenus(AtlasInfo * pAtlasInfo, size_t atlasIdx)
{
    size_t vertexNum = pAtlasInfo->atlasVertices[atlasIdx].size();
    size_t edgeNum = pAtlasInfo->atlasEdges[atlasIdx].size();
    size_t faceNum = pAtlasInfo->atlasFaces[atlasIdx].size();
    int32_t euler = static_cast<int32_t>(vertexNum) - static_cast<int32_t>(edgeNum) + static_cast<int32_t>(faceNum);
    int32_t boundaryCount = 1;
    int32_t genus = (2 - euler - boundaryCount) / 2;
    return genus;
}

bool TextureOptimizerV2::WriteAtlasSVG(const TriMesh * pMesh, size_t atlasNum, size_t size, const std::string & output)
{
    if (pMesh == nullptr || atlasNum == 0 || size == 0)
    {
        return false;
    }

    std::vector<std::vector<OpenMesh::VertexHandle>> boundaryVertices = MeshBoundaryVertices(pMesh);

    svg::Dimensions dimensions(size, size);
    svg::Document doc(output, svg::Layout(dimensions, svg::Layout::BottomLeft));

    svg::Polygon border(svg::Stroke(3, svg::Color::Black));
    border << svg::Point(0, 0) << svg::Point(dimensions.width, 0) << svg::Point(dimensions.width, dimensions.height) << svg::Point(0, dimensions.height);
    doc << border;

    std::default_random_engine engine(pMesh->n_vertices());
    std::uniform_int_distribution<int> uniform(0, 255);

    for (const auto & boundaryVertex : boundaryVertices)
    {
        svg::Polygon atlasPolygon = svg::Polygon(svg::Color(uniform(engine), uniform(engine), uniform(engine)), svg::Stroke(2, svg::Color::Black));
        for (const auto & vh : boundaryVertex)
        {
            for (const auto heh : pMesh->vih_range(vh))
            {
                if (!pMesh->is_boundary(heh))
                {
                    const auto & uv = pMesh->texcoord2D(heh);
                    atlasPolygon << svg::Point(uv[0] * size, uv[1] * size);
                    break;
                }
            }
        }
        doc << atlasPolygon;
    }

    return doc.save();
}