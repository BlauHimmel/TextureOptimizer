#include "TextureOptimizer.hpp"

#include <chrono>

#include "Logger.hpp"
#include "BarycentricParameterization.hpp"

bool TextureOptimizer::Run(uint32_t width, uint32_t height, EParameterization parameterization, TriMesh * pCutMesh)
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

    if (!IsDiskTopology(m_pMesh))
    {
        Logger::Info("Cutting the mesh...");
        if (!CutToDisk(m_pMesh) || GetMeshGenus(m_pMesh) != 0 || GetMeshEuler(m_pMesh) != 1)
        {
            Logger::Error("Cannot cut the mesh into a topological disk.");
            return false;
        }
        Logger::Info("<CutToDisk> Completed!");
    }
    else
    {
        Logger::Info("Since mesh is already a topology disk, <CutToDisk> is passed.");
    }

    TriMesh originMesh(*m_pMesh);
    if (pCutMesh != nullptr)
    {
        *pCutMesh = originMesh;
    }

    Logger::Info("Parameterizing the mesh...");
    TriMesh uvMesh(*m_pMesh);
    Real uvWidth = 1.0, uvHeight = 1.0;
    if (parameterization == EParameterization::EBarycentric)
    {
        BarycentricMapping(uvMesh, EBoundary::Square, EBoundaryFixWeight::Length, ECoefficient::DiscreteHarmonic);
    }
    else if (parameterization == EParameterization::ESLIM)
    {
        SLIMProxy(&uvMesh, &uvWidth, &uvHeight);
    }
    else if (parameterization == EParameterization::EEBP)
    {
        EBPProxy(&uvMesh, &uvWidth, &uvHeight);
    }
    else if (parameterization == EParameterization::ESMI04)
    {
        SMI04Proxy(&uvMesh, &uvWidth, &uvHeight);
    }
    else
    {
        Logger::Error("Unkonwn parameterization method.");
        return false;
    }
    Logger::Info("<MeshParameterization> Completed!");

    Logger::Info("Applying paramterization result...");
    if (!MeshUVTransfer(m_pMesh, &uvMesh))
    {
        Logger::Error("Error occurred when applying parameterizing result.");
        return false;
    }
    Logger::Info("<MeshUVTransfer> Completed!");

    Logger::Info("Integrating the texture...");
    auto integratedWidth = static_cast<uint32_t>((width == uint32_t(-1) ? m_pTexture->Width() : width) * uvWidth);
    auto integratedHeight = static_cast<uint32_t>((height == uint32_t(-1) ? m_pTexture->Height() : height) * uvHeight);
    if (!TextureIntegrate(&originMesh, m_pMesh, m_pTexture, integratedWidth, integratedHeight))
    {
        Logger::Error("Error occurred when integrating the texture.");
        return false;
    }
    Logger::Info("<TextureIntegrate> Completed!");

    return true;
}
