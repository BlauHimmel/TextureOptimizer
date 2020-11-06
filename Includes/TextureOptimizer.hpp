#ifndef TEXTURE_OPTIMIZER_HPP
#define TEXTURE_OPTIMIZER_HPP

#include "Declaration.hpp"
#include "Texture.hpp"

enum class EParameterization
{
    EBarycentric = 0,
    ESLIM = 1
};

class TextureOptimizer
{
public:
    TextureOptimizer(TriMesh * pMesh, Texture * pTexture) : m_pMesh(pMesh), m_pTexture(pTexture) { }
    bool Run(uint32_t width, uint32_t height, EParameterization parameterization, TriMesh * pCutMesh = nullptr);

    static bool TextureIntegrate(const TriMesh * pOriginMesh, const TriMesh * pNewMesh, Texture * pTexture, uint32_t width, uint32_t height);
    static void SLIMProxy(TriMesh * pMesh);

private:
    TriMesh * m_pMesh;
    Texture * m_pTexture;
};

#endif //TEXTURE_OPTIMIZER_HPP
