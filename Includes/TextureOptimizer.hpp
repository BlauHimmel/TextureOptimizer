#ifndef TEXTURE_OPTIMIZER_HPP
#define TEXTURE_OPTIMIZER_HPP

#include "Declaration.hpp"
#include "Texture.hpp"

class TextureOptimizer
{
public:
    TextureOptimizer(TriMesh * pMesh, Texture * pTexture) : m_pMesh(pMesh), m_pTexture(pTexture) { }
    bool Run();

private:
    TriMesh * m_pMesh;
    Texture * m_pTexture;
};

#endif //TEXTURE_OPTIMIZER_HPP
