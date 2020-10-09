#include "TextureOptimizer.hpp"

bool TextureOptimizer::Run()
{
    if (m_pMesh == nullptr || m_pTexture == nullptr || !m_pTexture->IsValid())
    {
        return false;
    }

    for (auto vh : m_pMesh->vertices())
    {
        if (!m_pMesh->is_manifold(vh))
        {
            std::cout << "Vertex [" << vh.idx() << "] is non-manifold!" << std::endl;
        }
    }

    return true;
}
