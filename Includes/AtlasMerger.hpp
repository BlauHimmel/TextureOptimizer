#ifndef ATLAS_MERGER_HPP
#define ATLAS_MERGER_HPP

#include "Declaration.hpp"

class AtlasMerger
{
public:
    explicit AtlasMerger(TriMesh * pMesh) : m_pMesh(pMesh) { }

    static size_t MarkAtlasIndex(TriMesh * pMesh);

    static void DisplayAtlas(TriMesh * pMesh, size_t atlasNum);

private:
    TriMesh * m_pMesh;
};

#endif //ATLAS_MERGER_HPP
