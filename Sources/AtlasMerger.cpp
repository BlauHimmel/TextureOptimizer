#pragma warning(disable:4244 4018 4305 4819 4267 4005)
#include "AtlasMerger.hpp"

#include <unordered_set>
#include <queue>
#include <random>

#include <igl/opengl/glfw/Viewer.h>

#include "Logger.hpp"

struct TexCoord2DHasher
{
    size_t operator()(const TriMesh::TexCoord2D & uv) const
    {
        return std::hash<Real>()(uv[0]) ^ std::hash<Real>()(uv[1]);
    }
};

size_t AtlasMerger::MarkAtlasIndex(TriMesh * pMesh)
{
    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    pMesh->add_property(atlasIdxFProp, "atlasIdx");

    for (auto fh : pMesh->faces())
    {
        pMesh->property(atlasIdxFProp, fh) = size_t(-1);
    }

    size_t atlasIdx = 0;

    while (true)
    {
        int seedFaceIdx = -1;
        for (auto fh : pMesh->faces())
        {
            if (pMesh->property(atlasIdxFProp, fh) == size_t(-1))
            {
                seedFaceIdx = fh.idx();
                break;
            }
        }

        if (seedFaceIdx == -1)
        {
            break;
        }

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

            if (pMesh->property(atlasIdxFProp, fh) != size_t(-1))
            {
                continue;
            }

            int existUVCount = 0;
            for (auto heh : pMesh->fh_range(fh))
            {
                const auto & uv = pMesh->texcoord2D(heh);
                if (atlasUVSet.count(uv) != 0)
                {
                    existUVCount++;
                }
            }

            if (existUVCount <= 1)
            {
                continue;
            }

            pMesh->property(atlasIdxFProp, fh) = atlasIdx;
            for (auto heh : pMesh->fh_range(fh))
            {
                const auto & uv = pMesh->texcoord2D(heh);
                atlasUVSet.insert(uv);
            }

            for (auto adjFH : pMesh->ff_range(fh))
            {
                if (pMesh->property(atlasIdxFProp, adjFH) == size_t(-1))
                {
                    faceQueue.push(adjFH);
                }
            }
        }

        atlasIdx++;
    }

    DisplayAtlas(pMesh, atlasIdx);

    return atlasIdx;
}

void AtlasMerger::DisplayAtlas(TriMesh * pMesh, size_t atlasNum)
{
    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    if (!pMesh->get_property_handle(atlasIdxFProp, "atlasIdx"))
    {
        Logger::Error("To display atlas, mesh must have face property \"atlasIdx\".");
        return;
    }

    std::default_random_engine engine(pMesh->n_vertices());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::vector<std::vector<double>> randColors;
    for (size_t i = 0; i < atlasNum; i++)
    {
        randColors.push_back({ uniform(engine), uniform(engine), uniform(engine) });
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXd C;
    Eigen::MatrixXi F;

    V.resize(pMesh->n_vertices(), 3);
    for (auto vh : pMesh->vertices())
    {
        V(vh.idx(), 0) = pMesh->point(vh)[0];
        V(vh.idx(), 1) = pMesh->point(vh)[1];
        V(vh.idx(), 2) = pMesh->point(vh)[2];
    }

    F.resize(pMesh->n_faces(), 3);
    C.resize(pMesh->n_faces(), 3);
    for (auto fh : pMesh->faces())
    {
        auto fvIter = pMesh->fv_range(fh).begin();
        const auto vh0 = *fvIter;
        *fvIter++;
        const auto vh1 = *fvIter;
        *fvIter++;
        const auto vh2 = *fvIter;
        *fvIter++;

        F(fh.idx(), 0) = vh0.idx();
        F(fh.idx(), 1) = vh1.idx();
        F(fh.idx(), 2) = vh2.idx();

        C(fh.idx(), 0) =  randColors[pMesh->property(atlasIdxFProp, fh)][0];
        C(fh.idx(), 1) =  randColors[pMesh->property(atlasIdxFProp, fh)][1];
        C(fh.idx(), 2) =  randColors[pMesh->property(atlasIdxFProp, fh)][2];
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

}
