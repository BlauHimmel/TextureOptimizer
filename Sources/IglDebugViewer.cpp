#pragma warning(disable:4244 4018 4305 4819 4267 4005)
#include "IglDebugViewer.hpp"

#include <random>

#include <igl/opengl/glfw/Viewer.h>

#include "Logger.hpp"

void DisplayAtlas(TriMesh * pMesh, size_t atlasNum, const std::string & atlasIdxFPropName, const std::string & title)
{
    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    if (!pMesh->get_property_handle(atlasIdxFProp, atlasIdxFPropName))
    {
        Logger::Error("To display atlas, mesh must have face property \"atlasIdx\".");
        return;
    }

    std::default_random_engine engine(pMesh->n_vertices() + atlasNum);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::vector<std::vector<double>> randColors;
    randColors.reserve(atlasNum);
    for (size_t i = 0; i < atlasNum; i++)
    {
        std::vector<double> color = { uniform(engine), uniform(engine), uniform(engine) };
        /* avoid dark color */
        while ((color[0] + color[1] + color[3]) / 3.0 < 0.1)
        {
            color = { uniform(engine), uniform(engine), uniform(engine) };
        }
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

        size_t atlasIdx = pMesh->property(atlasIdxFProp, fh);

        if (atlasIdx != size_t(-1))
        {
            C(fh.idx(), 0) =  randColors[atlasIdx][0];
            C(fh.idx(), 1) =  randColors[atlasIdx][1];
            C(fh.idx(), 2) =  randColors[atlasIdx][2];
        }
        else
        {
            C(fh.idx(), 0) =  0.0;
            C(fh.idx(), 1) =  0.0;
            C(fh.idx(), 2) =  0.0;
        }
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch(true, false, title);
}

void DisplayVF(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const std::string & title)
{
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.launch(true, false, title);
}

void DisplayVFC(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::MatrixXd & C, const std::string & title)
{
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch(true, false, title);
}