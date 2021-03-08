#pragma warning(disable:4244 4018 4305 4819 4267 4005)
#include "IglDebugViewer.hpp"

#include <random>

#include <igl/opengl/glfw/Viewer.h>
#include <stb_image_write.h>

#include "Logger.hpp"

static bool WritePNG(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> & R,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> & G,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> & B,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> & A,
    const std::string & png_file
)
{
    assert((R.rows() == G.rows()) && (G.rows() == B.rows()) && (B.rows() == A.rows()));
    assert((R.cols() == G.cols()) && (G.cols() == B.cols()) && (B.cols() == A.cols()));

    const int comp = 4;                                  // 4 Channels Red, Green, Blue, Alpha
    const int stride_in_bytes = R.rows() * comp;           // Length of one row in bytes
    std::vector<unsigned char> data(R.size()*comp,0);     // The image itself;

    for (unsigned i = 0; i<R.rows();++i)
    {
        for (unsigned j = 0; j < R.cols(); ++j)
        {
            data[(j * R.rows() * comp) + (i * comp) + 0] = R(i,R.cols()-1-j);
            data[(j * R.rows() * comp) + (i * comp) + 1] = G(i,R.cols()-1-j);
            data[(j * R.rows() * comp) + (i * comp) + 2] = B(i,R.cols()-1-j);
            data[(j * R.rows() * comp) + (i * comp) + 3] = A(i,R.cols()-1-j);
        }
    }

    stbi_write_png(png_file.c_str(), R.rows(), R.cols(), comp, data.data(), stride_in_bytes);

    return true;
}

void DisplayAtlas(TriMesh * pMesh, size_t atlasNum, const std::string & atlasIdxFPropName, const std::string & title, const std::string & screenshotTag, const std::string & screenshotDir)
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
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) -> bool
    {
        if (key == 'U')
        {
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(800,800);
            char buffer[128];
            time_t Time;
            time(&Time);
            strftime(buffer, sizeof(buffer), "-%Y_%m_%d-%H_%M_%S.png", localtime(&Time));
            std::string filename = screenshotDir + screenshotTag + std::string(buffer);
            viewer.core().draw_buffer(viewer.data(),false,R,G,B,A);
            WritePNG(R, G, B, A,filename);
            Logger::Info("Write screen shot: \"%s\"!", filename.c_str());
        }
        return false;
    };
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.core().background_color = { 1.0f, 1.0f, 1.0f, 0.0f };
    viewer.launch(false, false, title, 1000, 1000);
}

void DisplayVF(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const std::string & title, const std::string & screenshotTag, const std::string & screenshotDir)
{
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) -> bool
    {
        if (key == 'U')
        {
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(800,800);
            char buffer[128];
            time_t Time;
            time(&Time);
            strftime(buffer, sizeof(buffer), "-%Y_%m_%d-%H_%M_%S.png", localtime(&Time));
            std::string filename = screenshotDir + screenshotTag + std::string(buffer);
            viewer.core().draw_buffer(viewer.data(),false,R,G,B,A);
            WritePNG(R, G, B, A,filename);
            Logger::Info("Write screen shot: \"%s\"!", filename.c_str());
        }
        return false;
    };
    viewer.data().set_mesh(V, F);
    viewer.core().background_color = { 1.0f, 1.0f, 1.0f, 0.0f };
    viewer.launch(false, false, title, 1000, 1000);
}

void DisplayVFC(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::MatrixXd & C, const std::string & title, const std::string & screenshotTag, const std::string & screenshotDir)
{
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) -> bool
    {
        if (key == 'U')
        {
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(800,800);
            char buffer[128];
            time_t Time;
            time(&Time);
            strftime(buffer, sizeof(buffer), "-%Y_%m_%d-%H_%M_%S.png", localtime(&Time));
            std::string filename = screenshotDir + screenshotTag + std::string(buffer);
            viewer.core().draw_buffer(viewer.data(),false,R,G,B,A);
            WritePNG(R, G, B, A,filename);
            Logger::Info("Write screen shot: \"%s\"!", filename.c_str());
        }
        return false;
    };
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.core().background_color = { 1.0f, 1.0f, 1.0f, 0.0f };
    viewer.launch(false, false, title, 1000, 1000);
}

void DisplayVFD(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::VectorXd & D, const std::string & title, const std::string & screenshotTag, const std::string & screenshotDir)
{
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers) -> bool
    {
        if (key == 'U')
        {
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(800,800);
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(800,800);
            char buffer[128];
            time_t Time;
            time(&Time);
            strftime(buffer, sizeof(buffer), "-%Y_%m_%d-%H_%M_%S.png", localtime(&Time));
            std::string filename = screenshotDir + screenshotTag + std::string(buffer);
            viewer.core().draw_buffer(viewer.data(),false,R,G,B,A);
            WritePNG(R, G, B, A,filename);
            Logger::Info("Write screen shot: \"%s\"!", filename.c_str());
        }
        return false;
    };
    viewer.data().set_mesh(V, F);
    viewer.data().set_data(D, igl::COLOR_MAP_TYPE_JET);
    viewer.core().background_color = { 1.0f, 1.0f, 1.0f, 0.0f };
    viewer.launch(false, false, title, 1000, 1000);
}