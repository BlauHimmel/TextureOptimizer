#include <string>
#include <iostream>
#include <filesystem>

#include <lyra/lyra.hpp>

#include "OMT.hpp"
#include "Logger.hpp"
#include "MeshUtils.hpp"

int main(int argc, const char ** argv)
{
    /* ************************ parse and check args ************************ */

    bool bShowHelp = false;
    std::string meshFilename, outputFilename;
    double omtThreshold = 6e-6;
    int maxIterationsHLBFGS = 400;

    auto cli = lyra::help(bShowHelp)
        | lyra::opt(meshFilename, "mesh")["-m"]["--mesh"]("File name of the input mesh").required()
        | lyra::opt(outputFilename, "output")["-o"]["--output"]("Directory of the output files.").required()
        | lyra::opt(omtThreshold, "stop")["-s"]["--stop"]("OMT stop threshold, usually be in range [4e-6~8e-6].").optional()
        | lyra::opt(maxIterationsHLBFGS, "iterations")["-i"]["--iter"]("Max iterations of HLBFGS.").optional();

    auto result = cli.parse( {argc, argv} );
    if (!result)
    {
        std::cerr << result.errorMessage() << std::endl;
        std::cerr << cli << std::endl;
        return 1;
    }

    if (bShowHelp)
    {
        std::cout << cli << std::endl;
        return 0;
    }

    if (omtThreshold <= 0.0)
    {
        Logger::Warn("Paramter '-s = %lf' is smaller than 0.0, reset to default value 0.2", omtThreshold);
        omtThreshold = 6e-6;
    }

    /* check input args */
    if (outputFilename.back() != '\\')
    {
        outputFilename += "\\";
    }

    std::filesystem::path meshPath(meshFilename);

    std::string meshOutput = outputFilename + meshPath.stem().string() + ".omt" + meshPath.extension().string();

    std::string omtErrorFigureOutput = std::filesystem::path(meshOutput).replace_extension(".error.svg").string();
    std::string hlbfgsErrorOutput = std::filesystem::path(meshOutput).replace_extension(".hlbfgs.error.txt").string();
    std::string gradientDescentErrorOutput = std::filesystem::path(meshOutput).replace_extension(".gd.error.txt").string();

    std::string omtMissFigureOutput = std::filesystem::path(meshOutput).replace_extension(".miss.svg").string();
    std::string hlbfgsMissOutput = std::filesystem::path(meshOutput).replace_extension(".hlbfgs.miss.txt").string();
    std::string gradientDescentMissOutput = std::filesystem::path(meshOutput).replace_extension(".gd.miss.txt").string();

    /* ************************ parse and check args ************************ */

    /* ************************ load mesh ************************ */

    /* load mesh from file */
    TriMesh mesh;
    mesh.request_vertex_texcoords2D();

    OpenMesh::IO::Options rOpts;
    rOpts += OpenMesh::IO::Options::VertexTexCoord;

    Logger::Info("Loading mesh...");
    if (!OpenMesh::IO::read_mesh(mesh, meshFilename, rOpts))
    {
        Logger::Error("Could not load mesh: \"%s\"!", meshFilename.c_str());
        return 1;
    }
    Logger::Info("Successfully load mesh: \"%s\"!", meshFilename.c_str());

    Logger::Info("Mesh info: %zd vertices, %zd faces, and %zd edges, euler %d, genus %d.",
                 mesh.n_vertices(), mesh.n_faces(), mesh.n_edges(), GetMeshEuler(&mesh), GetMeshGenus(&mesh));

    /* ************************ load mesh ************************ */

    /* ************************ run OMT ************************ */

    OMT2D omt(&mesh);

    if (!omt.Initialize(maxIterationsHLBFGS))
    {
        Logger::Error("Failed to initialize OMT2D.");
        return 1;
    }

    omt.Run(omtThreshold);

    /* ************************ run OMT ************************ */

    /* ************************ show and write stats ************ */

    omt.ShowAndSaveErrorFigure(omtErrorFigureOutput);
    omt.WriteHLBFGSErrorInfo(hlbfgsErrorOutput);
    omt.WriteGradientDescentErrorInfo(gradientDescentErrorOutput);

    omt.ShowAndSaveMissingPointFigure(omtMissFigureOutput);
    omt.WriteHLBFGSMissingPointInfo(hlbfgsMissOutput);
    omt.WriteGradientDescentMissingPointInfo(gradientDescentMissOutput);

    /* ************************ show and write stats ************** */

    /* ************************ write mesh ************************ */

    OpenMesh::IO::Options wOpts;
    wOpts += OpenMesh::IO::Options::VertexTexCoord;

    Logger::Info("Writing mesh...");
    if (!OpenMesh::IO::write_mesh(mesh, meshOutput, wOpts))
    {
        Logger::Error("Could not write mesh: \"%s\"!", meshOutput.c_str());
        return 1;
    }
    Logger::Info("Successfully write mesh: \"%s\"!", meshOutput.c_str());

    /* ************************ write mesh ************************ */

    return 0;
}