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
    float step;

    auto cli = lyra::help(bShowHelp)
        | lyra::opt(meshFilename, "mesh")["-m"]["--mesh"]("File name of the input mesh").required()
        | lyra::opt(outputFilename, "output")["-o"]["--output"]("Directory of the output files.").required()
        | lyra::opt(step, "step")["-s"]["--step"]("Optimization step.").required();

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

    /* check input args */
    if (outputFilename.back() != '\\')
    {
        outputFilename += "\\";
    }

    std::filesystem::path meshPath(meshFilename);

    std::string meshOutput = outputFilename + meshPath.stem().string() + ".omt" + meshPath.extension().string();

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

    if (!omt.Initialize())
    {
        Logger::Error("Failed to initialize OMT2D.");
        return 1;
    }

    omt.Run(step);

    /* ************************ run OMT ************************ */

    return 0;
}