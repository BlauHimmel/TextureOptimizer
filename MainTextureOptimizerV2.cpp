#include <string>
#include <iostream>
#include <filesystem>

#include <lyra/lyra.hpp>

#include "TextureOptimizer.hpp"
#include "Logger.hpp"
#include "MeshUtils.hpp"
#include "AtlasMerger.hpp"

int main(int argc, const char ** argv)
{
    /* ************************ parse and check args ************************ */

    bool bShowHelp = false;
    std::string meshFilename, textureFilename;
    std::string outputFilename;
    uint32_t outputTextureWidth(-1);
    uint32_t outputTextureHeight(-1);
    uint32_t parameterizationMethod = 0;

    auto cli = lyra::help(bShowHelp)
        | lyra::opt(meshFilename, "mesh")["-m"]["--mesh"]("File name of the input mesh").required()
        | lyra::opt(textureFilename, "texture")["-t"]["--texture"]("File name of the input texture").required()
        | lyra::opt(outputFilename, "output")["-o"]["--output"]("Directory of the output files.").required()
        | lyra::opt(parameterizationMethod, "parameterization")["-p"]["--param"]("Method used for parameterization: 0: Harmonic, 1: SLIM, 2: EBP.").required().choices(0, 1, 2)
        | lyra::opt(outputTextureWidth, "width")["-x"]["--width"]("Width of the output texture.").optional()
        | lyra::opt(outputTextureHeight, "height")["-y"]["--height"]("Height of the output texture.").optional();

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
    std::filesystem::path texturePath(textureFilename);

    std::string meshOutput = outputFilename + meshPath.stem().string() + ".opt" + meshPath.extension().string();
    std::string textureOutput = outputFilename + texturePath.stem().string() + ".opt" + texturePath.extension().string();

    /* ************************ parse and check args ************************ */

    /* ************************ load texture ************************ */

    /* load texture from file */
    Texture texture;

    Logger::Info("Loading texture...");
    if (!texture.Load(textureFilename))
    {
        Logger::Error("Could not load texture: \"%s\"!", textureFilename.c_str());
        return 1;
    }
    Logger::Info("Successfully load texture: \"%s\"!", textureFilename.c_str());

    /* ************************ load texture ************************ */

    /* ************************ load mesh ************************ */

    /* load mesh from file */
    TriMesh mesh;
    mesh.request_halfedge_texcoords2D();

    OpenMesh::IO::Options rOpts;
    rOpts += OpenMesh::IO::Options::FaceTexCoord;

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

    /* ************************ run optimizer ************************ */

    Logger::Info("Optimizing the texture...");
    AtlasMerger atlasMerger(&mesh, &texture);
    if (!atlasMerger.Run(outputTextureWidth, outputTextureHeight, EParameterization(parameterizationMethod), meshOutput, textureOutput))
    {
        Logger::Error("Could not optimize the texture!");
        return 1;
    }
    Logger::Info("Successfully optimize the texture!");

    /* ************************ run optimizer ************************ */

    return 0;
}