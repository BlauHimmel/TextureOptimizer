#include <string>
#include <iostream>

#include <lyra/lyra.hpp>

#include "TextureOptimizer.hpp"

int main(int argc, const char ** argv)
{
    bool bShowHelp = false;
    std::string meshFilename, textureFilename;

    auto cli = lyra::help(bShowHelp)
        | lyra::opt(meshFilename, "mesh")["-m"]["--mesh"]("File name of the input mesh").required()
        | lyra::opt(textureFilename, "texture")["-t"]["--texture"]("File name of the input texture").required();

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

    Texture texture;
    if (!texture.Load(textureFilename))
    {
        std::cerr << "Could not load texture: \"" << textureFilename << "\"!"  << std::endl;
        return 1;
    }

    TriMesh mesh;
    mesh.request_halfedge_texcoords2D();

    OpenMesh::IO::Options opts;
    opts += OpenMesh::IO::Options::FaceTexCoord;

    if (!OpenMesh::IO::read_mesh(mesh, meshFilename, opts))
    {
        std::cerr << "Could not load mesh: \"" << textureFilename << "\"!"  << std::endl;
        return 1;
    }

    TextureOptimizer textureOptimizer(&mesh, &texture);
    textureOptimizer.Run();

    return 0;
}