#include <string>
#include <iostream>
#include <random>
#include <filesystem>

#include <lyra/lyra.hpp>
#include <simple_svg.hpp>

#include "Logger.hpp"
#include "MeshUtils.hpp"
#include "TextureOptimizerV2.hpp"

int main(int argc, const char ** argv)
{
    /* ************************ parse and check args ************************ */

    bool bShowHelp = false;
    std::string meshFilename;

    auto cli = lyra::help(bShowHelp)
               | lyra::opt(meshFilename, "mesh")["-m"]["--mesh"]("File name of the input mesh").required();

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

    std::filesystem::path meshPath(meshFilename);

    std::string svgOutput = std::filesystem::path(meshPath).replace_extension(".atlas.svg").string();
    std::string infoOutput = std::filesystem::path(meshPath).replace_extension(".info.txt").string();

    /* ************************ parse and check args ************************ */

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

    double PE = ComputePackingEfficiency(&mesh);
    int32_t euler = GetMeshEuler(&mesh);
    int32_t genus = GetMeshGenus(&mesh);

    Logger::Info("Mesh info: %zd vertices, %zd faces, and %zd edges, euler %d, genus %d, PE %lf.",
                 mesh.n_vertices(), mesh.n_faces(), mesh.n_edges(), euler, genus, PE);

    std::ofstream infoFile(infoOutput);
    infoFile << "V: " << mesh.n_vertices() << std::endl;
    infoFile << "F: " << mesh.n_faces() << std::endl;
    infoFile << "E: " << mesh.n_edges() << std::endl;
    infoFile << "euler: " << euler << std::endl;
    infoFile << "genus: " << genus << std::endl;
    infoFile << "PE: " << PE << std::endl;
    infoFile.close();

    /* ************************ load mesh ************************ */

    /* ************************ run ************************ */

    auto atlasInfo = TextureOptimizerV2::MarkAtlasIndex(&mesh);

    OpenMesh::FPropHandleT<size_t> atlasIdxFProp;
    mesh.get_property_handle(atlasIdxFProp, "atlasIdx");

    svg::Dimensions dimensions(1000, 1000);
    svg::Document doc(svgOutput, svg::Layout(dimensions, svg::Layout::BottomLeft));

    svg::Polygon border(svg::Stroke(3, svg::Color::Black));
    border << svg::Point(0, 0) << svg::Point(dimensions.width, 0) << svg::Point(dimensions.width, dimensions.height) << svg::Point(0, dimensions.height);
    doc << border;

    std::default_random_engine engine(unsigned int(mesh.n_vertices()));
    std::uniform_int_distribution<int> uniform(0, 255);

    for (const auto & kv : atlasInfo->atlasVertices)
    {
        TriMesh submesh(mesh);

        submesh.request_face_status();
        submesh.request_edge_status();
        submesh.request_vertex_status();

        for (const auto fh : submesh.faces())
        {
            if (mesh.property(atlasIdxFProp, fh) != kv.first)
            {
                submesh.delete_face(fh);
            }
        }

        submesh.garbage_collection();
        submesh.release_face_status();
        submesh.release_edge_status();
        submesh.release_vertex_status();

        svg::Polygon atlasPolygon = svg::Polygon(svg::Color(uniform(engine), uniform(engine), uniform(engine)), svg::Stroke(2, svg::Color::Black));
        std::vector<std::vector<OpenMesh::VertexHandle>> boundaryVertices = MeshBoundaryVertices(&submesh);

        for (const auto & vh : boundaryVertices[0])
        {
            for (const auto heh : submesh.vih_range(vh))
            {
                if (!submesh.is_boundary(heh))
                {
                    const auto & uv = submesh.texcoord2D(heh);
                    atlasPolygon << svg::Point(uv[0] * 1000, uv[1] * 1000);
                    break;
                }
            }
        }
        doc << atlasPolygon;
    }

    if (!doc.save())
    {
        Logger::Error("Could not write atlas svg: \"%s\"!", svgOutput.c_str());
        return 1;
    }
    Logger::Info("Successfully write atlas svg: \"%s\"!", svgOutput.c_str());

    /* ************************ run ************************ */

    return 0;
}