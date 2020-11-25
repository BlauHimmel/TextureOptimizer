#include "EBPWrapper.h"

#include "BiljectivePara.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

std::vector<std::vector<double>> EBP(std::vector<std::vector<double>> & vertices, std::vector<std::vector<size_t>> & facets)
{
    Mesh mesh;

    std::vector<OpenMesh::VertexHandle> vhs;
    vhs.reserve(vertices.size());

    for (auto & vertex : vertices)
    {
        const auto vh = mesh.add_vertex({ vertex[0], vertex[1], vertex[2] });
        vhs.push_back(vh);
    }

    for (auto & facet : facets)
    {
        std::vector<OpenMesh::VertexHandle> faceVHs;
        faceVHs.push_back(vhs[facet[0]]);
        faceVHs.push_back(vhs[facet[1]]);
        faceVHs.push_back(vhs[facet[2]]);
        mesh.add_face(faceVHs);
    }

    auto * bil_para = new BiljectivePara(mesh);

    bil_para->load();
    auto uv = bil_para->parameterization();

    delete bil_para;
    bil_para = nullptr;

    return uv;
}