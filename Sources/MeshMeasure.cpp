#pragma warning(disable:4067)
#include "MeshMeasure.hpp"

#include <tinycolormap.hpp>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"

void MeshMeasure::AreaDistortion(const TriMesh * pMesh, const std::string & statsOutput)
{
    std::vector<float> areaDistortions(pMesh->n_vertices(), 0.0f);

    Real totalMeshArea = 0.0f;
    for (const auto & fh : pMesh->faces())
    {
        TriMesh::Point points[3];

        size_t index = 0;
        for (const auto & vh : pMesh->fv_range(fh))
        {
            points[index++] = pMesh->point(vh);
        }

        const auto & va = points[0];
        const auto & vb = points[1];
        const auto & vc = points[2];

        auto lab = va - vb;
        auto lac = vc - vb;

        Real area = lab.cross(lac).norm() * 0.5;
        totalMeshArea += area;
    }

    for (const auto & vh : pMesh->vertices())
    {
        Real oneRingMeshArea = 0.0;
        Real oneRingUVArea = 0.0;
        for (const auto & fh : pMesh->vf_range(vh))
        {
            TriMesh::Point points[3];
            TriMesh::TexCoord2D uvs[3];

            size_t index = 0;
            for (const auto & fhvh : pMesh->fv_range(fh))
            {
                points[index++] = pMesh->point(fhvh);
            }

            index = 0;
            for (const auto & heh : pMesh->fh_range(fh))
            {
                uvs[index++] = pMesh->texcoord2D(heh);
            }

            {
                const auto & va = points[0];
                const auto & vb = points[1];
                const auto & vc = points[2];

                auto lab = va - vb;
                auto lac = vc - vb;

                Real area = lab.cross(lac).norm() * 0.5;
                oneRingMeshArea += area / 3.0;
            }

            {
                const auto & va = TriMesh::Point(uvs[0][0], uvs[0][1], 0.0);
                const auto & vb = TriMesh::Point(uvs[1][0], uvs[1][1], 0.0);
                const auto & vc = TriMesh::Point(uvs[2][0], uvs[2][1], 0.0);

                auto lab = va - vb;
                auto lac = vc - vb;

                Real area = lab.cross(lac).norm() * 0.5;
                oneRingUVArea += area / 3.0;
            }
        }

        auto meshAreas = static_cast<float>(oneRingMeshArea / totalMeshArea);
        auto uvAreas = static_cast<float>(oneRingUVArea);
        areaDistortions[vh.idx()] = std::log(uvAreas / meshAreas);
    }

    float maxDistortion = -std::numeric_limits<float>::max();
    float minDistortion =  std::numeric_limits<float>::min();
    for (float areaDistortion : areaDistortions)
    {
        maxDistortion = std::max(maxDistortion, std::abs(areaDistortion));
        minDistortion = std::min(minDistortion, std::abs(areaDistortion));
    }

    maxDistortion = 2.0f;

    std::vector<float> normalizedAreaDistortions(pMesh->n_vertices(), 0.0f);
    for (size_t i = 0; i < areaDistortions.size(); i++)
    {
        normalizedAreaDistortions[i] = (std::abs(areaDistortions[i]) - minDistortion) / (maxDistortion - minDistortion);
    }

    /* visualization used for debug */
    Eigen::MatrixXd C;
    C.resize(pMesh->n_vertices(), 3);

    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        tinycolormap::Color color = tinycolormap::GetColor(normalizedAreaDistortions[i], tinycolormap::ColormapType::Heat);
        C(i, 0) = color.r();
        C(i, 1) = color.g();
        C(i, 2) = color.b();
    }

    Eigen::MatrixXi F;
    F.resize(pMesh->n_faces(), 3);

    for (int i = 0; i < pMesh->n_faces(); i++)
    {
        const auto fh = TriMesh::FaceHandle(i);

        auto fvIter = pMesh->fv_range(fh).begin();
        auto fvIterEnd = pMesh->fv_range(fh).end();

        const auto vh0 = *fvIter;
        *fvIter++;
        const auto vh1 = *fvIter;
        *fvIter++;
        const auto vh2 = *fvIter;
        *fvIter++;

        F(i, 0) = vh0.idx();
        F(i, 1) = vh1.idx();
        F(i, 2) = vh2.idx();
    }

    Eigen::MatrixXd V;
    V.resize(pMesh->n_vertices(), 3);

    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        V(i, 0) = point[0];
        V(i, 1) = point[1];
        V(i, 2) = point[2];
    }

    DisplayVFC(V, F, C, Logger::Format("Area Distortion (Mesh) [%f, %f]", minDistortion, maxDistortion));

    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        for (const auto heh : pMesh->vih_range(vh))
        {
            if (!pMesh->is_boundary(heh))
            {
                const auto & uv = pMesh->texcoord2D(heh);
                V(i, 0) = uv[0];
                V(i, 1) = uv[1];
                V(i, 2) = 0.0;
                break;
            }
        }
    }

    DisplayVFC(V, F, C, Logger::Format("Area Distortion (UV) [%f, %f]", minDistortion, maxDistortion));

    std::ofstream statsFile(statsOutput);
    for (float areaDistortion : areaDistortions)
    {
        statsFile << areaDistortion << std::endl;
    }
    statsFile.close();
    Logger::Info("Successfully write area distortion file: \"%s\"!", statsOutput.c_str());
}