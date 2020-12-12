#pragma warning(disable:4067 4244)
#include "MeshMeasure.hpp"

#include <tinycolormap.hpp>
#include <matplotlibcpp.h>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"

void MeshMeasure::AreaDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput)
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

        auto lba = va - vb;
        auto lbc = vc - vb;

        Real area = lba.cross(lbc).norm() * 0.5;
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

                auto lba = va - vb;
                auto lbc = vc - vb;

                Real area = lba.cross(lbc).norm() * 0.5;
                oneRingMeshArea += area / 3.0;
            }

            {
                const auto & va = TriMesh::Point(uvs[0][0], uvs[0][1], 0.0);
                const auto & vb = TriMesh::Point(uvs[1][0], uvs[1][1], 0.0);
                const auto & vc = TriMesh::Point(uvs[2][0], uvs[2][1], 0.0);

                auto lba = va - vb;
                auto lbc = vc - vb;

                Real area = lba.cross(lbc).norm() * 0.5;
                oneRingUVArea += area / 3.0;
            }
        }

        auto meshAreas = static_cast<float>(oneRingMeshArea / totalMeshArea);
        auto uvAreas = static_cast<float>(oneRingUVArea);
        areaDistortions[vh.idx()] = std::abs(std::log(uvAreas / meshAreas));
    }

    float maxDistortion = -std::numeric_limits<float>::max();
    float minDistortion =  std::numeric_limits<float>::min();
    for (float areaDistortion : areaDistortions)
    {
        maxDistortion = std::max(maxDistortion, areaDistortion);
        minDistortion = std::min(minDistortion, areaDistortion);
    }

    maxDistortion = 2.0f;

    std::vector<float> normalizedAreaDistortions(pMesh->n_vertices(), 0.0f);
    for (size_t i = 0; i < areaDistortions.size(); i++)
    {
        normalizedAreaDistortions[i] = (areaDistortions[i] - minDistortion) / (maxDistortion - minDistortion);
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

    matplotlibcpp::figure_size(480, 360);
    matplotlibcpp::hist(areaDistortions, 1000);
    matplotlibcpp::title("Area distortion");
    matplotlibcpp::save(figureOutput);
    matplotlibcpp::show();
    Logger::Info("Successfully write area distortion figure: \"%s\"!", figureOutput.c_str());
}

void MeshMeasure::AngleDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput)
{
    std::vector<float> angleDistortions(pMesh->n_vertices(), 0.0f);

    for (const auto vh : pMesh->vertices())
    {
        Real oneRingDeltaAngle = 0.0;
        Real angleNum = 0;
        for (const auto fh : pMesh->vf_range(vh))
        {
            TriMesh::Point points[2];
            TriMesh::TexCoord2D uvs[2];

            TriMesh::Point point;
            TriMesh::TexCoord2D uv;

            size_t index = 0;
            for (const auto & fhvh : pMesh->fv_range(fh))
            {
                if (fhvh.idx() != vh.idx())
                {
                    points[index++] = pMesh->point(fhvh);
                }
                else
                {
                    point = pMesh->point(fhvh);
                }
            }

            index = 0;
            for (const auto & heh : pMesh->fh_range(fh))
            {
                if (pMesh->to_vertex_handle(heh).idx() != vh.idx())
                {
                    uvs[index++] = pMesh->texcoord2D(heh);
                }
                else
                {
                    uv = pMesh->texcoord2D(heh);
                }
            }

            Real meshAngle = 0.0;
            {
                const auto & va = point;
                const auto & vb = points[0];
                const auto & vc = points[1];

                const auto lab = vb - va;
                const auto lac = vc - va;

                Real denominator = lab.norm() * lac.norm();
                if (denominator != 0.0)
                {
                    Real cosine = std::clamp(lab.dot(lac) / denominator, 0.0, 1.0);
                    meshAngle = acos(cosine);
                }
            }

            Real uvAngle = 0.0;
            {
                const auto & va = TriMesh::Point(uv[0], uv[1], 0.0);
                const auto & vb = TriMesh::Point(uvs[0][0], uvs[0][1], 0.0);
                const auto & vc = TriMesh::Point(uvs[1][0], uvs[1][1], 0.0);

                const auto lab = vb - va;
                const auto lac = vc - va;

                Real denominator = lab.norm() * lac.norm();
                if (denominator != 0.0)
                {
                    Real cosine = std::clamp(lab.dot(lac) / denominator, 0.0, 1.0);
                    uvAngle = acos(cosine);
                }
            }

            oneRingDeltaAngle += std::abs(meshAngle - uvAngle);
            angleNum += 1.0;
        }
        angleDistortions[vh.idx()] = static_cast<float>(oneRingDeltaAngle / angleNum);
    }

    float maxDistortion = -std::numeric_limits<float>::max();
    float minDistortion =  std::numeric_limits<float>::min();
    for (float angleDistortion : angleDistortions)
    {
        maxDistortion = std::max(maxDistortion, angleDistortion);
        minDistortion = std::min(minDistortion, angleDistortion);
    }

    maxDistortion = 1.0f;

    std::vector<float> normalizedAngleDistortions(pMesh->n_vertices(), 0.0f);
    for (size_t i = 0; i < angleDistortions.size(); i++)
    {
        normalizedAngleDistortions[i] = (angleDistortions[i] - minDistortion) / (maxDistortion - minDistortion);
    }

    /* visualization used for debug */
    Eigen::MatrixXd C;
    C.resize(pMesh->n_vertices(), 3);

    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        tinycolormap::Color color = tinycolormap::GetColor(normalizedAngleDistortions[i], tinycolormap::ColormapType::Heat);
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

    DisplayVFC(V, F, C, Logger::Format("Angle Distortion (Mesh) [%f, %f]", minDistortion, maxDistortion));

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

    DisplayVFC(V, F, C, Logger::Format("Angle Distortion (UV) [%f, %f]", minDistortion, maxDistortion));

    std::ofstream statsFile(statsOutput);
    for (float angleDistortion : angleDistortions)
    {
        statsFile << angleDistortion << std::endl;
    }
    statsFile.close();
    Logger::Info("Successfully write angle distortion file: \"%s\"!", statsOutput.c_str());

    matplotlibcpp::figure_size(480, 360);
    matplotlibcpp::hist(angleDistortions, 1000);
    matplotlibcpp::title("Angle distortion");
    matplotlibcpp::save(figureOutput);
    matplotlibcpp::show();
    Logger::Info("Successfully write angle distortion figure: \"%s\"!", figureOutput.c_str());
}
