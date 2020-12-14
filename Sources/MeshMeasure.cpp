#pragma warning(disable:4067 4244)
#include "MeshMeasure.hpp"

#include <tinycolormap.hpp>
#include <matplotlibcpp.h>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"

void MeshMeasure::AreaDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput)
{
    std::vector<float> areaDistortions(pMesh->n_vertices(), 0.0f);

    Real totalMeshArea = 0.0;
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
                    Real cosine = std::clamp(lab.dot(lac) / denominator, Real(-1.0), Real(1.0));
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
                    Real cosine = std::clamp(lab.dot(lac) / denominator, Real(-1.0), Real(1.0));
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

class Jacobian
{
public:
    using Singular = std::pair<Real, Real>;

public:
    static std::vector<Singular> ComputeSingularValues(const TriMesh * pMesh)
    {
        TriMesh mesh(*pMesh);

        Real totalMeshArea = 0.0;
        for (const auto & fh : mesh.faces())
        {
            TriMesh::Point points[3];

            size_t index = 0;
            for (const auto & vh : mesh.fv_range(fh))
            {
                points[index++] = mesh.point(vh);
            }

            const auto & va = points[0];
            const auto & vb = points[1];
            const auto & vc = points[2];

            auto lba = va - vb;
            auto lbc = vc - vb;

            Real area = lba.cross(lbc).norm() * 0.5;
            totalMeshArea += area;
        }

        Real scale = std::sqrt(1.0 / totalMeshArea);

        for (const auto & vh : mesh.vertices())
        {
            auto & point = mesh.point(vh);
            point *= scale;
            mesh.set_point(vh, point);
        }

        std::vector<Singular> singularValues(mesh.n_faces(), std::make_pair(Real(0.0), Real(0.0)));
        for (const auto & fh : mesh.faces())
        {
            singularValues[fh.idx()] = ComputeSingularValue(&mesh, fh);
        }

        return singularValues;
    }

private:
    struct Matrix2D
    {
        OpenMesh::VectorT<Real, 2> Point1;
        OpenMesh::VectorT<Real, 2> Point2;
    } ;

    static Matrix2D TruncateTo2D(const TriMesh::Point & p0, const TriMesh::Point & p1, const TriMesh::Point & p2)
    {
        const auto v1 = p1 - p0;
        const auto v2 = p2 - p0;

        Matrix2D mat;
        mat.Point1.data()[0] = v1[0];
        mat.Point1.data()[1] = v1[1];
        mat.Point2.data()[0] = v2[0];
        mat.Point2.data()[1] = v2[1];

        return mat;
    }

    static Matrix2D RotateTo2D(const TriMesh::Point & p0, const TriMesh::Point & p1, const TriMesh::Point & p2)
    {
        const auto l01 = (p0 - p1).norm();
        const auto l12 = (p1 - p2).norm();
        const auto l20 = (p2 - p0).norm();

        auto cos = (l01 * l01 + l20 * l20 - l12 * l12) / (2.0 * l01 * l20);
        cos = std::clamp(cos, Real(-1.0), Real(1.0));
        auto angle = std::acos(cos);

        Matrix2D mat;
        mat.Point1[0] = l01;
        mat.Point1[1] = 0;
        mat.Point2[0] = l20 * std::cos(angle);
        mat.Point2[1] = l20 * std::sin(angle);

        return mat;
    }

    static Singular ComputeSingularValue(TriMesh * pMesh, TriMesh::FaceHandle fh)
    {
        TriMesh::Point pts[3];
        TriMesh::Point uvs[3];

        size_t index = 0;
        for (const auto & fhvh : pMesh->fv_range(fh))
        {
            pts[index++] = pMesh->point(fhvh);
        }

        index = 0;
        for (const auto & heh : pMesh->fh_range(fh))
        {
            const auto & uv = pMesh->texcoord2D(heh);
            uvs[index++] = TriMesh::Point(uv[0], uv[1], 0.0);
        }

        const auto meshTriangle = RotateTo2D(pts[0], pts[1], pts[2]);
        const auto uvTriangle = TruncateTo2D(uvs[0], uvs[1], uvs[2]);

        const auto a = uvTriangle.Point1[0] / meshTriangle.Point1[0];
        const auto c = uvTriangle.Point1[1] / meshTriangle.Point1[0];
        const auto b = (uvTriangle.Point2[0] - meshTriangle.Point2[0] * a) / meshTriangle.Point2[1];
        const auto d = (uvTriangle.Point2[1] - meshTriangle.Point2[0] * c) / meshTriangle.Point2[1];

        const auto sqrt1 = std::sqrt(std::pow(b + c, 2) + std::pow(a - d, 2));
        const auto sqrt2 = std::sqrt(std::pow(b - c, 2) + std::pow(a + d, 2));
        const auto sigma1 = 0.5 * (sqrt1 - sqrt2);
        const auto sigma2 = 0.5 * (sqrt1 + sqrt2);

        if (isnan(sigma1) || isnan(sigma2))
        {
            return std::make_pair(Real(0.0), Real(0.0));
        }
        else
        {
            return std::make_pair(sigma1, sigma2);
        }
    }
};

void MeshMeasure::SymmetricDirichletDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput)
{
    std::vector<float> symmetricDirichletDistortions(pMesh->n_faces(), 0.0f);
    std::vector<Jacobian::Singular> singularValues = Jacobian::ComputeSingularValues(pMesh);

    for (size_t i = 0; i < pMesh->n_faces(); i++)
    {
        if (singularValues[i].first == 0.0 || singularValues[i].second == 0.0)
        {
            symmetricDirichletDistortions[i] = 0.0;
        }
        else
        {
            symmetricDirichletDistortions[i] = std::abs(std::log(0.25 * (std::pow(singularValues[i].first, 2) + std::pow(singularValues[i].second, 2) +
                std::pow(singularValues[i].first, -2) + std::pow(singularValues[i].second, -2))));
        }
    }

    float maxDistortion = -std::numeric_limits<float>::max();
    float minDistortion =  std::numeric_limits<float>::min();
    for (float symmetricDirichletDistortion : symmetricDirichletDistortions)
    {
        maxDistortion = std::max(maxDistortion, symmetricDirichletDistortion);
        minDistortion = std::min(minDistortion, symmetricDirichletDistortion);
    }

    maxDistortion = 4.0f;

    std::vector<float> normalizedSymmetricDirichletDistortions(pMesh->n_faces(), 0.0f);
    for (size_t i = 0; i < normalizedSymmetricDirichletDistortions.size(); i++)
    {
        normalizedSymmetricDirichletDistortions[i] = (symmetricDirichletDistortions[i] - minDistortion) / (maxDistortion - minDistortion);
    }

    /* visualization used for debug */
    Eigen::MatrixXd C;
    C.resize(pMesh->n_faces(), 3);

    for (int i = 0; i < pMesh->n_faces(); i++)
    {
        tinycolormap::Color color = tinycolormap::GetColor(normalizedSymmetricDirichletDistortions[i], tinycolormap::ColormapType::Heat);
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

    DisplayVFC(V, F, C, Logger::Format("Symmetric Dirichlet Distortion (Mesh) [%f, %f]", minDistortion, maxDistortion));

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

    DisplayVFC(V, F, C, Logger::Format("Symmetric Dirichlet Distortion (UV) [%f, %f]", minDistortion, maxDistortion));

    std::ofstream statsFile(statsOutput);
    for (float symmetricDirichletDistortion : symmetricDirichletDistortions)
    {
        statsFile << symmetricDirichletDistortion << std::endl;
    }
    statsFile.close();
    Logger::Info("Successfully write symmetric Dirichlet distortion file: \"%s\"!", statsOutput.c_str());

    matplotlibcpp::figure_size(480, 360);
    matplotlibcpp::hist(symmetricDirichletDistortions, 1000);
    matplotlibcpp::title("Symmetric Dirichlet distortion");
    matplotlibcpp::save(figureOutput);
    matplotlibcpp::show();
    Logger::Info("Successfully write symmetric Dirichlet distortion figure: \"%s\"!", figureOutput.c_str());
}