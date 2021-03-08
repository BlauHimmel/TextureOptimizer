#pragma warning(disable:4067 4244)
#include "MeshMeasure.hpp"

#include <tinycolormap.hpp>
#include <matplotlibcpp.h>

#include "IglDebugViewer.hpp"
#include "Logger.hpp"
#include "MeshUtils.hpp"

void MeshMeasure::AreaDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput, const std::string & screenshotDir)
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

    DisplayVFC(V, F, C, Logger::Format("Area Distortion (Mesh) [%f, %f]", minDistortion, maxDistortion), "AreaDistortion", screenshotDir);

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

    DisplayVFC(V, F, C, Logger::Format("Area Distortion (UV) [%f, %f]", minDistortion, maxDistortion), "AreaDistortionUV", screenshotDir);

    std::ofstream statsFile(statsOutput);
    for (float areaDistortion : areaDistortions)
    {
        statsFile << areaDistortion << std::endl;
    }
    statsFile.close();
    Logger::Info("Successfully write area distortion file: \"%s\"!", statsOutput.c_str());

    matplotlibcpp::figure_size(480, 360);
    matplotlibcpp::hist(areaDistortions, 1000);
    matplotlibcpp::save(figureOutput);
    matplotlibcpp::show();

    Logger::Info("Successfully write area distortion figure: \"%s\"!", figureOutput.c_str());
}

void MeshMeasure::AngleDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput, const std::string & screenshotDir)
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

    maxDistortion = 2.0f;

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

    DisplayVFC(V, F, C, Logger::Format("Angle Distortion (Mesh) [%f, %f]", minDistortion, maxDistortion), "AngleDistortion", screenshotDir);

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

    DisplayVFC(V, F, C, Logger::Format("Angle Distortion (UV) [%f, %f]", minDistortion, maxDistortion), "AngleDistortionUV", screenshotDir);

    std::ofstream statsFile(statsOutput);
    for (float angleDistortion : angleDistortions)
    {
        statsFile << angleDistortion << std::endl;
    }
    statsFile.close();
    Logger::Info("Successfully write angle distortion file: \"%s\"!", statsOutput.c_str());

    matplotlibcpp::figure_size(480, 360);
    matplotlibcpp::hist(angleDistortions, 1000);
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

void MeshMeasure::SymmetricDirichletDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput, const std::string & screenshotDir)
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

    maxDistortion = 2.0f;

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

    DisplayVFC(V, F, C, Logger::Format("Symmetric Dirichlet Distortion (Mesh) [%f, %f]", minDistortion, maxDistortion), "SymDistortion", screenshotDir);

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

    DisplayVFC(V, F, C, Logger::Format("Symmetric Dirichlet Distortion (UV) [%f, %f]", minDistortion, maxDistortion), "SymDistortionUV", screenshotDir);

    std::ofstream statsFile(statsOutput);
    for (float symmetricDirichletDistortion : symmetricDirichletDistortions)
    {
        statsFile << symmetricDirichletDistortion << std::endl;
    }
    statsFile.close();
    Logger::Info("Successfully write symmetric Dirichlet distortion file: \"%s\"!", statsOutput.c_str());

    matplotlibcpp::figure_size(480, 360);
    matplotlibcpp::hist(symmetricDirichletDistortions, 1000);
    matplotlibcpp::save(figureOutput);
    matplotlibcpp::show();

    Logger::Info("Successfully write symmetric Dirichlet distortion figure: \"%s\"!", figureOutput.c_str());
}

/* Atlas refinement with bounded packing efficiency (Sec. 4) */
double ComputeSignalApproximationError(const TriMesh * pMesh, const TriMesh * pUVMesh, const Eigen::Vector3d & bboxMin, const Eigen::Vector3d & bboxMax)
{
    const double splitN = 8.0;
    const double checkerDivide = 100.0;
    const double textureSize = 4096.0;

    Eigen::Vector3d gridLength = (bboxMax - bboxMin) / checkerDivide;
    double deltaU = (bboxMax - bboxMin).maxCoeff() / textureSize;
    double deltaV = (bboxMax - bboxMin).maxCoeff() / textureSize;

    auto GetcolorFunc = [&](const Eigen::Vector3d & pt)
    {
        auto pt1 = (pt - bboxMin).cwiseQuotient(gridLength) ;
        bool block = (((std::lround(std::floor(pt1[0])) + std::lround(std::floor(pt1[1])) + std::lround(std::floor(pt1[2]))) & 1) == 0);
        return block ? Eigen::Vector3d(1.0, 1.0, 1.0) : Eigen::Vector3d(0.0, 0.0, 0.0);
    };

    auto GetInteriorPointFunc = [](const std::vector<Eigen::Vector3d> & vList, double u, double v)
    {
        return u * vList[0] + v * vList[1] + (1.0 - u - v) * vList[2];
    };

    auto SignalErrorFunc = [&](const std::vector<Eigen::Vector3d> & vList, const std::vector<Eigen::Vector3d> & uvList, int i0, int j0, int i1, int j1, int i2, int j2)
    {
        std::vector<Eigen::Vector3d> pt(3), ptColor(3);
        double divisor = 1.0 / splitN;

        pt[0] = GetInteriorPointFunc(uvList, i0 * divisor, j0 * divisor);
        pt[1] = GetInteriorPointFunc(uvList, i1 * divisor, j1 * divisor);
        pt[2] = GetInteriorPointFunc(uvList, i2 * divisor, j2 * divisor);

        ptColor[0] = GetcolorFunc(GetInteriorPointFunc(vList, i0 * divisor, j0 * divisor));
        ptColor[1] = GetcolorFunc(GetInteriorPointFunc(vList, i1 * divisor, j1 * divisor));
        ptColor[2] = GetcolorFunc(GetInteriorPointFunc(vList, i2 * divisor, j2 * divisor));

        Eigen::Matrix3d paraMat;
        Eigen::Matrix3d signalMat;

        for (int i = 0; i < 3; i++)
        {
            paraMat(0, i) = pt[i][0] / deltaU;
            paraMat(1, i) = pt[i][1] / deltaV;
            paraMat(2, i) = 1.0;

            signalMat(0, i) = ptColor[i][0];
            signalMat(1, i) = ptColor[i][1];
            signalMat(2, i) = ptColor[i][2];
        }

        Eigen::Matrix3d jacobianMat = signalMat * paraMat.inverse();
        return jacobianMat.col(0).squaredNorm() + jacobianMat.col(1).squaredNorm();
    };

    double error = 0.0;
    double totalArea = 0.0;
    for (auto fh : pMesh->faces())
    {
        double faceArea = pMesh->calc_sector_area(*(pMesh->cfh_begin(fh)));
        totalArea += faceArea;
        faceArea /= splitN * splitN;

        std::vector<Eigen::Vector3d> vList, uvList;
        vList.reserve(3);
        uvList.reserve(3);
        for (const auto & heh : pMesh->fh_range(fh))
        {
            vList.push_back(pMesh->point(pMesh->to_vertex_handle(heh)));
            uvList.push_back(pUVMesh->point(pUVMesh->to_vertex_handle(heh)));
        }

        double faceError = 0.0;
        for (int j = 0; j <= splitN; j++)
        {
            for (int k = 0; k <= splitN - j; k++)
            {
                if (j + k < splitN)
                {
                    double signalError = SignalErrorFunc(vList, uvList, j, k, j + 1, k, j, k + 1);
                    if (signalError < 1e16)
                    {
                        faceError += faceArea * signalError;
                    }
                }
                if (j > 0 && k > 0)
                {
                    double signalError = SignalErrorFunc(vList, uvList, j, k, j - 1, k, j, k - 1);
                    if (signalError < 1e16)
                    {
                        faceError += faceArea * signalError;
                    }
                }
            }
        }
        error += faceError;
    }

    error /= 12.0 * totalArea;
    return error;
}

void MeshMeasure::EnergyStats(const TriMesh * pMesh, size_t clusterNum, const std::string & statsOutput, float totalTime)
{
    TriMesh uvMesh;
    std::vector<OpenMesh::VertexHandle> vhs;
    vhs.reserve(pMesh->n_vertices());

    for (const auto vh : pMesh->vertices())
    {
        TriMesh::TexCoord2D uv;
        for (const auto heh : pMesh->vih_range(vh))
        {
            if (!pMesh->is_boundary(heh))
            {
                uv = pMesh->texcoord2D(heh);
                break;
            }
        }

        for (const auto heh : pMesh->vih_range(vh))
        {
            if (!pMesh->is_boundary(heh))
            {
                if (pMesh->texcoord2D(heh) != uv)
                {
                    Logger::Error("Multiple different texcoords assigned to single vertex!");
                    return;
                }
            }
        }

        const auto newVH = uvMesh.add_vertex({uv[0], uv[1], 0.0});
        vhs.push_back(newVH);
    }

    for (const auto fh : pMesh->faces())
    {
        std::vector<OpenMesh::VertexHandle> faceVHs;
        for (const auto vh : pMesh->fv_range(fh))
        {
            faceVHs.push_back(vhs[vh.idx()]);
        }
        uvMesh.add_face(faceVHs);
    }

    Real totalAtlasArea = 1.0;
    Real atlasArea = 0.0;
    for (const auto & fh : uvMesh.faces())
    {
        atlasArea += uvMesh.calc_face_area(fh);
    }

    double PE = atlasArea / totalAtlasArea;

    std::vector<Real> boundaryDistances = MeshBoundaryDistances(pMesh);
    Real boundaryTotalDistance = 0.0;
    for (Real boundaryDistance : boundaryDistances)
    {
        boundaryTotalDistance += boundaryDistance;
    }

    Real xMin =  std::numeric_limits<Real>::max();
    Real yMin =  std::numeric_limits<Real>::max();
    Real zMin =  std::numeric_limits<Real>::max();
    Real xMax = -std::numeric_limits<Real>::max();
    Real yMax = -std::numeric_limits<Real>::max();
    Real zMax = -std::numeric_limits<Real>::max();

    for (const auto & vh : pMesh->vertices())
    {
        const auto & point = pMesh->point(vh);
        xMin = std::min(xMin, point[0]);
        yMin = std::min(yMin, point[1]);
        zMin = std::min(zMin, point[2]);
        xMax = std::max(xMax, point[0]);
        yMax = std::max(yMax, point[1]);
        zMax = std::max(zMax, point[2]);
    }

    Real diagonalAABB = std::sqrt(std::pow(xMax - xMin, 2.0) + std::pow(yMax - yMin, 2.0) + std::pow(zMax - zMin, 2.0));
    double BL = boundaryTotalDistance / diagonalAABB;

    std::vector<Jacobian::Singular> singularValues = Jacobian::ComputeSingularValues(pMesh);
    Real totalMeshArea = 0.0;
    std::vector<Real> faceAreas(pMesh->n_faces(), 0.0);
    for (const auto & fh : pMesh->faces())
    {
        faceAreas[fh.idx()] = pMesh->calc_face_area(fh);
        totalMeshArea += faceAreas[fh.idx()];
    }

    double Ed = 0.0;
    for (size_t i = 0; i < pMesh->n_faces(); i++)
    {
        double sigma1 = singularValues[i].first;
        double sigma2 = singularValues[i].second;
        if (sigma1 != 0.0 && sigma2 != 0.0)
        {
            double symDirichlet = std::pow(sigma1, 2.0) + std::pow(sigma2, 2.0) + std::pow(sigma1, -2.0) + std::pow(sigma2, -2.0);
            if (symDirichlet < 1e16)
            {
                Ed += (0.25 * (faceAreas[i] / totalMeshArea) * symDirichlet);
            }
        }
    }

    Eigen::Vector3d bboxMin(xMin, yMin, zMin);
    Eigen::Vector3d bboxMax(xMax, yMax, zMax);
    double SAE = ComputeSignalApproximationError(pMesh, &uvMesh, bboxMin, bboxMax);

    std::ofstream energyStatsFile(statsOutput);
    energyStatsFile << "Cluster Num: " << clusterNum << std::endl;
    energyStatsFile << "PE: " << PE << std::endl;
    energyStatsFile << "BL: " << BL << std::endl;
    energyStatsFile << "Ed: " << Ed << std::endl;
    energyStatsFile << "SAE: " << SAE << std::endl;
    energyStatsFile << "Time: " << totalTime << std::endl;
    energyStatsFile.close();
}