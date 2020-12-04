#include "OMT.hpp"

#include "Logger.hpp"

void OMT2D::NormalizeUVToUnitSquare(TriMesh * pMesh)
{
    Real uMax = -std::numeric_limits<Real>::max();
    Real vMax = -std::numeric_limits<Real>::max();
    Real uMin =  std::numeric_limits<Real>::max();
    Real vMin =  std::numeric_limits<Real>::max();

    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        uMax = std::max(uMax, uv[0]);
        vMax = std::max(vMax, uv[1]);
        uMin = std::min(uMin, uv[0]);
        vMin = std::min(vMin, uv[1]);
    }

    Real invDeltaU = 1.0 / (uMax - uMin);
    Real invDeltaV = 1.0 / (vMax - vMin);

    for (auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        pMesh->set_texcoord2D(vh, {
            (uv[0] - uMin) * invDeltaU * 2.0 - 1.0,
            (uv[1] - vMin) * invDeltaV * 2.0 - 1.0
        });
    }
}

bool OMT2D::CheckInitParamterization(TriMesh * pMesh)
{
    std::set<std::pair<Real, Real>> uvSet;
    bool bOverlapped = false;
    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        auto [iter, bSuccess] = uvSet.insert({ uv[0], uv[1] });
        if (!bSuccess)
        {
            Logger::Error("Overlapped uv texcoord found. index = %d, uv = (%lf, %lf).", vh.idx(), uv[0], uv[1]);
            bOverlapped = true;
        }
    }

    return !bOverlapped;
}

void OMT2D::ComputeTargetArea(const TriMesh * pMesh, std::vector<Real> & targetAreas)
{
    targetAreas.resize(pMesh->n_vertices(), 0);

    Real totalMeshArea = 0.0;
    for (const auto & fh : pMesh->faces())
    {
        TriMesh::Point points[3];
        size_t Index = 0;

        for (const auto & vh : pMesh->fv_range(fh))
        {
            points[Index++] = pMesh->point(vh);
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
        for (const auto & fh : pMesh->vf_range(vh))
        {
            TriMesh::Point points[3];
            size_t Index = 0;

            for (const auto & fhvh : pMesh->fv_range(fh))
            {
                points[Index++] = pMesh->point(fhvh);
            }

            const auto & va = points[0];
            const auto & vb = points[1];
            const auto & vc = points[2];

            auto lab = va - vb;
            auto lac = vc - vb;

            Real area = lab.cross(lac).norm() * 0.5;
            oneRingMeshArea += area / 3.0;
        }

        const Real totalTargetArea = 4.0;
        targetAreas[vh.idx()] = oneRingMeshArea / totalMeshArea * totalTargetArea;
    }
}