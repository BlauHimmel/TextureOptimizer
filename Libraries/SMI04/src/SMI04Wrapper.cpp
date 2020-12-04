#include "SMI04Wrapper.h"

#include"Point3d.h"
#include"Point2d.h"
#include"IDList.h"
#include"PointTool.h"
#include"PolarList.h"
#include"IDSet.h"
#include"PCBCGSolver.h"
#include"Polyhedron.h"

std::vector<std::vector<double>> SMI04(std::vector<std::vector<double>> & vertices, std::vector<std::vector<size_t>> & facets, const SMI04Configuration * pConfig, double * pSTL2error)
{
    std::vector<std::vector<double>> uv(vertices.size(), std::vector<double>(2, 0.0));

    Polyhedron * pPoly = nullptr;

    if (pConfig == nullptr)
    {
        pPoly = new Polyhedron();
    }
    else
    {
        pPoly = new Polyhedron(pConfig->pickID, pConfig->paramtype, pConfig->boundarytype, pConfig->weighttype,
            pConfig->iteNum, pConfig->gammaP, pConfig->PCBCGerror, pConfig->smooth, pConfig->intrinsiclambda,
            pConfig->boundarysigma);
    }

    pPoly->fromMyMesh(vertices, facets);
    pPoly->param();
    double stl2error = pPoly->toMyMeshTexture(uv);

    if (pSTL2error != nullptr)
    {
        *pSTL2error = stl2error;
    }

	delete pPoly;
    return uv;
}

