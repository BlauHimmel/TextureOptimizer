#ifndef OMT_HPP
#define OMT_HPP

#include "Declaration.hpp"

class OMT2D
{
public:
    explicit OMT2D(const TriMesh * pMesh) : m_pMesh(pMesh) { }

private:
    static void NormalizeUVToUnitSquare(TriMesh * pMesh);
    static bool CheckInitParamterization(TriMesh * pMesh);
    static void ComputeTargetArea(const TriMesh * pMesh, std::vector<Real> & targetAreas);

private:
    const TriMesh * m_pMesh = nullptr;

};

#endif //OMT_HPP
