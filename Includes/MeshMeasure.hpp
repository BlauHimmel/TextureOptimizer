#ifndef MESH_MEASURE_HPP
#define MESH_MEASURE_HPP

#include "Declaration.hpp"

class MeshMeasure
{
public:
    static void AreaDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput);
    static void AngleDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput);
    static void SymmetricDirichletDistortion(const TriMesh * pMesh, const std::string & statsOutput, const std::string & figureOutput);
};

#endif //MESH_MEASURE_HPP
