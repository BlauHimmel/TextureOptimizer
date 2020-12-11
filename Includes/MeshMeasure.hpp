#ifndef MESH_MEASURE_HPP
#define MESH_MEASURE_HPP

#include "Declaration.hpp"

class MeshMeasure
{
public:
    static void AreaDistortion(const TriMesh * pMesh, const std::string & statsOutput);
};

#endif //MESH_MEASURE_HPP
