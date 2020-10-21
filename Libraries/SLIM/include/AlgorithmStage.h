#ifndef ALGORITHM_STAGE_H
#define ALGORITHM_STAGE_H

#pragma warning(disable:4244 4018 4819 4305 4267 4715 4996 4101 4819 4267)

#ifdef HAS_GUI

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <map>
#include <set>
#include <vector>

#include <igl/viewer/Viewer.h>

#include "Param_State.h"

class AlgorithmStage {

public:
 AlgorithmStage(Param_State* dd_param) : m_dd(dd_param) {}
 virtual void key_pressed(igl::viewer::Viewer& viewer, unsigned char key) = 0;

 Param_State* m_dd;
};

#endif
#endif // ALGORITHM_STAGE_H
