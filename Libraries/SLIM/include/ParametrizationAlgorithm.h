#ifndef PARAMETRIZATION_ALGORITHM_H
#define PARAMETRIZATION_ALGORITHM_H

#pragma warning(disable:4244 4018 4819 4305 4267 4715 4996 4101 4819 4267)

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <map>
#include <set>
#include <vector>

#ifdef HAS_GUI
#include <igl/viewer/Viewer.h>
#endif

#include "StateManager.h"

class ParametrizationAlgorithm {

public:
 ParametrizationAlgorithm(StateManager& state_manager, Param_State* m_state) : m_stateManager(state_manager),
 															 m_state(m_state) {}

 StateManager& m_stateManager;
 Param_State* m_state;
};

#endif // PARAMETRIZATION_ALGORITHM_H
