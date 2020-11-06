#include "SLIMWrapper.hpp"

#include <vector>

#include "Param_State.h"
#include "GlobalLocalParametrization.h"
#include "StateManager.h"

std::vector<std::vector<double>> SLIM(std::vector<std::vector<double>> & vertices, std::vector<std::vector<size_t>> & facets, int iterations, int * pCurrIterations)
{
    Param_State state;
    state.method = Param_State::GLOBAL_ARAP_IRLS;
    state.flips_linesearch = true;
    state.update_all_energies = false;
    state.proximal_p = 0.0001;

    state.F.resize(facets.size(), facets[0].size());
    for (size_t i = 0; i < facets.size(); i++)
    {
        state.F(i, 0) = facets[i][0];
        state.F(i, 1) = facets[i][1];
        state.F(i, 2) = facets[i][2];
    }

    state.V.resize(vertices.size(), vertices[0].size());
    for (size_t i = 0; i < vertices.size(); i++)
    {
        state.V(i, 0) = vertices[i][0];
        state.V(i, 1) = vertices[i][1];
        state.V(i, 2) = vertices[i][2];
    }

    state.v_num = state.V.rows();
    state.f_num = state.F.rows();

    igl::doublearea(state.V, state.F, state.M);
    state.M /= 2.0;

    state.global_local_energy = Param_State::SYMMETRIC_DIRICHLET;
    state.cnt_flips = false;

    state.mesh_area = state.M.sum();
    state.V /= std::sqrt(state.mesh_area);
    state.mesh_area = 1;

    StateManager stateManager;
    GlobalLocalParametrization param(stateManager, &state);
    param.init_parametrization();

    for (int i = 0; i < iterations; i++)
    {
        param.single_iteration();
        if (pCurrIterations != nullptr)
        {
            *pCurrIterations = i + 1;
        }
    }

    std::vector<std::vector<double>> uv(state.uv.rows(), std::vector<double>(state.uv.cols()));
    for (size_t i = 0; i < uv.size(); i++)
    {
        for (size_t j = 0; j < uv[i].size(); j++)
        {
            uv[i][j] = state.uv(i, j);
        }
    }

    return uv;
}