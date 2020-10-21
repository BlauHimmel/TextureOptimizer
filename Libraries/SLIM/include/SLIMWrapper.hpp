#ifndef SLIM_WRAPPER_HPP
#define SLIM_WRAPPER_HPP

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>

std::vector<std::vector<double>> SLIM(std::vector<std::vector<double>> & vertices, std::vector<std::vector<size_t>> & facets, int iterations = 20, int * pCurrIterations = nullptr);

#endif //SLIM_WRAPPER_HPP
