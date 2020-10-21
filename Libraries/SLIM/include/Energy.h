#ifndef ENERGY_H
#define ENERGY_H

#pragma warning(disable:4244 4018 4819 4305 4267 4715 4996 4101 4819 4267)

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <map>
#include <set>
#include <vector>


class Energy {

public:
 virtual double compute_energy(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
 								 Eigen::MatrixXd& uv) = 0;

 virtual void compute_grad(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
 								 Eigen::MatrixXd& uv, Eigen::MatrixXd& grad) = 0;
};

#endif // ENERGY_H
