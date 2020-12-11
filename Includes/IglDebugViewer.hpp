#ifndef IGL_DEBUG_VIEWER_HPP
#define IGL_DEBUG_VIEWER_HPP

#include "Declaration.hpp"

void DisplayAtlas(TriMesh * pMesh, size_t atlasNum, const std::string & atlasIdxFPropName, const std::string & title);

void DisplayVF(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const std::string & title);

void DisplayVFC(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::MatrixXd & C, const std::string & title);

void DisplayVFD(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::VectorXd & D, const std::string & title);

#endif //IGL_DEBUG_VIEWER_HPP
