#ifndef CLUSTERING_ALGORITHM_HPP
#define CLUSTERING_ALGORITHM_HPP

#include <unordered_set>

#include "Declaration.hpp"

struct DualNodeType
{
    std::unordered_set<int> faces;
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    double c = 0.0;

    DualNodeType() = default;
    DualNodeType(int facet, const Eigen::Matrix3d & A, const Eigen::Vector3d & b, double c);
    DualNodeType operator+(const DualNodeType & rhs) const;
};

// Ref: Hierarchical Face Clustering on Polygonal Surfaces
class ClusteringAlgorithm
{
public:
    std::vector<std::vector<int>> Cluster(TriMesh * pMesh, double threshold = 2e-2);

private:
    void GetDualEdges(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, std::vector<std::pair<int, int>> & dualEdges) const;
    double EvaluateCost(const DualNodeType & dn1, const DualNodeType & dn2) const;
};

#endif //CLUSTERING_ALGORITHM_HPP
