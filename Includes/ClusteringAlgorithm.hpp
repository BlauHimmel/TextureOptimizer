#ifndef CLUSTERING_ALGORITHM_HPP
#define CLUSTERING_ALGORITHM_HPP

#include <unordered_set>
#include <unordered_map>

#include "Declaration.hpp"

struct PairHasher
{
    size_t operator()(const std::pair<int, int> & pt) const
    {
        return std::hash<Real>()(pt.first) ^ std::hash<Real>()(pt.second);
    }
};

struct DualNodeType
{
    std::unordered_set<int> faces;
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    double c = 0.0;
    int id = -1;

    /*double area = 0.0;
    double perimeter = 0.0;
    double irregularity = 0.0;*/

    const TriMesh * pAttachedMesh = nullptr;
    const std::vector<double> * pFaceAreas = nullptr;
    const std::vector<double> * pEdgeLengths = nullptr;
    const std::unordered_map<std::pair<int, int>, double, PairHasher> * pIntersectLength = nullptr;

    DualNodeType() = default;
    DualNodeType(TriMesh * pMesh, const std::unordered_map<std::pair<int, int>, double, PairHasher> & intersectLength,
        const std::vector<double> & faceAreas, const std::vector<double> & edgeLengths, int facet,
        Eigen::Matrix3d A, const Eigen::Vector3d & b, double c);
    DualNodeType operator+(const DualNodeType & rhs) const;
};

// Ref: Hierarchical Face Clustering on Polygonal Surfaces
class ClusteringAlgorithm
{
public:
    static std::vector<std::vector<int>> Cluster(TriMesh * pMesh, size_t clusterNum = 9, double threshold = 0.02);

private:
    static void GetDualEdges(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, std::vector<std::pair<int, int>> & dualEdges);
    static double EvaluateCost(const DualNodeType & dn1, const DualNodeType & dn2, const std::unordered_map<std::pair<int, int>, double, PairHasher> & intersectLength);
};

#endif //CLUSTERING_ALGORITHM_HPP
