#ifndef OMT_HPP
#define OMT_HPP

#include "Declaration.hpp"

template <typename T1, typename T2>
struct PairHasher
{
    size_t operator()(const std::pair<T1, T2> & pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

class OMT2D
{
public:
    using HessianWeightMap = std::unordered_map<std::pair<int, int>, Real, PairHasher<int, int>>;

    explicit OMT2D(TriMesh * pMesh) : m_pMesh(pMesh) { }

    bool Initialize();
    void Run(int iterations, Real step);

private:
    static void NormalizeUVToUnitSquare(TriMesh * pMesh);
    static bool CheckInitParamterization(TriMesh * pMesh);
    static void InitializeWeight(const TriMesh * pMesh, std::vector<Real> & weights);
    static void ComputeTargetArea(const TriMesh * pMesh, std::vector<Real> & targetAreas);
    static bool ComputePowerDiagram(const TriMesh * pMesh, const std::vector<Real> & weights, const std::vector<Real> & targetAreas,
        std::vector<Real> & currentAreas, std::vector<Real> & gradients, HessianWeightMap & hessianWeightMap);
    static void UpdateWeight(const std::vector<Real> & updateVectors, Real step, std::vector<Real> & weights);
    static void ComputeUpdateVectorGD(const std::vector<Real> & gradients, std::vector<Real> & updateVectors);
    static Real ComputeError(const std::vector<Real> & gradients);

private:
    TriMesh * m_pMesh;
    std::vector<Real> m_TargetAreas;     /* vertex num */
    std::vector<Real> m_CurrentAreas;    /* vertex num */
    std::vector<Real> m_Weights;         /* vertex num */
    std::vector<Real> m_Gradients;       /* vertex num */
    std::vector<Real> m_UpdateVectors;   /* vertex num */
    HessianWeightMap m_HessianWeightMap; /* <<i, j>, h(i, j)> */
};

#endif //OMT_HPP
