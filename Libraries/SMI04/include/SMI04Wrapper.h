#ifndef SMI04_WRAPPER_H
#define SMI04_WRAPPER_H

#include <vector>

struct SMI04Configuration
{
    /* --------------------------------------------------------------------------------------------------------------- */
    /* U^{1} or U^{Opt} */
    /* paramtype=1: U^{1} parameterization (after 1st step optimization) or paramtype = 2 (default): optimal parameterization. */
    int paramtype = 2;
    /* --------------------------------------------------------------------------------------------------------------- */
    /* Boundary Map */
    /* boundarytype = 0; unit square (default) */
    /* boundarytype = 1; unit circle */
    /* boundarytype = 2; natural boundary of the intrinsic parameterization */
    int boundarytype = 0;
    /* Start point ID */
    /* You can set a ID of boundary vertex which will be a conner vertex when the boundary map is a unit square. If it is
     * "-1" (default) the smallest number of boundary vertex ID will be the conner. */
    int pickID = -1;
    /* --------------------------------------------------------------------------------------------------------------- */
    /* Initial parameterization */
    /* weighttype=0; Shape Preserving (default): Floater CAGD 1997 */
    /* weighttype=1; Tutte 1963. */
    /* weighttype=2; Harmonic Map: Eck et al. SIGGRAPH 1995. */
    /* weighttype=3; Intrinsic Map: Desbrun et al. EUROGRAPHICS 2002. */
    /* blending parameter: ((1 - intrinsiclambda) * Chi + intrinsiclambda * Authalic). If intrinsiclambda = 1 then it is
     * equivalent to Eck's map. 0.5 is a default. 0 <= intrinsiclambda <= 1. */
    /* weighttype=4; Mean value: Floater CAGD 2003. */
    int weighttype = 0;
    double intrinsiclambda = 0.5;
    /* --------------------------------------------------------------------------------------------------------------- */
    /* Transfer Parameter */
    /* In the paper we expressed "\eta". gammaP = 1.0 is a default. 0< gammaP <=1. See the Figure 4 of the paper. */
    double gammaP = 1.0;
    /* --------------------------------------------------------------------------------------------------------------- */
    /* PCBCG parameters */
    /* It is better to tune when you apply to large meshes. */
    /* Approximation Error Threshold */
    /* 0.1^{6} is a default setting. When |Ax-b|/|b|<= PCBCGerror, linear solver would stop, see Numerical Recipes in C++. */
    double PCBCGerror = pow(0.1,6.0);
    /* Maximum Iteration Number */
    /* iteNum = 2000 is a default setting. If you apply the program to the mesh which has more than 20000 vertices then
     * you should increase the iteNum according to the vertex number of input mesh */
    int iteNum = 2000;
    /* --------------------------------------------------------------------------------------------------------------- */
    /* If you will have a problem (instability) by using smooth = 1 (default) then it is better to use smooth = 2. However
     * you should first try to change the parameters of the PCBCG (iteNum,PCBCGerror). */
    int smooth = 1;
    /* --------------------------------------------------------------------------------------------------------------- */
    /* With-Without Boundary Stretch */
    /* When we consider a metric tensor, it is not defined on the surface boundary. In discrete case, sometimes the boundary
     * triangle stretches case the numerical instability such that some of them go to almost zero, -\inf or \inf. Therefore,
     * this option allows us to ignore the boundary triangle stretches when we test the stretch distortion to decide whether
     * stop the optimization or not. If boundarysigma is equal to 0 then we sum up the triangle stretches excepting boundary
     * triangles. Otherwise, say boundarysigma=1, we sum up all triangle stretches. boundarysigma=1 is a default setting. */
    int boundarysigma = 1;
    /* --------------------------------------------------------------------------------------------------------------- */
};

std::vector<std::vector<double>> SMI04(std::vector<std::vector<double>> & vertices, std::vector<std::vector<size_t>> & facets, const SMI04Configuration * pConfig = nullptr, double * pSTL2error = nullptr);

#endif //SMI04_WRAPPER_H
