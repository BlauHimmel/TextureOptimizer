#include "BarycentricParameterization.hpp"

#include <functional>
#include <mkl.h>

#include "Logger.hpp"
#include "MeshUtils.hpp"

using namespace BarycentricInternal;

namespace BarycentricInternal
{
    void FindInteriorBoundaryVertex(const TriMesh & mesh, std::vector<int> & interiorVertexIdxs,
                                    std::vector<int> & boundaryVertexIdxs)
    {
        interiorVertexIdxs.clear();
        boundaryVertexIdxs.clear();

        for (int i = 0; i < mesh.n_vertices(); i++)
        {
            const auto vh = TriMesh::VertexHandle(i);
            if (!mesh.is_boundary(vh))
            {
                interiorVertexIdxs.push_back(vh.idx());
            }
        }

        auto boundaries = MeshBoundaryVertices(&mesh);
        if (boundaries.size() != 1)
        {
            Logger::Error("Barycentric parameterization can only run on single boundary mesh.");
            throw std::runtime_error("Barycentric parameterization can only run on single boundary mesh.");
        }

        for (auto vh : boundaries[0])
        {
            boundaryVertexIdxs.push_back(vh.idx());
        }
    }

    void FixSquareBoundaryVertices(const std::vector<Real> & weight, std::vector<Real> & boundaryVertexPositions)
    {
        Real maxX = 1.0, minX = 0.0;
        Real maxY = 1.0, minY = 0.0;
        Real x = minX, y = minY, z = 0.0;
        Real boundaryLength = (maxX - minX + maxY - minY) * 2.0;
        int directionIdx = 0;
        Real compensate = 0.0;

        boundaryVertexPositions.resize(weight.size() * 3, 0.0);

        for (int i = 0; i < weight.size(); i++)
        {
            Real step = boundaryLength * weight[i];

            boundaryVertexPositions[3 * i + 0] = x;
            boundaryVertexPositions[3 * i + 1] = y;
            boundaryVertexPositions[3 * i + 2] = z;

            if (directionIdx == 0)
            {
                if (x + step > maxX)
                {
                    compensate = step - (maxX - x);
                    x = maxX;
                    directionIdx++;
                }
                else
                {
                    x += step;
                    if (x == maxX)
                    {
                        directionIdx++;
                    }
                }
            }
            else if (directionIdx == 1)
            {
                if (y + step > maxY)
                {
                    compensate = step - (maxY - y);
                    y = maxY;
                    directionIdx++;
                }
                else
                {
                    y += step;
                    if (compensate != 0.0)
                    {
                        y += compensate;
                        compensate = 0.0;
                    }
                    if (y == maxY)
                    {
                        directionIdx++;
                    }
                }
            }
            else if (directionIdx == 2)
            {
                if (x - step < minX)
                {
                    compensate = step - (x - minX);
                    x = minX;
                    directionIdx++;
                }
                else
                {
                    x -= step;
                    if (compensate != 0.0)
                    {
                        x -= compensate;
                        compensate = 0.0;
                    }
                    if (x == minX)
                    {
                        directionIdx++;
                    }
                }
            }
            else if (directionIdx == 3)
            {
                if (y - step < minY)
                {
                    compensate = step - (y - minY);
                    y = minY;
                    directionIdx++;
                }
                else
                {
                    y -= step;
                    if (compensate != 0.0)
                    {
                        y -= compensate;
                        compensate = 0.0;
                    }
                    if (y == minY)
                    {
                        directionIdx++;
                    }
                }
            }
        }
    }

    void FixCircleBoundaryVertices(const std::vector<Real> & weight, std::vector<Real> & boundaryVertexPositions)
    {
        Real radius = 0.5;
        Real theta = 0.0;
        Real parameters = 2.0 * M_PI;

        boundaryVertexPositions.resize(weight.size() * 3, 0.0);

        for (int i = 0; i < weight.size(); i++)
        {
            Real x = radius * std::cos(theta);
            Real y = radius * std::sin(theta);
            Real z = 0.0;

            boundaryVertexPositions[3 * i + 0] = x + radius;
            boundaryVertexPositions[3 * i + 1] = y + radius;
            boundaryVertexPositions[3 * i + 2] = z;

            Real step = parameters * weight[i];
            theta += step;
        }
    }

    void FixBoundaryVertices(const TriMesh & mesh, const std::vector<int> & boundaryVertexIdxs, EBoundary boundary,
                             EBoundaryFixWeight fixWeight, std::vector<Real> & boundaryVertexPositions)
    {
        std::vector<Real> weight;

        if (fixWeight == EBoundaryFixWeight::Mean)
        {
            Real meanWeight = 1.0 / Real(boundaryVertexIdxs.size());
            weight.resize(boundaryVertexIdxs.size(), meanWeight);
        }
        else if (fixWeight == EBoundaryFixWeight::Length)
        {
            weight.resize(boundaryVertexIdxs.size());
            Real totalLength = 0.0;

            for (int i = 0; i < boundaryVertexIdxs.size() - 1; i++)
            {
                const auto vh0 = TriMesh::VertexHandle(boundaryVertexIdxs[i]);
                const auto vh1 = TriMesh::VertexHandle(boundaryVertexIdxs[i + 1]);

                const auto & p0 = mesh.point(vh0);
                const auto & p1 = mesh.point(vh1);

                Real length = (p0 - p1).norm();

                weight[i] = length;
                totalLength += length;
            }

            const auto vh0 = TriMesh::VertexHandle(boundaryVertexIdxs[boundaryVertexIdxs.size() - 1]);
            const auto vh1 = TriMesh::VertexHandle(boundaryVertexIdxs[0]);

            const auto & p0 = mesh.point(vh0);
            const auto & p1 = mesh.point(vh1);

            Real length = (p0 - p1).norm();

            weight[boundaryVertexIdxs.size() - 1] = length;
            totalLength += length;

            Real invTotalLength = 1.0 / totalLength;
            for (int i = 0; i < boundaryVertexIdxs.size() - 1; i++)
            {
                weight[i] *= invTotalLength;
            }
        }
        else
        {
            Logger::Error("Unknown fix weight found when fixing the boundary vertices.");
            throw std::runtime_error("Unknown fix weight found when fixing the boundary vertices.");
        }

        if (boundary == EBoundary::Square)
        {
            FixSquareBoundaryVertices(weight, boundaryVertexPositions);
        }
        else if (boundary == EBoundary::Circle)
        {
            FixCircleBoundaryVertices(weight, boundaryVertexPositions);
        }
        else
        {
            Logger::Error("Unknown boundary shape found when fixing the boundary vertices.");
            throw std::runtime_error("Unknown boundary shape found when fixing the boundary vertices.");
        }
    }

    Real WijWachspressCoordinates(const TriMesh &mesh, int vertexIdxI, int vertexIdxJ)
    {
        /**
        *      vj
        *    //||\\
        *   vk || vl
        *    \\||//
        *      vi
        */

        const auto vhi = TriMesh::VertexHandle(vertexIdxI);
        const auto vhj = TriMesh::VertexHandle(vertexIdxJ);
        const auto hehij = mesh.find_halfedge(vhi, vhj);
        const auto eh = mesh.edge_handle(hehij);

        const auto hehji = mesh.opposite_halfedge_handle(hehij);
        const auto hehil = mesh.next_halfedge_handle(hehji);
        const auto hehlj = mesh.next_halfedge_handle(hehil);

        const auto rij = mesh.calc_edge_length(eh);
        const auto alphaji = mesh.calc_sector_angle(hehij);
        const auto betaij = mesh.calc_sector_angle(hehlj);

        if (rij == 0.0)
        {
            return 0.0;
        }

        return (1.0 / std::tan(alphaji) + 1.0 / std::tan(betaij)) / (rij * rij);
    }

    Real WijDiscreteHarmonicCoordinates(const TriMesh &mesh, int vertexIdxI, int vertexIdxJ)
    {
        /**
        *      vj
        *    //||\\
        *   vk || vl
        *    \\||//
        *      vi
        */

        const auto vhi = TriMesh::VertexHandle(vertexIdxI);
        const auto vhj = TriMesh::VertexHandle(vertexIdxJ);
        const auto hehij = mesh.find_halfedge(vhi, vhj);

        const auto hehji = mesh.opposite_halfedge_handle(hehij);
        const auto hehjk = mesh.next_halfedge_handle(hehij);
        const auto hehil = mesh.next_halfedge_handle(hehji);

        const auto gammaij = mesh.calc_sector_angle(hehil);
        const auto gammaji = mesh.calc_sector_angle(hehjk);

        return 1.0 / std::tan(gammaij) + 1.0 / std::tan(gammaji);
    }

    Real WijMeanValueCoordinates(const TriMesh &mesh, int vertexIdxI, int vertexIdxJ)
    {
        /**
        *      vj
        *    //||\\
        *   vk || vl
        *    \\||//
        *      vi
        */

        const auto vhi = TriMesh::VertexHandle(vertexIdxI);
        const auto vhj = TriMesh::VertexHandle(vertexIdxJ);
        const auto hehij = mesh.find_halfedge(vhi, vhj);
        const auto eh = mesh.edge_handle(hehij);

        const auto hehji = mesh.opposite_halfedge_handle(hehij);
        const auto hehjk = mesh.next_halfedge_handle(hehij);
        const auto hehki = mesh.next_halfedge_handle(hehjk);

        const auto rij = mesh.calc_edge_length(eh);
        const auto betaji = mesh.calc_sector_angle(hehki);
        const auto alphaij = mesh.calc_sector_angle(hehji);

        if (rij == 0.0)
        {
            return 0.0;
        }

        return (std::tan(0.5 * alphaij) + std::tan(0.5 * betaji)) / rij;
    }

    Real WijTutteCoordinates(const TriMesh &mesh, int vertexIdxI, int vertexIdxJ)
    {
        return 1.0;
    }
}

void BarycentricMapping(TriMesh & mesh, EBoundary boundary, EBoundaryFixWeight fixWeight, ECoefficient coefficent)
{
    std::function<double(int, int)> wijFunc;

    if (coefficent == ECoefficient::Wachspress)
    {
        wijFunc = std::bind(&WijWachspressCoordinates, std::ref(mesh), std::placeholders::_1, std::placeholders::_2);
    }
    else if (coefficent == ECoefficient::DiscreteHarmonic)
    {
        wijFunc = std::bind(&WijDiscreteHarmonicCoordinates, std::ref(mesh), std::placeholders::_1, std::placeholders::_2);
    }
    else if (coefficent == ECoefficient::MeanValue)
    {
        wijFunc = std::bind(&WijMeanValueCoordinates, std::ref(mesh), std::placeholders::_1, std::placeholders::_2);
    }
    else if (coefficent == ECoefficient::Tutte)
    {
        wijFunc = std::bind(&WijTutteCoordinates, std::ref(mesh), std::placeholders::_1, std::placeholders::_2);
    }
    else
    {
        Logger::Error("Unknown barycentric coefficent found when computing Lambda_ij");
        throw std::runtime_error("Unknown barycentric coefficent found when computing Lambda_ij");
    }

    std::vector<int> interiorVertexIdxs;
    std::vector<int> boundaryVertexIdxs;
    std::vector<Real> boundaryVertexPositions;

    FindInteriorBoundaryVertex(mesh, interiorVertexIdxs, boundaryVertexIdxs);
    FixBoundaryVertices(mesh, boundaryVertexIdxs, boundary, fixWeight, boundaryVertexPositions);

    /* Use Pardiso to solve the sparse system. Code is ugly but efficient (Modified from the doc of Pardiso). */
    /* Lowercase variables are used for Pardiso */

    int n = int(interiorVertexIdxs.size());

    std::vector<std::vector<Real>> wijLists(n);
    std::vector<Real> sigmaWijLists(n);
    std::vector<std::vector<int>> adjVertexIdxLists(n);

    for (int i = 0; i < n; i++)
    {
        const auto vh = TriMesh::VertexHandle(interiorVertexIdxs[i]);
        for (auto vhAdjacent : mesh.vv_range(vh))
        {
            adjVertexIdxLists[i].push_back(vhAdjacent.idx());
        }
        wijLists[i].resize(adjVertexIdxLists[i].size());

        Real sigmaWij = 0.0;

        for (int j = 0; j < adjVertexIdxLists[i].size(); j++)
        {
            Real wij = wijFunc(interiorVertexIdxs[i], adjVertexIdxLists[i][j]);
            wijLists[i][j] = wij;
            sigmaWij += wij;
        }
        sigmaWijLists[i] = sigmaWij;
    }

    std::vector<Real> interiorVertexPositions;
    interiorVertexPositions.resize(interiorVertexIdxs.size() * 3, 0.0);

    std::vector<MKL_INT> ia;
    std::vector<MKL_INT> ja;
    std::vector<double> a;

    auto * pA/*Size = n * 1*/ = reinterpret_cast<double *>(mkl_malloc(sizeof(double) * n, 64));
    auto * pB/*Size = n * 2*/ = reinterpret_cast<double *>(mkl_malloc(sizeof(double) * n * 2, 64));

    memset(pA, 0, sizeof(double) * n);
    memset(pB, 0, sizeof(double) * n * 2);

    int nonZeroIdx = 0;
    int prevRowIdx = 0;

    for (int i = 0; i < n; i++)
    {
        pA[i] = 1.0;
        for (int j = 0; j < adjVertexIdxLists[i].size(); j++)
        {
            auto iter = std::find(interiorVertexIdxs.begin(), interiorVertexIdxs.end(), adjVertexIdxLists[i][j]);
            if (iter != interiorVertexIdxs.end())
            {
                auto lambdaij = static_cast<double>(wijLists[i][j] / sigmaWijLists[i]);
                int interiorIdx = int(iter - interiorVertexIdxs.begin());
                pA[interiorIdx] = -1.0 * lambdaij;
            }
        }

        for (int j = 0; j < n; j++)
        {
            if (pA[j] != 0.0)
            {
                nonZeroIdx++;
                int rowIdx = i + 1;
                int colIdx = j + 1;
                a.push_back(pA[j]);
                pA[j] = 0;
                ja.push_back(colIdx);

                if (rowIdx - prevRowIdx == 1)
                {
                    prevRowIdx = rowIdx;
                    ia.push_back(nonZeroIdx);
                }
            }
        }
    }
    ia.push_back(MKL_INT(a.size() + 1));

    mkl_free(pA);

    for (int i = 0; i < n; i++)
    {
        double u = 0.0, v = 0.0;
        for (int j = 0; j < adjVertexIdxLists[i].size(); j++)
        {
            const auto vh = TriMesh::VertexHandle(adjVertexIdxLists[i][j]);
            if (mesh.is_boundary(vh))
            {
                auto lambdaij = static_cast<double>(wijLists[i][j] / sigmaWijLists[i]);
                auto iter = std::find(boundaryVertexIdxs.begin(), boundaryVertexIdxs.end(), adjVertexIdxLists[i][j]);
                int boundaryIdx = int(iter - boundaryVertexIdxs.begin());
                u += double(lambdaij * boundaryVertexPositions[boundaryIdx * 3 + 0]);
                v += double(lambdaij * boundaryVertexPositions[boundaryIdx * 3 + 1]);
            }
        }

        pB[0 + i] = u;
        pB[n + i] = v;
    }

    /*Real unsymmetric matrix*/
    MKL_INT mtype = 11;
    /*Descriptor of main sparse matrix properties*/
    matrix_descr descrA {};
    /*Structure with sparse matrix stored in CSR format*/
    sparse_matrix_t csrA;
    sparse_operation_t transA;
    /*RHS and solution vectors.*/
    std::unique_ptr<double[]> x(new double[n * 2]);
    /*Number of right hand sides.*/
    MKL_INT nrhs = 2;
    /*Internal solver memory pointer pt,*/
    /*32-bit: int pt[64]; 64-bit: long int pt[64]*/
    /*or void *pt[64] should be OK on both architectures*/
    void * pt[64];
    /*Pardiso control parameters.*/
    MKL_INT iparm[64];
    MKL_INT maxfct, mnum, phase, error, msglvl;
    /* Auxiliary variables. */
    MKL_INT i, j;
    /* Double dummy */
    double ddum;
    /* Integer dummy. */
    MKL_INT idum;
    /*Setup Pardiso control parameters*/
    for (i = 0; i < 64; i++) { iparm[i] = 0; }
    iparm[0] = 1;         /* No solver default */
    iparm[1] = 2;         /* Fill-in reordering from METIS */
    iparm[3] = 0;         /* No iterative-direct algorithm */
    iparm[4] = 0;         /* No user fill-in reducing permutation */
    iparm[5] = 0;         /* Write solution into x */
    iparm[6] = 0;         /* Not in use */
    iparm[7] = 2;         /* Max numbers of iterative refinement steps */
    iparm[8] = 0;         /* Not in use */
    iparm[9] = 13;        /* Perturb the pivot elements with 1E-13 */
    iparm[10] = 1;        /* Use nonsymmetric permutation and scaling MPS */
    iparm[11] = 0;        /* Conjugate transposed/transpose solve */
    iparm[12] = 1;        /* Maximum weighted matching algorithm is switched-on (default for non-symmetric) */
    iparm[13] = 0;        /* Output: Number of perturbed pivots */
    iparm[14] = 0;        /* Not in use */
    iparm[15] = 0;        /* Not in use */
    iparm[16] = 0;        /* Not in use */
    iparm[17] = -1;       /* Output: Number of nonzeros in the factor LU */
    iparm[18] = -1;       /* Output: Mflops for LU factorization */
    iparm[19] = 0;        /* Output: Numbers of CG Iterations */
    maxfct = 1;           /* Maximum number of numerical factorizations. */
    mnum = 1;             /* Which factorization to use. */
    msglvl = 0;           /* Print statistical information  */
    error = 0;            /* Initialize error flag */

    /*Initialize the internal solver memory pointer. This is only*/
    /*necessary for the FIRST call of the PARDISO solver.*/
    for (i = 0; i < 64; i++) { pt[i] = nullptr; }

    /*Reordering and Symbolic Factorization. This step also allocates*/
    /*all memory that is necessary for the factorization.*/
    phase = 11;
    PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a.data(), ia.data(), ja.data(), &idum, &nrhs, iparm, &msglvl, &ddum, &ddum, &error);
    if (error != 0)
    {
        Logger::Error("Error occur during symbolic factorization: %d", error);
        throw std::runtime_error(Logger::Format("Error occur during symbolic factorization: %d", error));
    }

    /*Numerical factorization.*/
    phase = 22;
    PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a.data(), ia.data(), ja.data(), &idum, &nrhs, iparm, &msglvl, &ddum, &ddum, &error);

    if (error != 0)
    {
        Logger::Error("Error occur during numerical factorization: %d", error);
        throw std::runtime_error(Logger::Format("Error occur during numerical factorization: %d", error));
    }

    phase = 33;

    /*Back substitution and iterative refinement.*/
    descrA.type = SPARSE_MATRIX_TYPE_GENERAL;
    descrA.mode = SPARSE_FILL_MODE_UPPER;
    descrA.diag = SPARSE_DIAG_NON_UNIT;
    mkl_sparse_d_create_csr(&csrA, SPARSE_INDEX_BASE_ONE, n, n, ia.data(), ia.data() + 1, ja.data(), a.data());

    iparm[11] = 0;
    transA = SPARSE_OPERATION_NON_TRANSPOSE;

    PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a.data(), ia.data(), ja.data(), &idum, &nrhs, iparm, &msglvl, pB, x.get(), &error);

    if (error != 0)
    {
        Logger::Error("Error occur during final stage: %d", error);
        throw std::runtime_error(Logger::Format("Error occur during final stage: %d", error));
    }

    for (j = 0; j < n; j++)
    {
        interiorVertexPositions[j * 3 + 0] = x[0 + j];
        interiorVertexPositions[j * 3 + 1] = x[n + j];
    }

    mkl_sparse_destroy(csrA);

    /*Termination and release of memory.*/
    phase = -1;
    PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, &ddum, ia.data(), ja.data(), &idum, &nrhs, iparm, &msglvl, &ddum, &ddum, &error);

    mkl_free(pB);

    for (int k = 0; k < interiorVertexIdxs.size(); k++)
    {
        const auto vh = TriMesh::VertexHandle(interiorVertexIdxs[k]);
        mesh.set_point(vh, {interiorVertexPositions[3 * k + 0], interiorVertexPositions[3 * k + 1], interiorVertexPositions[3 * k + 2] });
    }

    for (int k = 0; k < boundaryVertexIdxs.size(); k++)
    {
        const auto vh = TriMesh::VertexHandle(boundaryVertexIdxs[k]);
        mesh.set_point(vh, {boundaryVertexPositions[3 * k + 0], boundaryVertexPositions[3 * k + 1], boundaryVertexPositions[3 * k + 2] });
    }
}