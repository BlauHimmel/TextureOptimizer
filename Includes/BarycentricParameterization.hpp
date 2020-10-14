#ifndef BARYCENTRICPA_RAMETERIZATION_HPP
#define BARYCENTRICPA_RAMETERIZATION_HPP

#include "Declaration.hpp"

enum class EBoundary
{
    Circle, Square
};

enum class EBoundaryFixWeight
{
    Mean, Length
};

enum class ECoefficient
{
    MeanValue, DiscreteHarmonic, Wachspress, Tutte
};

namespace BarycentricInternal
{
    /** \brief Find the interior vertices and boundary vertices. Boundary
    * vertices are store one by one on the loop.
    *
    * @param mesh               mesh to be operated
    * @param interiorVertexIdxs Indices of the interior point (out-of-order)
    * @param boundaryVertexIdxs Indices of the boundary point (one by one)
    */
    void FindInteriorBoundaryVertex(const TriMesh & mesh, std::vector<int> & interiorVertexIdxs,
                                    std::vector<int> & boundaryVertexIdxs);

    /** \brief Compute the position of each boundary vertex according to
    * Weight passed in. Vertices will be fixed on a unit square.
    *
    * @param weight                  Weight of each boundary vertex.
    * @param boundaryVertexPositions Positions of each vertex.(x1, y1, z1,
    *                                x2, y2, z2, ..., xn, yn, zn)
    */
    void FixSquareBoundaryVertices(const std::vector<Real> & weight, std::vector<Real> & boundaryVertexPositions);

    /** \brief Compute the position of each boundary vertex according to
    * Weight passed in. Vertices will be fixed on a unit circle.
    *
    * @param weight                  Weight of each boundary vertex.
    * @param boundaryVertexPositions Positions of each vertex.(x1, y1, z1,
    *                                x2, y2, z2, ..., xn, yn, zn)
    */
    void FixCircleBoundaryVertices(const std::vector<Real> & weight, std::vector<Real> & boundaryVertexPositions);

    /** \brief Compute the position of each boundary vertex.
    *
    * @param mesh                    Mesh to be operated
    * @param boundaryVertexIdxs      Indices of the boundary point (one by one)
    * @param boundary                Shape of the boundary
    * @param fixWeight               How to fix the boundary point?
    * @param boundaryVertexPositions Positions of each vertex.(x1, y1, z1,
    *                                x2, y2, z2, ..., xn, yn, zn)
    */
    void FixBoundaryVertices(const TriMesh & mesh, const std::vector<int> & boundaryVertexIdxs, EBoundary boundary,
                             EBoundaryFixWeight fixWeight, std::vector<Real> & boundaryVertexPositions);

    /** \brief Compute Wij term (Wachspress coordinates).
    *
    * @param mesh       Mesh to be operated
    * @param vertexIdxI Index of the vertex Xi
    * @param vertexIdxJ Index of the vertex Xj
    */
    Real WijWachspressCoordinates(const TriMesh & mesh, int vertexIdxI, int vertexIdxJ);

    /** \brief Compute Wij term (Discrete harmonic coordinates).
    *
    * @param mesh       Mesh to be operated
    * @param vertexIdxI Index of the vertex Xi
    * @param vertexIdxJ Index of the vertex Xj
    */
    Real WijDiscreteHarmonicCoordinates(const TriMesh & mesh, int vertexIdxI, int vertexIdxJ);

    /** \brief Compute Wij term (Mean value coordinates).
    *
    * @param mesh       Mesh to be operated
    * @param vertexIdxI Index of the vertex Xi
    * @param vertexIdxJ Index of the vertex Xj
    */
    Real WijMeanValueCoordinates(const TriMesh & mesh, int vertexIdxI, int vertexIdxJ);

    /** \brief Compute Wij term (Tutte coordinates).
    *
    * @param mesh       Mesh to be operated
    * @param vertexIdxI Index of the vertex Xi
    * @param vertexIdxJ Index of the vertex Xj
    */
    Real WijTutteCoordinates(const TriMesh & mesh, int vertexIdxI, int vertexIdxJ);
}

/** \brief Parameterize the mesh using barycentric mapping.
*
* @param mesh       Mesh to be operated
* @param boundary   Shape of the boundary
* @param fixWeight  How to fix the boundary point?
* @param coefficent Barycentric coefficient function
*/
void BarycentricMapping(TriMesh & mesh, EBoundary boundary, EBoundaryFixWeight fixWeight, ECoefficient coefficent);

#endif //BARYCENTRIC_PARAMETERIZATION_HPP
