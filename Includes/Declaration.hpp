#ifndef DECLARATION_HPP
#define DECLARATION_HPP

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Geometry/EigenVectorT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

struct MeshTraits : public OpenMesh::DefaultTraits
{
    using Point      = Eigen::Vector3d;
    using Normal     = Eigen::Vector3d;
    using TexCoord1D = double;
    using TexCoord2D = Eigen::Vector2d;
    using TexCoord3D = Eigen::Vector3d;
};

using TriMesh = OpenMesh::PolyMesh_ArrayKernelT<MeshTraits>;
using Real = TriMesh::Scalar;

using Vector2 = Eigen::Matrix<Real, 2, 1>;
using Vector3 = Eigen::Matrix<Real, 3, 1>;
using Vector4 = Eigen::Matrix<Real, 4, 1>;
using VectorX = Eigen::Matrix<Real, Eigen::Dynamic, 1>;

using Matrix2 = Eigen::Matrix<Real, 2, 2>;
using Matrix3 = Eigen::Matrix<Real, 3, 3>;
using Matrix4 = Eigen::Matrix<Real, 4, 4>;
using MatrixX = Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic>;

using SparseMatrix = Eigen::SparseMatrix<Real, Eigen::ColMajor>;

using Index = Eigen::Index;
using Size = std::ptrdiff_t;

#endif //DECLARATION_HPP
