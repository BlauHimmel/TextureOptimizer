#include "TextureOptimizer.hpp"

#include <algorithm>
#include <limits>
#include <thread>
#include <chrono>

#include <progress_bar.h>
#include <SLIMWrapper.hpp>

#include "Logger.hpp"
#include "BarycentricParameterization.hpp"
#include "AtlasMerger.hpp"
#include "MeshUtils.hpp"

bool TextureOptimizer::Run(uint32_t width, uint32_t height, EParameterization parameterization, TriMesh * pCutMesh)
{
    if (m_pMesh == nullptr || m_pTexture == nullptr || !m_pTexture->IsValid())
    {
        return false;
    }

    Logger::Info("Removing small components of the mesh...");
    RemoveSmallComponent(m_pMesh);
    Logger::Info("<RemoveSmallComponent> Completed!");

    Logger::Info("Repairing non-manifold vertices of the mesh...");
    RepairNonManifold(m_pMesh);
    Logger::Info("<RepairNonManifold> Completed!");

    Logger::Info("Cutting the mesh...");
    if (!CutToDisk(m_pMesh) || GetMeshGenus(m_pMesh) != 0)
    {
        Logger::Error("Cannot cut the mesh into a topological disk.");
        return false;
    }
    Logger::Info("<CutToDisk> Completed!");

    TriMesh originMesh(*m_pMesh);
    if (pCutMesh != nullptr)
    {
        *pCutMesh = originMesh;
    }

    Logger::Info("Parameterizing the mesh...");
    TriMesh uvMesh(*m_pMesh);

    if (parameterization == EParameterization::EBarycentric)
    {
        BarycentricMapping(uvMesh, EBoundary::Square, EBoundaryFixWeight::Length, ECoefficient::DiscreteHarmonic);
    }
    else if (parameterization == EParameterization::ESLIM)
    {
        SLIMProxy(&uvMesh);
    }
    else
    {
        Logger::Error("Unkonwn parameterization method.");
        return false;
    }

    if (!MeshUVTransfer(m_pMesh, &uvMesh))
    {
        Logger::Error("Error occurred when parameterizing the mesh.");
        return false;
    }
    Logger::Info("<MeshParameterization> Completed!");

    Logger::Info("Integrating the texture...");
    if (!TextureIntegrate(&originMesh, m_pMesh, m_pTexture, width, height))
    {
        Logger::Error("Error occurred when integrating the texture.");
        return false;
    }
    Logger::Info("<TextureIntegrate> Completed!");

    return true;
}

bool TextureOptimizer::TextureIntegrate(const TriMesh * pOriginMesh, const TriMesh * pNewMesh, Texture * pTexture, uint32_t width, uint32_t height)
{
    if (pOriginMesh->n_vertices() != pNewMesh->n_vertices() ||
        pOriginMesh->n_edges() != pNewMesh->n_edges() ||
        pOriginMesh->n_faces() != pNewMesh->n_faces() ||
        pOriginMesh->n_halfedges() != pNewMesh->n_halfedges())
    {
        Logger::Error("When integrate texture, input meshes must have same topology.");
        return false;
    }

    uint32_t targetWidth = (width == uint32_t(-1) ? pTexture->Width() : width);
    uint32_t targetHeight = (height == uint32_t(-1) ? pTexture->Height() : height);
    uint32_t targetChannels = pTexture->Channels();

    Texture integratedTexture;
    integratedTexture.Create(targetWidth, targetHeight, targetChannels);

    /* AABB of triangle v0, v1, v2 in image, v* should be texcoord in [0, 1]^2 */
    auto GetTriangleAABBInImageFunc = [](const Vector2 & v0, const Vector2 & v1, const Vector2 & v2, uint32_t width,
            uint32_t height) -> std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> /* AABB[xMin, yMin, xMax, yMax] */
    {
        Real xMin =  std::numeric_limits<Real>::max();
        Real yMin =  std::numeric_limits<Real>::max();
        Real xMax = -std::numeric_limits<Real>::max();
        Real yMax = -std::numeric_limits<Real>::max();

        xMin = std::min(xMin, v0[0]); xMax = std::max(xMax, v0[0]);
        xMin = std::min(xMin, v1[0]); xMax = std::max(xMax, v1[0]);
        xMin = std::min(xMin, v2[0]); xMax = std::max(xMax, v2[0]);

        yMin = std::min(yMin, v0[1]); yMax = std::max(yMax, v0[1]);
        yMin = std::min(yMin, v1[1]); yMax = std::max(yMax, v1[1]);
        yMin = std::min(yMin, v2[1]); yMax = std::max(yMax, v2[1]);

        auto xMinInt = static_cast<uint32_t>(std::floor(xMin * width));
        auto yMinInt = static_cast<uint32_t>(std::floor(yMin * height));
        auto xMaxInt = static_cast<uint32_t>(std::ceil(xMax * width));
        auto yMaxInt = static_cast<uint32_t>(std::ceil(yMax * height));

        xMinInt = std::clamp(xMinInt, 0u, width - 1);
        yMinInt = std::clamp(yMinInt, 0u, height - 1);
        xMaxInt = std::clamp(xMaxInt, 0u, width - 1);
        yMaxInt = std::clamp(yMaxInt, 0u, height - 1);

        return std::make_tuple<>(xMinInt, yMinInt, xMaxInt, yMaxInt);
    };

    /* Whether v is inside triangle v0, v1, v2 */
    auto IsInsideTriangleFunc = [](const Vector2 & v, const Vector2 & v0, const Vector2 & v1, const Vector2 & v2) -> bool
    {
        Real signOfTriangle = (v1[0] - v0[0]) * (v2[1] - v0[1]) - (v1[1] - v0[1]) * (v2[0] - v0[0]);
        Real signOfAB = (v1[0] - v0[0]) * (v[1] - v0[1]) - (v1[1] - v0[1]) * (v[0] - v0[0]);
        Real signOfCA = (v0[0] - v2[0]) * (v[1] - v2[1]) - (v0[1] - v2[1]) * (v[0] - v2[0]);
        Real signOfBC = (v2[0] - v1[0]) * (v[1] - v2[1]) - (v2[1] - v1[1]) * (v[0] - v2[0]);

        bool d1 = (signOfAB * signOfTriangle >= 0);
        bool d2 = (signOfCA * signOfTriangle >= 0);
        bool d3 = (signOfBC * signOfTriangle >= 0);

        return d1 && d2 && d3;
    };

    /* affine matrix from triangle va0, va1, va2 to triangle vb0, vb1, vb2 */
    auto GetTransformFunc = [](const Vector2 & va0, const Vector2 & va1, const Vector2 & va2,
            const Vector2 & vb0, const Vector2 & vb1, const Vector2 & vb2) -> Matrix3
    {
        Matrix3 A;
        A(0, 0) = va0[0]; A(0, 1) = va0[1]; A(0, 2) = 1;
        A(1, 0) = va1[0]; A(1, 1) = va1[1]; A(1, 2) = 1;
        A(2, 0) = va2[0]; A(2, 1) = va2[1]; A(2, 2) = 1;

        Matrix3 B;
        B(0, 0) = vb0[0]; B(0, 1) = vb0[1]; B(0, 2) = 1;
        B(1, 0) = vb1[0]; B(1, 1) = vb1[1]; B(1, 2) = 1;
        B(2, 0) = vb2[0]; B(2, 1) = vb2[1]; B(2, 2) = 1;

        /* Ax=B */
        return A.inverse() * B;
    };

    /* apply affine matrix t (by left multiply) to v */
    auto ApplyTransformFunc = [](const Vector2 & v, const Matrix3 & t) -> Vector2
    {
        Eigen::Matrix<Real, 1, 3> vec(v[0], v[1], 1);
        vec *= t;
        return Vector2(vec[0], vec[1]);
    };

    const uint8_t BLANK = 0, COPIED = 1, EXPANDED = 2;
    std::vector<std::vector<uint8_t>> flag(targetHeight, std::vector<uint8_t>(targetWidth, BLANK));

    for (int i = 0; i < pOriginMesh->n_faces(); i++)
    {
        const auto fh = TriMesh::FaceHandle(i);

        auto heha0 = pOriginMesh->halfedge_handle(fh);
        auto heha1 = pOriginMesh->next_halfedge_handle(heha0);
        auto heha2 = pOriginMesh->next_halfedge_handle(heha1);

        const auto & va0 = pOriginMesh->texcoord2D(heha0);
        const auto & va1 = pOriginMesh->texcoord2D(heha1);
        const auto & va2 = pOriginMesh->texcoord2D(heha2);

        auto hehb0 = pNewMesh->halfedge_handle(fh);
        auto hehb1 = pNewMesh->next_halfedge_handle(hehb0);
        auto hehb2 = pNewMesh->next_halfedge_handle(hehb1);

        const auto & vb0 = pNewMesh->texcoord2D(hehb0);
        const auto & vb1 = pNewMesh->texcoord2D(hehb1);
        const auto & vb2 = pNewMesh->texcoord2D(hehb2);

        auto t = GetTransformFunc(vb0, vb1, vb2, va0, va1, va2);

        auto aabb = GetTriangleAABBInImageFunc(vb0, vb1, vb2, targetWidth, targetHeight);

        uint32_t yMin = std::get<1>(aabb);
        uint32_t yMax = std::get<3>(aabb);
        uint32_t xMin = std::get<0>(aabb);
        uint32_t xMax = std::get<2>(aabb);

        for (uint32_t y = yMin; y <= yMax; y++)
        {
            for (uint32_t x = xMin; x <= xMax; x++)
            {
                Vector2 pos(static_cast<Real>(x) / static_cast<Real>(targetWidth), static_cast<Real>(y) / static_cast<Real>(targetHeight));
                if (IsInsideTriangleFunc(pos, vb0, vb1, vb2))
                {
                    Vector2 originPos = ApplyTransformFunc(pos, t);

                    auto x0 = static_cast<uint32_t>(std::floor(originPos[0] * pTexture->Width()));
                    auto y0 = static_cast<uint32_t>(std::floor(originPos[1] * pTexture->Height()));
                    auto x1 = x0 + 1;
                    auto y1 = y0 + 1;

                    x0 = std::clamp(x0, 0u, pTexture->Width() - 1);
                    y0 = std::clamp(y0, 0u, pTexture->Height() - 1);
                    x1 = std::clamp(x1, 0u, pTexture->Width() - 1);
                    y1 = std::clamp(y1, 0u, pTexture->Height() - 1);

                    for (uint32_t c = 0; c < targetChannels; c++)
                    {
                        uint32_t sum = static_cast<uint32_t>(pTexture->At(x0, pTexture->Height() - 1 - y0, c)) +
                                static_cast<uint32_t>(pTexture->At(x0, pTexture->Height() - 1 - y1, c)) +
                                static_cast<uint32_t>(pTexture->At(x1, pTexture->Height() - 1 - y0, c)) +
                                static_cast<uint32_t>(pTexture->At(x1, pTexture->Height() - 1 - y1, c));

                        integratedTexture.At(x, targetHeight - 1 - y, c) = static_cast<uint8_t>(sum / 4);
                    }

                    flag[y][x] = COPIED;
                }
            }
        }
    }

    for (uint32_t y = 0; y < targetWidth; y++)
    {
        for (uint32_t x = 0; x < targetHeight; x++)
        {
            if (flag[y][x] == BLANK)
            {
                uint32_t n = 0;
                std::vector<uint32_t> sums(targetChannels, 0u);

                for (uint32_t y0 = std::max(y - 1, 0u); y0 <= y + 1; y0++)
                {
                    for (uint32_t x0 = std::max(x - 1, 0u); x0 <= x + 1; x0++)
                    {
                        if (x0 < targetWidth && y0 < targetHeight && flag[y0][x0] == COPIED)
                        {
                            n++;
                            for (uint32_t c = 0; c < targetChannels; c++)
                            {
                                sums[c] += static_cast<uint32_t>(integratedTexture.At(x0, y0, c));
                            }
                        }
                    }
                }

                if (n > 0)
                {
                    for (uint32_t c = 0; c < targetChannels; c++)
                    {
                        integratedTexture.At(x, y, c) = static_cast<uint8_t>(sums[c] / n);
                    }
                    flag[y][x] = EXPANDED;
                }
            }
        }
    }

    for (uint32_t y = 0; y < targetWidth; y++)
    {
        for (uint32_t x = 0; x < targetHeight; x++)
        {
            for (uint32_t c = 0; c < targetChannels; c++)
            {
                pTexture->At(x, y, c) = integratedTexture.At(x, y, c);
            }
        }
    }

    return true;
}

void TextureOptimizer::SLIMProxy(TriMesh * pMesh)
{
    int faceNumber = static_cast<int>(pMesh->n_faces());
    int vertexNumber =  static_cast<int>(pMesh->n_vertices());

    /* facet(i, j) : index of the j-th vertex at i-th triangle */
    std::vector<std::vector<size_t>> facets(faceNumber, std::vector<size_t>(3));

    for (int i = 0; i < faceNumber; i++)
    {
        const auto fh = TriMesh::FaceHandle(i);

        auto fvIter = pMesh->fv_range(fh).begin();
        auto fvIterEnd = pMesh->fv_range(fh).end();

        const auto vh0 = *fvIter;
        *fvIter++;
        const auto vh1 = *fvIter;
        *fvIter++;
        const auto vh2 = *fvIter;
        *fvIter++;

        facets[i][0] = vh0.idx();
        facets[i][1] = vh1.idx();
        facets[i][2] = vh2.idx();
    }

    /* vertices(i, j) : value of the j-th component of position at i-th vertex */
    std::vector<std::vector<double>> vertices(vertexNumber, std::vector<double>(3));

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        vertices[i][0] = point[0];
        vertices[i][1] = point[1];
        vertices[i][2] = point[2];
    }

    const int ITERATIONS = 20;

    bmc::progress_bar progressBar(0, ITERATIONS, 80, false, true);

    bool bStopped = false;
    int currIterations = 0;
    std::thread progressThread([](bmc::progress_bar & progress, bool & stopped, int & current)
    {
        while (!stopped)
        {
            progress.set(current);
            std::cout << progress;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }, std::ref(progressBar), std::ref(bStopped), std::ref(currIterations));

    auto uv = SLIM(vertices, facets, ITERATIONS, &currIterations);

    bStopped = true;
    progressThread.join();
    progressBar.set(ITERATIONS);
    std::cout << progressBar << std::endl;

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle(i);
        auto & point = pMesh->point(vh);
        point[0] = uv[i][0];
        point[1] = uv[i][1];
    }

    Real xMin =  std::numeric_limits<Real>::max();
    Real yMin =  std::numeric_limits<Real>::max();
    Real xMax = -std::numeric_limits<Real>::max();
    Real yMax = -std::numeric_limits<Real>::max();

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        const auto & point = pMesh->point(vh);

        xMin = std::min(xMin, point[0]);
        xMax = std::max(xMax, point[0]);

        yMin = std::min(yMin, point[1]);
        yMax = std::max(yMax, point[1]);
    }

    Real invDeltaX = 1.0 / (xMax - xMin);
    Real invDeltaY = 1.0 / (yMax - yMin);

    for (int i = 0; i < vertexNumber; i++)
    {
        const auto vh = TriMesh::VertexHandle (i);
        auto & point = pMesh->point(vh);
        point[0] = (point[0] - xMin) * invDeltaX;
        point[1] = (point[1] - yMin) * invDeltaY;
    }
}
