#include "TextureOptimizer.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <thread>
#include <chrono>

#include <progress_bar.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

#include "Logger.hpp"
#include "CutToDiskImpl.hpp"
#include "BarycentricParameterization.hpp"

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

bool TextureOptimizer::Run(uint32_t width, uint32_t height)
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

    Logger::Info("Parameterizing the mesh...");
    TriMesh uvMesh(*m_pMesh);
    BarycentricMapping(uvMesh, EBoundary::Square, EBoundaryFixWeight::Length, ECoefficient::DiscreteHarmonic);
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

std::vector<std::vector<OpenMesh::VertexHandle>> TextureOptimizer::MeshComponentVertices(const TriMesh * pMesh)
{
    TriMesh mesh(*pMesh);
    std::vector<std::vector<OpenMesh::VertexHandle>> componentVertices;

    OpenMesh::VPropHandleT<bool> visitedVProp;
    mesh.add_property(visitedVProp);
    for (auto vh : mesh.vertices())
    {
        mesh.property(visitedVProp, vh) = false;
    }

    auto vertexIter = mesh.vertices_begin();

    while (true)
    {
        /* find an unvisited vertex */
        bool bFound = false;
        OpenMesh::VertexHandle foundVH;
        for (auto iter = vertexIter; iter != mesh.vertices_end(); ++iter)
        {
            if (!mesh.property(visitedVProp, *iter))
            {
                bFound = true;
                foundVH = *iter;
                mesh.property(visitedVProp, *iter) = true;
                vertexIter = iter;
                break;
            }
        }

        /* if not found, then finished */
        if (!bFound) { break; }

        std::vector<OpenMesh::VertexHandle> handles;
        std::vector<OpenMesh::VertexHandle> component;
        handles.push_back(foundVH);

        /* grow from found vertex */
        while (!handles.empty())
        {
            auto currentVH = handles.back();
            handles.pop_back();
            component.push_back(currentVH);

            for (auto iter = mesh.vv_iter(currentVH); iter.is_valid(); ++iter)
            {
                if (!mesh.property(visitedVProp, *iter))
                {
                    mesh.property(visitedVProp, *iter) = true;
                    handles.push_back(*iter);
                }
            }
        }

        componentVertices.push_back(component);
    }

    return componentVertices;
}

std::vector<size_t> TextureOptimizer::MeshComponentVertexNums(const TriMesh * pMesh)
{
    auto compVertices = MeshComponentVertices(pMesh);
    std::vector<size_t> vertexNums;
    vertexNums.reserve(compVertices.size());
    for (const auto & comp : compVertices)
    {
        vertexNums.push_back(comp.size());
    }
    return vertexNums;
}

size_t TextureOptimizer::MeshComponentCount(const TriMesh * pMesh)
{
    return MeshComponentVertices(pMesh).size();
}

std::vector<std::vector<OpenMesh::VertexHandle>> TextureOptimizer::MeshBoundaryVertices(const TriMesh *pMesh)
{
    TriMesh mesh(*pMesh);
    std::vector<std::vector<OpenMesh::VertexHandle>> boundaryVertices;

    OpenMesh::HPropHandleT<bool> visitedHProp;
    mesh.add_property(visitedHProp);
    for (auto heh : mesh.halfedges())
    {
        mesh.property(visitedHProp, heh) = false;
    }

    for (auto heh : mesh.halfedges())
    {
        if (mesh.property(visitedHProp, heh)) { continue; }
        if (!mesh.is_boundary(heh)) { continue; }

        std::vector<OpenMesh::VertexHandle> boundary;
        boundary.push_back(mesh.from_vertex_handle(heh));
        mesh.property(visitedHProp, heh) = true;

        auto nextHeh = mesh.next_halfedge_handle(heh);
        while (mesh.is_boundary(nextHeh) && !mesh.property(visitedHProp, nextHeh))
        {
            boundary.push_back(mesh.from_vertex_handle(nextHeh));
            mesh.property(visitedHProp, nextHeh) = true;
            nextHeh = mesh.next_halfedge_handle(nextHeh);
        }
        boundaryVertices.push_back(boundary);
    }

    return boundaryVertices;
}

std::vector<size_t> TextureOptimizer::MeshBoundaryLengths(const TriMesh * pMesh)
{
    auto boundaryVertices = MeshBoundaryVertices(pMesh);
    std::vector<size_t> boundaryLengths;
    boundaryLengths.reserve(boundaryVertices.size());
    for (const auto & boundary : boundaryVertices)
    {
        boundaryLengths.push_back(boundary.size());
    }
    return boundaryLengths;
}

size_t TextureOptimizer::MeshBoundaryCount(const TriMesh * pMesh)
{
    return MeshBoundaryLengths(pMesh).size();
}

void TextureOptimizer::RemoveSmallComponent(TriMesh * pMesh)
{
    auto componentVHs = MeshComponentVertices(pMesh);

    Logger::Info("%zd unique components found!", componentVHs.size());
    for (size_t i = 0; i < componentVHs.size(); i++)
    {
        Logger::Info("Component %zd: %zd vertices.", i, componentVHs[i].size());
    }

    if (componentVHs.size() <= 1) { return; }

    size_t maxComponentIdx(-1);
    size_t maxComponentVertexNum(0);
    for (size_t i = 0; i < componentVHs.size(); i++)
    {
        if (componentVHs[i].size() > maxComponentVertexNum)
        {
            maxComponentVertexNum = componentVHs[i].size();
            maxComponentIdx = i;
        }
    }

    pMesh->request_face_status();
    pMesh->request_edge_status();
    pMesh->request_vertex_status();

    for (size_t i = 0; i < componentVHs.size(); i++)
    {
        if (i == maxComponentIdx) { continue; }
        for (auto componentVH : componentVHs[i])
        {
            pMesh->delete_vertex(componentVH);
        }
    }

    pMesh->garbage_collection();
    pMesh->release_face_status();
    pMesh->release_edge_status();
    pMesh->release_vertex_status();

    Logger::Info("Component %zd (%zd vertices) was reserved, others were removed.", maxComponentIdx, maxComponentVertexNum);
}

void TextureOptimizer::RepairNonManifold(TriMesh * pMesh)
{
    if (pMesh == nullptr) { return; }

    std::vector<OpenMesh::VertexHandle> nonManifoldVHs;
    for (auto vh : pMesh->vertices())
    {
        if (!pMesh->is_manifold(vh))
        {
            nonManifoldVHs.push_back(vh);
        }
    }

    Logger::Info("%zd non-manifold vertices found!", nonManifoldVHs.size());

    if (nonManifoldVHs.empty()) { return; }

    uint32_t newVertexNum = 0;

    OpenMesh::FPropHandleT<size_t> componentFProp;
    pMesh->add_property(componentFProp);

    for (auto vh : pMesh->vertices())
    {
        /* unmark all faces around the vertices */
        for (auto fh : pMesh->vf_range(vh))
        {
            pMesh->property(componentFProp, fh) = 0;
        }

        size_t componentCount = 1;

        /* search and isolate new components, then shared vertices will be duplicated */
        for (auto fh : pMesh->vf_range(vh))
        {
            /* find the first face in the component */
            std::vector<OpenMesh::FaceHandle> checkedNeighbour;
            if (pMesh->property(componentFProp, fh) == 0)
            {
                pMesh->property(componentFProp, fh) = componentCount;
                checkedNeighbour.push_back(fh);
            }

            /* if a reference face was found, a new component exists and
               a new vertex is required (except for the 1st component) */
            OpenMesh::VertexHandle newVH;
            if (componentCount > 1 && !checkedNeighbour.empty())
            {
                TriMesh::Point pt = pMesh->point(vh);
                newVH = pMesh->add_vertex(pt);
                newVertexNum++;
            }

            /* check all adjacent faces of the reference */
            while (!checkedNeighbour.empty())
            {
                OpenMesh::FaceHandle currFH = checkedNeighbour.back();
                checkedNeighbour.pop_back();

                /* get all neighbour faces of the current face */
                std::vector<OpenMesh::VertexHandle> adjVHs;
                for (auto adjVH : pMesh->fv_range(currFH))
                {
                    adjVHs.push_back(adjVH);
                    if (adjVH != vh)
                    {
                        /* find the next neighbour face over edge <adjVH-vh> */
                        OpenMesh::FaceHandle neighbourFH;
                        for (auto neighbourFHIter = pMesh->vf_begin(vh);
                            neighbourFHIter.is_valid() && !neighbourFH.is_valid();
                            ++neighbourFHIter)
                        {
                            if (*neighbourFHIter != currFH)
                            {
                                for (auto neighbourVHIter = pMesh->fv_begin(*neighbourFHIter);
                                     neighbourVHIter.is_valid() && !neighbourFH.is_valid();
                                     ++neighbourVHIter)
                                {
                                    if (*neighbourVHIter == adjVH)
                                    {
                                        neighbourFH = *neighbourFHIter;
                                    }
                                }
                            }
                        }

                        /* if such face was found, it is in the same component as the reference face */
                        if (neighbourFH.is_valid() && pMesh->property(componentFProp, neighbourFH) == 0)
                        {
                            pMesh->property(componentFProp, neighbourFH) = componentCount;
                            checkedNeighbour.push_back(neighbourFH);
                        }
                    }
                }

                /* if one face was not found in the 1st component, then it is a new component */
                if (componentCount > 1 && newVH.is_valid())
                {
                    std::replace(adjVHs.begin(), adjVHs.end(), OpenMesh::VertexHandle(vh), newVH);
                    pMesh->delete_face(currFH, false);
                    pMesh->add_face(adjVHs);
                }
            }

            /* all faces which belong to 'vh' and inside the same component found
               the next face will be in a new component */
            componentCount++;
        }
    }

    pMesh->remove_property(componentFProp);
    pMesh->garbage_collection();
    Logger::Info("Non-manifold fixed, %u new vertices created!", newVertexNum);
}

bool TextureOptimizer::CutToDisk(TriMesh * pMesh)
{
    const size_t SEED_FACE_NUM = 100;
    size_t faceIdxIncrement = pMesh->n_faces() / SEED_FACE_NUM;

    std::vector<size_t> seedIdxs;
    for (size_t seedIdx = 0; seedIdx < pMesh->n_faces(); seedIdx += faceIdxIncrement)
    {
        seedIdxs.push_back(seedIdx);
    }

    bmc::progress_bar progressBar(0, seedIdxs.size(), 80, false, true);
    std::mutex mutex;

    auto KernelFunc = [&seedIdxs, &pMesh, &mutex, &progressBar](tbb::blocked_range<size_t> & range, std::pair<size_t, size_t> value) -> std::pair<size_t, size_t>
    {
        for (size_t i = range.begin(); i != range.end(); i++)
        {
            mutex.lock();
            progressBar++;
            mutex.unlock();

            TriMesh testMesh(*pMesh);
            CutToDiskImpl(&testMesh, seedIdxs[i]);

            auto boundaries = MeshBoundaryLengths(&testMesh);
            if (boundaries.size() != 1) { continue; }

            if (boundaries[0] < value.second)
            {
                value.first = seedIdxs[i];
                value.second = boundaries[0];
            }
        }

        return value;
    };

    auto ReduceFunc = [](std::pair<size_t, size_t> a, std::pair<size_t, size_t> b) -> std::pair<size_t, size_t>
    {
        return a.second < b.second ? a : b;
    };

    bool bStopped = false;
    std::thread progressThread([](bmc::progress_bar & progress, bool & stopped)
    {
        while (!stopped)
        {
            std::cout << progress;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }, std::ref(progressBar), std::ref(bStopped));

    tbb::blocked_range<size_t> range(0, seedIdxs.size());
    auto result = tbb::parallel_reduce(range, std::make_pair(size_t(-1), std::numeric_limits<size_t>::max()), KernelFunc, ReduceFunc);
    bStopped = true;
    progressThread.join();
    std::cout << progressBar << std::endl;

    if (result.first == size_t(-1) || result.second == std::numeric_limits<size_t>::max())
    {
        return false;
    }

    CutToDiskImpl(pMesh, result.first);
    return true;
}

size_t TextureOptimizer::GetMeshEuler(const TriMesh * pMesh)
{
    return pMesh->n_vertices() - pMesh->n_edges() + pMesh->n_faces();
}

size_t TextureOptimizer::GetMeshGenus(const TriMesh * pMesh)
{
    size_t euler = GetMeshEuler(pMesh);
    return (2 - euler - MeshBoundaryCount(pMesh)) / 2;
}

bool TextureOptimizer::MeshUVTransfer(TriMesh * pMesh, const TriMesh * pUVMesh)
{
    /* Try best to ensure Mesh and UVMesh have the same topology. */
    if (pMesh->n_vertices() != pUVMesh->n_vertices() ||
        pMesh->n_edges() != pUVMesh->n_edges() ||
        pMesh->n_faces() != pUVMesh->n_faces() ||
        pMesh->n_halfedges() != pUVMesh->n_halfedges())
    {
        Logger::Error("When transfer UV, input meshes must have same topology.");
        return false;
    }

    auto KernelFunc = [&](tbb::blocked_range<size_t> & range)
    {
        for (size_t i = range.begin(); i < range.end(); i++)
        {
            const auto heh = TriMesh::HalfedgeHandle(int(i));
            const auto vh = pUVMesh->to_vertex_handle(heh);
            const auto & point = pUVMesh->point(vh);
            TriMesh::TexCoord2D uv { point[0], point[1] };
            pMesh->set_texcoord2D(heh, uv);
        }
    };

    tbb::blocked_range<size_t> range(0, pMesh->n_halfedges());
    tbb::parallel_for(range, KernelFunc);

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
