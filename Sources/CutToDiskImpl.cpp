#include "CutToDiskImpl.hpp"

#include <deque>
#include <map>
#include <memory>

#include "Logger.hpp"

#ifdef _DEBUG
#include "TextureOptimizer.hpp"
#endif

/* Cut the mesh use 'cuts'. */
static void CutMesh(TriMesh * pMesh, const std::vector<std::vector<int>> & cuts)
{
    /* Store the indices of the cycle's halfedge handle */
    std::vector<int> cycleHalfedgedIdxs;

    for (auto & cut : cuts)
    {
        for (int j = 1; j < cut.size(); j++)
        {
            const auto vh = TriMesh::VertexHandle(cut[j]);
            const auto vhAdjacent = TriMesh::VertexHandle(cut[j - 1]);

            const auto heh0 = pMesh->find_halfedge(vh, vhAdjacent);
            const auto heh1 = pMesh->find_halfedge(vhAdjacent, vh);

            if (heh0 != TriMesh::InvalidHalfedgeHandle)
            {
                cycleHalfedgedIdxs.push_back(heh0.idx());
            }

            if (heh1 != TriMesh::InvalidHalfedgeHandle)
            {
                cycleHalfedgedIdxs.push_back(heh1.idx());
            }
        }
    }

    /* Store the indices of the cycle's inner halfedge handle */
    std::vector<int> cycleInnerHalfedgedIdxs;

    /* We first find the start halfedge point */
    auto startHEH = TriMesh::HalfedgeHandle(cycleHalfedgedIdxs[0]);
    auto currentHEH = startHEH;

    bool bIsInnerHEH = true;

    do
    {
        if (pMesh->is_boundary(currentHEH))
        {
            bIsInnerHEH = false;
            break;
        }

        currentHEH = pMesh->next_halfedge_handle(currentHEH);

        while (std::find(cycleHalfedgedIdxs.begin(), cycleHalfedgedIdxs.end(), currentHEH.idx()) == cycleHalfedgedIdxs.end())
        {
            currentHEH = pMesh->opposite_halfedge_handle(currentHEH);
            currentHEH = pMesh->next_halfedge_handle(currentHEH);
        }

    } while (currentHEH != startHEH);

    if (!bIsInnerHEH)
    {
        startHEH = pMesh->opposite_halfedge_handle(startHEH);
    }

    currentHEH = startHEH;

    /* Then push the inner halfedges handles into the vector */
    do
    {
        cycleInnerHalfedgedIdxs.push_back(currentHEH.idx());
        currentHEH = pMesh->next_halfedge_handle(currentHEH);

        while (std::find(cycleHalfedgedIdxs.begin(), cycleHalfedgedIdxs.end(), currentHEH.idx()) == cycleHalfedgedIdxs.end())
        {
            currentHEH = pMesh->opposite_halfedge_handle(currentHEH);
            currentHEH = pMesh->next_halfedge_handle(currentHEH);
        }

    } while (currentHEH != startHEH);

    /* Key is the index of face handle and the old vertex handle in it, value
    is the index of new vertex handle. fhOldNewVhIdxPairs[Pair(FH, OldVH)] tell
    us that at face 'FH', vertex 'OldVH' need to be replaced by the search result
    (if had) */
    std::map<std::pair<int, int>, int> fhOldNewVhIdxPairs;

    /* Store the indices of updated faces' vertex handle (F0-VH0, F0-VH1,
    F0-VH2, F1-VH0, F1-VH1, F1-VH2....). We first store the old vertex
    handle and then use 'fhOldNewVhIdxPairs' to update the vector. */
    std::vector<int> newFacetsVhIdxs;

    /* Store the indices of the face handle that are to be updated*/
    std::vector<int> updatedFacetIdxs;

    for (int i = 0; i < cycleInnerHalfedgedIdxs.size(); i++)
    {
        auto heh = TriMesh::HalfedgeHandle(cycleInnerHalfedgedIdxs[i]);

        const auto oldVh = pMesh->from_vertex_handle(heh);
        const auto oldPoint = pMesh->point(oldVh);

        const auto newVH = pMesh->add_vertex(oldPoint);

        if (pMesh->has_vertex_texcoords2D())
        {
            const auto oldTexcoord = pMesh->texcoord2D(oldVh);
            pMesh->set_texcoord2D(newVH, oldTexcoord);
        }

        heh = pMesh->prev_halfedge_handle(heh);

        do
        {
            auto fh = pMesh->face_handle(heh);
            auto fvIter = pMesh->fv_range(fh).begin();
            auto fvIterEnd = pMesh->fv_range(fh).end();

            auto vh0 = *fvIter; fvIter++;
            auto vh1 = *fvIter; fvIter++;
            auto vh2 = *fvIter; fvIter++;

#ifdef _DEBUG
            if (fvIter != fvIterEnd)
            {
                throw std::runtime_error("Non triangle facet found while cutting the mesh to disk.");
            }
#endif
            fhOldNewVhIdxPairs.insert({{fh.idx(), oldVh.idx() }, newVH.idx() });
            newFacetsVhIdxs.insert(newFacetsVhIdxs.end(), {vh0.idx(), vh1.idx(), vh2.idx() });
            updatedFacetIdxs.push_back(fh.idx());

            const auto hehOpposite = pMesh->opposite_halfedge_handle(heh);
            if (std::find(cycleInnerHalfedgedIdxs.begin(), cycleInnerHalfedgedIdxs.end(), hehOpposite.idx()) != cycleInnerHalfedgedIdxs.end())
            {
                break;
            }

            heh = pMesh->prev_halfedge_handle(hehOpposite);
            if (std::find(cycleInnerHalfedgedIdxs.begin(), cycleInnerHalfedgedIdxs.end(), heh.idx()) != cycleInnerHalfedgedIdxs.end())
            {
                break;
            }

        } while (!pMesh->is_boundary(heh));

        if (!pMesh->is_boundary(heh))
        {
            fhOldNewVhIdxPairs.insert({{pMesh->face_handle(heh).idx(), oldVh.idx() }, newVH.idx() });
        }
    }

    /* Using 'fhOldNewVhIdxPairs' and 'updatedFacetIdxs' to update 'newFacetsVhIdxs' */
    for (int i = 0; i < updatedFacetIdxs.size(); i++)
    {
        auto fhIdx = updatedFacetIdxs[i];

        auto & vh0Idx = newFacetsVhIdxs[i * 3 + 0];
        auto & vh1Idx = newFacetsVhIdxs[i * 3 + 1];
        auto & vh2Idx = newFacetsVhIdxs[i * 3 + 2];

        auto findIter0 = fhOldNewVhIdxPairs.find({fhIdx, vh0Idx });
        auto findIter1 = fhOldNewVhIdxPairs.find({fhIdx, vh1Idx });
        auto findIter2 = fhOldNewVhIdxPairs.find({fhIdx, vh2Idx });

        if (findIter0 != fhOldNewVhIdxPairs.end()) { vh0Idx = findIter0->second; }
        if (findIter1 != fhOldNewVhIdxPairs.end()) { vh1Idx = findIter1->second; }
        if (findIter2 != fhOldNewVhIdxPairs.end()) { vh2Idx = findIter2->second; }
    }

    /* Create the new mesh. */
    TriMesh newMesh;
    std::vector<TriMesh::VertexHandle> newMeshVHs;
    std::vector<std::vector<TriMesh::VertexHandle>> newMeshFacetsVHs;

    if (pMesh->has_vertex_texcoords2D())
    {
        newMesh.request_vertex_texcoords2D();
    }

    if (pMesh->has_halfedge_texcoords2D())
    {
        newMesh.request_halfedge_texcoords2D();
    }

    /* Move the vertex handle to the new mesh */
    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        const auto oriVH = TriMesh::VertexHandle(i);
        const auto point = pMesh->point(oriVH);
        const auto newVH = newMesh.add_vertex(point);

        if (pMesh->has_vertex_texcoords2D())
        {
            const auto & texcoord = pMesh->texcoord2D(oriVH);
            newMesh.set_texcoord2D(newVH, texcoord);
        }

        newMeshVHs.push_back(newVH);
    }

    /* If the face need to be updated, we use the vertex handle store in 'newFacetsVhIdxs',
    otherwise we just use the old vertex handle */
    for (int i = 0; i < pMesh->n_faces(); i++)
    {
        const auto originFH = TriMesh::FaceHandle(i);
        const auto iter = std::find(updatedFacetIdxs.begin(), updatedFacetIdxs.end(), originFH.idx());

        TriMesh::VertexHandle vh0, vh1, vh2;

        if (iter != updatedFacetIdxs.end())
        {
            const auto idx = iter - updatedFacetIdxs.begin();
            vh0 = TriMesh::VertexHandle(newFacetsVhIdxs[idx * 3 + 0]);
            vh1 = TriMesh::VertexHandle(newFacetsVhIdxs[idx * 3 + 1]);
            vh2 = TriMesh::VertexHandle(newFacetsVhIdxs[idx * 3 + 2]);
        }
        else
        {
            auto fvIter = pMesh->fv_range(originFH).begin();
            auto fvIterEnd = pMesh->fv_range(originFH).end();

            vh0 = *fvIter; fvIter++;
            vh1 = *fvIter; fvIter++;
            vh2 = *fvIter; fvIter++;
        }

        std::vector<TriMesh::VertexHandle> newMeshFacetVHs = {vh0, vh1, vh2 };
        const auto newFH = newMesh.add_face(newMeshFacetVHs);
        newMeshFacetsVHs.push_back(newMeshFacetVHs);

        if (pMesh->has_halfedge_texcoords2D())
        {
            for (auto originHEH : pMesh->fh_range(originFH))
            {
                auto originToVH = pMesh->to_vertex_handle(originHEH);

                for (auto newHEH : newMesh.fh_range(newFH))
                {
                    auto newToVH = newMesh.to_vertex_handle(newHEH);

                    auto newToPoint = newMesh.point(newToVH);
                    auto originToPoint = pMesh->point(originToVH);

                    if (newToPoint == originToPoint)
                    {
                        const auto & texcoord = pMesh->texcoord2D(originHEH);
                        newMesh.set_texcoord2D(newHEH, texcoord);
                        break;
                    }
                }
            }
        }
    }

    /* Delete the useless vertex. */
    *pMesh = newMesh;
    pMesh->request_vertex_status();
    pMesh->delete_isolated_vertices();
    pMesh->garbage_collection();
    pMesh->release_vertex_status();
}

void CutToDiskImpl(TriMesh * pMesh)
{
    int faceNumber = static_cast<int>(pMesh->n_faces());
    int edgeNumber = static_cast<int>(pMesh->n_edges());
    int vertexNumber =  static_cast<int>(pMesh->n_vertices());

    if (faceNumber == 0 || edgeNumber == 0 || vertexNumber == 0)
    {
        Logger::Warn("You tried to cut an empty mesh to disk, nothing happen.");
        return;
    }

    /* facet(i, j) : index of the j-th vertex at i-th triangle */
    Eigen::MatrixXi facets(faceNumber, 3);

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

#ifdef _DEBUG
        if (fvIter != fvIterEnd)
        {
            throw std::runtime_error("Non triangle facet found while cutting the mesh to disk.");
        }
#endif

        facets(i, 0) = vh0.idx();
        facets(i, 1) = vh1.idx();
        facets(i, 2) = vh2.idx();
    }

    /* Key is the indices of edge's endpoints, value is the facet indices
    that the edge adjacent to. (Index is with respect to 'facets')*/
    std::map<std::pair<int, int>, std::vector<int>> edges;

    for (int i = 0; i < faceNumber; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            int v0Idx = facets(i, j);
            int v1Idx = facets(i, (j + 1) % 3);

            std::pair<int, int> edge;
            edge.first = std::min(v0Idx, v1Idx);
            edge.second = std::max(v0Idx, v1Idx);

            edges[edge].push_back(i);
        }
    }

#ifdef _DEBUG
    if (edges.size() != edgeNumber)
    {
        throw std::runtime_error("Size of the map \"edges\" is not equal to the mesh's edge number.");
    }
#endif

    /* edgeVertices(i, j) : index of the j-th vertex at i-th edge in 'edges' */
    Eigen::MatrixXi edgeVertices(edgeNumber, 2);
    /* edgeFaces(i, j) : index of the j-th facet at i-th edge in 'edges' */
    Eigen::MatrixXi edgeFaces(edgeNumber, 2);
    /* faceEdges(i, j) : index of the j-th edge at i-th facet in 'facets' */
    Eigen::MatrixXi faceEdges(faceNumber, 3);
    /* Index of the boundary edges in 'edges' */
    std::set<int> boundaryEdges;
    /* Key is the indices of edge's endpoints in 'facet', value is the indices of the edge in 'edge'. */
    std::map<std::pair<int, int>, int> edgeIdxs;

    {
        /* Use scope here to avoid name conflict. */
        int edgeIdx = 0;
        for (auto edge : edges)
        {
            edgeIdxs[edge.first] = edgeIdx;

            edgeVertices(edgeIdx, 0) = edge.first.first;
            edgeVertices(edgeIdx, 1) = edge.first.second;

            edgeFaces(edgeIdx, 0) = edge.second[0];
            if (edge.second.size() > 1)
            {
                edgeFaces(edgeIdx, 1) = edge.second[1];
            }
            else
            {
                edgeFaces(edgeIdx, 1) = -1;
                boundaryEdges.insert(edgeIdx);
            }

            edgeIdx++;
        }
    }

    for (int i = 0; i < faceNumber; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            int v0Idx = facets(i, j);
            int v1Idx = facets(i, (j + 1) % 3);

            std::pair<int, int> edge;
            edge.first = std::min(v0Idx, v1Idx);
            edge.second = std::max(v0Idx, v1Idx);

            faceEdges(i, j) = edgeIdxs[edge];
        }
    }

#ifdef _DEBUG
    if (edgeIdxs.size() != edgeNumber)
    {
        throw std::runtime_error("Size of the map \"edgeIdxs\" is not equal to the mesh's edge number.");
    }

    {
        const auto boundaries = TextureOptimizer::MeshBoundaryVertices(pMesh);
        size_t boundaryEdgeNumber = 0;
        for (const auto boundary : boundaries)
        {
            /* boundary is a loop, so edge number is equal to the vertex number.  */
            boundaryEdgeNumber += boundary.size();
        }

        if (boundaryEdges.size() != boundaryEdgeNumber)
        {
            throw std::runtime_error("Size of the map \"boundaryEdges\" is not equal to the mesh's boundary edge number.");
        }
    }
#endif

    std::unique_ptr<bool[]> deletedTags(new bool[faceNumber]);
    for (int i = 0; i < faceNumber; i++)
    {
        deletedTags[i] = false;
    }

    std::set<int> deletedEdges/* Idx in 'edges' */;
    /* Loop over all faces, flooding on the mesh */
    for (int faceIdx = 0; faceIdx < faceNumber; faceIdx++)
    {
        /* Stop at first undeleted face */
        if (deletedTags[faceIdx])
        {
            continue;
        }

        deletedTags[faceIdx] = true;

        std::deque<int> processEdges;
        for (int i = 0; i < 3; i++)
        {
            int edgeIdx = faceEdges(faceIdx, i);

            if (boundaryEdges.count(edgeIdx) > 0)
            {
                continue;
            }

            int deletedNumber = 0;
            if (deletedTags[edgeFaces(edgeIdx, 0)])
            {
                deletedNumber++;
            }
            if (deletedTags[edgeFaces(edgeIdx, 1)])
            {
                deletedNumber++;
            }

            if (deletedNumber == 1)
            {
                processEdges.push_back(edgeIdx);
            }
        }

        /* Delete all faces adjacent to edges with exactly one adjacent face */
        while (!processEdges.empty())
        {
            int nextEdgeIdx = processEdges.front();
            processEdges.pop_front();

            int toDeleteFaceIdx = faceNumber;
            if (!deletedTags[edgeFaces(nextEdgeIdx, 0)])
            {
                toDeleteFaceIdx = edgeFaces(nextEdgeIdx, 0);
            }
            if (!deletedTags[edgeFaces(nextEdgeIdx, 1)])
            {
                toDeleteFaceIdx = edgeFaces(nextEdgeIdx, 1);
            }

            if (toDeleteFaceIdx != faceNumber)
            {
                deletedEdges.insert(nextEdgeIdx);
                deletedTags[toDeleteFaceIdx] = true;

                for (int i = 0; i < 3; i++)
                {
                    int edgeIdx = faceEdges(toDeleteFaceIdx, i);

                    if (boundaryEdges.count(edgeIdx) > 0)
                    {
                        continue;
                    }

                    int deletedNumber = 0;
                    if (deletedTags[edgeFaces(edgeIdx, 0)])
                    {
                        deletedNumber++;
                    }
                    if (deletedTags[edgeFaces(edgeIdx, 1)])
                    {
                        deletedNumber++;
                    }

                    if (deletedNumber == 1)
                    {
                        processEdges.push_back(edgeIdx);
                    }
                }
            }
        }
    }

    /* Find the non-deleted edges */
    std::vector<int> nonDeletedEdges/* Idx in 'edges' */;
    for (int i = 0; i < edgeNumber; i++)
    {
        if (deletedEdges.count(i) == 0)
        {
            nonDeletedEdges.push_back(i);
        }
    }

    /* Prune spines (The non-deleted edges now looks like a loop spine) */

    /* Key is the index of non-deleted edges' vertex in 'facets', value is
       the non-deleted edge's indics in 'edges'. */
    std::map<int, std::vector<int>> spineVertexEdges;
    for (int nonDeletedEdgeIdx : nonDeletedEdges)
    {
        spineVertexEdges[edgeVertices(nonDeletedEdgeIdx, 0)].push_back(nonDeletedEdgeIdx);
        spineVertexEdges[edgeVertices(nonDeletedEdgeIdx, 1)].push_back(nonDeletedEdgeIdx);
    }

    /* Store the index of the vertices that are on the branches of the spines */
    std::deque<int> processVertices;

    /* Key is the index of non-deleted edges' vertex in 'facets', value is the number
    of the non-deleted edge adjacent to this vertex. */
    std::map<int, int> spineVertexNumbers;
    for (const auto & spineVertexEdge : spineVertexEdges)
    {
        spineVertexNumbers[spineVertexEdge.first] = int(spineVertexEdge.second.size());
        if (spineVertexEdge.second.size() == 1)
        {
            processVertices.push_back(spineVertexEdge.first);
        }
    }

    /* Store the index of the edges that are on the branches of the spines. */
    std::set<int> branchEdges/* Idx in 'edges' */;

    /* Remove the vertices that are on the branches of the spines. */
    while (!processVertices.empty())
    {
        int vertexIdx = processVertices.front();
        processVertices.pop_front();

        for (int edgeIdx : spineVertexEdges[vertexIdx])
        {
            if (branchEdges.count(edgeIdx) == 0)
            {
                branchEdges.insert(edgeIdx);

                for (int j = 0; j < 2; j++)
                {
                    spineVertexNumbers[edgeVertices(edgeIdx, j)]--;

                    if (spineVertexNumbers[edgeVertices(edgeIdx, j)] == 1)
                    {
                        processVertices.push_back(edgeVertices(edgeIdx, j));
                    }
                }
            }
        }
    }

    std::vector<int> loopEdges/* Idx in 'edges' */;
    for (int edgeIdx : nonDeletedEdges)
    {
        /* edges that are in the 'nonDeletedEdges' and not in the 'branchEdges',
        i.e. edges that are on the spine but not on branches of the spine. */
        if (branchEdges.count(edgeIdx) == 0)
        {
            loopEdges.push_back(edgeIdx);
        }
    }

    int loopEdgeNumber = static_cast<int>(loopEdges.size());
    if (loopEdgeNumber == 0)
    {
        Logger::Warn("The number of the edges that are on boundary is zero.");
        return;
    }

    /* Key is the index of boundary edges' vertex in 'facets', value is the boundary
    edge's indics in 'edges'. */
    std::map<int, std::vector<int>> loopVertexEdges;
    for (int edgeIdx : loopEdges)
    {
        loopVertexEdges[edgeVertices(edgeIdx, 0)].push_back(edgeIdx);
        loopVertexEdges[edgeVertices(edgeIdx, 1)].push_back(edgeIdx);
    }

    /* List of cuts. Each cut is a sequence of vertex indices (where
    pairs of consecutive vertices share a face), is simple, and is either
    a closed loop (in which the first and last indices are identical) or
    an open curve. cuts are edge-disjoint. */
    std::vector<std::vector<int>> cuts;

    std::set<int> usedEdges/* Idx in 'edges' */;
    for (int loopEdgeIdx : loopEdges)
    {
        /* Make a cycle or chain starting from this edge. */
        while (usedEdges.count(loopEdgeIdx) == 0)
        {
            std::vector<int> cycleVertices/* Idx in 'facets' */;
            std::vector<int> cycleEdges/* Idx in 'edges' */;

            cycleVertices.push_back(edgeVertices(loopEdgeIdx, 0));
            cycleVertices.push_back(edgeVertices(loopEdgeIdx, 1));

            cycleEdges.push_back(loopEdgeIdx);

            /* Key is the index of the vertex in 'facets', value is
            the sequences number of the vertex in the cycle. */
            std::map<int, int> cycleIdx;
            cycleIdx[cycleVertices[0]] = 0;
            cycleIdx[cycleVertices[1]] = 1;

            int currentVertexIdx = edgeVertices(loopEdgeIdx, 1);
            int currentEdgeIdx = loopEdgeIdx;
            bool bFoundCycle = false;

            while (currentVertexIdx != -1 && !bFoundCycle)
            {
                int nextVertexIdx = -1;
                int nextEdgeIdx = -1;

                for (int loopVertexEdgeIdx : loopVertexEdges[currentVertexIdx])
                {
                    if (usedEdges.count(loopVertexEdgeIdx) == 0 && loopVertexEdgeIdx != currentEdgeIdx)
                    {
                        int vertexLocalIdx = 0;
                        if (currentVertexIdx == edgeVertices(loopVertexEdgeIdx, vertexLocalIdx))
                        {
                            vertexLocalIdx = 1;
                        }

                        nextVertexIdx = edgeVertices(loopVertexEdgeIdx, vertexLocalIdx);
                        nextEdgeIdx = loopVertexEdgeIdx;
                        break;
                    }
                }

                if (nextVertexIdx != -1)
                {
                    auto kvIter = cycleIdx.find(nextVertexIdx);
                    if (kvIter != cycleIdx.end())
                    {
                        /* We have hit ourselves */
                        std::vector<int> cut;
                        for (int i = kvIter->second; i < cycleVertices.size(); i++)
                        {
                            cut.push_back(cycleVertices[i]);
                        }
                        cut.push_back(nextVertexIdx);
                        cuts.push_back(cut);

                        for (int i = kvIter->second; i < cycleEdges.size(); i++)
                        {
                            usedEdges.insert(cycleEdges[i]);
                        }
                        usedEdges.insert(nextEdgeIdx);
                        bFoundCycle = true;
                    }
                    else
                    {
                        cycleIdx[nextVertexIdx] = int(cycleVertices.size());
                        cycleVertices.push_back(nextVertexIdx);
                        cycleEdges.push_back(nextEdgeIdx);
                    }
                }

                currentVertexIdx = nextVertexIdx;
                currentEdgeIdx = nextEdgeIdx;
            }

            if (!bFoundCycle)
            {
                /* We have hit a dead end, reverse and try another direction */
                std::reverse(cycleVertices.begin(), cycleVertices.end());
                std::reverse(cycleEdges.begin(), cycleEdges.end());

                currentVertexIdx = cycleVertices.back();
                currentEdgeIdx = cycleEdges.back();

                while (currentVertexIdx != -1 && !bFoundCycle)
                {
                    int nextVertexIdx = -1;
                    int nextEdgeIdx = -1;

                    for (int loopVertexEdgeIdx : loopVertexEdges[currentVertexIdx])
                    {
                        if (usedEdges.count(loopVertexEdgeIdx) == 0 && loopVertexEdgeIdx != currentEdgeIdx)
                        {
                            int vertexLocalIdx = 0;
                            if (currentVertexIdx == edgeVertices(loopVertexEdgeIdx, vertexLocalIdx))
                            {
                                vertexLocalIdx = 1;
                            }

                            nextVertexIdx = edgeVertices(loopVertexEdgeIdx, vertexLocalIdx);
                            nextEdgeIdx = loopVertexEdgeIdx;
                            break;
                        }
                    }

                    if (nextVertexIdx != -1)
                    {
                        auto kvIter = cycleIdx.find(nextVertexIdx);
                        if (kvIter != cycleIdx.end())
                        {
                            /* We have hit ourselves */
                            std::vector<int> cut;
                            for (int i = kvIter->second; i < cycleVertices.size(); i++)
                            {
                                cut.push_back(cycleVertices[i]);
                            }
                            cut.push_back(nextVertexIdx);
                            cuts.push_back(cut);

                            for (int i = kvIter->second; i < cycleEdges.size(); i++)
                            {
                                usedEdges.insert(cycleEdges[i]);
                            }
                            usedEdges.insert(nextEdgeIdx);
                            bFoundCycle = true;
                        }
                        else
                        {
                            cycleIdx[nextVertexIdx] = int(cycleVertices.size());
                            cycleVertices.push_back(nextVertexIdx);
                            cycleEdges.push_back(nextEdgeIdx);
                        }
                    }

                    currentVertexIdx = nextVertexIdx;
                    currentEdgeIdx = nextEdgeIdx;
                }

                if (!bFoundCycle)
                {
                    /* We have found a chain */
                    std::vector<int> cut;
                    cut.reserve(cycleVertices.size());
                    for (int cycleVertice : cycleVertices)
                    {
                        cut.push_back(cycleVertice);
                    }
                    cuts.push_back(cut);
                    for (int & cycleEdge : cycleEdges)
                    {
                        usedEdges.insert(cycleEdge);
                    }
                }
            }
        }
    }

    CutMesh(pMesh, cuts);
}