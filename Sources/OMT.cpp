#include "OMT.hpp"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_adaptation_traits_2.h>
#include <CGAL/Regular_triangulation_adaptation_policies_2.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "Logger.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using RegularTriangulation = CGAL::Regular_triangulation_2<Kernel>;
using AdaptationTraits = CGAL::Regular_triangulation_adaptation_traits_2<RegularTriangulation>;
using DegeneracyRemovalPolicy = CGAL::Regular_triangulation_degeneracy_removal_policy_2<RegularTriangulation>;
using VoronoiDiagram = CGAL::Voronoi_diagram_2<RegularTriangulation, AdaptationTraits, DegeneracyRemovalPolicy>;

bool OMT2D::Initialize()
{
    if (!CheckInitParamterization(m_pMesh))
    {
        Logger::Error("Illegal initial parameterization!");
        return false;
    }

    NormalizeUVToUnitSquare(m_pMesh);
    InitializeWeight(m_pMesh,m_Weights);
    ComputeTargetArea(m_pMesh,m_TargetAreas);

    if (!ComputePowerDiagram(m_pMesh, m_Weights, m_TargetAreas, m_CurrentAreas, m_Gradients,m_HessianWeightMap))
    {
        Logger::Error("Missing point found when computing power diagram!");
        return false;
    }

    Logger::Info("Initial Error = %.16lf", ComputeError(m_Gradients));

    return true;
}

void OMT2D::Run(int iterations, Real step)
{
    for (int iter = 0; iter < iterations; iter++)
    {
        Real currentStep = step;
        ComputeUpdateVectorGD(m_Gradients, m_UpdateVectors);

        while (true)
        {
            UpdateWeight(m_UpdateVectors, currentStep, m_Weights);
            bool bSuccess = ComputePowerDiagram(m_pMesh, m_Weights, m_TargetAreas, m_CurrentAreas, m_Gradients,m_HessianWeightMap);

            if (!bSuccess)
            {
                UpdateWeight(m_UpdateVectors, -currentStep, m_Weights);
                currentStep *= 0.5;
                Logger::Info("Rollback = %.16lf", currentStep);
                continue;
            }

            Logger::Info("Iter = %d, Error = %.16lf", iter + 1, ComputeError(m_Gradients));
            break;
        }
    }
}

void OMT2D::NormalizeUVToUnitSquare(TriMesh * pMesh)
{
    Real uMax = -std::numeric_limits<Real>::max();
    Real vMax = -std::numeric_limits<Real>::max();
    Real uMin =  std::numeric_limits<Real>::max();
    Real vMin =  std::numeric_limits<Real>::max();

    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        uMax = std::max(uMax, uv[0]);
        vMax = std::max(vMax, uv[1]);
        uMin = std::min(uMin, uv[0]);
        vMin = std::min(vMin, uv[1]);
    }

    Real invDeltaU = 1.0 / (uMax - uMin);
    Real invDeltaV = 1.0 / (vMax - vMin);

    for (auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        pMesh->set_texcoord2D(vh, {
            (uv[0] - uMin) * invDeltaU * 2.0 - 1.0,
            (uv[1] - vMin) * invDeltaV * 2.0 - 1.0
        });
    }
}

bool OMT2D::CheckInitParamterization(TriMesh * pMesh)
{
    std::set<std::pair<Real, Real>> uvSet;
    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        auto [iter, bSuccess] = uvSet.insert({ uv[0], uv[1] });
        if (!bSuccess)
        {
            Logger::Error("Overlapped uv texcoord found. index = %d, uv = (%lf, %lf).", vh.idx(), uv[0], uv[1]);
            return false;
        }
    }

    return true;
}

void OMT2D::InitializeWeight(const TriMesh * pMesh, std::vector<Real> & weights)
{
    weights.resize(pMesh->n_vertices(), 0);
}

void OMT2D::ComputeTargetArea(const TriMesh * pMesh, std::vector<Real> & targetAreas)
{
    targetAreas.resize(pMesh->n_vertices(), 0);

    Real totalMeshArea = 0.0;
    for (const auto & fh : pMesh->faces())
    {
        TriMesh::Point points[3];
        size_t Index = 0;

        for (const auto & vh : pMesh->fv_range(fh))
        {
            points[Index++] = pMesh->point(vh);
        }

        const auto & va = points[0];
        const auto & vb = points[1];
        const auto & vc = points[2];

        auto lab = va - vb;
        auto lac = vc - vb;

        Real area = lab.cross(lac).norm() * 0.5;
        totalMeshArea += area;
    }

    for (const auto & vh : pMesh->vertices())
    {
        Real oneRingMeshArea = 0.0;
        for (const auto & heh : pMesh->voh_range(vh))
        {
            auto thisHalfedge = heh;
            auto nextHalfedge = pMesh->next_halfedge_handle(heh);

            auto vertexHandleA = pMesh->from_vertex_handle(thisHalfedge);
            auto vertexHandleB = pMesh->to_vertex_handle(thisHalfedge);
            auto vertexHandleC = pMesh->to_vertex_handle(nextHalfedge);

            const auto & va = pMesh->point(vertexHandleA);
            const auto & vb = pMesh->point(vertexHandleB);
            const auto & vc = pMesh->point(vertexHandleC);

            auto lab = va - vb;
            auto lac = vc - vb;

            Real area = lab.cross(lac).norm() * 0.5;
            oneRingMeshArea += area / 3.0;
        }

        const Real totalTargetArea = 4.0;
        targetAreas[vh.idx()] = oneRingMeshArea / totalMeshArea * totalTargetArea;
    }
}

/**
 * CGAL often returns objects that are either segments or rays. This converts
 * these objects into segments. If the object would have resolved into a ray,
 * that ray is intersected with the bounding box defined above and returned as
 * a segment.
 * @param obj CGAL objects that are either segments or rays.
 * @param bOutgoing direction of the ray, if obj was an rays.
 * @return
 */
static Kernel::Segment_2 ConvertToSegment2(const CGAL::Object obj, bool bOutgoing)
{
    /* One of these will succeed and one will have a NULL pointer */
    const auto * pSegment = CGAL::object_cast<Kernel::Segment_2>(&obj);
    const auto * pRay = CGAL::object_cast<Kernel::Ray_2>(&obj);

    if (pSegment != nullptr)
    {
        return *pSegment;
    }
    else
    {
        const auto & source = pRay->source();
        const auto & direction = pRay->direction();
        const auto endPoint  = Kernel::Point_2(source.x() + 100000.0 * direction.dx(),source.y() + 100000.0 * direction.dy());
        if(bOutgoing)
        {
            return Kernel::Segment_2(pRay->source(), endPoint);
        }
        else
        {
            return Kernel::Segment_2(endPoint, pRay->source());
        }

    }
}

bool OMT2D::ComputePowerDiagram(const TriMesh * pMesh, const std::vector<Real> & weights, const std::vector<Real> & targetAreas,
    std::vector<Real> & currentAreas, std::vector<Real> & gradients, HessianWeightMap & hessianWeightMap)
{
    currentAreas.resize(pMesh->n_vertices(), 0);
    gradients.resize(pMesh->n_vertices(), 0);
    hessianWeightMap.clear();

    static std::vector<RegularTriangulation::Weighted_point> weightedPoints;
    weightedPoints.resize(pMesh->n_vertices());

    std::unordered_map<std::pair<Real, Real>, size_t, PairHasher<Real, Real>> uvIndexMap;

    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        weightedPoints[vh.idx()] = RegularTriangulation::Weighted_point(
            Kernel::Point_2(uv[0], uv[1]), weights[vh.idx()]
        );
        uvIndexMap[std::make_pair(uv[0], uv[1])] = vh.idx();
//        std::cout << "WeightedPoint (" << uv[0] << ", " << uv[1] << ", " << weights[vh.idx()] << ") [" << vh.idx() << "]\n";
    }

    /* Bounding box of the points. This will be used to crop the Voronoi diagram later. */
    const Kernel::Iso_rectangle_2 bbox(-1.0, -1.0, 1.0, 1.0);

    // In order to crop the Voronoi diagram, we need to convert the bounding box
    // into a polygon.
    CGAL::Polygon_2<Kernel> bboxPolyon;
    bboxPolyon.push_back(Kernel::Point_2(bbox.xmin(), bbox.ymin()));
    bboxPolyon.push_back(Kernel::Point_2(bbox.xmax(), bbox.ymin()));
    bboxPolyon.push_back(Kernel::Point_2(bbox.xmax(), bbox.ymax()));
    bboxPolyon.push_back(Kernel::Point_2(bbox.xmin(), bbox.ymax()));

    /* Create a Regular Triangulation from the points. */
    RegularTriangulation regularTriangulation(weightedPoints.begin(), weightedPoints.end());
    regularTriangulation.is_valid();

    /* Wrap the triangulation with a Voronoi diagram adaptor. This is necessary to get the Voronoi faces. */
    VoronoiDiagram voronoiDiagram(regularTriangulation);

//    if (regularTriangulation.number_of_vertices() != pMesh->n_vertices() || voronoiDiagram.number_of_faces() != pMesh->n_vertices())
//    {
//        return false;
//    }

//    size_t faceIdx = 0;

//    std::set<size_t> cntSet;
//    double totalCellArea = 0.0;

    /* Loop over the faces of the Voronoi diagram in some arbitrary order. */
    for(VoronoiDiagram::Face_iterator fiter = voronoiDiagram.faces_begin(); fiter != voronoiDiagram.faces_end(); ++fiter/*, faceIdx++*/)
    {
        CGAL::Polygon_2<Kernel> polygon;

        /* Edge circulators traverse endlessly around a face. Make a note of the starting point so we know when to quit. */
        VoronoiDiagram::Face::Ccb_halfedge_circulator edgeCirculatorstart = fiter->ccb();

        /* Find a bounded edge to start on */
        for(; edgeCirculatorstart->is_unbounded(); edgeCirculatorstart++) { }

        /* Current location of the edge circulator */
        VoronoiDiagram::Face::Ccb_halfedge_circulator edgeCirculator = edgeCirculatorstart;

        do
        {
            // A half edge circulator representing a ray doesn't carry direction
            // information. To get it, we take the dual of the dual of the half-edge.
            // The dual of a half-edge circulator is the edge of a Delaunay triangle.
            // The dual of the edge of Delaunay triangle is either a segment or a ray.
            const CGAL::Object segmentDual = voronoiDiagram.dual().dual(edgeCirculator->dual());

            /* Convert the segment or ray into a segment */
            const Kernel::Segment_2 segment = ConvertToSegment2(segmentDual, edgeCirculator->has_target());

            polygon.push_back(segment.source());

            // If the segment has no target, it's a ray. This means that the next
            // segment will also be a ray. We need to connect those two rays with a
            // segment. The following accomplishes this.
            if(!edgeCirculator->has_target())
            {
                const CGAL::Object nextsegmentDual = voronoiDiagram.dual().dual(edgeCirculator->next()->dual());
                const Kernel::Segment_2 nextSegment = ConvertToSegment2(nextsegmentDual, edgeCirculator->next()->has_target());
                polygon.push_back(nextSegment.target());
            }
        }
        while (++edgeCirculator != edgeCirculatorstart ); /* Loop until we get back to the beginning */

        // Perform the intersection. Since CGAL is very general, it believes the
        // result might be multiple polygons with holes.
        std::list<CGAL::Polygon_with_holes_2<Kernel>> intersectionPolygons;
        CGAL::intersection(polygon, bboxPolyon, std::back_inserter(intersectionPolygons));

        // But we know better. The intersection of a convex polygon and a box is
        // always a single polygon without holes. Let's assert this.
//        assert(intersectionPolygons.size() == 1);
        if (intersectionPolygons.size() != 1)
        {
            continue;
        }

        // And recover the polygon of interest
        CGAL::Polygon_with_holes_2<Kernel> & polygonWithHoles = intersectionPolygons.front();
        CGAL::Polygon_2<Kernel> & polygonOuterBoundary = polygonWithHoles.outer_boundary();

//        std::cout << faceIdx << ", \"POLYGON ((";
//        for(auto v = polygonOuterBoundary.vertices_begin(); v != polygonOuterBoundary.vertices_end(); v++)
//        {
//            std::cout<<v->x()<<" "<<v->y()<<", ";
//        }
//
//        std::cout << polygonOuterBoundary.vertices_begin()->x() << " " << polygonOuterBoundary.vertices_begin()->y() << "))\"\n";

        const RegularTriangulation::Weighted_point & site = regularTriangulation.point(fiter->dual());

        const std::pair<Real, Real> uv = std::make_pair(CGAL::to_double(site.hx()), CGAL::to_double(site.hy()));

        if (uvIndexMap.count(uv) == 0)
        {
            currentAreas.resize(pMesh->n_vertices(), 0);
            gradients.resize(pMesh->n_vertices(), 0);
            return false;
        }

//        std::cout << "DUAL (" << site.hx() << ", " << site.hy() << ", " << site.hw() << ") ---->" << ((uvIndexMap.count(uv) > 0) ? std::to_string(uvIndexMap[uv]) : "##") << " \n";
//        if (!cntSet.insert(uvIndexMap[uv]).second)
//        {
//            std::cout << "!!!!!!!!!!!!!!!!!!FALSE!!!!!!!!!!!!!!!!!!!!! " << uvIndexMap[uv] << "!!!!\n";
//        }
        Kernel::FT polygonArae = -1.0;
        CGAL::area_2(polygonOuterBoundary.vertices_begin(), polygonOuterBoundary.vertices_end(), polygonArae);
//        totalCellArea += CGAL::to_double(polygonArae);
//        std::cout << "polygonArae =  " << polygonArae << "!!!!\n";
//        std::cout << "=========================================\n";
//        std::cout << "=========================================\n";

        if (polygonArae == 0.0)
        {
            currentAreas.resize(pMesh->n_vertices(), 0);
            gradients.resize(pMesh->n_vertices(), 0);
            return false;
        }

        size_t uvIndex = uvIndexMap[uv];
        currentAreas[uvIndex] = static_cast<Real>(CGAL::to_double(polygonArae));
        gradients[uvIndex] = targetAreas[uvIndex] - currentAreas[uvIndex];
//        std::cout << "uvIndex =  " << uvIndex << "!!!!\n";
    }
//    std::cout << "totalCellArea =  " << totalCellArea << "!!!!\n";

    std::unordered_map<int, std::vector<int>> adjMap; /*  */

    /* Loop over the edges of the Voronoi diagram in some arbitrary order. */
    for(VoronoiDiagram::Edge_iterator eiter = voronoiDiagram.edges_begin(); eiter != voronoiDiagram.edges_end(); ++eiter)
    {
        // A half edge circulator representing a ray doesn't carry direction
        // information. To get it, we take the dual of the dual of the half-edge.
        // The dual of a half-edge circulator is the edge of a Delaunay triangle.
        // The dual of the edge of Delaunay triangle is either a segment or a ray.
        const CGAL::Object segmentObj = voronoiDiagram.dual().dual(eiter->dual());

        /* Convert the segment or ray into a segment */
        const Kernel::Segment_2 segment = ConvertToSegment2(segmentObj, eiter->has_target());

        const RegularTriangulation::Segment segmentDuel = voronoiDiagram.dual().segment(eiter->dual());

        Real length = static_cast<Real>(std::sqrt(CGAL::to_double(segment.squared_length())));
        Real dualLength = static_cast<Real>(std::sqrt(CGAL::to_double(segmentDuel.squared_length())));

        if (length >= 2.8284271247462/* sqrt(2^2 + 2^2) diagonal of the square domain [-1, 1]^2 */)
        {
            Kernel::Iso_rectangle_2 bboxRectangle(Kernel::Point_2(bbox.xmin(), bbox.ymin()), Kernel::Point_2(bbox.xmax(), bbox.ymax()));
            CGAL::Object result = CGAL::intersection(segment, bboxRectangle);

            const auto * pSegment = CGAL::object_cast<Kernel::Segment_2>(&result);

            if (pSegment != nullptr)
            {
                length = static_cast<Real>(std::sqrt(CGAL::to_double(pSegment->squared_length())));
//                std::cout << "!!!!!!!!!!!!!!!!!!NORMAL!!!!!!!!!!!!!!!!!!!!\n";
//                std::cout << "(" << segment.source().hx() << ", " <<  segment.source().hy() << ") ---> (" <<  segment.target().hx() << ", " <<  segment.target().hy() << ")\n";
            }
            else
            {
                length = 0.0;
//                std::cout << "!!!!!!!!!!!!!!!!!!NULL!!!!!!!!!!!!!!!!!!!!\n";
//                std::cout << "(" << segment.source().hx() << ", " <<  segment.source().hy() << ") ---> (" <<  segment.target().hx() << ", " <<  segment.target().hy() << ")\n";
            }
        }

//        std::cout << "segment length = " << length << " dual segment length = " << dualLength <<std::endl;

        Real hessianWeight = -std::abs(length / dualLength);

        const Kernel::Point_2 y0 = segmentDuel.source();
        const Kernel::Point_2 y1 = segmentDuel.target();

        const std::pair<Real, Real> uv0 = std::make_pair(CGAL::to_double(y0.hx()), CGAL::to_double(y0.hy()));
        const std::pair<Real, Real> uv1 = std::make_pair(CGAL::to_double(y1.hx()), CGAL::to_double(y1.hy()));

        const int idx0 = uvIndexMap.count(uv0) > 0 ? static_cast<int>(uvIndexMap[uv0]) : -1;
        const int idx1 = uvIndexMap.count(uv1) > 0 ? static_cast<int>(uvIndexMap[uv1]) : -1;

        if (idx0 == -1 || idx1 == -1)
        {
            currentAreas.resize(pMesh->n_vertices(), 0);
            gradients.resize(pMesh->n_vertices(), 0);
            hessianWeightMap.clear();
            return false;
        }

        hessianWeightMap[std::make_pair(idx0, idx1)] = hessianWeight;
        hessianWeightMap[std::make_pair(idx1, idx0)] = hessianWeight;
        adjMap[idx0].push_back(idx1);
        adjMap[idx1].push_back(idx0);

//        std::cout << "hessian[" << idx0 << ", " << idx1 << "] = " << hessianWeight << std::endl;
//        std::cout << "hessian[" << idx1 << ", " << idx0 << "] = " << hessianWeight << std::endl;
    }

    for (int i = 0; i < pMesh->n_vertices(); i++)
    {
        Real sum = 0.0;
        for (size_t j = 0; adjMap.count(i) > 0 && j < adjMap[i].size(); j++)
        {
            sum += hessianWeightMap[std::make_pair(i, adjMap[i][j])];
        }
        hessianWeightMap[std::make_pair(i, i)] = -sum;
//        std::cout << "#hessian[" << i << ", " << i << "] = " << (-sum) << std::endl;
    }

    return true;
}

void OMT2D::UpdateWeight(const std::vector<Real> & updateVectors, Real step, std::vector<Real> & weights)
{
    for (size_t i = 0; i < updateVectors.size(); i++)
    {
        weights[i] += (step * updateVectors[i]);
    }
}

void OMT2D::ComputeUpdateVectorGD(const std::vector<Real> & gradients, std::vector<Real> & updateVectors)
{
    updateVectors.resize(gradients.size());
    for (size_t i = 0; i < gradients.size(); i++)
    {
        updateVectors[i] = -gradients[i];
    }
}

Real OMT2D::ComputeError(const std::vector<Real> & gradients)
{
    Real error = 0.0;
    for (Real gradient : gradients)
    {
        error += std::pow(gradient, 2.0);
    }
    return error;
}