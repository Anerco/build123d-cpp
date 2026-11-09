// Topology members ported from build123d 0.13 that the C++ lacked: Shape.distances /
// copy_attributes_to / entities / as_shape (shape_core.py); Mixin1D.positions(deflection),
// curvature_comb, Edge.distribute_locations / find_tangent / trim / trim_to_length /
// find_intersection_points, Wire.trim / geom_equal / param_at / order_chamfer_edges
// (one_d.py); Mixin2D.split_by_perimeter / _wrap_edge, Face.offset / axes_of_symmetry /
// radii / seams / semi_angle / uv_face / sew_faces / make_surface_patch / wrap /
// wrap_faces, Shell.location_at, sort_wires_by_build_order (two_d.py); Mixin3D.dprism /
// hollow, Solid.extrude_linear_with_rotation / revolve / sweep with wires (three_d.py);
// Compound.do_children_intersect / extrude / make_triad (composite.py).
#include <cfloat>
#include <cmath>
#include <set>

#include <BOPAlgo_Splitter.hxx>
#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFeat_MakeDPrism.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomProjLib.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <ShapeFix_Wire.hxx>
#include <StdFail_NotDone.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_SequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Torus.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

TopAbs_ShapeEnum topabs_of(ShapeType t) {
    switch (t) {
        case ShapeType::VERTEX: return TopAbs_VERTEX;
        case ShapeType::EDGE: return TopAbs_EDGE;
        case ShapeType::WIRE: return TopAbs_WIRE;
        case ShapeType::FACE: return TopAbs_FACE;
        case ShapeType::SHELL: return TopAbs_SHELL;
        case ShapeType::SOLID: return TopAbs_SOLID;
        case ShapeType::COMPSOLID: return TopAbs_COMPSOLID;
        case ShapeType::COMPOUND: return TopAbs_COMPOUND;
        default: return TopAbs_SHAPE;
    }
}

/// numpy.linspace(start, stop, num, endpoint).
std::vector<double> linspace(double start, double stop, int num, bool endpoint) {
    std::vector<double> y;
    if (num <= 0) return y;
    const int div = endpoint ? num - 1 : num;
    const double delta = stop - start;
    const double step = div > 0 ? delta / div : std::nan("");
    for (int i = 0; i < num; ++i) y.push_back(div > 0 ? i * step + start : i * delta + start);
    if (endpoint && num > 1) y.back() = stop;
    return y;
}

}  // namespace

// ---------------------------------------------------------------------------
// Shape
// ---------------------------------------------------------------------------

std::vector<double> Shape::distances(const std::vector<Shape>& others) const {
    require_not_null("Cannot calculate distance to or from an empty shape");
    std::vector<double> out;
    BRepExtrema_DistShapeShape dist_calc;
    dist_calc.LoadS1(obj_);
    for (const auto& other : others) {
        if (!other) throw std::invalid_argument("Cannot calculate distance to or from an empty shape");
        occt("distances", [&] {
            dist_calc.LoadS2(other.wrapped());
            dist_calc.Perform();
            out.push_back(dist_calc.Value());
            return 0;
        });
    }
    return out;
}

void Shape::copy_attributes_to(Shape& target, const std::vector<std::string>& exceptions) const {
    detail::copy_attributes_to(*this, target, exceptions);
}

std::vector<TopoDS_Shape> Shape::entities(ShapeType topo_type) const {
    if (obj_.IsNull()) return {};
    return detail::entities(obj_, topabs_of(topo_type));
}

Shape Shape::as_shape(const IntersectArg& obj) {
    if (const auto* s = std::get_if<Shape>(&obj)) return *s;
    if (const auto* v = std::get_if<Vector>(&obj)) return Vertex(*v);
    if (const auto* l = std::get_if<Location>(&obj)) return Vertex(l->position());
    if (const auto* a = std::get_if<Axis>(&obj)) return Edge(*a);
    return Face(BRepBuilderAPI_MakeFace(std::get<Plane>(obj).wrapped()).Face());
}

// ---------------------------------------------------------------------------
// Mixin1D
// ---------------------------------------------------------------------------

std::vector<Vector> Mixin1D::positions(const PositionsOptions& options) const {
    if (options.deflection) {
        require_not_null("Can't find positions on an empty curve");
        const auto discretize = [&](const Adaptor3d_Curve& curve) {
            std::vector<Vector> out;
            occt("positions", [&] {
                const GeomAbs_Shape continuity = curve.Continuity();
                std::vector<double> params;
                if (continuity == GeomAbs_C2 || continuity == GeomAbs_C3 || continuity == GeomAbs_CN) {
                    GCPnts_UniformDeflection discretizer;
                    discretizer.Initialize(curve, *options.deflection, curve.FirstParameter(), curve.LastParameter());
                    if (!discretizer.IsDone() || discretizer.NbPoints() == 0) {
                        throw std::runtime_error("Deflection calculation failed");
                    }
                    for (int i = 1; i <= discretizer.NbPoints(); ++i) params.push_back(discretizer.Parameter(i));
                } else {
                    GCPnts_QuasiUniformDeflection discretizer;
                    discretizer.Initialize(curve, *options.deflection, curve.FirstParameter(), curve.LastParameter());
                    if (!discretizer.IsDone() || discretizer.NbPoints() == 0) {
                        throw std::runtime_error("Deflection calculation failed");
                    }
                    for (int i = 1; i <= discretizer.NbPoints(); ++i) params.push_back(discretizer.Parameter(i));
                }
                for (double u : params) out.emplace_back(curve.Value(u));
                return 0;
            });
            return out;
        };
        if (obj_.ShapeType() == TopAbs_WIRE) return discretize(BRepAdaptor_CompCurve(TopoDS::Wire(obj_)));
        return discretize(BRepAdaptor_Curve(TopoDS::Edge(obj_)));
    }
    if (options.distances) return positions(*options.distances, options.position_mode);
    throw std::invalid_argument("Either distances or deflection must be provided");
}

ShapeList<Edge> Mixin1D::curvature_comb(int count, std::optional<double> max_tooth_size) const {
    if (is_null()) throw std::invalid_argument("Can't create curvature_comb for empty curve");
    const auto pln = common_plane();
    if (!pln || std::abs(std::abs(pln->z_dir.Z) - 1.0) > TOLERANCE) {
        throw std::invalid_argument("curvature_comb only works for curves on Plane.XY");
    }
    // If periodic the first and last tooth would be the same so skip them
    const std::vector<double> u_values = linspace(0, 1, count, !is_closed());
    // first pass: gather kappas for scaling
    std::vector<double> kappas;
    std::vector<Vector> tangents;
    for (double u : u_values) {
        const Vector tangent = derivative_at(u, 1);
        const Vector curvature = derivative_at(u, 2);
        tangents.push_back(tangent);
        const Vector cross = tangent.cross(curvature);
        const double kappa = cross.length() / (std::pow(tangent.length(), 3) + TOLERANCE);
        const double sign = cross.Z >= 0 ? 1.0 : -1.0;  // signed for XY
        kappas.push_back(sign * kappa);
    }
    // choose a scale so the tallest tooth is max_tooth_size
    double max_kappa = 0.0;
    for (double k : kappas) max_kappa = std::max(max_kappa, std::abs(k));
    const double max_kappa_size = std::max(TOLERANCE, max_kappa);
    const Vector size = bounding_box().size;
    const double curve_size = std::max({size.X, size.Y, size.Z});
    const double tooth = max_tooth_size ? *max_tooth_size : curve_size / 10;
    const double scale = tooth / max_kappa_size;
    ShapeList<Edge> comb_edges;
    for (std::size_t i = 0; i < u_values.size(); ++i) {
        const double length = scale * kappas[i];
        if (std::abs(length) < TOLERANCE) continue;  // avoid tiny teeth
        const Vector pnt_on_curve = position_at(u_values[i]);
        // left normal in XY (principal normal direction for a planar curve)
        const Vector kappa_dir = tangents[i].normalized().cross(Vector(0, 0, 1));
        comb_edges.push_back(Edge::make_line(pnt_on_curve, pnt_on_curve + kappa_dir * length));
    }
    return comb_edges;
}

// ---------------------------------------------------------------------------
// Edge
// ---------------------------------------------------------------------------

Edge Edge::trim(double start, const Vector& end) const { return trim(start, param_at_point(end)); }
Edge Edge::trim(const Vector& start, double end) const { return trim(param_at_point(start), end); }

Edge Edge::trim_to_length(const Vector& start, double length) const {
    require_not_null("Can't trim empty edge");
    return trim_to_length(param_at_point(start), length);
}

std::vector<Location> Edge::distribute_locations(int count, double start, double stop, bool positions_only) const {
    if (count < 2) throw std::invalid_argument("count must be two or greater");
    std::vector<double> t_values;
    for (int i = 0; i < count; ++i) t_values.push_back(start + i * (stop - start) / (count - 1));
    std::vector<Location> locs = locations(t_values);
    if (positions_only) {
        for (auto& loc : locs) loc.set_orientation(Vector(0, 0, 0));
    }
    return locs;
}

std::vector<double> Edge::find_tangent(double angle_in) const {
    const double angle = py_float_mod(angle_in, 360.0);  // always positive 0..360
    std::vector<double> u_values;
    if (geom_type() == GeomType::LINE) {
        if (tangent_angle_at(0) == angle) u_values.push_back(0);
        return u_values;
    }
    // Solve geometrically: a tangent-angle curve and its intercepts with horizontal lines
    const int periodic = is_closed() ? 1 : 0;
    std::vector<Vector> tan_pnts;
    std::optional<double> previous_tangent;
    // When angles go from 360 to 0 a discontinuity is created so add 360 to these values
    double discontinuities = 0.0;
    for (int i = 0; i < 101 - periodic; ++i) {
        double tangent = tangent_angle_at(i / 100.0) + discontinuities * 360;
        if (previous_tangent && std::abs(*previous_tangent - tangent) > 300) {
            discontinuities = std::copysign(1.0, *previous_tangent - tangent);
            tangent += 360 * discontinuities;
        }
        previous_tangent = tangent;
        tan_pnts.emplace_back(i / 100.0, tangent);
    }
    const Edge tan_curve = Edge::make_spline(tan_pnts);
    const BoundBox tan_curve_bbox = tan_curve.bounding_box();
    const long long min_range = 360 * static_cast<long long>(std::floor(tan_curve_bbox.min.Y / 360));
    const long long max_range = 360 * static_cast<long long>(std::ceil(tan_curve_bbox.max.Y / 360));
    std::vector<Vector> intercept_pnts;
    for (long long i = min_range; i < max_range + 1; i += 360) {
        const Edge line = Edge::make_line(Vector(0, angle + i, 0), Vector(100, angle + i, 0));
        for (const auto& p : tan_curve.find_intersection_points(line)) intercept_pnts.push_back(p);
    }
    for (const auto& p : intercept_pnts) u_values.push_back(p.X);
    return u_values;
}

namespace {

/// Edge.find_intersection_points: the crossings of 2D curves on their common plane.
std::vector<Vector> edge_intersection_points(const Edge& self, const std::optional<Edge>& other, double tolerance) {
    if (self.is_null()) throw std::invalid_argument("Can't find intersections of empty edge");
    // To determine the 2D plane to work on
    std::vector<Shape> lines;
    if (other) lines.push_back(*other);
    const auto plane = self.common_plane(lines);
    if (!plane) throw std::invalid_argument("All objects must be on the same plane");
    std::vector<Vector> crosses;
    occt("find_intersection_points", [&] {
        // Convert the plane into a Geom_Surface
        const TopoDS_Face pln_shape = BRepBuilderAPI_MakeFace(plane->wrapped()).Face();
        const Handle(Geom_Surface) edge_surface = BRep_Tool::Surface(pln_shape);
        double first, last;
        const Handle(Geom2d_Curve) self_2d_curve =
            BRep_Tool::CurveOnPlane(TopoDS::Edge(self.wrapped()), edge_surface, TopLoc_Location(), first, last);
        std::vector<gp_Pnt2d> points;
        if (other && !other->is_null()) {
            const Handle(Geom2d_Curve) edge_2d_curve =
                BRep_Tool::CurveOnPlane(TopoDS::Edge(other->wrapped()), edge_surface, TopLoc_Location(), first, last);
            Geom2dAPI_InterCurveCurve intersector(self_2d_curve, edge_2d_curve, tolerance);
            for (int i = 1; i <= intersector.NbPoints(); ++i) points.push_back(intersector.Point(i));
        } else {
            Geom2dAPI_InterCurveCurve intersector(self_2d_curve, tolerance);
            for (int i = 1; i <= intersector.NbPoints(); ++i) points.push_back(intersector.Point(i));
        }
        // Convert back to global coordinates
        for (const auto& p : points) crosses.push_back(plane->from_local_coords(Vector(p.X(), p.Y())));
        return 0;
    });
    // crosses may contain points beyond the ends of the edge so filter those out
    std::vector<Vector> valid_crosses;
    for (const auto& pnt : crosses) {
        try {
            if (other) {
                if (self.distance_to(pnt) <= TOLERANCE && other->distance_to(pnt) <= TOLERANCE) {
                    valid_crosses.push_back(pnt);
                }
            } else if (self.distance_to(pnt) <= TOLERANCE) {
                valid_crosses.push_back(pnt);
            }
        } catch (const std::invalid_argument&) {
            // skip invalid points
        }
    }
    return valid_crosses;
}

}  // namespace

std::vector<Vector> Edge::find_intersection_points(const Edge& other, double tolerance) const {
    return edge_intersection_points(*this, other, tolerance);
}

std::vector<Vector> Edge::find_intersection_points(const Axis& axis, double tolerance) const {
    if (is_null()) throw std::invalid_argument("Can't find intersections of empty edge");
    // Convert an Axis into an edge at least as large as self and Axis start point
    const BoundBox self_bbox_w_edge = bounding_box().add(Vertex(axis.position).bounding_box());
    const Edge line = Edge::make_line(axis.position + axis.direction * (-1 * self_bbox_w_edge.diagonal()),
                                      axis.position + axis.direction * self_bbox_w_edge.diagonal());
    return edge_intersection_points(*this, line, tolerance);
}

std::vector<Vector> Edge::find_intersection_points(double tolerance) const {
    return edge_intersection_points(*this, std::nullopt, tolerance);
}

namespace {

/// Edge._extend_spline: a spline extended past one end and snapped to the surface.
std::pair<Edge, Handle(Geom_Curve)> extend_spline(const Edge& edge, bool at_start,
                                                  const Handle(Geom_Surface)& geom_surface,
                                                  double extension_factor) {
    if (edge.is_null()) throw std::invalid_argument("Can't extend empty spline");
    if (edge.geom_type() != GeomType::BSPLINE) throw std::invalid_argument("_extend_spline only works with splines");
    double first, last;
    const Handle(Geom_BSplineCurve) curve_original =
        Handle(Geom_BSplineCurve)::DownCast(BRep_Tool::Curve(TopoDS::Edge(edge.wrapped()), first, last));
    if (curve_original.IsNull()) throw std::invalid_argument("_extend_spline only works with splines");
    std::vector<Vector> poles;
    for (int i = 1; i <= curve_original->NbPoles(); ++i) poles.emplace_back(curve_original->Pole(i));
    // Find position and tangent past end of spline to extend it
    const std::array<double, 2> ends =
        at_start ? std::array<double, 2>{-extension_factor, 1} : std::array<double, 2>{0, 1 + extension_factor};
    if (at_start) {
        poles.insert(poles.begin(), edge.position_at(-extension_factor));
    } else {
        poles.push_back(edge.position_at(1 + extension_factor));
    }
    const std::vector<Vector> tangents{edge.tangent_at(ends[0]), edge.tangent_at(ends[1])};
    const Edge extended_edge = Edge::make_spline(poles, tangents);
    const Handle(Geom_Curve) geom_curve = BRep_Tool::Curve(TopoDS::Edge(extended_edge.wrapped()), first, last);
    const Handle(Geom_Curve) snapped_geom_curve = GeomProjLib::Project(geom_curve, geom_surface);
    if (snapped_geom_curve.IsNull()) throw std::runtime_error("Failed to snap extended edge to surface");
    // Build a new projected edge
    return {Edge(BRepBuilderAPI_MakeEdge(snapped_geom_curve).Edge()), snapped_geom_curve};
}

}  // namespace

// ---------------------------------------------------------------------------
// Wire
// ---------------------------------------------------------------------------

Wire Wire::trim(double start_u_in, double end_u_in) const {
    const double start_u = std::min(start_u_in, end_u_in);
    const double end_u = std::max(start_u_in, end_u_in);
    // Extract the edges in order
    const auto ordered_edges = edges().sort_by(static_cast<const Mixin1D&>(*this));
    // If this is really just an edge, skip the complexity of a Wire
    if (ordered_edges.size() == 1) return Wire(std::vector<Edge>{ordered_edges[0].trim(start_u, end_u)});
    const double total_length = length();
    const double start_len = start_u * total_length;
    const double end_len = end_u * total_length;
    std::vector<Edge> trimmed_edges;
    double cur_length = 0.0;
    for (const auto& edge : ordered_edges) {
        const double edge_len = edge.length();
        const double edge_start = cur_length;
        const double edge_end = cur_length + edge_len;
        cur_length = edge_end;
        if (edge_end <= start_len || edge_start >= end_len) continue;  // skip
        if (edge_start >= start_len && edge_end <= end_len) {
            trimmed_edges.push_back(edge);  // keep whole Edge
        } else {
            // Normalize trim points relative to this edge
            const double trim_start_len = std::max(start_len, edge_start);
            const double trim_end_len = std::min(end_len, edge_end);
            const double u0 = (trim_start_len - edge_start) / edge_len;
            const double u1 = (trim_end_len - edge_start) / edge_len;
            if (std::abs(u1 - u0) > TOLERANCE) trimmed_edges.push_back(edge.trim(u0, u1));
        }
    }
    return Wire(trimmed_edges);
}

Wire Wire::trim(const Vector& start, const Vector& end) const { return trim(param_at_point(start), param_at_point(end)); }
Wire Wire::trim(double start, const Vector& end) const { return trim(start, param_at_point(end)); }
Wire Wire::trim(const Vector& start, double end) const { return trim(param_at_point(start), end); }

bool Wire::geom_equal(const Wire& other, double tol, int num_interpolation_points) const {
    // order_edges gives a consistent edge ordering and orientation
    const auto edges1 = order_edges();
    const auto edges2 = other.order_edges();
    if (edges1.size() != edges2.size()) return false;
    for (std::size_t i = 0; i < edges1.size(); ++i) {
        const auto idx = static_cast<std::ptrdiff_t>(i);
        if (!edges1[idx].geom_equal(edges2[idx], tol, num_interpolation_points)) return false;
    }
    return true;
}

double Wire::param_at(double position) const {
    require_not_null("Can't find a parameter on an empty wire");
    const BRepAdaptor_CompCurve curve(TopoDS::Wire(obj_));
    // Compute the target point along the wire and project it to the comp-curve
    const Vector target_pnt = position_at(position);
    return occt("Wire.param_at", [&] {
        Extrema_ExtPC extrema(target_pnt.to_pnt(), curve);
        if (!extrema.IsDone() || extrema.NbExt() == 0) throw std::runtime_error("Failed to find point on curve");
        double min_dist = std::numeric_limits<double>::infinity();
        std::optional<gp_Pnt> closest_pnt;
        double closest_param = 0.0;
        for (int i = 1; i <= extrema.NbExt(); ++i) {
            const double dist = extrema.SquareDistance(i);
            if (dist < min_dist) {
                min_dist = dist;
                closest_pnt = extrema.Point(i).Value();
                closest_param = extrema.Point(i).Parameter();
            }
        }
        if (!closest_pnt || (Vector(*closest_pnt) - target_pnt).length() > TOLERANCE) {
            throw std::runtime_error("Failed to find point on curve");
        }
        return closest_param;
    });
}

std::pair<Edge, Edge> Wire::order_chamfer_edges(const std::optional<Edge>& reference_edge,
                                                const std::pair<Edge, Edge>& edges) {
    if (reference_edge) {
        const auto& [edge1, edge2] = edges;
        if (edge1.is_same(*reference_edge)) return {edge1, edge2};
        if (edge2.is_same(*reference_edge)) return {edge2, edge1};
        throw std::invalid_argument("reference edge not in edges");
    }
    return edges;
}

// ---------------------------------------------------------------------------
// Mixin2D
// ---------------------------------------------------------------------------

namespace {

std::vector<TopoDS_Edge> split_edge_at(const Edge& edge, const Vertex& split_vertex);

/// Mixin2D.split_by_perimeter's process_sides: the side as nothing, one shape, a Shell sewn
/// from the faces, or the faces themselves.
ShapeList<Shape> process_sides(const std::vector<Shape>& sides) {
    if (sides.empty()) return {};
    if (sides.size() == 1) return {sides[0]};
    std::vector<TopoDS_Shape> faces;
    for (const auto& s : sides) faces.push_back(s.wrapped());
    const TopoDS_Shape potential_shell = detail::sew_faces(faces);
    if (!potential_shell.IsNull() && potential_shell.ShapeType() == TopAbs_SHELL) return {Shape(potential_shell)};
    return ShapeList<Shape>(sides);
}

double side_perimeter(const ShapeList<Shape>& side) {
    double total = 0.0;
    for (const auto& s : side) {
        for (const auto& e : s.edges()) total += e.length();
    }
    return total;
}

}  // namespace

PerimeterSplit Mixin2D::split_by_perimeter(const Mixin1D& perimeter, Keep keep) const {
    if (keep != Keep::INSIDE && keep != Keep::OUTSIDE && keep != Keep::BOTH) {
        throw std::invalid_argument("keep must be one of Keep.INSIDE, Keep.OUTSIDE, or Keep.BOTH");
    }
    // Process the perimeter
    if (!perimeter.is_closed()) throw std::invalid_argument("perimeter must be a closed Wire or Edge");
    TopTools_SequenceOfShape perimeter_edges;
    std::vector<Edge> seams;
    for (const auto& face : faces()) {
        for (const auto& seam : face.seams()) seams.push_back(seam);
    }
    const auto add_unique_vertex = [](std::vector<Vertex>& vertices, const Vertex& vertex) {
        for (const auto& existing : vertices) {
            if (vertex.distance_to(existing) <= TOLERANCE) return;
        }
        vertices.push_back(vertex);
    };
    for (const auto& perimeter_edge : perimeter.edges()) {
        if (!perimeter_edge) continue;
        std::vector<Vertex> seam_vertices;
        for (const auto& seam : seams) {
            const auto seam_intersection = perimeter_edge.intersect(IntersectArg(seam));
            if (seam_intersection.empty()) continue;
            for (const auto& vertex : ShapeList<Shape>(seam_intersection).vertices()) {
                bool interior = true;
                for (const auto& edge_vertex : perimeter_edge.vertices()) {
                    interior = interior && vertex.distance_to(edge_vertex) > TOLERANCE;
                }
                if (interior) add_unique_vertex(seam_vertices, vertex);
            }
        }
        // split the edge at all the interior seam vertices
        std::vector<Edge> segments{perimeter_edge};
        for (const auto& vertex : seam_vertices) {
            std::vector<Edge> next_segments;
            for (const auto& segment : segments) {
                bool at_end = false;
                for (const auto& edge_vertex : segment.vertices()) {
                    at_end = at_end || vertex.distance_to(edge_vertex) <= TOLERANCE;
                }
                if (segment.distance_to(vertex) > TOLERANCE || at_end) {
                    next_segments.push_back(segment);
                    continue;
                }
                for (const auto& split : split_edge_at(segment, vertex)) next_segments.emplace_back(split);
            }
            segments = next_segments;
        }
        for (const auto& split_edge : segments) perimeter_edges.Append(split_edge.wrapped());
    }
    // Split the Face or Shell by the perimeter edges
    std::vector<Shape> lefts, rights;
    occt("split_by_perimeter", [&] {
        BRepFeat_SplitShape constructor(obj_);
        constructor.Add(perimeter_edges);
        constructor.Build();
        for (const auto& s : constructor.Left()) {
            if (!s.IsNull()) lefts.emplace_back(s);
        }
        for (const auto& s : constructor.Right()) {
            if (!s.IsNull()) rights.emplace_back(s);
        }
        return 0;
    });
    const ShapeList<Shape> left = process_sides(lefts);
    const ShapeList<Shape> right = process_sides(rights);
    // Is left or right the inside?
    const double perimeter_length = perimeter.length();
    const bool left_inside = std::abs(perimeter_length - side_perimeter(left)) <
                             std::abs(perimeter_length - side_perimeter(right));
    PerimeterSplit result;
    const ShapeList<Shape>& inside = left_inside ? left : right;
    const ShapeList<Shape>& outside = left_inside ? right : left;
    if (keep == Keep::INSIDE || keep == Keep::BOTH) {
        result.inside = inside;
        result.insert(result.end(), inside.begin(), inside.end());
    }
    if (keep == Keep::OUTSIDE || keep == Keep::BOTH) {
        result.outside = outside;
        result.insert(result.end(), outside.begin(), outside.end());
    }
    return result;
}

namespace {

std::vector<TopoDS_Edge> split_edge_at(const Edge& edge, const Vertex& split_vertex) {
    // one_d._split_edge_at_vertex
    return occt("split_edge_at_vertex", [&] {
        BOPAlgo_Splitter splitter;
        splitter.AddArgument(edge.wrapped());
        splitter.AddTool(split_vertex.wrapped());
        splitter.Perform();
        std::vector<TopoDS_Edge> split_edges;
        for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
            split_edges.push_back(TopoDS::Edge(exp.Current()));
        }
        return split_edges;
    });
}

/// Mixin2D._wrap_edge: an edge on the XY plane wrapped onto the surface (Face or Shell).
Edge wrap_edge(const Shape& self, const Edge& planar_edge, const Location& surface_loc, bool snap_to_face,
               double tolerance) {
    if (self.is_null()) throw std::invalid_argument("Can't wrap around an empty face");
    const Vector target_object_center = self.center(CenterOf::BOUNDING_BOX);
    const Vector surface_x_direction = surface_loc.x_axis().direction;
    // The intersection point and normal of the closest surface face along direction
    const auto intersect_surface_normal = [&](const Vector& point, const Vector& direction) {
        const Axis axis(point, direction);
        const auto faces = self.faces_intersected_by_axis(axis).sort_by(
            [&](const Face& f) { return f.distance_to(point); });
        if (faces.empty()) throw std::runtime_error("wrapping over surface boundary, try difference surface_loc");
        const auto inter = faces[0].find_intersection_points(axis);
        if (inter.empty()) throw std::runtime_error("wrapping over surface boundary, try difference surface_loc");
        std::size_t best = 0;
        for (std::size_t i = 1; i < inter.size(); ++i) {
            if ((inter[i].first - point).length() < (inter[best].first - point).length()) best = i;
        }
        return inter[best];
    };
    // Project a 2D offset from a local surface frame onto the 3D surface
    const auto find_point_on_surface = [&](const Vector& current_point, const Vector& normal,
                                           const Vector& relative_position) {
        const Plane local_plane(current_point, surface_x_direction, normal);
        const Vector world_point = local_plane.from_local_coords(relative_position);
        return intersect_surface_normal(world_point, world_point - target_object_center);
    };
    const double planar_edge_length = planar_edge.length();
    int subdivisions = 3;
    const int max_loops = 10;
    int loop_count = 0;
    double length_error = DBL_MAX;
    // Find the location on the surface to start
    Vector start_pnt, start_normal;
    if (planar_edge.position_at(0).length() > tolerance) {
        // The start point isn't at the surface_loc so wrap a line to find it
        const Edge to_start_edge = Edge::make_line(Vector(0, 0), planar_edge.position_at(0));
        const Edge wrapped_to_start_edge = wrap_edge(self, to_start_edge, surface_loc, true, tolerance);
        start_pnt = wrapped_to_start_edge.position_at(1);
        start_normal = intersect_surface_normal(start_pnt, start_pnt - target_object_center).second;
    } else {
        // The start point is at the surface location
        start_pnt = surface_loc.position();
        start_normal = surface_loc.z_axis().direction;
    }
    Edge wrapped_edge;
    while (length_error > tolerance && loop_count < max_loops) {
        // Seed the wrapped path
        std::vector<Vector> wrapped_edge_points;
        Vector current_point = start_pnt, current_normal = start_normal;
        wrapped_edge_points.push_back(current_point);
        // Subdivide and propagate
        for (int div = 1; div < subdivisions + (planar_edge.is_closed() ? 0 : 1); ++div) {
            const Vector prev = planar_edge.position_at(static_cast<double>(div - 1) / subdivisions);
            const Vector curr = planar_edge.position_at(static_cast<double>(div) / subdivisions);
            const auto next = find_point_on_surface(current_point, current_normal, curr - prev);
            current_point = next.first;
            current_normal = next.second;
            wrapped_edge_points.push_back(current_point);
        }
        // Build and evaluate
        wrapped_edge = Edge::make_spline(wrapped_edge_points, {}, planar_edge.is_closed());
        length_error = std::abs(planar_edge_length - wrapped_edge.length());
        subdivisions *= 2;
        loop_count += 1;
    }
    if (length_error > tolerance) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "Length error of %.6f exceeds tolerance %s", length_error,
                      py_repr(tolerance).c_str());
        throw std::runtime_error(buf);
    }
    if (!wrapped_edge || !wrapped_edge.is_valid()) throw std::runtime_error("Wrapped edge is invalid");
    if (!snap_to_face) return wrapped_edge;
    // Project the curve onto the surface
    return occt("wrap", [&] {
        const Handle(Geom_Surface) surface_handle = BRep_Tool::Surface(TopoDS::Face(self.wrapped()));
        double first, last;
        const Handle(Geom_Curve) curve_handle = BRep_Tool::Curve(TopoDS::Edge(wrapped_edge.wrapped()), first, last);
        const Handle(Geom_Curve) proj_curve_handle = GeomProjLib::Project(curve_handle, surface_handle);
        if (proj_curve_handle.IsNull()) {
            throw std::runtime_error("Projection failed, try setting `snap_to_face` to False.");
        }
        return Edge(BRepBuilderAPI_MakeEdge(proj_curve_handle).Edge());
    });
}

/// Face._wrap_wire.
Wire wrap_wire(const Face& self, const Wire& planar_wire, const Location& surface_loc, double tolerance,
               double extension_factor) {
    // Part 1: Preparation
    const Vector surface_point = surface_loc.position();
    const Vector surface_x_direction = surface_loc.x_axis().direction;
    const Handle(Geom_Surface) surface_geometry = BRep_Tool::Surface(TopoDS::Face(self.wrapped()));
    if (planar_wire.edges().size() == 1) {
        return Wire(std::vector<Edge>{wrap_edge(self, planar_wire.edge(), surface_loc, true, tolerance)});
    }
    const auto planar_edges = planar_wire.order_edges();
    std::vector<Edge> wrapped_edges;
    // Part 2: wrap each edge from where the previous one ended
    Vector edge_surface_point, planar_edge_end_point;
    if (planar_edges[0].position_at(0) == Vector(0, 0, 0)) {
        edge_surface_point = surface_point;
        planar_edge_end_point = Vector(0, 0, 0);
    } else {
        // a wrapped construction line gets to the beginning of the first edge
        const Edge construction_line = Edge::make_line(Vector(0, 0, 0), planar_edges[0].position_at(0));
        const Edge wrapped_construction_line = wrap_edge(self, construction_line, surface_loc, true, tolerance);
        edge_surface_point = wrapped_construction_line.position_at(1);
        planar_edge_end_point = planar_edges[0].position_at(0);
    }
    Location edge_surface_location(Plane(edge_surface_point, surface_x_direction, self.normal_at(edge_surface_point)));
    for (const auto& planar_edge : planar_edges) {
        const Edge local_planar_edge = planar_edge.translate(-planar_edge_end_point);
        const Edge wrapped_edge = wrap_edge(self, local_planar_edge, edge_surface_location, true, tolerance);
        edge_surface_point = wrapped_edge.position_at(1);
        edge_surface_location =
            Location(Plane(edge_surface_point, surface_x_direction, self.normal_at(edge_surface_point)));
        planar_edge_end_point = planar_edge.position_at(1);
        wrapped_edges.push_back(wrapped_edge);
    }
    // For open wires we're finished
    if (!planar_wire.is_closed()) return Wire(wrapped_edges);
    // Part 3: extend the first and last edges so that they cross, and trim them there
    const auto [first_edge, first_curve] = extend_spline(wrapped_edges.front(), true, surface_geometry, extension_factor);
    const auto [last_edge, last_curve] = extend_spline(wrapped_edges.back(), false, surface_geometry, extension_factor);
    return occt("wrap", [&] {
        GeomAPI_ExtremaCurveCurve extrema(first_curve, last_curve);
        if (extrema.NbExtrema() < 1) {
            throw std::runtime_error("Extended first/last edges do not intersect; increase extension.");
        }
        double param_first, param_last;
        extrema.Parameters(1, param_first, param_last);
        const double u_start_first = first_edge.param_at(0), u_end_first = first_edge.param_at(1);
        const double new_start = (param_first - u_start_first) / (u_end_first - u_start_first);
        const Edge trimmed_first = first_edge.trim(new_start, 1.0);
        const double u_start_last = last_edge.param_at(0), u_end_last = last_edge.param_at(1);
        const double new_end = (param_last - u_start_last) / (u_end_last - u_start_last);
        const Edge trimmed_last = last_edge.trim(0.0, new_end);
        wrapped_edges.front() = trimmed_first;
        wrapped_edges.back() = trimmed_last;
        // Part 4: Build a wire from the edges and fix it to close gaps
        const double closing_error = (trimmed_first.position_at(0) - trimmed_last.position_at(1)).length();
        BRepBuilderAPI_MakeWire wire_builder;
        TopTools_ListOfShape combined_edges;
        for (const auto& edge : wrapped_edges) combined_edges.Append(edge.wrapped());
        wire_builder.Add(combined_edges);
        wire_builder.Build();
        const TopoDS_Wire raw_wrapped_wire = wire_builder.Wire();
        ShapeFix_Wire wire_fixer;
        wire_fixer.SetPrecision(2 * closing_error);  // enable fixing start/end gaps
        wire_fixer.Load(raw_wrapped_wire);
        wire_fixer.FixReorder();
        wire_fixer.FixConnected();
        const Wire wrapped_wire(wire_fixer.Wire());
        // Part 5: Validate
        if (!wrapped_wire.is_valid()) throw std::runtime_error("wrapped wire is not valid");
        return wrapped_wire;
    });
}

}  // namespace

// ---------------------------------------------------------------------------
// Face
// ---------------------------------------------------------------------------

Face Face::offset(double amount) const { return moved(Location(normal_at() * amount)); }

std::optional<std::pair<double, double>> Face::radii() const {
    if (geom_type() != GeomType::TORUS) return std::nullopt;
    const BRepAdaptor_Surface surface(TopoDS::Face(obj_));
    const gp_Torus torus = surface.Torus();
    return std::make_pair(torus.MajorRadius(), torus.MinorRadius());
}

ShapeList<Edge> Face::seams() const {
    ShapeAnalysis_Edge sae;
    const TopoDS_Face& face = TopoDS::Face(obj_);
    return edges().filter_by([&](const Edge& e) { return sae.IsSeam(TopoDS::Edge(e.wrapped()), face); });
}

std::optional<double> Face::semi_angle() const {
    if (geom_type() != GeomType::CONE) return std::nullopt;
    // build123d reads the surface itself; a trimmed cone gives None
    const Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(obj_));
    if (!Handle(Geom_RectangularTrimmedSurface)::DownCast(surface).IsNull()) return std::nullopt;
    return Handle(Geom_ConicalSurface)::DownCast(surface)->SemiAngle() * RAD2DEG;
}

Face Face::uv_face() const {
    const TopoDS_Face xy_face = BRepBuilderAPI_MakeFace(Plane::XY.wrapped()).Face();
    const Handle(Geom_Surface) xy_surface = BRep_Tool::Surface(xy_face);
    const TopoDS_Face& face = TopoDS::Face(obj_);
    const auto uv_edge = [&](const TopoDS_Edge& native_edge) {
        double first, last;
        BRep_Tool::Range(native_edge, face, first, last);
        double pfirst = first, plast = last;
        const Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(native_edge, face, pfirst, plast);
        BRepBuilderAPI_MakeEdge edge_builder(pcurve, xy_surface, first, last);
        if (!edge_builder.IsDone()) throw std::invalid_argument("Unable to convert pcurve to a planar edge");
        TopoDS_Edge topods_edge = edge_builder.Edge();
        if (native_edge.Orientation() == TopAbs_REVERSED) topods_edge = TopoDS::Edge(topods_edge.Reversed());
        return Edge(topods_edge);
    };
    const auto uv_wire = [&](const Wire& source_wire) {
        std::vector<Edge> uv_edges;
        for (BRepTools_WireExplorer exp(TopoDS::Wire(source_wire.wrapped())); exp.More(); exp.Next()) {
            uv_edges.push_back(uv_edge(TopoDS::Edge(exp.Current())));
        }
        return Wire(uv_edges);
    };
    return occt("uv_face", [&] {
        const Wire outer = uv_wire(outer_wire());
        std::vector<Wire> inner;
        for (const auto& w : inner_wires()) inner.push_back(uv_wire(w));
        return Face(outer, inner);
    });
}

std::vector<ShapeList<Face>> Face::sew_faces(const std::vector<Face>& faces) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& f : faces) topods.push_back(f.wrapped());
    const TopoDS_Shape sewed_shape = detail::sew_faces(topods);
    std::vector<ShapeList<Face>> sewn_faces;
    for (const auto& top_level_shape : top_level(sewed_shape)) {
        switch (top_level_shape.ShapeType()) {
            case TopAbs_FACE: sewn_faces.push_back(ShapeList<Face>{Face(top_level_shape)}); break;
            case TopAbs_SHELL: sewn_faces.push_back(Shell(top_level_shape).faces()); break;
            case TopAbs_SOLID: {
                ShapeList<Face> solid_faces;
                for (const auto& f : detail::entities(top_level_shape, TopAbs_FACE)) solid_faces.emplace_back(f);
                sewn_faces.push_back(solid_faces);
                break;
            }
            default: throw std::runtime_error("SewedShape returned an unexpected shape type");
        }
    }
    return sewn_faces;
}

Face Face::make_surface_patch(const std::vector<std::tuple<Edge, Face, ContinuityLevel>>& edge_face_constraints,
                              const std::vector<Edge>& edge_constraints, const std::vector<Vector>& point_constraints) {
    const auto continuity = [](ContinuityLevel level) {
        switch (level) {
            case ContinuityLevel::C0: return GeomAbs_C0;
            case ContinuityLevel::C1: return GeomAbs_G1;
            default: return GeomAbs_G2;
        }
    };
    BRepOffsetAPI_MakeFilling patch;
    Face result;
    try {
        for (const auto& [edge, face, level] : edge_face_constraints) {
            patch.Add(TopoDS::Edge(edge.wrapped()), TopoDS::Face(face.wrapped()), continuity(level));
        }
        for (const auto& edge : edge_constraints) patch.Add(TopoDS::Edge(edge.wrapped()), continuity(ContinuityLevel::C0));
        for (const auto& point : point_constraints) patch.Add(point.to_pnt());
        patch.Build();
        result = Face(TopoDS::Face(patch.Shape()));
    } catch (const Standard_Failure&) {
        throw std::runtime_error("Error building non-planar face with provided constraints");
    }
    result = result.fix();
    if (!result.is_valid() || !result) throw std::runtime_error("Non planar face is invalid");
    return result;
}

Edge Face::wrap(const Edge& planar_shape, const Location& surface_loc, double tolerance, double) const {
    return wrap_edge(*this, planar_shape, surface_loc, true, tolerance);
}

Wire Face::wrap(const Wire& planar_shape, const Location& surface_loc, double tolerance, double extension_factor) const {
    return wrap_wire(*this, planar_shape, surface_loc, tolerance, extension_factor);
}

Face Face::wrap(const Face& planar_face, const Location& surface_loc, double tolerance, double extension_factor) const {
    const Wire wrapped_perimeter = wrap_wire(*this, planar_face.outer_wire(), surface_loc, tolerance, extension_factor);
    std::vector<Wire> wrapped_holes;
    for (const auto& w : planar_face.inner_wires()) {
        wrapped_holes.push_back(wrap_wire(*this, w, surface_loc, tolerance, extension_factor));
    }
    Face wrapped_face = Face::make_surface(wrapped_perimeter, {surface_loc.position()}, wrapped_holes);
    // Potentially flip the wrapped face to match the surface
    const Vector surface_normal = surface_loc.z_axis().direction;
    const Vector wrapped_normal = wrapped_face.normal_at(surface_loc.position());
    if (surface_normal.dot(wrapped_normal) < 0) wrapped_face = -wrapped_face;
    return wrapped_face;
}

ShapeList<Face> Face::wrap_faces(const std::vector<Face>& faces, const Mixin1D& path, double start) const {
    const double path_length = path.length();
    const double first_face_min_x = faces.at(0).bounding_box().min.X;
    // Position each face at the origin and wrap onto surface
    ShapeList<Face> wrapped_faces;
    for (const auto& face : faces) {
        const BoundBox bbox = face.bounding_box();
        const double face_center_x = (bbox.min.X + bbox.max.X) / 2;
        const double delta_x = face_center_x - first_face_min_x;
        const double relative_position_on_wire = start + delta_x / path_length;
        const Vector path_position = path.position_at(relative_position_on_wire);
        const Location surface_location(
            Plane(path_position, path.tangent_at(relative_position_on_wire), normal_at(path_position)));
        // shift back to the origin (build123d moves the caller's face in place)
        Face shifted = face;
        Location loc = shifted.location();
        loc.set_position(loc.position() - Vector(delta_x, 0, 0));
        shifted = shifted.located(loc);
        wrapped_faces.push_back(wrap(shifted, surface_location));
    }
    return wrapped_faces;
}

std::vector<Axis> Face::axes_of_symmetry() const {
    if (is_null()) throw std::invalid_argument("Can't determine axes_of_symmetry of empty face");
    if (!is_planar()) throw std::invalid_argument("axes_of_symmetry only supports for planar faces");
    const Vector cog = center();
    const Vector normal = normal_at();
    std::vector<Vector> cross_dirs;
    const auto shape_inner_wires = inner_wires();
    if (!shape_inner_wires.empty()) {
        std::vector<Face> hole_faces;
        for (const auto& w : shape_inner_wires) hole_faces.emplace_back(w);
        const Vector holes_centroid = Shape::combined_center(hole_faces);
        // If the holes aren't centered on the cog the axis of symmetry must be through the
        // cog and hole centroid
        if ((holes_centroid - cog).length() > TOLERANCE) {
            cross_dirs.push_back((holes_centroid - cog).normalized());
        } else {
            for (const auto& f : hole_faces) cross_dirs.push_back((f.center() - cog).normalized());
        }
    } else {
        const auto shape_edges = outer_wire().edges();
        const auto curved_edges = shape_edges.filter_by(GeomType::LINE, true);
        std::vector<Edge> sample_edges(shape_edges.begin(), shape_edges.end());
        if (!curved_edges.empty()) {
            const auto corners = OrientedBoundBox(*this).corners();
            for (int i = 0; i < 4; ++i) sample_edges.push_back(Edge::make_line(corners.at(i), corners.at((i + 1) % 4)));
        }
        for (const auto& e : sample_edges) {
            for (double p : {0.0, 0.5, 1.0}) cross_dirs.push_back((e.position_at(p) - cog).normalized());
        }
    }
    // a set of directions (build123d's set order is arbitrary; first occurrence order here)
    std::vector<Vector> symmetry_dirs;
    const auto category = [](const ShapeList<Shape>& side) { return side.empty() ? 0 : (side.size() == 1 ? 1 : 2); };
    double intersect_area = 0.0;
    for (const auto& cross_dir : cross_dirs) {
        // Split the face by the potential axis and flip the top
        const Plane split_plane(cog, cross_dir, cross_dir.cross(normal));
        const SplitResult halves = split(split_plane, Keep::BOTH);
        if (category(halves.top) != category(halves.bottom)) continue;  // exit early if not same
        if (halves.top.empty() || halves.bottom.empty()) continue;
        if (halves.top.size() != halves.bottom.size()) continue;
        const Axis sort_axis(cog, cross_dir);
        const auto bottom_list = halves.bottom.sort_by(sort_axis);
        ShapeList<Shape> top_flipped;
        for (const auto& f : halves.top) top_flipped.push_back(Face(f.wrapped()).mirror(split_plane));
        top_flipped = top_flipped.sort_by(sort_axis);
        double bottom_area = 0.0;
        for (const auto& f : bottom_list) bottom_area += f.area();
        bool missed = false;
        for (std::size_t i = 0; i < top_flipped.size(); ++i) {
            const auto idx = static_cast<std::ptrdiff_t>(i);
            const auto intersection = top_flipped[idx].intersect(IntersectArg(bottom_list[idx]));
            if (intersection.empty()) {
                intersect_area = -1.0;
                missed = true;
                break;
            }
            intersect_area = 0.0;
            for (const auto& f : ShapeList<Shape>(intersection).faces()) intersect_area += f.area();
        }
        if (missed || intersect_area == -1.0) continue;
        // Are the top/bottom the same?
        if (std::abs(intersect_area - bottom_area) < TOLERANCE) {
            const bool known = std::any_of(symmetry_dirs.begin(), symmetry_dirs.end(),
                                           [&](const Vector& d) { return d == cross_dir; });
            if (symmetry_dirs.empty()) {
                symmetry_dirs.push_back(cross_dir);
            } else {
                const bool opposite = std::any_of(symmetry_dirs.begin(), symmetry_dirs.end(),
                                                  [&](const Vector& d) { return d.dot(cross_dir) < -1 + TOLERANCE; });
                if (!opposite && !known) symmetry_dirs.push_back(cross_dir);
            }
        }
    }
    std::vector<Axis> symmetry_axes;
    for (const auto& d : symmetry_dirs) symmetry_axes.emplace_back(cog, d);
    return symmetry_axes;
}

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

Location Shell::location_at(const Vector& surface_point, std::optional<Vector> x_dir) const {
    // the closest face gives the location
    const auto face = faces().sort_by([&](const Face& f) { return f.distance_to(surface_point); })[0];
    return face.location_at(surface_point, x_dir);
}

std::vector<std::vector<Wire>> sort_wires_by_build_order(const std::vector<Wire>& wire_list) {
    // check if we have something to sort at all
    if (wire_list.size() < 2) return {wire_list};
    // make a Face, NB: this might return a compound of faces
    const Face faces(wire_list[0], std::vector<Wire>(wire_list.begin() + 1, wire_list.end()));
    std::vector<std::vector<Wire>> return_value;
    for (const auto& face : faces.faces()) {
        std::vector<Wire> group{face.outer_wire()};
        for (const auto& w : face.inner_wires()) group.push_back(w);
        return_value.push_back(group);
    }
    return return_value;
}

// ---------------------------------------------------------------------------
// Mixin3D
// ---------------------------------------------------------------------------

TopoDS_Shape Mixin3D::dprism_topods(const std::optional<Face>& basis, const std::vector<Face>& faces,
                                    std::optional<double> depth, double taper, const std::optional<Face>& up_to_face,
                                    bool thru_all, bool additive) const {
    TopoDS_Shape shape = obj_;
    for (const auto& face : faces) {
        shape = occt("dprism", [&] {
            BRepFeat_MakeDPrism feat(shape, TopoDS::Face(face.wrapped()),
                                     basis ? TopoDS::Face(basis->wrapped()) : TopoDS_Face(), taper * DEG2RAD,
                                     additive ? 1 : 0, false);
            if (up_to_face) {
                feat.Perform(up_to_face->wrapped());
            } else if (thru_all || !depth) {
                feat.PerformThruAll();
            } else {
                feat.Perform(*depth);
            }
            return feat.Shape();
        });
    }
    return unwrap(shape, true);
}

TopoDS_Shape Mixin3D::hollow_topods(const std::vector<Face>& faces, double thickness, double tolerance,
                                    Kind kind) const {
    if (kind == Kind::TANGENT) throw std::invalid_argument("Kind.TANGENT not supported");
    return occt("hollow", [&]() -> TopoDS_Shape {
        TopTools_ListOfShape occ_faces_list;
        for (const auto& face : faces) occ_faces_list.Append(face.wrapped());
        BRepOffsetAPI_MakeThickSolid shell_builder;
        shell_builder.MakeThickSolidByJoin(obj_, occ_faces_list, thickness, tolerance, BRepOffset_Skin, true, false,
                                           kind == Kind::ARC ? GeomAbs_Arc : GeomAbs_Intersection);
        shell_builder.Build();
        if (!faces.empty()) return unwrap(shell_builder.Shape(), true);
        // if no faces provided a watertight solid will be constructed
        const TopoDS_Shell shell1 = TopoDS::Shell(Shape(shell_builder.Shape()).shells()[0].wrapped());
        const TopoDS_Shell shell2 = TopoDS::Shell(shells()[0].wrapped());
        // shell1 can be the outer or inner shell depending on the thickness sign
        BRepBuilderAPI_MakeSolid sol = thickness > 0 ? BRepBuilderAPI_MakeSolid(shell1, shell2)
                                                     : BRepBuilderAPI_MakeSolid(shell2, shell1);
        // fix needed for the orientations (Shape.fix: only an invalid shape is fixed)
        const TopoDS_Shape solid = sol.Shape();
        return BRepCheck_Analyzer(solid).IsValid() ? solid : fix_shape(solid);
    });
}

// ---------------------------------------------------------------------------
// Solid
// ---------------------------------------------------------------------------

Solid Solid::extrude_linear_with_rotation(const Wire& section, const Vector& center, const Vector& normal, double angle,
                                          const std::vector<Wire>& inner_wires) {
    // A 'twisted prism': each wire swept along a straight spine with a helix as auxiliary
    // spine, the holes cut out of the outer solid
    const auto extrude_aux_spine = [](const TopoDS_Wire& wire, const TopoDS_Wire& spine, const TopoDS_Wire& aux_spine) {
        BRepOffsetAPI_MakePipeShell extrude_builder(spine);
        extrude_builder.SetMode(aux_spine, false);  // auxiliary spine
        extrude_builder.Add(wire);
        extrude_builder.Build();
        extrude_builder.MakeSolid();
        return extrude_builder.Shape();
    };
    return Solid(occt("extrude_linear_with_rotation", [&] {
        // make straight spine
        const Edge straight_spine_e = Edge::make_line(center, center.add(normal));
        const Wire straight_spine_w = Wire::combine(std::vector<Edge>{straight_spine_e})[0];
        // make an auxiliary spine
        const double pitch = 360.0 / angle * normal.length();
        const Wire aux_spine_w(std::vector<Edge>{Edge::make_helix(pitch, normal.length(), 1, center, normal)});
        const TopoDS_Wire& spine = TopoDS::Wire(straight_spine_w.wrapped());
        const TopoDS_Wire& aux = TopoDS::Wire(aux_spine_w.wrapped());
        // extrude the outer wire and the inner wires
        const TopoDS_Shape outer_solid = extrude_aux_spine(TopoDS::Wire(section.wrapped()), spine, aux);
        std::vector<TopoDS_Shape> inner_solids;
        for (const auto& w : inner_wires) inner_solids.push_back(extrude_aux_spine(TopoDS::Wire(w.wrapped()), spine, aux));
        // subtract the compound of the inner solids from the outer solid
        const TopoDS_Shape difference = BRepAlgoAPI_Cut(outer_solid, make_compound(inner_solids)).Shape();
        return unwrap(difference, true);
    }));
}

Solid Solid::revolve(const Wire& section, double angle, const Axis& axis, const std::vector<Wire>& inner_wires) {
    return revolve(Face(section, inner_wires), angle, axis);
}

Solid Solid::sweep(const Wire& section, const Shape& path, const std::vector<Wire>& inner_wires, bool make_solid,
                   bool is_frenet, std::optional<Vector> binormal_dir, const std::optional<Wire>& binormal_wire,
                   Transition transition) {
    std::vector<Solid> shapes;
    std::vector<Wire> wires{section};
    wires.insert(wires.end(), inner_wires.begin(), inner_wires.end());
    for (const auto& wire : wires) {
        shapes.push_back(sweep(wire, path, make_solid, is_frenet, binormal_dir, binormal_wire, transition));
    }
    if (shapes.size() == 1) return shapes[0];
    const Shape hollow = shapes[0].cut(std::vector<Shape>(shapes.begin() + 1, shapes.end()));
    return Solid(unwrap(hollow.wrapped(), true));
}

// ---------------------------------------------------------------------------
// Compound
// ---------------------------------------------------------------------------

std::tuple<bool, std::pair<std::optional<Shape>, std::optional<Shape>>, double> Compound::do_children_intersect(
    bool include_parent, double tolerance) const {
    // anytree's PreOrderIter over the assembly
    std::vector<Shape> nodes;
    std::function<void(const Shape&)> visit = [&](const Shape& node) {
        nodes.push_back(node);
        for (const auto& child : node.children) visit(child);
    };
    visit(*this);
    if (!include_parent) nodes.erase(nodes.begin());
    std::vector<Solid> children_bbox;
    for (const auto& child : nodes) children_bbox.push_back(Solid::from_bounding_box(child.bounding_box()));
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        for (std::size_t j = i + 1; j < nodes.size(); ++j) {
            // First check for bounding box intersections, then the actual objects
            if (children_bbox[i].intersect(IntersectArg(children_bbox[j])).empty()) continue;
            const auto obj_intersection = nodes[i].intersect(IntersectArg(nodes[j]));
            if (obj_intersection.empty()) continue;
            double common_volume = 0.0;
            for (const auto& s : ShapeList<Shape>(obj_intersection).solids()) common_volume += s.volume();
            if (common_volume > tolerance) return {true, {nodes[i], nodes[j]}, common_volume};
        }
    }
    return {false, {std::nullopt, std::nullopt}, 0.0};
}

Compound Compound::extrude(const Shell& obj, const Vector& direction) {
    return Compound(extrude_shape(obj.wrapped(), direction));
}

Curve Compound::make_triad(double axes_scale) {
    const Edge x_axis = Edge::make_line({0, 0, 0}, {axes_scale, 0, 0});
    const Edge y_axis = Edge::make_line({0, 0, 0}, {0, axes_scale, 0});
    const Edge z_axis = Edge::make_line({0, 0, 0}, {0, 0, axes_scale});
    const Edge arrow_arc =
        Edge::make_spline({{0, 0, 0}, {-axes_scale / 20, axes_scale / 30, 0}}, {{-1, 0, 0}, {-1, 1.5, 0}});
    const Wire arrow(std::vector<Edge>{arrow_arc, arrow_arc.mirror(Plane::XZ)});
    const auto label = [&](const std::string& text, Align2 align) {
        return Compound::make_text(text, axes_scale / 4, "singleline", "", FontStyle::REGULAR,
                                   {TextAlign::CENTER, TextAlign::CENTER}, align);
    };
    const auto x_label = label("X", Align2(Align::MIN, Align::CENTER)).moved(Location(x_axis.position_at(1))).edges();
    const auto y_label = label("Y", Align2(Align::MIN, Align::CENTER))
                             .rotate(Axis::Z, 90)
                             .moved(Location(y_axis.position_at(1)))
                             .edges();
    const auto z_label = label("Z", Align2(Align::CENTER, Align::MIN))
                             .rotate(Axis::Y, 90)
                             .rotate(Axis::X, 90)
                             .moved(Location(z_axis.position_at(1)))
                             .edges();
    std::vector<Shape> parts{x_axis,
                             y_axis,
                             z_axis,
                             arrow.moved(Location(x_axis.position_at(1))),
                             arrow.rotate(Axis::Z, 90).moved(Location(y_axis.position_at(1))),
                             arrow.rotate(Axis::Y, -90).moved(Location(z_axis.position_at(1)))};
    for (const auto* labels : {&x_label, &y_label, &z_label}) parts.insert(parts.end(), labels->begin(), labels->end());
    return Curve(parts);
}

}  // namespace build123d
