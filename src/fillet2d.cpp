// 2D fillets and chamfers of wires and faces as build123d 0.13 makes them
// (one_d.py: _fillet_wire_corner and its helpers, Wire.fillet_2d/chamfer_2d;
// two_d.py: Face.fillet_2d/chamfer_2d), each recording its ShapeHistory.
#include <cmath>

#include <BOPAlgo_Splitter.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GccEnt_Position.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dGcc_Circ2d2TanRad.hxx>
#include <Geom2dGcc_QualifiedCurve.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Plane.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Pln.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

// ---------------------------------------------------------------------------
// The part of constrained_lines.py that the wire fillet uses (_make_2tan_rad_arcs)
// ---------------------------------------------------------------------------

/// Map parameter u into [first, first + period).
double norm_on_period(double u, double first, double period) {
    if (std::abs(u - first) < TOLERANCE) return first;
    return py_float_mod(u - first, period) + first;
}

/// Forward (positive) delta from u1 to u2 on a periodic domain anchored at first.
double forward_delta(double u1, double u2, double first, double period) {
    const double u1n = norm_on_period(u1, first, period);
    const double u2n = norm_on_period(u2, first, period);
    double delta = u2n - u1n;
    if (delta < 0.0) delta += period;
    return delta;
}

const gp_Pln& pln_xy() {
    static const gp_Pln plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)));
    return plane;
}

const Handle(Geom_Plane)& surf_xy() {
    static const Handle(Geom_Plane) surface = new Geom_Plane(pln_xy());
    return surface;
}

/// A 3D edge on XY from a trimmed 2D circle segment [u1, u2].
TopoDS_Edge edge_from_circle(const Handle(Geom2d_Circle)& circle, double u1, double u2) {
    Handle(Geom2d_TrimmedCurve) arc2d = new Geom2d_TrimmedCurve(circle, u1, u2, true);
    TopoDS_Edge circle_edge = BRepBuilderAPI_MakeEdge(arc2d, surf_xy()).Edge();
    BRepLib::BuildCurves3d(circle_edge);
    return circle_edge;
}

/// Both the forward (minor) and the complementary (major) arcs between two circle parameters.
std::vector<TopoDS_Edge> two_arc_edges_from_params(const gp_Circ2d& circ, double u1, double u2) {
    Handle(Geom2d_Circle) h2d_circle = new Geom2d_Circle(circ);
    const double period = h2d_circle->Period();
    const double d = forward_delta(u1, u2, 0.0, period);
    const double u1n = norm_on_period(u1, 0.0, period);
    const double u2n = norm_on_period(u2, 0.0, period);
    if (d <= TOLERANCE || std::abs(period - d) <= TOLERANCE) return {};
    return {edge_from_circle(h2d_circle, u1n, u1n + d), edge_from_circle(h2d_circle, u2n, u2n + (period - d))};
}

struct Curve2d {
    Handle(Geom2d_Curve) h2d;
    double first = 0, last = 0;
};

Curve2d edge_to_2d(const TopoDS_Edge& edge) {
    double f, l;
    Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, f, l);
    double first, last;
    BRep_Tool::Range(edge, first, last);
    return {GeomAPI::To2d(curve3d, pln_xy()), first, last};
}

bool param_in_trim(double u, const Curve2d& c) {
    if (c.h2d->IsPeriodic()) u = norm_on_period(u, c.first, c.h2d->Period());
    return c.first - TOLERANCE <= u && u <= c.last + TOLERANCE;
}

/// _make_2tan_rad_arcs(edge1, edge2, radius=radius, sagitta=Sagitta.BOTH): every planar
/// arc of the radius tangent to both (XY) edges, minor and major arcs of each circle.
ShapeList<Edge> make_2tan_rad_arcs(const Edge& e1, const Edge& e2, double radius) {
    const Curve2d curves[2] = {edge_to_2d(TopoDS::Edge(e1.wrapped())), edge_to_2d(TopoDS::Edge(e2.wrapped()))};
    const Geom2dAdaptor_Curve adapt1(curves[0].h2d, curves[0].first, curves[0].last);
    const Geom2dAdaptor_Curve adapt2(curves[1].h2d, curves[1].first, curves[1].last);
    const Geom2dGcc_QualifiedCurve q1(adapt1, GccEnt_unqualified), q2(adapt2, GccEnt_unqualified);
    Geom2dGcc_Circ2d2TanRad gcc(q1, q2, radius, TOLERANCE);
    if (!gcc.IsDone() || gcc.NbSolutions() == 0) throw std::runtime_error("Unable to find a tangent arc");

    std::vector<TopoDS_Edge> solutions;
    for (int i = 1; i <= gcc.NbSolutions(); ++i) {
        const gp_Circ2d circ = gcc.ThisSolution(i);
        const gp_Pnt2d center = circ.Location();
        const double sol_radius = circ.Radius();
        double u_circ_s[2], u_arg_s[2];
        gp_Pnt2d p_tmp;
        gcc.Tangency1(i, u_circ_s[0], u_arg_s[0], p_tmp);
        gcc.Tangency2(i, u_circ_s[1], u_arg_s[1], p_tmp);
        double corrected[2];
        bool valid_solution = true;
        for (int j = 0; j < 2 && valid_solution; ++j) {
            const Curve2d& c = curves[j];
            const gp_Pnt2d p_s = c.h2d->Value(u_arg_s[j]);
            // Verify the solver's point, else correct it by projecting the center
            if (std::abs(p_s.Distance(center) - sol_radius) < TOLERANCE && param_in_trim(u_arg_s[j], c)) {
                corrected[j] = u_circ_s[j];
                continue;
            }
            Geom2dAPI_ProjectPointOnCurve proj(gp_Pnt2d(center.X(), center.Y()), c.h2d);
            if (proj.NbPoints() == 0) {
                valid_solution = false;
                break;
            }
            const double u_arg_act = proj.LowerDistanceParameter();
            const gp_Pnt2d p_act = c.h2d->Value(u_arg_act);
            if (std::abs(p_act.Distance(center) - sol_radius) > TOLERANCE || !param_in_trim(u_arg_act, c)) {
                valid_solution = false;
                break;
            }
            corrected[j] = std::atan2(p_act.Y() - center.Y(), p_act.X() - center.X());
        }
        if (!valid_solution) continue;
        for (const auto& e : two_arc_edges_from_params(circ, corrected[0], corrected[1])) solutions.push_back(e);
    }
    ShapeList<Edge> out;
    for (const auto& e : solutions) out.push_back(Edge(e));
    return out;
}

// ---------------------------------------------------------------------------
// _fillet_wire_corner and its helpers (one_d.py)
// ---------------------------------------------------------------------------

struct WireFilletCorner {
    Wire wire;
    Vertex vertex;
    ShapeList<Edge> all_edges;
    ShapeList<Edge> connected_edges;
    std::vector<std::size_t> connected_edge_indices;
};

struct WireFilletSolution {
    std::vector<TopoDS_Edge> trimmed_topods_edges;  // null where the edge was consumed
    TopoDS_Edge fillet_topods_edge;
};

std::string vertex_label(const Vertex& v) {
    return "Vertex(" + std::to_string(v.X()) + ", " + std::to_string(v.Y()) + ", " + std::to_string(v.Z()) + ")";
}

WireFilletCorner analyze_wire_fillet_corner(const Wire& wire, const Vertex& vertex) {
    WireFilletCorner corner{wire, vertex, wire.edges(), {}, {}};
    for (std::size_t i = 0; i < corner.all_edges.size(); ++i) {
        const Edge& edge = corner.all_edges[static_cast<std::ptrdiff_t>(i)];
        bool shares = false;
        for (const auto& ev : edge.vertices()) shares = shares || ev.wrapped().IsSame(vertex.wrapped());
        if (shares) corner.connected_edges.push_back(edge);
    }
    if (corner.connected_edges.empty()) {
        throw std::invalid_argument("Could not find shared vertex on wire: " + vertex_label(vertex));
    }
    if (corner.connected_edges.size() != 2) {
        throw std::invalid_argument("Vertex must connect exactly two edges: " + vertex_label(vertex));
    }
    for (const auto& e : corner.connected_edges) {
        for (std::size_t i = 0; i < corner.all_edges.size(); ++i) {
            if (corner.all_edges[static_cast<std::ptrdiff_t>(i)].is_same(e)) {
                corner.connected_edge_indices.push_back(i);
                break;
            }
        }
    }
    return corner;
}

bool topods_edge_contains_vertex(const TopoDS_Edge& edge, const TopoDS_Vertex& vertex) {
    const gp_Pnt p = BRep_Tool::Pnt(vertex);
    return BRep_Tool::Pnt(TopExp::FirstVertex(edge)).Distance(p) < TOLERANCE ||
           BRep_Tool::Pnt(TopExp::LastVertex(edge)).Distance(p) < TOLERANCE;
}

std::vector<TopoDS_Edge> split_edge_at_vertex(const Edge& edge, const Vertex& split_vertex) {
    BOPAlgo_Splitter splitter;
    splitter.AddArgument(edge.wrapped());
    splitter.AddTool(split_vertex.wrapped());
    splitter.Perform();
    std::vector<TopoDS_Edge> split_edges;
    for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
        split_edges.push_back(TopoDS::Edge(exp.Current()));
    }
    return split_edges;
}

bool wire_fillet_corner_is_tangent_continuous(const WireFilletCorner& corner) {
    const Edge& e0 = corner.connected_edges[0];
    const Edge& e1 = corner.connected_edges[1];
    const Vector v(corner.vertex);
    const Vector tangent0 = e0.tangent_at(v), tangent1 = e1.tangent_at(v);
    if (tangent0.cross(tangent1).length() > TOLERANCE) return false;
    double sample_length = std::min(e0.length(), e1.length()) * 1e-3;
    sample_length = std::min({sample_length, e0.length() * 0.25, e1.length() * 0.25});
    sample_length = std::max(sample_length, TOLERANCE * 100);
    Vector reflected[2];
    for (int i = 0; i < 2; ++i) {
        const Edge& edge = corner.connected_edges[i];
        Vector sample_point;
        if (TopExp::FirstVertex(TopoDS::Edge(edge.wrapped())).IsSame(corner.vertex.wrapped())) {
            sample_point = edge.position_at(sample_length, PositionMode::LENGTH);
        } else {
            sample_point = edge.position_at(edge.length() - sample_length, PositionMode::LENGTH);
        }
        reflected[i] = v * 2 - sample_point;
    }
    return std::min(e1.distance_to(reflected[0]), e0.distance_to(reflected[1])) <=
           std::max(TOLERANCE * 10, sample_length * 0.25);
}

std::optional<WireFilletSolution> solve_wire_fillet_corner(const WireFilletCorner& corner, double radius) {
    const ShapeList<Edge> fillet_arcs =
        make_2tan_rad_arcs(corner.connected_edges[0], corner.connected_edges[1], radius);
    if (fillet_arcs.empty()) return std::nullopt;
    const Edge fillet_arc = fillet_arcs.sort_by_distance(static_cast<const Shape&>(corner.vertex))[0];
    WireFilletSolution solution;
    for (const auto& connected_edge : corner.connected_edges) {
        Vertex other_vertex;
        for (const auto& v : connected_edge.vertices()) {
            if (!v.is_same(corner.vertex)) {
                other_vertex = v;
                break;
            }
        }
        const Vertex fillet_vertex = fillet_arc.vertices().sort_by_distance(static_cast<const Shape&>(connected_edge))[0];
        const Vertex split_vertex(TopoDS_Shape(BRepBuilderAPI_MakeVertex(Vector(fillet_vertex).to_pnt()).Vertex()));
        TopoDS_Edge edge_result;
        for (const auto& edge : split_edge_at_vertex(connected_edge.copy(), split_vertex)) {
            if (topods_edge_contains_vertex(edge, TopoDS::Vertex(other_vertex.wrapped())) &&
                !topods_edge_contains_vertex(edge, TopoDS::Vertex(corner.vertex.wrapped()))) {
                edge_result = edge;
                break;
            }
        }
        solution.trimmed_topods_edges.push_back(edge_result);
    }
    solution.fillet_topods_edge = TopoDS::Edge(fillet_arc.wrapped());
    return solution;
}

void record_matching_vertices(ShapeHistory& history, const TopoDS_Shape& source, const TopoDS_Shape& placed) {
    std::vector<TopoDS_Shape> placed_vertices;
    for (const auto& v : tracked_subshapes({placed})) {
        if (v.ShapeType() == TopAbs_VERTEX) placed_vertices.push_back(v);
    }
    for (const auto& old : tracked_subshapes({source})) {
        if (old.ShapeType() != TopAbs_VERTEX) continue;
        const gp_Pnt old_point = BRep_Tool::Pnt(TopoDS::Vertex(old));
        for (const auto& added : placed_vertices) {
            if (added.IsSame(old)) break;
            if (BRep_Tool::Pnt(TopoDS::Vertex(added)).Distance(old_point) < TOLERANCE) {
                history.wrapped()->AddModified(old, added);
                break;
            }
        }
    }
}

/// Replace the two connected edges with the trimmed ones and the fillet and rebuild the
/// wire, recording the corner: the vertex removed, the edges trimmed, the arc generated
/// from the vertex, every other edge modified into MakeWire's copy of it.
Wire splice_wire_fillet_corner(const WireFilletCorner& corner, WireFilletSolution solution) {
    std::vector<TopoDS_Shape> all_topods_edges;
    for (const auto& e : corner.all_edges) all_topods_edges.push_back(e.wrapped());
    std::vector<TopoDS_Shape> sources = all_topods_edges;  // null for the arc
    auto history = std::make_shared<ShapeHistory>();

    std::vector<std::size_t> indices_to_remove;
    for (std::size_t i = 0; i < solution.trimmed_topods_edges.size(); ++i) {
        const std::size_t edge_idx = corner.connected_edge_indices[i];
        TopoDS_Edge& trimmed = solution.trimmed_topods_edges[i];
        if (trimmed.IsNull()) {
            if (std::find(indices_to_remove.begin(), indices_to_remove.end(), edge_idx) == indices_to_remove.end()) {
                indices_to_remove.push_back(edge_idx);
            }
            history->wrapped()->Remove(corner.connected_edges[static_cast<std::ptrdiff_t>(i)].wrapped());
            continue;
        }
        if (trimmed.Orientation() != corner.connected_edges[static_cast<std::ptrdiff_t>(i)].wrapped().Orientation()) {
            trimmed.Reverse();
        }
        all_topods_edges[edge_idx] = trimmed;
    }
    // insert index before removal (in the original index space)
    const std::size_t n = all_topods_edges.size();
    std::size_t insert_index = corner.connected_edge_indices[1] == (corner.connected_edge_indices[0] + 1) % n
                                   ? corner.connected_edge_indices[0] + 1
                                   : corner.connected_edge_indices[1] + 1;
    std::sort(indices_to_remove.rbegin(), indices_to_remove.rend());
    for (std::size_t idx : indices_to_remove) {
        all_topods_edges.erase(all_topods_edges.begin() + static_cast<std::ptrdiff_t>(idx));
        sources.erase(sources.begin() + static_cast<std::ptrdiff_t>(idx));
        if (idx < insert_index) --insert_index;
    }
    all_topods_edges.insert(all_topods_edges.begin() + static_cast<std::ptrdiff_t>(insert_index),
                            solution.fillet_topods_edge);
    sources.insert(sources.begin() + static_cast<std::ptrdiff_t>(insert_index), TopoDS_Shape());

    // MakeWire copies each edge so that neighbours share vertices: add them one at a
    // time and record each as it was placed in the wire
    const TopoDS_Shape& corner_vertex = corner.vertex.wrapped();
    return occt("fillet_2d", [&] {
        BRepBuilderAPI_MakeWire wire_builder;
        for (std::size_t i = 0; i < all_topods_edges.size(); ++i) {
            wire_builder.Add(TopoDS::Edge(all_topods_edges[i]));
            const TopoDS_Edge placed = wire_builder.Edge();
            if (sources[i].IsNull()) {
                history->wrapped()->AddGenerated(corner_vertex, placed);
                for (const auto& arc_end : tracked_subshapes({placed})) {
                    if (arc_end.ShapeType() == TopAbs_VERTEX) history->wrapped()->AddGenerated(corner_vertex, arc_end);
                }
            } else {
                if (!placed.IsSame(sources[i])) history->wrapped()->AddModified(sources[i], placed);
                record_matching_vertices(*history, sources[i], placed);
            }
        }
        history->wrapped()->Remove(corner_vertex);
        wire_builder.Build();
        return made_by(Wire(wire_builder.Wire()), history);
    });
}

/// Fillet a single planar (XY) wire corner; the same wire if the corner is smooth.
Wire fillet_wire_corner(const Wire& wire, const Vertex& vertex, double radius, bool& changed) {
    changed = false;
    const WireFilletCorner corner = analyze_wire_fillet_corner(wire, vertex);
    if (wire_fillet_corner_is_tangent_continuous(corner)) return wire;
    auto solution = solve_wire_fillet_corner(corner, radius);
    if (solution) {
        Wire new_wire = splice_wire_fillet_corner(corner, std::move(*solution));
        if (!wire.is_closed() || new_wire.is_closed()) {
            changed = true;
            return new_wire;
        }
        throw std::invalid_argument("Filleting failed to create a closed wire.");
    }
    throw std::invalid_argument("Fillet algorithm failed for " + vertex_label(vertex) + " with radius " +
                                std::to_string(radius));
}

}  // namespace

// ---------------------------------------------------------------------------
// Wire
// ---------------------------------------------------------------------------

Wire Wire::fillet_2d(double radius, const std::vector<Vertex>& vertices) const {
    if (is_null()) throw std::invalid_argument("Can't fillet an empty wire");
    const auto wire_pln = common_plane();
    if (!wire_pln) throw std::invalid_argument("Wire is not planar " + to_string());

    // Force the wire to Plane.XY for the fillet operation
    Wire filleted_wire = wire_pln->to_local_coords(*this);
    auto record = ShapeHistory::from_relocation(obj_, filleted_wire.wrapped());
    for (const auto& vertex : vertices) {
        const Vertex vertex_local = wire_pln->to_local_coords(vertex);
        const Vertex current_vertex =
            filleted_wire.vertices().sort_by_distance(static_cast<const Shape&>(vertex_local))[0];
        if ((Vector(current_vertex) - Vector(vertex_local)).length() > TOLERANCE) {
            throw std::invalid_argument("Could not find fillet vertex on wire: " + vertex_label(vertex));
        }
        bool changed = false;
        filleted_wire = fillet_wire_corner(filleted_wire, current_vertex, radius, changed);
        if (changed) record->merge(filleted_wire.history());
    }

    // Return the filleted wire to the wire plane
    const Wire globalized = wire_pln->from_local_coords(filleted_wire);
    record->merge(ShapeHistory::from_relocation(filleted_wire.wrapped(), globalized.wrapped()));

    // Transform the wire back to the original location, not that of the wire plane
    const Location old_loc = globalized.location();
    const Location new_loc(obj_.Location());
    const gp_Trsf trsf = (new_loc.inverse() * old_loc).wrapped().Transformation();
    const TopoDS_Shape base_wire = globalized.wrapped().Located(TopLoc_Location());
    record->merge(ShapeHistory::from_relocation(globalized.wrapped(), base_wire));
    const TopoDS_Shape transformed = occt("fillet_2d", [&] {
        BRepBuilderAPI_Transform transformer(base_wire, trsf, true);
        const TopoDS_Shape shape = transformer.Shape();
        record->merge(ShapeHistory::from_algorithm(transformer, {base_wire}, shape));
        return shape;
    });
    Wire final_wire(transformed.Located(obj_.Location()));
    record->merge(ShapeHistory::from_relocation(transformed, final_wire.wrapped()));

    // Ensure the wire direction is the same
    if (is_forward() != final_wire.is_forward()) ShapeAccess::wrapped(final_wire).Reverse();
    return made_by(final_wire, record);
}

Wire Wire::chamfer_2d(double distance, double distance2, const std::vector<Vertex>& vertices,
                      std::optional<Edge> reference_edge) const {
    if (is_null()) throw std::invalid_argument("Can't chamfer empty wire");
    // A face to chamfer
    const TopoDS_Face unchamfered_face = make_face_from_wires(TopoDS::Wire(obj_));
    return occt("chamfer_2d", [&] {
        BRepFilletAPI_MakeFillet2d chamfer_builder(unchamfered_face);
        OccShapeListMap vertex_edge_map;
        TopExp::MapShapesAndAncestors(unchamfered_face, TopAbs_VERTEX, TopAbs_EDGE, vertex_edge_map);
        for (const auto& v : vertices) {
            if (v.is_null()) continue;
            const OccShapeList& edge_list = vertex_edge_map.FindFromKey(v.wrapped());
            const auto [edge1, edge2] =
                Wire::order_chamfer_edges(reference_edge, {Edge(edge_list.First()), Edge(edge_list.Last())});
            chamfer_builder.AddChamfer(TopoDS::Edge(edge1.wrapped()), TopoDS::Edge(edge2.wrapped()), distance,
                                       distance2);
        }
        chamfer_builder.Build();
        ShapeFix_Shape shape_fix(chamfer_builder.Shape());
        shape_fix.Perform();
        const TopoDS_Shape chamfered_face = shape_fix.Shape();
        if (chamfered_face.ShapeType() != TopAbs_FACE) {
            throw std::runtime_error("An internal error occured creating the chamfer");
        }
        // The outer wire
        Wire result(BRepTools::OuterWire(TopoDS::Face(chamfered_face)));
        return made_by(result, ShapeHistory::from_algorithm(chamfer_builder, {obj_}, result.wrapped()));
    });
}

// ---------------------------------------------------------------------------
// Face
// ---------------------------------------------------------------------------

Face Face::fillet_2d(double radius, const std::vector<Vertex>& vertices_in) const {
    std::vector<Vertex> vertices;
    for (const auto& v : vertices_in) {
        if (!v.is_null()) vertices.push_back(v);
    }
    if (vertices.empty()) return *this;

    std::vector<Wire> wires{outer_wire()};
    for (const auto& w : inner_wires()) wires.push_back(w);
    std::vector<Wire> filleted_wires;
    auto record = std::make_shared<ShapeHistory>();
    for (const auto& wire : wires) {
        const auto wire_vertices = wire.vertices();
        std::vector<Vertex> vertices_in_wire;
        for (const auto& vertex : vertices) {
            for (const auto& wv : wire_vertices) {
                if (wv.wrapped().IsSame(vertex.wrapped())) {
                    vertices_in_wire.push_back(vertex);
                    break;
                }
            }
        }
        if (!vertices_in_wire.empty()) {
            const Wire filleted = wire.fillet_2d(radius, vertices_in_wire);
            record->merge(filleted.history());
            filleted_wires.push_back(filleted);
        } else {
            filleted_wires.push_back(wire);
        }
    }
    Face filleted_face(filleted_wires[0], std::vector<Wire>(filleted_wires.begin() + 1, filleted_wires.end()));
    if (normal_at() != filleted_face.normal_at()) filleted_face = -filleted_face;
    record->add_modified(obj_, filleted_face.wrapped());
    return made_by(filleted_face, record);
}

Face Face::chamfer_2d(double distance, double distance2, const std::vector<Vertex>& vertices,
                      std::optional<Edge> reference_edge) const {
    return occt("chamfer_2d", [&] {
        BRepFilletAPI_MakeFillet2d chamfer_builder(TopoDS::Face(obj_));
        OccShapeListMap vertex_edge_map;
        TopExp::MapShapesAndAncestors(obj_, TopAbs_VERTEX, TopAbs_EDGE, vertex_edge_map);
        for (const auto& v : vertices) {
            const OccShapeList& edge_list = vertex_edge_map.FindFromKey(v.wrapped());
            const auto [edge1, edge2] =
                Wire::order_chamfer_edges(reference_edge, {Edge(edge_list.First()), Edge(edge_list.Last())});
            chamfer_builder.AddChamfer(TopoDS::Edge(edge1.wrapped()), TopoDS::Edge(edge2.wrapped()), distance,
                                       distance2);
        }
        chamfer_builder.Build();
        Face result = Face(chamfer_builder.Shape()).fix();
        auto history = ShapeHistory::from_algorithm(chamfer_builder, {obj_}, result.wrapped());
        history->add_modified(obj_, result.wrapped());
        return made_by(result, history);
    });
}

}  // namespace build123d
