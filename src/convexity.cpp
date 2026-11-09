// Convexity (build123d 0.13): Vertex.convexity (zero_d.py), Mixin1D.convexity and
// is_interior with _dihedral_convexity (one_d.py), Face.convexity and
// is_circular_convex/concave (two_d.py), Shape.convexity (shape_core.py).
#include <cmath>
#include <set>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <IntTools_FClass2d.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

Convexity combine(const std::set<Convexity>& kinds) {
    std::set<Convexity> turning = kinds;
    turning.erase(Convexity::SMOOTH);
    if (turning.empty()) return Convexity::SMOOTH;
    if (turning.size() == 1) return *turning.begin();
    return Convexity::SADDLE;
}

// ---- Edge (Mixin1D.convexity)

/// Does `face` meet itself along `edge`, as a cylinder does at its seam?
bool edge_is_seam_of(const TopoDS_Shape& edge, const TopoDS_Face& face) {
    int count = 0;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        if (exp.Current().IsSame(edge)) ++count;
    }
    return count > 1;
}

/// `edge` with the orientation it has in `face`'s boundary.
TopoDS_Edge edge_as_bounded_by(const TopoDS_Face& face, const TopoDS_Shape& edge) {
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        if (exp.Current().IsSame(edge)) return TopoDS::Edge(exp.Current());
    }
    throw std::invalid_argument("edge is not on the face");
}

/// Classify the crease between two faces along their shared edge, sampling the
/// direction leading from the edge into the second face against the first face's
/// tangent plane.
Convexity dihedral_convexity(const TopoDS_Edge& edge, const TopoDS_Face& face1, const TopoDS_Face& face2,
                             int samples = 7, double sin_tolerance = 1e-4) {
    const BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter(), last = curve.LastParameter();
    double f1, l1, f2, l2;
    Handle(Geom2d_Curve) pcurve1 = BRep_Tool::CurveOnSurface(edge, face1, f1, l1);
    Handle(Geom2d_Curve) pcurve2 = BRep_Tool::CurveOnSurface(edge, face2, f2, l2);
    if (pcurve1.IsNull() || pcurve2.IsNull()) throw std::invalid_argument("the edge has no curve on a face");
    const bool reversed_in_face2 = edge_as_bounded_by(face2, edge).Orientation() == TopAbs_REVERSED;
    BRepGProp_Face props1(face1), props2(face2);
    std::set<Convexity> kinds;
    for (int i = 0; i < samples; ++i) {
        const double param = first + (last - first) * (i + 0.5) / samples;
        gp_Pnt point;
        gp_Vec tangent;
        curve.D1(param, point, tangent);
        const gp_Pnt2d uv1 = pcurve1->Value(param), uv2 = pcurve2->Value(param);
        gp_Vec normal1, normal2;
        props1.Normal(uv1.X(), uv1.Y(), point, normal1);
        props2.Normal(uv2.X(), uv2.Y(), point, normal2);
        if (std::min({tangent.Magnitude(), normal1.Magnitude(), normal2.Magnitude()}) < 1e-12) continue;
        tangent.Normalize();
        normal1.Normalize();
        normal2.Normalize();
        if (normal1.Crossed(normal2).Magnitude() < sin_tolerance) {
            kinds.insert(Convexity::SMOOTH);
            continue;
        }
        if (reversed_in_face2) tangent.Reverse();
        const gp_Vec into_face2 = normal2.Crossed(tangent);
        kinds.insert(into_face2.Dot(normal1) < 0 ? Convexity::CONVEX : Convexity::CONCAVE);
    }
    if (kinds.empty()) throw std::invalid_argument("the faces are degenerate along this edge");
    return combine(kinds);
}

Convexity edge_convexity(const Shape& edge) {
    const auto faces = topo_explore_connected_faces(edge);
    if (faces.size() == 1) {
        if (edge_is_seam_of(edge.wrapped(), faces[0])) return Convexity::SMOOTH;
        throw std::invalid_argument(
            "this edge lies on one face only, so there is no crease to classify - convexity needs an edge "
            "shared by two faces");
    }
    if (faces.size() != 2) {
        throw std::invalid_argument(std::to_string(faces.size()) +
                                    " faces meet at this edge - convexity needs exactly two");
    }
    return dihedral_convexity(TopoDS::Edge(edge.wrapped()), faces[0], faces[1]);
}

// ---- Vertex (Vertex.convexity)

/// The innermost face this vertex was selected through, if any.
std::optional<TopoDS_Face> owning_face(const Shape& vertex) {
    const auto path = vertex.topo_path();
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        if (!it->is_null() && it->wrapped().ShapeType() == TopAbs_FACE) return TopoDS::Face(it->wrapped());
    }
    return std::nullopt;
}

/// Unit directions leading away from the vertex in the face's uv.
std::vector<std::array<double, 2>> face_tangents(const TopoDS_Vertex& vertex, const TopoDS_Face& face) {
    const gp_Pnt2d here = BRep_Tool::Parameters(vertex, face);
    std::vector<TopoDS_Shape> seen;
    std::vector<std::array<double, 2>> directions;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        if (std::any_of(seen.begin(), seen.end(), [&](const TopoDS_Shape& o) { return edge.IsSame(o); })) {
            continue;  // a seam edge is met once per side
        }
        seen.push_back(edge);
        const BRepAdaptor_Curve2d curve(edge, face);
        for (const auto& [param, sign] : {std::pair<double, double>{curve.FirstParameter(), 1.0},
                                          std::pair<double, double>{curve.LastParameter(), -1.0}}) {
            const gp_Pnt2d end = curve.Value(param);
            if ((end.X() - here.X()) * (end.X() - here.X()) + (end.Y() - here.Y()) * (end.Y() - here.Y()) > TOLERANCE) {
                continue;
            }
            gp_Pnt2d point;
            gp_Vec2d tangent;
            curve.D1(param, point, tangent);
            const double length = std::sqrt(tangent.X() * tangent.X() + tangent.Y() * tangent.Y());
            if (length > 0) directions.push_back({sign * tangent.X() / length, sign * tangent.Y() / length});
        }
    }
    return directions;
}

/// Which way the face's boundary turns at this corner.
Convexity corner_convexity(const TopoDS_Vertex& vertex, const TopoDS_Face& face) {
    const auto directions = face_tangents(vertex, face);
    if (directions.size() != 2) {
        throw std::invalid_argument(std::to_string(directions.size()) +
                                    " edge end(s) meet this vertex on the face, expected 2 - a corner is where "
                                    "exactly two of them do");
    }
    const double bisector[2] = {directions[0][0] + directions[1][0], directions[0][1] + directions[1][1]};
    const double span = std::sqrt(bisector[0] * bisector[0] + bisector[1] * bisector[1]);
    if (span < TOLERANCE) return Convexity::SMOOTH;  // the boundary runs straight through
    double u_min, u_max, v_min, v_max;
    BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
    const double step =
        1e-4 * std::sqrt((u_max - u_min) * (u_max - u_min) + (v_max - v_min) * (v_max - v_min)) / span;
    const gp_Pnt2d here = BRep_Tool::Parameters(vertex, face);
    const gp_Pnt2d probe(here.X() + bisector[0] * step, here.Y() + bisector[1] * step);
    IntTools_FClass2d classifier(face, TOLERANCE);
    return classifier.Perform(probe) == TopAbs_IN ? Convexity::CONVEX : Convexity::CONCAVE;
}

/// Classify by the creases meeting at this vertex of a solid or shell.
Convexity crease_convexity(const Shape& vertex) {
    const auto parent = vertex.topo_parent();
    if (!parent || parent->is_null()) {
        throw std::invalid_argument(
            "this vertex was not selected from a shape, so there is nothing to classify it against - take it "
            "from a face or solid");
    }
    OccShapeListMap vertex_edge_map;
    TopExp::MapShapesAndAncestors(parent->wrapped(), TopAbs_VERTEX, TopAbs_EDGE, vertex_edge_map);
    std::vector<TopoDS_Shape> keys;
    for (int i = 1; i <= vertex_edge_map.Extent(); ++i) keys.push_back(vertex_edge_map.FindKey(i));
    const auto own = find_same_topods(vertex.wrapped(), keys);
    if (!own) throw std::invalid_argument("this vertex is not part of its topo_parent");
    std::set<Convexity> kinds;
    for (const auto& topods_edge : vertex_edge_map.FindFromKey(*own)) {
        Shape edge(topods_edge);
        edge.set_topo_path(vertex.topo_path());
        kinds.insert(edge_convexity(edge));
    }
    return combine(kinds);
}

// ---- Face (Face.convexity)

Convexity face_convexity(const Shape& face_shape) {
    if (face_shape.is_null()) throw std::invalid_argument("an empty face has no convexity");
    const TopoDS_Face& face = TopoDS::Face(face_shape.wrapped());
    const BRepAdaptor_Surface surface(face);
    BRepGProp_Face outward(face);
    double u_min, u_max, v_min, v_max;
    BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
    const double fractions[3] = {0.15, 0.5, 0.85};
    std::set<Convexity> kinds;
    for (double u_frac : fractions) {
        for (double v_frac : fractions) {
            const double u_val = u_min + (u_max - u_min) * u_frac;
            const double v_val = v_min + (v_max - v_min) * v_frac;
            BRepLProp_SLProps props(surface, u_val, v_val, 2, TOLERANCE);
            if (!props.IsCurvatureDefined()) continue;
            gp_Pnt point;
            gp_Vec normal;
            outward.Normal(u_val, v_val, point, normal);
            if (normal.Magnitude() < 1e-12) continue;
            // curvatures signed toward the outward normal: negative bends away
            const double sign = props.Normal().Dot(gp_Dir(normal)) > 0 ? 1.0 : -1.0;
            double low = sign * props.MinCurvature();
            double high = sign * props.MaxCurvature();
            if (low > high) std::swap(low, high);
            const bool bends_away = low < -TOLERANCE;
            const bool bends_into = high > TOLERANCE;
            if (bends_away && bends_into) return Convexity::SADDLE;
            if (bends_away) {
                kinds.insert(Convexity::CONVEX);
            } else if (bends_into) {
                kinds.insert(Convexity::CONCAVE);
            } else {
                kinds.insert(Convexity::SMOOTH);
            }
        }
    }
    if (kinds.empty()) throw std::invalid_argument("the surface is degenerate everywhere it was sampled");
    return combine(kinds);
}

}  // namespace

Convexity Shape::convexity() const {
    if (!is_null()) {
        switch (obj_.ShapeType()) {
            case TopAbs_VERTEX: {
                const auto face = owning_face(*this);
                if (face) return corner_convexity(TopoDS::Vertex(obj_), *face);
                return crease_convexity(*this);
            }
            case TopAbs_EDGE:
            case TopAbs_WIRE: return edge_convexity(*this);
            case TopAbs_FACE: return face_convexity(*this);
            default: break;
        }
    }
    throw std::invalid_argument("a " + build123d::to_string(shape_type()) +
                                " has no convexity - only a Vertex, Edge or Face selected from a larger shape does");
}

bool Mixin1D::is_interior() const { return convexity() == Convexity::CONCAVE; }
bool Face::is_circular_convex() const { return convexity() == Convexity::CONVEX; }
bool Face::is_circular_concave() const { return convexity() == Convexity::CONCAVE; }

}  // namespace build123d
