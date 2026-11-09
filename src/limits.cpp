// Trimming curves to other objects: Edge.trim_to_other and Edge.trim_infinite
// (topology/one_d.py) and the Limit used by the arcs' arc_size and PolarLine's length.
//
// trim_to_other uses build123d 0.13's Shape.intersect. This file ports the part of it an
// Edge needs (Mixin1D/Mixin2D/Solid/Compound/Vertex._intersect with a 1D operand, and the
// _bool_op_list/expand helpers) until the general intersection lands in the library.
#include <memory>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>

#include "build123d/objects.hpp"
#include "internal.hpp"
#include "optimize.hpp"
#include "py_compat.hpp"

namespace build123d {

using namespace detail;

namespace {

using Shapes = std::vector<TopoDS_Shape>;

/// Shape._bool_op_list for a Common or a Section: the top-level pieces of the cleaned result.
Shapes bool_op_list(const TopoDS_Shape& arg, const TopoDS_Shape& tool, bool section) {
    TopoDS_Shape result = occt("intersect", [&] {
        std::unique_ptr<BRepAlgoAPI_BooleanOperation> operation;
        if (section) {
            operation = std::make_unique<BRepAlgoAPI_Section>();
        } else {
            operation = std::make_unique<BRepAlgoAPI_Common>();
        }
        operation->SetArguments(to_list({arg}));
        operation->SetTools(to_list({tool}));
        operation->SetRunParallel(true);
        operation->Build();
        return operation->Shape();
    });
    if (clean_enabled()) result = clean_shape(result);
    if (result.IsNull()) return {};
    result = unwrap(result, true);
    if (result.ShapeType() == TopAbs_COMPOUND) return top_level(result);
    return {result};
}

/// ShapeList.expand: compounds dissolved, wires to edges, shells to faces, nulls dropped.
Shapes expand(const Shapes& shapes) {
    Shapes out;
    for (const auto& s : shapes) {
        if (s.IsNull()) continue;
        switch (s.ShapeType()) {
            case TopAbs_COMPOUND: {
                const Shapes inner = expand(children(s));
                out.insert(out.end(), inner.begin(), inner.end());
                break;
            }
            case TopAbs_SHELL:
                for (const auto& f : entities(s, TopAbs_FACE)) out.push_back(f);
                break;
            case TopAbs_WIRE:
                for (const auto& e : Wire(s).edges()) out.push_back(e.wrapped());
                break;
            default: out.push_back(s);
        }
    }
    return out;
}

/// ShapeList(set(...)): drop shapes that are the same (TShape and location) as earlier ones.
Shapes unique(const Shapes& shapes) {
    Shapes out;
    for (const auto& s : shapes) {
        if (std::none_of(out.begin(), out.end(), [&](const TopoDS_Shape& o) { return o.IsSame(s); })) {
            out.push_back(s);
        }
    }
    return out;
}

/// Edge.trim_infinite on an operand, sized from the shape it's intersected with.
TopoDS_Shape finite_operand(const TopoDS_Shape& shape_with_bbox, const TopoDS_Shape& other) {
    if (other.ShapeType() != TopAbs_EDGE) return other;
    const Edge edge(other);
    if (!edge.is_infinite()) return other;
    const BoundBox bbox = Shape(shape_with_bbox).bounding_box(std::nullopt, false);
    return edge.trim_infinite(bbox.diagonal() + (edge.center() - bbox.center()).length()).wrapped();
}

Shapes intersect_with_1d(const TopoDS_Shape& self, const TopoDS_Shape& other, double tolerance);

/// Mixin1D._intersect(other) for a shape `other`.
Shapes mixin1d_intersect(const TopoDS_Shape& self, TopoDS_Shape other, double tolerance) {
    other = finite_operand(self, other);
    Shapes results;
    switch (other.ShapeType()) {
        case TopAbs_EDGE:
        case TopAbs_WIRE: {
            // Common (collinear overlap) + Section (crossing vertices)
            results = expand(bool_op_list(self, other, false));
            for (const auto& s : bool_op_list(self, other, true)) {
                if (s.ShapeType() == TopAbs_VERTEX) results.push_back(s);
            }
            break;
        }
        case TopAbs_VERTEX:
            if (Shape(other).distance_to(Shape(self)) <= tolerance) results.push_back(other);
            break;
        default:
            // Delegate to higher-order shapes (Face, Solid, etc.)
            results = intersect_with_1d(other, self, tolerance);
    }
    return results;
}

/// X._intersect(other) where `other` is an Edge or Wire, for any shape type X.
Shapes intersect_with_1d(const TopoDS_Shape& self, const TopoDS_Shape& other_in, double tolerance) {
    switch (self.ShapeType()) {
        case TopAbs_VERTEX:
            // Vertex._intersect delegates to the edge
            return mixin1d_intersect(other_in, self, tolerance);
        case TopAbs_EDGE:
        case TopAbs_WIRE: return mixin1d_intersect(self, other_in, tolerance);
        case TopAbs_FACE:
        case TopAbs_SHELL:
            // 2D + Edge: Section for intersection
            return bool_op_list(self, finite_operand(self, other_in), true);
        case TopAbs_SOLID:
            // Solid + Edge: Common
            return expand(bool_op_list(self, finite_operand(self, other_in), false));
        default: {
            // Compound: distribute over the elements (OR semantics)
            Shapes results;
            for (const auto& element : children(self)) {
                const Shapes r = intersect_with_1d(element, other_in, tolerance);
                results.insert(results.end(), r.begin(), r.end());
            }
            return unique(results);
        }
    }
}

/// Shape.intersect(other) for an Edge and one operand, as shapes (empty for None).
Shapes edge_intersect(const Edge& self, const TopoDS_Shape& other) {
    if (self.is_null()) throw std::invalid_argument("Can't intersect an empty edge");
    if (other.IsNull()) return {};
    return unique(expand(mixin1d_intersect(self.wrapped(), other, 1e-6)));
}

/// Edge.param_at_point as build123d computes it: a vertex or the projected point gives the
/// parameter along the underlying curve (the edge's orientation is ignored), validated with
/// position_at; otherwise a bounded search with scipy's minimize_scalar.
double python_param_at_point(const Edge& edge, const Vector& pnt) {
    if (edge.is_null()) throw std::invalid_argument("Can't find param on empty edge");
    const TopoDS_Edge& e = TopoDS::Edge(edge.wrapped());
    double param_min, param_max;
    BRep_Tool::Range(e, param_min, param_max);

    // Method 1: the point is a Vertex
    const auto vertices = edge.vertices();
    std::size_t nearest = 0;
    for (std::size_t i = 1; i < vertices.size(); ++i) {
        if ((Vector(vertices[i]) - pnt).length() < (Vector(vertices[nearest]) - pnt).length()) nearest = i;
    }
    if (!vertices.empty() && (Vector(vertices[nearest]) - pnt).length() <= TOLERANCE) {
        const double param = BRep_Tool::Parameter(TopoDS::Vertex(vertices[nearest].wrapped()), e);
        BRepAdaptor_Curve curve_adaptor(e);
        return GCPnts_AbscissaPoint::Length(curve_adaptor, param_min, param) / GCPnts_AbscissaPoint::Length(curve_adaptor);
    }

    const double separation = edge.distance_to(pnt);
    if (!(std::abs(separation) <= TOLERANCE)) {
        throw std::invalid_argument("point (" + pnt.to_string() + ") is " + std::to_string(separation) + " from edge");
    }

    // Method 2: project the point onto the edge
    const double u_value = occt("param_at_point", [&] {
        double f, l;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(e, f, l);
        GeomAPI_ProjectPointOnCurve projector(pnt.to_pnt(), curve);
        double param = projector.LowerDistanceParameter();
        BRepAdaptor_Curve curve_adaptor(e);
        if (curve_adaptor.IsPeriodic()) param = param_min + py_mod(param - param_min, curve_adaptor.Period());
        return GCPnts_AbscissaPoint::Length(curve_adaptor, param_min, param) / GCPnts_AbscissaPoint::Length(curve_adaptor);
    });
    if ((edge.position_at(u_value) - pnt).length() < TOLERANCE) return u_value;

    // Method 3: search the edge for the point, in ever smaller intervals
    const int max_divisions = 10;
    for (int division = 0; division < max_divisions; ++division) {
        const int intervals = 1 << division;
        const double step = 1.0 / intervals;
        for (int i = 0; i < intervals; ++i) {
            const double lo = i * step, hi = (i + 1) * step;
            const ScalarResult result = minimize_scalar_bounded(
                [&](double u) { return (edge.position_at(u) - pnt).length(); }, lo, hi, TOLERANCE / 2);
            // Early exit if we're below resolution limit
            if (result.fun < (edge.position_at(result.x + TOLERANCE) - edge.position_at(result.x - TOLERANCE)).length()) {
                return py_round(result.x, 6);
            }
        }
    }
    throw std::runtime_error("Unable to find parameter, Edge is too complex");
}

/// The shortest trim of `edge` from its start to one of the intersection vertices.
std::optional<Edge> shortest_trim(const Edge& edge, const Shapes& intersections) {
    if (intersections.empty()) return std::nullopt;
    std::vector<Edge> trims;
    for (const auto& s : intersections) {
        for (const auto& v : Shape(s).vertices()) trims.push_back(edge.trim(0, python_param_at_point(edge, Vector(v))));
    }
    if (trims.empty()) return std::nullopt;
    std::stable_sort(trims.begin(), trims.end(), [](const Edge& a, const Edge& b) { return a.length() < b.length(); });
    return trims.front();
}

}  // namespace

// ---------------------------------------------------------------------------
// Edge
// ---------------------------------------------------------------------------

Edge Edge::trim_infinite(double half_length) const {
    if (!is_infinite()) return *this;
    const Vector origin = center();
    const Vector direction = (end_point() - start_point()).normalized();
    return Edge::make_line(origin - direction * half_length, origin + direction * half_length);
}

std::optional<Edge> Edge::trim_to_other(const Shape& other) const {
    return shortest_trim(*this, edge_intersect(*this, other.wrapped()));
}

std::optional<Edge> Edge::trim_to_other(const Axis& other) const {
    return shortest_trim(*this, edge_intersect(*this, Edge(other).wrapped()));
}

std::optional<Edge> Edge::trim_to_other(const Location& other) const {
    return shortest_trim(*this, edge_intersect(*this, Vertex(other.position()).wrapped()));
}

std::optional<Edge> Edge::trim_to_other(const Plane& other) const {
    if (is_null()) throw std::invalid_argument("Can't intersect an empty edge");
    // 1D + Plane: run Section directly with an infinite face
    const TopoDS_Shape face = occt("intersect", [&] { return TopoDS_Shape(BRepBuilderAPI_MakeFace(other.wrapped()).Face()); });
    return shortest_trim(*this, unique(expand(expand(bool_op_list(obj_, face, true)))));
}

std::optional<Edge> Edge::trim_to_other(const Vector& other) const {
    return shortest_trim(*this, edge_intersect(*this, Vertex(other).wrapped()));
}

// ---------------------------------------------------------------------------
// Limit
// ---------------------------------------------------------------------------

Limit::Limit(const Shape& shape) : value(shape) {}
Limit::Limit(const Axis& axis) : value(axis) {}
Limit::Limit(const Location& location) : value(location) {}
Limit::Limit(const Plane& plane) : value(plane) {}
Limit::Limit(const Vector& point) : value(point) {}
Limit::Limit(double x, double y, double z) : value(Vector(x, y, z)) {}

std::optional<Edge> Limit::trim(const Edge& edge) const {
    return std::visit([&](const auto& v) { return edge.trim_to_other(v); }, value);
}

std::string Limit::to_string() const {
    struct Visitor {
        std::string operator()(const Shape& s) const { return s.to_string(); }
        std::string operator()(const Axis& a) const { return a.to_string(); }
        std::string operator()(const Location& l) const { return l.to_string(); }
        std::string operator()(const Plane& p) const { return p.to_string(); }
        std::string operator()(const Vector& v) const { return v.to_string(); }
    };
    return std::visit(Visitor{}, value);
}

}  // namespace build123d
