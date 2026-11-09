#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp_Face.hxx>
#include <Geom_Curve.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <TopoDS.hxx>

#include "internal.hpp"

namespace build123d::detail {

namespace {

const char* dim_name(int d) {
    switch (d) {
        case 0: return "0D";
        case 1: return "1D";
        case 2: return "2D";
        case 3: return "3D";
        default: return "mixed";
    }
}

std::vector<TopoDS_Shape> edges_of(const TopoDS_Shape& s) {
    std::vector<TopoDS_Shape> out;
    if (s.ShapeType() == TopAbs_EDGE) return {s};
    if (s.ShapeType() == TopAbs_WIRE) {
        for (BRepTools_WireExplorer exp(TopoDS::Wire(s)); exp.More(); exp.Next()) out.push_back(exp.Current());
        return out;
    }
    for (const auto& e : entities(s, TopAbs_EDGE)) out.push_back(e);
    return out;
}

TopoDS_Shape try_wire(const std::vector<TopoDS_Shape>& edges) {
    BRepBuilderAPI_MakeWire builder;
    builder.Add(to_list(edges));
    builder.Build();
    if (!builder.IsDone()) return TopoDS_Shape();
    return builder.Wire();
}

TopoDS_Shape curve_add(const Shape& a, const std::vector<TopoDS_Shape>& others) {
    std::vector<TopoDS_Shape> summand_edges;
    for (const auto& o : others) {
        for (const auto& e : edges_of(o)) summand_edges.push_back(e);
    }
    TopoDS_Shape result;
    if (!a) {
        if (others.size() == 1) {
            result = others[0];
        } else {
            result = occt("Curve +", [&] { return try_wire(summand_edges); });
            if (result.IsNull()) {
                std::vector<TopoDS_Shape> tools(others.begin() + 1, others.end());
                result = bool_op(BoolOp::FUSE, {others[0]}, tools, 0.0, false);
            }
        }
    } else {
        std::vector<TopoDS_Shape> all;
        for (const auto& t : top_level(a.wrapped())) {
            for (const auto& e : edges_of(t)) all.push_back(e);
        }
        all.insert(all.end(), summand_edges.begin(), summand_edges.end());
        result = occt("Curve +", [&] { return try_wire(all); });
        if (result.IsNull()) result = bool_op(BoolOp::FUSE, {a.wrapped()}, others, 0.0, false);
    }
    if (clean_enabled()) result = clean_shape(result);
    return result;
}

}  // namespace

TopoDS_Shape algebra(AlgebraOp op, const Shape& a, const std::vector<const Shape*>& b, int static_dim) {
    std::vector<TopoDS_Shape> others;
    for (const Shape* s : b) {
        if (s == nullptr || s->is_null()) continue;
        for (const auto& t : top_level(s->wrapped())) others.push_back(t);
    }
    const int a_dim = static_dim >= 0 ? static_dim : (a ? a.dim() : (others.empty() ? -1 : topods_dim(others[0])));

    switch (op) {
        case AlgebraOp::ADD: {
            if (others.empty()) return a.wrapped();
            if (a_dim < 0) throw std::invalid_argument("Dimensions of objects to add to are inconsistent");
            for (const auto& o : others) {
                if (topods_dim(o) != a_dim) {
                    throw std::invalid_argument(std::string("Only shapes with the same dimension can be added: not ") +
                                                dim_name(a_dim) + " and " + dim_name(topods_dim(o)));
                }
            }
            if (a_dim == 1) return curve_add(a, others);
            std::vector<TopoDS_Shape> summands = top_level(a.wrapped());
            summands.insert(summands.end(), others.begin(), others.end());
            if (summands.size() <= 1) return make_compound(summands);
            const std::vector<TopoDS_Shape> tools(summands.begin() + 1, summands.end());
            return bool_op(BoolOp::FUSE, {summands[0]}, tools, TOLERANCE);
        }
        case AlgebraOp::SUB: {
            if (a.is_null()) throw std::invalid_argument("Cannot subtract shape from empty compound");
            if (others.empty()) return a.wrapped();
            if (a_dim < 0) throw std::invalid_argument("Dimensions of objects to subtract from are inconsistent");
            for (const auto& o : others) {
                const int d = topods_dim(o);
                if (d < 0) throw std::invalid_argument("Dimensions of objects to subtract from are inconsistent");
                if (d < a_dim) {
                    throw std::invalid_argument(
                        std::string("Only shapes with equal or greater dimension can be subtracted: not ") +
                        dim_name(a_dim) + " and " + dim_name(d));
                }
            }
            return bool_op(BoolOp::CUT, {a.wrapped()}, others);
        }
        case AlgebraOp::AND: {
            if (!a) throw std::invalid_argument("Cannot intersect shape with empty compound");
            for (const Shape* s : b) {
                if (s == nullptr || !*s) throw std::invalid_argument("Cannot intersect shape with empty compound");
            }
            return bool_op(BoolOp::COMMON, {a.wrapped()}, others);
        }
    }
    return TopoDS_Shape();
}

std::vector<TopoDS_Shape> list_entities(const TopoDS_Shape& shape, int topabs_type) {
    return entities(shape, static_cast<TopAbs_ShapeEnum>(topabs_type));
}

double round_digits(double value, int digits) { return py_round(value, digits); }

Vector shape_center(const Shape& s) { return s.center(); }

double shape_distance(const Shape& s, const Shape& other) { return s.distance_to(other); }
double shape_distance(const Shape& s, const Vector& point) { return s.distance_to(point); }

double sort_key_axis(const Shape& s, const Axis& axis) {
    return (s.center() - axis.position).dot(axis.direction);
}

double sort_key(const Shape& s, SortBy sort_by) {
    if (s.is_null()) throw std::invalid_argument("Can't sort an empty shape");
    const auto type = s.wrapped().ShapeType();
    switch (sort_by) {
        case SortBy::LENGTH:
            if (type == TopAbs_FACE) {
                auto l = Face(s.wrapped()).length();
                if (!l) throw std::invalid_argument("length is only defined for planar faces");
                return *l;
            }
            return linear_length(s.wrapped());
        case SortBy::RADIUS:
            if (type == TopAbs_FACE) {
                auto r = Face(s.wrapped()).radius();
                if (!r) throw std::invalid_argument("Face has no radius");
                return *r;
            }
            if (type == TopAbs_EDGE || type == TopAbs_WIRE) return Mixin1D(s.wrapped()).radius();
            throw std::invalid_argument("Shape has no radius");
        case SortBy::AREA: return s.area();
        case SortBy::VOLUME: return s.volume();
        case SortBy::DISTANCE: return s.center().length();
    }
    return 0.0;
}

double sort_key_path(const Shape& s, const Mixin1D& path) {
    const auto pts = path.closest_points(s.center());
    return path.param_at_point(pts.first);
}

namespace {

bool planar_face_axis(const TopoDS_Shape& shape, const Shape& s, Axis& out) {
    const TopoDS_Face& face = TopoDS::Face(shape);
    if (!GeomLib_IsPlanarSurface(BRep_Tool::Surface(face), TOLERANCE).IsPlanar()) return false;
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(face).Normal(u0, v0, p, n);
    out = Axis(s.center(), Vector(n).normalized());
    return true;
}

}  // namespace

bool axis_parallel_predicate(const Shape& s, const Axis& axis, double tolerance) {
    if (s.is_null()) return false;
    const auto type = s.wrapped().ShapeType();
    Axis shape_axis = Axis::Z;
    if (type == TopAbs_FACE) {
        if (!planar_face_axis(s.wrapped(), s, shape_axis)) return false;
    } else if (type == TopAbs_EDGE && s.geom_type() == GeomType::LINE) {
        BRepAdaptor_Curve curve(TopoDS::Edge(s.wrapped()));
        gp_Pnt p;
        gp_Vec d;
        curve.D1(curve.FirstParameter(), p, d);
        shape_axis = Axis(Vector(p), Vector(gp_Dir(d)));
    } else {
        return false;
    }
    return axis.is_parallel(shape_axis, tolerance);
}

bool plane_parallel_predicate(const Shape& s, const Plane& plane, double tolerance) {
    if (s.is_null()) return false;
    const Axis plane_axis(plane.origin, plane.z_dir);
    const auto type = s.wrapped().ShapeType();
    if (type == TopAbs_FACE) {
        Axis shape_axis = Axis::Z;
        if (!planar_face_axis(s.wrapped(), s, shape_axis)) return false;
        return plane_axis.is_parallel(shape_axis, tolerance);
    }
    if (type == TopAbs_WIRE) {
        for (BRepTools_WireExplorer exp(TopoDS::Wire(s.wrapped())); exp.More(); exp.Next()) {
            if (!plane_parallel_predicate(Shape(exp.Current()), plane, tolerance)) return false;
        }
        return true;
    }
    if (type == TopAbs_EDGE) {
        TopLoc_Location loc;
        double f, l;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(s.wrapped()), loc, f, l);
        if (curve.IsNull()) return false;
        gp_XYZ normal = plane.z_dir.to_dir().Transformed(loc.Transformation().Inverted()).XYZ();
        return ShapeAnalysis_Curve::IsPlanar(curve, normal, tolerance);
    }
    return false;
}

}  // namespace build123d::detail
