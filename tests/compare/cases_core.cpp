// Core (geometry/topology) cases compared with cases_core_py.py.
#include <algorithm>
#include <tuple>

#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

double rnd(double v, int digits = 4) {
    const double f = std::pow(10.0, digits);
    double r = std::round(v * f) / f;
    return r == 0.0 ? 0.0 : r;
}

double measure(const Shape& s) {
    switch (s.shape_type()) {
        case ShapeType::EDGE:
        case ShapeType::WIRE: return Mixin1D(s.wrapped()).length();
        case ShapeType::FACE:
        case ShapeType::SHELL: return s.area();
        case ShapeType::SOLID:
        case ShapeType::COMPOUND: return s.volume();
        default: return 0.0;
    }
}

/// A list of shapes as sorted [type, center, measure] rows (Python's set order is arbitrary).
template <class L>
V describe(const L& shapes) {
    std::vector<std::tuple<std::string, double, double, double, double>> rows;
    for (const auto& s : shapes) {
        const Vector c = s.center();
        rows.emplace_back(to_string(s.shape_type()), rnd(c.X), rnd(c.Y), rnd(c.Z), rnd(measure(s)));
    }
    std::sort(rows.begin(), rows.end());
    std::vector<V> out;
    for (const auto& [t, x, y, z, m] : rows) out.push_back(V::list({t, x, y, z, m}));
    return V::list(out);
}

V vec(const Vector& v) { return V::list({rnd(v.X, 6), rnd(v.Y, 6), rnd(v.Z, 6)}); }

V axis_v(const Axis& a) { return V::obj({{"kind", "Axis"}, {"position", vec(a.position)}, {"direction", vec(a.direction)}}); }
V loc_v(const Location& l) {
    return V::obj({{"kind", "Location"}, {"position", vec(l.position())}, {"orientation", vec(l.orientation())}});
}
V plane_v(const Plane& p) {
    return V::obj({{"kind", "Plane"}, {"origin", vec(p.origin)}, {"x_dir", vec(p.x_dir)}, {"z_dir", vec(p.z_dir)}});
}
V vector_v(const Vector& v) { return V::obj({{"kind", "Vector"}, {"value", vec(v)}}); }

template <class... Ts>
V geom(const std::optional<std::variant<Ts...>>& r) {
    if (!r) return V();
    return std::visit(
        [](const auto& x) -> V {
            using X = std::decay_t<decltype(x)>;
            if constexpr (std::is_same_v<X, Vector>) {
                return vector_v(x);
            } else if constexpr (std::is_same_v<X, Axis>) {
                return axis_v(x);
            } else if constexpr (std::is_same_v<X, Plane>) {
                return plane_v(x);
            } else {
                return loc_v(x);
            }
        },
        *r);
}
V geom(const std::optional<Vector>& r) { return r ? vector_v(*r) : V(); }

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    // ---- 1a: a & [b, c] chains (a ∩ b ∩ c); a Compound operand distributes
    add("core_and_list_part", [] { return Box(4, 4, 4) & std::vector<Part>{Pos(1, 0, 0) * Box(4, 4, 4), Sphere(2.5)}; });
    add("core_and_list_sketch", [] {
        return Rectangle(4, 4) & std::vector<Sketch>{Pos(1, 0) * Rectangle(4, 4), Circle(2.2)};
    });
    add("core_and_chain_equivalent", [] { return (Box(4, 4, 4) & (Pos(1, 0, 0) * Box(4, 4, 4))) & Sphere(2.5); });
    add("core_and_compound_or", [] {
        return Box(4, 4, 4) & Compound(std::vector<Part>{Pos(-2, 0, 0) * Box(1, 1, 1), Pos(2, 0, 0) * Box(1, 1, 1)});
    });
    add_value("core_intersect_chain_list", [] {
        const Part b = Box(4, 4, 4);
        return describe(b.intersect(std::vector<IntersectArg>{Pos(1, 0, 0) * Box(4, 4, 4), Sphere(2.5)}));
    });

    // ---- 1b: Vertex arithmetic
    add_value("core_vertex_arithmetic", [] {
        const Vertex a(1, 2, 3), b(4, 5, 6);
        Vertex c = a;
        c += Vector(0, 0, 15);
        return V::list({V(a + b), V(a + Vector(0.5, 0.5, 0.5)), V(b - a), V(b - Vector(1, 1, 1)),
                        V(a + Vector(1, 2)), V(c), V((a + b).shape_type() == ShapeType::VERTEX)});
    });

    // ---- 2: & and intersect for every dimension combination
    add("core_curve_and", [] { return Line({0, 0}, {1, 1}) & Line({0.25, 0.25}, {0.75, 0.75}); });
    add("core_line_cross", [] { return Line({0, 0}, {2, 2}) & Line({0, 2}, {2, 0}); });
    add("core_curve_curve", [] {
        const Curve c = Curve() + std::vector<Shape>{Line({-2, 0}, {2, 0}), Line({2, 0}, {2, 3})};
        return c & Curve(CenterArc({0, 0}, 1.5, 0, 360));
    });
    add("core_edge_face_cross", [] { return Line({0, 0, -1}, {0, 0, 1}) & Rectangle(2, 2); });
    add("core_edge_face_inplane", [] { return Line({-3, 0}, {3, 0}) & Circle(1); });
    add("core_face_edge", [] { return Circle(1) & Line({-3, 0.5}, {3, 0.5}); });
    add("core_part_curve", [] { return Box(2, 2, 2) & Line({-3, 0, 0}, {3, 0, 0}); });
    add("core_curve_part", [] { return Curve(Spline({{-3, 0, 0}, {0, 0.5, 0.2}, {3, 0, 0}})) & Sphere(2); });
    add("core_sketch_part", [] { return (Plane::XZ * Rectangle(6, 6)) & Box(2, 3, 4); });
    add("core_part_sketch", [] { return Box(2, 3, 4) & (Plane::XZ * Circle(3)); });
    add("core_face_face_cross", [] { return Rectangle(2, 2) & (Plane::XZ * Rectangle(1, 1)); });
    add("core_sketch_sketch_coplanar", [] { return Rectangle(20, 10) & Circle(7); });
    add("core_part_axis", [] { return Cylinder(2, 6) & Axis::Z; });
    add("core_part_axis_offset", [] { return Box(2, 2, 2) & Axis({0.5, 0, 0}, {1, 1, 0}); });
    add("core_part_plane", [] { return Box(2, 3, 4) & Plane::XY.offset(1); });
    add("core_part_vector", [] { return Box(2, 2, 2) & Vector(0.5, 0.5, 0.5); });
    add("core_sketch_location", [] { return Rectangle(2, 2) & Location(Vector(0.5, 0.25, 0)); });
    add("core_curve_plane", [] { return Curve(Helix(2, 6, 3)) & Plane::XY.offset(3); });
    add("core_curve_axis", [] { return Curve(Line({-1, -1}, {1, 1})) & Axis::X; });
    add_value("core_intersect_vertex", [] {
        const Vertex v(1, 0, 0);
        return V::list({describe(v.intersect(Vertex(1, 0, 0))), describe(v.intersect(Vector(1, 0, 1))),
                        describe(v.intersect(Axis::X)), describe(v.intersect(Plane::XZ)),
                        describe(v.intersect(Box(4, 4, 4))), describe(Box(4, 4, 4).intersect(v))});
    });
    add_value("core_intersect_edge_vertex", [] {
        const Edge e = Edge::make_line({0, 0}, {2, 0});
        return V::list({describe(e.intersect(Vector(1, 0, 0))), describe(e.intersect(Location(Vector(3, 0, 0)))),
                        describe(Vertex(1, 0, 0).intersect(e))});
    });
    add_value("core_intersect_tolerance", [] {
        const Edge e = Edge::make_line({0, 0}, {2, 0});
        return V::list({describe(e.intersect(Vertex(1, 1e-4, 0))), describe(e.intersect(Vertex(1, 1e-4, 0), 1e-3))});
    });
    add_value("core_intersect_touched", [] {
        const Part a = Box(2, 2, 2);
        const Part b = Pos(2, 0, 0) * Box(2, 2, 2);
        const Part c = Pos(2, 2, 0) * Box(2, 2, 2);
        return V::list({describe(a.intersect(b)), describe(a.intersect(b, 1e-6, true)), describe(a.intersect(c, 1e-6, true))});
    });
    add_value("core_touch", [] {
        const Part a = Box(2, 2, 2);
        const Face f1 = Face::make_rect(2, 2);
        const Face f2 = Face::make_rect(2, 2, Plane(Vector(2, 2, 0)));
        return V::list({describe(a.touch(Pos(2, 0, 0) * Box(2, 2, 2))), describe(a.touch(Pos(2, 2, 2) * Box(2, 2, 2))),
                        describe(f1.touch(f2)), describe(a.touch(Edge::make_line({1, -2, 0}, {1, 2, 0}))),
                        describe(a.solids()[0].touch(Vertex(1, 1, 1)))});
    });
    add_value("core_intersect_face_face_touched", [] {
        const Face f1 = Face::make_rect(2, 2);
        const Face f2 = Face::make_rect(2, 2, Plane(Vector(2, 2, 0)));
        const Face f3 = Face::make_rect(2, 2, Plane(Vector(1, 0, 0)));
        return V::list({describe(f1.intersect(f2)), describe(f1.intersect(f2, 1e-6, true)), describe(f1.intersect(f3))});
    });
    add_value("core_intersect_compound_children", [] {
        const Compound assy = Compound::make_assembly({Pos(-3, 0, 0) * Box(2, 2, 2), Pos(3, 0, 0) * Sphere(1)});
        return describe(assy.intersect(Axis::X));
    });

    // ---- 2: geometry intersections
    add_value("core_geom_axis_axis", [] {
        const Axis a({0, 1, 0}, {1, 0, 0});
        return V::list({geom(a.intersect(Axis({3, 0, 0}, {0, 1, 0}))), geom(a.intersect(Axis({5, 1, 0}, {-1, 0, 0}))),
                        geom(a.intersect(Axis({0, 2, 0}, {1, 0, 0}))), geom(a.intersect(Axis({0, 0, 1}, {0, 1, 0}))),
                        geom(a & Axis({2, 0, 0}, {0, 1, 0}))});
    });
    add_value("core_geom_axis_plane", [] {
        return V::list({geom(Axis::X.intersect(Plane::XY)), geom(Axis({0, 0, 5}, {0, 0, -1}).intersect(Plane::XY)),
                        geom(Plane::XY.intersect(Axis({0, 0, 5}, {1, 0, 0}))), geom(Plane::XZ & Axis({1, 2, 3}, {0, 1, 0}))});
    });
    add_value("core_geom_plane_plane", [] {
        return V::list({geom(Plane::XY.intersect(Plane::XY.offset(1))), geom(Plane::XY.intersect(Plane::XZ)),
                        geom(Plane::XY.intersect(Plane(Vector(1, 2, 0)))), geom(Plane::XY & Plane::YZ),
                        geom(Plane::XY.intersect(Plane::XY.reverse()))});
    });
    add_value("core_geom_location", [] {
        const Location l(Vector(1, 2, 3), Vector(0, 0, 45));
        return V::list({geom(l.intersect(Vector(1, 2, 3))), geom(l.intersect(Vector(1, 2, 4))),
                        geom(l.intersect(Location(Vector(1, 2, 3), Vector(0, 0, 45)))),
                        geom(l.intersect(Location(Vector(1, 2, 3), Vector(10, 0, 0)))),
                        geom(l.intersect(Location(Vector(0, 0, 0)))), geom(l.intersect(Axis({0, 2, 3}, {1, 0, 0}))),
                        geom(l.intersect(Axis({1, 2, 0}, {0, 0, 1}))), geom(l.intersect(Plane::XY.offset(3))),
                        geom(l.intersect(Plane::XZ.offset(-2))), geom(Axis({1, 2, 0}, {0, 0, 1}).intersect(l)),
                        geom(Plane::XY.offset(3) & l), geom(l & Plane::XY)});
    });
    add_value("core_geom_vector", [] {
        const Vector v(1, 0, 0);
        return V::list({geom(v.intersect(Vector(1, 0, 0))), geom(v.intersect(Vector(1, 0, 1e-7))),
                        geom(v.intersect(Vector(2, 0, 0))), geom(v.intersect(Axis::X)), geom(v.intersect(Axis::Y)),
                        geom(v.intersect(Plane::XY)), geom(v.intersect(Plane::YZ)), geom(v.intersect(Location(v))),
                        geom(v & Plane::XZ), geom(v & Axis::X)});
    });
    add_value("core_geom_with_shape", [] {
        const Part b = Box(2, 2, 2);
        return V::list({describe(Vector(0, 0, 0) & b), describe(Axis::Z & b), describe(Plane::XY & b),
                        describe(Location(Vector(0.5, 0, 0)) & b), describe(Axis::Z.intersect(b)),
                        describe(Plane::YZ.intersect(b))});
    });
}
