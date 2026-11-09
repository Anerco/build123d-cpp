// Core (geometry/topology) cases compared with cases_core_py.py.
#include <algorithm>
#include <regex>
#include <tuple>
#include <unordered_set>

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
    // ---- 3: history and Select (upstream tests/test_direct_api/test_history.py)
    add_value("core_select_fused_sketch", [] {
        Sketch cross = Rectangle(8, 2);
        cross += Rectangle(2, 8);
        return V::list({V(cross.vertices(Select::NEW).size()), V(cross.vertices(Select::LAST).size()),
                        V(cross.edges(Select::NEW).size()), V(cross.edges(Select::LAST).size()),
                        describe(cross.vertices(Select::NEW))});
    });
    add_value("core_select_fused_part", [] {
        const Part part = Box(10, 10, 10) + Pos(0, 0, 5) * Cylinder(2, 5, {Align::CENTER, Align::CENTER, Align::MIN});
        return V::list({describe(part.faces(Select::LAST)), V(part.solids(Select::NEW).size())});
    });
    add_value("core_select_box_plus_cylinder", [] {
        const Part part = Box(10, 10, 10) + Cylinder(2, 20);
        return V::list({describe(part.edges(Select::NEW)), describe(part.edges(Select::LAST)),
                        describe(part.faces(Select::NEW)), describe(part.faces(Select::LAST))});
    });
    add_value("core_select_cut_and", [] {
        const Part holed = Box(10, 10, 10) - Cylinder(1, 20);
        const Part bar = Box(10, 10, 10) & Box(30, 2, 2);
        return V::list({describe(holed.faces(Select::LAST)), V(holed.edges(Select::NEW).size()),
                        V(bar.faces(Select::LAST).size()), V(bar.faces(Select::NEW).size())});
    });
    add_value("core_select_chained", [] {
        const Part three = Box(10, 10, 10) + Pos(0, 0, 5) * Cylinder(1, 5, {Align::CENTER, Align::CENTER, Align::MIN}) +
                           Pos(5, 0, 0) * Sphere(1);
        return describe(three.faces(Select::LAST));
    });
    add_value("core_select_multi_piece_self", [] {
        const Sketch two = Rectangle(2, 2) + Pos(5, 0) * Rectangle(2, 2);
        const Sketch plus = two + Pos(5, 0) * Rectangle(1, 4);
        return V::list({V(plus.faces().size()), V(plus.faces(Select::LAST).size())});
    });
    add_value("core_select_fillet_chamfer", [] {
        const Part rounded = fillet(Box(10, 10, 10).edges().filter_by(Axis::Z), 1);
        const Part bevelled = chamfer(Box(10, 10, 10).edges().group_by(Axis::Z)[-1], 1);
        return V::list({V(rounded.faces(Select::LAST).size()), V(rounded.edges(Select::NEW).size()),
                        V(bevelled.faces(Select::NEW).size()), V(bevelled.faces(Select::LAST).size()),
                        V(bevelled.edges(Select::NEW).size())});
    });
    add_value("core_select_joined_lines", [] {
        const Curve bent = Line({0, 0}, {1, 0}) + Line({1, 0}, {1, 1});
        const Curve straight = Line({0, 0}, {1, 0}) + Line({1, 0}, {2, 0});
        const Curve chain = Curve() + std::vector<Shape>{Line({0, 0}, {1, 0}), Line({1, 0}, {1, 1}), Line({1, 1}, {0, 1})};
        return V::list({V(bent.edges().size()), V(bent.edges(Select::LAST).size()), V(bent.edges(Select::NEW).size()),
                        V(bent.vertices(Select::LAST).size()), V(bent.vertices(Select::NEW).size()),
                        V(straight.edges(Select::LAST).size()), V(straight.vertices(Select::NEW).size()),
                        V(chain.edges(Select::LAST).size())});
    });
    add_value("core_select_no_record", [] {
        bool raised = false;
        try {
            (void)Rectangle(8, 2).vertices(Select::NEW);
        } catch (const std::invalid_argument&) {
            raised = true;
        }
        return V::list({V(raised), V(Rectangle(8, 2).vertices(Select::ALL).size())});
    });
    add_value("core_select_2d_fillet_chamfer", [] {
        const Sketch rect = Rectangle(10, 6);
        const Sketch rounded = fillet(rect.vertices(), 1);
        const Sketch bevelled = chamfer(rect.vertices().group_by(Axis::X)[-1], 1);
        return V::list({V(rounded.edges(Select::NEW).size()), V(rounded.edges(Select::LAST).size()),
                        V(bevelled.edges(Select::NEW).size()), V(bevelled.vertices(Select::LAST).size())});
    });
    add_value("core_select_draft_loft_clean", [] {
        const Part box = Box(10, 10, 10);
        const Part drafted = draft(box.faces().filter_by(Axis::X), Plane::XY.offset(-5), 5);
        const Part lofted = loft(std::vector<Sketch>{Rectangle(4, 4), Pos(0, 0, 5) * Circle(1)});
        const Part cleaned = (Box(2, 2, 2) + Pos(2, 0, 0) * Box(2, 2, 2)).clean();
        return V::list({V(drafted.faces(Select::NEW).size()), V(drafted.faces(Select::LAST).size()),
                        V(lofted.faces(Select::NEW).size()), V(lofted.edges(Select::LAST).size()),
                        V(cleaned.faces(Select::NEW).size()), V(cleaned.faces(Select::LAST).size())});
    });
    add_value("core_select_curve_fillet", [] {
        const Curve poly = Curve(Polyline({{0, 0}, {10, 0}, {10, 10}, {0, 10}}));
        const Curve rounded = fillet(poly.vertices(), 2);
        return V::list({V(rounded.edges(Select::NEW).size()), V(rounded.edges(Select::LAST).size()),
                        V(rounded.edges().size()), describe(rounded.edges())});
    });
    add_value("core_new_edges", [] {
        const Part box = Box(10, 10, 10);
        const Part cyl = Cylinder(2, 20);
        const Part both = box + cyl;
        return describe(new_edges(std::vector<Shape>{box, cyl}, both));
    });

    // ---- 4: convexity
    add_value("core_convexity_edges", [] {
        const auto names = [](const auto& list) {
            std::vector<V> out;
            for (const auto& s : list) {
                try {
                    out.emplace_back(std::string(s.convexity() == Convexity::CONVEX    ? "CONVEX"
                                                 : s.convexity() == Convexity::CONCAVE ? "CONCAVE"
                                                 : s.convexity() == Convexity::SMOOTH  ? "SMOOTH"
                                                                                       : "SADDLE"));
                } catch (const std::invalid_argument&) {
                    out.emplace_back("error");
                }
            }
            return V::list(out);
        };
        const Part pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4);
        const Part step = Box(10, 10, 2) + Pos(2.5, 0, 2) * Box(5, 10, 2);
        const Part rounded = fillet(Box(10, 10, 10).edges().group_by(Axis::Z)[-1], 1);
        const Part tube = Cylinder(5, 10) - Cylinder(2, 10);
        return V::list({names(Box(1, 1, 1).edges()), names(pocket.edges()), names(step.edges()), names(rounded.edges()),
                        names(tube.edges()), names(Rectangle(2, 2).edges())});
    });
    add_value("core_convexity_faces_vertices", [] {
        const auto names = [](const auto& list) {
            std::vector<V> out;
            for (const auto& s : list) {
                try {
                    const Convexity c = s.convexity();
                    out.emplace_back(std::string(c == Convexity::CONVEX    ? "CONVEX"
                                                 : c == Convexity::CONCAVE ? "CONCAVE"
                                                 : c == Convexity::SMOOTH  ? "SMOOTH"
                                                                           : "SADDLE"));
                } catch (const std::invalid_argument&) {
                    out.emplace_back("error");
                }
            }
            return V::list(out);
        };
        const Part tube = Cylinder(5, 10) - Cylinder(2, 10);
        const Part step = Box(10, 10, 2) + Pos(2.5, 0, 2) * Box(5, 10, 2);
        const Sketch ell = Rectangle(4, 4) - Pos(1, 1) * Rectangle(2, 2);
        return V::list({names(tube.faces()), names(Torus(5, 1).faces()), names(Sphere(2).faces()),
                        names(step.vertices()), names(ell.faces()[0].vertices()),
                        names((Box(2, 2, 2) - Sphere(0.8)).faces())});
    });
    add_value("core_convexity_selectors", [] {
        const Part pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4);
        const auto groups = pocket.edges().group_by(by_convexity);
        std::vector<V> sizes;
        for (std::size_t i = 0; i < groups.size(); ++i) {
            sizes.emplace_back(V::list({V(groups.keys[i]), V(groups[static_cast<std::ptrdiff_t>(i)].size())}));
        }
        const auto concave = pocket.edges().filter_by(Convexity::CONCAVE);
        const auto not_concave = pocket.edges().filter_by(Convexity::CONCAVE, true);
        const Part tube = Cylinder(5, 10) - Cylinder(2, 10);
        std::vector<V> flags;
        for (const auto& f : tube.faces()) flags.emplace_back(V::list({V(f.is_circular_convex()), V(f.is_circular_concave())}));
        std::vector<V> interior;
        for (const auto& e : concave) interior.emplace_back(e.is_interior());
        return V::list({V::list(sizes), V(concave.size()), V(not_concave.size()), V::list(flags), V::list(interior)});
    });

    // ---- 5: topology helpers
    add_value("core_topo_explore", [] {
        const Part box = Box(10, 10, 10);
        const Edge top = box.edges().group_by(Axis::Z)[-1].sort_by(Axis::X)[0];
        const Part rounded = fillet(Box(10, 10, 10).edges().filter_by(Axis::Z), 2);
        const Edge vertical = rounded.edges().filter_by(Axis::Z).sort_by(Axis::X)[0];
        const auto c0 = topo_explore_connected_edges(vertical);
        const auto c1 = topo_explore_connected_edges(vertical, std::nullopt, ContinuityLevel::C1);
        const auto c2 = topo_explore_connected_edges(vertical, rounded, ContinuityLevel::C2);
        const auto edges = box.edges();
        const auto common = topo_explore_common_vertex(edges[0], edges[1]);
        const auto none = topo_explore_common_vertex(edges[0], edges[2]);
        return V::list({describe(topo_explore_connected_edges(top)), describe(c0), describe(c1), describe(c2),
                        common ? V(*common) : V(), none ? V(*none) : V()});
    });
    add_value("core_topo_distance", [] {
        const Part box = Box(10, 10, 10);
        const Face top = box.faces().sort_by(Axis::Z)[-1];
        const auto faces = box.faces().sort_by(topo_distance_to(top));
        std::vector<V> keys;
        const auto key = topo_distance_to(top);
        for (const auto& f : faces) keys.emplace_back(key(f));
        const auto edge_key = topo_distance_to(box.edges().sort_by(Axis::Z)[0]);
        std::vector<V> edge_keys;
        for (const auto& e : box.edges().sort_by(edge_key)) edge_keys.emplace_back(edge_key(e));
        const auto vertex_key = topo_distance_to(box.vertices().sort_by(Axis::Z)[0]);
        std::vector<V> vertex_keys;
        for (const auto& v : box.vertices()) vertex_keys.emplace_back(vertex_key(v));
        const auto groups = box.faces().group_by(topo_distance_to(std::vector<Face>{top}));
        return V::list({V::list(keys), V::list(edge_keys), V::list(vertex_keys), V(groups.size())});
    });
    add_value("core_utils", [] {
        const Edge a = Edge::make_line({0, 0}, {1, 0});
        const Edge b = Edge::make_line({1, 0}, {1, 1});
        const Edge c = Edge::make_line({5, 5}, {6, 5});
        const auto wires = edges_to_wires({a, b, c});
        const auto d = delta(std::vector<Edge>{a, b, a.reversed()}, std::vector<Edge>{a});
        return V::list({V(polar(2, 30)), V(wires.size()), describe(wires), V(d.size())});
    });

    // ---- 6: mass, material, inertia
    add_value("core_mass_inertia", [] {
        const Part box = Pos(1, 2, 3) * Box(2, 4, 6);
        const auto m = box.matrix_of_inertia();
        std::vector<V> rows;
        for (const auto& r : m) rows.push_back(V::list({r[0], r[1], r[2]}));
        std::vector<V> moments;
        for (const auto& [axis, moment] : Cylinder(2, 5).principal_properties()) moments.emplace_back(moment);
        const auto sm = box.static_moments();
        return V::list({V::list(rows), V::list(moments), V::list({sm[0], sm[1], sm[2]}),
                        V(box.radius_of_gyration(Axis::X)), V(Sphere(2).radius_of_gyration(Axis::Z)),
                        V(Edge::make_line({0, 0}, {3, 4}).compute_volume()), V(Face::make_rect(2, 3).compute_volume()),
                        V(Vertex(1, 2, 3).compute_volume()), V(box.compute_volume())});
    });
    add_value("core_mass_material", [] {
        Part box = Box(10, 10, 10);
        box.material = "aluminum";
        Part cyl = Cylinder(5, 10);
        cyl.material = resolve_material("Mild Steel");
        Shape assy_child = Pos(20, 0, 0) * Box(1, 1, 1);
        Compound assy = Compound::make_assembly({box, assy_child});
        assy.material = "copper";
        Shell shell = Box(2, 2, 2).shells()[0];
        shell.material = "abs";
        Part none = Box(1, 1, 1);
        bool missing = false;
        try {
            (void)none.mass();
        } catch (const std::invalid_argument&) {
            missing = true;
        }
        return V::list({V(box.mass()), V(box.mass(Unit::KG)), V(box.mass(Unit::LB, Unit::MM)), V(box.mass(Unit::G, Unit::CM)),
                        V(box.compute_mass(Unit::KG, Unit::IN)), V(cyl.mass()), V(assy.mass()), V(shell.mass()),
                        V(Rectangle(2, 2).mass()), V(Edge::make_line({0, 0}, {1, 0}).mass()), V(missing),
                        V(box.material->material.density), V(box.material->name()), V(material_names().size())});
    });
    add_value("core_combined_center", [] {
        const std::vector<Shape> objects{Box(2, 2, 2), Pos(10, 0, 0) * Sphere(1), Pos(0, 5, 0) * Cylinder(1, 4)};
        bool geometry_error = false;
        try {
            (void)Shape::combined_center(objects, CenterOf::GEOMETRY);
        } catch (const std::invalid_argument&) {
            geometry_error = true;
        }
        return V::list({V(Shape::combined_center(objects)), V(Shape::combined_center(objects, CenterOf::BOUNDING_BOX)),
                        V(geometry_error),
                        V(Shape::combined_center(std::vector<Edge>{Edge::make_line({0, 0}, {2, 0}), Edge::make_line({0, 1}, {0, 5})}))});
    });

    // ---- 7: Matrix and OrientedBoundBox
    add_value("core_matrix", [] {
        Matrix m({{1, 0, 0, 1}, {0, 1, 0, 2}, {0, 0, 1, 3}});
        m.rotate(Axis::Z, 90);
        const Matrix inv = m.inverse();
        const Matrix prod = m.multiply(inv);
        std::vector<V> values;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) values.emplace_back(m(r, c));
        }
        bool bad_size = false, bad_row = false;
        try {
            Matrix({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
        } catch (const std::invalid_argument&) {
            bad_size = true;
        }
        try {
            Matrix({{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 1, 1}});
        } catch (const std::invalid_argument&) {
            bad_row = true;
        }
        return V::list({V::list(values), V::list_of(m.transposed_list()), V(m.multiply(Vector(1, 0, 0))),
                        V(prod(0, 0)), V(prod(0, 3)), V(Vector(1, 2, 3).transform(m)),
                        V(Vector(1, 2, 3).transform(m, true)), V(bad_size), V(bad_row),
                        V(Vector(1.23456, -2.5, 3.5).round(2)), V(Vector(1.5, 2.5, -0.5).round())});
    });
    add("core_transform_geometry", [] {
        const Matrix skew({{1, 0.5, 0, 0}, {0, 1, 0, 0}, {0, 0, 2, 1}});
        return Box(2, 2, 2).transform_geometry(skew);
    });
    add("core_transform_shape_matrix", [] {
        Matrix m;
        m.rotate(Axis::X, 30);
        return Cylinder(1, 3).transform_shape(m);
    });
    add_value("core_oriented_bound_box", [] {
        const Part part = Rot(10, 20, 30) * Box(2, 4, 6);
        const OrientedBoundBox obb = part.oriented_bounding_box();
        std::vector<double> sizes{obb.size().X, obb.size().Y, obb.size().Z};
        std::sort(sizes.begin(), sizes.end());
        const OrientedBoundBox face_obb = Face::make_rect(2, 3).oriented_bounding_box();
        const OrientedBoundBox line_obb = Edge::make_line({0, 0, 0}, {3, 4, 0}).oriented_bounding_box();
        const OrientedBoundBox small = (Rot(10, 20, 30) * Box(1, 1, 1)).oriented_bounding_box();
        const Solid box = Solid::from_bounding_box(obb);
        return V::list({V::list_of(sizes), V(obb.diagonal()), V(obb.center()), V(obb.corners().size()),
                        V(face_obb.corners().size()), V(line_obb.corners().size()), V(line_obb.size().length()),
                        V(obb.is_completely_inside(small)), V(small.is_completely_inside(obb)),
                        V(obb.is_outside(Vector(0, 0, 0))), V(obb.is_outside(Vector(10, 10, 10))),
                        V(box.volume()), V(box.center())});
    });

    // ---- 8: Shape and geometry misc
    add_value("core_hash_sets", [] {
        const Part box = Box(1, 2, 3);
        std::unordered_set<Edge> edges;
        for (const auto& e : box.edges()) edges.insert(e);
        for (const auto& e : box.edges()) edges.insert(e.reversed());
        const auto box_faces = box.faces();
        std::unordered_set<Face> faces(box_faces.begin(), box_faces.end());
        for (const auto& f : box.faces()) faces.insert(f);
        const std::unordered_set<Vector> vectors{Vector(1, 2, 3), Vector(1.000001, 2, 3), Vector(1.0001, 2, 3)};
        const std::unordered_set<Location> locations{Location(Vector(1, 2, 3)), Location(Vector(1, 2, 3), Vector(0, 0, 360)),
                                                     Pos(1, 2, 3), Location(Vector(1, 2, 3), Vector(0, 0, 90))};
        const std::unordered_set<Plane> planes{Plane::XY, Plane::XY.offset(0), Plane::XZ};
        const std::unordered_set<Axis> axes{Axis::X, Axis({0, 0, 0}, {2, 0, 0}), Axis::Y};
        const Edge e = box.edges()[0];
        return V::list({V(edges.size()), V(faces.size()), V(vectors.size()), V(locations.size()), V(planes.size()),
                        V(axes.size()), V(e.hash() == e.reversed().hash()), V(e == e.reversed()),
                        V(box.hash() == box.moved(Location()).hash()), V(box.hash() == box.moved(Pos(1, 0, 0)).hash()),
                        V(box == box.moved(Pos(1, 0, 0))), V(Shape().hash() == 0)});
    });
    add("core_to_splines_box", [] { return Box(2, 3, 4).to_splines(); });
    add("core_to_splines_cylinder", [] { return Cylinder(1, 2).to_splines(5, 1e-4); });
    add_value("core_to_splines_types", [] {
        std::vector<V> types;
        for (const auto& f : Cylinder(1, 2).to_splines().faces()) types.emplace_back(to_string(f.geom_type()));
        std::vector<V> edge_types;
        for (const auto& e : Edge::make_circle(1).to_splines(3, 1e-3, true).edges()) {
            edge_types.emplace_back(to_string(e.geom_type()));
        }
        bool empty_error = false;
        try {
            (void)Solid().to_splines();
        } catch (const std::invalid_argument&) {
            empty_error = true;
        }
        return V::list({V::list(types), V::list(edge_types), V(empty_error)});
    });
    add_value("core_tessellate_with_uvs", [] {
        const auto summary = [](const Shape& s, bool atlas, double gutter) {
            const auto [vertices, triangles, normals, uvs] = s.tessellate_with_uvs(0.1, 0.1, atlas, gutter);
            double u_min = 1e300, u_max = -1e300, v_min = 1e300, v_max = -1e300, u_sum = 0, v_sum = 0;
            for (const auto& uv : uvs) {
                u_min = std::min(u_min, uv[0]);
                u_max = std::max(u_max, uv[0]);
                v_min = std::min(v_min, uv[1]);
                v_max = std::max(v_max, uv[1]);
                u_sum += uv[0];
                v_sum += uv[1];
            }
            double n_sum = 0;
            for (const auto& n : normals) n_sum += n.X + 2 * n.Y + 3 * n.Z;
            long index_sum = 0;
            for (const auto& t : triangles) index_sum += t[0] + 2L * t[1] + 3L * t[2];
            return V::list({V(vertices.size()), V(triangles.size()), V(normals.size()), V(uvs.size()), V(rnd(u_min, 6)),
                            V(rnd(u_max, 6)), V(rnd(v_min, 6)), V(rnd(v_max, 6)), V(rnd(u_sum, 3)), V(rnd(v_sum, 3)),
                            V(rnd(n_sum, 3)), V(static_cast<double>(index_sum))});
        };
        const Part box = Box(1, 2, 3);
        const Part cyl = Cylinder(1, 2);
        return V::list({summary(box, true, 0.0), summary(box, false, 0.0), summary(box, true, 0.01),
                        summary(cyl, true, 0.0), summary(Pos(1, 2, 3) * Sphere(1), true, 0.0)});
    });
    add_value("core_show_topology", [] {
        const auto strip = [](const std::string& s) {
            return std::regex_replace(s, std::regex("at 0x[0-9a-f]+"), "at X");
        };
        const auto strip_class = [](const std::string& s) {
            return std::regex_replace(s, std::regex("\\S+ +at 0x[0-9a-f]+"), "at X");
        };
        const Part box = Box(1, 2, 3);
        Shape child1 = Pos(1, 0, 0) * Box(1, 1, 1);
        Compound sub = Compound::make_assembly({Sphere(1)}, "sub");
        const Compound assy = Compound::make_assembly({child1, sub}, "assy");
        return V::list({V(strip(box.show_topology(ShapeType::FACE))),
                        V(strip(box.solids()[0].show_topology(ShapeType::EDGE, false))),
                        V(strip(Face::make_rect(1, 1).show_topology())), V(strip_class(assy.show_topology())),
                        V(strip_class(assy.show_topology(ShapeType::VERTEX, true)))});
    });
    add_value("core_split_both", [] {
        const Part box = Box(2, 2, 2);
        const auto [top, bottom] = box.split(Plane::XY.offset(0.5), Keep::BOTH);
        const Part u_shape = Box(6, 2, 4) - Pos(0, 0, 1) * Box(2, 2, 2);
        const SplitResult prongs = u_shape.split(Plane::XY, Keep::BOTH);
        const SplitResult none_below = box.split(Plane::XY.offset(-5), Keep::BOTTOM);
        const SplitResult all = u_shape.split(Plane::XY, Keep::ALL);
        const Part op_both = split(u_shape, Plane::XY, Keep::BOTH);
        const Part op_bottom = split(u_shape, Plane::XY, Keep::BOTTOM);
        return V::list({describe(top), describe(bottom), describe(prongs.top), describe(prongs.bottom), V(prongs.size()),
                        V(none_below.size()), V(all.size()), describe(op_both.solids()), describe(op_bottom.solids())});
    });
    add_value("core_face_wire_warning", [] {
        std::vector<V> messages;
        const auto previous = set_warning_handler([&](const std::string& m) { messages.emplace_back(m); });
        const Sketch holed = Rectangle(4, 4) - Circle(1);
        const double with_holes = holed.faces()[0].wire().length();
        const double plain = Face::make_rect(2, 2).wire().length();
        set_warning_handler(previous);
        return V::list({V::list(messages), V(with_holes), V(plain)});
    });
    add("core_rotate_transform", [] { return Box(1, 2, 3).rotate(Axis::Z, 30, true); });
    add("core_translate_transform", [] { return Box(1, 2, 3).translate(Vector(1, 2, 3), true); });
    add_value("core_transform_keeps_location", [] {
        const Part moved = Box(1, 2, 3).translate(Vector(1, 2, 3));
        const Part regenerated = Box(1, 2, 3).translate(Vector(1, 2, 3), true);
        const Part rotated = Box(1, 2, 3).rotate(Axis::Z, 30, true);
        return V::list({V(moved.location().position()), V(regenerated.location().position()),
                        V(rotated.location().orientation()), V(regenerated.center()), V(rotated.faces()[0].normal_at())});
    });

    add_value("core_geom_with_shape", [] {
        const Part b = Box(2, 2, 2);
        return V::list({describe(Vector(0, 0, 0) & b), describe(Axis::Z & b), describe(Plane::XY & b),
                        describe(Location(Vector(0.5, 0, 0)) & b), describe(Axis::Z.intersect(b)),
                        describe(Plane::YZ.intersect(b))});
    });
}
