// Core (geometry/topology) cases compared with cases_core_py.py.
#include <algorithm>
#include <regex>
#include <tuple>
#include <unordered_set>

#include <TopoDS.hxx>

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
            sizes.emplace_back(V::list({V(static_cast<double>(groups.keys[i])), V(groups[static_cast<std::ptrdiff_t>(i)].size())}));
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

    // ---- 9: ShapeList
    add_value("core_shapelist_sort", [] {
        const Part box = Box(1, 2, 3);
        const Axis skew({1, 2, 3}, {1, 0.3, 0.7});
        const auto centers = [](const auto& list) {
            std::vector<V> out;
            for (const auto& s : list) out.emplace_back(s.center());
            return V::list(out);
        };
        const auto measures = [](const auto& list) {
            std::vector<V> out;
            for (const auto& s : list) out.emplace_back(rnd(measure(s), 6));
            return V::list(out);
        };
        const auto raises = [](auto f) {
            try {
                f();
            } catch (const std::exception&) {
                return true;
            }
            return false;
        };
        const Solid cone = Solid::make_cone(2, 1, 2);
        const auto circles = cone.edges().filter_by(GeomType::CIRCLE);
        std::vector<V> radii;
        for (const auto& e : circles.sort_by(SortBy::RADIUS, true)) radii.emplace_back(e.radius());
        const Part cyl = Cylinder(1, 2);
        std::vector<Solid> labelled;
        for (const char* name : {"b", "c", "a"}) {
            Solid b = Solid::make_box(1, 1, 1);
            b.label = name;
            labelled.push_back(b);
        }
        std::vector<V> labels;
        for (const auto& b : ShapeList<Solid>(labelled).sort_by([](const Solid& b) { return b.label; })) {
            labels.emplace_back(b.label);
        }
        const Edge path = Edge::make_line({0, 0, 0}, {1, 2, 3});
        const ShapeList<Solid> solids{Solid::make_box(3, 1, 1), Solid::make_box(1, 1, 1), Solid::make_box(2, 1, 1)};
        return V::list({centers(box.vertices().sort_by(skew)), centers(box.vertices().sort_by(skew, true)),
                        centers(box.faces().sort_by(skew)), measures(box.faces().sort_by(SortBy::AREA)),
                        measures(box.faces() < SortBy::AREA), V::list(radii),
                        V(raises([&] { (void)cone.edges().sort_by(SortBy::RADIUS); })),
                        V(raises([&] { (void)cyl.faces().sort_by(SortBy::RADIUS); })),
                        V(cyl.faces().filter_by(GeomType::CYLINDER).sort_by(SortBy::RADIUS).size()),
                        measures(box.edges().sort_by(SortBy::LENGTH)),
                        centers((Pos(1, 1.2, 1.1) * Box(1, 2, 3)).vertices().sort_by(SortBy::DISTANCE)),
                        measures(solids.sort_by(SortBy::VOLUME)),
                        measures(box.faces().sort_by([](const Face& f) { return f.area(); }, true)), V::list(labels),
                        centers(Box(1, 2, 3, Align::MIN).vertices().sort_by(path)),
                        centers(box.edges().sort_by_distance(Vector(-0.2, 0.13, 0.5))),
                        centers(box.faces().sort_by_distance(Vertex(5, 0.3, 0.25), true)),
                        V(raises([&] { (void)box.vertices().sort_by(SortBy::LENGTH); }))});
    });
    add_value("core_shapelist_group", [] {
        const auto sizes = [](const auto& groups) {
            std::vector<V> out;
            for (const auto& g : groups) out.emplace_back(g.size());
            return V::list(out);
        };
        const auto keys = [](const auto& groups) { return V::list_of(groups.keys); };
        const Solid box = Solid::make_box(1, 1, 2);
        const Edge line = Edge::make_line({0, 0, 0}, {1, 1, 2});
        std::vector<Solid> boxes;
        for (int i = 0; i < 6; ++i) {
            Solid b = Solid::make_box(1, 1, 1);
            b.label = i < 3 ? "A" : (i < 5 ? "B" : "");
            boxes.push_back(b);
        }
        const auto by_label = ShapeList<Solid>(boxes).group_by([](const Solid& b) { return b.label; });
        std::vector<V> label_keys;
        for (const auto& k : by_label.keys) label_keys.emplace_back(k);
        const Part box1 = Box(2, 2, 2);
        const Part box2 = Box(2, 2, 2).translate(Vector(1, 1, 1));
        const auto interior = [](const Edge& e) { return e.is_interior(); };
        const auto fused = (box1 + box2).edges().group_by(interior);
        const auto cut = (box1 - box2).edges().group_by(interior);
        bool missing = false;
        try {
            (void)by_label.group("C");
        } catch (const std::out_of_range&) {
            missing = true;
        }
        const ShapeList<Solid> solids{Solid::make_box(1, 1, 1), Solid::make_box(1, 1, 2), Solid::make_box(1, 1, 1),
                                      Solid::make_box(1, 1, 3), Solid::make_box(1, 1, 2)};
        return V::list({sizes(box.vertices().group_by(Axis::Z)), keys(box.vertices().group_by(Axis::Z)),
                        keys(box.vertices().group_by(Axis::Z, true)),
                        keys(box.vertices().group_by(Axis({0, 0, 0}, {1, 1, 0.5}), false, 2)),
                        sizes(box.edges().group_by(SortBy::LENGTH)), sizes(box.faces().group_by(SortBy::AREA)),
                        sizes(box.vertices().group_by(line)), keys(box.vertices().group_by(line)),
                        sizes(by_label), V::list(label_keys), V(by_label.group("A").size()), V(missing),
                        V(by_label.group_for(boxes[3]).size()), V(fused.group(true).size()),
                        V(fused.group(false).size()), V(cut.group(true).size()), V(cut.group(false).size()),
                        V((solids << SortBy::VOLUME).size()), V((solids >> SortBy::VOLUME).size()),
                        V((box.faces() >> Axis::X).size()), V((box.faces() << Axis::Z)[0].center())});
    });
    add_value("core_shapelist_filter", [] {
        const Solid box = Solid::make_box(1, 1, 1);
        const Part c1 = Cylinder(1, 3);
        const Part c2 = Cylinder(1, 3, Rotation(90, 0, 0));
        const Part bar = Box(1, 2, 3);
        const auto counts_by_position = [&](const Axis& axis, double lo, double hi) {
            std::vector<V> out;
            for (const auto& inclusive : {std::pair<bool, bool>{true, true}, {true, false}, {false, true}, {false, false}}) {
                out.emplace_back(bar.vertices().filter_by_position(axis, lo, hi, inclusive).size());
            }
            out.emplace_back(bar.faces().filter_by_position(axis, lo - 0.1, hi - 0.2).size());
            return V::list(out);
        };
        std::vector<V> positioned;
        for (const auto& f : bar.faces().filter_by_position(Axis({0, 0, 0}, {1, 0.3, 0.7}), -0.6, 0.6)) {
            positioned.emplace_back(f.center());
        }
        const Part pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4);
        return V::list({V(box.faces().filter_by(Axis::X).size()), V(box.edges().filter_by(Axis::X).size()),
                        V(box.vertices().filter_by(Axis::X).size()), V(c1.faces().filter_by(Plane::XY).size()),
                        V(c1.edges().filter_by(Plane::XY).size()), V(c2.faces().filter_by(Plane::XZ).size()),
                        V(c2.edges().filter_by(Plane::XZ).size()), V(c1.wires().filter_by(Plane::XY).size()),
                        V(c2.wires().filter_by(Plane::XZ).size()),
                        V((Pos(1, 2, 3) * c2).edges().filter_by(Plane::XZ).size()),
                        V(Solid::make_cylinder(1, 1).faces().filter_by(GeomType::PLANE, true).size()),
                        counts_by_position(Axis::Z, -1.5, 1.5), counts_by_position(Axis::Y, -1, 0),
                        V::list(positioned), V((box.edges() | Axis::Z).size()), V((c1.faces() | GeomType::PLANE).size()),
                        V((c1.edges() | Plane::XY).size()), V((pocket.edges() | Convexity::CONCAVE).size()),
                        V(box.edges().filter_by([](const Edge& e) { return e.center().Z > 0.5; }).size()),
                        V(box.edges().filter_by(Axis::Z, true).size())});
    });
    add_value("core_shapelist_ops", [] {
        const Part box = Box(1, 1, 1);
        const Part cyl = Cylinder(1, 1);
        const ShapeList<Shape> sl{box, cyl};
        const ShapeList<Shape> same{cyl, box};
        const ShapeList<Shape> repeated{cyl, box, box};
        const ShapeList<Shape> other{Box(1, 1, 1), Box(1, 2, 3)};
        const auto faces = Box(1, 2, 3).faces();
        const auto top = faces.sort_by(Axis::Z).slice(-2);
        const auto doubled = faces + faces;
        const auto measures = [](const auto& list) {
            std::vector<V> out;
            for (const auto& s : list) out.emplace_back(rnd(measure(s), 6));
            return V::list(out);
        };
        const auto centers = [](const auto& list) {
            std::vector<V> out;
            for (const auto& s : list) out.emplace_back(s.center());
            return V::list(out);
        };
        ShapeList<Face> appended = faces.slice(0, 1);
        appended += faces[1];
        appended += faces.slice(2, 4);
        const auto messages = [](auto f) -> V {
            try {
                f();
            } catch (const std::invalid_argument& e) {
                return V(std::string(e.what()));
            }
            return V();
        };
        ShapeList<Shape> nested;
        const Wire square = Wire::make_rect(1, 1);
        const Shell shell = Box(1, 1, 1).shells()[0];
        nested.push_back(Compound(std::vector<Shape>{Solid::make_box(1, 1, 1), square, Compound(std::vector<Shape>{shell})}));
        nested.push_back(Vertex(1, 2, 3));
        return V::list({V(sl == same), V(sl == repeated), V(sl != same), V(sl == other), V(sl != other),
                        describe(doubled - top), describe(faces - top), describe(doubled & top),
                        V((doubled - top).size()), V((doubled & top).size()), V((faces + faces[0]).size()),
                        V(appended.size()), centers(faces.slice(std::nullopt, std::nullopt, 2)),
                        centers(faces.slice(std::nullopt, std::nullopt, -1)), centers(faces.slice(1, -1, 2)),
                        centers(faces.slice(5, 1, -2)), centers(faces.slice(-2)), V(faces.slice(10).size()),
                        centers(faces.slice(-10, 2)), V(faces.first().center()), V(faces.last().center()),
                        V(ShapeList<Shape>().center()), V(ShapeList<Vertex>{Vertex(0, 0, 0), Vertex(1, 0, 0), Vertex(2, 0, 0)}.center()),
                        V(ShapeList<Shape>{box, Vertex(1, 1, 1)}.compound().volume()), V(sl.compounds().size()),
                        messages([&] { (void)sl.compound(); }), messages([&] { (void)faces.edge(); }),
                        messages([&] { (void)faces.solid(); }), describe(nested.expand()), measures(faces.slice(0, 2))});
    });

    // ---- 10: geometry operators and helpers
    add_value("core_geom_plane_lists", [] {
        const auto planes = [](const std::vector<Plane>& list) {
            std::vector<V> out;
            for (const auto& p : list) out.push_back(plane_v(p));
            return V::list(out);
        };
        const auto locations = [](const std::vector<Location>& list) {
            std::vector<V> out;
            for (const auto& l : list) out.push_back(loc_v(l));
            return V::list(out);
        };
        Plane moved = Plane::XY;
        moved.move(Plane::XZ.offset(2));
        Plane y_set = Plane::XY.offset(1);
        y_set.set_y_dir(Vector(1, 1, 0));
        return V::list({planes(Pos(1, 2, 3) * std::vector<Plane>{Plane::XY, Plane::XZ}),
                        planes(std::vector<Location>{Pos(1, 0, 0), Rot(0, 0, 90)} * Plane::XZ),
                        planes(std::vector<Plane>{Plane::XY.offset(1), Plane::YZ} * Plane::XZ),
                        locations(Plane::XZ * std::vector<Plane>{Plane::XY.offset(1), Plane::YZ}),
                        locations(Plane::XZ * std::vector<Location>{Pos(1, 2, 3), Rot(10, 0, 0)}),
                        plane_v(Plane::XY.moved(Plane::YZ.offset(3))), plane_v(moved), plane_v(y_set),
                        plane_v(Plane::from_points({1, 2, 3}, {2, 2, 3}, {1, 3, 4})),
                        loc_v(Rot(Vector(10, 20, 30))), loc_v(Rot(Vector(10, 20, 30), Extrinsic::ZYX)),
                        loc_v(Rot(Axis({1, 0, 0}, {0, 0, 1}), 90)), V(Vector(1, 2, 3) / 3),
                        V(Plane::get_topods_face_normal(TopoDS::Face(Face::make_rect(1, 1, Plane::XZ.offset(2)).wrapped())))});
    });
    add_value("core_geom_bound_box", [] {
        const auto box = [](Vector lo, Vector hi) { return Solid::make_box(hi.X - lo.X, hi.Y - lo.Y, hi.Z - lo.Z, Plane(lo)).bounding_box(); };
        const BoundBox a = box({0, 0, 0}, {4, 4, 4});
        const BoundBox inner = box({1, 1, 1}, {2, 2, 2});
        const BoundBox flush = box({0, 1, 1}, {2, 2, 2});
        const BoundBox beside = box({4, 0, 0}, {6, 4, 4});
        const BoundBox crossing = box({3, 1, 1}, {6, 2, 2});
        const BoundBox far = box({10, 10, 10}, {11, 11, 11});
        const BoundBox face = Face::make_rect(2, 2, Plane(Vector(2, 2, 4))).bounding_box();
        const BoundBox face2 = Face::make_rect(1, 1, Plane(Vector(2, 2, 4))).bounding_box();
        const BoundBox other_level = Face::make_rect(1, 1, Plane(Vector(2, 2, 5))).bounding_box();
        std::vector<V> table;
        for (const BoundBox* o : {&inner, &flush, &beside, &crossing, &far, &face, &a}) {
            table.push_back(V::list({V(a.covers(*o)), V(a.contains(*o)), V(a.contains_properly(*o)), V(o->within(a)),
                                     V(a.intersects(*o)), V(a.disjoint(*o)), V(a.touches(*o)), V(a.overlaps(*o))}));
        }
        const auto bbox_v = [](const BoundBox& b) { return V::list({V(b.min), V(b.max), V(b.size)}); };
        const auto outside = BoundBox::find_outside_box_2d(a, inner);
        const auto none = BoundBox::find_outside_box_2d(a, beside);
        return V::list({V::list(table), V(face.contains(face2)), V(face.contains_properly(face2)),
                        V(face.overlaps(face2)), V(face.touches(other_level)), V(face.contains(other_level)),
                        V(inner.is_inside(a)), outside ? bbox_v(*outside) : V(), none ? bbox_v(*none) : V(),
                        bbox_v(Plane::XZ.to_local_coords(inner)), bbox_v(Plane::XZ.offset(1).from_local_coords(inner)),
                        bbox_v(BoundBox(Box(1, 2, 3))), bbox_v(BoundBox(Shape())), V(inner.to_string()),
                        V(BoundBox(Shape()).diagonal())});
    });
    add_value("core_geom_color", [] {
        const auto t = [](const Color& c) { return V::list_of(c.to_tuple()); };
        const auto raises = [](auto f) -> V {
            try {
                f();
            } catch (const std::exception&) {
                return V(true);
            }
            return V(false);
        };
        std::vector<V> sets;
        for (const auto& set : {Color::categorical_set(3), Color::categorical_set(4, 0.3, std::vector<double>{0.1, 0.2, 0.3, 0.4}),
                                Color::categorical_set(2, 0xff8000), Color::categorical_set(5, 0.95, 0.5),
                                Color::categorical_set(0)}) {
            std::vector<V> colors;
            for (const auto& c : set) colors.push_back(t(c));
            sets.push_back(V::list(colors));
        }
        return V::list({t(Color()), t(Color("red")), t(Color(" Red ")), t(Color("goldenrod1")), t(Color("#ff0000", 0.5)),
                        t(Color("#f008")), t(Color("#FF0000", 0.3)), t(Color("#0f0")), t(Color(0xff0000, 0x80)),
                        t(Color(0xff0000, 1)), t(Color(0x000001)), t(Color(0.5)), t(Color(0.5, 0.2)),
                        t(Color(0.1, 0.2, 0.3, 0.3)), t(Color(0.5, 0.5, 0.5, 2.0)), t(Color("SteelBlue", 0.25)),
                        V(Color(1, 0, 0).to_string()), V(Color(0.1, 0.5, 0.9).str()), V(Color("aqua").str()),
                        V(Color(0.5, 0.5, 0.5).str()), V(Color("lightgrey").str()), V(Color("goldenrod1").str()),
                        raises([] { Color("dark red"); }), raises([] { Color("#gg0000"); }),
                        raises([] { Color("#ff0000zz"); }), raises([] { Color("#ff000"); }),
                        raises([] { Color("rebeccapurple"); }), raises([] { Color(2.0, 0, 0); }),
                        raises([] { Color(-0.1, 0, 0); }), raises([] { Color(0x1000000); }), V::list(sets),
                        raises([] { Color::categorical_set(3, 1.5); }),
                        raises([] { Color::categorical_set(3, 0.0, std::vector<double>{1.0}); }),
                        raises([] { Color::categorical_set(3, -1); })});
    });

    // ---- 11: missing topology members
    add_value("core_shape_members11", [] {
        const Part box = Box(2, 2, 2);
        const auto distances = box.distances({Pos(5, 0, 0) * Box(1, 1, 1), Pos(0, 0, 3) * Sphere(0.5), Box(1, 1, 1)});
        Part labeled = Box(1, 1, 1);
        labeled.label = "src";
        labeled.color = Color("red");
        Part target = Cylinder(1, 1);
        labeled.copy_attributes_to(target, {"_color"});
        Part target2 = Cylinder(1, 1);
        target2.label = "keep";
        labeled.copy_attributes_to(target2);
        return V::list({V::list_of(distances), V(box.entities(ShapeType::FACE).size()),
                        V(box.entities(ShapeType::EDGE).size()), V(box.entities(ShapeType::VERTEX).size()),
                        V(Shape::as_shape(Vector(1, 2, 3)).shape_type() == ShapeType::VERTEX),
                        V(Shape::as_shape(Axis::X).shape_type() == ShapeType::EDGE),
                        V(Shape::as_shape(Plane::XY).shape_type() == ShapeType::FACE), V(target.label),
                        V(target.color.has_value()), V(target2.label), V(target2.color.has_value())});
    });
    add_value("core_positions_deflection", [] {
        const Edge spline = Edge::make_spline({{0, 0, 0}, {3, 2, 0}, {6, -1, 1}, {9, 0, 0}});
        const Edge circle = Edge::make_circle(5);
        const Wire wire = Wire::make_polygon({{0, 0, 0}, {5, 0, 0}, {5, 5, 0}}, false);
        const auto summary = [](const std::vector<Vector>& pts) {
            Vector sum;
            for (const auto& p : pts) sum += p;
            return V::list({V(pts.size()), V(sum), pts.empty() ? V() : V(pts.front()), pts.empty() ? V() : V(pts.back())});
        };
        bool missing = false;
        try {
            (void)spline.positions(PositionsOptions{});
        } catch (const std::invalid_argument&) {
            missing = true;
        }
        return V::list({summary(spline.positions({.deflection = 0.01})), summary(circle.positions({.deflection = 0.05})),
                        summary(wire.positions({.deflection = 0.1})),
                        summary(spline.positions({.distances = std::vector<double>{0, 2, 4}, .position_mode = PositionMode::LENGTH})),
                        V(missing)});
    });
    add_value("core_curvature_comb", [] {
        const Edge spline = Edge::make_spline({{0, 0, 0}, {3, 2, 0}, {6, -1, 0}, {9, 0, 0}});
        const Wire ring(Edge::make_circle(4));
        bool not_xy = false;
        try {
            (void)Edge::make_circle(1, Plane::XZ).curvature_comb();
        } catch (const std::invalid_argument&) {
            not_xy = true;
        }
        return V::list({describe(spline.curvature_comb(20)), describe(ring.curvature_comb(12, 1.0)),
                        V(Edge::make_line({0, 0}, {5, 0}).curvature_comb().size()), V(not_xy)});
    });
    add_value("core_edge_members11", [] {
        const Edge arc = Edge::make_circle(5, Plane::XY, 0, 270);
        const Edge spline = Edge::make_spline({{0, 0, 0}, {3, 3, 0}, {6, 0, 0}, {9, 3, 0}});
        std::vector<V> locs;
        for (const auto& l : spline.distribute_locations(4, 0.1, 0.9)) locs.push_back(loc_v(l));
        std::vector<V> flat;
        for (const auto& l : arc.distribute_locations(3, 0, 1, true)) flat.push_back(loc_v(l));
        bool few = false;
        try {
            (void)arc.distribute_locations(1);
        } catch (const std::invalid_argument&) {
            few = true;
        }
        const Edge loop = Edge::make_spline({{0, 0, 0}, {4, 4, 0}, {4, 0, 0}, {0, 4, 0}});
        return V::list({V::list(locs), V::list(flat), V(few), V::list_of(arc.find_tangent(180)),
                        V::list_of(Edge::make_circle(3).find_tangent(45)), V::list_of(spline.find_tangent(0)),
                        V::list_of(Edge::make_line({0, 0}, {1, 1}).find_tangent(45)),
                        V::list_of(Edge::make_line({0, 0}, {1, 1}).find_tangent(10)),
                        describe(std::vector<Edge>{arc.trim(Vector(5, 0, 0), Vector(-5, 0, 0)), arc.trim(0.1, Vector(0, 5, 0)),
                                                   arc.trim_to_length(Vector(0, 5, 0), 3)}),
                        V::list_of(loop.find_intersection_points()),
                        V::list_of(spline.find_intersection_points(Edge::make_line({0, 1.5}, {9, 1.5}))),
                        V::list_of(spline.find_intersection_points(Axis({0, 1}, {1, 0})))});
    });
    add_value("core_wire_members11", [] {
        const Wire wire = Wire::make_polygon({{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 0}}, false);
        const Wire twin = Wire::make_polygon({{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 0}}, false);
        const Wire other = Wire::make_polygon({{0, 0, 0}, {4, 0, 0}, {4, 3.5, 0}}, false);
        const auto edges = wire.edges();
        const auto [first, second] = Wire::order_chamfer_edges(edges[1], {edges[0], edges[1]});
        bool not_in = false;
        try {
            (void)Wire::order_chamfer_edges(edges[2], {edges[0], edges[1]});
        } catch (const std::invalid_argument&) {
            not_in = true;
        }
        return V::list({describe(wire.trim(0.2, 0.7).edges()), describe(wire.trim(0.7, 0.2).edges()),
                        describe(wire.trim(Vector(1, 0, 0), Vector(4, 2, 0)).edges()), describe(wire.trim(0.0, Vector(2, 3, 0)).edges()),
                        V(wire.geom_equal(twin)), V(wire.geom_equal(other)), V(wire.param_at(0.0)), V(wire.param_at(0.5)),
                        V(wire.param_at(1.0)), V(first.is_same(edges[1])), V(second.is_same(edges[0])), V(not_in)});
    });
    add_value("core_face_members11", [] {
        const auto torus = Torus(5, 1).faces()[0];
        const auto cyl_face = Cylinder(2, 4).faces().filter_by(GeomType::CYLINDER)[0];
        const auto cone_face = Cone(2, 1, 3).faces().filter_by(GeomType::CONE)[0];
        const auto r = torus.radii();
        const auto plain = Face::make_rect(2, 2).radii();
        std::vector<ShapeList<Face>> sewn = Face::sew_faces(
            {Face::make_rect(1, 1), Face::make_rect(1, 1, Plane(Vector(1, 0, 0))), Face::make_rect(1, 1, Plane(Vector(5, 0, 0)))});
        std::vector<double> group_sizes;
        for (const auto& g : sewn) group_sizes.push_back(static_cast<double>(g.size()));
        std::sort(group_sizes.begin(), group_sizes.end());
        const V groups = V::list_of(group_sizes);
        const Face patch = Face::make_surface_patch({}, Wire::make_polygon({{0, 0, 0}, {4, 0, 1}, {4, 4, 0}, {0, 4, 1}}).edges(),
                                                    {Vector(2, 2, 1.5)});
        const Part base = Box(4, 4, 2, Align::MIN);
        const Face top = base.faces().sort_by(Axis::Z)[-1];
        const Face side = base.faces().sort_by(Axis::X)[0];
        const Face filled = Face::make_surface_patch({{top.edges().sort_by(Axis::X)[0], top, ContinuityLevel::C0}},
                                                     {}, {Vector(-1, 2, 3)});
        return V::list({r ? V::list({V(r->first), V(r->second)}) : V(), plain ? V(true) : V(),
                        describe(cyl_face.seams()), V(cone_face.semi_angle() ? V(*cone_face.semi_angle()) : V()),
                        V(Face::make_rect(1, 1).semi_angle().has_value()), describe(std::vector<Face>{cyl_face.uv_face(), Sphere(1).faces()[0].uv_face()}),
                        describe(std::vector<Face>{Face::make_rect(2, 2).offset(3), side.offset(-1)}), groups,
                        describe(std::vector<Face>{patch, filled}), V(patch.is_valid())});
    });
    add_value("core_axes_of_symmetry", [] {
        const auto dirs = [](const Face& f) {
            std::vector<std::array<double, 3>> rows;
            for (const auto& a : f.axes_of_symmetry()) {
                Vector d = a.direction;
                rows.push_back({rnd(d.X, 5), rnd(d.Y, 5), rnd(d.Z, 5)});
            }
            std::sort(rows.begin(), rows.end());
            std::vector<V> out;
            for (const auto& r : rows) out.push_back(V::list({r[0], r[1], r[2]}));
            return V::list(out);
        };
        const Sketch ell = Rectangle(4, 4) - Pos(1, 1) * Rectangle(2, 2);
        const Sketch holed = Rectangle(6, 4) - Pos(1.5, 0) * Circle(0.5);
        const Sketch slot = SlotOverall(8, 2);
        return V::list({dirs(Face::make_rect(4, 2)), dirs(Face::make_rect(3, 3)), dirs(ell.faces()[0]), dirs(holed.faces()[0]),
                        dirs(slot.faces()[0]), dirs(Triangle({.a = 3, .b = 4, .c = 5}).faces()[0])});
    });
    add_value("core_split_by_perimeter", [] {
        const Face square = Face::make_rect(10, 10);
        const Wire circle(Edge::make_circle(2));
        const auto inside = square.split_by_perimeter(circle);
        const auto outside = square.split_by_perimeter(circle, Keep::OUTSIDE);
        const auto [in_both, out_both] = square.split_by_perimeter(circle, Keep::BOTH);
        const Shell box_shell = Box(4, 4, 4).shells()[0];
        const Wire band = Wire::make_rect(1, 1, Plane(Vector(0, 0, 2)));
        const auto shell_inside = box_shell.split_by_perimeter(band);
        bool open = false;
        try {
            (void)square.split_by_perimeter(Edge::make_line({0, 0}, {1, 0}));
        } catch (const std::invalid_argument&) {
            open = true;
        }
        return V::list({describe(inside), describe(outside), describe(in_both), describe(out_both), describe(shell_inside),
                        V(open)});
    });
    add_value("core_wrap", [] {
        const Face surface = Cylinder(10, 30).faces().filter_by(GeomType::CYLINDER)[0];
        const Location loc(Plane(Vector(10, 0, 0), Vector(0, 1, 0), Vector(1, 0, 0)));
        const Edge wrapped_edge = surface.wrap(Edge::make_line({0, 0}, {5, 0}), loc);
        const Wire wrapped_wire = surface.wrap(Wire::make_polygon({{0, 0, 0}, {4, 0, 0}, {4, 3, 0}}, false), loc);
        const Face wrapped_face = surface.wrap(Face::make_rect(4, 2), loc);
        const auto faces = surface.wrap_faces({Face::make_rect(2, 1), Face::make_rect(1, 1, Plane(Vector(3, 0, 0)))},
                                              Edge::make_circle(10), 0.0);
        return V::list({V(wrapped_edge.length()), V(wrapped_edge.position_at(1)), V(wrapped_wire.length()),
                        V(wrapped_wire.position_at(1)), V(wrapped_face.area()), V(wrapped_face.center()),
                        V(wrapped_face.edges().size()), describe(faces)});
    });
    add("core_dprism_faces", [] {
        const Solid box = Solid::make_box(10, 10, 10);
        const Face top = box.faces().sort_by(Axis::Z)[-1];
        return box.dprism(top, std::vector<Face>{Face::make_rect(4, 4, Plane(Vector(5, 5, 10)))}, 3.0, 5.0, std::nullopt,
                          false, false);
    });
    add("core_dprism_wires", [] {
        const Solid box = Solid::make_box(10, 10, 10);
        return box.dprism(std::nullopt,
                          std::vector<Wire>{Wire::make_rect(6, 6, Plane(Vector(5, 5, 10))), Wire::make_circle(1, Plane(Vector(5, 5, 10)))},
                          std::nullopt, 0.0, std::nullopt, true, false);
    });
    add("core_hollow_open", [] {
        const Solid box = Solid::make_box(10, 10, 10);
        return box.hollow({box.faces().sort_by(Axis::Z)[-1]}, -1.0);
    });
    add("core_hollow_closed", [] { return Solid::make_box(10, 10, 10).hollow({}, 1.0, 1e-4, Kind::INTERSECTION); });
    add("core_twist_wire", [] {
        return Solid::extrude_linear_with_rotation(Wire::make_rect(4, 4), {0, 0, 0}, {0, 0, 10}, 45,
                                                   {Wire::make_circle(1)});
    });
    add("core_revolve_wire", [] {
        return Solid::revolve(Wire::make_rect(2, 2, Plane(Vector(5, 0, 0))), 270, Axis::Y,
                              {Wire::make_circle(0.5, Plane(Vector(5, 0, 0)))});
    });
    add("core_sweep_inner", [] {
        const Wire path(Edge::make_three_point_arc({0, 0, 0}, {5, 0, 5}, {10, 0, 0}));
        return Solid::sweep(Wire::make_circle(2, Plane::YZ), path, {Wire::make_circle(1, Plane::YZ)});
    });
    add_value("core_compound_members11", [] {
        const Shape a = Box(2, 2, 2);
        const Shape b = Pos(1, 0, 0) * Box(2, 2, 2);
        const Shape c = Pos(10, 0, 0) * Box(1, 1, 1);
        const Compound overlapping = Compound::make_assembly({a, b, c});
        const Compound apart = Compound::make_assembly({a, c});
        const auto [hit, pair, volume] = overlapping.do_children_intersect();
        const auto [miss, none, zero] = apart.do_children_intersect();
        const Compound prism = Compound::extrude(Box(1, 1, 1).shells()[0], Vector(0, 0, 2));
        const Curve triad = Compound::make_triad(10);
        return V::list({V(hit), pair.first ? V(pair.first->center()) : V(), pair.second ? V(pair.second->center()) : V(),
                        V(volume), V(miss), V(none.first.has_value()), V(zero), V(prism.faces().size()),
                        V(prism.compounds().size()), describe(triad.edges().slice(0, 9))});
    });
    add_value("core_sort_wires_by_build_order", [] {
        const auto groups = sort_wires_by_build_order(
            {Wire::make_rect(10, 10), Wire::make_circle(1), Wire::make_rect(2, 2, Plane(Vector(3, 3, 0)))});
        std::vector<V> out;
        for (const auto& g : groups) out.push_back(describe(g));
        return V::list({V(groups.size()), V::list(out), V(sort_wires_by_build_order({Wire::make_circle(1)}).size())});
    });

    // ---- copy_attributes_to: label, color and material survive like in build123d
    add_value("core_attributes_survive", [] {
        const auto attrs = [](const Shape& s) {
            return V::list({V(s.label), s.color ? V::list_of(s.color->to_tuple()) : V(),
                            s.material ? V(s.material->name()) : V()});
        };
        Part b = Box(1, 1, 1);
        b.label = "box";
        b.material = "aluminum";  // (build123d recolors from the material's PBR look; the color follows)
        b.color = Color("red");
        const Part s = Pos(0.5, 0, 0) * Sphere(0.6);
        const Part far = Pos(5, 0, 0) * Box(1, 1, 1);
        Sketch r = Rectangle(2, 2);
        r.label = "rect";
        r.color = Color("blue");
        Curve l = Line({0, 0}, {1, 0});
        l.label = "line";
        Compound c(std::vector<Shape>{Box(1, 1, 1)});
        c.label = "single";
        c.color = Color("green");
        Compound nested(std::vector<Shape>{Compound(std::vector<Shape>{Box(1, 1, 1)})});
        nested.label = "nested";
        return V::list({attrs(b + s), attrs(b + far), attrs(b - s), attrs(b & s), attrs(Pos(1, 0, 0) * b),
                        attrs(b.moved(Location(Vector(1, 2, 3)))), attrs(b.located(Location(Vector(1, 2, 3)))),
                        attrs(b.rotate(Axis::Z, 30)), attrs(b.translate(Vector(1, 0, 0))), attrs(b.mirror(Plane::YZ)),
                        attrs(b.scale(2)), attrs(b.clean()), attrs(b.solid()), attrs(b.unwrap()), attrs(r + Circle(1.5)),
                        attrs(r - Circle(0.5)), attrs(r & Circle(1.2)), attrs(l + Line({1, 0}, {1, 1})),
                        attrs(c.unwrap()), attrs(c.unwrap(false)), attrs(nested.unwrap()), attrs(nested.unwrap(false)),
                        attrs(b - std::vector<Part>{s, far}), attrs(fillet(b.edges().filter_by(Axis::Z), 0.1)),
                        attrs(b.transform_geometry(Matrix())), attrs(-r.faces()[0])});
    });

    // ---- 13: behaviour never compared before
    add_value("core_mixin1d_queries", [] {
        const Edge spline = Edge::make_spline({{0, 0, 0}, {3, 2, 1}, {6, -1, 2}, {9, 0, 0}});
        const Wire wire = Wire::make_polygon({{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 2}}, false);
        const Edge arc = Edge::make_circle(5, Plane::XZ, 10, 200);
        std::vector<V> derivatives;
        for (const Mixin1D* c : std::vector<const Mixin1D*>{&spline, &wire, &arc}) {
            for (int order : {1, 2, 3}) {
                derivatives.emplace_back(c->derivative_at(0.3, order));
                derivatives.emplace_back(c->derivative_at(0.3, order, PositionMode::LENGTH));
            }
            derivatives.emplace_back(c->derivative_at(c->position_at(0.6), 1));
        }
        std::vector<V> angles;
        for (double u : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            angles.emplace_back(arc.tangent_angle_at(u, PositionMode::PARAMETER, Plane::XZ));
            angles.emplace_back(Edge::make_circle(2).tangent_angle_at(u));
            angles.emplace_back(Edge::make_circle(2).tangent_angle_at(u * 3, PositionMode::LENGTH, Plane::XY.rotated({0, 0, 30})));
        }
        const auto planar = Edge::make_line({0, 0, 1}, {1, 1, 1}).common_plane({Edge::make_line({2, 0, 1}, {3, 4, 1})});
        const auto not_planar = Edge::make_line({0, 0, 0}, {1, 0, 0}).common_plane({Edge::make_line({0, 1, 1}, {0, 2, 3})});
        const auto line_only = Edge::make_line({0, 0, 0}, {1, 1, 0}).common_plane();
        std::vector<V> params;
        for (const auto& p : {Vector(1, 0.5, 0), Vector(4, 1.5, 0), Vector(2, 3, 1)}) params.emplace_back(wire.param_at_point(p));
        params.emplace_back(spline.param_at_point(spline.position_at(0.37)));
        std::vector<V> locs;
        for (const auto& l : spline.locations({0.1, 0.5, 0.9})) locs.push_back(loc_v(l));
        locs.push_back(loc_v(spline.location_at(0.4, PositionMode::PARAMETER, FrameMethod::FRENET, Vector(0, 0, 1))));
        locs.push_back(loc_v(wire.location_at(2.5, PositionMode::LENGTH, FrameMethod::CORRECTED)));
        locs.push_back(loc_v(spline.location_at(4.0, PositionMode::LENGTH, FrameMethod::CORRECTED)));
        bool no_normal = false;
        try {
            (void)Edge::make_line({0, 0, 0}, {1, 0, 0}).normal();
        } catch (const std::exception&) {
            no_normal = true;
        }
        V straight_normal;
        try {
            straight_normal = V(wire.edges()[0].normal() == Vector());
        } catch (const std::exception&) {
        }
        return V::list({V::list(derivatives), V::list(angles), planar ? plane_v(*planar) : V(), V(not_planar.has_value()),
                        line_only ? plane_v(*line_only) : V(), V::list(params), V::list(locs),
                        V(Edge::make_circle(3, Plane::XZ).normal()), straight_normal, V(no_normal)});
    });
    add_value("core_edge_wire_queries", [] {
        const Edge a = Edge::make_line({0, 0, 0}, {4, 0, 0});
        const Edge b = Edge::make_three_point_arc({0, 2, 0}, {2, 3, 0}, {4, 2, 0});
        const Edge rev = b.reversed();
        const Edge rebuilt = b.reversed(true);
        const Wire w1 = Wire::make_polygon({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, false);
        const Wire w2 = Wire::make_polygon({{1, 1, 0}, {0, 2, 0}, {-1, 1, 0}}, false);
        const Wire messy(std::vector<Edge>{Edge::make_line({0, 0}, {1, 0}), Edge::make_line({1, 1}, {1, 0}),
                                           Edge::make_line({1, 1}, {0, 1})});
        std::vector<V> ordered;
        for (const auto& e : messy.order_edges()) ordered.push_back(V::list({V(e.position_at(0)), V(e.position_at(1))}));
        const auto combined = Wire::combine(std::vector<Edge>{Edge::make_line({0, 0}, {1, 0}), Edge::make_line({1, 0}, {1, 1}),
                                                              Edge::make_line({5, 5}, {6, 5})});
        return V::list({describe(std::vector<Edge>{Edge::make_mid_way(a, b), Edge::make_mid_way(a, b, 0.25)}),
                        V(rev.position_at(0)), V(rev.tangent_at(0)), V(rebuilt.position_at(0)), V(rebuilt.tangent_at(0)),
                        V(rev.is_forward()), V(rebuilt.is_forward()), describe(w1.stitch(w2).edges()), V(w1.stitch(w2).length()),
                        V::list(ordered), V(combined.size()), describe(combined), V(w1.close().is_closed()), V(w1.close().length()),
                        V(Wire::make_ellipse(3, 2, Plane::XY, 0, 90, AngularDirection::COUNTER_CLOCKWISE, true).length()),
                        V(Wire::make_ellipse(3, 2, Plane::XY, 0, 90, AngularDirection::COUNTER_CLOCKWISE, false).length()),
                        V(Wire::make_ellipse(3, 2, Plane::XY, 0, 90, AngularDirection::COUNTER_CLOCKWISE, true).edges().size())});
    });
    add_value("core_face_queries", [] {
        const Face rect = Face::make_rect(4, 2, Plane::XZ.offset(1));
        const Face tilted = Face::make_rect(3, 1).rotate(Axis::Z, 30);
        const Face square = Face::make_rect(2, 2);
        const Face cyl = Cylinder(2, 4).faces().filter_by(GeomType::CYLINDER)[0];
        const Face holed = square.make_holes({Wire::make_circle(0.5)});
        const auto opt = [](const auto& o) { return o ? V(*o) : V(); };
        const Face ell = (Rectangle(4, 4) - Pos(1, 1) * Rectangle(2, 2)).faces()[0];
        const Face chamfered = square.chamfer_2d(0.5, 0.25, {square.vertices()[0]}, square.edges()[0]);
        return V::list({opt(rect.width()), opt(rect.length()), opt(tilted.width()), opt(tilted.length()), opt(cyl.width()),
                        opt(square.geometry()), opt(rect.geometry()), opt(ell.geometry()), opt(cyl.geometry()),
                        opt(cyl.radius()), opt(rect.radius()), V(cyl.axis_of_rotation() ? axis_v(*cyl.axis_of_rotation()) : V()),
                        V(rect.axis_of_rotation().has_value()), loc_v(rect.location_at(0.2, 0.7)),
                        loc_v(cyl.location_at(0.25, 0.5)), loc_v(rect.center_location()), loc_v(cyl.center_location()),
                        V(rect.is_coplanar(Plane::XZ.offset(1))), V(rect.is_coplanar(Plane::XZ)), V(square.is_inside(Vector(0.5, 0.5, 0))),
                        V(square.is_inside(Vector(3, 0, 0))), V(holed.area()), V(holed.without_holes().area()),
                        V(holed.inner_wires().size()), V(chamfered.area()), describe(chamfered.edges()),
                        describe(std::vector<Face>{Face::make_surface_from_curves(Edge::make_line({0, 0, 0}, {4, 0, 0}),
                                                                                   Edge::make_three_point_arc({0, 0, 3}, {2, 1, 3}, {4, 0, 3}))})});
    });
    add_value("core_shell_solid_queries", [] {
        const Shell lofted = Shell::make_loft({Wire::make_rect(4, 4), Wire::make_circle(1, Plane::XY.offset(3))});
        const Shell revolved = Shell::revolve(Edge::make_line({2, 0, 0}, {3, 0, 2}), 180, Axis::Z);
        const Shell extruded = Shell::extrude(Wire::make_polygon({{0, 0, 0}, {2, 0, 0}, {2, 1, 0}}, false), Vector(0, 0, 3));
        const Solid from_bbox = Solid::from_bounding_box((Rot(10, 20, 0) * Box(1, 2, 3)).bounding_box());
        const Part box = Box(4, 4, 4);
        const Compound mixed(std::vector<Shape>{Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1),
                                                Compound(std::vector<Shape>{Pos(0, 4, 0) * Box(1, 1, 1), Face::make_rect(1, 1)})});
        return V::list({describe(std::vector<Shell>{lofted, revolved, extruded}), V(from_bbox.volume()), V(from_bbox.center()),
                        V(box.is_inside(Vector(1, 1, 1))), V(box.is_inside(Vector(2, 0, 0))), V(box.is_inside(Vector(2.1, 0, 0))),
                        V(box.is_inside(Vector(2.0005, 0, 0), 1e-3)), V(mixed.get_type<Solid>().size()),
                        V(mixed.get_type<Face>().size()), V(mixed.get_type<Compound>().size()), V(mixed.elements().size())});
    });
    add_value("core_shape_queries", [] {
        const Part box = Box(2, 2, 2);
        const Part sphere = Pos(5, 1, 0) * Sphere(1);
        const auto [d, p1, p2] = box.distance_to_with_closest_points(sphere);
        const auto [dp, q1, q2] = box.distance_to_with_closest_points(Vector(4, 4, 4));
        const auto closest = box.closest_points(sphere);
        const auto hits = box.faces_intersected_by_axis(Axis({-5, 0.2, 0.3}, {1, 0, 0}));
        const auto [vertices, triangles] = Sphere(1).tessellate(0.1, 0.2);
        Vector vsum;
        for (const auto& v : vertices) vsum += v;
        const Part moved = box.transformed(Vector(10, 20, 30), Vector(1, 2, 3));
        const Part scaled = box.scale(Vector(1, 2, 3), Vector(1, 1, 1));
        const Part scaled2 = box.scale(2, Vector(1, 0, 0));
        return V::list({V(box.distance_to(sphere)), V(box.distance_to(Vector(4, 4, 4))), V(d), V(p1), V(p2), V(dp), V(q1), V(q2),
                        V(closest.first), V(closest.second), describe(hits), V(vertices.size()), V(triangles.size()), V(vsum),
                        V(moved.center()), V(moved.faces()[0].normal_at()), V(scaled.bounding_box().min), V(scaled.bounding_box().max),
                        V(scaled2.bounding_box().min), V(scaled2.bounding_box().max)});
    });
    add_value("core_skip_clean", [] {
        const Part a = Box(2, 2, 2);
        const Part b = Pos(2, 0, 0) * Box(2, 2, 2);
        std::size_t faces_skip;
        {
            SkipClean skip;
            faces_skip = (a + b).faces().size();
        }
        return V::list({V((a + b).faces().size()), V(faces_skip), V((a + b).faces().size())});
    });
    add_value("core_plane_helpers", [] {
        const Plane p = Plane::XY.offset(2).rotated({0, 0, 30});
        bool not_in = false;
        try {
            (void)p.shift_origin(Vector(0, 0, 5));
        } catch (const std::invalid_argument&) {
            not_in = true;
        }
        return V::list({plane_v(p.shift_origin(Vector(1, 1, 2))), plane_v(p.shift_origin(Axis({3, 4, 0}, {0, 0, 1}))),
                        plane_v(Plane::XZ.shift_origin(Vertex(1, 0, 4))), V(not_in),
                        loc_v(p.location_between(Plane::YZ.offset(1))), plane_v(p.rotated({10, 20, 30}, Extrinsic::XYZ))});
    });

    add_value("core_geom_with_shape", [] {
        const Part b = Box(2, 2, 2);
        return V::list({describe(Vector(0, 0, 0) & b), describe(Axis::Z & b), describe(Plane::XY & b),
                        describe(Location(Vector(0.5, 0, 0)) & b), describe(Axis::Z.intersect(b)),
                        describe(Plane::YZ.intersect(b))});
    });
}
