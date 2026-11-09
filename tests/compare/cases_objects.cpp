// Objects, operations, text and drafting compared with cases_objects_py.py.
#include "case_registry.hpp"

#include "../../src/optimize.hpp"

using namespace build123d;
using compare::V;

namespace {

/// The enum member's name, like Python's Enum.name ("MIN" for Align::MIN).
std::string name(Align a) {
    const std::string s = to_string(a);
    return s.substr(s.find('.') + 1);
}
V align_values(const Align2& a) { return V::list({name(a.x), name(a.y)}); }
V align_values(const Align3& a) { return V::list({name(a.x), name(a.y), name(a.z)}); }

/// A custom sketch object written like Python's custom objects (examples/custom_sketch_objects.py).
class Club : public BaseSketchObject {
public:
    explicit Club(double size, double rotation = 0, std::optional<Align2> align = std::nullopt)
        : BaseSketchObject(club(size), rotation, align), club_size(size) {}
    double club_size;

private:
    static Sketch club(double size) {
        return Pos(0, size / 3) * Circle(size / 4) + Pos(size / 4, 0) * Circle(size / 4) +
               Pos(-size / 4, 0) * Circle(size / 4) + Pos(0, -size / 3) * Rectangle(size / 6, size / 2);
    }
};

/// A custom part object.
class Bracket : public BasePartObject {
public:
    Bracket(double size, const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt)
        : BasePartObject(Box(size, size, size / 4, Align::MIN) + Box(size / 4, size, size, Align::MIN), rotation,
                         align) {}
};

V location_values(const Location& l) { return V::list({l.position(), l.orientation()}); }

V locations_values(const std::vector<Location>& locs) {
    std::vector<V> out;
    for (const auto& l : locs) out.push_back(location_values(l));
    return V::list(out);
}

/// Length and end points of each edge, in order.
V edges_values(const std::vector<Edge>& edges) {
    std::vector<V> out;
    for (const auto& e : edges) out.push_back(V::list({e.length(), e.position_at(0), e.position_at(1)}));
    return V::list(out);
}

V edge_values(const Edge& e) { return V::list({e.length(), e.position_at(0), e.position_at(1)}); }
V optional_edge_values(const std::optional<Edge>& e) { return e ? edge_values(*e) : V(nullptr); }

V nelder_mead_values(const std::function<double(const std::vector<double>&)>& f, const std::vector<double>& x0,
                     detail::NelderMeadOptions options) {
    const auto r = detail::minimize_nelder_mead(f, x0, options);
    return V::obj({{"x", V::list_of(r.x)}, {"fun", r.fun}, {"nit", r.nit}, {"nfev", r.nfev}, {"status", r.status}});
}

}  // namespace

COMPARE_CASES {
    // ---- base classes for custom objects
    compare::add("obj_base_sketch", [] {
        return BaseSketchObject(Rectangle(4, 2) + Pos(2, 0) * Circle(1), 30, Align2(Align::MIN, Align::MAX));
    });
    compare::add("obj_base_sketch_plain", [] { return BaseSketchObject(Rectangle(4, 2).face()); });
    compare::add("obj_base_part", [] {
        return BasePartObject(Solid::make_box(1, 2, 3), Rotation(10, 20, 30), Align3(Align::MIN, Align::CENTER, Align::MAX));
    });
    compare::add("obj_base_part_plain", [] { return BasePartObject(Pos(1, 2, 3) * Cylinder(1, 2)); });
    compare::add("obj_base_curve", [] {
        return BaseCurveObject(Curve(std::vector<Edge>{Line({0, 0}, {1, 0}), Line({1, 0}, {1, 1})}));
    });
    compare::add("obj_base_line", [] { return BaseLineObject(Polyline({{0, 0}, {1, 0}, {1, 1}})); });
    compare::add("obj_base_edge", [] { return BaseEdgeObject(Edge::make_line({0, 0}, {3, 4})); });
    compare::add("obj_custom_club", [] { return Club(10, 45, Align2(Align::MIN, Align::MIN)); });
    compare::add("obj_custom_bracket", [] { return Bracket(4, Rotation(0, 0, 30), Align3(Align::CENTER)); });
    compare::add_value("obj_custom_attrs", [] {
        const Club c(10, 45);
        const Bracket b(4, Rotation(0, 0, 30));
        return V::obj({{"club_rotation", c.rotation}, {"bracket_rotation", b.rotation.orientation()}});
    });

    // ---- object attributes (vars() of each object)
    compare::add_value("obj_attrs_sketch", [] {
        const Circle circle(2, 90, Align::MIN);
        const Ellipse ellipse(3, 2, 15);
        const Polygon polygon({{0, 0}, {1, 0}, {1, 1}}, 10, Align::CENTER);
        const Rectangle rect(3, 2, 5);
        const RectangleRounded rr(3, 2, 0.5);
        const RegularPolygon rp(3, 6, false, 20);
        const SlotArc sa(CenterArc({0, 0}, 5, 0, 90), 1, 10);
        const SlotCenterPoint scp({0, 0}, {2, 0}, 1, 5);
        const SlotCenterToCenter sctc(3, 1, 7);
        const SlotOverall so(4, 1, 8);
        const Superellipse se(4, 2, 3, 12, 9);
        const Trapezoid tz(5, 2, 60, 0.0, 4);
        const Trapezoid tz2(5, 2, 60, 75.0);
        const Triangle tri({.a = 3, .b = 4, .c = 5}, Align::NONE, 12);
        std::vector<V> pts;
        for (const auto& p : polygon.pts) pts.emplace_back(p);
        return V::obj({
            {"circle", V::list({circle.radius, circle.arc_size, align_values(circle.align), circle.rotation})},
            {"ellipse", V::list({ellipse.x_radius, ellipse.y_radius, align_values(ellipse.align), ellipse.rotation})},
            {"polygon", V::list({V::list(pts), align_values(polygon.align), polygon.rotation})},
            {"rect", V::list({rect.width, rect.rectangle_height, align_values(rect.align), rect.rotation})},
            {"rect_rounded", V::list({rr.width, rr.rectangle_height, rr.radius, align_values(rr.align), rr.rotation})},
            {"regular_polygon",
             V::list({rp.radius, rp.apothem, rp.side_count, align_values(rp.align), rp.rotation})},
            {"slot_arc", V::list({sa.arc.wrapped().ShapeType() == TopAbs_EDGE, sa.slot_height, sa.rotation})},
            {"slot_center_point", V::list({scp.slot_center, scp.point, scp.slot_height, scp.rotation})},
            {"slot_center_to_center", V::list({sctc.center_separation, sctc.slot_height, sctc.rotation})},
            {"slot_overall", V::list({so.width, so.slot_height, so.rotation})},
            {"superellipse",
             V::list({se.width, se.height_, se.order, se.point_count, align_values(se.align), se.rotation})},
            {"trapezoid", V::list({tz.width, tz.trapezoid_height, tz.left_side_angle, tz.right_side_angle,
                                   align_values(tz.align), tz.rotation, tz2.right_side_angle})},
            {"triangle", V::list({tri.a, tri.b, tri.c, tri.A, tri.B, tri.C, tri.rotation})},
        });
    });
    compare::add_value("obj_attrs_part", [] {
        const Box box(1, 2, 3, Rotation(10, 0, 0));
        const Cone cone(2, 1, 3, 270, Rotation(), Align::MIN);
        const ConvexPolyhedron cp({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, Rotation(0, 0, 5));
        const CounterBoreHole cbh(1, 2, 1, 5);
        const CounterSinkHole csh(1, 2, 5, 90);
        const Cylinder cyl(1, 2, 180, Rotation(), Align3(Align::MIN, Align::CENTER, Align::MAX));
        const Hole hole(1, 5);
        const Sphere sphere(2, -45, 60, 270);
        const Torus torus(3, 1, 10, 300, 270);
        const Wedge wedge(1, 2, 3, 0, 0, 1, 1);
        return V::obj({
            {"box", V::list({box.length, box.width, box.box_height, box.rotation.orientation()})},
            {"cone", V::list({cone.bottom_radius, cone.top_radius, cone.cone_height, cone.arc_size,
                              align_values(cone.align)})},
            {"convex_polyhedron", V::list({cp.rotation.orientation()})},
            {"counter_bore_hole",
             V::list({cbh.radius, cbh.counter_bore_radius, cbh.counter_bore_depth, cbh.hole_depth})},
            {"counter_sink_hole",
             V::list({csh.radius, csh.counter_sink_radius, csh.hole_depth, csh.counter_sink_angle})},
            {"cylinder", V::list({cyl.radius, cyl.cylinder_height, cyl.arc_size, align_values(cyl.align)})},
            {"hole", V::list({hole.radius, hole.hole_depth})},
            {"sphere", V::list({sphere.radius, sphere.arc_size1, sphere.arc_size2, sphere.arc_size3,
                                align_values(sphere.align)})},
            {"torus", V::list({torus.major_radius, torus.minor_radius, torus.minor_start_angle,
                               torus.minor_end_angle, torus.major_angle, align_values(torus.align)})},
            {"wedge", V::list({wedge.xsize, wedge.ysize, wedge.zsize, wedge.xmin, wedge.zmin, wedge.xmax,
                               wedge.zmax, align_values(wedge.align)})},
        });
    });
    compare::add("obj_hole_no_depth", [] { return Hole(1); });
    compare::add("obj_counter_bore_no_depth", [] { return CounterBoreHole(1, 2, 1); });
    compare::add("obj_counter_sink_no_depth", [] { return CounterSinkHole(1, 2); });

    // ---- Locations
    compare::add_value("obj_locations_faces", [] {
        return locations_values(Locations((Rot(10, 20, 30) * Box(1, 2, 3)).faces()));
    });
    compare::add_value("obj_locations_faces_explicit", [] {
        return locations_values(Locations((Pos(1, 0, 0) * Cylinder(1, 2)).faces()));
    });
    compare::add_value("obj_locations_vertices", [] {
        return locations_values(Locations((Pos(1, 2, 3) * Box(1, 2, 3)).vertices()));
    });
    compare::add_value("obj_locations_planes", [] {
        return locations_values(Locations(std::vector<Plane>{Plane::XY, Plane::XZ.offset(2), Plane::isometric}));
    });
    compare::add_value("obj_locations_axes", [] {
        return locations_values(Locations(Axis::X, Axis(Vector(1, 2, 3), Vector(0, 1, 1))));
    });
    compare::add_value("obj_locations_axis_list", [] {
        return locations_values(Locations(std::vector<Axis>{Axis::Y, Axis::Z}));
    });
    compare::add_value("obj_locations_mixed", [] {
        const auto face = Box(1, 2, 3).faces().sort_by(Axis::X).last();
        return locations_values(Locations(Vector(1, 2), Pos(3, 4) * Rot(0, 0, 30), Axis::Z, Plane::YZ,
                                          Vertex(1, 1, 1), face, std::vector<Vector>{{5, 5}, {6, 6}}));
    });
    compare::add("obj_locations_faces_shape", [] {
        const Part box = Box(4, 4, 4);
        return box + Locations(box.faces()) * Cylinder(0.5, 1);
    });
    compare::add("obj_locations_mixed_shape", [] {
        return Sketch() + Locations(Vector(0, 0), Pos(5, 0) * Rot(0, 0, 45), Plane::XY.offset(1),
                                    Axis(Vector(-5, 0, 0), Vector(0, 0, 1))) *
                              Rectangle(2, 1);
    });
    compare::add_value("obj_location_list_attrs", [] {
        const GridLocations grid(2, 3, 3, 2, Align2(Align::MIN, Align::MAX));
        const HexLocations hex(1, 3, 2, true, Align::MIN);
        const PolarLocations polar(3, 4, 10, 180, false, true);
        const Locations locs(Vector(1, 2), Vector(3, 4));
        return V::obj({
            {"grid", V::list({grid.x_spacing, grid.y_spacing, grid.x_count, grid.y_count, align_values(grid.align),
                              grid.size, grid.min, grid.max, static_cast<double>(grid.planes.size()),
                              locations_values(grid.local_locations())})},
            {"hex", V::list({hex.radius, hex.apothem, hex.diagonal, hex.x_count, hex.y_count, hex.major_radius,
                             align_values(hex.align), locations_values(hex.local_locations())})},
            {"polar", locations_values(polar.local_locations())},
            {"locations", locations_values(locs.local_locations())},
        });
    });

    // ---- scipy's Nelder-Mead (used by DoubleTangentArc)
    compare::add_value("obj_nelder_mead", [] {
        using Opt = detail::NelderMeadOptions;
        Opt bounded;
        bounded.bounds = std::vector<std::pair<double, double>>{{0.0, 10.0}};
        bounded.xatol = bounded.fatol = 1e-6;
        Opt sine;
        sine.bounds = std::vector<std::pair<double, double>>{{0.0, 3.0}};
        sine.xatol = sine.fatol = 1e-6;
        Opt near_top;
        near_top.bounds = std::vector<std::pair<double, double>>{{0.0, 10.0}};
        Opt box2;
        box2.bounds = std::vector<std::pair<double, double>>{{0.0, 0.5}, {-1.0, 1.0}};
        box2.xatol = box2.fatol = 1e-8;
        Opt capped;
        capped.maxiter = 15;
        auto rosen = [](const std::vector<double>& x) {
            return 100 * std::pow(x[1] - x[0] * x[0], 2) + std::pow(1 - x[0], 2);
        };
        return V::list({
            nelder_mead_values([](const std::vector<double>& x) { return (x[0] - 3) * (x[0] - 3) + 1; }, {0.0}, bounded),
            nelder_mead_values([](const std::vector<double>& x) { return std::abs(std::sin(x[0]) - 0.5); }, {0.0}, sine),
            nelder_mead_values([](const std::vector<double>& x) { return (x[0] - 5) * (x[0] - 5); }, {9.99}, near_top),
            nelder_mead_values(rosen, {-1.2, 1.0}, Opt{}),
            nelder_mead_values(rosen, {0.4, 0.4}, box2),
            nelder_mead_values(rosen, {-1.2, 1.0}, capped),
        });
    });

    compare::add_value("obj_minimize_scalar_bounded", [] {
        auto run = [](const std::function<double(double)>& f, double lo, double hi, double xatol) {
            const auto r = detail::minimize_scalar_bounded(f, lo, hi, xatol);
            return V::list({r.x, r.fun, r.nfev, r.status});
        };
        return V::list({
            run([](double x) { return (x - 0.3) * (x - 0.3); }, 0, 1, 5e-7),
            run([](double x) { return std::abs(std::sin(5 * x) - 0.2); }, 0, 1, 1e-5),
            run([](double x) { return std::cos(x); }, 0.5, 0.75, 5e-7),
            run([](double x) { return std::exp(x); }, -2, 3, 1e-5),
        });
    });

    // ---- BlendCurve
    compare::add("obj_blend_c0", [] {
        return BlendCurve(Line({0, 0}, {1, 0}), Line({2, 0}, {2, 1}), ContinuityLevel::C0);
    });
    compare::add("obj_blend_c2_auto", [] {
        const Edge m1 = Pos(-10, 3) * CenterArc({0, 0}, 5, -10, 200).reversed();
        const Edge m2 = Pos(5, -13) * Spline({{-3, 9}, {0, 0}, {3, 9}});
        return BlendCurve(m1, m2);
    });
    compare::add_value("obj_blend_c1_c2_scalars", [] {
        const Edge m1 = Pos(-10, 3) * CenterArc({0, 0}, 5, -10, 200).reversed();
        const Edge m2 = Pos(5, -13) * Spline({{-3, 9}, {0, 0}, {3, 9}});
        std::vector<V> out;
        for (auto c : {ContinuityLevel::C1, ContinuityLevel::C2}) {
            const BlendCurve bc(m1, m2, c, std::make_pair(m1.position_at(1), m2.position_at(0)),
                                std::make_pair(1.7, 0.8));
            out.push_back(V::list({bc.length(), bc.derivative_at(0, 1), bc.derivative_at(1, 1), bc.derivative_at(0, 2),
                                   bc.derivative_at(1, 2), bc.position_at(0.3)}));
        }
        return V::list(out);
    });
    compare::add("obj_blend_bad_end_point", [] {
        const Edge m1 = CenterArc({0, 0}, 5, -10, 200);
        return BlendCurve(m1, Line({10, 0}, {12, 3}), ContinuityLevel::C1,
                          std::make_pair(m1.position_at(0.5), Vector(10, 0)));
    });

    // ---- DoubleTangentArc
    compare::add("obj_double_tangent_line", [] { return DoubleTangentArc({0, 0}, {1, 0}, Line({0, 5}, {10, 5})); });
    compare::add("obj_double_tangent_bottom", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Line({0, 5}, {10, 5}), Keep::BOTTOM);
    });
    compare::add("obj_double_tangent_wire", [] {
        return DoubleTangentArc({0, 0}, {1, 1}, Polyline({{6, 0}, {6, 8}, {10, 12}}));
    });
    compare::add("obj_double_tangent_arc_target", [] {
        return DoubleTangentArc({0, 0}, {0, 1}, CenterArc({10, 0}, 3, 90, 180));
    });
    compare::add("obj_double_tangent_spline", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Spline({{-5, 8}, {0, 6}, {6, 9}}), Keep::BOTTOM);
    });
    compare::add("obj_double_tangent_curve_error", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Curve(Line({0, 5}, {10, 5})));
    });
    compare::add("obj_double_tangent_keep_error", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Line({0, 5}, {10, 5}), Keep::BOTH);
    });

    // ---- Airfoil
    compare::add("obj_airfoil_2412", [] { return Airfoil("2412"); });
    compare::add("obj_airfoil_finite", [] { return Airfoil("NACA 2412", 40, true); });
    compare::add("obj_airfoil_0012", [] { return Airfoil("0012", 60); });
    compare::add("obj_airfoil_fraction", [] { return Airfoil("2213.323", 30); });
    compare::add("obj_airfoil_number", [] { return Airfoil(4415.0, 25, true); });
    compare::add("obj_airfoil_camber", [] { return Airfoil("4412", 40).camber_line(); });
    compare::add_value("obj_airfoil_attrs", [] {
        const Airfoil a("2412", 40);
        const auto [m, p, t] = Airfoil::parse_naca4("NACA 2213.323");
        const auto [m2, p2, t2] = Airfoil::parse_naca4(6409.0);
        return V::list({a.code, a.max_camber, a.camber_pos, a.thickness, a.finite_te, m, p, t, m2, p2, t2});
    });
    compare::add("obj_airfoil_bad_code", [] { return Airfoil("2x12"); });

    // ---- ConstrainedArcs / Edge::make_constrained_arcs
    compare::add_value("obj_carcs_2tan_rad_lines", [] {
        const Line e1({-2, 0}, {2, 0}), e2({0, -2}, {0, 2});
        return V::list({edges_values(Edge::make_constrained_arcs(e1, e2, {.radius = 0.5, .sagitta = Sagitta::BOTH})),
                        edges_values(Edge::make_constrained_arcs(e1, e2, {.radius = 0.5})),
                        edges_values(Edge::make_constrained_arcs(TangencyTarget(e1, Tangency::UNQUALIFIED),
                                                                 TangencyTarget(e2, Tangency::UNQUALIFIED),
                                                                 {.radius = 0.5, .sagitta = Sagitta::LONG}))});
    });
    compare::add_value("obj_carcs_2tan_rad_arc_line", [] {
        return edges_values(Edge::make_constrained_arcs(CenterArc({0, 0}, 1, 0, 90), Line({1, 0}, {2, 0}),
                                                        {.radius = 0.5}));
    });
    compare::add_value("obj_carcs_2tan_rad_points", [] {
        return V::list({edges_values(Edge::make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), {.radius = 0.5})),
                        edges_values(Edge::make_constrained_arcs(Vertex(0, 0), Vertex(0, 0.5), {.radius = 0.5})),
                        edges_values(Edge::make_constrained_arcs({0, 0.5}, Line({0, 0}, {1, 0}), {.radius = 0.5}))});
    });
    compare::add("obj_carcs_2tan_rad_none", [] {
        return Curve(Edge::make_constrained_arcs(Vector(0, 0), Vector(10, 0), {.radius = 2}));
    });
    compare::add_value("obj_carcs_2tan_on", [] {
        const Edge c1 = PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL);
        const Line c2({4, -2}, {4, 2}), on({3, -2}, {3, 2});
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(c1, Tangency::UNQUALIFIED),
                                                     TangencyTarget(c2, Tangency::UNQUALIFIED), {.center_on = on})),
            edges_values(Edge::make_constrained_arcs(Vector(0, 3), Vector(5, 0), {.center_on = Line({0, -5}, {0, 5})})),
            edges_values(Edge::make_constrained_arcs(Line({-5, 3}, {5, 3}), Vector(5, 0), {.center_on = Axis::Y})),
            edges_values(Edge::make_constrained_arcs(Line({-10, 0}, {10, 0}), Line({0, -10}, {0, 10}),
                                                     {.center_on = Line({-10, 1}, {10, 1}), .sagitta = Sagitta::BOTH})),
            edges_values(Edge::make_constrained_arcs(Line({-10, 0}, {10, 0}), Line({0, -10}, {0, 10}),
                                                     {.center_on = Line({3, -10}, {3, 10}), .sagitta = Sagitta::LONG})),
            edges_values(Edge::make_constrained_arcs(Line({0, 0}, {0.01, 0}), Line({10, -10}, {10, 10}),
                                                     {.center_on = Line({5, -10}, {5, 10})})),
        });
    });
    compare::add("obj_carcs_2tan_on_none", [] {
        return Curve(Edge::make_constrained_arcs(Line({-5, 3}, {5, 3}), Line({-5, 0}, {5, 0}),
                                                 {.center_on = Line({-5, -1}, {5, -1})}));
    });
    compare::add("obj_carcs_2tan_on_trim", [] {
        const CenterArc c1({0, 20}, 20, -90, 90);
        const Line ln2(c1.position_at(1), {10, 20});
        const Line ln3({10, 10}, ln2.position_at(1));
        return ConstrainedArcs(c1, ln2, {.center_on = ln3});
    });
    compare::add_value("obj_carcs_tan_cen", [] {
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(PolarLine({0, 0}, 4, 60), Tangency::UNQUALIFIED),
                                                     {.center = Vector(2, 1)})),
            edges_values(Edge::make_constrained_arcs(Axis::X, {.center = Vector(2, 1, 5)})),
            edges_values(Edge::make_constrained_arcs(CenterArc({0, 0}, 1, 180, 5), {.center = Vector(2, 0)})),
            edges_values(Edge::make_constrained_arcs({-2.5, 1.5}, {.center = Vector(-2, 1)})),
            edges_values(Edge::make_constrained_arcs({-2.5, 1.5}, {.center = Vertex(-2, 1)})),
            edges_values(Edge::make_constrained_arcs({-2.5, 1.5}, {.center = Pos(1, 0) * Vertex(-2, 1)})),
            edges_values(Edge::make_constrained_arcs(Vector(0, 0), {.center = Vector(0, 0)})),
        });
    });
    compare::add_value("obj_carcs_tan_on_rad", [] {
        const Edge c1 = PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL);
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(c1, Tangency::UNQUALIFIED),
                                                     {.radius = 1, .center_on = Line({3, -2}, {3, 2})})),
            edges_values(Edge::make_constrained_arcs(c1, {.radius = 1, .center_on = Axis::X})),
            edges_values(Edge::make_constrained_arcs(Vector(1, 1), {.radius = 2, .center_on = Axis::X})),
        });
    });
    compare::add("obj_carcs_tan_on_rad_none", [] {
        return Curve(Edge::make_constrained_arcs(Line({0, 10}, {10, 10}), {.radius = 1, .center_on = Axis::X}));
    });
    compare::add_value("obj_carcs_3tan", [] {
        const Edge c5 = PolarLine({0, 0}, 4, 60), c6 = PolarLine({0, 0}, 4, 40);
        const CenterArc c7({0, 0}, 4, 0, 90);
        const CenterArc a1({0, 20}, 20, -90, 90), a2({5, 25}, 10, -90, 90);
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(c5, Tangency::UNQUALIFIED),
                                                     TangencyTarget(c6, Tangency::UNQUALIFIED),
                                                     TangencyTarget(c7, Tangency::UNQUALIFIED))),
            edges_values(Edge::make_constrained_arcs(c5, c6, c7, {.sagitta = Sagitta::BOTH})),
            edges_values(Edge::make_constrained_arcs(Line({-1, 0}, {-1, 2}), Line({1, 0}, {1, 2}),
                                                     Line({-1, 0}, {-0.75, 0}))),
            edges_values(Edge::make_constrained_arcs(a1, a2, Line(a1.position_at(1), a2.position_at(1)))),
            edges_values(Edge::make_constrained_arcs(a1, a2, RadiusArc(a1.position_at(1), a2.position_at(1), 10))),
        });
    });
    compare::add("obj_carcs_3tan_parallel", [] {
        return Curve(Edge::make_constrained_arcs(Line({0, 0}, {10, 0}), Line({0, 2}, {10, 2}), Line({0, 5}, {10, 5})));
    });
    compare::add("obj_carcs_eggplant", [] {
        const CenterArc left({-2, 0}, 0.75, 0, 360), right({2, 0}, 1.0, 0, 360);
        const Edge bottom = Edge::make_constrained_arcs(TangencyTarget(right, Tangency::OUTSIDE),
                                                        TangencyTarget(left, Tangency::OUTSIDE), {.radius = 6})
                                .sort_by(Axis::Y)[0];
        const Edge top = Edge::make_constrained_arcs(TangencyTarget(right, Tangency::ENCLOSING),
                                                     TangencyTarget(left, Tangency::ENCLOSING), {.radius = 8})
                             .sort_by(Axis::Y)[-1];
        const ThreePointArc r(bottom.vertices().sort_by(Axis::X)[-1], right.position_at(0),
                              top.vertices().sort_by(Axis::X)[-1]);
        const ThreePointArc l(bottom.vertices().sort_by(Axis::X)[0], left.position_at(0.5),
                              top.vertices().sort_by(Axis::X)[0]);
        return make_face(std::vector<Edge>{l, top, r, bottom});
    });
    compare::add("obj_carcs_object_selector", [] {
        return ConstrainedArcs(Edge::make_circle(1.0), Axis::Y, {.radius = 1},
                               [](const ShapeList<Edge>& arcs) { return arcs.group_by(Axis::Y)[-1]; });
    });
    compare::add("obj_carcs_object_all", [] {
        return ConstrainedArcs(CenterArc({-5, 0}, 4, 0, 360), CenterArc({5, 0}, 3, 0, 360), {.radius = 10});
    });
    compare::add("obj_carcs_object_first", [] {
        return ConstrainedArcs(PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL), Line({4, -2}, {4, 2}),
                               {.center_on = Axis(Vector(3, 0), Vector(0, 1))},
                               [](const ShapeList<Edge>& arcs) { return arcs[0]; });
    });
    compare::add("obj_carcs_object_shortest", [] {
        return ConstrainedArcs(PolarLine({0, 0}, 4, 60), PolarLine({0, 0}, 4, 40), CenterArc({0, 0}, 4, 0, 90),
                               {.sagitta = Sagitta::BOTH},
                               [](const ShapeList<Edge>& arcs) { return arcs.sort_by(SortBy::LENGTH)[0]; });
    });
    compare::add("obj_carcs_object_center", [] {
        return ConstrainedArcs(Axis::Y, {.center = Vector(-2, 1)}, [](const ShapeList<Edge>& arcs) { return arcs[0]; });
    });
    compare::add("obj_carcs_object_tan_on_rad", [] {
        return ConstrainedArcs(PolarLine({0, 0}, 5, -20), {.radius = 1, .center_on = Line({3, -2}, {3, 2})});
    });
    compare::add("obj_carcs_object_empty_selection", [] {
        return ConstrainedArcs(Edge::make_circle(1.0), Axis::Y, {.radius = 1},
                               [](const ShapeList<Edge>& arcs) { return arcs.filter_by(GeomType::LINE); });
    });
    compare::add("obj_carcs_axis_tangency_error", [] {
        return Curve(Edge::make_constrained_arcs(TangencyTarget(Axis::X, Tangency::ENCLOSING),
                                                 CenterArc({0, 5}, 2, 0, 360), {.radius = 3}));
    });
    compare::add("obj_carcs_ambiguous_error", [] {
        return Curve(Edge::make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), {.radius = 0.5, .center = Vector(0, 0.25)}));
    });
    compare::add("obj_carcs_radius_error", [] {
        return Curve(Edge::make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), {.radius = -0.5}));
    });
    compare::add("obj_carcs_center_on_face_error", [] {
        return Curve(Edge::make_constrained_arcs(PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL),
                                                 {.radius = 1, .center_on = Vector(1, 1)}));
    });
    compare::add("obj_carcs_3tan_points_error", [] {
        return Curve(Edge::make_constrained_arcs(Line({0, 0}, {1, 0}), Line({0, 0}, {0, 1}), Vector(3, 3)));
    });

    // ---- ConstrainedLines / Edge::make_constrained_lines
    compare::add_value("obj_clines", [] {
        const Edge unit = Edge::make_circle(1.0);
        const Edge moved = unit.translate({3, 0, 0});
        return V::list({
            edges_values(Edge::make_constrained_lines(unit, moved)),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING), moved)),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING),
                                                      TangencyTarget(moved, Tangency::ENCLOSING))),
            edges_values(Edge::make_constrained_lines(unit, Vector(2, 0))),
            edges_values(Edge::make_constrained_lines(unit, {3, 0})),
            edges_values(Edge::make_constrained_lines(unit, unit)),
            edges_values(Edge::make_constrained_lines(unit, Axis::X, {.angle = 30})),
            edges_values(Edge::make_constrained_lines(unit, Axis::Y, {.angle = 30})),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING), Axis::Y, {.angle = 30})),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING), Axis::Y,
                                                      {.direction = Vector(1, 1)})),
            edges_values(Edge::make_constrained_lines(unit, Axis::X, {.angle = 0})),
            edges_values(Edge::make_constrained_lines(unit, Axis(Vector(1, 0), Vector(0, 1)), {.angle = 0})),
            edges_values(Edge::make_constrained_lines(Vector(0, 6), CenterArc({-5, 0}, 4, 0, 360))),
            edges_values(Edge::make_constrained_lines(Axis::X, Edge::make_circle(1.0, Plane::XY.offset(0)).translate({0, 3}))),
        });
    });
    compare::add("obj_clines_none", [] { return Curve(Edge::make_constrained_lines(Edge::make_circle(1.0), Vector(0, 0))); });
    compare::add("obj_clines_vertical_ref_error", [] {
        return Curve(Edge::make_constrained_lines(Edge::make_circle(1.0), Axis::Z, {.angle = 1}));
    });
    compare::add("obj_clines_two_points_error", [] {
        return Curve(Edge::make_constrained_lines(Vector(0, 0), Vector(1, 1)));
    });
    compare::add("obj_clines_object_all", [] {
        return ConstrainedLines(CenterArc({-5, 0}, 4, 0, 360), CenterArc({5, 0}, 3, 0, 360));
    });
    compare::add("obj_clines_object_first", [] {
        return ConstrainedLines(CenterArc({-5, 0}, 4, 0, 360), {0, 6}, {},
                                [](const ShapeList<Edge>& lines) { return lines[0]; });
    });
    compare::add("obj_clines_object_closest", [] {
        return ConstrainedLines(CenterArc({-5, 0}, 4, 0, 360), Axis::Y, {.angle = 30},
                                [](const ShapeList<Edge>& lines) { return lines.sort_by_distance(Vector(0, 0))[0]; });
    });

    // ---- limits: Edge::trim_to_other, trim_infinite, arc_size/length limits
    compare::add_value("obj_trim_to_other", [] {
        const double h = std::sqrt(2.0) / 2;
        const std::vector<Edge> bases{CenterArc({0, 0}, 1, 0, 180), CenterArc({0, 0}, 1, 0, 360)};
        const Sketch rect = Rectangle(1, 1, Align2(Align::MAX, Align::MIN)).rotate(Axis::Z, -45);
        std::vector<V> out;
        for (const Edge& base : bases) {
            out.push_back(optional_edge_values(base.trim_to_other(Line({0, 0}, {1, 1}))));
            out.push_back(optional_edge_values(base.trim_to_other(CenterArc({0, 0}, 1, 45, 90))));
            out.push_back(optional_edge_values(base.trim_to_other(rect.wire())));
            out.push_back(optional_edge_values(base.trim_to_other(rect)));
            out.push_back(optional_edge_values(base.trim_to_other(Curve(std::vector<Edge>(rect.edges())))));
            out.push_back(optional_edge_values(
                base.trim_to_other(Box(1, 1, 1, Align3(Align::MAX, Align::MIN, Align::CENTER)).rotate(Axis::Z, -45))));
            out.push_back(optional_edge_values(base.trim_to_other(Axis(Vector(0, 0, 0), Vector(1, 1, 0)))));
            out.push_back(optional_edge_values(base.trim_to_other(Location(Vector(h, h), Vector(0, 0, 1)))));
            out.push_back(optional_edge_values(base.trim_to_other(Plane(Vector(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)))));
            out.push_back(optional_edge_values(base.trim_to_other(Vector(1, 0).rotate(Axis::Z, 45))));
            out.push_back(optional_edge_values(base.trim_to_other(Vector(h, h))));
            out.push_back(optional_edge_values(base.trim_to_other(Vector(0, 2))));
            out.push_back(optional_edge_values(base.trim_to_other(Vertex(h, h, 0))));
            out.push_back(optional_edge_values(base.trim_to_other(Pos(0, 0.5) * Circle(0.8))));
            out.push_back(optional_edge_values(base.trim_to_other(Line({-2, 0.5}, {2, 0.5}))));
            out.push_back(optional_edge_values(base.reversed().trim_to_other(Line({0, 0}, {1, 1}))));
        }
        return V::list(out);
    });
    compare::add_value("obj_trim_infinite", [] {
        return V::list({edge_values(Edge(Axis::X).trim_infinite(5)),
                        edge_values(Edge(Axis(Vector(1, 2, 3), Vector(0, 1, 1))).trim_infinite(2)),
                        edge_values(Line({0, 0}, {1, 1}).trim_infinite(5)), Edge(Axis::Y).is_infinite()});
    });
    compare::add_value("obj_center_arc_limits", [] {
        const CenterArc c1({1, 0}, 1, 0, Line({1, 0}, {2, 1}));
        const CenterArc c2({1, 0}, 1, 0, Line({1, 0}, {2, -1}));
        const CenterArc c3({0, 0}, 2, 30, Axis::Y);
        const CenterArc c4({0, 0}, 2, 30, Plane::XZ);
        const CenterArc c5({0, 0}, 2, 10, Pos(0, 3) * Circle(1.5));
        const CenterArc c6({0, 0}, 2, 10, Vector(0, -2));
        return V::list({edge_values(c1), c1 % 0, edge_values(c2), c2 % 0, edge_values(c3), edge_values(c4),
                        edge_values(c5), edge_values(c6)});
    });
    compare::add("obj_center_arc_limit_error", [] { return CenterArc({1, 0}, 1, 0, {5, 0}); });
    compare::add_value("obj_conic_arc_limits", [] {
        return V::list({
            edge_values(EllipticalCenterArc({0, 0}, 2, 1, 0, Line({0, 0}, {0, 2}))),
            edge_values(EllipticalCenterArc({0, 0}, 2, 1, 0, Line({0, 0}, {0, -2}))),
            edge_values(EllipticalCenterArc({1, 1}, 3, 1, 0, Axis(Vector(1, 1), Vector(1, 1)), 20)),
            edge_values(ParabolicCenterArc({0, 0}, 0.5, 0, Line({0, 1}, {5, 1}))),
            edge_values(ParabolicCenterArc({0, 0}, 0.5, 0, Line({1, -2}, {1, 2}))),
            edge_values(ParabolicCenterArc({0, 0}, 0.5, -30, Axis::X, 15)),
            edge_values(HyperbolicCenterArc({0, 0}, 2, 1, 0, Line({0, 1}, {10, 1}))),
            edge_values(HyperbolicCenterArc({0, 0}, 2, 1, 0, Line({3, -2}, {3, 2}))),
            edge_values(HyperbolicCenterArc({0, 0}, 1, 2, 0, Line({-5, 3}, {5, 3}), 10)),
        });
    });
    compare::add("obj_elliptical_limit_error", [] { return EllipticalCenterArc({0, 0}, 2, 1, 0, {0, 5}); });
    compare::add("obj_parabolic_limit_error", [] { return ParabolicCenterArc({0, 0}, 0.5, 0, {0, 5}); });
    compare::add("obj_hyperbolic_limit_error", [] { return HyperbolicCenterArc({0, 0}, 2, 1, 0, {0, 5}); });
    compare::add_value("obj_jern_arc_limits", [] {
        const JernArc j1({1, 0}, {0, 1}, 1, Line({1, 0}, {2, 1}));
        const JernArc j2({1, 0}, {0, 1}, 1, Line({1, 0}, {0, 1}));
        const JernArc j3({0, 0}, {1, 1}, 3, Axis::Y);
        const JernArc j4({0, 0, 1}, {0, 0, 1}, 2, Plane::XY.offset(3));
        return V::list({edge_values(j1), j1.center_point, j1.end_of_arc, j1.start, edge_values(j2), j2.center_point,
                        edge_values(j3), j3.center_point, edge_values(j4), j4.center_point});
    });
    compare::add("obj_jern_arc_limit_error", [] { return JernArc({1, 0}, {0, 1}, 1, {5, 0}); });
    compare::add_value("obj_polar_line_limits", [] {
        const double h = std::sqrt(2.0) / 2;
        const Sketch rect = Rectangle(1, 1, Align2(Align::MAX, Align::MIN)).rotate(Axis::Z, -45);
        const std::vector<Limit> limits{
            Line({0, 0}, {1, 1}),
            CenterArc({0, 0}, 1, 45, 90),
            rect.wire(),
            rect,
            Curve(std::vector<Edge>(rect.edges())),
            Box(1, 1, 1, Align3(Align::MAX, Align::MIN, Align::CENTER)).rotate(Axis::Z, -45),
            Axis(Vector(0, 0, 0), Vector(1, 1, 0)),
            Location(Vector(h, h), Vector(0, 0, 1)),
            Plane(Vector(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)),
            Vector(1, 0).rotate(Axis::Z, 45),
            Limit(h, h),
        };
        std::vector<V> out;
        for (const auto& limit : limits) out.push_back(edge_values(PolarLine({std::sqrt(2.0), 0}, limit, 135)));
        out.push_back(edge_values(PolarLine({2, 0}, Axis::Y, 135)));
        out.push_back(edge_values(PolarLine({0, 0}, Line({3, -1}, {3, 5}), Vector(1, 1))));
        return V::list(out);
    });
    compare::add("obj_polar_line_limit_point_error", [] { return PolarLine({std::sqrt(2.0), 0}, {0, 2}, 135); });
    compare::add("obj_polar_line_limit_shape_error", [] {
        return PolarLine({std::sqrt(2.0), 0}, Line({3, 0}, {3, 1}), 135);
    });
    compare::add("obj_polar_line_limit_behind_error", [] { return PolarLine({0, 0}, Axis::Y, 0.0); });
    compare::add("obj_polar_line_limit_location_aligned", [] {
        return PolarLine({0, 0}, Location(Plane(Vector(3, 0), std::nullopt, Vector(1, 0, 0))), 0.0);
    });
}
