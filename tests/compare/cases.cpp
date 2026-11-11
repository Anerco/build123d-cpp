// Models built with build123d-cpp; tests/compare/cases_py.py builds the same models with
// Python build123d and compare.py checks that the results match.
#include <build123d/build123d.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>

#include "case_registry.hpp"
#include "gate.hpp"
#include "show.hpp"

namespace {
std::string occt_failure_text(const Standard_Failure& e) {
#if OCC_VERSION_HEX >= 0x080000
    return e.what();
#else
    const char* message = e.GetMessageString();
    return std::string(e.DynamicType()->Name()) + ": " + (message != nullptr ? message : "");
#endif
}
}  // namespace

using namespace build123d;

namespace {

struct Case {
    std::string name;
    std::function<Shape()> build;
};

std::string escape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        if (c == '\n') {
            out += "\\n";
            continue;
        }
        out += c;
    }
    return out;
}

Part plate_with_holes() { return Box(30, 30, 5) - std::vector<Part>{Pos(-10, -10) * Cylinder(2, 5), Pos(10, -10) * Cylinder(2, 5), Pos(0, 10) * Cylinder(2, 5)}; }

std::vector<Case> make_cases() {
    std::vector<Case> c;
    auto add = [&](const std::string& name, std::function<Shape()> f) { c.push_back({name, std::move(f)}); };

    // ---- primitives
    add("box", [] { return Box(10, 20, 30); });
    add("box_align_min", [] { return Box(10, 20, 30, Align::MIN); });
    add("box_align_mixed", [] { return Box(10, 20, 30, {Align::MIN, Align::CENTER, Align::MAX}); });
    add("box_rot", [] { return Box(10, 20, 30, {10, 20, 30}); });
    add("cylinder", [] { return Cylinder(5, 10); });
    add("cylinder_arc", [] { return Cylinder(5, 10, 270, Rotation(), Align::MIN); });
    add("cone", [] { return Cone(5, 2, 10); });
    add("sphere", [] { return Sphere(7); });
    add("sphere_partial", [] { return Sphere(7, -45, 60, 200); });
    add("torus", [] { return Torus(10, 2); });
    add("wedge", [] { return Wedge(10, 5, 8, 2, 2, 6, 6); });
    add("convex_polyhedron", [] {
        return ConvexPolyhedron({{0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {0, 0, 10}, {3, 3, 3}, {10, 10, 10}});
    });

    // ---- booleans
    add("bool_sub", [] { return Box(10, 10, 10) - Cylinder(3, 20); });
    add("bool_add", [] { return Box(10, 10, 10) + Pos(5, 5, 5) * Sphere(4); });
    add("bool_and", [] { return Box(10, 10, 10) & Sphere(7); });
    add("bool_list", [] { return plate_with_holes(); });
    add("sketch_bool", [] { return Rectangle(20, 10) - Circle(3) + Pos(10, 0) * Circle(5); });
    add("sketch_and", [] { return Rectangle(20, 10) & Circle(7); });
    add("sketch_minus_part", [] { return Rectangle(20, 10) - Pos(5, 0) * Box(4, 4, 4); });
    add("bool_iadd", [] {
        Part p;
        for (int i = 0; i < 4; ++i) p += Pos(i * 5.0, 0, 0) * Box(4, 4, 4);
        return p;
    });

    // ---- placement
    add("plane_mul", [] { return Plane::XZ * Pos(5, 3) * Rot(0, 0, 30) * Box(4, 2, 1); });
    add("plane_offset", [] { return Plane::XY.offset(5) * Rectangle(3, 2); });
    add("plane_rotated", [] { return Plane::XY.rotated({30, 0, 45}) * Pos(2, 0) * Box(1, 2, 3); });
    add("location_inverse", [] { return (Pos(1, 2, 3) * Rot(10, 20, 30)).inverse() * Box(2, 3, 4); });
    add("grid_locs", [] { return Box(50, 50, 5) - GridLocations(10, 10, 4, 4) * Cylinder(2, 5); });
    add("grid_locs_min", [] { return Sketch() + GridLocations(5, 7, 3, 2, Align::MIN) * Rectangle(2, 2); });
    add("polar_locs", [] { return Cylinder(20, 5) - PolarLocations(15, 8) * Box(4, 2, 5); });
    add("polar_locs_norot", [] { return Sketch() + PolarLocations(10, 5, 15, 180, false, true) * Rectangle(3, 1); });
    add("hex_locs", [] { return Rectangle(60, 60) - HexLocations(5, 4, 4) * RegularPolygon(4, 6); });
    add("locations_list", [] {
        return Sketch() + Locations({Pos(0, 0), Pos(10, 5), Pos(-10, 5) * Rot(0, 0, 45)}) * Rectangle(3, 3);
    });

    // ---- sketch objects
    add("circle", [] { return Circle(5); });
    add("circle_sector", [] { return Circle(5, 90); });
    add("ellipse", [] { return Ellipse(10, 5, 30); });
    add("rect_rounded", [] { return RectangleRounded(20, 10, 2); });
    add("polygon", [] { return Polygon({{0, 0}, {10, 0}, {12, 5}, {3, 8}}, Align::CENTER); });
    add("polygon_cw", [] { return Polygon({{0, 0}, {3, 8}, {12, 5}, {10, 0}}); });
    add("regular_polygon", [] { return RegularPolygon(10, 7, true, 10); });
    add("regular_polygon_minor", [] { return RegularPolygon(10, 6, false, 0, {Align::MIN, Align::MIN}); });
    add("trapezoid", [] { return Trapezoid(20, 8, 60, 75); });
    add("triangle_sss", [] { return Triangle({.a = 10, .b = 8, .c = 7}); });
    add("triangle_asa", [] { return Triangle({.a = 10, .B = 50, .C = 60}); });
    add("slot_overall", [] { return SlotOverall(20, 6, 15); });
    add("slot_c2c", [] { return SlotCenterToCenter(15, 4); });
    add("slot_cp", [] { return SlotCenterPoint({0, 0}, {5, 3}, 3); });
    add("slot_arc", [] { return SlotArc(CenterArc({0, 0}, 10, 0, 90), 3); });
    add("superellipse", [] { return Superellipse(20, 10, 4); });

    // ---- part operations
    add("extrude", [] { return extrude(Rectangle(10, 5), 3); });
    add("extrude_neg", [] { return extrude(Rectangle(10, 5), -3); });
    add("extrude_both", [] { return extrude(Circle(4) - Circle(2), 5, {.both = true}); });
    add("extrude_taper", [] { return extrude(Rectangle(10, 10), 5, {.taper = 10}); });
    add("extrude_taper_hole", [] { return extrude(Rectangle(10, 10) - Circle(2), 5, {.taper = 5}); });
    add("extrude_dir", [] { return extrude(Rectangle(10, 5), 4, {.dir = Vector(1, 0, 1)}); });
    add("extrude_plane", [] { return extrude(Plane::XZ * Circle(3), 10); });
    add("extrude_until", [] {
        const Part base = Pos(0, 0, -1) * Box(20, 20, 2) + Pos(0, 0, 10) * Rot(20, 0, 0) * Box(30, 30, 2);
        return extrude(Circle(3), Until::NEXT, base);
    });
    add("extrude_until_last", [] {
        const Part base = Pos(0, 0, 5) * Box(20, 20, 2) + Pos(0, 0, 12) * Box(20, 20, 2);
        return extrude(Circle(3), Until::LAST, base);
    });
    add("revolve", [] { return revolve(Pos(10, 0) * Rectangle(4, 6), Axis::Y, 270); });
    add("revolve_full", [] {
        return revolve(Plane::XZ * Polygon({{5, 0}, {10, 0}, {10, 3}, {6, 8}}), Axis::Z);
    });
    add("loft", [] {
        return loft(std::vector<Shape>{Rectangle(10, 10), Pos(0, 0, 10) * Circle(4), Pos(0, 0, 20) * Rectangle(4, 8)});
    });
    add("loft_ruled", [] {
        return loft(std::vector<Shape>{Rectangle(10, 10), Pos(0, 0, 10) * Circle(4), Pos(0, 0, 20) * Rectangle(4, 8)},
                    true);
    });
    add("loft_holes", [] {
        return loft(std::vector<Shape>{Rectangle(20, 20) - Circle(3), Pos(0, 0, 10) * (Rectangle(10, 10) - Circle(2))});
    });
    add("loft_vertex", [] { return loft(std::vector<Shape>{Circle(5), Vertex(0, 0, 10)}); });
    add("loft_sketch", [] {
        Sketch art;
        for (int i = 0; i <= 10; ++i) art += Plane(Vector(0, 0, i * 3.0)) * Circle(10 * std::sin(i * PI / 10) + 5);
        return loft(art);
    });
    add("find_intersection_points", [] {
        const Part p = Sphere(10) - Box(4, 4, 30);
        std::vector<Edge> normals;
        double scale = 1.0;
        for (const auto& [point, normal] : p.find_intersection_points(Axis({-20, 1, 0.5}, {1, 0.05, 0.02}))) {
            normals.push_back(Edge::make_line(point, point + normal * scale));
            scale += 1.0;
        }
        return Compound(normals);
    });
    add("project_to_shape", [] {
        const Sketch square = Plane::ZX.offset(-80) * Rectangle(20, 20);
        return Compound(square.face().project_to_shape(Sphere(50), Vector(0, 1, 0)));
    });
    add("project_to_shape_edge", [] {
        const Sketch square = Plane::YZ.offset(-50) * Rectangle(10, 10);
        return square.face().project_to_shape(Rot(0, 0, 45) * Box(20, 20, 20), Vector(1, 0, 0))[0];
    });
    add("project_faces", [] {
        const Part sphere = Sphere(50);
        const Part obj = sphere - Pos(-50, 0, -70) * (Plane::YZ * Cylinder(80, 100, {Align::CENTER, Align::CENTER, Align::MIN}));
        const Edge arch_path = obj.edges().sort_by().first();
        const Text text("abc", 15, {.align = Align2(Align::MIN, Align::CENTER)});
        return Sketch(sphere.project_faces(text.faces(), arch_path));
    });
    add("surface_from_points", [] {
        std::vector<std::vector<Vector>> pts;
        for (int j = 0; j < 6; ++j) {
            pts.emplace_back();
            for (int i = 0; i < 7; ++i) pts.back().emplace_back(i * 2.0, j * 2.0, std::sin(i * 0.7) * std::cos(j * 0.5) * 3);
        }
        return Face::make_surface_from_array_of_points(pts);
    });
    add("surface_from_points_smooth", [] {
        std::vector<std::vector<Vector>> pts;
        for (int j = 0; j < 6; ++j) {
            pts.emplace_back();
            for (int i = 0; i < 7; ++i) pts.back().emplace_back(i * 2.0, j * 2.0, std::sin(i * 0.7) * std::cos(j * 0.5) * 3);
        }
        return Face::make_surface_from_array_of_points(pts, 1e-2, std::array<double, 3>{1.0, 0.5, 0.1});
    });
    add("sweep", [] {
        const JernArc path({0, 0, 0}, {0, 0, 1}, 20, 90);
        return sweep(Plane(path.position_at(0), std::nullopt, path.tangent_at(0.0)) * Circle(2), path);
    });
    add("sweep_helix", [] {
        const Helix helix(5, 20, 10);
        const Sketch profile = Plane(helix.position_at(0), std::nullopt, helix.tangent_at(0.0)) * Circle(1);
        return sweep(profile, helix, {.is_frenet = true});
    });
    add("sweep_multisection", [] {
        const Line path({0, 0, 0}, {0, 0, 20});
        return sweep(std::vector<Sketch>{Rectangle(6, 6), Pos(0, 0, 20) * Circle(2)}, path, {.multisection = true});
    });
    add("sweep_edges", [] { return sweep(Line({0, 0, 0}, {5, 0, 0}), Line({0, 0, 0}, {0, 0, 7})); });
    add("thicken", [] { return thicken(Rectangle(10, 10), 2); });
    add("thicken_both", [] { return thicken(Circle(5), 1, std::nullopt, true); });
    add("section", [] { return section(Sphere(10) - Box(4, 4, 30), Plane::XZ); });
    add("section_height", [] { return section(Box(10, 10, 10) - Cylinder(3, 10), Plane::XY, 2); });

    // ---- fillet / chamfer
    add("fillet3d", [] {
        const Part p = Box(10, 10, 10);
        return fillet(p.edges().filter_by(Axis::Z), 2);
    });
    add("fillet3d_top", [] {
        const Part p = Box(10, 10, 10) - Cylinder(3, 10);
        return fillet(p.edges().group_by(Axis::Z)[-1].filter_by(GeomType::LINE), 1);
    });
    add("fillet3d_hole", [] {
        const Part p = Box(10, 10, 10) - Cylinder(3, 10);
        return fillet(p.edges().group_by(Axis::Z)[-1] | GeomType::CIRCLE, 1);
    });
    add("chamfer3d", [] { return chamfer(Box(10, 10, 10).edges(), 1); });
    add("chamfer3d_angle", [] {
        const Part p = Box(10, 10, 10);
        return chamfer(p.edges().group_by(Axis::Z)[-1], 2, std::nullopt, 30);
    });
    add("chamfer3d_asym", [] {
        const Part p = Box(10, 10, 10);
        return chamfer(p.edges().group_by(Axis::Z)[-1], 1, 2);
    });
    add("fillet2d", [] {
        const Sketch s = Rectangle(10, 6);
        return fillet(s.vertices(), 1);
    });
    add("fillet2d_some", [] {
        const Sketch s = Rectangle(10, 6) - Circle(1);
        return fillet(s.vertices().group_by(Axis::X)[-1], 2);
    });
    add("chamfer2d", [] { return chamfer(Rectangle(10, 6).vertices(), 1); });
    add("fillet1d", [] {
        const Polyline line({{0, 0}, {10, 0}, {10, 10}, {20, 10}});
        return fillet(line.vertices(), 2);
    });

    // ---- generic operations
    add("offset2d", [] { return offset(Rectangle(10, 6), 2); });
    add("offset2d_inter", [] { return offset(Rectangle(10, 6) - Circle(1), 1, {.kind = Kind::INTERSECTION}); });
    add("offset3d_hollow", [] {
        const Part b = Box(20, 20, 10);
        return offset(b, -1, {.openings = {b.faces().sort_by(Axis::Z)[-1]}});
    });
    add("offset3d_grow", [] { return offset(Box(10, 10, 10), 1); });
    add("offset_curve", [] { return offset(Polyline({{0, 0}, {10, 0}, {10, 10}}), 1); });
    add("offset_curve_side", [] { return offset(Line({0, 0}, {10, 0}), 1, {.side = Side::LEFT}); });
    add("mirror", [] {
        const Part p = Box(10, 5, 5, Align::MIN);
        return p + mirror(p, Plane::YZ);
    });
    add("mirror_sketch", [] { return mirror(Pos(5, 3) * Circle(2), Plane::XZ); });
    add("split", [] { return split(Sphere(10), Plane::XY); });
    add("split_both", [] { return split(Box(10, 10, 10), Plane::XZ.rotated({0, 0, 30}), Keep::BOTH); });
    add("split_bottom", [] { return split(Box(10, 10, 10), Plane::YZ.offset(2), Keep::BOTTOM); });
    add("split_face", [] { return split(Box(10, 10, 10), Pos(0, 0, 2) * Sphere(6).face(), Keep::TOP); });
    add("scale", [] { return scale(Box(2, 2, 2), 3); });
    add("scale_nonuniform", [] { return scale(Sphere(2), Vector(1, 2, 3)); });
    add("bounding_box_op", [] { return bounding_box(Pos(1, 2, 3) * Sphere(4)); });
    add("bounding_box_2d", [] { return bounding_box(Pos(1, 2) * Circle(4)); });

    // ---- sketch operations
    add("make_face", [] {
        return make_face(std::vector<Edge>{Line({0, 0}, {10, 0}), Line({10, 0}, {5, 8}), Line({5, 8}, {0, 0})});
    });
    add("make_face_curve", [] {
        const Curve c = Line({0, 0}, {10, 0}) + ThreePointArc({10, 0}, {5, 5}, {0, 0});
        return make_face(c);
    });
    add("make_hull", [] { return make_hull((Circle(3) + Pos(10, 0) * Circle(2)).edges()); });
    add("trace", [] { return trace(Polyline({{0, 0}, {10, 0}, {10, 10}}), 1); });

    // ---- curves
    add("line_add", [] { return Line({0, 0}, {10, 0}) + Line({10, 0}, {10, 10}); });
    add("polyline_closed", [] { return Polyline({{0, 0}, {10, 0}, {5, 5}}, true); });
    add("spline", [] { return Spline({{0, 0}, {5, 5}, {10, 0}, {15, 5}}); });
    add("spline_tangents", [] { return Spline({{0, 0}, {5, 5}, {10, 0}}, {{0, 1}, {0, -1}}); });
    add("spline_periodic", [] { return Spline({{0, 0}, {5, 5}, {10, 0}, {5, -5}}, {}, {}, true); });
    add("bezier", [] { return Bezier({{0, 0}, {5, 10}, {10, -10}, {15, 0}}); });
    add("bezier_rational", [] { return Bezier({{0, 0}, {5, 10}, {10, 0}}, {1, 2, 1}); });
    add("bspline", [] {
        return BSpline({{0, 0}, {2, 4}, {5, 5}, {8, -2}, {10, 0}}, {0, 0, 0, 0, 0.5, 1, 1, 1, 1}, 3);
    });
    add("bspline_rational", [] {
        return BSpline({{0, 0}, {5, 8}, {10, 0}}, {0, 0, 0, 1, 1, 1}, 2, {1, 3, 1});
    });
    add("elliptical_start_arc", [] { return EllipticalStartArc({1, 2}, {1, 1}, 8, 4, 150, 30.0); });
    add("elliptical_start_arc_axis", [] {
        return EllipticalStartArc({1, 2}, {0, 1}, 8, 4, -120, std::nullopt, Vector(1, 1));
    });
    add("parabolic_arc", [] { return ParabolicCenterArc({1, 2}, 3, -40, 100, 20); });
    add("hyperbolic_arc", [] { return HyperbolicCenterArc({1, 2}, 4, 3, -30, 70, 10); });
    add("hyperbolic_arc_tall", [] { return HyperbolicCenterArc({0, 0}, 2, 5, 10, -60); });
    add("three_point_arc", [] { return ThreePointArc({0, 0}, {5, 5}, {10, 0}); });
    add("center_arc", [] { return CenterArc({0, 0}, 10, 30, -120); });
    add("center_arc_ccw", [] { return CenterArc({1, 2}, 10, 30, 200); });
    add("radius_arc", [] { return RadiusArc({0, 0}, {10, 0}, 8); });
    add("radius_arc_neg", [] { return RadiusArc({0, 0}, {10, 0}, -8, false); });
    add("sagitta_arc", [] { return SagittaArc({0, 0}, {10, 0}, 3); });
    add("tangent_arc", [] { return TangentArc({0, 0}, {10, 0}, {0, 1}); });
    add("jern_arc", [] { return JernArc({0, 0}, {1, 1}, 5, 120); });
    add("elliptical_arc", [] { return EllipticalCenterArc({0, 0}, 10, 5, 0, 200, 15); });
    add("helix", [] { return Helix(2, 10, 5); });
    add("helix_cone", [] { return Helix(2, 10, 5, {0, 0, 0}, {0, 0, 1}, 10); });
    add("helix_left", [] { return Helix(3, 9, 4, {1, 2, 3}, {0, 1, 1}, 0, true); });
    add("polar_line", [] { return PolarLine({0, 0}, 10, 30); });
    add("polar_line_dir", [] { return PolarLine({1, 1}, 5, Vector(1, 2), LengthMode::HORIZONTAL); });
    add("fillet_polyline", [] { return FilletPolyline({{0, 0}, {10, 0}, {10, 10}, {20, 10}}, 2); });
    add("fillet_polyline_closed", [] { return FilletPolyline({{0, 0}, {10, 0}, {10, 10}, {0, 10}}, 2, true); });
    add("intersecting_line", [] { return IntersectingLine({0, 0}, {1, 1}, Line({10, 0}, {0, 10})); });

    // ---- selectors
    add("sel_sort", [] { return (Box(10, 20, 30).faces() > Axis::Z)[-1]; });
    add("sel_group_op", [] { return Compound(Box(10, 20, 30).edges() >> Axis::Z); });
    add("sel_group_min", [] { return Compound((Box(10, 20, 30) - Cylinder(2, 30)).edges() << Axis::Z); });
    add("sel_filter_axis", [] { return Compound(Box(10, 20, 30).edges() | Axis::Y); });
    add("sel_filter_geomtype", [] {
        return Compound((Box(10, 10, 10) - Cylinder(3, 10)).faces() | GeomType::CYLINDER);
    });
    add("sel_filter_position", [] {
        const Part p = Box(10, 10, 10, Align::MIN) + Pos(0, 0, 10) * Box(5, 5, 5, Align::MIN);
        return Compound(p.faces().filter_by_position(Axis::Z, 0, 10, {true, false}));
    });
    add("sel_sort_area", [] { return (Box(10, 10, 10) - Pos(5, 5) * Cylinder(3, 10)).faces().sort_by(SortBy::AREA)[-1]; });
    add("sel_lambda", [] {
        return Compound(Box(4, 8, 12).edges().filter_by([](const Edge& e) { return e.length() > 5; }));
    });
    add("sel_plane", [] { return Compound(Box(4, 8, 12).faces().filter_by(Plane::XZ)); });
    add("sel_group_area", [] { return Compound(Box(4, 8, 12).faces().group_by(SortBy::AREA)[-1]); });
    add("sel_sort_reverse", [] { return (Box(10, 20, 30).faces() < Axis::X)[0]; });
    add("sel_set_ops", [] {
        const Part p = Box(10, 10, 10);
        return Compound((p.edges() - (p.edges() | Axis::Z)) + p.faces().sort_by(Axis::Z)[0].edges());
    });

    // ---- curve positions
    add("location_at", [] {
        const Spline path({{0, 0, 0}, {5, 5, 2}, {10, 0, 4}, {15, 5, 0}});
        return (path ^ 0.3) * Box(1, 2, 3);
    });
    add("location_at_wire", [] {
        const Polyline path({{0, 0, 0}, {10, 0, 0}, {10, 10, 5}});
        return path.location_at(0.75) * Box(1, 2, 3);
    });
    add("position_at", [] {
        const Spline path({{0, 0, 0}, {5, 5, 2}, {10, 0, 4}});
        return Pos(path.position_at(0.6)) * Sphere(1);
    });
    add("tangent_at", [] {
        const ThreePointArc arc({0, 0}, {5, 5}, {10, 0});
        return Plane(arc.position_at(0.25), std::nullopt, arc % 0.25) * Circle(1);
    });

    // ---- text
    add("text", [] { return Text("Hello", 10); });
    add("text_aligned", [] {
        return Text("b123d", 5, {.font_style = FontStyle::BOLD, .align = Align2(Align::MIN, Align::MIN), .rotation = 30});
    });
    return c;
}

}  // namespace

int main(int argc, char** argv) {
    // The cases of this file, then those registered by the cases_<area>.cpp files.
    std::vector<compare::Entry> entries;
    for (auto& c : make_cases()) entries.push_back({c.name, c.build, {}});
    for (const auto& e : compare::registry()) entries.push_back(e);

    std::set<std::string> seen;
    for (const auto& e : entries) {
        if (!seen.insert(e.name).second) {
            std::cout << "{\"name\": \"" << e.name << "\", \"error\": \"duplicate case name\"}" << std::endl;
        }
    }
    const std::set<std::string> only(argv + 1, argv + argc);
    for (const auto& e : entries) {
        if (!only.empty() && only.count(e.name) == 0) continue;
        try {
            if (e.shape) {
                const Shape shape = e.shape();
                // the operation cache's gate compares the shape's bytes too (tests/gate), taken
                // before show() measures it
                const std::string brep = gate::brep_hash(shape);
                example::show(shape, e.name);
                if (!brep.empty()) std::cout << "{\"name\": " << compare::V::quote(e.name + " brep") << ", \"brep\": \"" << brep << "\"}" << std::endl;
            } else {
                const std::string values = e.value().dump();
                std::cout << "{\"name\": " << compare::V::quote(e.name) << ", \"values\": " << values << "}"
                          << std::endl;
            }
        } catch (const Standard_Failure& ex) {
            // Before OCCT 8, Standard_Failure doesn't derive from std::exception.
            std::cout << "{\"name\": \"" << e.name << "\", \"error\": \"" << escape(occt_failure_text(ex)) << "\"}"
                      << std::endl;
        } catch (const std::exception& ex) {
            std::cout << "{\"name\": \"" << e.name << "\", \"error\": \"" << escape(ex.what()) << "\"}" << std::endl;
        }
    }
    return 0;
}
