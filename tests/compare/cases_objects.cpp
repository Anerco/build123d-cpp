// Objects, operations, text and drafting compared with cases_objects_py.py.
#include "case_registry.hpp"

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
}
