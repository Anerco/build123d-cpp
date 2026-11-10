// Port of bd_warehouse's sprocket.py.
#include "bd_warehouse/sprocket.hpp"

#include <algorithm>
#include <cmath>

#include "data.hpp"

namespace bd_warehouse {

using namespace build123d;

namespace {

/// ShapeList.sort_by(axis) of points (Vectors sort by their position along the axis).
std::vector<Vector> sort_points_by(std::vector<Vector> points, const Axis& axis) {
    const Location axis_as_location = axis.location().inverse();
    std::stable_sort(points.begin(), points.end(), [&](const Vector& a, const Vector& b) {
        return (axis_as_location * Location(a)).position().Z < (axis_as_location * Location(b)).position().Z;
    });
    return points;
}

}  // namespace

double Sprocket::sprocket_pitch_radius(int num_teeth, double chain_pitch) {
    return std::sqrt(chain_pitch * chain_pitch / (2 * (1 - std::cos(2 * M_PI / num_teeth))));
}

double Sprocket::sprocket_circumference(int num_teeth, double chain_pitch) {
    return 2 * M_PI * std::sqrt(chain_pitch * chain_pitch / (2 * (1 - std::cos(2 * M_PI / num_teeth))));
}

Sprocket::Sprocket(int num_teeth_, double chain_pitch_, double roller_diameter_, double clearance_, double thickness_,
                   double bolt_circle_diameter_, int num_mount_bolts_, double mount_bolt_diameter_,
                   double bore_diameter_, const Rotation& rotation, std::optional<Align3> align)
    : Sprocket(num_teeth_, SprocketOptions{chain_pitch_, roller_diameter_, clearance_, thickness_, bolt_circle_diameter_,
                                           num_mount_bolts_, mount_bolt_diameter_, bore_diameter_, rotation, align}) {}

Sprocket::Sprocket(int num_teeth_, const SprocketOptions& o) {
    num_teeth = num_teeth_;
    chain_pitch = o.chain_pitch;
    roller_diameter = o.roller_diameter;
    clearance = o.clearance;
    thickness = o.thickness;
    bolt_circle_diameter = o.bolt_circle_diameter;
    num_mount_bolts = o.num_mount_bolts;
    mount_bolt_diameter = o.mount_bolt_diameter;
    bore_diameter = o.bore_diameter;

    // Validate inputs: ensure that the roller would fit in the chain
    if (roller_diameter >= chain_pitch) {
        throw std::invalid_argument("roller_diameter " + detail::py_str(roller_diameter) +
                                    " is too large for chain_pitch " + detail::py_str(chain_pitch));
    }
    if (num_teeth <= 2) {
        throw std::invalid_argument("num_teeth must be an integer greater than 2 not " + std::to_string(num_teeth));
    }
    // Create the sprocket, without an extraneous Compound wrapper
    Shape sprocket = _make_sprocket();
    if (!sprocket.is_null() && sprocket.wrapped().ShapeType() == TopAbs_COMPOUND) {
        sprocket = Compound(sprocket.wrapped()).unwrap(true);
    }
    assign(sprocket, o.rotation, o.align);
}

Shape Sprocket::_make_sprocket() {
    const Wire tooth_tip = _make_tooth_outline(num_teeth, chain_pitch, roller_diameter, clearance);
    std::vector<Edge> outline = tooth_tip.edges();
    outline.push_back(Line(Vector(0, 0), tooth_tip.position_at(0)));
    outline.push_back(Line(Vector(0, 0), tooth_tip.position_at(1)));
    const Face tooth_face = Pos(0, 0, -thickness / 2) * Face(Wire(outline));
    Part tooth = build123d::extrude(tooth_face, thickness, ExtrudeOptions{.dir = Vector(0, 0, 1)});

    // Chamfer the outside edges if the sprocket has "flat" teeth
    flat_teeth_ = tooth_tip.edges().size() == 5;
    if (flat_teeth_) {
        const Face tip_face = tooth.faces().filter_by(GeomType::CYLINDER).sort_by_distance(Vector(0, 0))[-1];
        const ShapeList<Edge> to_chamfer = tip_face.edges().filter_by(GeomType::CIRCLE);
        tooth = build123d::chamfer(to_chamfer, thickness * 0.25, thickness * 0.5, std::nullopt, tip_face);
    }
    Part sprocket = Solid() + (PolarLocations(0, num_teeth) * tooth);
    sprocket.set_orientation(sprocket.orientation() + Vector(0, 0, 90));

    // Create bolt holes
    if (bolt_circle_diameter != 0 && num_mount_bolts != 0 && mount_bolt_diameter != 0) {
        sprocket -= PolarLocations(bolt_circle_diameter / 2, num_mount_bolts) *
                    Cylinder(mount_bolt_diameter / 2, thickness);
    }
    // Create a central bore
    if (bore_diameter != 0) sprocket -= Cylinder(bore_diameter / 2, thickness);
    return sprocket;
}

double Sprocket::pitch_radius() const { return sprocket_pitch_radius(num_teeth, chain_pitch); }

double Sprocket::outer_radius() const {
    if (flat_teeth_) return pitch_radius() + roller_diameter / 4;
    return std::sqrt(std::pow(pitch_radius(), 2) - std::pow(chain_pitch / 2, 2)) +
           std::sqrt(std::pow(chain_pitch - roller_diameter / 2, 2) - std::pow(chain_pitch / 2, 2));
}

double Sprocket::pitch_circumference() const { return sprocket_circumference(num_teeth, chain_pitch); }

Face Sprocket::plan() const {
    const Wire tooth_tip = _make_tooth_outline(num_teeth, chain_pitch, roller_diameter, clearance);
    std::vector<Edge> outline = tooth_tip.edges();
    outline.push_back(Line(Vector(0, 0), tooth_tip.position_at(0)));
    outline.push_back(Line(Vector(0, 0), tooth_tip.position_at(1)));
    Face tooth_face{Wire(outline)};
    if (tooth_face.normal_at().Z == -1) tooth_face = -tooth_face;
    const Sketch sprocket_plan = Sketch() + (PolarLocations(0, num_teeth) * tooth_face);
    return sprocket_plan.face();
}

Wire Sprocket::_make_tooth_outline(int num_teeth, double chain_pitch, double roller_diameter, double clearance) {
    const double roller_rad = roller_diameter / 2 + clearance;
    const double tooth_a_degrees = 360.0 / num_teeth;
    const double pitch_rad = std::sqrt(std::pow(chain_pitch, 2) / (2 * (1 - std::cos(detail::radians(tooth_a_degrees)))));
    const double outer_rad = pitch_rad + roller_rad / 2;

    const CenterArc outer_circle(Vector(0, 0), outer_rad, 0, 360);
    const CenterArc roller_circle(Vector(pitch_rad, 0).rotate(Axis::Z, tooth_a_degrees / 2), roller_rad, 0, 360);
    const CenterArc link_circle(Vector(pitch_rad, 0).rotate(Axis::Z, -tooth_a_degrees / 2), chain_pitch - roller_rad, 0,
                                360);

    const Vector roller_line_pnt =
        Line(roller_circle.arc_center(), link_circle.arc_center()).position_at(roller_rad / chain_pitch);
    const std::vector<Vector> crossings = link_circle.find_intersection_points(Edge(outer_circle));
    if (crossings.empty()) throw std::out_of_range("list index out of range");
    const Vector outer_pnt = sort_points_by(crossings, Axis::Y).back();
    const Vector roller_start_pnt = PolarLine(Vector(0, 0), pitch_rad - roller_rad, tooth_a_degrees / 2).position_at(1);
    const Edge arc1 = roller_circle.trim(roller_start_pnt, roller_line_pnt);
    if (outer_pnt.Y > 0) {  // "Flat" topped sprockets
        const Edge arc2 = link_circle.trim(roller_line_pnt, outer_pnt);
        const RadiusArc arc3(outer_pnt, Vector(outer_pnt.X, -outer_pnt.Y), outer_rad);
        const Edge arc4 = arc2.mirror(Plane::XZ);
        const Edge arc5 = arc1.mirror(Plane::XZ);
        return Wire(std::vector<Edge>{arc1, arc2, arc3, arc4, arc5});
    }
    const ShapeList<Shape> axis_crossings = link_circle.intersect(Axis::X);
    if (axis_crossings.empty()) throw std::runtime_error("link circle does not cross the X axis");
    const Vector link_axis_pnt = Vector(Vertex(axis_crossings.sort_by(Axis::X)[-1].wrapped()));
    const Edge arc2 = link_circle.trim(roller_line_pnt, link_axis_pnt);
    const Edge arc3 = arc2.mirror(Plane::XZ);
    const Edge arc4 = arc1.mirror(Plane::XZ);
    return Wire(std::vector<Edge>{arc1, arc2, arc3, arc4});
}

}  // namespace bd_warehouse
