// Parts positioned by rigid, revolute, linear, cylindrical and ball joints.
// Port of build123d's examples/joints_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

namespace {

// Built in a free function: inside a class derived from Part, Part's member functions
// (fillet, ...) would hide the free operations of the same name.
Part joint_box(double length, double width, double height, double radius, double taper) {
    Part obj = extrude(Rectangle(length, width), height, {.taper = taper});
    if (radius != 0.0) obj = fillet(obj.edges(), radius);
    obj -= Rot(0, 90, 0) * Cylinder(width / 4, length);
    return obj;
}

}  // namespace

/// A filleted box with joints: a box of the given dimensions with all of the edges filleted.
class JointBox : public Part {
public:
    JointBox(double length, double width, double height, double radius = 0.0, double taper = 0.0)
        : Part(joint_box(length, width, height, radius, taper)) {}
};

int main() {
    // Base Object
    const Location loc(Vector(1, 1, 1), Vector(1, 1, 1), 30);
    Part base = loc * JointBox(10, 10, 10, 0.0, 3);

    const auto base_top_edges = base.edges().filter_by(loc.x_axis()).group_by(loc.z_axis())[-1];

    // Rigid Joint
    Part fixed_arm = JointBox(1, 1, 5, 0.2);
    RigidJoint j1("side", base, Plane(base.faces().sort_by(loc.x_axis()).last()).location());
    RigidJoint j2("top", fixed_arm, (-Plane(fixed_arm.faces().sort_by().last())).location());
    base.joints["side"].connect_to(fixed_arm.joints["top"]);
    // or
    // j1.connect_to(j2);

    // Hinge
    Part hinge_arm = JointBox(2, 1, 10, 0.0, 1);
    const Edge swing_arm_hinge_edge =
        hinge_arm.edges().group_by(SortBy::LENGTH)[-1].sort_by(Axis::X).slice(-2).sort_by(Axis::Y)[0];
    const Axis swing_arm_hinge_axis(swing_arm_hinge_edge);
    const Edge base_corner_edge = base.edges().sort_by(Axis({0, 0, 0}, {1, 1, 0}))[-1];
    const Axis base_hinge_axis(base_corner_edge);
    RevoluteJoint j3("hinge", base, base_hinge_axis, std::nullopt, {0, 180});
    RigidJoint j4("corner", hinge_arm, swing_arm_hinge_axis.location());
    base.joints["hinge"].connect_to(hinge_arm.joints["corner"], {.angle = 90});

    // Slider
    Part slider_arm = JointBox(4, 1, 2, 0.2);
    LinearJoint s1("slide", base, Axis(Edge::make_mid_way(base_top_edges[0], base_top_edges[1], 0.67)),
                   {0, base_top_edges[0].length()});
    RigidJoint s2("slide", slider_arm, Location(Vector(0, 0, 0)));
    base.joints["slide"].connect_to(slider_arm.joints["slide"], {.position = 8});
    // s1.connect_to(s2, 8);

    // Cylindrical
    const Face hole_face = base.faces().sort_by(Axis::Y)[0];
    const Axis hole_axis(hole_face.center(), -hole_face.normal_at());
    Part screw_arm = JointBox(1, 1, 10, 0.49);
    CylindricalJoint j5("hole", base, hole_axis, std::nullopt, {-10, 10});
    RigidJoint j6("screw", screw_arm, screw_arm.faces().sort_by(Axis::Z)[-1].location());
    j5.connect_to(j6, -1, 90);

    // PinSlotJoint
    LinearJoint j7("slot", base, Axis(Edge::make_mid_way(base_top_edges[0], base_top_edges[1], 0.33)),
                   {0, base_top_edges[0].length()});
    Part pin_arm = JointBox(2, 1, 2);
    RevoluteJoint j8("pin", pin_arm, Axis::Z, std::nullopt, {0, 360});
    j7.connect_to(j8, 6, 60);

    // BallJoint
    BallJoint j9("socket", base, Plane(base.faces().sort_by(Axis::X)[0]).location());
    Part ball = JointBox(2, 2, 2, 0.99);
    RigidJoint j10("ball", ball, Location(Vector(0, 0, 1)));
    j9.connect_to(j10, Rotation(10, 20, 30));

    example::show(base, "joints_base");
    example::show(fixed_arm, "joints_fixed_arm");
    example::show(hinge_arm, "joints_hinge_arm");
    example::show(slider_arm, "joints_slider_arm");
    example::show(screw_arm, "joints_screw_arm");
    example::show(pin_arm, "joints_pin_arm");
    example::show(ball, "joints_ball");
    export_step(Compound::make_assembly({base, fixed_arm, hinge_arm, slider_arm, screw_arm, pin_arm, ball}, "joints"),
                "joints.step");
    return 0;
}
