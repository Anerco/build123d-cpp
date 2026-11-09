// The variants of extrude: simple, both directions, many faces at once, a non-planar
// face along a direction, and up to the last/next surface of a target.
// Port of build123d's examples/extrude_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    // Extrude a simple shape
    const Part simple = extrude(Text("O", 10), 5);

    // Extrude in both directions
    const Part both = extrude(Text("O", 10), 5, {.both = true});

    // Extrude many faces at once
    Part multiple = Box(10, 10, 10);
    std::vector<Sketch> faces;
    for (const auto& face : multiple.faces()) {
        for (const auto& loc : GridLocations(5, 5, 2, 2)) faces.push_back(Plane(face) * loc * Text("Ω", 3));
    }
    std::vector<Part> extrusions;
    for (const auto& face : faces) extrusions.push_back(extrude(face, 1));
    multiple += extrusions;

    // Extrude a non-planar face in a given direction
    Part non_planar = Rot(90, 0, 0) * Cylinder(10, 20, {Align::CENTER, Align::MIN, Align::CENTER});
    non_planar &= Box(10, 10, 10, {Align::CENTER, Align::CENTER, Align::MIN});
    non_planar = extrude(non_planar.faces().sort_by(Axis::Z).first(), 2, {.dir = Vector(0, 0, 1)});

    // Extrude until the last surface of a target
    const double rad = 3, rev = 25;
    const Sketch circle = Pos(0, rev) * Circle(rad);
    const Part ex26_target = revolve(circle, Axis::X, 180);
    const Sketch rect = Rectangle(rad, rev);
    const Part ex26 = extrude(rect, Until::LAST, ex26_target, {.clean = false});

    // Extrude until the next surface of a target
    Part ex27 = revolve(circle, Axis::X, 90);
    const Sketch circle2 = Plane::XZ * Pos(0, rev) * Circle(rad);
    ex27 += revolve(circle2, Axis::X, 150);
    const Sketch rect27 = Plane::XY.offset(-60) * Rectangle(rad, rev + 25);
    const Part extrusion27 = extrude(rect27, Until::NEXT, ex27);

    example::show(simple, "extrude_simple");
    example::show(both, "extrude_both");
    example::show(multiple, "extrude_multiple");
    example::show(non_planar, "extrude_non_planar");
    example::show(ex26_target, "extrude_until_last_target");
    example::show(ex26, "extrude_until_last");
    example::show(ex27, "extrude_until_next_target");
    example::show(extrusion27, "extrude_until_next");
    export_step(multiple, "extrude_multiple.step");
    return 0;
}
