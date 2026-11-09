// Keyboard key cap: tapered, dished, shelled, with ribs and a stem socket.
// Port of build123d's examples/key_cap_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    // Taper Extrude and Extrude to "next" while creating a Cherry MX key cap
    // See: https://www.cherrymx.de/en/dev.html

    const Sketch plan = Rectangle(18 * MM, 18 * MM);
    Part key_cap = extrude(plan, 10 * MM, {.taper = 15});

    // Create a dish in the top of the key
    key_cap -= Location({0, -3 * MM, 47 * MM}, {90, 0, 0}) * Sphere(40 * MM);

    // Fillet all the edges except the bottom
    key_cap = fillet(key_cap.edges().filter_by_position(Axis::Z, 0, 30 * MM, {false, true}), 1 * MM);

    // Hollow out the key by subtracting a scaled version
    key_cap -= scale(key_cap, Vector(0.925, 0.925, 0.85));

    // Determine the internal size of the key cap
    const Face key_cap_section = section(key_cap, Plane::XY.offset(4 * MM)).face();
    const Vector key_cap_internal_size = key_cap_section.inner_wires()[0].bounding_box().size;

    // Add support ribs while leaving room for the switch
    Sketch ribs = Rectangle(key_cap_internal_size.X, 0.5 * MM);
    ribs += Rectangle(0.5 * MM, key_cap_internal_size.Y);
    ribs += Circle(5.51 * MM / 2);

    // Extrude the mount and ribs to the key cap underside
    key_cap += extrude(Pos(0, 0, 4 * MM) * ribs, Until::NEXT, key_cap);

    // Find the face on the bottom of the ribs to build onto
    Sketch socket = Circle(5.5 * MM / 2);
    socket -= Rectangle(4.1 * MM, 1.17 * MM);
    socket -= Rectangle(1.17 * MM, 4.1 * MM);
    key_cap += extrude(Plane::XY.offset(4 * MM) * socket, -3.5 * MM);

    example::show(key_cap, "key_cap");
    export_step(key_cap, "key_cap.step");
    return 0;
}
