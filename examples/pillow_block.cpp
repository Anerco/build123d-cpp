// Pillow block: a filleted plate with a bearing counterbore and four screw counterbores.
// Port of build123d's examples/pillow_block_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const double height = 60, width = 80, thickness = 10, padding = 12;
    const double screw_shaft_radius = 1.5, screw_head_radius = 3, screw_head_height = 3;
    const double bearing_axle_radius = 4, bearing_radius = 11, bearing_thickness = 7;

    // Make the plan of the pillow block
    Sketch plan = Rectangle(width, height);
    plan = fillet(plan.vertices(), 5);

    // Extrude the plan into a block
    Part pillow_block = extrude(plan, thickness);
    const Plane plane(pillow_block.faces().sort_by().last());

    // Create the bearing hole
    pillow_block -= plane * CounterBoreHole(bearing_axle_radius, bearing_radius, bearing_thickness, height);

    // Create the screw holes
    pillow_block -= plane * GridLocations(width - 2 * padding, height - 2 * padding, 2, 2) *
                    CounterBoreHole(screw_shaft_radius, screw_head_radius, screw_head_height, height);

    example::show(pillow_block, "pillow_block");
    export_step(pillow_block, "pillow_block.step");
    return 0;
}
