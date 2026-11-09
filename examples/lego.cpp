// Lego block: walls, ribs and hollow supports extruded from a sketch, with pips on top.
// Port of build123d's examples/lego_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const int pip_count = 6;

    const double lego_unit_size = 8;
    const double pip_height = 1.8;
    const double pip_diameter = 4.8;
    const double block_length = lego_unit_size * pip_count;
    const double block_width = 16;
    const double base_height = 9.6;
    const double support_outer_diameter = 6.5;
    const double support_inner_diameter = 4.8;
    const double ridge_width = 0.6;
    const double ridge_depth = 0.3;
    const double wall_thickness = 1.2;

    // Draw the base of the block
    Sketch plan = Rectangle(block_length, block_width);
    // Subtract an offset to create the block walls
    plan -= offset(plan, -wall_thickness, {.kind = Kind::INTERSECTION});
    // Add a grid of lengthwise and widthwise bars
    plan += GridLocations(0, lego_unit_size, 1, 2) * Rectangle(block_length, ridge_width);
    plan += GridLocations(lego_unit_size, 0, pip_count, 1) * Rectangle(ridge_width, block_width);
    // Subtract a rectangle leaving ribs on the block walls
    plan -= Rectangle(block_length - 2 * (wall_thickness + ridge_depth),
                      block_width - 2 * (wall_thickness + ridge_depth));
    // Add a row of hollow circles to the center
    const Sketch ring = Circle(support_outer_diameter / 2) - Circle(support_inner_diameter / 2);
    plan += GridLocations(lego_unit_size, 0, pip_count - 1, 1) * ring;

    // Extrude this base sketch to the height of the walls
    Part lego = extrude(plan, base_height - wall_thickness);
    // Create a box on the top of the walls
    lego += Pos(0, 0, lego.vertices().sort_by().last().Z()) *
            Box(block_length, block_width, wall_thickness, {Align::CENTER, Align::CENTER, Align::MIN});
    // Create a workplane on the top of the block
    const Plane plane(lego.faces().sort_by().last());
    // Add a grid of pips on the top of the block
    lego += plane * GridLocations(lego_unit_size, lego_unit_size, pip_count, 2) *
            Cylinder(pip_diameter / 2, pip_height, {Align::CENTER, Align::CENTER, Align::MIN});

    example::show(lego, "lego");
    export_step(lego, "lego.step");
    return 0;
}
