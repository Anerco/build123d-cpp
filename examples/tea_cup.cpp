// Tea cup: a revolved and hollowed bowl with a swept handle.
// Port of build123d's examples/tea_cup_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const double wall_thickness = 3 * MM;
    const double fillet_radius = wall_thickness * 0.49;

    // Create the bowl of the cup as a revolved cross section

    // Start & end points with control tangents
    Curve s = Spline({{30 * MM, 10 * MM}, {69 * MM, 105 * MM}}, {{1, 0.5}, {0.7, 1}}, {1.75, 1});
    // Lines to finish creating ½ the bowl shape
    s += Polyline({s.position_at(0), s.position_at(0) + Vector(10 * MM, -10 * MM), {0, 0},
                   {0, s.position_at(1).Y}, s.position_at(1)});
    const Sketch bowl_section = Plane::XZ * make_face(s);  // Create a filled 2D shape
    Part tea_cup = revolve(bowl_section, Axis::Z);

    // Hollow out the bowl with openings on the top and bottom
    tea_cup = offset(tea_cup, -wall_thickness, {.openings = tea_cup.faces().filter_by(GeomType::PLANE)});

    // Add a bottom to the bowl
    tea_cup += Pos(0, 0, s.position_at(0).Y) * Cylinder(s.position_at(0).X, wall_thickness);

    // Smooth out all the edges
    tea_cup = fillet(tea_cup.edges(), fillet_radius);

    // Determine where the handle contacts the bowl
    std::vector<Vector> handle_intersections;
    for (const double vertical_offset : {35 * MM, 80 * MM}) {
        const Axis axis({0, 0, vertical_offset}, {1, 0, 0});
        handle_intersections.push_back(tea_cup.find_intersection_points(axis).back().first);
    }
    // Create a path for handle creation
    const Spline path_spline({handle_intersections[0] - Vector(wall_thickness / 2, 0, 0),
                              handle_intersections[0] + Vector(35 * MM, 0, 30 * MM),
                              handle_intersections[0] + Vector(40 * MM, 0, 60 * MM),
                              handle_intersections[1] - Vector(wall_thickness / 2, 0, 0)},
                             {{1, 0, 1.25}, {-0.2, 0, -1}});

    // Align the cross section to the beginning of the path
    const Location location = path_spline ^ 0;
    const Sketch handle_cross_section = location * RectangleRounded(wall_thickness, 8 * MM, fillet_radius);

    // Sweep handle cross section along path
    tea_cup += sweep(handle_cross_section, path_spline);

    example::show(tea_cup, "tea_cup");
    export_step(tea_cup, "tea_cup.step");
    return 0;
}
