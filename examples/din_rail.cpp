// A 1 m length of 35x7.5 mm DIN rail with slots.
// Port of build123d's examples/din_rail_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    // 35x7.5mm DIN Rail Dimensions
    const double overall_width = 35, top_width = 27, height = 7.5, thickness = 1, fillet_radius = 0.8;
    const int rail_length = 1000;
    const double slot_width = 6.2, slot_length = 15;
    const int slot_pitch = 25;

    // Create the rail profile
    Sketch din = Rectangle(overall_width, thickness, {Align::CENTER, Align::MIN});
    din += Rectangle(top_width, height, {Align::CENTER, Align::MIN});
    din -= Rectangle(top_width - 2 * thickness, height - thickness, {Align::CENTER, Align::MIN});

    // Fillet the inside and outside corners
    const auto inside_vertices = din.vertices()
                                     .filter_by_position(Axis::Y, 0.0, height, {false, false})
                                     .filter_by_position(Axis::X, -overall_width / 2, overall_width / 2, {false, false});
    din = fillet(inside_vertices, fillet_radius);
    const auto outside_vertices = din.vertices().filter_by([&](const Vertex& v) {
        return (v.Y() == 0.0 || v.Y() == height) && -overall_width / 2 < v.X() && v.X() < overall_width / 2;
    });
    din = fillet(outside_vertices, fillet_radius + thickness);

    // Extrude the profile and cut the slots
    Part rail = extrude(din, rail_length);
    const Plane plane(rail.faces().sort_by(Axis::Y).last());
    std::vector<Face> slot_faces;
    for (const auto& loc : GridLocations(0, slot_pitch, 1, rail_length / slot_pitch - 1)) {
        slot_faces.push_back((plane * loc * Rot(0, 0, 90) * SlotOverall(slot_length, slot_width)).faces()[0]);
    }
    rail -= extrude(slot_faces, -height);
    rail = Plane::XZ * rail;

    example::show(rail, "din_rail");
    export_step(rail, "din_rail.step");
    return 0;
}
