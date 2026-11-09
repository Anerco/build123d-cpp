// Heat exchanger: a bundle of tubes through a plate with filleted joints.
// Port of build123d's examples/heat_exchanger_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include <cmath>
#include <iostream>

#include "show.hpp"

using namespace build123d;

int main() {
    const double exchanger_diameter = 10 * CM;
    const double exchanger_length = 30 * CM;
    const double plate_thickness = 5 * MM;
    // 149 tubes
    const double tube_diameter = 5 * MM;
    const double tube_spacing = 2 * MM;
    const double tube_wall_thickness = 0.5 * MM;
    const double tube_extension = 3 * MM;
    const double bundle_diameter = exchanger_diameter - 2 * tube_diameter;
    const double fillet_radius = tube_spacing / 3;

    // Build the parts of the heat exchanger
    const int count = static_cast<int>(exchanger_diameter / tube_diameter);
    std::vector<Location> tube_locations;
    for (const auto& l : HexLocations((tube_diameter + tube_spacing) / 2, count, count)) {
        if (l.position().length() < bundle_diameter / 2) tube_locations.push_back(l);
    }

    const Sketch ring = Circle(tube_diameter / 2) - Circle(tube_diameter / 2 - tube_wall_thickness);
    const Sketch tube_plan = Sketch() + tube_locations * ring;

    Part heat_exchanger = extrude(tube_plan, exchanger_length / 2);

    const Plane plate_plane(Vector(0, 0, exchanger_length / 2 - tube_extension - plate_thickness));
    const Sketch plate =
        Circle(exchanger_diameter / 2) - tube_locations * Circle(tube_diameter / 2 - tube_wall_thickness);

    heat_exchanger += extrude(plate_plane * plate, plate_thickness);
    const auto edges = heat_exchanger.edges()
                           .filter_by(GeomType::CIRCLE)
                           .group_by([](const Edge& e) { return e.radius(); })[1]
                           .group_by()[2];
    const double half_volume_before_fillet = heat_exchanger.volume();
    heat_exchanger = fillet(edges, fillet_radius);
    const double half_volume_after_fillet = heat_exchanger.volume();
    heat_exchanger += mirror(heat_exchanger, Plane::XY);

    const double fillet_volume = 2 * (half_volume_after_fillet - half_volume_before_fillet);
    if (std::abs(fillet_volume - 469.88331045553787) >= 1e-3) {
        std::cerr << "unexpected fillet volume " << fillet_volume << "\n";
        return 1;
    }

    example::show(heat_exchanger, "heat_exchanger");
    export_step(heat_exchanger, "heat_exchanger.step");
    return 0;
}
