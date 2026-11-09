// A closed 3D curve of splines, an arc and a helix.
// Port of build123d's examples/roller_coaster_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const Spline powerup({{0, 0, 0}, {50, 0, 50}, {100, 0, 0}}, {{1, 0, 0}, {1, 0, 0}}, {0.5, 2});
    const RadiusArc corner(powerup.position_at(1), {100, 60, 0}, -30);
    const Helix screw(75, 150, 15, {75, 40, 15}, {-1, 0, 0});

    Curve roller_coaster = Curve() + (powerup + corner + screw);
    roller_coaster += Spline({corner.position_at(1), screw.position_at(0)}, {corner % 1, screw % 0});
    roller_coaster += Spline({screw.position_at(1), {-100, 30, 10}, powerup.position_at(0)}, {screw % 1, powerup % 0});

    example::show(roller_coaster, "roller_coaster");
    export_step(roller_coaster, "roller_coaster.step");
    return 0;
}
