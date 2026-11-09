// Spheres subtracted at workplanes on the top and bottom faces of a plate.
// Port of build123d's examples/multiple_workplanes_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    Part obj = Box(5, 5, 1);
    std::vector<Plane> planes;
    for (const auto& f : obj.faces().filter_by(Axis::Z)) planes.emplace_back(f);
    obj -= planes * Sphere(1.8);

    example::show(obj, "multiple_workplanes");
    export_step(obj, "multiple_workplanes.step");
    return 0;
}
