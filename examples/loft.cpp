// A vase-like loft through circles of varying size, then shelled.
// Port of build123d's examples/loft_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include <cmath>

#include "show.hpp"

using namespace build123d;

int main() {
    const int slice_count = 10;

    Sketch art;
    for (int i = 0; i <= slice_count; ++i) {
        const Plane plane(Vector(0, 0, i * 3.0));
        art += plane * Circle(10 * std::sin(i * PI / slice_count) + 5);
    }

    Part solid = loft(art);
    const auto top_bottom = solid.faces().filter_by(GeomType::PLANE);
    solid = offset(solid, 0.5, {.openings = top_bottom});

    example::show(solid, "loft");
    export_step(solid, "loft.step");
    return 0;
}
