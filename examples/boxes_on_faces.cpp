// A small plate placed on every face of a cube.
// Port of build123d's examples/boxes_on_faces_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    Part b = Box(3, 3, 3);
    const Part b2 = Rot(0, 0, 45) * extrude(Rectangle(1, 2), 0.1);
    std::vector<Plane> planes;
    for (const auto& f : b.faces()) planes.emplace_back(f);
    for (const auto& plane : planes) b += plane * b2;

    example::show(b, "boxes_on_faces");
    export_step(b, "boxes_on_faces.step");
    return 0;
}
