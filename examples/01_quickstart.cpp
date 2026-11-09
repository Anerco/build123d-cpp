// A plate with holes, filleted vertical edges and a counterbore — the C++ version of
//
//   from build123d import *
//   plate = Box(80, 60, 10) - [loc * Cylinder(3, 10) for loc in GridLocations(60, 40, 2, 2)]
//   plate = fillet(plate.edges().filter_by(Axis.Z), 5)
//   plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10)
#include <build123d/build123d.hpp>

#include <iostream>

using namespace build123d;

int main() {
    Part plate = Box(80, 60, 10) - GridLocations(60, 40, 2, 2) * Cylinder(3, 10);
    plate = fillet(plate.edges().filter_by(Axis::Z), 5);
    plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10);

    std::cout << "volume: " << plate.volume() << "\n";
    std::cout << "faces:  " << plate.faces().size() << "\n";
    std::cout << "bbox:   " << plate.bounding_box() << "\n";

    export_step(plate, "plate.step");
    export_stl(plate, "plate.stl");
    return 0;
}
