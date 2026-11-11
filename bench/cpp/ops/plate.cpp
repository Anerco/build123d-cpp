// The README's example.
#include "../bench.hpp"

using namespace build123d;

static Part model() {
    Part plate = Box(80, 60, 10) - GridLocations(60, 40, 2, 2) * Cylinder(3, 10);
    plate = fillet(plate.edges().filter_by(Axis::Z), 5);
    plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10);
    return plate;
}

BENCH_MODEL("plate", model)
