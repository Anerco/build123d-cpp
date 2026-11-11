// One boolean: a plate minus 225 cylinders.
#include "../bench.hpp"

using namespace build123d;

static Part model() { return Box(200, 200, 10) - GridLocations(12, 12, 15, 15) * Cylinder(4, 10); }

BENCH_MODEL("op_holes", model)
