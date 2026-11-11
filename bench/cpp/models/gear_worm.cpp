#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(WormGear(2, 2, 10, 30, 30, 12)); }

BENCH_MODEL("gear_worm", model)
