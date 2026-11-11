#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(SpurGear(1.5, 31, 20, 4)); }

BENCH_MODEL("gear_spur", model)
