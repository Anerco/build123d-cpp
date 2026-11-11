#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(HelicalGear(2, 13, 20, 45, 10)); }

BENCH_MODEL("gear_helical", model)
