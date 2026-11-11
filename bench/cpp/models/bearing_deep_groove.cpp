#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(SingleRowDeepGrooveBallBearing("M8-22-7")); }

BENCH_MODEL("bearing_deep_groove", model)
