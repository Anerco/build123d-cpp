#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(Sprocket(32)); }

BENCH_MODEL("sprocket", model)
