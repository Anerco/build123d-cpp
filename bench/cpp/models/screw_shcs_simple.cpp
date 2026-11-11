#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(SocketHeadCapScrew("M6-1", 20)); }

BENCH_MODEL("screw_shcs_simple", model)
