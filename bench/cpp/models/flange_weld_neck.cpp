#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(WeldNeckFlange("4", 150)); }

BENCH_MODEL("flange_weld_neck", model)
