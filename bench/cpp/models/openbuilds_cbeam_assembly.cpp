#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(CBeamAssembly(300.0)); }

BENCH_MODEL("openbuilds_cbeam_assembly", model)
