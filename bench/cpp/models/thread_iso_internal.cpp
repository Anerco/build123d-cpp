#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(IsoThread(10, 1.5, 20, false)); }

BENCH_MODEL("thread_iso_internal", model)
