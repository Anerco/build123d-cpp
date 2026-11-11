#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() { return build123d::Shape(HexNut("M6-1", {.simple = false})); }

BENCH_MODEL("nut_hex_threaded", model)
