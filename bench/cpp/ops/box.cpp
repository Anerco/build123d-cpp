// The smallest model: one box.
#include "../bench.hpp"

using namespace build123d;

static Part model() { return Box(10, 10, 10); }

BENCH_MODEL("box", model)
