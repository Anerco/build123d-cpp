// Fillet every edge (120) of a block with nine pockets.
#include "../bench.hpp"

using namespace build123d;

// built on first use (a namespace-scope Part would be built before the library's own statics)
static const Part& block() {
    static const Part b = Box(60, 60, 10) - GridLocations(18, 18, 3, 3) * (Pos(0, 0, 3) * Box(10, 10, 4));
    return b;
}

static Part model() { return fillet(block().edges(), 1); }

BENCH_MODEL("op_fillet", model)
