// Replaces examples/show.hpp for the benchmark copies of the examples: show() keeps the
// shapes, the example's own export_step() is dropped (the Python examples don't export; the
// whole-run measurement exports once, in both languages, in the harness), and the example's
// main() becomes a function the harness calls.
#pragma once

#include "shim_show.hpp"

#define export_step(...) ((void)0)
#define main bench_example_main
