// A line of text, as faces.
#include "../bench.hpp"

using namespace build123d;

static Sketch model() { return Text("The quick brown fox jumps over the lazy dog", 10); }

BENCH_MODEL("op_text", model)
