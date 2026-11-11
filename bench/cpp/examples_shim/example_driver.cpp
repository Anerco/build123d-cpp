// The harness around one example program (its main() renamed bench_example_main).
#include "../bench.hpp"
#include "shim_show.hpp"

int bench_example_main();

int main(int argc, char** argv) {
    return bench::main_for(argc, argv, [] {
        example::shown().clear();
        bench_example_main();
        bench::Shapes out = std::move(example::shown());
        example::shown().clear();
        return out;
    });
}
