// Export and import alone: `op_io <what> time ...` with what = export_step, export_stl,
// export_gltf or import_step. The part is op_holes' plate (225 holes), built before timing;
// the meshes STL and glTF export make are removed before each run (outside the timed
// region), or the next export would reuse them. import_step reads $BENCH_STEP_IN.
#include "../bench.hpp"

#include <BRepTools.hxx>

#include <cstdlib>
#include <string>

using namespace build123d;

int main(int argc, char** argv) {
    const std::string what = argv[1];
    const std::string out_dir = std::getenv("BENCH_OUT") ? std::getenv("BENCH_OUT") : "/tmp";
    const Part part = Box(200, 200, 10) - GridLocations(12, 12, 15, 15) * Cylinder(4, 10);
    std::function<bench::Shapes()> build;
    if (what == "export_step") {
        build = [&] { export_step(part, out_dir + "/op_cpp.step"); return bench::Shapes{{what, part}}; };
    } else if (what == "export_stl") {
        build = [&] { export_stl(part, out_dir + "/op_cpp.stl"); return bench::Shapes{{what, part}}; };
    } else if (what == "export_gltf") {
        build = [&] { export_gltf(part, out_dir + "/op_cpp.gltf"); return bench::Shapes{{what, part}}; };
    } else if (what == "import_step") {
        const std::string in = std::getenv("BENCH_STEP_IN");
        build = [in, what] { return bench::Shapes{{what, import_step(in)}}; };
    } else {
        return 2;
    }
    return bench::main_for(argc - 1, argv + 1, build, [&] { BRepTools::Clean(part.wrapped()); });
}
