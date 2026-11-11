// LD_PRELOAD shim: counts and times two OCCT calls in a C++ program, the ones py/ocp_calls.py
// finds the Python side spending OCCT time in that the port may not make:
// BRepBndLib::AddOptimal (optimal bounding boxes) and the BRepBuilderAPI_Copy constructor
// (deep shape copies). Prints the totals to stderr at exit.
//
//   LD_PRELOAD=bin/count_calls.so bin/<model> time 2 10 10 0
#include <dlfcn.h>

#include <chrono>
#include <cstdio>

class TopoDS_Shape;
class Bnd_Box;
class BRepBuilderAPI_Copy;

namespace {
using clk = std::chrono::steady_clock;
struct Tally {
    const char* name;
    long calls = 0;
    double ms = 0;
    ~Tally() { std::fprintf(stderr, "count_calls: %s %ld calls %.3f ms\n", name, calls, ms); }
};
Tally add_optimal{"AddOptimal"};
Tally copy{"BRepBuilderAPI_Copy"};
}  // namespace

// BRepBndLib::AddOptimal(const TopoDS_Shape&, Bnd_Box&, bool, bool)
extern "C" void _ZN10BRepBndLib10AddOptimalERK12TopoDS_ShapeR7Bnd_Boxbb(const TopoDS_Shape& s, Bnd_Box& b,
                                                                       bool tri, bool tol) {
    using F = void (*)(const TopoDS_Shape&, Bnd_Box&, bool, bool);
    static F real = reinterpret_cast<F>(dlsym(RTLD_NEXT, "_ZN10BRepBndLib10AddOptimalERK12TopoDS_ShapeR7Bnd_Boxbb"));
    const auto t0 = clk::now();
    real(s, b, tri, tol);
    add_optimal.ms += std::chrono::duration<double, std::milli>(clk::now() - t0).count();
    ++add_optimal.calls;
}

// BRepBuilderAPI_Copy::BRepBuilderAPI_Copy(const TopoDS_Shape&, bool, bool)
extern "C" void _ZN19BRepBuilderAPI_CopyC1ERK12TopoDS_Shapebb(BRepBuilderAPI_Copy* self, const TopoDS_Shape& s,
                                                              bool geom, bool mesh) {
    using F = void (*)(BRepBuilderAPI_Copy*, const TopoDS_Shape&, bool, bool);
    static F real = reinterpret_cast<F>(dlsym(RTLD_NEXT, "_ZN19BRepBuilderAPI_CopyC1ERK12TopoDS_Shapebb"));
    const auto t0 = clk::now();
    real(self, s, geom, mesh);
    copy.ms += std::chrono::duration<double, std::milli>(clk::now() - t0).count();
    ++copy.calls;
}
