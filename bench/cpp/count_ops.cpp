// LD_PRELOAD shim: counts the OCCT operations a C++ program constructs (booleans, fillets,
// UnifySameDomain, ShapeFix_Shape), to compare with the same constructors counted on the Python
// side by py/ocp_calls.py. Prints the counts to stderr at exit.
//
//   LD_PRELOAD=bin/count_ops.so bin/<model> time 0 5 5 0
#include <dlfcn.h>

#include <cstdio>

namespace {
struct Count {
    const char* name;
    long n = 0;
    ~Count() { std::fprintf(stderr, "count_ops: %-30s %ld\n", name, n); }
};
}  // namespace

#define FORWARD(counter, label, sym, params, args)                                      \
    namespace {                                                                         \
    Count counter{label};                                                               \
    }                                                                                   \
    extern "C" void sym params {                                                        \
        using F = void(*) params;                                                       \
        static F real = reinterpret_cast<F>(dlsym(RTLD_NEXT, #sym));                    \
        ++counter.n;                                                                    \
        real args;                                                                      \
    }

FORWARD(fuse, "BRepAlgoAPI_Fuse", _ZN16BRepAlgoAPI_FuseC1Ev, (void* self), (self))
FORWARD(cut, "BRepAlgoAPI_Cut", _ZN15BRepAlgoAPI_CutC1Ev, (void* self), (self))
FORWARD(common, "BRepAlgoAPI_Common", _ZN18BRepAlgoAPI_CommonC1Ev, (void* self), (self))
FORWARD(unify, "ShapeUpgrade_UnifySameDomain", _ZN28ShapeUpgrade_UnifySameDomainC1ERK12TopoDS_Shapebbb,
        (void* self, const void* s, bool a, bool b, bool c), (self, s, a, b, c))
FORWARD(fix, "ShapeFix_Shape", _ZN14ShapeFix_ShapeC1ERK12TopoDS_Shape, (void* self, const void* s), (self, s))
FORWARD(fillet, "BRepFilletAPI_MakeFillet", _ZN24BRepFilletAPI_MakeFilletC1ERK12TopoDS_Shape18ChFi3d_FilletShape,
        (void* self, const void* s, int t), (self, s, t))
