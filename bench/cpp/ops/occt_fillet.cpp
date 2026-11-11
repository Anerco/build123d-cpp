// Control: op_fillet's fillet called on OCCT directly (no build123d layer), as
// py/ops/occt_fillet.py does through OCP.
#include "../bench.hpp"

#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

using namespace build123d;

// built on first use (a namespace-scope Part would be built before the library's own statics)
static const Part& block() {
    static const Part b = Box(60, 60, 10) - GridLocations(18, 18, 3, 3) * (Pos(0, 0, 3) * Box(10, 10, 4));
    return b;
}

static Compound model() {
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(block().wrapped(), TopAbs_EDGE, edges);
    BRepFilletAPI_MakeFillet mk(block().wrapped());
    for (int i = 1; i <= edges.Extent(); ++i) mk.Add(1.0, TopoDS::Edge(edges.FindKey(i)));
    mk.Build();
    return Compound(mk.Shape());
}

BENCH_MODEL("occt_fillet", model)
