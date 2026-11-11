// Control: op_holes' boolean called on OCCT directly (no build123d layer), as py/ops/occt_holes.py
// does through OCP. Measures the two OCCT builds against each other.
#include "../bench.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Ax2.hxx>

static build123d::Compound model() {
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -5), 200, 200, 10).Shape();
    TopTools_ListOfShape args, tools;
    args.Append(box);
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            const gp_Ax2 axis(gp_Pnt(-84 + 12 * i, -84 + 12 * j, -5), gp_Dir(0, 0, 1));
            tools.Append(BRepPrimAPI_MakeCylinder(axis, 4, 10).Shape());
        }
    }
    BRepAlgoAPI_Cut op;
    op.SetArguments(args);
    op.SetTools(tools);
    op.SetRunParallel(true);
    op.Build();
    return build123d::Compound(op.Shape());
}

BENCH_MODEL("occt_holes", model)
