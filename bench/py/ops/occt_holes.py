"""Control: op_holes' boolean called on OCCT directly through OCP (no build123d layer), as
cpp/ops/occt_holes.cpp does. Measures the two OCCT builds against each other."""
from build123d import Compound
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.collections import List_TopoDS_Shape as TopTools_ListOfShape


def model():
    box = BRepPrimAPI_MakeBox(gp_Pnt(-100, -100, -5), 200, 200, 10).Shape()
    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(box)
    for i in range(15):
        for j in range(15):
            axis = gp_Ax2(gp_Pnt(-84 + 12 * i, -84 + 12 * j, -5), gp_Dir(0, 0, 1))
            tools.Append(BRepPrimAPI_MakeCylinder(axis, 4, 10).Shape())
    op = BRepAlgoAPI_Cut()
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetRunParallel(True)
    op.Build()
    return Compound(op.Shape())
