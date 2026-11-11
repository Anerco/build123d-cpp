"""Control: op_fillet's fillet called on OCCT directly through OCP (no build123d layer), as
cpp/ops/occt_fillet.cpp does."""
from build123d import Box, Compound, GridLocations, Pos
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as TopTools_IndexedMapOfShape

block = Box(60, 60, 10) - GridLocations(18, 18, 3, 3) * (Pos(0, 0, 3) * Box(10, 10, 4))


def model():
    edges = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(block.wrapped, TopAbs_EDGE, edges)
    mk = BRepFilletAPI_MakeFillet(block.wrapped)
    for i in range(1, edges.Extent() + 1):
        mk.Add(1.0, TopoDS.Edge(edges.FindKey(i)))
    mk.Build()
    return Compound(mk.Shape())
