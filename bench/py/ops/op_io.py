"""Export and import alone, the twin of cpp/ops/op_io.cpp: functions export_step, export_stl,
export_gltf and import_step. The part is op_holes' plate (225 holes), built before timing; the
meshes STL and glTF export make are removed before each run (outside the timed region), or the
next export would reuse them. import_step reads $BENCH_STEP_IN."""
import os

from build123d import Box, Cylinder, GridLocations
from build123d import export_gltf as _export_gltf
from build123d import export_step as _export_step
from build123d import export_stl as _export_stl
from build123d import import_step as _import_step
from OCP.BRepTools import BRepTools

OUT = os.environ.get("BENCH_OUT", "/tmp")
part = Box(200, 200, 10) - GridLocations(12, 12, 15, 15) * Cylinder(4, 10)


def prepare():
    BRepTools.Clean_s(part.wrapped)


def export_step():
    _export_step(part, f"{OUT}/op_py.step")
    return part


def export_stl():
    _export_stl(part, f"{OUT}/op_py.stl")
    return part


def export_gltf():
    _export_gltf(part, f"{OUT}/op_py.gltf")
    return part


def import_step():
    return _import_step(os.environ["BENCH_STEP_IN"])
