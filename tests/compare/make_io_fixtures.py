"""Write the files in tests/compare/data that cases_io.cpp reads: data written by Python
build123d, so the C++ side can be checked reading it. Run it with Python build123d 0.13.0:

    python tests/compare/make_io_fixtures.py
"""

import os

from build123d import *  # noqa: F403
from build123d.persistence import serialize_location, serialize_shape

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")


def write(name, data):
    with open(os.path.join(DATA, name), "wb") as f:
        f.write(data)


def persistence():
    shape = Pos(1, 2, 3) * (Box(5, 6, 7) - Cylinder(2, 7)) + Pos(0, 0, 5) * Torus(4, 1)
    write("persist_py_shape.bin", serialize_shape(shape.wrapped))
    meshed = Rot(10, 20, 30) * Sphere(3)
    meshed.mesh(0.5, 0.5)
    write("persist_py_meshed.bin", serialize_shape(meshed.wrapped))
    write("persist_py_location.bin", serialize_location(Location((4, -5, 6), (30, 60, -45)).wrapped))


def step_files():
    # A nested assembly with names that import_step cleans
    red = Box(2, 2, 2)
    red.label = "red box (v1.0)"
    red.color = Color("red")
    ball = Pos(4, 0, 0) * Sphere(1)
    ball.label = "ball\u200b\tone"
    ball.color = Color(0.2, 0.4, 0.6, 0.8)
    inner = Compound(children=[ball, Pos(0, 4, 0) * Cylinder(1, 2)], label="inner.asm")
    inner.color = Color("green")
    outer = Compound(children=[red, Pos(0, 0, 5) * inner], label="outer")
    export_step(outer, os.path.join(DATA, "py_assembly.step"), timestamp="2024-01-01T00:00:00")
    # A single solid in inches
    export_step(Solid.make_cone(2, 1, 3), os.path.join(DATA, "py_cone_in.step"), unit=Unit.IN,
                timestamp="2024-01-01T00:00:00")
    face_colors_step(os.path.join(DATA, "py_face_colors.step"))


def face_colors_step(path):
    """A solid without a color whose faces have colors (import_step's largest-face fallback)."""
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.Quantity import Quantity_Color, Quantity_ColorRGBA, Quantity_TypeOfColor
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_StepModelType
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    app = XCAFApp_Application.GetApplication_s()
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    app.InitDocument(doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    solid = Solid.make_box(10, 2, 1)
    label = shape_tool.AddShape(solid.wrapped, False)
    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (0, 1, 1), (1, 0, 1)]
    for face, rgb in zip(solid.faces(), colors):
        sub = shape_tool.AddSubShape(label, face.wrapped)
        c = Quantity_ColorRGBA(Quantity_Color(*rgb, Quantity_TypeOfColor.Quantity_TOC_sRGB), 1.0)
        color_tool.SetColor(sub, c, XCAFDoc_ColorType.XCAFDoc_ColorSurf)
    writer = STEPCAFControl_Writer()
    writer.SetColorMode(True)
    writer.Transfer(doc, STEPControl_StepModelType.STEPControl_AsIs)
    assert writer.Write(path) == IFSelect_ReturnStatus.IFSelect_RetDone


def svg_files():
    # A hidden-line drawing and a sketch, as Python's ExportSVG writes them
    part = Box(30, 20, 10) - Pos(5, 0, 0) * Cylinder(4, 10) + Pos(-8, 0, 5) * Sphere(5)
    view = part.project_to_viewport((40, -60, 50))
    e = ExportSVG(unit=Unit.MM, line_weight=0.3)
    e.add_layer("hidden", line_color=Color("gray"), line_type=LineType.ISO_DOT)
    sketch = Pos(0, 60) * (Rectangle(40, 20) - Circle(5) - Pos(12, 0) * SlotOverall(8, 3))
    sketch.color = Color("orange")
    e.add_shape(view[0])
    e.add_shape(view[1], layer="hidden")
    e.add_shape(sketch)
    e.add_shape(Pos(-30, 60) * Text("SVG", 8))
    e.write(os.path.join(DATA, "svg_export.svg"))


def main():
    os.makedirs(DATA, exist_ok=True)
    persistence()
    step_files()
    svg_files()


if __name__ == "__main__":
    main()
