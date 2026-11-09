"""Write the files in tests/compare/data that cases_io.cpp reads: data written by Python
build123d, so the C++ side can be checked reading it. Run it with Python build123d 0.13.0:

    python tests/compare/make_io_fixtures.py
"""

import os

import ezdxf
from ezdxf.enums import TextEntityAlignment

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


# ---- DXF files for import_dxf, written by ezdxf (as CAD programs write them)
def entities_doc():
    doc = ezdxf.new("R2018", setup=False)
    msp = doc.modelspace()
    msp.add_line((0, 0), (10, 0))
    msp.add_line((0, 2, 1), (10, 4, 1))
    msp.add_line((5, 5, 1), (5, 5, 1))  # degenerate: skipped
    msp.add_arc((20, 0), 5, 30, 120)
    msp.add_arc((35, 0), 4, 300, 60)  # crosses 0 degrees
    msp.add_arc((50, 0), 2, -90, 450.5)
    msp.add_circle((60, 0, 2), 3)
    msp.add_ellipse((0, 20), major_axis=(6, 2), ratio=0.4)
    msp.add_ellipse((20, 20), major_axis=(0, 5), ratio=0.5, start_param=0.5, end_param=4.0)
    msp.add_ellipse((40, 20, 1), major_axis=(-4, -3), ratio=0.3, start_param=5.0, end_param=1.0)
    msp.add_lwpolyline([(0, 40, 0), (10, 40, 0.5), (10, 50, 0), (0, 50, -1)], format="xyb", close=True,
                       dxfattribs={"elevation": 2})
    msp.add_lwpolyline([(20, 40, 0), (30, 45, 0.25), (35, 40, 0)], format="xyb")
    msp.add_lwpolyline([(40, 40), (45, 42)], format="xy")  # one segment: an Edge
    msp.add_lwpolyline([(50, 40), (50, 40)])  # degenerate
    msp.add_lwpolyline([(60, 40)])  # fewer than 2 points
    pl = msp.add_polyline2d([(0, 60, 0, 0, 0.3), (10, 60, 0, 0, 0), (10, 70, 0, 0, -0.6), (0, 70, 0, 0, 0)],
                            format="xyseb", close=True)
    msp.add_polyline2d([(20, 60), (30, 65), (40, 60)], format="xy", dxfattribs={"elevation": (0, 0, 3)})
    msp.add_point((5, 80, 3))
    msp.add_point((15, 80))
    msp.add_spline(fit_points=[(0, 90), (5, 95), (10, 90), (15, 95)])
    s = msp.add_spline(fit_points=[(20, 90), (25, 96), (30, 91)])
    s.dxf.start_tangent = (1, 0, 0)
    s.dxf.end_tangent = (0, -1, 0)
    msp.add_open_spline(control_points=[(40, 90), (45, 100), (50, 90), (55, 100), (60, 92)], degree=3)
    msp.add_rational_spline(control_points=[(0, 110), (5, 120), (10, 110), (15, 120)], weights=[1, 2, 0.5, 1],
                            degree=3)
    msp.add_open_spline(control_points=[(20, 110), (25, 118), (30, 110)], degree=2)
    msp.add_text("Hello", height=3, rotation=15).set_placement((0, 130))
    msp.add_text("Mid", height=2).set_placement((20, 130), align=TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text("TR", height=2.5).set_placement((40, 130), align=TextEntityAlignment.TOP_RIGHT)
    msp.add_text("BL", height=2.5).set_placement((50, 130), align=TextEntityAlignment.BOTTOM_LEFT)
    msp.add_mtext("Plain {\\C1;red} 50%%d", dxfattribs={"char_height": 2.5, "attachment_point": 5,
                                                         "insert": (0, 150), "rotation": 10})
    msp.add_mtext("x\\S1^2;y", dxfattribs={"char_height": 3, "attachment_point": 9, "insert": (30, 150)})
    msp.add_xline((0, 0), (1, 1))  # not converted: warning
    blk = doc.blocks.new("BLK", base_point=(1, 1))
    blk.add_line((0, 0), (5, 0))
    blk.add_circle((2, 2), 1)
    blk.add_point((3, 3))
    msp.add_blockref("BLK", (70, 0), dxfattribs={"xscale": 2, "yscale": 0.5, "rotation": 30})
    msp.add_blockref("blk", (90, 0), dxfattribs={"column_count": 2, "row_count": 3, "column_spacing": 8,
                                                 "row_spacing": 6, "rotation": 90})
    inner = doc.blocks.new("INNER")
    inner.add_blockref("BLK", (10, 10), dxfattribs={"rotation": 45})
    inner.add_lwpolyline([(0, 0), (4, 0), (4, 4)], close=True)
    msp.add_blockref("INNER", (110, 0), dxfattribs={"xscale": 1.5, "yscale": 1.5, "zscale": 0})
    doc.saveas(os.path.join(DATA, "dxf_entities.dxf"))


def hatch_doc():
    doc = ezdxf.new("R2013", setup=False)
    msp = doc.modelspace()
    h = msp.add_hatch(color=2)
    h.paths.add_polyline_path([(0, 0), (10, 0), (10, 10), (0, 10)], is_closed=True, flags=1)
    h.paths.add_polyline_path([(2, 2, 0.4), (8, 2, 0), (8, 8, 0), (2, 8, 0)], is_closed=True, flags=16)
    h.paths.add_polyline_path([(4, 4), (6, 4), (6, 6), (4, 6)], is_closed=True, flags=0)
    h2 = msp.add_hatch(dxfattribs={"elevation": (0, 0, 2), "hatch_style": 1})
    h2.paths.add_polyline_path([(20, 0), (30, 0), (30, 10), (20, 10)], is_closed=True, flags=1)
    h2.paths.add_polyline_path([(22, 2), (28, 2), (28, 8)], is_closed=True, flags=16)
    h2.paths.add_polyline_path([(24, 4), (26, 4), (26, 6)], is_closed=True, flags=0)
    h3 = msp.add_hatch(dxfattribs={"hatch_style": 2})
    h3.paths.add_polyline_path([(40, 0), (50, 0), (50, 10)], is_closed=True, flags=1)
    h3.paths.add_polyline_path([(42, 1), (48, 1), (48, 7)], is_closed=True, flags=16)
    h4 = msp.add_hatch()
    ep = h4.paths.add_edge_path(flags=1)
    ep.add_line((60, 0), (70, 0))
    ep.add_arc((70, 5), 5, -90, 90, ccw=True)
    ep.add_line((70, 10), (60, 10))
    ep.add_ellipse((60, 5), major_axis=(0, 5), ratio=0.4, start_angle=0, end_angle=180, ccw=True)
    ep2 = h4.paths.add_edge_path(flags=0)
    ep2.add_arc((65, 5), 2, 0, 360, ccw=False)
    h5 = msp.add_hatch()
    ep3 = h5.paths.add_edge_path(flags=1)
    ep3.add_spline(control_points=[(80, 0), (85, 8), (90, 0), (95, 8)], knot_values=[0, 0, 0, 0, 1, 1, 1, 1],
                   degree=3)
    ep3.add_line((95, 8), (80, 0))
    ep4 = h5.paths.add_edge_path(flags=0)
    ep4.add_ellipse((86, 3), major_axis=(2, 1), ratio=0.5, start_angle=30, end_angle=300, ccw=False)
    h6 = msp.add_hatch()
    h6.paths.add_polyline_path([(100, 0), (110, 0)], is_closed=False, flags=1)
    h6.paths.add_polyline_path([(100, 5), (100, 5)], is_closed=True, flags=1)
    doc.saveas(os.path.join(DATA, "dxf_hatch.dxf"))


def r12_doc():
    doc = ezdxf.new("R12", setup=False)
    msp = doc.modelspace()
    msp.add_line((0, 0), (10, 5))
    msp.add_arc((20, 0), 5, 200, 10)
    msp.add_circle((30, 0), 2)
    msp.add_polyline2d([(0, 20, 0, 0, 1), (10, 20, 0, 0, 0), (10, 30, 0, 0, 0)], format="xyseb", close=True)
    msp.add_point((1, 2, 3))
    msp.add_text("R12", height=4).set_placement((0, 40), align=TextEntityAlignment.CENTER)
    blk = doc.blocks.new("B12", base_point=(0, 0))
    blk.add_circle((0, 0), 1)
    blk.add_line((-1, 0), (1, 0))
    msp.add_blockref("B12", (50, 50), dxfattribs={"xscale": 3, "yscale": 3, "rotation": 10})
    doc.saveas(os.path.join(DATA, "dxf_r12.dxf"))


def binary_doc():
    doc = ezdxf.new("R2010", setup=False)
    msp = doc.modelspace()
    msp.add_line((0, 0), (3, 4))
    msp.add_circle((5, 5), 1.5)
    msp.add_lwpolyline([(0, 10, 0.2), (5, 10, 0), (5, 15, 0)], format="xyb", close=True)
    msp.add_text("Größe ±1", height=2).set_placement((0, 20))
    msp.add_open_spline(control_points=[(0, 30), (2, 33), (4, 30), (6, 33)])
    doc.saveas(os.path.join(DATA, "dxf_binary.dxf"), fmt="bin")


def cp1252_doc():
    doc = ezdxf.new("R2000", setup=False)
    msp = doc.modelspace()
    msp.add_text("Größe €5 – ok", height=2).set_placement((0, 0))
    msp.add_mtext("Ä %%p0,1 %%c8", dxfattribs={"char_height": 2, "insert": (0, 10)})
    msp.add_line((0, -5), (10, -5))
    doc.saveas(os.path.join(DATA, "dxf_cp1252.dxf"))


def face_doc():
    doc = ezdxf.new("R2018", setup=False)
    msp = doc.modelspace()
    msp.add_line((0, 0), (1, 0))
    msp.add_3dface([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)])
    doc.saveas(os.path.join(DATA, "dxf_3dface.dxf"))


def polyline3d_doc():
    doc = ezdxf.new("R2018", setup=False)
    msp = doc.modelspace()
    msp.add_polyline3d([(0, 0, 0), (1, 0, 1), (1, 1, 2)])
    doc.saveas(os.path.join(DATA, "dxf_polyline3d.dxf"))


def dxf_files():
    entities_doc()
    hatch_doc()
    r12_doc()
    binary_doc()
    cp1252_doc()
    face_doc()
    polyline3d_doc()


def main():
    os.makedirs(DATA, exist_ok=True)
    persistence()
    step_files()
    svg_files()
    dxf_files()


if __name__ == "__main__":
    main()
