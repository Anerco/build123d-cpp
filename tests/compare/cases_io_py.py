"""Python twins of cases_io.cpp."""

import datetime
import io
import json
import os
import tempfile

from build123d import *  # noqa: F403
from build123d.persistence import deserialize_location, deserialize_shape, serialize_location, serialize_shape

from values import V

HERE = os.path.dirname(os.path.abspath(__file__))


def exchange_dir(side):
    path = os.path.join(tempfile.gettempdir(), "build123d_compare_io", side)
    os.makedirs(path, exist_ok=True)
    return path


def cpp_file(name):
    """A file the C++ side wrote (compare.py runs it first)."""
    return os.path.join(exchange_dir("cpp"), name)


def fixture(name):
    return os.path.join(HERE, "data", name)


def read_bytes(path):
    with open(path, "rb") as f:
        return f.read()


def read_text(path):
    return read_bytes(path).decode("utf-8", errors="surrogateescape")


def py_file(name):
    return os.path.join(exchange_dir("py"), name)


def color_value(c):
    return None if c is None else list(tuple(c))


def shape_tree(s):
    loc = s.location if s.wrapped is not None else Location()
    return V(
        {
            "type": type(s).__name__ if type(s).__name__ in ("Vertex", "Edge", "Wire", "Face", "Shell", "Solid") else "Compound",
            "label": s.label,
            # the stored color: the getter would inherit the parent's (assembly-tree behaviour)
            "color": color_value(s._color),
            "position": loc.position,
            "orientation": loc.orientation,
            "volume": s.volume if s.wrapped is not None else 0.0,
            "faces": len(s.faces()) if s.wrapped is not None else 0,
            "children": [shape_tree(c) for c in s.children],
        }
    )


def obj_canonical(text):
    """See cases_io.cpp: "f" lines sorted, normals as numbers."""
    lines = text.split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    faces = sorted(line for line in lines if line.startswith("f "))
    normals = [[float(x) for x in line.split()[1:]] for line in lines if line.startswith("vn ")]
    others = "".join(line + "\n" for line in lines if not line.startswith("f ") and not line.startswith("vn "))
    return [others, "".join(f + "\n" for f in faces), normals]


def step_canonical(text):
    """See cases_io.cpp: the header, and the sorted entities with #N references blanked."""
    header, data = text.split("DATA;\n", 1)
    entities = []
    current = ""
    i = 0
    while i < len(data):
        c = data[i]
        if c == "\n":
            if current.endswith(";"):
                entities.append(current)
                current = ""
            while i + 1 < len(data) and data[i + 1] == " ":
                i += 1
        elif c == "#" and i + 1 < len(data) and data[i + 1].isdigit():
            current += "#"
            while i + 1 < len(data) and data[i + 1].isdigit():
                i += 1
        else:
            current += c
        i += 1
    if current:
        entities.append(current)
    return [header, sorted(entities)]


def sample_assembly():
    red = Box(2, 2, 2)
    red.label = "red box (v1.0)"
    red.color = Color("red")
    ball = Pos(4, 0, 5) * Sphere(1)
    ball.label = "ball\tone"
    ball.color = Color(0.2, 0.4, 0.6, 0.8)
    inner = Compound(children=[ball, Pos(0, 4, 5) * Cylinder(1, 2)], label="inner.asm")
    inner.color = Color("green")
    return Compound(children=[red, inner], label="outer")


def edge_summary(e):
    return [e.geom_type, e.length, e.start_point(), e.end_point()]


def viewport(vh):
    visible, hidden = vh
    return V({"visible": [edge_summary(e) for e in visible], "hidden": [edge_summary(e) for e in hidden]})


def hlr_holed_part():
    part = Box(10, 10, 10) - Cylinder(3, 10)
    return viewport(part.project_to_viewport((-20, 15, 30), (0, 1, 0)))


def hlr_look_at():
    part = Box(10, 10, 10) - Pos(2, 0, 0) * Sphere(4)
    return viewport(part.project_to_viewport((0, -50, 0), (0, 0, 1), (5, 0, 0)))


def hlr_perspective():
    part = Cylinder(5, 10) + Pos(0, 0, 5) * Box(12, 3, 3)
    return viewport(part.project_to_viewport((30, -40, 25), (0, 0, 1), None, 60.0))


def json_encode():
    data = {
        "a": Axis((1, 2, 3), (0, 1, 0)),
        "c": Color("red"),
        "c2": Color(0.1, 0.2, 0.3, 0.4),
        "l": Location((1, 2, 3), (10, 20, 30)),
        "p": Plane.XZ,
        "v": Vector(1, 2.5, -0.0),
        "list": [1, 2.0, None, True, "x\u00e9\n\x7f\U0001F600\"\\/"],
        "nested": {"empty": {}, "e2": []},
    }
    return V(
        [
            json.dumps(data, cls=GeomEncoder),
            json.dumps(data, cls=GeomEncoder, indent=2),
            json.dumps(data, cls=GeomEncoder, indent=0),
        ]
    )


def json_floats():
    floats = [0.1, 1e-05, 1e16, 1e15, 123456789012345678.0, 5e-324, -0.0, 1.0, 100.0, 0.0001, 1.5e-7, 2.5, 1.0 / 3.0,
              1e22, 1.7976931348623157e308, -2.5e-10, 12345.678, float("nan"), float("inf"), float("-inf"), 0, -7,
              9007199254740993]
    return V(json.dumps(floats))


def json_dumps_each():
    return V(
        [
            json.dumps(Axis.Z, cls=GeomEncoder),
            json.dumps(Color(0.5, 0.25, 1), cls=GeomEncoder),
            json.dumps(Location((1, 2, 3), (0, 0, 90)), cls=GeomEncoder),
            json.dumps(Plane.isometric, cls=GeomEncoder, indent=4),
            json.dumps(Vector(1e-7, 3, 1e300), cls=GeomEncoder),
        ]
    )


def json_decode():
    out = []
    for text in [
        '{"Axis": [[1.0, 2.0, 3.0], [0.0, 1.0, 0.0]]}',
        '{"Color": [0.1, 0.2, 0.3]}',
        '{"Location": [[1, 2, 3], [10, 20, 30]]}',
        '{"Plane": [[0, 0, 1], [1, 0, 0], [0, -1, 0]]}',
        '{"Vector": [1, 2]}',
    ]:
        g = json.loads(text, object_hook=GeomEncoder.geometry_hook)
        out.append(json.dumps(g, cls=GeomEncoder))
    return V(out)


def json_hook_two_keys():
    json.loads('{"Vector": [1, 2, 3], "Axis": [[0, 0, 0], [0, 0, 1]]}', object_hook=GeomEncoder.geometry_hook)
    return V("decoded")


def json_hook_unknown():
    json.loads('{"Foo": [1]}', object_hook=GeomEncoder.geometry_hook)
    return V("decoded")


def json_roundtrip():
    text = (
        '{"b": [1, 2.50, -0.0, 1E3, 1e-7, true, false, null, {"x": "\\u00e9\\ud83d\\ude00\\n\\/"}],'
        ' "a": {}, "b": 3, "nan": NaN, "inf": -Infinity}'
    )
    return V(json.dumps(json.loads(text)))


def json_malformed():
    json.loads("[1, 2,]")
    return V("parsed")


def persist_shape_bytes():
    return V(
        [
            serialize_shape(Box(1, 2, 3).wrapped).hex(),
            serialize_shape((Box(4, 4, 4) - Cylinder(1, 4)).wrapped).hex(),
            serialize_shape(Edge.make_circle(2).wrapped).hex(),
        ]
    )


def persist_shape_from_py():
    data = read_bytes(fixture("persist_py_shape.bin"))
    shape = Pos(1, 2, 3) * (Box(5, 6, 7) - Cylinder(2, 7)) + Pos(0, 0, 5) * Torus(4, 1)
    assert serialize_shape(shape.wrapped) == data, "fixture is out of date"
    return Compound.cast(deserialize_shape(data))


def persist_location_from_py():
    loc = Location(deserialize_location(read_bytes(fixture("persist_py_location.bin"))))
    return V({"position": loc.position, "orientation": loc.orientation})


def persist_location_round_trip():
    loc = Location(deserialize_location(serialize_location(Location((1, 2, 3), (10, 20, 30)).wrapped)))
    return V({"position": loc.position, "orientation": loc.orientation})


def step_text_box():
    box = Box(1, 2, 3)
    box.label = "my box"
    export_step(box, py_file("box.step"), Unit.MM, True, PrecisionMode.AVERAGE, timestamp="2024-01-01T00:00:00")
    return V(read_text(py_file("box.step")))


def step_text_assembly():
    export_step(sample_assembly(), py_file("assembly.step"), timestamp="2024-01-01T00:00:00")
    return V(step_canonical(read_text(py_file("assembly.step"))))


def step_text_options():
    part = Cylinder(2, 5) - Box(1, 1, 5)
    part.color = Color(0.1, 0.5, 0.9)
    stream = io.BytesIO()
    export_step(part, stream, Unit.IN, False, PrecisionMode.GREATEST, timestamp="2025-06-07T08:09:10")
    export_step(Edge.make_circle(3), py_file("circle.step"), Unit.M, True, PrecisionMode.LEAST,
                timestamp=datetime.datetime(2025, 6, 7, 8, 9, 10))
    return V([stream.getvalue().decode(), read_text(py_file("circle.step"))])


def step_import_cpp_assembly():
    export_step(sample_assembly(), py_file("assembly2.step"))
    return shape_tree(import_step(py_file("assembly2.step")))


def step_import_single_solid():
    s = Solid.make_box(1, 2, 3)
    s.label = "solid one"
    s.color = Color("red")
    export_step(s, py_file("solid.step"))
    return shape_tree(import_step(py_file("solid.step")))


def brep_text():
    part = Rot(10, 20, 30) * (Box(3, 3, 3) - Sphere(1.8))
    export_brep(part, py_file("part.brep"))
    stream = io.BytesIO()
    export_brep(part, stream)
    return V([read_text(py_file("part.brep")), stream.getvalue().decode()])


def brep_import():
    export_brep(Pos(1, 2, 3) * Face.make_rect(2, 3), py_file("face.brep"))
    return shape_tree(import_brep(py_file("face.brep")))


def stl_ascii():
    export_stl(Box(1, 2, 3), py_file("box.stl"), 1e-3, 0.1, True)
    return V(read_text(py_file("box.stl")))


def stl_binary():
    export_stl(Cylinder(2, 3), py_file("cyl.stl"), 0.05, 0.3)
    return V(read_bytes(py_file("cyl.stl")).hex())


def stl_import_units():
    export_stl(Box(1, 2, 3), py_file("unit_box.stl"))
    out = []
    for u in [Unit.MM, Unit.MC, Unit.CM, Unit.M, Unit.IN, Unit.FT]:
        bb = import_stl(py_file("unit_box.stl"), u).bounding_box()
        out.append([bb.min, bb.max])
    return V(out)


def stl_import_bad_unit():
    export_stl(Box(1, 2, 3), py_file("unit_box2.stl"))
    import_stl(py_file("unit_box2.stl"), Unit.KG)
    return V("imported")


def obj_uvs():
    part = Box(4, 3, 2) - Cylinder(1, 2)
    export_obj(part, py_file("part.obj"), 0.01, 0.2)
    return V(obj_canonical(read_text(py_file("part.obj"))))


def obj_options():
    part = Sphere(2) + Pos(2, 0, 0) * Box(1, 1, 1)
    out = []
    for kwargs in [dict(include_uvs=False), dict(atlas_packing=False), dict(atlas_gutter=0.02)]:
        stream = io.BytesIO()
        export_obj(part, stream, 0.05, 0.3, **kwargs)
        out.append(obj_canonical(stream.getvalue().decode()))
    return V(out)


def obj_sketch():
    stream = io.BytesIO()
    export_obj(Circle(2) - Rectangle(1, 1), stream, 0.01)
    return V(obj_canonical(stream.getvalue().decode()))


def gltf_assembly():
    export_gltf(sample_assembly(), py_file("assembly.gltf"))
    return V([json.loads(read_text(py_file("assembly.gltf"))), len(read_bytes(py_file("assembly.bin")))])


def glb_json(data):
    length = int.from_bytes(data[12:16], "little")
    return json.loads(data[20 : 20 + length])


def gltf_glb_options():
    part = Rot(0, 0, 30) * (Cylinder(10, 5) - Box(4, 4, 5))
    part.label = "part"
    part.color = Color(0.25, 0.5, 0.75, 0.5)
    export_gltf(part, py_file("part.glb"), Unit.M, True, 0.01, 0.3)
    glb = read_bytes(py_file("part.glb"))
    return V([glb_json(glb), len(glb)])


def gltf_sketch_curve():
    export_gltf(Circle(3) - Rectangle(1, 2), py_file("sketch.gltf"))
    return V(json.loads(read_text(py_file("sketch.gltf"))))


CASES = {
    # ---- glTF
    "io_gltf_assembly": gltf_assembly,
    "io_gltf_glb_options": gltf_glb_options,
    "io_gltf_sketch_curve": gltf_sketch_curve,
    "io_gltf_missing_dir": lambda: V(export_gltf(Box(1, 1, 1), py_file("no/dir/x.gltf"))),
    # ---- STEP
    "io_step_text_box": step_text_box,
    "io_step_text_assembly": step_text_assembly,
    "io_step_text_options": step_text_options,
    "io_step_import_py_assembly": lambda: shape_tree(import_step(fixture("py_assembly.step"))),
    "io_step_import_cpp_assembly": step_import_cpp_assembly,
    "io_step_import_face_colors": lambda: shape_tree(import_step(fixture("py_face_colors.step"))),
    "io_step_import_inches": lambda: shape_tree(import_step(fixture("py_cone_in.step"))),
    "io_step_import_single_solid": step_import_single_solid,
    "io_step_import_missing": lambda: shape_tree(import_step(py_file("missing.step"))),
    "io_step_empty": lambda: V(export_step(Part(), py_file("empty.step"))),
    "io_step_missing_dir": lambda: V(export_step(Box(1, 1, 1), py_file("no/dir/x.step"))),
    # ---- BREP
    "io_brep_text": brep_text,
    "io_brep_import": brep_import,
    "io_brep_missing_dir": lambda: V(export_brep(Box(1, 1, 1), py_file("no/dir/x.brep"))),
    "io_brep_import_missing": lambda: shape_tree(import_brep(py_file("missing.brep"))),
    # ---- STL
    "io_stl_ascii": stl_ascii,
    "io_stl_binary": stl_binary,
    "io_stl_import_units": stl_import_units,
    "io_stl_import_bad_unit": stl_import_bad_unit,
    "io_stl_missing_dir": lambda: V(export_stl(Box(1, 1, 1), py_file("no/dir/x.stl"))),
    # ---- OBJ
    "io_obj_uvs": obj_uvs,
    "io_obj_options": obj_options,
    "io_obj_sketch": obj_sketch,
    "io_obj_missing_dir": lambda: V(export_obj(Box(1, 1, 1), py_file("no/dir/x.obj"))),
    # ---- persistence
    "io_persist_shape_bytes": persist_shape_bytes,
    "io_persist_shape_to_py": lambda: Compound.cast(deserialize_shape(read_bytes(cpp_file("persist_shape.bin")))),
    "io_persist_shape_from_py": persist_shape_from_py,
    "io_persist_meshed_from_py": lambda: Compound.cast(deserialize_shape(read_bytes(fixture("persist_py_meshed.bin")))),
    "io_persist_location_bytes": lambda: V(
        [
            bytes(serialize_location(Location((1, 2, 3), (10, 20, 30)).wrapped)).hex(),
            bytes(serialize_location(Location((-1.5, 1e-3, 7), (0, 90, 45)).wrapped)).hex(),
            bytes(serialize_location(Location().wrapped)).hex(),
        ]
    ),
    "io_persist_location_from_py": persist_location_from_py,
    "io_persist_location_round_trip": persist_location_round_trip,
    # ---- project_to_viewport
    "io_hlr_box": lambda: viewport(Box(1, 2, 3).project_to_viewport((10, -10, 10))),
    "io_hlr_holed_part": hlr_holed_part,
    "io_hlr_look_at": hlr_look_at,
    "io_hlr_perspective": hlr_perspective,
    "io_hlr_solid": lambda: viewport(Solid.make_torus(10, 2).project_to_viewport((5, -10, 20))),
    "io_hlr_face": lambda: viewport(Face.make_rect(4, 2, Plane.XZ).project_to_viewport((1, -10, 3), (0, 0, 1))),
    "io_hlr_shell": lambda: viewport(
        Shell.extrude(Wire.make_circle(3), (0, 0, 4)).project_to_viewport((10, 10, 10))
    ),
    "io_hlr_edge": lambda: viewport(Edge.make_circle(5, Plane.XY, 0, 270).project_to_viewport((3, -20, 10))),
    "io_hlr_wire": lambda: viewport(
        Wire.make_polygon([(0, 0, 0), (5, 0, 0), (5, 5, 2), (0, 5, 0)]).project_to_viewport((-7, -12, 9))
    ),
    "io_hlr_sketch": lambda: viewport((Circle(5) - Rectangle(3, 3)).project_to_viewport((0, -10, 10))),
    "io_hlr_zero_focus": lambda: viewport(Box(2, 3, 4).project_to_viewport((10, 10, 10), (0, 0, 1), (1, 0, 0), 0.0)),
    "io_hlr_empty": lambda: viewport(Part().project_to_viewport((1, 1, 1))),
    # ---- GeomEncoder / JSON
    "io_json_encode": json_encode,
    "io_json_floats": json_floats,
    "io_json_dumps_each": json_dumps_each,
    "io_json_decode": json_decode,
    "io_json_hook_two_keys": json_hook_two_keys,
    "io_json_hook_unknown": json_hook_unknown,
    "io_json_roundtrip": json_roundtrip,
    "io_json_malformed": json_malformed,
}


# ---- 2D export: SVG
from build123d.exporters import ColorIndex, Drawing  # noqa: E402


def svg_text(e):
    stream = io.BytesIO()
    e.write(stream)
    return stream.getvalue().decode()


def svg_basic():
    e = ExportSVG()
    e.add_layer("hid", fill_color=None, line_color=(1, 0, 0), line_type=LineType.HIDDEN)
    s = Circle(5) - Rectangle(2, 2)
    s.color = Color(0.1, 0.5, 0.9, 0.5)
    e.add_shape(s)
    e.add_shape(Line((0, 0), (10, 5)), layer="hid")
    e.write(py_file("basic.svg"))
    return V(read_text(py_file("basic.svg")))


def svg_drawing():
    part = Box(10, 8, 4) - Pos(2, 0, 0) * Cylinder(2, 4) + Pos(-3, 0, 2) * Sphere(2)
    view = Drawing(part, look_from=(1, -2, 1.5))
    e = ExportSVG(unit=Unit.IN, scale=2, margin=5, fit_to_stroke=False, precision=3, line_weight=0.3)
    e.add_layer("hidden", line_color=Color("gray"), line_weight=0.2, line_type=LineType.ISO_DOT)
    e.add_shape(view.visible_lines)
    e.add_shape(view.hidden_lines, layer="hidden")
    return V(svg_text(e))


def svg_viewport():
    part = Cylinder(5, 10) - Pos(0, 0, 5) * Box(3, 12, 4)
    visible, hidden = part.project_to_viewport((20, -30, 25))
    e = ExportSVG(unit=Unit.CM, dot_length=DotLength.QCAD_IMPERIAL)
    e.add_layer("hidden", line_type=LineType.DASHDOT2)
    e.add_layer("other", line_type=LineType.DIVIDE)
    e.add_shape(visible)
    e.add_shape(hidden, layer="hidden")
    e.add_shape(Edge.make_line((0, 0), (1, 1)), layer="other")
    return V(svg_text(e))


def svg_curves():
    e = ExportSVG(precision=4, dot_length=0.05)
    e.add_layer("dots", line_type=LineType.DOT)
    e.add_shape(Edge.make_ellipse(5, 2, Plane.XY, 30, 250))
    e.add_shape(Edge.make_ellipse(3, 1, Plane.XY.rotated((0, 0, 25))))
    e.add_shape(Edge.make_circle(4, Plane.XY.offset(0).moved(Pos(10, 0)), 20, 300))
    e.add_shape(Edge.make_circle(2, Plane.XY.moved(Pos(-10, 0))))
    e.add_shape(Edge.make_circle(2, Plane.YX.moved(Pos(-10, 5)), 0, 120))
    e.add_shape(Edge.make_spline([(0, 0), (3, 4), (6, -1), (9, 2)]), layer="dots")
    e.add_shape(Edge.make_bezier((0, 5), (2, 9), (5, 5)), layer="dots")
    e.add_shape(Wire.make_polygon([(0, -5), (4, -5), (4, -8)], close=False), layer="dots")
    e.add_shape(Polyline((-2, -2), (-4, -6), (-1, -7)).edges()[0].reversed(), layer="dots")
    e.add_shape(Edge.make_helix(2, 4, 1).moved(Pos(15, 15)))
    return V(svg_text(e))


def svg_faces():
    e = ExportSVG(fill_color=Color("yellow"))
    e.add_layer("holes", fill_color=None)
    e.add_layer("wires", line_color=Color(0.0, 0.5, 0.0, 0.25))
    holed = Rectangle(20, 10) - GridLocations(5, 3, 3, 2) * Circle(1) - Pos(8, 0) * Ellipse(1.5, 1)
    holed.color = Color("blue")
    e.add_shape(holed, layer="holes")
    e.add_shape(Pos(30, 0) * (Circle(4) + Pos(3, 0) * Rectangle(4, 4)), reverse_wires=True)
    e.add_shape(Pos(0, 20) * SlotOverall(10, 3), layer="wires", reverse_wires=True)
    c = Pos(0, -20) * Curve(Wire.make_rect(6, 3).edges())
    c.color = Color("red")
    e.add_shape(c, layer="wires")
    e.add_shape(Pos(20, 20) * Text("b3d", 6))
    return V(svg_text(e))


def svg_colors():
    out = []
    for c in [Color("red"), Color(0.2, 0.4, 0.6, 0.123456), "#0080FF", (255, 128, 0), ColorIndex.GRAY,
              ColorIndex.BLACK, None]:
        e = ExportSVG(fill_color=c, line_color=c)
        s = Rectangle(1, 1)
        s.color = Color(1, 1, 1)
        e.add_shape(s)
        e.add_shape(Edge.make_line((0, 0), (1, 1)))
        out.append(svg_text(e))
    e = ExportSVG()
    s = Rectangle(1, 1)
    s.color = Color(1, 1, 1)
    e.add_shape(s)
    e.add_shape(Edge.make_line((0, 0), (1, 1)))
    out.append(svg_text(e))
    return V(out)


def svg_line_types():
    out = []
    for lt in [LineType.CONTINUOUS, LineType.BORDER, LineType.CENTERX2, LineType.DOT2, LineType.PHANTOM2,
               LineType.ISO_DASH, LineType.ISO_LONG_DASH_TRIPLE_DOT, LineType.ISO_DOUBLE_DASH_TRIPLE_DOT]:
        for dot in [0.0, 0.01, 0.2]:
            e = ExportSVG(unit=Unit.IN, scale=0.5, line_weight=0.25, line_type=lt, dot_length=dot)
            e.add_shape(Edge.make_line((0, 0), (1, 1)))
            out.append(svg_text(e))
    return V(out)


def svg_non_planar():
    e = ExportSVG()
    e.add_shape(Box(1, 2, 3))
    return V(svg_text(e))


def svg_bad_unit():
    ExportSVG(unit=Unit.M)
    return V("created")


def svg_duplicate_layer():
    e = ExportSVG()
    e.add_layer("a")
    e.add_layer("a")
    return V("added")


def svg_undefined_layer():
    e = ExportSVG()
    e.add_shape(Box(1, 1, 1), layer="nope")
    return V("added")


def svg_empty():
    return V(svg_text(ExportSVG()))


CASES.update(
    {
        "io_svg_basic": svg_basic,
        "io_svg_drawing": svg_drawing,
        "io_svg_viewport": svg_viewport,
        "io_svg_curves": svg_curves,
        "io_svg_faces": svg_faces,
        "io_svg_colors": svg_colors,
        "io_svg_line_types": svg_line_types,
        "io_svg_non_planar": svg_non_planar,
        "io_svg_bad_unit": svg_bad_unit,
        "io_svg_duplicate_layer": svg_duplicate_layer,
        "io_svg_undefined_layer": svg_undefined_layer,
        "io_svg_empty": svg_empty,
    }
)


# ---- 2D export: DXF (see cases_io.cpp)
import math  # noqa: E402

import ezdxf  # noqa: E402


def dxf_shapes(e, r12):
    e.add_shape(Edge.make_line((0, 0), (10, 5)), layer="L1")
    e.add_shape(Edge.make_circle(3))
    e.add_shape(Edge.make_circle(3, Plane.XY, 10, 100))
    e.add_shape(Edge.make_circle(3, Plane.XY.reverse(), 10, 100))
    e.add_shape(Pos(5, 5) * (Rectangle(4, 2) - Circle(0.5)), layer="L2")
    if r12:
        return
    e.add_shape(Edge.make_ellipse(4, 2))
    e.add_shape(Edge.make_ellipse(4, 2, Plane.XY.reverse(), 20, 200))
    e.add_shape(Edge.make_ellipse(3, 1, Plane.XY.rotated((0, 0, 30)).moved(Pos(-5, 2)), 0, 90))
    e.add_shape(Edge.make_spline([(0, 0), (1, 2), (3, 1), (5, 4)]), layer="L2")
    e.add_shape(Edge.make_spline([(0, 0), (1, 2), (3, 1)], periodic=True), layer="L2")
    e.add_shape(Edge.make_bezier((0, 0), (1, 3), (4, 0), weights=[1, 2, 1]), layer="L2")
    e.add_shape(Edge.make_bezier((0, -2), (2, -5), (4, -2)), layer="L2")


def dxf_description(path):
    """What ezdxf reads from a DXF file: header units, tables, view and entities."""
    doc = ezdxf.readfile(path)
    auditor = doc.audit()
    desc = {
        "version": doc.dxfversion,
        "audit_errors": len(auditor.errors),
        "insunits": doc.header.get("$INSUNITS"),
        "measurement": doc.header.get("$MEASUREMENT"),
        "layers": [
            [l.dxf.name, l.dxf.color, l.dxf.linetype, l.dxf.get("lineweight") if doc.dxfversion > "AC1009" else None]
            for l in doc.layers
        ],
        "linetypes": [
            [lt.dxf.name, lt.dxf.description, [float(v) for v in lt.simplified_line_pattern()]]
            for lt in doc.linetypes
        ],
    }
    vports = doc.viewports.get("*Active")
    vp = vports[0]
    desc["vport"] = [list(vp.dxf.center), vp.dxf.height]
    entities = []
    for e in doc.modelspace():
        t = e.dxftype()
        item = [t, e.dxf.layer]
        if t == "LINE":
            item += [list(e.dxf.start), list(e.dxf.end)]
        elif t == "CIRCLE":
            item += [list(e.dxf.center), e.dxf.radius]
        elif t == "ARC":
            item += [list(e.dxf.center), e.dxf.radius, e.dxf.start_angle, e.dxf.end_angle]
        elif t == "ELLIPSE":
            item += [list(e.dxf.center), list(e.dxf.major_axis), e.dxf.ratio, e.dxf.start_param, e.dxf.end_param]
        elif t == "SPLINE":
            item += [e.dxf.degree, e.dxf.flags, list(e.knots), list(e.weights), [list(p) for p in e.control_points]]
        entities.append(item)
    desc["entities"] = entities
    return desc


def json_diff(a, b, path="", rel=1e-9, abs_tol=1e-9):
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            return [] if math.isclose(a, b, rel_tol=rel, abs_tol=abs_tol) else [f"{path}: {a!r} != {b!r}"]
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            return [f"{path}: length {len(a)} != {len(b)}"]
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += json_diff(x, y, f"{path}[{i}]", rel, abs_tol)
        return out
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in a:
            out += json_diff(a[k], b.get(k), f"{path}.{k}", rel, abs_tol)
        return out
    return [] if a == b else [f"{path}: {a!r} != {b!r}"]


def dxf_diff(name, python_path):
    """Compare the C++ file `name` with Python's file; "same" if ezdxf reads the same."""
    cpp = dxf_description(cpp_file(name))
    py = dxf_description(python_path)
    problems = json_diff(cpp, py)
    return [name, "same" if not problems else "; ".join(problems[:8])]


def dxf_version(v):
    def case():
        e = ExportDXF(v, Unit.MM, ColorIndex.BLUE, 0.35, LineType.CENTER)
        e.add_layer("L1", color=ColorIndex.RED, line_weight=0.5, line_type=LineType.DASHED)
        e.add_layer("L2")
        dxf_shapes(e, v == "R12")
        name = f"version_{v}.dxf"
        e.write(py_file(name))
        return V(dxf_diff(name, py_file(name)))

    return case


def dxf_units():
    out = []
    for u in [Unit.MC, Unit.MM, Unit.CM, Unit.M, Unit.IN, Unit.FT]:
        e = ExportDXF(ezdxf.DXF2013, u, None, None, LineType.ISO_DASH_DOT)
        e.add_layer("dots", line_type=LineType.DOT2)
        e.add_shape(Edge.make_line((0, 0), (1, 1)), layer="dots")
        name = f"unit_{list(Unit).index(u)}.dxf"
        e.write(py_file(name))
        out.append(dxf_diff(name, py_file(name)))
    return V(out)


def dxf_binary():
    out = []
    for v in ["R12", "R2004", "R2013"]:
        e = ExportDXF(v)
        e.add_layer("L1", color=ColorIndex.GREEN, line_type=LineType.PHANTOM)
        e.add_layer("L2", line_weight=0.13)
        dxf_shapes(e, v == "R12")
        name = f"binary_{v}.dxf"
        e.write(py_file(name), ascii_format=False)
        out.append(dxf_diff(name, py_file(name)))
    return V(out)


def dxf_text_hlr():
    e = ExportDXF(ezdxf.DXF2018, Unit.IN)
    e.add_layer("hidden", line_type=LineType.HIDDEN)
    part = Box(10, 8, 4) - Pos(2, 0, 0) * Cylinder(2, 4) + Pos(-3, 0, 2) * Sphere(2)
    visible, hidden = part.project_to_viewport((20, -30, 25))
    e.add_shape(visible)
    e.add_shape(hidden, layer="hidden")
    e.add_shape(Pos(0, 20) * Text("b3d", 6))
    e.write(py_file("text_hlr.dxf"))
    return V(dxf_diff("text_hlr.dxf", py_file("text_hlr.dxf")))


def dxf_non_planar():
    e = ExportDXF()
    e.add_shape(Box(1, 2, 3))
    e.write(py_file("non_planar.dxf"))
    return V(dxf_diff("non_planar.dxf", py_file("non_planar.dxf")))


def dxf_bad_unit():
    ExportDXF(ezdxf.DXF2013, Unit.KG)
    return V("created")


def dxf_r12_ellipse():
    e = ExportDXF("R12")
    e.add_shape(Edge.make_ellipse(2, 1))
    return V("added")


def dxf_duplicate_layer():
    e = ExportDXF()
    e.add_layer("A")
    e.add_layer("a")
    return V("added")


def dxf_bad_version():
    ExportDXF("R14")
    return V("created")


CASES.update({f"io_dxf_version_{v}": dxf_version(v) for v in ["R12", "R2000", "R2004", "R2007", "R2010", "R2013", "R2018"]})
CASES.update(
    {
        "io_dxf_units": dxf_units,
        "io_dxf_binary": dxf_binary,
        "io_dxf_text_hlr": dxf_text_hlr,
        "io_dxf_non_planar": dxf_non_planar,
        "io_dxf_bad_unit": dxf_bad_unit,
        "io_dxf_r12_ellipse": dxf_r12_ellipse,
        "io_dxf_duplicate_layer": dxf_duplicate_layer,
        "io_dxf_bad_version": dxf_bad_version,
    }
)


# ---- SVG import (see cases_io.cpp)
def svg_shapes(shapes):
    out = []
    for s in shapes:
        bb = s.bounding_box()
        extra = s.normal_at() if isinstance(s, Face) else [s.start_point(), s.end_point()]
        out.append(
            {
                "type": type(s).__name__,
                "label": s.label,
                "color": color_value(s.color),
                "area": s.area,
                "length": sum(e.length for e in s.edges()),
                "edges": [e.geom_type.name for e in s.edges()],
                "bbox": [bb.min, bb.max],
                "position": s.location.position,
                "normal_or_ends": extra,
            }
        )
    return V(out)


def svg_case(name, **kwargs):
    return lambda: svg_shapes(import_svg(fixture(name + ".svg"), **kwargs))


def svg_import_stream():
    with open(fixture("svg_inkscape.svg")) as f:
        return svg_shapes(import_svg(f, label_by="tag"))


for _n in ["svg_basic", "svg_inkscape", "svg_holes", "svg_edge"]:
    CASES[f"io_{_n}_default"] = svg_case(_n)
    CASES[f"io_{_n}_label_noalign"] = svg_case(_n, align=None, label_by="inkscape:label")
    CASES[f"io_{_n}_noflip_center"] = svg_case(_n, flip_y=False, align=Align.CENTER, ignore_visibility=True)
CASES["io_svg_import_align_pair"] = lambda: svg_shapes(
    import_svg(fixture("svg_basic.svg"), align=(Align.MAX, Align.CENTER), label_by="class")
)
CASES["io_svg_import_stream"] = svg_import_stream
CASES["io_svg_import_missing"] = lambda: svg_shapes(import_svg(cpp_file("missing.svg")))
CASES["io_svg_export_roundtrip"] = lambda: svg_shapes(import_svg(fixture("svg_export.svg"), label_by="id"))
CASES["io_svg_export_roundtrip_layers"] = lambda: svg_shapes(
    import_svg(fixture("svg_export.svg"), flip_y=False, label_by="stroke-dasharray")
)


def svg_buildline(name, *args):
    return lambda: V(list(import_svg_as_buildline_code(fixture(name + ".svg"), *args)))


for _n in ["svg_basic", "svg_inkscape", "svg_holes", "svg_edge", "svg_export"]:
    CASES[f"io_svg_buildline_{_n}"] = svg_buildline(_n)
CASES["io_svg_buildline_precision"] = svg_buildline("svg_basic", 3)
