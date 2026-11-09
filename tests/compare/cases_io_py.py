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


CASES = {
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
