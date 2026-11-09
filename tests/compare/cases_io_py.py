"""Python twins of cases_io.cpp."""

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


CASES = {
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
