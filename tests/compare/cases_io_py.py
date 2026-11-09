"""Python twins of cases_io.cpp."""

from build123d import *  # noqa: F403

from values import V


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


CASES = {
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
}
