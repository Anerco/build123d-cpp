"""Python twins of cases_core.cpp."""

from build123d import *  # noqa: F403

from values import V


def rnd(v, digits=4):
    r = round(v, digits)
    return 0.0 if r == 0 else r


def measure(s):
    if isinstance(s, (Edge, Wire)):
        return s.length
    if isinstance(s, (Face, Shell)):
        return s.area
    if isinstance(s, (Solid, Compound)):
        return s.volume
    return 0.0


def describe(shapes):
    """A list of shapes as sorted [type, center, measure] rows."""
    if shapes is None:
        return V([])
    rows = []
    for s in shapes:
        c = s.center()
        rows.append((type(s).__name__, rnd(c.X), rnd(c.Y), rnd(c.Z), rnd(measure(s))))
    return V([list(r) for r in sorted(rows)])


def vec(v):
    return [rnd(v.X, 6), rnd(v.Y, 6), rnd(v.Z, 6)]


def geom(r):
    if r is None:
        return None
    if isinstance(r, Vector):
        return {"kind": "Vector", "value": vec(r)}
    if isinstance(r, Axis):
        return {"kind": "Axis", "position": vec(r.position), "direction": vec(r.direction)}
    if isinstance(r, Plane):
        return {"kind": "Plane", "origin": vec(r.origin), "x_dir": vec(r.x_dir), "z_dir": vec(r.z_dir)}
    if isinstance(r, Location):
        return {"kind": "Location", "position": vec(r.position), "orientation": vec(r.orientation)}
    raise TypeError(type(r))


def vertex_arithmetic():
    a, b = Vertex(1, 2, 3), Vertex(4, 5, 6)
    c = a
    c += (0, 0, 15)
    return V([a + b, a + Vector(0.5, 0.5, 0.5), b - a, b - Vector(1, 1, 1), a + (1, 2), c, isinstance(a + b, Vertex)])


def curve_curve():
    c = Curve() + [Line((-2, 0), (2, 0)), Line((2, 0), (2, 3))]
    return c & Curve([CenterArc((0, 0), 1.5, 0, 360)])


def intersect_vertex():
    v = Vertex(1, 0, 0)
    return V([
        describe(v.intersect(Vertex(1, 0, 0))), describe(v.intersect(Vector(1, 0, 1))),
        describe(v.intersect(Axis.X)), describe(v.intersect(Plane.XZ)),
        describe(v.intersect(Box(4, 4, 4))), describe(Box(4, 4, 4).intersect(v)),
    ])


def intersect_edge_vertex():
    e = Edge.make_line((0, 0), (2, 0))
    return V([describe(e.intersect(Vector(1, 0, 0))), describe(e.intersect(Location((3, 0, 0)))),
              describe(Vertex(1, 0, 0).intersect(e))])


def intersect_tolerance():
    e = Edge.make_line((0, 0), (2, 0))
    return V([describe(e.intersect(Vertex(1, 1e-4, 0))), describe(e.intersect(Vertex(1, 1e-4, 0), tolerance=1e-3))])


def intersect_touched():
    a = Box(2, 2, 2)
    b = Pos(2, 0, 0) * Box(2, 2, 2)
    c = Pos(2, 2, 0) * Box(2, 2, 2)
    return V([describe(a.intersect(b)), describe(a.intersect(b, include_touched=True)),
              describe(a.intersect(c, include_touched=True))])


def touch():
    a = Box(2, 2, 2)
    f1 = Face.make_rect(2, 2)
    f2 = Face.make_rect(2, 2, Plane((2, 2, 0)))
    return V([describe(a.touch(Pos(2, 0, 0) * Box(2, 2, 2))), describe(a.touch(Pos(2, 2, 2) * Box(2, 2, 2))),
              describe(f1.touch(f2)), describe(a.touch(Edge.make_line((1, -2, 0), (1, 2, 0)))),
              describe(a.solids()[0].touch(Vertex(1, 1, 1)))])


def face_face_touched():
    f1 = Face.make_rect(2, 2)
    f2 = Face.make_rect(2, 2, Plane((2, 2, 0)))
    f3 = Face.make_rect(2, 2, Plane((1, 0, 0)))
    return V([describe(f1.intersect(f2)), describe(f1.intersect(f2, include_touched=True)), describe(f1.intersect(f3))])


def compound_children():
    assy = Compound(children=[Pos(-3, 0, 0) * Box(2, 2, 2), Pos(3, 0, 0) * Sphere(1)])
    return describe(assy.intersect(Axis.X))


def geom_axis_axis():
    a = Axis((0, 1, 0), (1, 0, 0))
    return V([geom(a.intersect(Axis((3, 0, 0), (0, 1, 0)))), geom(a.intersect(Axis((5, 1, 0), (-1, 0, 0)))),
              geom(a.intersect(Axis((0, 2, 0), (1, 0, 0)))), geom(a.intersect(Axis((0, 0, 1), (0, 1, 0)))),
              geom(a & Axis((2, 0, 0), (0, 1, 0)))])


def geom_axis_plane():
    return V([geom(Axis.X.intersect(Plane.XY)), geom(Axis((0, 0, 5), (0, 0, -1)).intersect(Plane.XY)),
              geom(Plane.XY.intersect(Axis((0, 0, 5), (1, 0, 0)))), geom(Plane.XZ & Axis((1, 2, 3), (0, 1, 0)))])


def geom_plane_plane():
    return V([geom(Plane.XY.intersect(Plane.XY.offset(1))), geom(Plane.XY.intersect(Plane.XZ)),
              geom(Plane.XY.intersect(Plane((1, 2, 0)))), geom(Plane.XY & Plane.YZ),
              geom(Plane.XY.intersect(Plane.XY.reverse()))])


def geom_location():
    l = Location((1, 2, 3), (0, 0, 45))
    return V([geom(l.intersect(Vector(1, 2, 3))), geom(l.intersect(Vector(1, 2, 4))),
              geom(l.intersect(Location((1, 2, 3), (0, 0, 45)))), geom(l.intersect(Location((1, 2, 3), (10, 0, 0)))),
              geom(l.intersect(Location((0, 0, 0)))), geom(l.intersect(Axis((0, 2, 3), (1, 0, 0)))),
              geom(l.intersect(Axis((1, 2, 0), (0, 0, 1)))), geom(l.intersect(Plane.XY.offset(3))),
              geom(l.intersect(Plane.XZ.offset(-2))), geom(Axis((1, 2, 0), (0, 0, 1)).intersect(l)),
              geom(Plane.XY.offset(3) & l), geom(l & Plane.XY)])


def geom_vector():
    v = Vector(1, 0, 0)
    return V([geom(v.intersect(Vector(1, 0, 0))), geom(v.intersect(Vector(1, 0, 1e-7))),
              geom(v.intersect(Vector(2, 0, 0))), geom(v.intersect(Axis.X)), geom(v.intersect(Axis.Y)),
              geom(v.intersect(Plane.XY)), geom(v.intersect(Plane.YZ)), geom(v.intersect(Location(v))),
              geom(v & Plane.XZ), geom(v & Axis.X)])


def geom_with_shape():
    b = Box(2, 2, 2)
    return V([describe(Vector(0, 0, 0) & b), describe(Axis.Z & b), describe(Plane.XY & b),
              describe(Location((0.5, 0, 0)) & b), describe(Axis.Z.intersect(b)), describe(Plane.YZ.intersect(b))])


CASES = {
    # 1a
    "core_and_list_part": lambda: Box(4, 4, 4) & [Pos(1, 0, 0) * Box(4, 4, 4), Sphere(2.5)],
    "core_and_list_sketch": lambda: Rectangle(4, 4) & [Pos(1, 0) * Rectangle(4, 4), Circle(2.2)],
    "core_and_chain_equivalent": lambda: (Box(4, 4, 4) & (Pos(1, 0, 0) * Box(4, 4, 4))) & Sphere(2.5),
    "core_and_compound_or": lambda: Box(4, 4, 4)
    & Compound([Pos(-2, 0, 0) * Box(1, 1, 1), Pos(2, 0, 0) * Box(1, 1, 1)]),
    "core_intersect_chain_list": lambda: describe(Box(4, 4, 4).intersect(Pos(1, 0, 0) * Box(4, 4, 4), Sphere(2.5))),
    # 1b
    "core_vertex_arithmetic": vertex_arithmetic,
    # 2: shapes
    "core_curve_and": lambda: Line((0, 0), (1, 1)) & Line((0.25, 0.25), (0.75, 0.75)),
    "core_line_cross": lambda: Line((0, 0), (2, 2)) & Line((0, 2), (2, 0)),
    "core_curve_curve": curve_curve,
    "core_edge_face_cross": lambda: Line((0, 0, -1), (0, 0, 1)) & Rectangle(2, 2),
    "core_edge_face_inplane": lambda: Line((-3, 0), (3, 0)) & Circle(1),
    "core_face_edge": lambda: Circle(1) & Line((-3, 0.5), (3, 0.5)),
    "core_part_curve": lambda: Box(2, 2, 2) & Line((-3, 0, 0), (3, 0, 0)),
    "core_curve_part": lambda: Curve([Spline((-3, 0, 0), (0, 0.5, 0.2), (3, 0, 0))]) & Sphere(2),
    "core_sketch_part": lambda: (Plane.XZ * Rectangle(6, 6)) & Box(2, 3, 4),
    "core_part_sketch": lambda: Box(2, 3, 4) & (Plane.XZ * Circle(3)),
    "core_face_face_cross": lambda: Rectangle(2, 2) & (Plane.XZ * Rectangle(1, 1)),
    "core_sketch_sketch_coplanar": lambda: Rectangle(20, 10) & Circle(7),
    "core_part_axis": lambda: Cylinder(2, 6) & Axis.Z,
    "core_part_axis_offset": lambda: Box(2, 2, 2) & Axis((0.5, 0, 0), (1, 1, 0)),
    "core_part_plane": lambda: Box(2, 3, 4) & Plane.XY.offset(1),
    "core_part_vector": lambda: Box(2, 2, 2) & Vector(0.5, 0.5, 0.5),
    "core_sketch_location": lambda: Rectangle(2, 2) & Location((0.5, 0.25, 0)),
    "core_curve_plane": lambda: Curve([Helix(2, 6, 3)]) & Plane.XY.offset(3),
    "core_curve_axis": lambda: Curve([Line((-1, -1), (1, 1))]) & Axis.X,
    "core_intersect_vertex": intersect_vertex,
    "core_intersect_edge_vertex": intersect_edge_vertex,
    "core_intersect_tolerance": intersect_tolerance,
    "core_intersect_touched": intersect_touched,
    "core_touch": touch,
    "core_intersect_face_face_touched": face_face_touched,
    "core_intersect_compound_children": compound_children,
    # 2: geometry
    "core_geom_axis_axis": geom_axis_axis,
    "core_geom_axis_plane": geom_axis_plane,
    "core_geom_plane_plane": geom_plane_plane,
    "core_geom_location": geom_location,
    "core_geom_vector": geom_vector,
    "core_geom_with_shape": geom_with_shape,
}
