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


def select_fused_sketch():
    cross = Rectangle(8, 2)
    cross += Rectangle(2, 8)
    return V([len(cross.vertices(Select.NEW)), len(cross.vertices(Select.LAST)), len(cross.edges(Select.NEW)),
              len(cross.edges(Select.LAST)), describe(cross.vertices(Select.NEW))])


def select_fused_part():
    part = Box(10, 10, 10) + Pos(Z=5) * Cylinder(2, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return V([describe(part.faces(Select.LAST)), len(part.solids(Select.NEW))])


def select_box_plus_cylinder():
    part = Box(10, 10, 10) + Cylinder(2, 20)
    return V([describe(part.edges(Select.NEW)), describe(part.edges(Select.LAST)),
              describe(part.faces(Select.NEW)), describe(part.faces(Select.LAST))])


def select_cut_and():
    holed = Box(10, 10, 10) - Cylinder(1, 20)
    bar = Box(10, 10, 10) & Box(30, 2, 2)
    return V([describe(holed.faces(Select.LAST)), len(holed.edges(Select.NEW)), len(bar.faces(Select.LAST)),
              len(bar.faces(Select.NEW))])


def select_chained():
    three = (Box(10, 10, 10) + Pos(Z=5) * Cylinder(1, 5, align=(Align.CENTER, Align.CENTER, Align.MIN))
             + Pos(X=5) * Sphere(1))
    return describe(three.faces(Select.LAST))


def select_multi_piece_self():
    two = Rectangle(2, 2) + Pos(X=5) * Rectangle(2, 2)
    plus = two + Pos(X=5) * Rectangle(1, 4)
    return V([len(plus.faces()), len(plus.faces(Select.LAST))])


def select_fillet_chamfer():
    rounded = fillet(Box(10, 10, 10).edges().filter_by(Axis.Z), 1)
    bevelled = chamfer(Box(10, 10, 10).edges().group_by(Axis.Z)[-1], 1)
    return V([len(rounded.faces(Select.LAST)), len(rounded.edges(Select.NEW)), len(bevelled.faces(Select.NEW)),
              len(bevelled.faces(Select.LAST)), len(bevelled.edges(Select.NEW))])


def select_joined_lines():
    bent = Line((0, 0), (1, 0)) + Line((1, 0), (1, 1))
    straight = Line((0, 0), (1, 0)) + Line((1, 0), (2, 0))
    chain = Curve() + [Line((0, 0), (1, 0)), Line((1, 0), (1, 1)), Line((1, 1), (0, 1))]
    return V([len(bent.edges()), len(bent.edges(Select.LAST)), len(bent.edges(Select.NEW)),
              len(bent.vertices(Select.LAST)), len(bent.vertices(Select.NEW)), len(straight.edges(Select.LAST)),
              len(straight.vertices(Select.NEW)), len(chain.edges(Select.LAST))])


def select_no_record():
    try:
        Rectangle(8, 2).vertices(Select.NEW)
        raised = False
    except ValueError:
        raised = True
    return V([raised, len(Rectangle(8, 2).vertices(Select.ALL))])


def select_2d_fillet_chamfer():
    rect = Rectangle(10, 6)
    rounded = fillet(rect.vertices(), 1)
    bevelled = chamfer(rect.vertices().group_by(Axis.X)[-1], 1)
    return V([len(rounded.edges(Select.NEW)), len(rounded.edges(Select.LAST)), len(bevelled.edges(Select.NEW)),
              len(bevelled.vertices(Select.LAST))])


def select_draft_loft_clean():
    box = Box(10, 10, 10)
    drafted = draft(box.faces().filter_by(Axis.X), Plane.XY.offset(-5), 5)
    lofted = loft([Rectangle(4, 4), Pos(0, 0, 5) * Circle(1)])
    cleaned = (Box(2, 2, 2) + Pos(2, 0, 0) * Box(2, 2, 2)).clean()
    return V([len(drafted.faces(Select.NEW)), len(drafted.faces(Select.LAST)), len(lofted.faces(Select.NEW)),
              len(lofted.edges(Select.LAST)), len(cleaned.faces(Select.NEW)), len(cleaned.faces(Select.LAST))])


def select_curve_fillet():
    poly = Curve([Polyline((0, 0), (10, 0), (10, 10), (0, 10))])
    rounded = fillet(poly.vertices(), 2)
    return V([len(rounded.edges(Select.NEW)), len(rounded.edges(Select.LAST)), len(rounded.edges()),
              describe(rounded.edges())])


def core_new_edges():
    box = Box(10, 10, 10)
    cyl = Cylinder(2, 20)
    both = box + cyl
    return describe(new_edges(box, cyl, combined=both))


def conv_names(shapes):
    out = []
    for s in shapes:
        try:
            out.append(s.convexity.name)
        except ValueError:
            out.append("error")
    return out


def convexity_edges():
    pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4)
    step = Box(10, 10, 2) + Pos(2.5, 0, 2) * Box(5, 10, 2)
    rounded = fillet(Box(10, 10, 10).edges().group_by(Axis.Z)[-1], 1)
    tube = Cylinder(5, 10) - Cylinder(2, 10)
    return V([conv_names(Box(1, 1, 1).edges()), conv_names(pocket.edges()), conv_names(step.edges()),
              conv_names(rounded.edges()), conv_names(tube.edges()), conv_names(Rectangle(2, 2).edges())])


def convexity_faces_vertices():
    tube = Cylinder(5, 10) - Cylinder(2, 10)
    step = Box(10, 10, 2) + Pos(2.5, 0, 2) * Box(5, 10, 2)
    ell = Rectangle(4, 4) - Pos(1, 1) * Rectangle(2, 2)
    return V([conv_names(tube.faces()), conv_names(Torus(5, 1).faces()), conv_names(Sphere(2).faces()),
              conv_names(step.vertices()), conv_names(ell.faces()[0].vertices()),
              conv_names((Box(2, 2, 2) - Sphere(0.8)).faces())])


def convexity_selectors():
    pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4)
    groups = pocket.edges().group_by(Convexity)
    sizes = [[k.value - 1, len(groups.group(k))] for k, _ in groups.key_to_group_index]
    concave = pocket.edges().filter_by(Convexity.CONCAVE)
    not_concave = pocket.edges().filter_by(Convexity.CONCAVE, reverse=True)
    tube = Cylinder(5, 10) - Cylinder(2, 10)
    flags = [[f.is_circular_convex, f.is_circular_concave] for f in tube.faces()]
    return V([sizes, len(concave), len(not_concave), flags, [e.is_interior for e in concave]])


def topo_explore():
    box = Box(10, 10, 10)
    top = box.edges().group_by(Axis.Z)[-1].sort_by(Axis.X)[0]
    rounded = fillet(Box(10, 10, 10).edges().filter_by(Axis.Z), 2)
    vertical = rounded.edges().filter_by(Axis.Z).sort_by(Axis.X)[0]
    c0 = topo_explore_connected_edges(vertical)
    c1 = topo_explore_connected_edges(vertical, continuity=ContinuityLevel.C1)
    c2 = topo_explore_connected_edges(vertical, rounded, ContinuityLevel.C2)
    edges = box.edges()
    return V([describe(topo_explore_connected_edges(top)), describe(c0), describe(c1), describe(c2),
              topo_explore_common_vertex(edges[0], edges[1]), topo_explore_common_vertex(edges[0], edges[2])])


def topo_distance():
    box = Box(10, 10, 10)
    top = box.faces().sort_by(Axis.Z)[-1]
    key = topo_distance_to(top)
    faces = box.faces().sort_by(key)
    edge_key = topo_distance_to(box.edges().sort_by(Axis.Z)[0])
    vertex_key = topo_distance_to(box.vertices().sort_by(Axis.Z)[0])
    groups = box.faces().group_by(topo_distance_to([top]))
    return V([[key(f) for f in faces], [edge_key(e) for e in box.edges().sort_by(edge_key)],
              [vertex_key(v) for v in box.vertices()], len(groups)])


def utils_case():
    a = Edge.make_line((0, 0), (1, 0))
    b = Edge.make_line((1, 0), (1, 1))
    c = Edge.make_line((5, 5), (6, 5))
    wires = edges_to_wires([a, b, c])
    d = delta([a, b, a.reversed()], [a])
    return V([Vector(polar(2, 30)), len(wires), describe(wires), len(d)])


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
    # 3: history
    "core_select_fused_sketch": select_fused_sketch,
    "core_select_fused_part": select_fused_part,
    "core_select_box_plus_cylinder": select_box_plus_cylinder,
    "core_select_cut_and": select_cut_and,
    "core_select_chained": select_chained,
    "core_select_multi_piece_self": select_multi_piece_self,
    "core_select_fillet_chamfer": select_fillet_chamfer,
    "core_select_joined_lines": select_joined_lines,
    "core_select_no_record": select_no_record,
    "core_select_2d_fillet_chamfer": select_2d_fillet_chamfer,
    "core_select_draft_loft_clean": select_draft_loft_clean,
    "core_select_curve_fillet": select_curve_fillet,
    "core_new_edges": core_new_edges,
    # 4: convexity
    "core_convexity_edges": convexity_edges,
    "core_convexity_faces_vertices": convexity_faces_vertices,
    "core_convexity_selectors": convexity_selectors,
    # 5: topology helpers
    "core_topo_explore": topo_explore,
    "core_topo_distance": topo_distance,
    "core_utils": utils_case,
}
