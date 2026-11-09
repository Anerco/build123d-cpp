"""Python twins of cases_core.cpp."""

from build123d import *  # noqa: F403
from build123d.topology import Shape

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


def touch_found():
    a = Box(2, 2, 2).solids()[0]
    overlapping = (Pos(1, 0, 0) * Box(2, 2, 2)).solids()[0]
    adjacent = (Pos(2, 0, 0) * Box(2, 2, 2)).solids()[0]
    found = a.intersect(overlapping)
    f1 = Face.make_rect(2, 2)
    f2 = Face.make_rect(2, 2, Plane((2, 2, 0)))
    f3 = Face.make_rect(2, 2, Plane((1, 0, 0)))
    found_faces = f1.intersect(f3)
    f1_edges = ShapeList(f1.edges())
    return V([describe(a.touch(overlapping)), describe(a.touch(overlapping, found_solids=found)),
              describe(a.touch(overlapping, found_solids=ShapeList())),
              describe(a.touch(adjacent, found_solids=ShapeList())),
              describe(f1.touch(f3)), describe(f1.touch(f3, found_faces=ShapeList())),
              describe(f1.touch(f3, found_faces=found_faces)),
              describe(f1.touch(f2, found_faces=ShapeList(), found_edges=ShapeList())),
              describe(f1.touch(f3, found_faces=ShapeList(), found_edges=f1_edges)),
              describe(f1.touch(f3, found_edges=f1_edges))])


def topology_utils_more():
    from build123d.topology import (  # pylint: disable=import-outside-toplevel
        find_max_dimension,
        isclose_b,
        offset_topods_face,
        topo_explore_connected_faces,
        unwrap_topods_compound,
    )

    def raises(f):
        try:
            f()
        except ValueError:
            return True
        return False

    box = Box(1, 2, 3)
    edge = box.edges().sort_by(Axis.Z)[0]
    connected = [Face(f) for f in topo_explore_connected_faces(edge)]
    with_parent = [Face(f) for f in topo_explore_connected_faces(edge, box)]
    nested = Compound([Compound([box])])
    fully = Shape.cast(unwrap_topods_compound(nested.wrapped))
    partly = Shape.cast(unwrap_topods_compound(nested.wrapped, False))
    two = Compound([box, Pos(5, 0, 0) * Sphere(1)])
    both = Shape.cast(unwrap_topods_compound(two.wrapped))
    offset = Shape.cast(offset_topods_face(Face.make_rect(2, 2).wrapped, 0.5))
    inf, nan = float("inf"), float("nan")
    return V([describe(connected), describe(with_parent),
              raises(lambda: topo_explore_connected_faces(Edge())),
              raises(lambda: topo_explore_connected_faces(Edge.make_line((0, 0), (1, 0)))),
              type(fully).__name__, type(partly).__name__, len(partly.solids()),
              type(both).__name__, describe([offset]), type(offset).__name__,
              find_max_dimension(box), find_max_dimension([box, Pos(5, 0, 0) * Sphere(1)]),
              [isclose_b(1, 1 + 1e-10), isclose_b(0, 1e-15), isclose_b(0, 1e-13), isclose_b(1, 1.1),
               isclose_b(inf, inf), isclose_b(nan, nan), isclose_b(1, 1.05, rel_tol=0.1)],
              raises(lambda: isclose_b(1, 1, rel_tol=-1))])


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


def mass_inertia():
    box = Pos(1, 2, 3) * Box(2, 4, 6)
    rows = box.matrix_of_inertia
    moments = [m for _, m in Cylinder(2, 5).principal_properties]
    sm = box.static_moments
    return V([rows, moments, list(sm), box.radius_of_gyration(Axis.X), Sphere(2).radius_of_gyration(Axis.Z),
              Edge.make_line((0, 0), (3, 4)).compute_volume(), Face.make_rect(2, 3).compute_volume(),
              Vertex(1, 2, 3).compute_volume(), box.compute_volume()])


def mass_material():
    box = Box(10, 10, 10)
    box.material = "aluminum"
    cyl = Cylinder(5, 10)
    cyl.material = "Mild Steel"
    assy_child = Pos(20, 0, 0) * Box(1, 1, 1)
    assy = Compound(children=[box, assy_child])
    assy.material = "copper"
    shell = Box(2, 2, 2).shells()[0]
    shell.material = "abs"
    none = Box(1, 1, 1)
    try:
        none.mass()
        missing = False
    except ValueError:
        missing = True
    import bd_materials
    return V([box.mass(), box.mass(Unit.KG), box.mass(Unit.LB, Unit.MM), box.mass(Unit.G, Unit.CM),
              box.compute_mass(Unit.KG, Unit.IN), cyl.mass(), assy.mass(), shell.mass(), Rectangle(2, 2).mass(),
              Edge.make_line((0, 0), (1, 0)).mass(), missing, box.material.material.density, box.material.name,
              len(bd_materials.material_names())])


def combined_center():
    objects = [Box(2, 2, 2), Pos(10, 0, 0) * Sphere(1), Pos(0, 5, 0) * Cylinder(1, 4)]
    try:
        Shape.combined_center(objects, CenterOf.GEOMETRY)
        geometry_error = False
    except ValueError:
        geometry_error = True
    return V([Shape.combined_center(objects), Shape.combined_center(objects, CenterOf.BOUNDING_BOX), geometry_error,
              Shape.combined_center([Edge.make_line((0, 0), (2, 0)), Edge.make_line((0, 1), (0, 5))])])


def matrix_case():
    m = Matrix([[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3]])
    m.rotate(Axis.Z, 90)
    inv = m.inverse()
    prod = m.multiply(inv)
    values = [m[r, c] for r in range(4) for c in range(4)]
    try:
        Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        bad_size = False
    except (TypeError, ValueError):
        bad_size = True
    try:
        Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 1, 1]])
        bad_row = False
    except (TypeError, ValueError):
        bad_row = True
    return V([values, m.transposed_list(), m.multiply(Vector(1, 0, 0)), prod[0, 0], prod[0, 3],
              Vector(1, 2, 3).transform(m), Vector(1, 2, 3).transform(m, True), bad_size, bad_row,
              round(Vector(1.23456, -2.5, 3.5), 2), round(Vector(1.5, 2.5, -0.5))])


def transform_geometry_case():
    skew = Matrix([[1, 0.5, 0, 0], [0, 1, 0, 0], [0, 0, 2, 1]])
    return Box(2, 2, 2).transform_geometry(skew)


def transform_shape_matrix():
    m = Matrix()
    m.rotate(Axis.X, 30)
    return Cylinder(1, 3).transform_shape(m)


def oriented_bound_box():
    part = Rot(10, 20, 30) * Box(2, 4, 6)
    obb = part.oriented_bounding_box()
    face_obb = Face.make_rect(2, 3).oriented_bounding_box()
    line_obb = Edge.make_line((0, 0, 0), (3, 4, 0)).oriented_bounding_box()
    small = (Rot(10, 20, 30) * Box(1, 1, 1)).oriented_bounding_box()
    box = Solid.from_bounding_box(obb)
    return V([sorted(obb.size), obb.diagonal, obb.center(), len(obb.corners), len(face_obb.corners),
              len(line_obb.corners), line_obb.size.length, obb.is_completely_inside(small),
              small.is_completely_inside(obb), obb.is_outside(Vector(0, 0, 0)), obb.is_outside(Vector(10, 10, 10)),
              box.volume, box.center()])


def hash_sets():
    box = Box(1, 2, 3)
    edges = set(box.edges()) | {e.reversed() for e in box.edges()}
    faces = set(box.faces()) | set(box.faces())
    vectors = {Vector(1, 2, 3), Vector(1.000001, 2, 3), Vector(1.0001, 2, 3)}
    locations = {Location((1, 2, 3)), Location((1, 2, 3), (0, 0, 360)), Pos(1, 2, 3), Location((1, 2, 3), (0, 0, 90))}
    planes = {Plane.XY, Plane.XY.offset(0), Plane.XZ}
    axes = {Axis.X, Axis((0, 0, 0), (2, 0, 0)), Axis.Y}
    e = box.edges()[0]
    return V([len(edges), len(faces), len(vectors), len(locations), len(planes), len(axes),
              hash(e) == hash(e.reversed()), e == e.reversed(), hash(box) == hash(box.moved(Location())),
              hash(box) == hash(box.moved(Pos(1, 0, 0))), box == box.moved(Pos(1, 0, 0)), hash(Shape()) == 0])


def to_splines_types():
    types = ["GeomType." + f.geom_type.name for f in Cylinder(1, 2).to_splines().faces()]
    edge_types = ["GeomType." + e.geom_type.name for e in Edge.make_circle(1).to_splines(nurbs=True).edges()]
    try:
        Solid().to_splines()
        empty_error = False
    except ValueError:
        empty_error = True
    return V([types, edge_types, empty_error])


def uv_summary(s, atlas, gutter):
    vertices, triangles, normals, uvs = s.tessellate_with_uvs(0.1, 0.1, atlas, gutter)
    us = [uv[0] for uv in uvs]
    vs = [uv[1] for uv in uvs]
    n_sum = sum(n.X + 2 * n.Y + 3 * n.Z for n in normals)
    index_sum = sum(t[0] + 2 * t[1] + 3 * t[2] for t in triangles)
    return [len(vertices), len(triangles), len(normals), len(uvs), rnd(min(us), 6), rnd(max(us), 6), rnd(min(vs), 6),
            rnd(max(vs), 6), rnd(sum(us), 3), rnd(sum(vs), 3), rnd(n_sum, 3), float(index_sum)]


def tessellate_with_uvs_case():
    box = Box(1, 2, 3)
    cyl = Cylinder(1, 2)
    return V([uv_summary(box, True, 0.0), uv_summary(box, False, 0.0), uv_summary(box, True, 0.01),
              uv_summary(cyl, True, 0.0), uv_summary(Pos(1, 2, 3) * Sphere(1), True, 0.0)])


def show_topology_case():
    import re

    def strip(s):
        return re.sub(r"at 0x[0-9a-f]+", "at X", s)

    def strip_class(s):
        return re.sub(r"\S+ +at 0x[0-9a-f]+", "at X", s)

    box = Box(1, 2, 3)
    child1 = Pos(1, 0, 0) * Box(1, 1, 1)
    sub = Compound(label="sub", children=[Sphere(1)])
    assy = Compound(label="assy", children=[child1, sub])
    return V([strip(box.show_topology("Face")), strip(box.solids()[0].show_topology("Edge", show_center=False)),
              strip(Face.make_rect(1, 1).show_topology()), strip_class(assy.show_topology()),
              strip_class(assy.show_topology(show_center=True))])


def as_list(x):
    return [] if x is None else (list(x) if isinstance(x, list) else [x])


def split_both():
    box = Box(2, 2, 2)
    top, bottom = box.split(Plane.XY.offset(0.5), Keep.BOTH)
    u_shape = Box(6, 2, 4) - Pos(0, 0, 1) * Box(2, 2, 2)
    p_top, p_bottom = u_shape.split(Plane.XY, Keep.BOTH)
    none_below = box.split(Plane.XY.offset(-5), Keep.BOTTOM)
    all_pieces = u_shape.split(Plane.XY, Keep.ALL)
    op_both = split(u_shape, Plane.XY, Keep.BOTH)
    op_bottom = split(u_shape, Plane.XY, Keep.BOTTOM)
    return V([describe(as_list(top)), describe(as_list(bottom)), describe(as_list(p_top)), describe(as_list(p_bottom)),
              len(as_list(p_top)) + len(as_list(p_bottom)), len(as_list(none_below)), len(all_pieces),
              describe(op_both.solids()), describe(op_bottom.solids())])


def face_wire_warning():
    import warnings

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        holed = Rectangle(4, 4) - Circle(1)
        with_holes = holed.faces()[0].wire().length
        plain = Face.make_rect(2, 2).wire().length
    return V([[str(w.message) for w in caught], with_holes, plain])


def transform_keeps_location():
    moved = Box(1, 2, 3).translate((1, 2, 3))
    regenerated = Box(1, 2, 3).translate((1, 2, 3), transform=True)
    rotated = Box(1, 2, 3).rotate(Axis.Z, 30, transform=True)
    return V([moved.location.position, regenerated.location.position, rotated.location.orientation,
              regenerated.center(), rotated.faces()[0].normal_at()])


def raises(f):
    try:
        f()
    except Exception:  # noqa: BLE001
        return True
    return False


def centers(shapes):
    return [s.center() for s in shapes]


def measures(shapes):
    return [rnd(measure(s), 6) for s in shapes]


def shapelist_sort():
    box = Box(1, 2, 3)
    skew = Axis((1, 2, 3), (1, 0.3, 0.7))
    cone = Solid.make_cone(2, 1, 2)
    circles = cone.edges().filter_by(GeomType.CIRCLE)
    radii = [e.radius for e in circles.sort_by(SortBy.RADIUS, reverse=True)]
    cyl = Cylinder(1, 2)
    labelled = []
    for name in ["b", "c", "a"]:
        b = Solid.make_box(1, 1, 1)
        b.label = name
        labelled.append(b)
    labels = [b.label for b in ShapeList(labelled).sort_by(lambda b: b.label)]
    path = Edge.make_line((0, 0, 0), (1, 2, 3))
    solids = ShapeList([Solid.make_box(3, 1, 1), Solid.make_box(1, 1, 1), Solid.make_box(2, 1, 1)])
    return V([centers(box.vertices().sort_by(skew)), centers(box.vertices().sort_by(skew, reverse=True)),
              centers(box.faces().sort_by(skew)), measures(box.faces().sort_by(SortBy.AREA)),
              measures(box.faces() < SortBy.AREA), radii,
              raises(lambda: cone.edges().sort_by(SortBy.RADIUS)),
              raises(lambda: cyl.faces().sort_by(SortBy.RADIUS)),
              len(cyl.faces().filter_by(GeomType.CYLINDER).sort_by(SortBy.RADIUS)),
              measures(box.edges().sort_by(SortBy.LENGTH)),
              centers((Pos(1, 1.2, 1.1) * Box(1, 2, 3)).vertices().sort_by(SortBy.DISTANCE)),
              measures(solids.sort_by(SortBy.VOLUME)),
              measures(box.faces().sort_by(lambda f: f.area, reverse=True)), labels,
              centers(Box(1, 2, 3, align=Align.MIN).vertices().sort_by(path)),
              centers(box.edges().sort_by_distance((-0.2, 0.13, 0.5))),
              centers(box.faces().sort_by_distance(Vertex(5, 0.3, 0.25), reverse=True)),
              raises(lambda: box.vertices().sort_by(SortBy.LENGTH))])


def shapelist_group():
    def sizes(groups):
        return [len(g) for g in groups]

    def keys(groups):
        return [k for k, _ in groups.key_to_group_index]

    box = Solid.make_box(1, 1, 2)
    line = Edge.make_line((0, 0, 0), (1, 1, 2))
    boxes = []
    for i in range(6):
        b = Solid.make_box(1, 1, 1)
        b.label = "A" if i < 3 else ("B" if i < 5 else "")
        boxes.append(b)
    by_label = ShapeList(boxes).group_by(lambda b: b.label)
    box1 = Box(2, 2, 2)
    box2 = Box(2, 2, 2).translate((1, 1, 1))
    fused = (box1 + box2).edges().group_by(Edge.is_interior)
    cut = (box1 - box2).edges().group_by(Edge.is_interior)
    try:
        by_label.group("C")
        missing = False
    except KeyError:
        missing = True
    solids = ShapeList([Solid.make_box(1, 1, 1), Solid.make_box(1, 1, 2), Solid.make_box(1, 1, 1),
                        Solid.make_box(1, 1, 3), Solid.make_box(1, 1, 2)])
    return V([sizes(box.vertices().group_by(Axis.Z)), keys(box.vertices().group_by(Axis.Z)),
              keys(box.vertices().group_by(Axis.Z, reverse=True)),
              keys(box.vertices().group_by(Axis((0, 0, 0), (1, 1, 0.5)), tol_digits=2)),
              sizes(box.edges().group_by(SortBy.LENGTH)), sizes(box.faces().group_by(SortBy.AREA)),
              sizes(box.vertices().group_by(line)), keys(box.vertices().group_by(line)),
              sizes(by_label), keys(by_label), len(by_label.group("A")), missing,
              len(by_label.group_for(boxes[3])), len(fused.group(True)), len(fused.group(False)),
              len(cut.group(True)), len(cut.group(False)), len(solids << SortBy.VOLUME),
              len(solids >> SortBy.VOLUME), len(box.faces() >> Axis.X), (box.faces() << Axis.Z)[0].center()])


def shapelist_filter():
    box = Solid.make_box(1, 1, 1)
    c1 = Cylinder(1, 3)
    c2 = Cylinder(1, 3, rotation=(90, 0, 0))
    bar = Box(1, 2, 3)

    def counts_by_position(axis, lo, hi):
        out = [len(bar.vertices().filter_by_position(axis, lo, hi, inclusive))
               for inclusive in [(True, True), (True, False), (False, True), (False, False)]]
        out.append(len(bar.faces().filter_by_position(axis, lo - 0.1, hi - 0.2)))
        return out

    positioned = centers(bar.faces().filter_by_position(Axis((0, 0, 0), (1, 0.3, 0.7)), -0.6, 0.6))
    pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4)
    return V([len(box.faces().filter_by(Axis.X)), len(box.edges().filter_by(Axis.X)),
              len(box.vertices().filter_by(Axis.X)), len(c1.faces().filter_by(Plane.XY)),
              len(c1.edges().filter_by(Plane.XY)), len(c2.faces().filter_by(Plane.XZ)),
              len(c2.edges().filter_by(Plane.XZ)), len(c1.wires().filter_by(Plane.XY)),
              len(c2.wires().filter_by(Plane.XZ)), len((Pos(1, 2, 3) * c2).edges().filter_by(Plane.XZ)),
              len(Solid.make_cylinder(1, 1).faces().filter_by(GeomType.PLANE, reverse=True)),
              counts_by_position(Axis.Z, -1.5, 1.5), counts_by_position(Axis.Y, -1, 0), positioned,
              len(box.edges() | Axis.Z), len(c1.faces() | GeomType.PLANE), len(c1.edges() | Plane.XY),
              len(pocket.edges() | Convexity.CONCAVE), len(box.edges().filter_by(lambda e: e.center().Z > 0.5)),
              len(box.edges().filter_by(Axis.Z, reverse=True))])


def shapelist_ops():
    box = Box(1, 1, 1)
    cyl = Cylinder(1, 1)
    sl = ShapeList([box, cyl])
    same = ShapeList([cyl, box])
    repeated = ShapeList([cyl, box, box])
    other = ShapeList([Box(1, 1, 1), Box(1, 2, 3)])
    faces = Box(1, 2, 3).faces()
    top = faces.sort_by(Axis.Z)[-2:]
    doubled = faces + faces
    appended = faces[0:1]
    appended += faces[1]
    appended += faces[2:4]

    def messages(f):
        try:
            f()
        except ValueError as e:
            return str(e)
        return None

    square = Wire.make_rect(1, 1)
    shell = Box(1, 1, 1).shells()[0]
    nested = ShapeList([Compound([Solid.make_box(1, 1, 1), square, Compound([shell])]), Vertex(1, 2, 3)])
    return V([sl == same, sl == repeated, sl != same, sl == other, sl != other, describe(doubled - top),
              describe(faces - top), describe(doubled & top), len(doubled - top), len(doubled & top),
              len(faces + faces[0]), len(appended), centers(faces[::2]), centers(faces[::-1]),
              centers(faces[1:-1:2]), centers(faces[5:1:-2]), centers(faces[-2:]), len(faces[10:]),
              centers(faces[-10:2]), faces.first.center(), faces.last.center(), ShapeList().center(),
              ShapeList(Vertex(i, 0, 0) for i in range(3)).center(),
              ShapeList([box, Vertex(1, 1, 1)]).compound().volume, len(sl.compounds()),
              messages(lambda: sl.compound()), messages(lambda: faces.edge()), messages(lambda: faces.solid()),
              describe(nested.expand()), measures(faces[0:2])])


def geom_plane_lists():
    def planes(lst):
        return [geom(p) for p in lst]

    def locations(lst):
        return [geom(loc) for loc in lst]

    moved = Plane.XY
    moved.move(Plane.XZ.offset(2))
    y_set = Plane.XY.offset(1)
    y_set.y_dir = (1, 1, 0)
    from build123d.geometry import Plane as GPlane
    return V([planes(Pos(1, 2, 3) * [Plane.XY, Plane.XZ]), planes([Pos(1, 0, 0), Rot(0, 0, 90)] * Plane.XZ),
              planes([Plane.XY.offset(1), Plane.YZ] * Plane.XZ), locations(Plane.XZ * [Plane.XY.offset(1), Plane.YZ]),
              locations(Plane.XZ * [Pos(1, 2, 3), Rot(10, 0, 0)]), geom(Plane.XY.moved(Plane.YZ.offset(3))), geom(moved),
              geom(y_set), geom(Plane([(1, 2, 3), (2, 2, 3), (1, 3, 4)])), geom(Rot((10, 20, 30))),
              geom(Rot((10, 20, 30), Extrinsic.ZYX)), geom(Rot(Axis((1, 0, 0), (0, 0, 1)), 90)), Vector(1, 2, 3) / 3,
              GPlane.get_topods_face_normal(Face.make_rect(1, 1, Plane.XZ.offset(2)).wrapped)])


def geom_bound_box():
    def box(lo, hi):
        return Solid.make_box(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], Plane(lo)).bounding_box()

    a = box((0, 0, 0), (4, 4, 4))
    inner = box((1, 1, 1), (2, 2, 2))
    flush = box((0, 1, 1), (2, 2, 2))
    beside = box((4, 0, 0), (6, 4, 4))
    crossing = box((3, 1, 1), (6, 2, 2))
    far = box((10, 10, 10), (11, 11, 11))
    face = Face.make_rect(2, 2, Plane((2, 2, 4))).bounding_box()
    face2 = Face.make_rect(1, 1, Plane((2, 2, 4))).bounding_box()
    other_level = Face.make_rect(1, 1, Plane((2, 2, 5))).bounding_box()
    table = [[a.covers(o), a.contains(o), a.contains_properly(o), o.within(a), a.intersects(o), a.disjoint(o),
              a.touches(o), a.overlaps(o)] for o in (inner, flush, beside, crossing, far, face, a)]

    def bbox_v(b):
        return [b.min, b.max, b.size]

    outside = BoundBox.find_outside_box_2d(a, inner)
    none = BoundBox.find_outside_box_2d(a, beside)
    return V([table, face.contains(face2), face.contains_properly(face2), face.overlaps(face2),
              face.touches(other_level), face.contains(other_level), inner.is_inside(a),
              bbox_v(outside) if outside else None, bbox_v(none) if none else None,
              bbox_v(Plane.XZ.to_local_coords(inner)), bbox_v(Plane.XZ.offset(1).from_local_coords(inner)),
              bbox_v(BoundBox(Box(1, 2, 3))), bbox_v(BoundBox(Shape())), repr(inner), BoundBox(Shape()).diagonal])


def geom_color():
    def t(c):
        return list(tuple(c))

    def raises(f):
        try:
            f()
        except Exception:  # noqa: BLE001
            return True
        return False

    sets = [[t(c) for c in s] for s in (Color.categorical_set(3), Color.categorical_set(4, 0.3, [0.1, 0.2, 0.3, 0.4]),
                                        Color.categorical_set(2, 0xFF8000), Color.categorical_set(5, 0.95, 0.5),
                                        Color.categorical_set(0))]
    return V([t(Color()), t(Color("red")), t(Color(" Red ")), t(Color("goldenrod1")), t(Color("#ff0000", 0.5)),
              t(Color("#f008")), t(Color("#FF0000", 0.3)), t(Color("#0f0")), t(Color(0xFF0000, 0x80)),
              t(Color(0xFF0000, 1)), t(Color(0x000001)), t(Color(0.5)), t(Color(0.5, 0.2)), t(Color(0.1, 0.2, 0.3, 0.3)),
              t(Color(0.5, 0.5, 0.5, 2.0)), t(Color("SteelBlue", 0.25)), repr(Color(1, 0, 0)), str(Color(0.1, 0.5, 0.9)),
              str(Color("aqua")), str(Color(0.5, 0.5, 0.5)), str(Color("lightgrey")), str(Color("goldenrod1")),
              raises(lambda: Color("dark red")), raises(lambda: Color("#gg0000")), raises(lambda: Color("#ff0000zz")),
              raises(lambda: Color("#ff000")), raises(lambda: Color("rebeccapurple")), raises(lambda: Color(2.0, 0, 0)),
              raises(lambda: Color(-0.1, 0, 0)), raises(lambda: Color(0x1000000)), sets,
              raises(lambda: Color.categorical_set(3, 1.5)), raises(lambda: Color.categorical_set(3, 0.0, [1.0])),
              raises(lambda: Color.categorical_set(3, -1))])


def shape_members11():
    box = Box(2, 2, 2)
    distances = list(box.distances(Pos(5, 0, 0) * Box(1, 1, 1), Pos(0, 0, 3) * Sphere(0.5), Box(1, 1, 1)))
    labeled = Box(1, 1, 1)
    labeled.label = "src"
    labeled.color = Color("red")
    target = Cylinder(1, 1)
    labeled.copy_attributes_to(target, ["_color", "wrapped", "_NodeMixin__children"])
    target2 = Cylinder(1, 1)
    target2.label = "keep"
    labeled.copy_attributes_to(target2, ["wrapped", "_NodeMixin__children"])
    return V([distances, len(box.entities("Face")), len(box.entities("Edge")), len(box.entities("Vertex")),
              isinstance(Shape.as_shape(Vector(1, 2, 3)), Vertex), isinstance(Shape.as_shape(Axis.X), Edge),
              isinstance(Shape.as_shape(Plane.XY), Face), target.label, target.color is not None, target2.label,
              target2.color is not None])


def positions_deflection():
    spline = Edge.make_spline([(0, 0, 0), (3, 2, 0), (6, -1, 1), (9, 0, 0)])
    circle = Edge.make_circle(5)
    wire = Wire.make_polygon([(0, 0, 0), (5, 0, 0), (5, 5, 0)], close=False)

    def summary(pts):
        total = sum(pts, Vector(0, 0, 0))
        return [len(pts), total, pts[0] if pts else None, pts[-1] if pts else None]

    try:
        spline.positions()
        missing = False
    except ValueError:
        missing = True
    return V([summary(spline.positions(deflection=0.01)), summary(circle.positions(deflection=0.05)),
              summary(wire.positions(deflection=0.1)), summary(spline.positions([0, 2, 4], PositionMode.LENGTH)), missing])


def curvature_comb_case():
    spline = Edge.make_spline([(0, 0, 0), (3, 2, 0), (6, -1, 0), (9, 0, 0)])
    ring = Wire(Edge.make_circle(4))
    try:
        Edge.make_circle(1, Plane.XZ).curvature_comb()
        not_xy = False
    except ValueError:
        not_xy = True
    return V([describe(spline.curvature_comb(20)), describe(ring.curvature_comb(12, 1.0)),
              len(Edge.make_line((0, 0), (5, 0)).curvature_comb()), not_xy])


def edge_members11():
    arc = Edge.make_circle(5, Plane.XY, 0, 270)
    spline = Edge.make_spline([(0, 0, 0), (3, 3, 0), (6, 0, 0), (9, 3, 0)])
    locs = [geom(loc) for loc in spline.distribute_locations(4, 0.1, 0.9)]
    flat = [geom(loc) for loc in arc.distribute_locations(3, 0, 1, positions_only=True)]
    try:
        arc.distribute_locations(1)
        few = False
    except ValueError:
        few = True
    loop = Edge.make_spline([(0, 0, 0), (4, 4, 0), (4, 0, 0), (0, 4, 0)])
    return V([locs, flat, few, arc.find_tangent(180), Edge.make_circle(3).find_tangent(45), spline.find_tangent(0),
              Edge.make_line((0, 0), (1, 1)).find_tangent(45), Edge.make_line((0, 0), (1, 1)).find_tangent(10),
              describe([arc.trim((5, 0, 0), (-5, 0, 0)), arc.trim(0.1, (0, 5, 0)), arc.trim_to_length((0, 5, 0), 3)]),
              list(loop.find_intersection_points()),
              list(spline.find_intersection_points(Edge.make_line((0, 1.5), (9, 1.5)))),
              list(spline.find_intersection_points(Axis((0, 1), (1, 0))))])


def wire_members11():
    wire = Wire.make_polygon([(0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 0)], close=False)
    twin = Wire.make_polygon([(0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 0)], close=False)
    other = Wire.make_polygon([(0, 0, 0), (4, 0, 0), (4, 3.5, 0)], close=False)
    edges = wire.edges()
    first, second = Wire.order_chamfer_edges(edges[1], (edges[0], edges[1]))
    try:
        Wire.order_chamfer_edges(edges[2], (edges[0], edges[1]))
        not_in = False
    except ValueError:
        not_in = True
    return V([describe(wire.trim(0.2, 0.7).edges()), describe(wire.trim(0.7, 0.2).edges()),
              describe(wire.trim((1, 0, 0), (4, 2, 0)).edges()), describe(wire.trim(0.0, (2, 3, 0)).edges()),
              wire.geom_equal(twin), wire.geom_equal(other), wire.param_at(0.0), wire.param_at(0.5), wire.param_at(1.0),
              first.is_same(edges[1]), second.is_same(edges[0]), not_in])


def face_members11():
    torus = Torus(5, 1).faces()[0]
    cyl_face = Cylinder(2, 4).faces().filter_by(GeomType.CYLINDER)[0]
    cone_face = Cone(2, 1, 3).faces().filter_by(GeomType.CONE)[0]
    r = torus.radii
    plain = Face.make_rect(2, 2).radii
    sewn = Face.sew_faces([Face.make_rect(1, 1), Face.make_rect(1, 1, Plane((1, 0, 0))),
                           Face.make_rect(1, 1, Plane((5, 0, 0)))])
    groups = sorted(float(len(g)) for g in sewn)
    patch = Face.make_surface_patch(
        edge_constraints=Wire.make_polygon([(0, 0, 0), (4, 0, 1), (4, 4, 0), (0, 4, 1)]).edges(),
        point_constraints=[(2, 2, 1.5)])
    base = Box(4, 4, 2, align=Align.MIN)
    top = base.faces().sort_by(Axis.Z)[-1]
    side = base.faces().sort_by(Axis.X)[0]
    filled = Face.make_surface_patch(edge_face_constraints=[(top.edges().sort_by(Axis.X)[0], top, ContinuityLevel.C0)],
                                     point_constraints=[(-1, 2, 3)])
    return V([list(r) if r else None, True if plain else None, describe(cyl_face.seams), cone_face.semi_angle,
              Face.make_rect(1, 1).semi_angle is not None, describe([cyl_face.uv_face, Sphere(1).faces()[0].uv_face]),
              describe([Face.make_rect(2, 2).offset(3), side.offset(-1)]), groups, describe([patch, filled]),
              patch.is_valid])


def axes_of_symmetry_case():
    def dirs(f):
        rows = sorted((rnd(a.direction.X, 5), rnd(a.direction.Y, 5), rnd(a.direction.Z, 5)) for a in f.axes_of_symmetry)
        return [list(r) for r in rows]

    ell = Rectangle(4, 4) - Pos(1, 1) * Rectangle(2, 2)
    holed = Rectangle(6, 4) - Pos(1.5, 0) * Circle(0.5)
    slot = SlotOverall(8, 2)
    return V([dirs(Face.make_rect(4, 2)), dirs(Face.make_rect(3, 3)), dirs(ell.faces()[0]), dirs(holed.faces()[0]),
              dirs(slot.faces()[0]), dirs(Triangle(a=3, b=4, c=5).faces()[0])])


def side_list(x):
    if x is None:
        return []
    if isinstance(x, list):
        return list(x)
    return [x]


def split_by_perimeter_case():
    square = Face.make_rect(10, 10)
    circle = Wire(Edge.make_circle(2))
    inside = square.split_by_perimeter(circle)
    outside = square.split_by_perimeter(circle, Keep.OUTSIDE)
    in_both, out_both = square.split_by_perimeter(circle, Keep.BOTH)
    box_shell = Box(4, 4, 4).shells()[0]
    band = Wire.make_rect(1, 1, Plane((0, 0, 2)))
    shell_inside = box_shell.split_by_perimeter(band)
    try:
        square.split_by_perimeter(Edge.make_line((0, 0), (1, 0)))
        open_ = False
    except ValueError:
        open_ = True
    return V([describe(side_list(inside)), describe(side_list(outside)), describe(side_list(in_both)),
              describe(side_list(out_both)), describe(side_list(shell_inside)), open_])


def wrap_case():
    surface = Cylinder(10, 30).faces().filter_by(GeomType.CYLINDER)[0]
    loc = Location(Plane((10, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)))
    wrapped_edge = surface.wrap(Edge.make_line((0, 0), (5, 0)), loc)
    wrapped_wire = surface.wrap(Wire.make_polygon([(0, 0, 0), (4, 0, 0), (4, 3, 0)], close=False), loc)
    wrapped_face = surface.wrap(Face.make_rect(4, 2), loc)
    faces = surface.wrap_faces([Face.make_rect(2, 1), Face.make_rect(1, 1, Plane((3, 0, 0)))], Edge.make_circle(10), 0.0)
    return V([wrapped_edge.length, wrapped_edge @ 1, wrapped_wire.length, wrapped_wire @ 1, wrapped_face.area,
              wrapped_face.center(), len(wrapped_face.edges()), describe(faces)])


def dprism_faces():
    box = Solid.make_box(10, 10, 10)
    top = box.faces().sort_by(Axis.Z)[-1]
    return box.dprism(top, [Face.make_rect(4, 4, Plane((5, 5, 10)))], 3.0, 5.0, None, thru_all=False, additive=False)


def dprism_wires():
    box = Solid.make_box(10, 10, 10)
    return box.dprism(None, [Wire.make_rect(6, 6, Plane((5, 5, 10))), Wire.make_circle(1, Plane((5, 5, 10)))],
                      None, 0.0, None, thru_all=True, additive=False)


def hollow_open():
    box = Solid.make_box(10, 10, 10)
    return box.hollow([box.faces().sort_by(Axis.Z)[-1]], -1.0)


def sweep_inner():
    path = Wire(Edge.make_three_point_arc((0, 0, 0), (5, 0, 5), (10, 0, 0)))
    return Solid.sweep(Wire.make_circle(2, Plane.YZ), path, [Wire.make_circle(1, Plane.YZ)])


def compound_members11():
    a = Box(2, 2, 2)
    b = Pos(1, 0, 0) * Box(2, 2, 2)
    c = Pos(10, 0, 0) * Box(1, 1, 1)
    overlapping = Compound(children=[a, b, c])
    hit, pair, volume = overlapping.do_children_intersect()
    a2 = Box(2, 2, 2)
    c2 = Pos(10, 0, 0) * Box(1, 1, 1)
    apart = Compound(children=[a2, c2])
    miss, none, zero = apart.do_children_intersect()
    prism = Compound.extrude(Box(1, 1, 1).shells()[0], (0, 0, 2))
    triad = Compound.make_triad(10)
    return V([hit, pair[0].center() if pair[0] else None, pair[1].center() if pair[1] else None, volume, miss,
              none[0] is not None, zero, len(prism.faces()), len(prism.compounds()), describe(triad.edges()[:9])])


def sort_wires_case():
    from build123d.topology import sort_wires_by_build_order
    groups = sort_wires_by_build_order([Wire.make_rect(10, 10), Wire.make_circle(1), Wire.make_rect(2, 2, Plane((3, 3, 0)))])
    return V([len(groups), [describe(g) for g in groups], len(sort_wires_by_build_order([Wire.make_circle(1)]))])


def attributes_survive():
    def attrs(x):
        return [x.label, list(tuple(x.color)) if x.color is not None else None,
                x.material.name if x.material is not None else None]

    b = Box(1, 1, 1)
    b.label = "box"
    b.material = "aluminum"
    b.color = Color("red")
    s = Pos(0.5, 0, 0) * Sphere(0.6)
    far = Pos(5, 0, 0) * Box(1, 1, 1)
    r = Rectangle(2, 2)
    r.label = "rect"
    r.color = Color("blue")
    l = Line((0, 0), (1, 0))
    l.label = "line"
    c = Compound([Box(1, 1, 1)])
    c.label = "single"
    c.color = Color("green")
    nested = Compound([Compound([Box(1, 1, 1)])])
    nested.label = "nested"
    return V([attrs(b + s), attrs(b + far), attrs(b - s), attrs(b & s), attrs(Pos(1, 0, 0) * b),
              attrs(b.moved(Location((1, 2, 3)))), attrs(b.located(Location((1, 2, 3)))), attrs(b.rotate(Axis.Z, 30)),
              attrs(b.translate((1, 0, 0))), attrs(b.mirror(Plane.YZ)), attrs(b.scale(2)), attrs(b.clean()),
              attrs(b.solid()), attrs(b.unwrap()), attrs(r + Circle(1.5)), attrs(r - Circle(0.5)),
              attrs(r & Circle(1.2)), attrs(l + Line((1, 0), (1, 1))), attrs(c.unwrap()), attrs(c.unwrap(False)),
              attrs(nested.unwrap()), attrs(nested.unwrap(False)), attrs(b - [s, far]),
              attrs(fillet(b.edges().filter_by(Axis.Z), 0.1)), attrs(b.transform_geometry(Matrix())),
              attrs(-r.faces()[0])])


def mixin1d_queries():
    spline = Edge.make_spline([(0, 0, 0), (3, 2, 1), (6, -1, 2), (9, 0, 0)])
    wire = Wire.make_polygon([(0, 0, 0), (4, 0, 0), (4, 3, 0), (0, 3, 2)], close=False)
    arc = Edge.make_circle(5, Plane.XZ, 10, 200)
    derivatives = []
    for c in (spline, wire, arc):
        for order in (1, 2, 3):
            derivatives.append(c.derivative_at(0.3, order))
            derivatives.append(c.derivative_at(0.3, order, PositionMode.LENGTH))
        derivatives.append(c.derivative_at(c @ 0.6, 1))
    angles = []
    for u in (0.0, 0.25, 0.5, 0.75, 1.0):
        angles.append(arc.tangent_angle_at(u, PositionMode.PARAMETER, Plane.XZ))
        angles.append(Edge.make_circle(2).tangent_angle_at(u))
        angles.append(Edge.make_circle(2).tangent_angle_at(u * 3, PositionMode.LENGTH, Plane.XY.rotated((0, 0, 30))))
    planar = Edge.make_line((0, 0, 1), (1, 1, 1)).common_plane(Edge.make_line((2, 0, 1), (3, 4, 1)))
    not_planar = Edge.make_line((0, 0, 0), (1, 0, 0)).common_plane(Edge.make_line((0, 1, 1), (0, 2, 3)))
    line_only = Edge.make_line((0, 0, 0), (1, 1, 0)).common_plane()
    params = [wire.param_at_point(p) for p in ((1, 0.5, 0), (4, 1.5, 0), (2, 3, 1))]
    params.append(spline.param_at_point(spline @ 0.37))
    locs = [geom(loc) for loc in spline.locations([0.1, 0.5, 0.9])]
    locs.append(geom(spline.location_at(0.4, PositionMode.PARAMETER, FrameMethod.FRENET, x_dir=(0, 0, 1))))
    locs.append(geom(wire.location_at(2.5, PositionMode.LENGTH, FrameMethod.CORRECTED)))
    locs.append(geom(spline.location_at(4.0, PositionMode.LENGTH, FrameMethod.CORRECTED)))
    try:
        Edge.make_line((0, 0, 0), (1, 0, 0)).normal()
        no_normal = False
    except Exception:  # noqa: BLE001
        no_normal = True
    try:
        straight_normal = wire.edges()[0].normal() == Vector()
    except Exception:  # noqa: BLE001
        straight_normal = None
    return V([derivatives, angles, geom(planar) if planar else None, not_planar is not None,
              geom(line_only) if line_only else None, params, locs, Edge.make_circle(3, Plane.XZ).normal(),
              straight_normal, no_normal])


def edge_wire_queries():
    a = Edge.make_line((0, 0, 0), (4, 0, 0))
    b = Edge.make_three_point_arc((0, 2, 0), (2, 3, 0), (4, 2, 0))
    rev = b.reversed()
    rebuilt = b.reversed(reconstruct=True)
    w1 = Wire.make_polygon([(0, 0, 0), (1, 0, 0), (1, 1, 0)], close=False)
    w2 = Wire.make_polygon([(1, 1, 0), (0, 2, 0), (-1, 1, 0)], close=False)
    messy = Wire([Edge.make_line((0, 0), (1, 0)), Edge.make_line((1, 1), (1, 0)), Edge.make_line((1, 1), (0, 1))])
    ordered = [[e @ 0, e @ 1] for e in messy.order_edges()]
    combined = Wire.combine([Edge.make_line((0, 0), (1, 0)), Edge.make_line((1, 0), (1, 1)),
                             Edge.make_line((5, 5), (6, 5))])
    return V([describe([Edge.make_mid_way(a, b), Edge.make_mid_way(a, b, 0.25)]), rev @ 0, rev % 0, rebuilt @ 0,
              rebuilt % 0, rev.is_forward, rebuilt.is_forward, describe(w1.stitch(w2).edges()), w1.stitch(w2).length,
              ordered, len(combined), describe(combined), w1.close().is_closed, w1.close().length,
              Wire.make_ellipse(3, 2, Plane.XY, 0, 90, AngularDirection.COUNTER_CLOCKWISE, True).length,
              Wire.make_ellipse(3, 2, Plane.XY, 0, 90, AngularDirection.COUNTER_CLOCKWISE, False).length,
              len(Wire.make_ellipse(3, 2, Plane.XY, 0, 90, AngularDirection.COUNTER_CLOCKWISE, True).edges())])


def face_queries():
    rect = Face.make_rect(4, 2, Plane.XZ.offset(1))
    tilted = Face.make_rect(3, 1).rotate(Axis.Z, 30)
    square = Face.make_rect(2, 2)
    cyl = Cylinder(2, 4).faces().filter_by(GeomType.CYLINDER)[0]
    holed = square.make_holes([Wire.make_circle(0.5)])
    ell = (Rectangle(4, 4) - Pos(1, 1) * Rectangle(2, 2)).faces()[0]
    chamfered = square.chamfer_2d(0.5, 0.25, [square.vertices()[0]], square.edges()[0])
    return V([rect.width, rect.length, tilted.width, tilted.length, cyl.width, square.geometry, rect.geometry,
              ell.geometry, cyl.geometry, cyl.radius, rect.radius,
              geom(cyl.axis_of_rotation) if cyl.axis_of_rotation else None, rect.axis_of_rotation is not None,
              geom(rect.location_at(0.2, 0.7)), geom(cyl.location_at(0.25, 0.5)), geom(rect.center_location),
              geom(cyl.center_location), rect.is_coplanar(Plane.XZ.offset(1)), rect.is_coplanar(Plane.XZ),
              square.is_inside((0.5, 0.5, 0)), square.is_inside((3, 0, 0)), holed.area, holed.without_holes().area,
              len(holed.inner_wires()), chamfered.area, describe(chamfered.edges()),
              describe([Face.make_surface_from_curves(Edge.make_line((0, 0, 0), (4, 0, 0)),
                                                      Edge.make_three_point_arc((0, 0, 3), (2, 1, 3), (4, 0, 3)))])])


def shell_solid_queries():
    lofted = Shell.make_loft([Wire.make_rect(4, 4), Wire.make_circle(1, Plane.XY.offset(3))])
    revolved = Shell.revolve(Edge.make_line((2, 0, 0), (3, 0, 2)), 180, Axis.Z)
    extruded = Shell.extrude(Wire.make_polygon([(0, 0, 0), (2, 0, 0), (2, 1, 0)], close=False), (0, 0, 3))
    from_bbox = Solid.from_bounding_box((Rot(10, 20, 0) * Box(1, 2, 3)).bounding_box())
    box = Box(4, 4, 4)
    mixed = Compound([Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1),
                      Compound([Pos(0, 4, 0) * Box(1, 1, 1), Face.make_rect(1, 1)])])
    return V([describe([lofted, revolved, extruded]), from_bbox.volume, from_bbox.center(), box.is_inside((1, 1, 1)),
              box.is_inside((2, 0, 0)), box.is_inside((2.1, 0, 0)), box.is_inside((2.0005, 0, 0), 1e-3),
              len(mixed.get_type(Solid)), len(mixed.get_type(Face)), len(mixed.get_type(Compound)), len(list(mixed))])


def shape_queries():
    box = Box(2, 2, 2)
    sphere = Pos(5, 1, 0) * Sphere(1)
    d, p1, p2 = box.distance_to_with_closest_points(sphere)
    dp, q1, q2 = box.distance_to_with_closest_points((4, 4, 4))
    closest = box.closest_points(sphere)
    hits = box.faces_intersected_by_axis(Axis((-5, 0.2, 0.3), (1, 0, 0)))
    vertices, triangles = Sphere(1).tessellate(0.1, 0.2)
    vsum = sum(vertices, Vector(0, 0, 0))
    moved = box.transformed((10, 20, 30), (1, 2, 3))
    scaled = box.scale((1, 2, 3), (1, 1, 1))
    scaled2 = box.scale(2, (1, 0, 0))
    return V([box.distance_to(sphere), box.distance_to((4, 4, 4)), d, p1, p2, dp, q1, q2, closest[0], closest[1],
              describe(hits), len(vertices), len(triangles), vsum, moved.center(), moved.faces()[0].normal_at(),
              scaled.bounding_box().min, scaled.bounding_box().max, scaled2.bounding_box().min, scaled2.bounding_box().max])


def skip_clean_case():
    from build123d.topology import SkipClean

    a = Box(2, 2, 2)
    b = Pos(2, 0, 0) * Box(2, 2, 2)
    with SkipClean():
        faces_skip = len((a + b).faces())
    return V([len((a + b).faces()), faces_skip, len((a + b).faces())])


def plane_helpers():
    p = Plane.XY.offset(2).rotated((0, 0, 30))
    try:
        p.shift_origin((0, 0, 5))
        not_in = False
    except ValueError:
        not_in = True
    return V([geom(p.shift_origin((1, 1, 2))), geom(p.shift_origin(Axis((3, 4, 0), (0, 0, 1)))),
              geom(Plane.XZ.shift_origin(Vertex(1, 0, 4))), not_in, geom(p.location_between(Plane.YZ.offset(1))),
              geom(p.rotated((10, 20, 30), Extrinsic.XYZ))])


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
    "core_touch_found": touch_found,
    "core_topology_utils_more": topology_utils_more,
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
    # 6: mass
    "core_mass_inertia": mass_inertia,
    "core_mass_material": mass_material,
    "core_combined_center": combined_center,
    # 7: Matrix, OrientedBoundBox
    "core_matrix": matrix_case,
    "core_transform_geometry": transform_geometry_case,
    "core_transform_shape_matrix": transform_shape_matrix,
    "core_oriented_bound_box": oriented_bound_box,
    # 8: Shape misc
    "core_hash_sets": hash_sets,
    "core_to_splines_box": lambda: Box(2, 3, 4).to_splines(),
    "core_to_splines_cylinder": lambda: Cylinder(1, 2).to_splines(5, 1e-4),
    "core_to_splines_types": to_splines_types,
    "core_tessellate_with_uvs": tessellate_with_uvs_case,
    "core_show_topology": show_topology_case,
    "core_split_both": split_both,
    "core_face_wire_warning": face_wire_warning,
    "core_rotate_transform": lambda: Box(1, 2, 3).rotate(Axis.Z, 30, transform=True),
    "core_translate_transform": lambda: Box(1, 2, 3).translate((1, 2, 3), transform=True),
    "core_transform_keeps_location": transform_keeps_location,
    # 9: ShapeList
    "core_shapelist_sort": shapelist_sort,
    "core_shapelist_group": shapelist_group,
    "core_shapelist_filter": shapelist_filter,
    "core_shapelist_ops": shapelist_ops,
    # 10: geometry
    "core_geom_plane_lists": geom_plane_lists,
    "core_geom_bound_box": geom_bound_box,
    "core_geom_color": geom_color,
    # 11: topology members
    "core_shape_members11": shape_members11,
    "core_positions_deflection": positions_deflection,
    "core_curvature_comb": curvature_comb_case,
    "core_edge_members11": edge_members11,
    "core_wire_members11": wire_members11,
    "core_face_members11": face_members11,
    "core_axes_of_symmetry": axes_of_symmetry_case,
    "core_split_by_perimeter": split_by_perimeter_case,
    "core_wrap": wrap_case,
    "core_dprism_faces": dprism_faces,
    "core_dprism_wires": dprism_wires,
    "core_hollow_open": hollow_open,
    "core_hollow_closed": lambda: Solid.make_box(10, 10, 10).hollow([], 1.0, 1e-4, Kind.INTERSECTION),
    "core_twist_wire": lambda: Solid.extrude_linear_with_rotation(Wire.make_rect(4, 4), (0, 0, 0), (0, 0, 10), 45,
                                                                 [Wire.make_circle(1)]),
    "core_revolve_wire": lambda: Solid.revolve(Wire.make_rect(2, 2, Plane((5, 0, 0))), 270, Axis.Y,
                                               [Wire.make_circle(0.5, Plane((5, 0, 0)))]),
    "core_sweep_inner": sweep_inner,
    "core_compound_members11": compound_members11,
    "core_sort_wires_by_build_order": sort_wires_case,
    "core_attributes_survive": attributes_survive,
    # 13: never compared before
    "core_mixin1d_queries": mixin1d_queries,
    "core_edge_wire_queries": edge_wire_queries,
    "core_face_queries": face_queries,
    "core_shell_solid_queries": shell_solid_queries,
    "core_shape_queries": shape_queries,
    "core_skip_clean": skip_clean_case,
    "core_plane_helpers": plane_helpers,
}
