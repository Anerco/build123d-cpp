"""Python twins of cases_objects.cpp (objects, operations, text and drafting)."""

from math import cos, exp, sin, sqrt

from scipy.optimize import minimize, minimize_scalar
from scipy.spatial import Voronoi

from build123d import *  # noqa: F403
from build123d.pack_utils import _pack2d
from build123d.topology.utils import tuplify

from values import V


def align_values(align):
    return [a.name for a in align]


class Club(BaseSketchObject):
    """A custom sketch object, as in examples/custom_sketch_objects.py."""

    def __init__(self, size, rotation=0, align=None, mode=Mode.ADD):
        club = (
            Pos(0, size / 3) * Circle(size / 4)
            + Pos(size / 4, 0) * Circle(size / 4)
            + Pos(-size / 4, 0) * Circle(size / 4)
            + Pos(0, -size / 3) * Rectangle(size / 6, size / 2)
        )
        self.club_size = size
        super().__init__(obj=club, rotation=rotation, align=align, mode=mode)


class Bracket(BasePartObject):
    """A custom part object."""

    def __init__(self, size, rotation=(0, 0, 0), align=None, mode=Mode.ADD):
        part = Box(size, size, size / 4, align=Align.MIN) + Box(size / 4, size, size, align=Align.MIN)
        super().__init__(part=part, rotation=rotation, align=align, mode=mode)


def obj_custom_attrs():
    c = Club(10, 45)
    b = Bracket(4, rotation=(0, 0, 30))
    return V({"club_rotation": c.rotation, "bracket_rotation": b.rotation.orientation})


def obj_attrs_sketch():
    circle = Circle(2, 90, align=Align.MIN)
    ellipse = Ellipse(3, 2, 15)
    polygon = Polygon((0, 0), (1, 0), (1, 1), rotation=10, align=Align.CENTER)
    rect = Rectangle(3, 2, 5)
    rr = RectangleRounded(3, 2, 0.5)
    rp = RegularPolygon(3, 6, False, 20)
    sa = SlotArc(CenterArc((0, 0), 5, 0, 90), 1, 10)
    scp = SlotCenterPoint((0, 0), (2, 0), 1, 5)
    sctc = SlotCenterToCenter(3, 1, 7)
    so = SlotOverall(4, 1, 8)
    se = Superellipse(4, 2, 3, 12, 9)
    tz = Trapezoid(5, 2, 60, 0.0, 4)
    tz2 = Trapezoid(5, 2, 60, 75.0)
    tri = Triangle(a=3, b=4, c=5, rotation=12)
    return V(
        {
            "circle": [circle.radius, circle.arc_size, align_values(circle.align), circle.rotation],
            "ellipse": [ellipse.x_radius, ellipse.y_radius, align_values(ellipse.align), ellipse.rotation],
            "polygon": [[Vector(p) for p in polygon.pts], align_values(polygon.align), polygon.rotation],
            "rect": [rect.width, rect.rectangle_height, align_values(rect.align), rect.rotation],
            "rect_rounded": [rr.width, rr.rectangle_height, rr.radius, align_values(rr.align), rr.rotation],
            "regular_polygon": [rp.radius, rp.apothem, rp.side_count, align_values(rp.align), rp.rotation],
            "slot_arc": [isinstance(sa.arc, Edge), sa.slot_height, sa.rotation],
            "slot_center_point": [scp.slot_center, scp.point, scp.slot_height, scp.rotation],
            "slot_center_to_center": [sctc.center_separation, sctc.slot_height, sctc.rotation],
            "slot_overall": [so.width, so.slot_height, so.rotation],
            "superellipse": [se.width, se.height_, se.order, se.point_count, align_values(se.align), se.rotation],
            "trapezoid": [
                tz.width,
                tz.trapezoid_height,
                tz.left_side_angle,
                tz.right_side_angle,
                align_values(tz.align),
                tz.rotation,
                tz2.right_side_angle,
            ],
            "triangle": [tri.a, tri.b, tri.c, tri.A, tri.B, tri.C, tri.rotation],
        }
    )


def obj_attrs_part():
    box = Box(1, 2, 3, rotation=(10, 0, 0))
    cone = Cone(2, 1, 3, 270, align=Align.MIN)
    cp = ConvexPolyhedron([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)], rotation=(0, 0, 5))
    cbh = CounterBoreHole(1, 2, 1, 5)
    csh = CounterSinkHole(1, 2, 5, 90)
    cyl = Cylinder(1, 2, 180, align=(Align.MIN, Align.CENTER, Align.MAX))
    hole = Hole(1, 5)
    sphere = Sphere(2, -45, 60, 270)
    torus = Torus(3, 1, 10, 300, 270)
    wedge = Wedge(1, 2, 3, 0, 0, 1, 1)

    def al(a):
        return [x.name for x in tuplify(a, 3)]

    return V(
        {
            "box": [box.length, box.width, box.box_height, box.rotation.orientation],
            "cone": [cone.bottom_radius, cone.top_radius, cone.cone_height, cone.arc_size, al(cone.align)],
            "convex_polyhedron": [cp.rotation.orientation],
            "counter_bore_hole": [cbh.radius, cbh.counter_bore_radius, cbh.counter_bore_depth, cbh.hole_depth],
            "counter_sink_hole": [csh.radius, csh.counter_sink_radius, csh.hole_depth, csh.counter_sink_angle],
            "cylinder": [cyl.radius, cyl.cylinder_height, cyl.arc_size, al(cyl.align)],
            "hole": [hole.radius, hole.hole_depth],
            "sphere": [sphere.radius, sphere.arc_size1, sphere.arc_size2, sphere.arc_size3, al(sphere.align)],
            "torus": [
                torus.major_radius,
                torus.minor_radius,
                torus.minor_start_angle,
                torus.minor_end_angle,
                torus.major_angle,
                al(torus.align),
            ],
            "wedge": [
                wedge.xsize,
                wedge.ysize,
                wedge.zsize,
                wedge.xmin,
                wedge.zmin,
                wedge.xmax,
                wedge.zmax,
                al(wedge.align),
            ],
        }
    )


def location_values(loc):
    return [loc.position, loc.orientation]


def locations_values(locs):
    return V([location_values(l) for l in locs])


def obj_locations_mixed():
    face = Box(1, 2, 3).faces().sort_by(Axis.X).last
    return locations_values(
        Locations((1, 2), Pos(3, 4) * Rot(0, 0, 30), Axis.Z, Plane.YZ, Vertex(1, 1, 1), face, [(5, 5), (6, 6)])
    )


def obj_locations_faces_shape():
    box = Box(4, 4, 4)
    return box + Locations(box.faces()) * Cylinder(0.5, 1)


def obj_location_list_attrs():
    grid = GridLocations(2, 3, 3, 2, align=(Align.MIN, Align.MAX))
    hex_ = HexLocations(1, 3, 2, major_radius=True, align=Align.MIN)
    polar = PolarLocations(3, 4, 10, 180, rotate=False, endpoint=True)
    locs = Locations((1, 2), (3, 4))
    return V(
        {
            "grid": [
                grid.x_spacing,
                grid.y_spacing,
                grid.x_count,
                grid.y_count,
                align_values(grid.align),
                grid.size,
                grid.min,
                grid.max,
                len(grid.planes),
                [location_values(l) for l in grid.local_locations],
            ],
            "hex": [
                hex_.radius,
                hex_.apothem,
                hex_.diagonal,
                hex_.x_count,
                hex_.y_count,
                hex_.major_radius,
                align_values(hex_.align),
                [location_values(l) for l in hex_.local_locations],
            ],
            "polar": [location_values(l) for l in polar.local_locations],
            "locations": [location_values(l) for l in locs.local_locations],
        }
    )


def edges_values(edges):
    return [[e.length, e @ 0, e @ 1] for e in edges]


def edge_values(e):
    return [e.length, e @ 0, e @ 1]


def optional_edge_values(e):
    return None if e is None else edge_values(e)


def obj_trim_to_other():
    h = sqrt(2) / 2
    bases = [CenterArc((0, 0), 1, 0, 180), CenterArc((0, 0), 1, 0, 360)]
    rect = Rectangle(1, 1, align=(Align.MAX, Align.MIN)).rotate(Axis.Z, -45)
    out = []
    for base in bases:
        others = [
            Line((0, 0), (1, 1)),
            CenterArc((0, 0), 1, 45, 90),
            rect.wire(),
            rect,
            Curve(rect.edges()),
            Box(1, 1, 1, align=(Align.MAX, Align.MIN, Align.CENTER)).rotate(Axis.Z, -45),
            Axis((0, 0, 0), (1, 1, 0)),
            Location((h, h), (0, 0, 1)),
            Plane((0, 0, 0), (1, 1, 0), (-1, 1, 0)),
            Vector(1, 0).rotate(Axis.Z, 45),
            (h, h),
            (0, 2),
            Vertex(h, h, 0),
            Pos(0, 0.5) * Circle(0.8),
            Line((-2, 0.5), (2, 0.5)),
        ]
        out.extend(optional_edge_values(base.trim_to_other(o)) for o in others)
        out.append(optional_edge_values(base.reversed().trim_to_other(Line((0, 0), (1, 1)))))
    return V(out)


def obj_center_arc_limits():
    c1 = CenterArc((1, 0), 1, 0, Line((1, 0), (2, 1)))
    c2 = CenterArc((1, 0), 1, 0, Line((1, 0), (2, -1)))
    c3 = CenterArc((0, 0), 2, 30, Axis.Y)
    c4 = CenterArc((0, 0), 2, 30, Plane.XZ)
    c5 = CenterArc((0, 0), 2, 10, Pos(0, 3) * Circle(1.5))
    c6 = CenterArc((0, 0), 2, 10, (0, -2))
    return V(
        [edge_values(c1), c1 % 0, edge_values(c2), c2 % 0, edge_values(c3), edge_values(c4), edge_values(c5), edge_values(c6)]
    )


def obj_jern_arc_limits():
    j1 = JernArc((1, 0), (0, 1), 1, Line((1, 0), (2, 1)))
    j2 = JernArc((1, 0), (0, 1), 1, Line((1, 0), (0, 1)))
    j3 = JernArc((0, 0), (1, 1), 3, Axis.Y)
    j4 = JernArc((0, 0, 1), (0, 0, 1), 2, Plane.XY.offset(3))
    return V(
        [
            edge_values(j1),
            j1.center_point,
            j1.end_of_arc,
            j1.start,
            edge_values(j2),
            j2.center_point,
            edge_values(j3),
            j3.center_point,
            edge_values(j4),
            j4.center_point,
        ]
    )


def obj_polar_line_limits():
    h = sqrt(2) / 2
    rect = Rectangle(1, 1, align=(Align.MAX, Align.MIN)).rotate(Axis.Z, -45)
    limits = [
        Line((0, 0), (1, 1)),
        CenterArc((0, 0), 1, 45, 90),
        rect.wire(),
        rect,
        Curve(rect.edges()),
        Box(1, 1, 1, align=(Align.MAX, Align.MIN, Align.CENTER)).rotate(Axis.Z, -45),
        Axis((0, 0, 0), (1, 1, 0)),
        Location((h, h), (0, 0, 1)),
        Plane((0, 0, 0), (1, 1, 0), (-1, 1, 0)),
        Vector(1, 0).rotate(Axis.Z, 45),
        (h, h),
    ]
    out = [edge_values(PolarLine((sqrt(2), 0), length=limit, angle=135)) for limit in limits]
    out.append(edge_values(PolarLine(start=(2, 0), length=Axis.Y, angle=135)))
    out.append(edge_values(PolarLine((0, 0), Line((3, -1), (3, 5)), direction=(1, 1))))
    return V(out)


def curve_values(c):
    bb = c.bounding_box()
    return [len(c.edges()), c.length, bb.min, bb.max, c @ 0, c @ 1, isinstance(c, Wire)]


def curves_values(curves):
    return [curve_values(c) for c in curves]


def obj_wire_project_multiple():
    target = Box(10, 10, 4) - Pos((0, 0, 2)) * Box(5, 5, 2)
    circle = Wire.make_circle(3, Plane.XY.offset(10))
    return V(
        [curves_values(circle.project_to_shape(target, (0, 0, -1))), curves_values(circle.project_to_shape(target, (0, 0, 1)))]
    )


def obj_wire_project_conical():
    target = Box(10, 10, 4) - Pos((0, 0, 2)) * Box(5, 5, 2)
    circle = Wire.make_circle(3, Plane.XY.offset(10))
    square = Wire.make_rect(4, 4, Plane.XY.offset(30))
    return V(
        [
            curves_values(circle.project_to_shape(target, center=(0, 0, 20))),
            curves_values(square.project_to_shape(Sphere(10), center=(0, 0, 40))),
            curves_values(Wire.make_circle(2, Plane.XZ.offset(-30)).project_to_shape(Sphere(10), center=(0, -50, 0))),
        ]
    )


def obj_edge_project():
    projection = Edge.make_circle(1, Plane.XY.offset(-5)).project_to_shape(Solid.make_box(1, 1, 1), (0, 0, 1))
    conical = Line((-2, -2, 10), (2, 2, 10)).project_to_shape(Sphere(5), center=(0, 0, 20))
    reversed_ = Edge.make_circle(1, Plane.XY.offset(-5)).reversed().project_to_shape(Solid.make_box(1, 1, 1), (0, 0, 1))
    return V([curves_values(projection), projection[0].arc_center, curves_values(conical), curves_values(reversed_)])


def obj_project_clean_wire():
    center_arc = Edge.make_ellipse(45, 30, start_angle=270 + 40, end_angle=270 - 40)
    center_surface = -Face.extrude(center_arc, (0, 0, 50)).moved(Location((0, 0, -25), (0, 0, 180)))
    tip_center_loc = center_surface.location_at(center_arc @ 1, x_dir=(1, 0, 0))
    normal = tip_center_loc.z_axis.direction
    planar_tip_arc = Edge.make_circle(12.5, start_angle=270, end_angle=450).locate(tip_center_loc)
    return V(curves_values(planar_tip_arc.project_to_shape(center_surface, -normal)))


def obj_mixin1d_project():
    rect = Face.make_rect(10, 10)
    circle = Edge.make_circle(1, Plane.XY.offset(5))
    sphere_face = Sphere(5).faces()[0]
    square = Wire.make_rect(2, 2, Plane.XY.offset(8))
    return V(
        [
            curve_values(circle.project(rect, (0, 0, -1))),
            curves_values(circle.project(rect, (0, 0, -1), closest=False)),
            curve_values(square.project(sphere_face, (0, 0, -1))),
            curves_values(square.project(sphere_face, (0, 0, -1), closest=False)),
            curves_values(Line((-1, 0, 8), (1, 0, 8)).project(sphere_face, (0, 0, -1), closest=False)),
        ]
    )


def obj_project_mixed():
    with_points = project([Rectangle(2, 2).face(), Vertex(1, 2, 3)], workplane=Plane.XY)
    c = project([Rectangle(2, 2).face(), Line((0, 0), (1, 1))], workplane=Plane.XY.offset(1))
    return V([list(with_points), c.area, len(c.faces()), c.center()])


def sample_points(edges, n=100):
    return [[p.X, p.Y] for e in edges for p in e.positions([i / n for i in range(n + 1)])]


def voronoi_values(pts):
    verts = [[float(v[0]), float(v[1])] for v in Voronoi(pts).vertices]
    verts.sort(key=lambda v: (round(v[0], 9), round(v[1], 9)))
    return V(verts)


def bbox_values(shapes):
    return V([[bb.min, bb.max] for bb in (s.bounding_box() for s in shapes)])


def pack2d_values(sizes):
    return V([list(p) for p in _pack2d(sizes, width_fn=lambda s: s[0], length_fn=lambda s: s[1])])


# (width, length) pairs drawn with random.seed(123456) as in upstream test_pack.py
RANDOM_BOXES = [
    (10, 1), (6, 1), (3, 2), (9, 1), (4, 8), (16, 6), (1, 17), (5, 13), (12, 2), (6, 2),
    (15, 17), (15, 15), (17, 6), (17, 18), (2, 4), (3, 4), (17, 3), (3, 5), (11, 19), (10, 15),
    (17, 20), (11, 12), (9, 8), (4, 9), (9, 3), (4, 7), (15, 13), (2, 1), (16, 4), (14, 4),
    (6, 13), (8, 1), (20, 17), (14, 4), (20, 17), (8, 17), (20, 17), (4, 13), (9, 19), (2, 12),
    (14, 4), (2, 17), (8, 2), (7, 6), (18, 8), (12, 12), (7, 3), (17, 11), (18, 17), (9, 19),
]  # fmt: skip
RANDOM_SLOTS = [
    (11, 4), (2, 1), (7, 6), (2, 1), (4, 1), (3, 1), (10, 8), (2, 1), (5, 4), (9, 2),
    (17, 6), (7, 4), (2, 1), (18, 1), (6, 5), (14, 9), (13, 7), (3, 1), (7, 5), (3, 1),
    (16, 9), (18, 17), (16, 12), (16, 15), (18, 4), (7, 6), (18, 13), (19, 9), (3, 1), (5, 3),
    (4, 2), (5, 1), (18, 2), (4, 3), (4, 3), (6, 2), (12, 11), (20, 2), (11, 4), (16, 3),
    (18, 8), (12, 6), (13, 12), (10, 6), (9, 4), (5, 1), (10, 9), (10, 6), (4, 3), (5, 3),
]  # fmt: skip


def obj_full_round_flipped():
    flipped = -Face.make_rect(34, 10)
    rounded = full_round((flipped.edges() << Axis.X)[0])
    return V([flipped.normal_at(), rounded.face().normal_at(), rounded.area])


def obj_full_round_two_edges():
    l1 = Edge.make_spline([(-1, 0), (1, 0)], tangents=((0, -8), (0, 8)), scale=True)
    l2 = Edge.make_line(l1 @ 0, l1 @ 1)
    face = Face(Wire([l1, l2]))
    return full_round(face.edges()[0])


def nelder_mead_values(f, x0, bounds=None, tol=None, maxiter=None):
    options = {} if maxiter is None else {"maxiter": maxiter}
    r = minimize(f, x0=x0, method="Nelder-Mead", bounds=bounds, tol=tol, options=options)
    return {"x": [float(v) for v in r.x], "fun": float(r.fun), "nit": int(r.nit), "nfev": int(r.nfev), "status": int(r.status)}


def obj_nelder_mead():
    def rosen(x):
        return 100 * (x[1] - x[0] * x[0]) ** 2 + (1 - x[0]) ** 2

    return V(
        [
            nelder_mead_values(lambda x: (x[0] - 3) * (x[0] - 3) + 1, 0.0, [(0.0, 10.0)], 1e-6),
            nelder_mead_values(lambda x: abs(sin(x[0]) - 0.5), 0.0, [(0.0, 3.0)], 1e-6),
            nelder_mead_values(lambda x: (x[0] - 5) * (x[0] - 5), 9.99, [(0.0, 10.0)]),
            nelder_mead_values(rosen, [-1.2, 1.0]),
            nelder_mead_values(rosen, [0.4, 0.4], [(0.0, 0.5), (-1.0, 1.0)], 1e-8),
            nelder_mead_values(rosen, [-1.2, 1.0], maxiter=15),
        ]
    )


def blend_edges():
    m1 = Pos(-10, 3) * CenterArc((0, 0), 5, -10, 200).reversed()
    m2 = Pos(5, -13) * Spline((-3, 9), (0, 0), (3, 9))
    return m1, m2


def obj_blend_c1_c2_scalars():
    m1, m2 = blend_edges()
    out = []
    for c in [ContinuityLevel.C1, ContinuityLevel.C2]:
        bc = BlendCurve(m1, m2, c, end_points=(m1 @ 1, m2 @ 0), tangent_scalars=(1.7, 0.8))
        out.append(
            [
                bc.length,
                bc.derivative_at(0, 1),
                bc.derivative_at(1, 1),
                bc.derivative_at(0, 2),
                bc.derivative_at(1, 2),
                bc @ 0.3,
            ]
        )
    return V(out)


def obj_blend_bad_end_point():
    m1 = CenterArc((0, 0), 5, -10, 200)
    return BlendCurve(m1, Line((10, 0), (12, 3)), ContinuityLevel.C1, end_points=(m1 @ 0.5, (10, 0)))


def obj_airfoil_attrs():
    a = Airfoil("2412", 40)
    m, p, t = Airfoil.parse_naca4("NACA 2213.323")
    m2, p2, t2 = Airfoil.parse_naca4(6409.0)
    return V([a.code, a.max_camber, a.camber_pos, a.thickness, a.finite_te, m, p, t, m2, p2, t2])


def obj_carcs_2tan_rad_lines():
    e1, e2 = Line((-2, 0), (2, 0)), Line((0, -2), (0, 2))
    return V(
        [
            edges_values(Edge.make_constrained_arcs(e1, e2, radius=0.5, sagitta=Sagitta.BOTH)),
            edges_values(Edge.make_constrained_arcs(e1, e2, radius=0.5)),
            edges_values(
                Edge.make_constrained_arcs(
                    (e1, Tangency.UNQUALIFIED), (e2, Tangency.UNQUALIFIED), radius=0.5, sagitta=Sagitta.LONG
                )
            ),
        ]
    )


def obj_carcs_2tan_rad_points():
    return V(
        [
            edges_values(Edge.make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), radius=0.5)),
            edges_values(Edge.make_constrained_arcs(Vertex(0, 0), Vertex(0, 0.5), radius=0.5)),
            edges_values(Edge.make_constrained_arcs((0, 0.5), Line((0, 0), (1, 0)), radius=0.5)),
        ]
    )


def obj_carcs_2tan_on():
    c1 = PolarLine((0, 0), 4, -20, length_mode=LengthMode.HORIZONTAL)
    c2, on = Line((4, -2), (4, 2)), Line((3, -2), (3, 2))
    return V(
        [
            edges_values(
                Edge.make_constrained_arcs((c1, Tangency.UNQUALIFIED), (c2, Tangency.UNQUALIFIED), center_on=on)
            ),
            edges_values(Edge.make_constrained_arcs(Vector(0, 3), Vector(5, 0), center_on=Line((0, -5), (0, 5)))),
            edges_values(Edge.make_constrained_arcs(Line((-5, 3), (5, 3)), Vector(5, 0), center_on=Axis.Y)),
            edges_values(
                Edge.make_constrained_arcs(
                    Line((-10, 0), (10, 0)),
                    Line((0, -10), (0, 10)),
                    center_on=Line((-10, 1), (10, 1)),
                    sagitta=Sagitta.BOTH,
                )
            ),
            edges_values(
                Edge.make_constrained_arcs(
                    Line((-10, 0), (10, 0)),
                    Line((0, -10), (0, 10)),
                    center_on=Line((3, -10), (3, 10)),
                    sagitta=Sagitta.LONG,
                )
            ),
            edges_values(
                Edge.make_constrained_arcs(
                    Line((0, 0), (0.01, 0)), Line((10, -10), (10, 10)), center_on=Line((5, -10), (5, 10))
                )
            ),
        ]
    )


def obj_carcs_2tan_on_trim():
    c1 = CenterArc((0, 20), 20, -90, 90)
    ln2 = Line(c1 @ 1, (10, 20))
    ln3 = Line((10, 10), ln2 @ 1)
    return ConstrainedArcs(c1, ln2, center_on=ln3)


def obj_carcs_tan_cen():
    return V(
        [
            edges_values(Edge.make_constrained_arcs((PolarLine((0, 0), 4, 60), Tangency.UNQUALIFIED), center=(2, 1))),
            edges_values(Edge.make_constrained_arcs(Axis.X, center=(2, 1, 5))),
            edges_values(Edge.make_constrained_arcs(CenterArc((0, 0), 1, 180, 5), center=(2, 0))),
            edges_values(Edge.make_constrained_arcs((-2.5, 1.5), center=(-2, 1))),
            edges_values(Edge.make_constrained_arcs((-2.5, 1.5), center=Vertex(-2, 1))),
            edges_values(Edge.make_constrained_arcs((-2.5, 1.5), center=Pos(1, 0) * Vertex(-2, 1))),
            edges_values(Edge.make_constrained_arcs(Vector(0, 0), center=Vector(0, 0))),
        ]
    )


def obj_carcs_tan_on_rad():
    c1 = PolarLine((0, 0), 4, -20, length_mode=LengthMode.HORIZONTAL)
    return V(
        [
            edges_values(
                Edge.make_constrained_arcs((c1, Tangency.UNQUALIFIED), radius=1, center_on=Line((3, -2), (3, 2)))
            ),
            edges_values(Edge.make_constrained_arcs(c1, radius=1, center_on=Axis.X)),
            edges_values(Edge.make_constrained_arcs(Vector(1, 1), radius=2, center_on=Axis.X)),
        ]
    )


def obj_carcs_3tan():
    c5, c6 = PolarLine((0, 0), 4, 60), PolarLine((0, 0), 4, 40)
    c7 = CenterArc((0, 0), 4, 0, 90)
    a1, a2 = CenterArc((0, 20), 20, -90, 90), CenterArc((5, 25), 10, -90, 90)
    return V(
        [
            edges_values(
                Edge.make_constrained_arcs(
                    (c5, Tangency.UNQUALIFIED), (c6, Tangency.UNQUALIFIED), (c7, Tangency.UNQUALIFIED)
                )
            ),
            edges_values(Edge.make_constrained_arcs(c5, c6, c7, sagitta=Sagitta.BOTH)),
            edges_values(
                Edge.make_constrained_arcs(Line((-1, 0), (-1, 2)), Line((1, 0), (1, 2)), Line((-1, 0), (-0.75, 0)))
            ),
            edges_values(Edge.make_constrained_arcs(a1, a2, Line(a1 @ 1, a2 @ 1))),
            edges_values(Edge.make_constrained_arcs(a1, a2, RadiusArc(a1 @ 1, a2 @ 1, 10))),
        ]
    )


def obj_carcs_eggplant():
    left, right = CenterArc((-2, 0), 0.75, 0, 360), CenterArc((2, 0), 1.0, 0, 360)
    bottom = Edge.make_constrained_arcs((right, Tangency.OUTSIDE), (left, Tangency.OUTSIDE), radius=6).sort_by(Axis.Y)[0]
    top = Edge.make_constrained_arcs((right, Tangency.ENCLOSING), (left, Tangency.ENCLOSING), radius=8).sort_by(
        Axis.Y
    )[-1]
    r = ThreePointArc(bottom.vertices().sort_by(Axis.X)[-1], right @ 0, top.vertices().sort_by(Axis.X)[-1])
    l = ThreePointArc(bottom.vertices().sort_by(Axis.X)[0], left @ 0.5, top.vertices().sort_by(Axis.X)[0])
    return make_face([l, top, r, bottom])


def obj_clines():
    unit = Edge.make_circle(1.0, Plane.XY)
    moved = unit.translate((3, 0, 0))
    return V(
        [
            edges_values(Edge.make_constrained_lines(unit, moved)),
            edges_values(Edge.make_constrained_lines((unit, Tangency.ENCLOSING), moved)),
            edges_values(Edge.make_constrained_lines((unit, Tangency.ENCLOSING), (moved, Tangency.ENCLOSING))),
            edges_values(Edge.make_constrained_lines(unit, Vector(2, 0))),
            edges_values(Edge.make_constrained_lines(unit, (3, 0))),
            edges_values(Edge.make_constrained_lines(unit, unit)),
            edges_values(Edge.make_constrained_lines(unit, Axis.X, angle=30)),
            edges_values(Edge.make_constrained_lines(unit, Axis.Y, angle=30)),
            edges_values(Edge.make_constrained_lines((unit, Tangency.ENCLOSING), Axis.Y, angle=30)),
            edges_values(Edge.make_constrained_lines((unit, Tangency.ENCLOSING), Axis.Y, direction=(1, 1))),
            edges_values(Edge.make_constrained_lines(unit, Axis.X, angle=0)),
            edges_values(Edge.make_constrained_lines(unit, Axis((1, 0), (0, 1)), angle=0)),
            edges_values(Edge.make_constrained_lines(Vector(0, 6), CenterArc((-5, 0), 4, 0, 360))),
            edges_values(Edge.make_constrained_lines(Axis.X, Edge.make_circle(1.0, Plane.XY.offset(0)).translate((0, 3)))),
        ]
    )


CASES = {
    # base classes for custom objects
    "obj_base_sketch": lambda: BaseSketchObject(
        Rectangle(4, 2) + Pos(2, 0) * Circle(1), rotation=30, align=(Align.MIN, Align.MAX)
    ),
    "obj_base_sketch_plain": lambda: BaseSketchObject(Rectangle(4, 2).face()),
    "obj_base_part": lambda: BasePartObject(
        Solid.make_box(1, 2, 3), rotation=(10, 20, 30), align=(Align.MIN, Align.CENTER, Align.MAX)
    ),
    "obj_base_part_plain": lambda: BasePartObject(Pos(1, 2, 3) * Cylinder(1, 2)),
    "obj_base_curve": lambda: BaseCurveObject(Curve([Line((0, 0), (1, 0)), Line((1, 0), (1, 1))])),
    "obj_base_line": lambda: BaseLineObject(Polyline((0, 0), (1, 0), (1, 1))),
    "obj_base_edge": lambda: BaseEdgeObject(Edge.make_line((0, 0), (3, 4))),
    "obj_custom_club": lambda: Club(10, 45, align=(Align.MIN, Align.MIN)),
    "obj_custom_bracket": lambda: Bracket(4, rotation=(0, 0, 30), align=Align.CENTER),
    "obj_custom_attrs": obj_custom_attrs,
    # object attributes
    "obj_attrs_sketch": obj_attrs_sketch,
    "obj_attrs_part": obj_attrs_part,
    "obj_hole_no_depth": lambda: Hole(1),
    "obj_counter_bore_no_depth": lambda: CounterBoreHole(1, 2, 1),
    "obj_counter_sink_no_depth": lambda: CounterSinkHole(1, 2),
    # Locations
    "obj_locations_faces": lambda: locations_values(Locations((Rot(10, 20, 30) * Box(1, 2, 3)).faces())),
    "obj_locations_faces_explicit": lambda: locations_values(Locations(*(Pos(1, 0, 0) * Cylinder(1, 2)).faces())),
    "obj_locations_vertices": lambda: locations_values(Locations((Pos(1, 2, 3) * Box(1, 2, 3)).vertices())),
    "obj_locations_planes": lambda: locations_values(Locations([Plane.XY, Plane.XZ.offset(2), Plane.isometric])),
    "obj_locations_axes": lambda: locations_values(Locations(Axis.X, Axis((1, 2, 3), (0, 1, 1)))),
    "obj_locations_axis_list": lambda: locations_values(Locations([Axis.Y, Axis.Z])),
    "obj_locations_mixed": obj_locations_mixed,
    "obj_locations_faces_shape": obj_locations_faces_shape,
    "obj_locations_mixed_shape": lambda: Sketch()
    + Locations((0, 0), Pos(5, 0) * Rot(0, 0, 45), Plane.XY.offset(1), Axis((-5, 0, 0), (0, 0, 1)))
    * Rectangle(2, 1),
    "obj_location_list_attrs": obj_location_list_attrs,
    # scipy's Nelder-Mead
    "obj_nelder_mead": obj_nelder_mead,
    "obj_minimize_scalar_bounded": lambda: V(
        [
            [float(r.x), float(r.fun), int(r.nfev), int(r.status)]
            for r in [
                minimize_scalar(lambda x: (x - 0.3) * (x - 0.3), bounds=(0, 1), method="bounded", options={"xatol": 5e-7}),
                minimize_scalar(lambda x: abs(sin(5 * x) - 0.2), bounds=(0, 1), method="bounded", options={"xatol": 1e-5}),
                minimize_scalar(lambda x: cos(x), bounds=(0.5, 0.75), method="bounded", options={"xatol": 5e-7}),
                minimize_scalar(lambda x: exp(x), bounds=(-2, 3), method="bounded", options={"xatol": 1e-5}),
            ]
        ]
    ),
    # BlendCurve
    "obj_blend_c0": lambda: BlendCurve(Line((0, 0), (1, 0)), Line((2, 0), (2, 1)), ContinuityLevel.C0),
    "obj_blend_c2_auto": lambda: BlendCurve(*blend_edges()),
    "obj_blend_c1_c2_scalars": obj_blend_c1_c2_scalars,
    "obj_blend_bad_end_point": obj_blend_bad_end_point,
    # DoubleTangentArc
    "obj_double_tangent_line": lambda: DoubleTangentArc((0, 0), (1, 0), Line((0, 5), (10, 5))),
    "obj_double_tangent_bottom": lambda: DoubleTangentArc((0, 0), (1, 0), Line((0, 5), (10, 5)), keep=Keep.BOTTOM),
    "obj_double_tangent_wire": lambda: DoubleTangentArc((0, 0), (1, 1), Polyline((6, 0), (6, 8), (10, 12))),
    "obj_double_tangent_arc_target": lambda: DoubleTangentArc((0, 0), (0, 1), CenterArc((10, 0), 3, 90, 180)),
    "obj_double_tangent_spline": lambda: DoubleTangentArc(
        (0, 0), (1, 0), Spline((-5, 8), (0, 6), (6, 9)), keep=Keep.BOTTOM
    ),
    "obj_double_tangent_curve_error": lambda: DoubleTangentArc((0, 0), (1, 0), Curve([Line((0, 5), (10, 5))])),
    "obj_double_tangent_keep_error": lambda: DoubleTangentArc((0, 0), (1, 0), Line((0, 5), (10, 5)), keep=Keep.BOTH),
    # Airfoil
    "obj_airfoil_2412": lambda: Airfoil("2412"),
    "obj_airfoil_finite": lambda: Airfoil("NACA 2412", 40, True),
    "obj_airfoil_0012": lambda: Airfoil("0012", 60),
    "obj_airfoil_fraction": lambda: Airfoil("2213.323", 30),
    "obj_airfoil_number": lambda: Airfoil(4415.0, 25, True),
    "obj_airfoil_camber": lambda: Airfoil("4412", 40).camber_line,
    "obj_airfoil_attrs": obj_airfoil_attrs,
    "obj_airfoil_bad_code": lambda: Airfoil("2x12"),
    # ConstrainedArcs / Edge.make_constrained_arcs
    "obj_carcs_2tan_rad_lines": obj_carcs_2tan_rad_lines,
    "obj_carcs_2tan_rad_arc_line": lambda: V(
        edges_values(Edge.make_constrained_arcs(CenterArc((0, 0), 1, 0, 90), Line((1, 0), (2, 0)), radius=0.5))
    ),
    "obj_carcs_2tan_rad_points": obj_carcs_2tan_rad_points,
    "obj_carcs_2tan_rad_none": lambda: Curve(Edge.make_constrained_arcs((0, 0), (10, 0), radius=2)),
    "obj_carcs_2tan_on": obj_carcs_2tan_on,
    "obj_carcs_2tan_on_none": lambda: Curve(
        Edge.make_constrained_arcs(Line((-5, 3), (5, 3)), Line((-5, 0), (5, 0)), center_on=Line((-5, -1), (5, -1)))
    ),
    "obj_carcs_2tan_on_trim": obj_carcs_2tan_on_trim,
    "obj_carcs_tan_cen": obj_carcs_tan_cen,
    "obj_carcs_tan_on_rad": obj_carcs_tan_on_rad,
    "obj_carcs_tan_on_rad_none": lambda: Curve(
        Edge.make_constrained_arcs(Line((0, 10), (10, 10)), radius=1, center_on=Axis.X)
    ),
    "obj_carcs_3tan": obj_carcs_3tan,
    "obj_carcs_3tan_parallel": lambda: Curve(
        Edge.make_constrained_arcs(Line((0, 0), (10, 0)), Line((0, 2), (10, 2)), Line((0, 5), (10, 5)))
    ),
    "obj_carcs_eggplant": obj_carcs_eggplant,
    "obj_carcs_object_selector": lambda: ConstrainedArcs(
        Edge.make_circle(1.0, Plane.XY), Axis.Y, radius=1, selector=lambda a: a.group_by(Axis.Y)[-1]
    ),
    "obj_carcs_object_all": lambda: ConstrainedArcs(CenterArc((-5, 0), 4, 0, 360), CenterArc((5, 0), 3, 0, 360), radius=10),
    "obj_carcs_object_first": lambda: ConstrainedArcs(
        PolarLine((0, 0), 4, -20, length_mode=LengthMode.HORIZONTAL),
        Line((4, -2), (4, 2)),
        center_on=Axis((3, 0), (0, 1)),
        selector=lambda arcs: arcs[0],
    ),
    "obj_carcs_object_shortest": lambda: ConstrainedArcs(
        PolarLine((0, 0), 4, 60),
        PolarLine((0, 0), 4, 40),
        CenterArc((0, 0), 4, 0, 90),
        sagitta=Sagitta.BOTH,
        selector=lambda arcs: arcs.sort_by(Edge.length)[0],
    ),
    "obj_carcs_object_center": lambda: ConstrainedArcs(Axis.Y, center=(-2, 1), selector=lambda arcs: arcs[0]),
    "obj_carcs_object_tan_on_rad": lambda: ConstrainedArcs(
        PolarLine((0, 0), 5, -20), radius=1, center_on=Line((3, -2), (3, 2))
    ),
    "obj_carcs_object_empty_selection": lambda: ConstrainedArcs(
        Edge.make_circle(1.0, Plane.XY), Axis.Y, radius=1, selector=lambda l: l.filter_by(GeomType.LINE)
    ),
    "obj_carcs_axis_tangency_error": lambda: Curve(
        Edge.make_constrained_arcs((Axis.X, Tangency.ENCLOSING), CenterArc((0, 5), 2, 0, 360), radius=3)
    ),
    "obj_carcs_ambiguous_error": lambda: Curve(
        Edge.make_constrained_arcs((0, 0), (0, 0.5), radius=0.5, center=(0, 0.25))
    ),
    "obj_carcs_radius_error": lambda: Curve(Edge.make_constrained_arcs((0, 0), (0, 0.5), radius=-0.5)),
    "obj_carcs_center_on_face_error": lambda: Curve(
        Edge.make_constrained_arcs(
            PolarLine((0, 0), 4, -20, length_mode=LengthMode.HORIZONTAL), radius=1, center_on=Vector(1, 1)
        )
    ),
    "obj_carcs_3tan_points_error": lambda: Curve(
        Edge.make_constrained_arcs(Line((0, 0), (1, 0)), Line((0, 0), (0, 1)), Vector(3, 3))
    ),
    # ConstrainedLines / Edge.make_constrained_lines
    "obj_clines": obj_clines,
    "obj_clines_none": lambda: Curve(Edge.make_constrained_lines(Edge.make_circle(1.0, Plane.XY), Vector(0, 0))),
    "obj_clines_vertical_ref_error": lambda: Curve(
        Edge.make_constrained_lines(Edge.make_circle(1.0, Plane.XY), Axis.Z, angle=1)
    ),
    "obj_clines_two_points_error": lambda: Curve(Edge.make_constrained_lines(Vector(0, 0), Vector(1, 1))),
    "obj_clines_object_all": lambda: ConstrainedLines(CenterArc((-5, 0), 4, 0, 360), CenterArc((5, 0), 3, 0, 360)),
    "obj_clines_object_first": lambda: ConstrainedLines(
        CenterArc((-5, 0), 4, 0, 360), (0, 6), selector=lambda lines: lines[0]
    ),
    "obj_clines_object_closest": lambda: ConstrainedLines(
        CenterArc((-5, 0), 4, 0, 360), Axis.Y, angle=30, selector=lambda lines: lines.sort_by_distance((0, 0))[0]
    ),
    # limits
    "obj_trim_to_other": obj_trim_to_other,
    "obj_trim_infinite": lambda: V(
        [
            edge_values(Edge(Axis.X).trim_infinite(5)),
            edge_values(Edge(Axis((1, 2, 3), (0, 1, 1))).trim_infinite(2)),
            edge_values(Line((0, 0), (1, 1)).trim_infinite(5)),
            Edge(Axis.Y).is_infinite,
        ]
    ),
    "obj_center_arc_limits": obj_center_arc_limits,
    "obj_center_arc_limit_error": lambda: CenterArc((1, 0), 1, 0, (5, 0)),
    "obj_conic_arc_limits": lambda: V(
        [
            edge_values(EllipticalCenterArc((0, 0), 2, 1, 0, arc_size=Line((0, 0), (0, 2)))),
            edge_values(EllipticalCenterArc((0, 0), 2, 1, 0, arc_size=Line((0, 0), (0, -2)))),
            edge_values(EllipticalCenterArc((1, 1), 3, 1, 0, arc_size=Axis((1, 1), (1, 1)), rotation=20)),
            edge_values(ParabolicCenterArc((0, 0), 0.5, 0, arc_size=Line((0, 1), (5, 1)))),
            edge_values(ParabolicCenterArc((0, 0), 0.5, 0, arc_size=Line((1, -2), (1, 2)))),
            edge_values(ParabolicCenterArc((0, 0), 0.5, -30, arc_size=Axis.X, rotation=15)),
            edge_values(HyperbolicCenterArc((0, 0), 2, 1, 0, arc_size=Line((0, 1), (10, 1)))),
            edge_values(HyperbolicCenterArc((0, 0), 2, 1, 0, arc_size=Line((3, -2), (3, 2)))),
            edge_values(HyperbolicCenterArc((0, 0), 1, 2, 0, arc_size=Line((-5, 3), (5, 3)), rotation=10)),
        ]
    ),
    "obj_elliptical_limit_error": lambda: EllipticalCenterArc((0, 0), 2, 1, 0, arc_size=(0, 5)),
    "obj_parabolic_limit_error": lambda: ParabolicCenterArc((0, 0), 0.5, 0, arc_size=(0, 5)),
    "obj_hyperbolic_limit_error": lambda: HyperbolicCenterArc((0, 0), 2, 1, 0, arc_size=(0, 5)),
    "obj_jern_arc_limits": obj_jern_arc_limits,
    "obj_jern_arc_limit_error": lambda: JernArc((1, 0), (0, 1), 1, (5, 0)),
    "obj_polar_line_limits": obj_polar_line_limits,
    "obj_polar_line_limit_point_error": lambda: PolarLine((sqrt(2), 0), length=(0, 2), angle=135),
    "obj_polar_line_limit_shape_error": lambda: PolarLine((sqrt(2), 0), length=Line((3, 0), (3, 1)), angle=135),
    "obj_polar_line_limit_behind_error": lambda: PolarLine((0, 0), Axis.Y, 0.0),
    "obj_polar_line_limit_location_aligned": lambda: PolarLine(
        (0, 0), Location(Plane((3, 0), z_dir=(1, 0, 0))), 0.0
    ),
    # projection
    "obj_wire_project_multiple": obj_wire_project_multiple,
    "obj_wire_project_conical": obj_wire_project_conical,
    "obj_edge_project": obj_edge_project,
    "obj_project_to_shape_no_direction": lambda: Compound(Wire.make_circle(1).project_to_shape(Sphere(50))),
    "obj_project_to_shape_both": lambda: Compound(
        Wire.make_circle(1).project_to_shape(Sphere(50), (0, 0, 1), (0, 0, 5))
    ),
    "obj_project_clean_wire": obj_project_clean_wire,
    "obj_mixin1d_project": obj_mixin1d_project,
    "obj_project_faces": lambda: project(Box(4, 6, 8).rotate(Axis.X, 30).faces(), workplane=Plane.XY.offset(-10)),
    "obj_project_face": lambda: project(
        Plane.XZ.offset(-5) * Rot(0, 30, 0) * Rectangle(4, 3).face(), workplane=Plane.XY
    ),
    "obj_project_face_ignores_target": lambda: project(Circle(3).face(), workplane=Plane.XY.offset(7), target=Box(1, 1, 1)),
    "obj_project_edges": lambda: project(Line((0, 0, 0), (5, 5, 0)), workplane=Plane.XY.offset(5)),
    "obj_project_sketch_ignored": lambda: project(Circle(3), workplane=Plane.XY.offset(5)),
    "obj_project_points": lambda: V(
        [
            list(project(Vector(1, 2, 3), Plane.XY)),
            list(project(Vertex(1, 2, 3), Plane.XZ)),
            list(project([Vector(1, 2, 3), Vector(4, 5, 6)], Plane.XY.offset(2))),
            list(project([Vertex(1, 2, 3), Vertex(-1, 0, 2)], Plane((1, 1, 1), (1, 0, 0), (0, 1, 1)))),
        ]
    ),
    "obj_project_mixed": obj_project_mixed,
    "obj_project_empty_error": lambda: project([], workplane=Plane.XY),
    # Voronoi vertices and full_round
    "obj_voronoi_rect": lambda: voronoi_values(sample_points(Rectangle(34, 10).edges().sort_by(Axis.X)[0:3])),
    "obj_voronoi_trapezoid": lambda: voronoi_values(
        sample_points([Trapezoid(0.5, 1, 82).edges().sort_by(Axis.Y)[i] for i in (3, 1, 2)])
    ),
    "obj_voronoi_triangle": lambda: voronoi_values(
        sample_points(Triangle(a=10, b=10, c=10).edges().sort_by(Axis.X)[0:3])
    ),
    "obj_voronoi_arcs": lambda: voronoi_values(
        sample_points([CenterArc((0, 0), 5, 0, 90), Line((0, 5), (-6, 5)), Spline((5, 0), (6, -3), (9, -4))], 40)
    ),
    "obj_full_round_trap_top": lambda: full_round(Trapezoid(0.5, 1, 90 - 8).edges().sort_by(Axis.Y)[-1]),
    "obj_full_round_trap_short": lambda: full_round(Trapezoid(0.5, 1, 90 - 8).edges().sort_by(SortBy.LENGTH)[0]),
    "obj_full_round_trap_invert": lambda: full_round(
        Trapezoid(0.5, 1, 90 - 8).edges().sort_by(SortBy.LENGTH)[0], invert=True
    ),
    "obj_full_round_trap_side": lambda: full_round(Trapezoid(0.5, 1, 90 - 8).edges().sort_by(Axis.X)[-1]),
    "obj_full_round_rect": lambda: full_round((Rectangle(34, 10).edges() << Axis.X)[0]),
    "obj_full_round_rect_invert": lambda: full_round((Rectangle(34, 10).edges() << Axis.X)[0], invert=True),
    "obj_full_round_rect_points": lambda: full_round((Rectangle(34, 10).edges() << Axis.X)[0], voronoi_point_count=37),
    "obj_full_round_triangle": lambda: full_round(Triangle(a=10, b=10, c=10).edges().sort_by(Axis.X)[0]),
    "obj_full_round_flipped": obj_full_round_flipped,
    "obj_full_round_slot_hole": lambda: full_round(((Rectangle(30, 10) - Pos(5, 0) * Circle(2)).edges() << Axis.X)[0]),
    "obj_full_round_no_parent": lambda: full_round(Edge.make_line((0, 0), (1, 0))),
    "obj_full_round_two_edges": obj_full_round_two_edges,
    # make_brake_formed
    "obj_brake_fillet_polyline": lambda: make_brake_formed(0.5, 1, FilletPolyline((0, 0), (5, 6), (10, 1), radius=1)),
    "obj_brake_widths": lambda: make_brake_formed(
        thickness=0.5, station_widths=[1, 2, 3, 4], line=FilletPolyline((0, 0), (5, 6), (10, 1), radius=1)
    ),
    "obj_brake_right": lambda: make_brake_formed(
        0.5, 1, FilletPolyline((0, 0), (5, 6), (10, 1), radius=1), side=Side.RIGHT
    ),
    "obj_brake_intersection": lambda: make_brake_formed(
        0.5, 1.5, Polyline((0, 0), (5, 6), (10, 1)), kind=Kind.INTERSECTION
    ),
    "obj_brake_tangent": lambda: make_brake_formed(0.5, 1.5, Polyline((0, 0), (5, 6), (10, 1)), kind=Kind.TANGENT),
    "obj_brake_polyline": lambda: make_brake_formed(0.5, 2, Polyline((0, 0), (5, 6), (10, 1))),
    "obj_brake_polyline_widths": lambda: make_brake_formed(0.5, [1, 2, 3], Polyline((0, 0), (5, 6), (10, 1))),
    "obj_brake_arc": lambda: make_brake_formed(0.5, 1, Edge.make_circle(5, start_angle=0, end_angle=60)),
    "obj_brake_straight": lambda: make_brake_formed(0.5, 1, Edge.make_line((0, 0), (5, 0))),
    "obj_brake_wrong_count": lambda: make_brake_formed(0.5, [1, 2], FilletPolyline((0, 0), (5, 6), (10, 1), radius=1)),
    "obj_brake_no_clean": lambda: make_brake_formed(
        0.5, 1, FilletPolyline((0, 0), (5, 6), (10, 1), radius=1), clean=False
    ),
    "obj_brake_curve": lambda: make_brake_formed(0.5, 2, Curve() + Polyline((0, 0), (5, 6), (10, 1))),
    "obj_brake_curve_steps": lambda: make_brake_formed(
        0.3, 1.5, Curve() + [Line((0, 0), (4, 0)), Line((4, 0), (6, 3)), Line((6, 3), (10, 3))]
    ),
    "obj_brake_xz": lambda: make_brake_formed(0.5, 1, Plane.XZ * FilletPolyline((0, 0), (5, 6), (10, 1), radius=1)),
    "obj_brake_spline": lambda: make_brake_formed(0.4, 1, Spline((0, 0), (3, 2), (6, -1), (9, 0))),
    "obj_brake_mixed_right": lambda: make_brake_formed(
        0.4,
        [1, 2, 1],
        Wire([Edge.make_line((0, 0), (4, 0)), Edge.make_tangent_arc((4, 0), (1, 0), (6, 2))]),
        side=Side.RIGHT,
    ),
    # pack
    "obj_pack2d_growth": lambda: pack2d_values([(2, 10), (3, 2)]),
    "obj_pack2d_ties": lambda: pack2d_values(
        [(3, 3), (2, 5), (5, 2), (3, 3), (1, 1), (4, 4), (2, 2), (5, 1), (1, 5), (0.5, 7)]
    ),
    "obj_pack2d_random": lambda: pack2d_values(RANDOM_BOXES),
    "obj_pack_simple": lambda: bbox_values(pack([Box(10, 2, 1), Box(1, 5, 1), Box(1, 5, 1)], padding=1)),
    "obj_pack_random_boxes": lambda: bbox_values(pack([Box(w, l, 1) for w, l in RANDOM_BOXES], 1)),
    "obj_pack_random_boxes_part": lambda: Part() + pack([Box(w, l, 1) for w, l in RANDOM_BOXES], 1),
    "obj_pack_random_slots": lambda: bbox_values(pack([SlotOverall(w, h) for w, h in RANDOM_SLOTS], 1)),
    "obj_pack_align_z": lambda: bbox_values(
        pack(
            [Pos(5, 5, 5) * Box(1, 2, 3), Pos(-3, 0, -2) * Sphere(1), Circle(2), Edge.make_line((0, 0), (3, 4))],
            0.5,
            align_z=True,
        )
    ),
    "obj_pack_vertices": lambda: bbox_values(pack([Vertex(1, 2, 3), Vertex(4, 5, 6)], 1)),
    "obj_pack_empty": lambda: bbox_values(pack([], 1)),
    "obj_pack_overlap": lambda: bbox_values(pack([Box(2, 2, 2), Box(2, 2, 2), Box(3, 1, 1)], -1.5)),
    "obj_pack_empty_part": lambda: bbox_values(pack([Part()], 1)),
    # operation signatures and inputs (lists, empty inputs)
    "obj_ops_extrude_empty_sketch": lambda: extrude(Sketch(), 1),
    "obj_ops_extrude_empty_list": lambda: extrude([], 1),
    "obj_ops_extrude_tuple_both": lambda: extrude(
        (Face.make_rect(1, 1), Pos(3, 0) * Face.make_rect(1, 1)), 1, both=True
    ),
    "obj_ops_extrude_list_until": lambda: extrude(
        [Face.make_rect(2, 2), Pos(5, 0) * Face.make_rect(1, 1)], until=Until.NEXT, target=Pos(0, 0, 5) * Box(20, 20, 1)
    ),
    "obj_ops_extrude_list_until_last": lambda: extrude(
        [Face.make_rect(2, 2), Pos(5, 0) * Face.make_rect(1, 1)],
        until=Until.LAST,
        target=Pos(0, 0, 5) * Box(20, 20, 1) + Pos(0, 0, 9) * Box(20, 20, 1),
    ),
    "obj_ops_extrude_empty_until": lambda: extrude(Sketch(), until=Until.NEXT),
    "obj_ops_extrude_until_no_target": lambda: extrude(Face.make_rect(1, 1), until=Until.NEXT),
    "obj_ops_revolve_sketches": lambda: revolve(
        [Plane.XZ * (Pos(3, 0) * Rectangle(1, 1)), Plane.XZ * (Pos(6, 0) * Circle(0.5))]
    ),
    "obj_ops_revolve_mixed": lambda: revolve(
        [Plane.XZ * (Pos(3, 0) * Rectangle(1, 1)), (Plane.XZ * (Pos(6, 0) * Circle(0.5))).face()], Axis.Z, 90
    ),
    "obj_ops_revolve_empty_sketch": lambda: revolve(Sketch()),
    "obj_ops_revolve_empty_list": lambda: revolve([]),
    "obj_ops_thicken_list": lambda: thicken([Face.make_rect(1, 1), Pos(3, 0) * Face.make_rect(1, 1)], 1),
    "obj_ops_thicken_list_both": lambda: thicken(
        [Face.make_rect(1, 1), Pos(3, 0) * Face.make_rect(1, 1)], 0.5, both=True
    ),
    "obj_ops_thicken_empty_sketch": lambda: thicken(Sketch(), 1),
    "obj_ops_thicken_empty_list": lambda: thicken([], 1),
    "obj_ops_draft_single_face": lambda: draft(
        Box(10, 10, 10).faces().filter_by(Axis.Z, reverse=True)[0], Plane.XY.offset(-5), 5
    ),
    "obj_ops_make_face_mixed": lambda: make_face(
        [Wire.make_polygon([(0, 0), (3, 0), (3, 2)], close=False), Curve() + Line((3, 2), (0, 0))]
    ),
    "obj_ops_make_face_curves": lambda: make_face([Curve() + Polyline((0, 0), (3, 0), (3, 2), (0, 0))]),
    "obj_ops_make_face_wires": lambda: make_face(
        [Wire.make_polygon([(0, 0), (4, 0)], close=False), Wire.make_polygon([(4, 0), (2, 3), (0, 0)], close=False)]
    ),
    "obj_ops_make_face_empty": lambda: make_face([]),
    "obj_ops_trace_mixed": lambda: trace(
        [Wire.make_polygon([(0, 0), (3, 0), (3, 2)], close=False), Curve() + Line((5, 5), (8, 5))], 0.5
    ),
    "obj_ops_trace_wires": lambda: trace([Wire.make_polygon([(0, 0), (3, 0), (3, 2)], close=False)]),
    "obj_ops_trace_empty": lambda: trace([]),
    "obj_ops_offset_default": lambda: offset(Rectangle(2, 2)),
    "obj_ops_offset_hole_splits": lambda: offset(Rectangle(10, 2) - Circle(0.8), -0.3),
    "obj_ops_offset_hole_grows": lambda: offset(Rectangle(10, 4) - Circle(0.8), -0.3),
    "obj_ops_offset_hole_vanishes": lambda: offset(Rectangle(10, 4) - Circle(0.8), 1),
    "obj_ops_offset_nested_compound": lambda: offset(
        Compound([Compound([Edge.make_line((0, 0), (4, 0))]), Edge.make_line((4, 0), (4, 3))]), 0.5
    ),
    "obj_ops_offset_compound_wire_edge": lambda: offset(
        Compound([Wire.make_polygon([(0, 0), (5, 0), (5, 5)], close=False), Edge.make_line((5, 5), (0, 5))]), 0.5
    ),
    "obj_ops_scale_default": lambda: scale(Box(2, 2, 2)),
    "obj_ops_scale_list_vector": lambda: scale([Box(2, 2, 2), Pos(5, 0, 0) * Box(2, 2, 2)], (1, 2, 3)),
    "obj_ops_scale_list_vector_about": lambda: scale(
        [Box(2, 2, 2), Pos(5, 0, 0) * Sphere(1)], (1, 2, 3), about=(1, 1, 1)
    ),
    "obj_ops_split_list_face": lambda: split(
        [Box(2, 2, 2), Pos(5, 0, 0) * Box(2, 2, 2)], Face.make_rect(20, 20, Plane.XY.offset(0.5))
    ),
    "obj_ops_split_list_face_both": lambda: split(
        [Box(2, 2, 2), Pos(5, 0, 0) * Sphere(1)], Face.make_rect(20, 20, Plane.XY.offset(0.25)), Keep.BOTH
    ),
    "obj_ops_sweep_curve_children": lambda: sweep(
        Curve() + [Line((0, 0), (1, 0)), Line((0, 2), (1, 2))], Edge.make_line((0, 0, 0), (0, 0, 5))
    ),
    "obj_ops_sweep_curve_connected": lambda: sweep(
        Curve() + [Line((0, 0), (1, 0)), Line((1, 0), (1, 1))], Edge.make_line((0, 0, 0), (0, 0, 5))
    ),
    "obj_ops_sweep_path_edges": lambda: sweep(
        Circle(1), [Edge.make_line((0, 0, 0), (0, 0, 5)), Edge.make_line((0, 0, 5), (3, 0, 8))]
    ),
    "obj_ops_sweep_path_edges_list": lambda: sweep(
        [Circle(1), Pos(0, 0, 0) * Rectangle(0.5, 0.5)],
        [Edge.make_line((0, 0, 0), (0, 0, 5)), Edge.make_line((0, 0, 5), (3, 0, 8))],
    ),
    "obj_ops_sweep_path_edges_bad": lambda: sweep(
        Circle(1), [Edge.make_line((0, 0, 0), (0, 0, 5)), Edge.make_line((0, 1, 6), (3, 0, 8))]
    ),
    "obj_ops_sweep_path_curve_bad": lambda: sweep(
        Circle(1), Curve() + [Line((0, 0, 0), (0, 0, 5)), Line((1, 1, 6), (3, 0, 8))]
    ),
    "obj_ops_sweep_mixed": lambda: sweep(
        [Face.make_rect(1, 1), Edge.make_line((3, 0), (4, 0))], Edge.make_line((0, 0, 0), (0, 0, 4))
    ),
    "obj_ops_sweep_mixed_holed": lambda: sweep(
        [Rectangle(4, 4) - Circle(1), Edge.make_line((3, 0), (4, 0))], Edge.make_line((0, 0, 0), (0, 0, 4))
    ),
    "obj_ops_sweep_compound_faces": lambda: sweep(
        Compound([Face.make_rect(1, 1), Pos(3, 0, 0) * Face.make_rect(1, 1)]), Edge.make_line((0, 0, 0), (0, 0, 4))
    ),
    "obj_ops_sweep_binormal_edge": lambda: sweep(
        Pos(5, 0, 0) * Rectangle(1, 1), Edge.make_helix(5, 10, 5), binormal=Edge.make_line((0, 0, 0), (0, 0, 10))
    ),
    "obj_ops_sweep_empty_list": lambda: sweep([], Edge.make_line((0, 0, 0), (0, 0, 5))),
}
