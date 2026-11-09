"""Python twins of cases_objects.cpp (objects, operations, text and drafting)."""

from math import sin

from scipy.optimize import minimize

from build123d import *  # noqa: F403
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
}
