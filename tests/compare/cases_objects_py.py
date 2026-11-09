"""Python twins of cases_objects.cpp (objects, operations, text and drafting)."""

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
}
