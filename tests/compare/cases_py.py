"""The models of cases.cpp built with Python build123d (algebra mode)."""

import glob
import importlib
import json
import os
import sys
from math import cos, pi, sin

from build123d import *  # noqa: F403
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from values import V


def metrics(name, shape):
    if isinstance(shape, (list, tuple)):
        shape = Compound(list(shape))
    solids, faces = len(shape.solids()), len(shape.faces())
    props = GProp_GProps()
    if solids:
        BRepGProp.VolumeProperties_s(shape.wrapped, props)
    elif faces:
        BRepGProp.SurfaceProperties_s(shape.wrapped, props)
    else:
        BRepGProp.LinearProperties_s(shape.wrapped, props)
    c = props.CentreOfMass()
    bb = shape.bounding_box()
    return {
        "name": name,
        "volume": shape.volume,
        "area": shape.area,
        "length": sum(e.length for e in shape.edges()),
        "solids": solids,
        "faces": faces,
        "edges": len(shape.edges()),
        "vertices": len(shape.vertices()),
        "bbox": [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z],
        "center": [c.X(), c.Y(), c.Z()],
    }


def plate_with_holes():
    return Box(30, 30, 5) - [
        Pos(x, y) * Cylinder(2, 5) for x, y in [(-10, -10), (10, -10), (0, 10)]
    ]


def bool_iadd():
    p = Part()
    for i in range(4):
        p += Pos(i * 5, 0, 0) * Box(4, 4, 4)
    return p


def extrude_until():
    base = Pos(0, 0, -1) * Box(20, 20, 2) + Pos(0, 0, 10) * Rot(20, 0, 0) * Box(30, 30, 2)
    return extrude(Circle(3), until=Until.NEXT, target=base)


def extrude_until_last():
    base = Pos(0, 0, 5) * Box(20, 20, 2) + Pos(0, 0, 12) * Box(20, 20, 2)
    return extrude(Circle(3), until=Until.LAST, target=base)


def sweep_case():
    path = JernArc((0, 0, 0), (0, 0, 1), 20, 90)
    return sweep(Plane(path @ 0, z_dir=path % 0) * Circle(2), path)


def sweep_helix():
    helix = Helix(5, 20, 10)
    profile = Plane(helix @ 0, z_dir=helix % 0) * Circle(1)
    return sweep(profile, helix, is_frenet=True)


def sweep_multisection():
    path = Line((0, 0, 0), (0, 0, 20))
    return sweep([Rectangle(6, 6), Pos(0, 0, 20) * Circle(2)], path, multisection=True)


def loft_sketch():
    art = Sketch()
    for i in range(11):
        art += Plane(origin=(0, 0, i * 3), z_dir=(0, 0, 1)) * Circle(10 * sin(i * pi / 10) + 5)
    return loft(art)


def find_intersection_points():
    p = Sphere(10) - Box(4, 4, 30)
    hits = p.find_intersection_points(Axis((-20, 1, 0.5), (1, 0.05, 0.02)))
    return Compound([Edge.make_line(pnt, pnt + n * (i + 1)) for i, (pnt, n) in enumerate(hits)])


def project_faces():
    sphere = Sphere(50)
    obj = sphere - Pos(-50, 0, -70) * (Plane.YZ * Cylinder(80, 100, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    arch_path = obj.edges().sort_by().first
    text = Text("abc", font_size=15, align=(Align.MIN, Align.CENTER))
    return Sketch(sphere.project_faces(text.faces(), path=arch_path))


def surface_from_points(smoothing=None):
    pts = [[(i * 2, j * 2, sin(i * 0.7) * cos(j * 0.5) * 3) for i in range(7)] for j in range(6)]
    return Face.make_surface_from_array_of_points(pts, smoothing=smoothing)


def fillet3d_top():
    p = Box(10, 10, 10) - Cylinder(3, 10)
    return fillet(p.edges().group_by(Axis.Z)[-1].filter_by(GeomType.LINE), 1)


def fillet3d_hole():
    p = Box(10, 10, 10) - Cylinder(3, 10)
    return fillet(p.edges().group_by(Axis.Z)[-1] | GeomType.CIRCLE, 1)


def chamfer3d_angle():
    p = Box(10, 10, 10)
    return chamfer(p.edges().group_by(Axis.Z)[-1], 2, angle=30)


def chamfer3d_asym():
    p = Box(10, 10, 10)
    return chamfer(p.edges().group_by(Axis.Z)[-1], 1, 2)


def fillet2d_some():
    s = Rectangle(10, 6) - Circle(1)
    return fillet(s.vertices().group_by(Axis.X)[-1], 2)


def offset3d_hollow():
    b = Box(20, 20, 10)
    return offset(b, -1, openings=b.faces().sort_by(Axis.Z)[-1])


def mirror_case():
    p = Box(10, 5, 5, align=Align.MIN)
    return p + mirror(p, Plane.YZ)


def sel_filter_position():
    p = Box(10, 10, 10, align=Align.MIN) + Pos(0, 0, 10) * Box(5, 5, 5, align=Align.MIN)
    return p.faces().filter_by_position(Axis.Z, 0, 10, inclusive=(True, False))


def sel_set_ops():
    p = Box(10, 10, 10)
    return (p.edges() - (p.edges() | Axis.Z)) + p.faces().sort_by(Axis.Z)[0].edges()


def location_at():
    path = Spline((0, 0, 0), (5, 5, 2), (10, 0, 4), (15, 5, 0))
    return (path ^ 0.3) * Box(1, 2, 3)


def location_at_wire():
    path = Polyline((0, 0, 0), (10, 0, 0), (10, 10, 5))
    return path.location_at(0.75) * Box(1, 2, 3)


def position_at():
    path = Spline((0, 0, 0), (5, 5, 2), (10, 0, 4))
    return Pos(path @ 0.6) * Sphere(1)


def tangent_at():
    arc = ThreePointArc((0, 0), (5, 5), (10, 0))
    return Plane(arc @ 0.25, z_dir=arc % 0.25) * Circle(1)


CASES = {
    # primitives
    "box": lambda: Box(10, 20, 30),
    "box_align_min": lambda: Box(10, 20, 30, align=Align.MIN),
    "box_align_mixed": lambda: Box(10, 20, 30, align=(Align.MIN, Align.CENTER, Align.MAX)),
    "box_rot": lambda: Box(10, 20, 30, rotation=(10, 20, 30)),
    "cylinder": lambda: Cylinder(5, 10),
    "cylinder_arc": lambda: Cylinder(5, 10, arc_size=270, align=Align.MIN),
    "cone": lambda: Cone(5, 2, 10),
    "sphere": lambda: Sphere(7),
    "sphere_partial": lambda: Sphere(7, -45, 60, 200),
    "torus": lambda: Torus(10, 2),
    "wedge": lambda: Wedge(10, 5, 8, 2, 2, 6, 6),
    "convex_polyhedron": lambda: ConvexPolyhedron(
        [(0, 0, 0), (10, 0, 0), (0, 10, 0), (0, 0, 10), (3, 3, 3), (10, 10, 10)]
    ),
    # booleans
    "bool_sub": lambda: Box(10, 10, 10) - Cylinder(3, 20),
    "bool_add": lambda: Box(10, 10, 10) + Pos(5, 5, 5) * Sphere(4),
    "bool_and": lambda: Box(10, 10, 10) & Sphere(7),
    "bool_list": plate_with_holes,
    "sketch_bool": lambda: Rectangle(20, 10) - Circle(3) + Pos(10, 0) * Circle(5),
    "sketch_and": lambda: Rectangle(20, 10) & Circle(7),
    "sketch_minus_part": lambda: Rectangle(20, 10) - Pos(5, 0) * Box(4, 4, 4),
    "bool_iadd": bool_iadd,
    # placement
    "plane_mul": lambda: Plane.XZ * Pos(5, 3) * Rot(0, 0, 30) * Box(4, 2, 1),
    "plane_offset": lambda: Plane.XY.offset(5) * Rectangle(3, 2),
    "plane_rotated": lambda: Plane.XY.rotated((30, 0, 45)) * Pos(2, 0) * Box(1, 2, 3),
    "location_inverse": lambda: (Pos(1, 2, 3) * Rot(10, 20, 30)).inverse() * Box(2, 3, 4),
    "grid_locs": lambda: Box(50, 50, 5) - GridLocations(10, 10, 4, 4) * Cylinder(2, 5),
    "grid_locs_min": lambda: Sketch() + GridLocations(5, 7, 3, 2, align=Align.MIN) * Rectangle(2, 2),
    "polar_locs": lambda: Cylinder(20, 5) - PolarLocations(15, 8) * Box(4, 2, 5),
    "polar_locs_norot": lambda: Sketch()
    + PolarLocations(10, 5, 15, 180, rotate=False, endpoint=True) * Rectangle(3, 1),
    "hex_locs": lambda: Rectangle(60, 60) - HexLocations(5, 4, 4) * RegularPolygon(4, 6),
    "locations_list": lambda: Sketch()
    + Locations(Pos(0, 0), Pos(10, 5), Pos(-10, 5) * Rot(0, 0, 45)) * Rectangle(3, 3),
    # sketch objects
    "circle": lambda: Circle(5),
    "circle_sector": lambda: Circle(5, arc_size=90),
    "ellipse": lambda: Ellipse(10, 5, rotation=30),
    "rect_rounded": lambda: RectangleRounded(20, 10, 2),
    "polygon": lambda: Polygon((0, 0), (10, 0), (12, 5), (3, 8), align=Align.CENTER),
    "polygon_cw": lambda: Polygon((0, 0), (3, 8), (12, 5), (10, 0)),
    "regular_polygon": lambda: RegularPolygon(10, 7, rotation=10),
    "regular_polygon_minor": lambda: RegularPolygon(10, 6, major_radius=False, align=(Align.MIN, Align.MIN)),
    "trapezoid": lambda: Trapezoid(20, 8, 60, 75),
    "triangle_sss": lambda: Triangle(a=10, b=8, c=7),
    "triangle_asa": lambda: Triangle(a=10, B=50, C=60),
    "slot_overall": lambda: SlotOverall(20, 6, rotation=15),
    "slot_c2c": lambda: SlotCenterToCenter(15, 4),
    "slot_cp": lambda: SlotCenterPoint((0, 0), (5, 3), 3),
    "slot_arc": lambda: SlotArc(CenterArc((0, 0), 10, 0, 90), 3),
    "superellipse": lambda: Superellipse(20, 10, 4),
    # part operations
    "extrude": lambda: extrude(Rectangle(10, 5), 3),
    "extrude_neg": lambda: extrude(Rectangle(10, 5), -3),
    "extrude_both": lambda: extrude(Circle(4) - Circle(2), 5, both=True),
    "extrude_taper": lambda: extrude(Rectangle(10, 10), 5, taper=10),
    "extrude_taper_hole": lambda: extrude(Rectangle(10, 10) - Circle(2), 5, taper=5),
    "extrude_dir": lambda: extrude(Rectangle(10, 5), 4, dir=(1, 0, 1)),
    "extrude_plane": lambda: extrude(Plane.XZ * Circle(3), 10),
    "extrude_until": extrude_until,
    "extrude_until_last": extrude_until_last,
    "revolve": lambda: revolve(Pos(10, 0) * Rectangle(4, 6), Axis.Y, 270),
    "revolve_full": lambda: revolve(Plane.XZ * Polygon((5, 0), (10, 0), (10, 3), (6, 8)), Axis.Z),
    "loft": lambda: loft([Rectangle(10, 10), Pos(0, 0, 10) * Circle(4), Pos(0, 0, 20) * Rectangle(4, 8)]),
    "loft_ruled": lambda: loft(
        [Rectangle(10, 10), Pos(0, 0, 10) * Circle(4), Pos(0, 0, 20) * Rectangle(4, 8)], ruled=True
    ),
    "loft_holes": lambda: loft([Rectangle(20, 20) - Circle(3), Pos(0, 0, 10) * (Rectangle(10, 10) - Circle(2))]),
    "loft_vertex": lambda: loft([Circle(5), Vertex(0, 0, 10)]),
    "loft_sketch": loft_sketch,
    "find_intersection_points": find_intersection_points,
    "project_to_shape": lambda: Compound(
        (Plane.ZX.offset(-80) * Rectangle(20, 20)).face().project_to_shape(Sphere(50), (0, 1, 0))
    ),
    "project_to_shape_edge": lambda: (Plane.YZ.offset(-50) * Rectangle(10, 10))
    .face()
    .project_to_shape(Rot(0, 0, 45) * Box(20, 20, 20), (1, 0, 0))[0],
    "project_faces": project_faces,
    "surface_from_points": surface_from_points,
    "surface_from_points_smooth": lambda: surface_from_points((1.0, 0.5, 0.1)),
    "sweep": sweep_case,
    "sweep_helix": sweep_helix,
    "sweep_multisection": sweep_multisection,
    "sweep_edges": lambda: sweep(Line((0, 0, 0), (5, 0, 0)), Line((0, 0, 0), (0, 0, 7))),
    "thicken": lambda: thicken(Rectangle(10, 10), 2),
    "thicken_both": lambda: thicken(Circle(5), 1, both=True),
    "section": lambda: section(Sphere(10) - Box(4, 4, 30), Plane.XZ),
    "section_height": lambda: section(Box(10, 10, 10) - Cylinder(3, 10), Plane.XY, 2),
    # fillet / chamfer
    "fillet3d": lambda: fillet((p := Box(10, 10, 10)).edges().filter_by(Axis.Z), 2),
    "fillet3d_top": fillet3d_top,
    "fillet3d_hole": fillet3d_hole,
    "chamfer3d": lambda: chamfer(Box(10, 10, 10).edges(), 1),
    "chamfer3d_angle": chamfer3d_angle,
    "chamfer3d_asym": chamfer3d_asym,
    "fillet2d": lambda: fillet(Rectangle(10, 6).vertices(), 1),
    "fillet2d_some": fillet2d_some,
    "chamfer2d": lambda: chamfer(Rectangle(10, 6).vertices(), 1),
    "fillet1d": lambda: fillet(Polyline((0, 0), (10, 0), (10, 10), (20, 10)).vertices(), 2),
    # generic operations
    "offset2d": lambda: offset(Rectangle(10, 6), 2),
    "offset2d_inter": lambda: offset(Rectangle(10, 6) - Circle(1), 1, kind=Kind.INTERSECTION),
    "offset3d_hollow": offset3d_hollow,
    "offset3d_grow": lambda: offset(Box(10, 10, 10), 1),
    "offset_curve": lambda: offset(Polyline((0, 0), (10, 0), (10, 10)), 1),
    "offset_curve_side": lambda: offset(Line((0, 0), (10, 0)), 1, side=Side.LEFT),
    "mirror": mirror_case,
    "mirror_sketch": lambda: mirror(Pos(5, 3) * Circle(2), Plane.XZ),
    "split": lambda: split(Sphere(10), Plane.XY),
    "split_both": lambda: split(Box(10, 10, 10), Plane.XZ.rotated((0, 0, 30)), keep=Keep.BOTH),
    "split_bottom": lambda: split(Box(10, 10, 10), Plane.YZ.offset(2), keep=Keep.BOTTOM),
    "split_face": lambda: split(Box(10, 10, 10), Pos(0, 0, 2) * Sphere(6).face(), keep=Keep.TOP),
    "scale": lambda: scale(Box(2, 2, 2), 3),
    "scale_nonuniform": lambda: scale(Sphere(2), (1, 2, 3)),
    "bounding_box_op": lambda: bounding_box(Pos(1, 2, 3) * Sphere(4)),
    "bounding_box_2d": lambda: bounding_box(Pos(1, 2) * Circle(4)),
    # sketch operations
    "make_face": lambda: make_face([Line((0, 0), (10, 0)), Line((10, 0), (5, 8)), Line((5, 8), (0, 0))]),
    "make_face_curve": lambda: make_face(Line((0, 0), (10, 0)) + ThreePointArc((10, 0), (5, 5), (0, 0))),
    "make_hull": lambda: make_hull((Circle(3) + Pos(10, 0) * Circle(2)).edges()),
    "trace": lambda: trace(Polyline((0, 0), (10, 0), (10, 10)), 1),
    # curves
    "line_add": lambda: Line((0, 0), (10, 0)) + Line((10, 0), (10, 10)),
    "polyline_closed": lambda: Polyline((0, 0), (10, 0), (5, 5), close=True),
    "spline": lambda: Spline((0, 0), (5, 5), (10, 0), (15, 5)),
    "spline_tangents": lambda: Spline((0, 0), (5, 5), (10, 0), tangents=((0, 1), (0, -1))),
    "spline_periodic": lambda: Spline((0, 0), (5, 5), (10, 0), (5, -5), periodic=True),
    "bezier": lambda: Bezier((0, 0), (5, 10), (10, -10), (15, 0)),
    "bezier_rational": lambda: Bezier((0, 0), (5, 10), (10, 0), weights=[1, 2, 1]),
    "bspline": lambda: BSpline(
        [(0, 0), (2, 4), (5, 5), (8, -2), (10, 0)], [0, 0, 0, 0, 0.5, 1, 1, 1, 1], 3
    ),
    "bspline_rational": lambda: BSpline([(0, 0), (5, 8), (10, 0)], [0, 0, 0, 1, 1, 1], 2, weights=[1, 3, 1]),
    "elliptical_start_arc": lambda: EllipticalStartArc((1, 2), (1, 1), 8, 4, 150, start_angle=30.0),
    "elliptical_start_arc_axis": lambda: EllipticalStartArc((1, 2), (0, 1), 8, 4, -120, major_axis_dir=(1, 1)),
    "parabolic_arc": lambda: ParabolicCenterArc((1, 2), 3, -40, arc_size=100, rotation=20),
    "hyperbolic_arc": lambda: HyperbolicCenterArc((1, 2), 4, 3, -30, arc_size=70, rotation=10),
    "hyperbolic_arc_tall": lambda: HyperbolicCenterArc((0, 0), 2, 5, 10, arc_size=-60),
    "three_point_arc": lambda: ThreePointArc((0, 0), (5, 5), (10, 0)),
    "center_arc": lambda: CenterArc((0, 0), 10, 30, -120),
    "center_arc_ccw": lambda: CenterArc((1, 2), 10, 30, 200),
    "radius_arc": lambda: RadiusArc((0, 0), (10, 0), 8),
    "radius_arc_neg": lambda: RadiusArc((0, 0), (10, 0), -8, short_sagitta=False),
    "sagitta_arc": lambda: SagittaArc((0, 0), (10, 0), 3),
    "tangent_arc": lambda: TangentArc((0, 0), (10, 0), tangent=(0, 1)),
    "jern_arc": lambda: JernArc((0, 0), (1, 1), 5, 120),
    "elliptical_arc": lambda: EllipticalCenterArc((0, 0), 10, 5, 0, arc_size=200, rotation=15),
    "helix": lambda: Helix(2, 10, 5),
    "helix_cone": lambda: Helix(2, 10, 5, cone_angle=10),
    "helix_left": lambda: Helix(3, 9, 4, (1, 2, 3), (0, 1, 1), lefthand=True),
    "polar_line": lambda: PolarLine((0, 0), 10, 30),
    "polar_line_dir": lambda: PolarLine((1, 1), 5, direction=(1, 2), length_mode=LengthMode.HORIZONTAL),
    "fillet_polyline": lambda: FilletPolyline((0, 0), (10, 0), (10, 10), (20, 10), radius=2),
    "fillet_polyline_closed": lambda: FilletPolyline((0, 0), (10, 0), (10, 10), (0, 10), radius=2, close=True),
    "intersecting_line": lambda: IntersectingLine((0, 0), (1, 1), Line((10, 0), (0, 10))),
    # selectors
    "sel_sort": lambda: (Box(10, 20, 30).faces() > Axis.Z)[-1],
    "sel_group_op": lambda: Box(10, 20, 30).edges() >> Axis.Z,
    "sel_group_min": lambda: (Box(10, 20, 30) - Cylinder(2, 30)).edges() << Axis.Z,
    "sel_filter_axis": lambda: Box(10, 20, 30).edges() | Axis.Y,
    "sel_filter_geomtype": lambda: (Box(10, 10, 10) - Cylinder(3, 10)).faces() | GeomType.CYLINDER,
    "sel_filter_position": sel_filter_position,
    "sel_sort_area": lambda: (Box(10, 10, 10) - Pos(5, 5) * Cylinder(3, 10)).faces().sort_by(SortBy.AREA)[-1],
    "sel_lambda": lambda: Box(4, 8, 12).edges().filter_by(lambda e: e.length > 5),
    "sel_plane": lambda: Box(4, 8, 12).faces().filter_by(Plane.XZ),
    "sel_group_area": lambda: Box(4, 8, 12).faces().group_by(SortBy.AREA)[-1],
    "sel_sort_reverse": lambda: (Box(10, 20, 30).faces() < Axis.X)[0],
    "sel_set_ops": sel_set_ops,
    # curve positions
    "location_at": location_at,
    "location_at_wire": location_at_wire,
    "position_at": position_at,
    "tangent_at": tangent_at,
    # text
    "text": lambda: Text("Hello", 10),
    "text_aligned": lambda: Text(
        "b123d", 5, font_style=FontStyle.BOLD, align=(Align.MIN, Align.MIN), rotation=30
    ),
}


def _load_case_modules():
    """Add the cases of every cases_<area>_py.py module next to this file."""
    here = os.path.dirname(os.path.abspath(__file__))
    for path in sorted(glob.glob(os.path.join(here, "cases_*_py.py"))):
        module = importlib.import_module(os.path.splitext(os.path.basename(path))[0])
        for name in module.CASES:
            if name in CASES:
                raise ValueError(f"duplicate case name {name!r} in {os.path.basename(path)}")
        CASES.update(module.CASES)


_load_case_modules()


def main():
    only = set(sys.argv[1:])
    for name, build in CASES.items():
        if only and name not in only:
            continue
        try:
            result = build()
            if isinstance(result, V):
                print(json.dumps({"name": name, "values": result.data}), flush=True)
            else:
                print(json.dumps(metrics(name, result)), flush=True)
        except Exception as exc:  # pylint: disable=broad-exception-caught
            print(json.dumps({"name": name, "error": f"{type(exc).__name__}: {exc}"}), flush=True)


if __name__ == "__main__":
    main()
