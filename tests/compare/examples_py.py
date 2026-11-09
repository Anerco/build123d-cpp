"""The examples in examples/*.cpp, as written for Python build123d (algebra mode).

Each function is a transcription of the matching build123d example and returns the
objects the C++ program passes to show(), under the same names.
Usage: python examples_py.py [example ...]
"""

# pylint: disable=wildcard-import,unused-wildcard-import,too-many-locals,invalid-name
import json
import sys
from itertools import product
from math import cos, pi, sin

from build123d import *  # noqa: F403

from cases_py import metrics


def tea_cup():
    wall_thickness = 3 * MM
    fillet_radius = wall_thickness * 0.49
    s = Spline(
        (30 * MM, 10 * MM),
        (69 * MM, 105 * MM),
        tangents=((1, 0.5), (0.7, 1)),
        tangent_scalars=(1.75, 1),
    )
    s += Polyline(s @ 0, s @ 0 + (10 * MM, -10 * MM), (0, 0), (0, (s @ 1).Y), s @ 1)
    bowl_section = Plane.XZ * make_face(s)
    tea_cup = revolve(bowl_section, axis=Axis.Z)
    tea_cup = offset(tea_cup, -wall_thickness, openings=tea_cup.faces().filter_by(GeomType.PLANE))
    tea_cup += Pos(0, 0, (s @ 0).Y) * Cylinder(radius=(s @ 0).X, height=wall_thickness)
    tea_cup = fillet(tea_cup.edges(), radius=fillet_radius)
    handle_intersections = [
        tea_cup.find_intersection_points(Axis(origin=(0, 0, vertical_offset), direction=(1, 0, 0)))[-1][0]
        for vertical_offset in [35 * MM, 80 * MM]
    ]
    path_spline = Spline(
        handle_intersections[0] - (wall_thickness / 2, 0, 0),
        handle_intersections[0] + (35 * MM, 0, 30 * MM),
        handle_intersections[0] + (40 * MM, 0, 60 * MM),
        handle_intersections[1] - (wall_thickness / 2, 0, 0),
        tangents=((1, 0, 1.25), (-0.2, 0, -1)),
    )
    location = path_spline ^ 0
    handle_cross_section = location * RectangleRounded(wall_thickness, 8 * MM, fillet_radius)
    tea_cup += sweep(handle_cross_section, path=path_spline)
    return {"tea_cup": tea_cup}


def vase():
    l1 = Line((0, 0), (12, 0))
    l2 = RadiusArc(l1 @ 1, (15, 20), 50)
    l3 = Spline(l2 @ 1, (22, 40), (20, 50), tangents=(l2 % 1, (-0.75, 1)))
    l4 = RadiusArc(l3 @ 1, l3 @ 1 + Vector(0, 5), 5)
    l5 = Spline(l4 @ 1, l4 @ 1 + Vector(2.5, 2.5), l4 @ 1 + Vector(0, 5), tangents=(l4 % 1, (-1, 0)))
    outline = l1 + l2 + l3 + l4 + l5
    outline += Polyline(l5 @ 1, l5 @ 1 + Vector(0, 1), (0, (l5 @ 1).Y + 1), l1 @ 0)
    profile = make_face(outline.edges())
    vase = revolve(profile, Axis.Y)
    vase = offset(vase, openings=vase.faces().sort_by(Axis.Y).last, amount=-1)
    top_edges = vase.edges().filter_by(GeomType.CIRCLE).filter_by_position(Axis.Y, 60, 62)
    vase = fillet(top_edges, radius=0.25)
    vase = fillet(vase.edges().sort_by(Axis.Y).first, radius=0.5)
    return {"vase": vase}


def heat_exchanger():
    exchanger_diameter = 10 * CM
    exchanger_length = 30 * CM
    plate_thickness = 5 * MM
    tube_diameter = 5 * MM
    tube_spacing = 2 * MM
    tube_wall_thickness = 0.5 * MM
    tube_extension = 3 * MM
    bundle_diameter = exchanger_diameter - 2 * tube_diameter
    fillet_radius = tube_spacing / 3
    tube_locations = [
        l
        for l in HexLocations(
            radius=(tube_diameter + tube_spacing) / 2,
            x_count=exchanger_diameter // tube_diameter,
            y_count=exchanger_diameter // tube_diameter,
        )
        if l.position.length < bundle_diameter / 2
    ]
    ring = Circle(tube_diameter / 2) - Circle(tube_diameter / 2 - tube_wall_thickness)
    tube_plan = Sketch() + tube_locations * ring
    heat_exchanger = extrude(tube_plan, exchanger_length / 2)
    plate_plane = Plane(origin=(0, 0, exchanger_length / 2 - tube_extension - plate_thickness), z_dir=(0, 0, 1))
    plate = Circle(radius=exchanger_diameter / 2) - tube_locations * Circle(
        radius=tube_diameter / 2 - tube_wall_thickness
    )
    heat_exchanger += extrude(plate_plane * plate, plate_thickness)
    edges = heat_exchanger.edges().filter_by(GeomType.CIRCLE).group_by(Edge.radius)[1].group_by()[2]
    half_volume_before_fillet = heat_exchanger.volume
    heat_exchanger = fillet(edges, radius=fillet_radius)
    half_volume_after_fillet = heat_exchanger.volume
    heat_exchanger += mirror(heat_exchanger, Plane.XY)
    fillet_volume = 2 * (half_volume_after_fillet - half_volume_before_fillet)
    assert abs(fillet_volume - 469.88331045553787) < 1e-3
    return {"heat_exchanger": heat_exchanger}


def handle():
    segment_count = 6
    handle_center_line = Spline(
        (-10, 0, 0),
        (0, 0, 5),
        (10, 0, 0),
        tangents=((0, 0, 1), (0, 0, -1)),
        tangent_scalars=(1.5, 1.5),
    )
    sections = Sketch()
    for i in range(segment_count + 1):
        location = handle_center_line ^ (i / segment_count)
        if i % segment_count == 0:
            circle = location * Circle(1)
        else:
            circle = location * Rectangle(1.25, 3)
            circle = fillet(circle.vertices(), radius=0.2)
        sections += circle
    handle = sweep(sections, path=handle_center_line, multisection=True)
    return {"handle_path": handle_center_line, "handle": handle}


def lego():
    pip_count = 6
    lego_unit_size = 8
    pip_height = 1.8
    pip_diameter = 4.8
    block_length = lego_unit_size * pip_count
    block_width = 16
    base_height = 9.6
    support_outer_diameter = 6.5
    support_inner_diameter = 4.8
    ridge_width = 0.6
    ridge_depth = 0.3
    wall_thickness = 1.2
    plan = Rectangle(width=block_length, height=block_width)
    plan -= offset(plan, -wall_thickness, kind=Kind.INTERSECTION)
    locs = GridLocations(x_spacing=0, y_spacing=lego_unit_size, x_count=1, y_count=2)
    plan += locs * Rectangle(width=block_length, height=ridge_width)
    locs = GridLocations(lego_unit_size, 0, pip_count, 1)
    plan += locs * Rectangle(width=ridge_width, height=block_width)
    plan -= Rectangle(
        block_length - 2 * (wall_thickness + ridge_depth),
        block_width - 2 * (wall_thickness + ridge_depth),
    )
    locs = GridLocations(x_spacing=lego_unit_size, y_spacing=0, x_count=pip_count - 1, y_count=1)
    ring = Circle(support_outer_diameter / 2) - Circle(support_inner_diameter / 2)
    plan += locs * ring
    lego = extrude(plan, amount=base_height - wall_thickness)
    lego += Pos(0, 0, lego.vertices().sort_by().last.Z) * Box(
        length=block_length,
        width=block_width,
        height=wall_thickness,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    plane = Plane(lego.faces().sort_by().last)
    locs = GridLocations(lego_unit_size, lego_unit_size, pip_count, 2)
    lego += (
        plane
        * locs
        * Cylinder(radius=pip_diameter / 2, height=pip_height, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )
    return {"lego": lego}


def key_cap():
    plan = Rectangle(18 * MM, 18 * MM)
    key_cap = extrude(plan, amount=10 * MM, taper=15)
    key_cap -= Location((0, -3 * MM, 47 * MM), (90, 0, 0)) * Sphere(40 * MM)
    key_cap = fillet(
        key_cap.edges().filter_by_position(Axis.Z, 0, 30 * MM, inclusive=(False, True)),
        radius=1 * MM,
    )
    key_cap -= scale(key_cap, (0.925, 0.925, 0.85))
    key_cap_section = section(key_cap, Plane.XY.offset(4 * MM)).face()
    key_cap_internal_size = key_cap_section.inner_wires()[0].bounding_box().size
    ribs = Rectangle(key_cap_internal_size.X, 0.5 * MM)
    ribs += Rectangle(0.5 * MM, key_cap_internal_size.Y)
    ribs += Circle(radius=5.51 * MM / 2)
    key_cap += extrude(Pos(0, 0, 4 * MM) * ribs, until=Until.NEXT, target=key_cap)
    socket = Circle(radius=5.5 * MM / 2)
    socket -= Rectangle(4.1 * MM, 1.17 * MM)
    socket -= Rectangle(1.17 * MM, 4.1 * MM)
    key_cap += extrude(Plane.XY.offset(4 * MM) * socket, amount=-3.5 * MM)
    return {"key_cap": key_cap}


def pillow_block():
    height, width, thickness, padding = 60, 80, 10, 12
    screw_shaft_radius, screw_head_radius, screw_head_height = 1.5, 3, 3
    bearing_axle_radius, bearing_radius, bearing_thickness = 4, 11, 7
    plan = Rectangle(width, height)
    plan = fillet(plan.vertices(), radius=5)
    pillow_block = extrude(plan, thickness)
    plane = Plane(pillow_block.faces().sort_by().last)
    pillow_block -= plane * CounterBoreHole(bearing_axle_radius, bearing_radius, bearing_thickness, height)
    locs = GridLocations(width - 2 * padding, height - 2 * padding, 2, 2)
    pillow_block -= plane * locs * CounterBoreHole(screw_shaft_radius, screw_head_radius, screw_head_height, height)
    return {"pillow_block": pillow_block}


def din_rail():
    overall_width, top_width, height, thickness, fillet_radius = 35, 27, 7.5, 1, 0.8
    rail_length = 1000
    slot_width, slot_length, slot_pitch = 6.2, 15, 25
    din = Rectangle(overall_width, thickness, align=(Align.CENTER, Align.MIN))
    din += Rectangle(top_width, height, align=(Align.CENTER, Align.MIN))
    din -= Rectangle(top_width - 2 * thickness, height - thickness, align=(Align.CENTER, Align.MIN))
    inside_vertices = (
        din.vertices()
        .filter_by_position(Axis.Y, 0.0, height, inclusive=(False, False))
        .filter_by_position(Axis.X, -overall_width / 2, overall_width / 2, inclusive=(False, False))
    )
    din = fillet(inside_vertices, radius=fillet_radius)
    outside_vertices = filter(
        lambda v: (v.Y == 0.0 or v.Y == height) and -overall_width / 2 < v.X < overall_width / 2,
        din.vertices(),
    )
    din = fillet(outside_vertices, radius=fillet_radius + thickness)
    rail = extrude(din, rail_length)
    plane = Plane(rail.faces().sort_by(Axis.Y).last)
    slot_faces = [
        (plane * loc * Rot(0, 0, 90) * SlotOverall(slot_length, slot_width)).faces()[0]
        for loc in GridLocations(0, slot_pitch, 1, rail_length // slot_pitch - 1)
    ]
    rail -= extrude(slot_faces, -height)
    rail = Plane.XZ * rail
    return {"din_rail": rail}


def pegboard_j_hook():
    pegd = 6.35 + 0.1
    c2c = 25.4
    arcd = 7.2
    both = 10
    topx = 6
    midx = 8
    maind = 0.82 * pegd
    midd = 1.0 * pegd
    hookd = 23
    hookx = 10
    splitz = maind / 2 - 0.1
    topangs = 70
    l1 = Line((-both, 0), (c2c - arcd / 2 - 0.5, 0))
    l2 = JernArc(start=l1 @ 1, tangent=l1 % 1, radius=arcd / 2, arc_size=topangs)
    l3 = PolarLine(start=l2 @ 1, length=topx, direction=l2 % 1)
    l4 = JernArc(start=l3 @ 1, tangent=l3 % 1, radius=arcd / 2, arc_size=-topangs)
    l5 = PolarLine(start=l4 @ 1, length=topx, direction=l4 % 1)
    l6 = JernArc(start=l1 @ 0, tangent=(l1 % 0).reverse(), radius=hookd / 2, arc_size=170)
    l7 = PolarLine(start=l6 @ 1, length=hookx, direction=l6 % 1)
    sprof = Curve() + (l1, l2, l3, l4, l5, l6, l7)
    wire = Wire(sprof.edges())
    mainp = sweep(Plane.YZ * Circle(radius=maind / 2), path=wire)
    stub = Line((0, 0), (0, midx + maind / 2))
    mainp += sweep(Plane.XZ * Circle(radius=midd / 2), path=stub)
    mainp = split(mainp, Plane(origin=(0, 0, -splitz)))
    mainp = split(mainp, Plane(origin=(0, 0, splitz)), keep=Keep.BOTTOM)
    return {"pegboard_j_hook": mainp}


def circuit_board():
    pcb_length = 70 * MM
    pcb_width = 30 * MM
    pcb_height = 3 * MM
    x_coords = product(range(65 // 5), (-15, -10, 10, 15))
    y_coords = product((30, 35), range(30 // 5 - 1))
    pcb = Rectangle(pcb_length, pcb_width)
    pcb -= [Pos(i * 5 - 30, y) * Circle(1) for i, y in x_coords]
    pcb -= [Pos(x, i * 5 - 10) * Circle(1) for x, i in y_coords]
    pcb -= [loc * Circle(2) for loc in GridLocations(60, 20, 2, 2)]
    pcb = extrude(pcb, pcb_height)
    return {"circuit_board": pcb}


def intersecting_chamfers():
    blocks = Pos(-1, -1, 0) * Box(1, 2, 1, align=(Align.CENTER, Align.MIN, Align.MIN))
    blocks += Box(1, 1, 2, align=(Align.CENTER, Align.MIN, Align.MIN))
    blocks += Pos(1, -1, 0) * Box(1, 2, 1, align=(Align.CENTER, Align.MIN, Align.MIN))
    bottom_edges = blocks.edges().filter_by_position(Axis.Z, 0, 1, inclusive=(True, False))
    blocks2 = chamfer(bottom_edges, length=0.1)
    top_edges = blocks2.edges().filter_by_position(Axis.Z, 1, 2, inclusive=(False, True))
    blocks2 = chamfer(top_edges, length=0.1)
    return {"intersecting_chamfers": blocks2}


def holes():
    thru_hole = Cylinder(radius=3, height=2)
    thru_hole -= Hole(radius=1, depth=2)
    recessed_counter_bore = Cylinder(radius=3, height=2)
    recessed_counter_bore -= CounterBoreHole(radius=1, depth=2, counter_bore_radius=1.5, counter_bore_depth=0.5)
    recessed_counter_sink = Cylinder(radius=3, height=2)
    recessed_counter_sink -= CounterSinkHole(radius=1, depth=2, counter_sink_radius=1.5)
    flush_counter_sink = Cylinder(radius=3, height=2)
    plane = Plane(flush_counter_sink.faces().sort_by().last)
    flush_counter_sink -= plane * CounterSinkHole(radius=1, depth=2, counter_sink_radius=1.5)
    return {
        "thru_hole": thru_hole,
        "recessed_counter_bore": recessed_counter_bore,
        "recessed_counter_sink": recessed_counter_sink,
        "flush_counter_sink": flush_counter_sink,
    }


def boxes_on_faces():
    b = Box(3, 3, 3)
    b2 = Rot(0, 0, 45) * extrude(Rectangle(1, 2), 0.1)
    for plane in [Plane(f) for f in b.faces()]:
        b += plane * b2
    return {"boxes_on_faces": b}


def multiple_workplanes():
    obj = Box(5, 5, 1)
    planes = [Plane(f) for f in obj.faces().filter_by(Axis.Z)]
    obj -= planes * Sphere(1.8)
    return {"multiple_workplanes": obj}


def loft_example():
    slice_count = 10
    art = Sketch()
    for i in range(slice_count + 1):
        plane = Plane(origin=(0, 0, i * 3), z_dir=(0, 0, 1))
        art += plane * Circle(10 * sin(i * pi / slice_count) + 5)
    art = loft(art)
    top_bottom = art.faces().filter_by(GeomType.PLANE)
    art = offset(art, openings=top_bottom, amount=0.5)
    return {"loft": art}


def extrude_example():
    simple = extrude(Text("O", font_size=10), amount=5)
    both = extrude(Text("O", font_size=10), amount=5, both=True)
    multiple = Box(10, 10, 10)
    faces = [
        Plane(face) * loc * Text("Ω", font_size=3)
        for face in multiple.faces()
        for loc in GridLocations(5, 5, 2, 2)
    ]
    multiple += [extrude(face, amount=1) for face in faces]
    non_planar = Rot(90, 0, 0) * Cylinder(10, 20, align=(Align.CENTER, Align.MIN, Align.CENTER))
    non_planar &= Box(10, 10, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
    non_planar = extrude(non_planar.faces().sort_by(Axis.Z).first, amount=2, dir=(0, 0, 1))
    rad, rev = 3, 25
    circle = Pos(0, rev) * Circle(rad)
    ex26_target = revolve(circle, Axis.X, revolution_arc=180)
    rect = Rectangle(rad, rev)
    ex26 = extrude(rect, until=Until.LAST, target=ex26_target, clean=False)
    circle = Pos(0, rev) * Circle(rad)
    ex27 = revolve(circle, Axis.X, revolution_arc=90)
    circle2 = Plane.XZ * Pos(0, rev) * Circle(rad)
    ex27 += revolve(circle2, Axis.X, revolution_arc=150)
    rect = Plane.XY.offset(-60) * Rectangle(rad, rev + 25)
    extrusion27 = extrude(rect, until=Until.NEXT, target=ex27)
    return {
        "extrude_simple": simple,
        "extrude_both": both,
        "extrude_multiple": multiple,
        "extrude_non_planar": non_planar,
        "extrude_until_last_target": ex26_target,
        "extrude_until_last": ex26,
        "extrude_until_next_target": ex27,
        "extrude_until_next": extrusion27,
    }


def clock():
    clock_radius = 10
    l1 = CenterArc((0, 0), clock_radius * 0.975, 0.75, 4.5)
    l2 = CenterArc((0, 0), clock_radius * 0.925, 0.75, 4.5)
    l3 = Line(l1 @ 0, l2 @ 0)
    l4 = Line(l1 @ 1, l2 @ 1)
    minute_indicator = make_face([l1, l3, l2, l4])
    minute_indicator = fillet(minute_indicator.vertices(), radius=clock_radius * 0.01)
    clock_face = Circle(clock_radius)
    clock_face -= PolarLocations(0, 60) * minute_indicator
    clock_face -= PolarLocations(clock_radius * 0.875, 12) * SlotOverall(clock_radius * 0.05, clock_radius * 0.025)
    clock_face -= [
        loc
        * Text(
            str(hour + 1),
            font_size=clock_radius * 0.175,
            font_style=FontStyle.BOLD,
            align=Align.CENTER,
        )
        for hour, loc in enumerate(PolarLocations(clock_radius * 0.75, 12, 60, -360, rotate=False))
    ]
    return {"clock_face": clock_face}


def roller_coaster():
    powerup = Spline(
        (0, 0, 0),
        (50, 0, 50),
        (100, 0, 0),
        tangents=((1, 0, 0), (1, 0, 0)),
        tangent_scalars=(0.5, 2),
    )
    corner = RadiusArc(powerup @ 1, (100, 60, 0), -30)
    screw = Helix(75, 150, 15, center=(75, 40, 15), direction=(-1, 0, 0))
    roller_coaster = Curve() + (powerup + corner + screw)
    roller_coaster += Spline(corner @ 1, screw @ 0, tangents=(corner % 1, screw % 0))
    roller_coaster += Spline(screw @ 1, (-100, 30, 10), powerup @ 0, tangents=(screw % 1, powerup % 0))
    return {"roller_coaster": roller_coaster}


def canadian_flag():
    height = 50
    width = 2 * height
    wave_amplitude = 3

    def surface(amplitude, u, v):
        return v * amplitude / 20 * cos(3.5 * pi * u) + amplitude / 10 * v * sin(1.1 * pi * v)

    the_wind = Face.make_surface_from_array_of_points(
        [
            [
                Vector(
                    width * (v * 1.1 / 40 - 0.05),
                    height * (u * 1.2 / 40 - 0.1),
                    height * surface(wave_amplitude, u / 40, v / 40) / 2,
                )
                for u in range(41)
            ]
            for v in range(41)
        ]
    )
    field_planar = Plane.XY.offset(10) * Rectangle(width / 4, height, align=Align.MIN)
    west_field_planar = field_planar.faces()[0]
    east_field_planar = mirror(west_field_planar, Plane.YZ.offset(width / 2))
    l1 = Polyline((0.0000, 0.0771), (0.0187, 0.0771), (0.0094, 0.2569))
    l2 = Polyline((0.0325, 0.2773), (0.2115, 0.2458), (0.1873, 0.3125))
    r1 = RadiusArc(l1 @ 1, l2 @ 0, 0.0271)
    l3 = Polyline((0.1915, 0.3277), (0.3875, 0.4865), (0.3433, 0.5071))
    r2 = TangentArc(l2 @ 1, l3 @ 0, tangent=l2 % 1)
    l4 = Polyline((0.3362, 0.5235), (0.375, 0.6427), (0.2621, 0.6188))
    r3 = SagittaArc(l3 @ 1, l4 @ 0, 0.003)
    l5 = Polyline((0.2469, 0.6267), (0.225, 0.6781), (0.1369, 0.5835))
    r4 = ThreePointArc(l4 @ 1, (l4 @ 1 + l5 @ 0) * 0.5 + Vector(-0.002, -0.002), l5 @ 0)
    l6 = Polyline((0.1138, 0.5954), (0.1562, 0.8146), (0.0881, 0.7752))
    s = Spline(l5 @ 1, l6 @ 0, tangents=(l5 % 1, l6 % 0), tangent_scalars=(2, 2))
    l7 = Line((0.0692, 0.7808), (0.0000, 0.9167))
    r5 = TangentArc(l6 @ 1, l7 @ 0, tangent=l6 % 1)
    outline = l1 + [l2, r1, l3, r2, l4, r3, l5, r4, l6, s, l7, r5]
    outline += mirror(outline, Plane.YZ)
    maple_leaf_planar = make_face(outline)
    center_field_planar = Rectangle(1, 1, align=(Align.CENTER, Align.MIN)) - maple_leaf_planar

    def scale_move(obj):
        return Plane((width / 2, 0, 10)) * scale(obj, height)

    def project(obj):
        return obj.faces()[0].project_to_shape(the_wind, (0, 0, -1))[0]

    maple_leaf_planar = scale_move(maple_leaf_planar)
    center_field_planar = scale_move(center_field_planar)
    west_field = project(west_field_planar)
    west_field.color = Color("red")
    east_field = project(east_field_planar)
    east_field.color = Color("red")
    center_field = project(center_field_planar)
    center_field.color = Color("white")
    maple_leaf = project(maple_leaf_planar)
    maple_leaf.color = Color("red")
    canadian_flag = Compound(children=[west_field, east_field, center_field, maple_leaf])
    return {"the_wind": the_wind, "canadian_flag": canadian_flag}


class Club(Sketch):
    def __init__(self, height):
        l0 = Line((0, -188), (76, -188))
        b0 = Bezier(l0 @ 1, (61, -185), (33, -173), (17, -81))
        b1 = Bezier(b0 @ 1, (49, -128), (146, -145), (167, -67))
        b2 = Bezier(b1 @ 1, (187, 9), (94, 52), (32, 18))
        b3 = Bezier(b2 @ 1, (92, 57), (113, 188), (0, 188))
        club = l0 + b0 + b1 + b2 + b3
        club += mirror(club, Plane.YZ)
        club = make_face(club)
        club = scale(club, height / club.bounding_box().size.Y)
        super().__init__(club.wrapped)


class Spade(Sketch):
    def __init__(self, height):
        b0 = Bezier((0, 198), (6, 190), (41, 127), (112, 61))
        b1 = Bezier(b0 @ 1, (242, -72), (114, -168), (11, -105))
        b2 = Bezier(b1 @ 1, (31, -174), (42, -179), (53, -198))
        l0 = Line(b2 @ 1, (0, -198))
        spade = b0 + b1 + b2 + l0
        spade += mirror(spade, Plane.YZ)
        spade = make_face(spade)
        spade = scale(spade, height / spade.bounding_box().size.Y)
        super().__init__(spade.wrapped)


class Heart(Sketch):
    def __init__(self, height):
        b1 = Bezier((0, 146), (20, 169), (67, 198), (97, 198))
        b2 = Bezier(b1 @ 1, (125, 198), (151, 186), (168, 167))
        b3 = Bezier(b2 @ 1, (197, 133), (194, 88), (158, 31))
        b4 = Bezier(b3 @ 1, (126, -13), (94, -48), (62, -95))
        b5 = Bezier(b4 @ 1, (40, -128), (0, -198))
        heart = b1 + b2 + b3 + b4 + b5
        heart += mirror(heart, Plane.YZ)
        heart = make_face(heart)
        heart = scale(heart, height / heart.bounding_box().size.Y)
        super().__init__(heart.wrapped)


class Diamond(Sketch):
    def __init__(self, height):
        diamond = Bezier((135, 0), (94, 69), (47, 134), (0, 198))
        diamond += mirror(diamond, Plane.XZ)
        diamond += mirror(diamond, Plane.YZ)
        diamond = make_face(diamond)
        diamond = scale(diamond, height / diamond.bounding_box().size.Y)
        super().__init__(diamond.wrapped)


def custom_sketch_objects():
    pocket_w = 2.5 * IN + 2 * MM
    pocket_l = 3.5 * IN + 2 * MM
    pocket_t = 0.5 * IN + 2 * MM
    wall_t = 3 * MM
    bottom_t = wall_t / 2
    lid_gap = 0.5 * MM
    lip_t = wall_t / 2 - lid_gap / 2
    box_plan = RectangleRounded(pocket_w + 2 * wall_t, pocket_l + 2 * wall_t, pocket_w / 15)
    box = extrude(box_plan, amount=bottom_t + pocket_t / 2)
    base_top = box.faces().sort_by(Axis.Z).last
    walls = Plane(base_top) * offset(box_plan, -lip_t)
    box += extrude(walls, amount=pocket_t / 2)
    top = Plane.XY.offset(wall_t / 2) * offset(box_plan, -wall_t)
    box -= extrude(top, amount=pocket_t)
    pocket = extrude(box_plan, amount=pocket_t / 2 + bottom_t)
    lid_bottom = offset(box_plan, -(wall_t - lip_t))
    pocket -= extrude(lid_bottom, amount=pocket_t / 2)
    pocket = Pos(0, 0, (wall_t + pocket_t) / 2) * pocket
    plane = Plane(pocket.faces().sort_by().last)
    suites = Pos(-0.3 * pocket_w, 0.3 * pocket_l) * Heart(pocket_l / 5)
    suites += Pos(-0.3 * pocket_w, -0.3 * pocket_l) * Diamond(pocket_l / 5)
    suites += Pos(0.3 * pocket_w, 0.3 * pocket_l) * Spade(pocket_l / 5)
    suites += Pos(0.3 * pocket_w, -0.3 * pocket_l) * Club(pocket_l / 5)
    suites = plane * suites
    lid = pocket - extrude(suites, dir=(0, 0, 1), amount=-wall_t)
    return {"card_box": box, "card_box_lid": lid}


def projection():
    sphere = Sphere(50)
    projection_direction = Vector(0, 1, 0)
    square = Plane.ZX.offset(-80) * Rectangle(20, 20)
    square_projected = square.faces()[0].project_to_shape(sphere, projection_direction)
    square_solids = Part() + [Solid.thicken(f, 2) for f in square_projected]
    face = square.faces()[0]
    projection_beams = loft([face, Pos(0, 160, 0) * face])
    projection_direction = Vector(0, -1, 0)
    flat_planar_text = Rot(90, 0, 0) * Text("Flat", font_size=30)
    flat_projected_text_faces = Sketch() + [
        f.project_to_shape(sphere, projection_direction)[0] for f in flat_planar_text.faces()
    ]
    flat_projection_beams = Part() + [
        extrude(f, dir=projection_direction, amount=80) for f in flat_planar_text.faces()
    ]
    cyl = Plane.YZ * Cylinder(80, 100, align=(Align.CENTER, Align.CENTER, Align.MIN))
    obj = sphere - Pos(-50, 0, -70) * cyl
    arch_path = obj.edges().sort_by().first
    text = Text("'the quick brown fox jumped over the lazy dog'", font_size=15, align=(Align.MIN, Align.CENTER))
    projected_text = Sketch(sphere.project_faces(text.faces(), path=arch_path))
    return {
        "projection_square_solids": square_solids,
        "projection_beams": projection_beams,
        "projection_flat_text": flat_projected_text_faces,
        "projection_flat_beams": flat_projection_beams,
        "projection_text_on_path": projected_text,
    }


def build123d_logo():
    logo_text = Text("123d", font_size=10, align=Align.MIN)
    font_height = logo_text.vertices().sort_by(Axis.Y).last.Y
    build_text = Text("build", font_size=5, align=Align.CENTER)
    build_bb = build_text.bounding_box()
    build_width = build_bb.max.X - build_bb.min.X
    l1 = Line((font_height * 0.3, 0), (font_height * 0.3, font_height))
    one = l1 + TangentArc(l1 @ 1, (0, font_height * 0.7), tangent=(l1 % 1) * -1)
    two = Pos(font_height * 0.35, 0) * Text("2", font_size=10, align=Align.MIN)
    three_d = Text("3d", font_size=10, align=Align.MIN)
    three_d = Pos(font_height * 1.1, 0) * extrude(three_d, amount=font_height * 0.3)
    logo_width = three_d.vertices().sort_by(Axis.X).last.X
    t1 = TangentArc((0, 0), (1, 0.75), tangent=(1, 0))
    arrow_left = t1 + mirror(t1, Plane.XZ)
    ext_line_length = font_height * 0.5
    dim_line_length = (logo_width - build_width - 2 * font_height * 0.05) / 2
    l1 = Line((0, -font_height * 0.1), (0, -ext_line_length - font_height * 0.1))
    l2 = Line(
        (logo_width, -font_height * 0.1),
        (logo_width, -ext_line_length - font_height * 0.1),
    )
    extension_lines = Curve() + (l1 + l2)
    extension_lines += Pos(*(l1 @ 0.5)) * arrow_left
    extension_lines += (Pos(*(l2 @ 0.5)) * Rot(Z=180)) * arrow_left
    extension_lines += Line(l1 @ 0.5, l1 @ 0.5 + Vector(dim_line_length, 0))
    extension_lines += Line(l2 @ 0.5, l2 @ 0.5 - Vector(dim_line_length, 0))
    p1 = Pos((l1 @ 0.5 + l2 @ 0.5) / 2 - Vector((build_bb.max.X + build_bb.min.X) / 2, 0))
    build = p1 * build_text
    cmpd = Compound([three_d, two, one, build, extension_lines])
    return {"build123d_logo": cmpd}


def build123d_customizable_logo():
    logo_text = Text("123d", font_size=10, align=Align.MIN)
    font_height = logo_text.vertices().sort_by(Axis.Y)[-1].Y
    build_text = Text("build", font_size=5, align=Align.CENTER)
    build_bb = build_text.bounding_box()
    build_width = build_bb.max.X - build_bb.min.X
    cust_text = Text("customizable", font_size=2.9, align=Align.CENTER, font_style=FontStyle.BOLD)
    cust_bb = cust_text.bounding_box()
    l1 = Line((font_height * 0.3, 0), (font_height * 0.3, font_height))
    one = l1 + TangentArc(l1 @ 1, (0, font_height * 0.7), tangent=(l1 % 1) * -1)
    two = Pos(font_height * 0.35, 0) * Text("2", font_size=10, align=Align.MIN)
    three_d = Text("3d", font_size=10, align=Align.MIN)
    three_d = Pos(font_height * 1.1, 0) * extrude(three_d, amount=font_height * 0.3)
    logo_width = three_d.vertices().sort_by(Axis.X)[-1].X
    t1 = TangentArc((0, 0), (1, 0.75), tangent=(1, 0))
    arrow_left = t1 + mirror(t1, Plane.XZ)
    ext_line_length = font_height * 0.5
    dim_line_length = (logo_width - build_width - 2 * font_height * 0.05) / 2
    l1 = Line((0, -font_height * 0.1), (0, -ext_line_length - font_height * 0.1))
    l2 = Line(
        (logo_width, -font_height * 0.1),
        (logo_width, -ext_line_length - font_height * 0.1),
    )
    extension_lines = Curve() + (l1 + l2)
    extension_lines += Pos(*(l1 @ 0.5)) * arrow_left
    extension_lines += (Pos(*(l2 @ 0.5)) * Rot(Z=180)) * arrow_left
    extension_lines += Line(l1 @ 0.5, l1 @ 0.5 + Vector(dim_line_length, 0))
    extension_lines += Line(l2 @ 0.5, l2 @ 0.5 - Vector(dim_line_length, 0))
    p1 = Pos((l1 @ 0.5 + l2 @ 0.5) / 2 - Vector((build_bb.max.X + build_bb.min.X) / 2, 0))
    build = p1 * build_text
    p2 = Pos((l1 @ 1 + l2 @ 1) / 2 - Vector(cust_bb.max.X + cust_bb.min.X, 1.4))
    build += p2 * cust_text
    cmpd = Compound([three_d, two, one, build, extension_lines])
    return {"build123d_customizable_logo": cmpd}


def joints():
    class JointBox(Part):
        def __init__(self, length, width, height, radius=0.0, taper=0.0):
            obj = extrude(Rectangle(length, width), amount=height, taper=taper)
            if radius != 0.0:
                obj = fillet(obj.edges(), radius=radius)
            obj -= Rot(0, 90, 0) * Cylinder(width / 4, length)
            super().__init__(obj.wrapped)

    loc = Location(Vector(1, 1, 1), (1, 1, 1), 30)
    base = loc * JointBox(10, 10, 10, taper=3)
    base_top_edges = base.edges().filter_by(loc.x_axis).group_by(loc.z_axis)[-1]
    fixed_arm = JointBox(1, 1, 5, 0.2)
    RigidJoint("side", base, Plane(base.faces().sort_by(loc.x_axis).last).location)
    RigidJoint("top", fixed_arm, (-Plane(fixed_arm.faces().sort_by().last)).location)
    base.joints["side"].connect_to(fixed_arm.joints["top"])
    hinge_arm = JointBox(2, 1, 10, taper=1)
    swing_arm_hinge_edge = hinge_arm.edges().group_by(SortBy.LENGTH)[-1].sort_by(Axis.X)[-2:].sort_by(Axis.Y)[0]
    swing_arm_hinge_axis = Axis(swing_arm_hinge_edge)
    base_corner_edge = base.edges().sort_by(Axis((0, 0, 0), (1, 1, 0)))[-1]
    base_hinge_axis = Axis(base_corner_edge)
    RevoluteJoint("hinge", base, axis=base_hinge_axis, angular_range=(0, 180))
    RigidJoint("corner", hinge_arm, swing_arm_hinge_axis.location)
    base.joints["hinge"].connect_to(hinge_arm.joints["corner"], angle=90)
    slider_arm = JointBox(4, 1, 2, 0.2)
    LinearJoint(
        "slide",
        base,
        axis=Axis(Edge.make_mid_way(*base_top_edges, 0.67)),
        linear_range=(0, base_top_edges[0].length),
    )
    RigidJoint("slide", slider_arm, Location(Vector(0, 0, 0)))
    base.joints["slide"].connect_to(slider_arm.joints["slide"], position=8)
    hole_axis = Axis(
        base.faces().sort_by(Axis.Y)[0].center(),
        -base.faces().sort_by(Axis.Y)[0].normal_at(),
    )
    screw_arm = JointBox(1, 1, 10, 0.49)
    j5 = CylindricalJoint("hole", base, hole_axis, linear_range=(-10, 10))
    j6 = RigidJoint("screw", screw_arm, screw_arm.faces().sort_by(Axis.Z)[-1].location)
    j5.connect_to(j6, position=-1, angle=90)
    j7 = LinearJoint(
        "slot",
        base,
        axis=Axis(Edge.make_mid_way(*base_top_edges, 0.33)),
        linear_range=(0, base_top_edges[0].length),
    )
    pin_arm = JointBox(2, 1, 2)
    j8 = RevoluteJoint("pin", pin_arm, axis=Axis.Z, angular_range=(0, 360))
    j7.connect_to(j8, position=6, angle=60)
    j9 = BallJoint("socket", base, Plane(base.faces().sort_by(Axis.X)[0]).location)
    ball = JointBox(2, 2, 2, 0.99)
    j10 = RigidJoint("ball", ball, Location(Vector(0, 0, 1)))
    j9.connect_to(j10, angles=(10, 20, 30))
    return {
        "joints_base": base,
        "joints_fixed_arm": fixed_arm,
        "joints_hinge_arm": hinge_arm,
        "joints_slider_arm": slider_arm,
        "joints_screw_arm": screw_arm,
        "joints_pin_arm": pin_arm,
        "joints_ball": ball,
    }


# executable name in build/examples -> example
EXAMPLES = {
    "tea_cup": tea_cup,
    "vase": vase,
    "heat_exchanger": heat_exchanger,
    "handle": handle,
    "lego": lego,
    "key_cap": key_cap,
    "pillow_block": pillow_block,
    "din_rail": din_rail,
    "pegboard_j_hook": pegboard_j_hook,
    "circuit_board": circuit_board,
    "intersecting_chamfers": intersecting_chamfers,
    "holes": holes,
    "boxes_on_faces": boxes_on_faces,
    "multiple_workplanes": multiple_workplanes,
    "loft": loft_example,
    "extrude": extrude_example,
    "clock": clock,
    "roller_coaster": roller_coaster,
    "canadian_flag": canadian_flag,
    "custom_sketch_objects": custom_sketch_objects,
    "projection": projection,
    "build123d_logo": build123d_logo,
    "build123d_customizable_logo": build123d_customizable_logo,
    "joints": joints,
}


def main():
    only = sys.argv[1:]
    for example, build in EXAMPLES.items():
        if only and example not in only:
            continue
        try:
            shapes = build()
        except Exception as exc:  # pylint: disable=broad-exception-caught
            print(json.dumps({"example": example, "name": example, "error": f"{type(exc).__name__}: {exc}"}))
            continue
        for name, shape in shapes.items():
            print(json.dumps({"example": example, **metrics(name, shape)}), flush=True)


if __name__ == "__main__":
    main()
