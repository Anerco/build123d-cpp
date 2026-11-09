"""Python twins of cases_joints.cpp: joints and the assembly tree."""

from build123d import (
    Axis,
    BallJoint,
    Box,
    Compound,
    CylindricalJoint,
    Cylinder,
    LinearJoint,
    Location,
    Plane,
    Pos,
    RevoluteJoint,
    RigidJoint,
    Rot,
    Sphere,
)

from values import V


def loc_values(loc):
    return {"position": loc.position, "orientation": loc.orientation}


def arm():
    return Box(1, 2, 6)


def block():
    return Pos(1, 2, 3) * Rot(10, 20, 30) * Box(10, 10, 10)


def make_tree():
    a = Pos(5, 0, 0) * Box(1, 1, 1)
    a.label = "a"
    b = Pos(0, 5, 0) * Sphere(1)
    b.label = "b"
    c = Rot(0, 0, 45) * Pos(0, 0, 5) * Cylinder(1, 2)
    c.label = "c"
    sub = Compound(children=[b, c], label="sub")
    sub.move(Location((10, 0, 0), (0, 0, 90)))
    root = Compound(children=[a, sub], label="root")
    root.move(Location((0, 0, 7), (30, 0, 0)))
    return root


def joints_rigid():
    base, other = block(), arm()
    j1 = RigidJoint("a", base, Location((5, 0, 0), (0, 90, 0)))
    j2 = RigidJoint("b", other, Location((0, 0, -3)))
    rel = j1.relative_to(j2)
    j1.connect_to(j2)
    return V(
        {
            "relative": loc_values(rel),
            "other": loc_values(other.location),
            "j1": loc_values(j1.location),
            "j2": loc_values(j2.location),
            "connected": j1.connected_to is j2,
        }
    )


def joints_revolute():
    base, other = block(), arm()
    hinge = RevoluteJoint("h", base, Axis((5, 5, 0), (0, 0, 1)), angular_range=(0, 270))
    pin = RigidJoint("p", other, Location((0, 1, 0)))
    out = []
    for angle in (0.0, 45.0, 270.0):
        hinge.connect_to(pin, angle=angle)
        out.append(loc_values(other.location))
    out.append(loc_values(hinge.location))
    out.append(hinge._angle)  # pylint: disable=protected-access
    return V(out)


def joints_revolute_reference():
    base, other = block(), arm()
    hinge = RevoluteJoint("h", base, Axis((0, 0, 0), (1, 0, 0)), angle_reference=(0, 1, 1), angular_range=(0, 360))
    pin = RigidJoint("p", other, Location((0, 0, 3), (0, 90, 0)))
    hinge.connect_to(pin, angle=30)
    first = loc_values(other.location)
    pin.connect_to(hinge, angle=120)
    return V([first, loc_values(base.location), hinge.angle_reference])


def joints_linear():
    base, slider, pinned = block(), arm(), Box(1, 1, 1)
    slide = LinearJoint("s", base, Axis((0, 0, 5), (1, 0, 0)), linear_range=(-5, 5))
    s = RigidJoint("r", slider, Location((0, 0, -3)))
    pin = RevoluteJoint("p", pinned, Axis.Z, angular_range=(0, 180))
    slide.connect_to(s)
    middle = loc_values(slider.location)
    slide.connect_to(s, position=4)
    at4 = loc_values(slider.location)
    slide.connect_to(pin, position=-2, angle=135)
    return V([middle, at4, loc_values(pinned.location), slide.position, slide.angle])


def joints_cylindrical():
    base, screw = block(), arm()
    hole = CylindricalJoint("c", base, Axis((0, -5, 0), (0, 1, 0)), linear_range=(-10, 10), angular_range=(0, 180))
    tip = RigidJoint("t", screw, Location((0, 0, 3)))
    hole.connect_to(tip)
    defaults = loc_values(screw.location)
    hole.connect_to(tip, position=-1, angle=90)
    return V([defaults, loc_values(screw.location), hole.position, hole.angle])


def joints_ball():
    base, ball = block(), Sphere(1)
    socket = BallJoint("b", base, Location((-5, 0, 0)), angular_range=((0, 360), (0, 360), (0, 360)))
    knob = RigidJoint("k", ball, Location((0, 0, 1)))
    socket.connect_to(knob)
    defaults = loc_values(ball.location)
    socket.connect_to(knob, angles=(10, 20, 30))
    return V([defaults, loc_values(ball.location), loc_values(socket.location)])


def joints_ball_out_of_range():
    base, ball = block(), Sphere(1)
    socket = BallJoint(
        "b", base, Location((-5, 0, 0)), angular_range=((0, 360), (0, 360), (0, 360)), angle_reference=Plane.XZ
    )
    knob = RigidJoint("k", ball, Location((0, 0, 1)))
    socket.connect_to(knob, angles=(10, 20, 30))
    return V(True)


def joints_rigid_to_others():
    base = block()
    a, b, c, d = arm(), arm(), arm(), arm()
    r = RigidJoint("r", base, Location((0, 0, 5)))
    rev = RevoluteJoint("rev", a, Axis.X, angular_range=(0, 90))
    lin = LinearJoint("lin", b, Axis.Y, linear_range=(0, 4))
    cyl = CylindricalJoint("cyl", c, Axis.Z, linear_range=(0, 2), angular_range=(0, 90))
    ball = BallJoint("ball", d)
    r.connect_to(rev, angle=30)
    r.connect_to(lin, position=3)
    r.connect_to(cyl, position=1, angle=45)
    r.connect_to(ball, angles=(5, 10, 15))
    return V([loc_values(s.location) for s in (a, b, c, d)])


def joints_generic_args():
    base, other = block(), arm()
    RevoluteJoint("h", base, Axis.Y, angular_range=(0, 180))
    RigidJoint("p", other, Location((1, 0, 0)))
    base.joints["h"].connect_to(other.joints["p"], angle=60)
    return V(
        {
            "other": loc_values(other.location),
            "keys": list(base.joints.keys()),
            "relative": loc_values(base.joints["h"].relative_to(other.joints["p"], angle=10)),
        }
    )


def joints_angle_out_of_range():
    base, other = block(), arm()
    hinge = RevoluteJoint("h", base, Axis.Z, angular_range=(0, 90))
    pin = RigidJoint("p", other)
    hinge.connect_to(pin, angle=120)
    return V(True)


def joints_position_out_of_range():
    base, other = block(), arm()
    slide = LinearJoint("s", base, Axis.Z, linear_range=(0, 1))
    pin = RigidJoint("p", other)
    slide.connect_to(pin, position=2)
    return V(True)


def joints_wrong_type():
    base, other = block(), arm()
    a = RevoluteJoint("a", base)
    b = RevoluteJoint("b", other)
    a.connect_to(b)
    return V(True)


def joints_bad_angle_reference():
    RevoluteJoint("h", block(), Axis.Z, angle_reference=(0, 0, 1))
    return V(True)


def joints_moved_copy():
    base = block()
    RigidJoint("a", base, Location((5, 0, 0)))
    LinearJoint("l", base, Axis.X, linear_range=(0, 10))
    moved = Pos(0, 0, 20) * Rot(0, 0, 90) * base
    return V(
        {
            "original": loc_values(base.joints["a"].location),
            "moved": loc_values(moved.joints["a"].location),
            "moved_axis": loc_values(moved.joints["l"].location),
            "count": len(moved.joints),
        }
    )


def joints_symbols():
    base = block()
    hinge = RevoluteJoint("h", base, Axis((5, 5, 0), (0, 0, 1)))
    slide = LinearJoint("s", base, Axis((0, 0, 5), (1, 0, 0)), linear_range=(-5, 5))
    hole = CylindricalJoint("c", base, Axis((0, -5, 0), (0, 1, 0)), linear_range=(-10, 10))
    return Compound([hinge.symbol, slide.symbol, hole.symbol])


def joints_symbol_rigid():
    base = block()
    corner = RigidJoint("r", base, Location((5, 5, 5), (0, 0, 30)))
    return corner.symbol


def joints_symbol_ball():
    base = block()
    socket = BallJoint(
        "b", base, Location((-5, 0, 0)), angular_range=((0, 360), (0, 360), (0, 360)), angle_reference=Plane.XZ
    )
    return socket.symbol


def tree_navigation():
    root = make_tree()
    sub = root.children[1]
    c = sub.children[1]
    return V(
        {
            "depth": [root.depth, sub.depth, c.depth],
            "height": [root.height, sub.height, c.height],
            "size": [root.size, sub.size, c.size],
            "is_leaf": [root.is_leaf, sub.is_leaf, c.is_leaf],
            "is_root": [root.is_root, sub.is_root, c.is_root],
            "path": [n.label for n in c.path],
            "ancestors": [n.label for n in c.ancestors],
            "descendants": [n.label for n in root.descendants],
            "leaves": [n.label for n in root.leaves],
            "siblings": [n.label for n in c.siblings],
            "root": c.root.label,
            "parent": c.parent.label,
        }
    )


def tree_global_location():
    root = make_tree()
    return V(
        [
            {"label": n.label, "location": loc_values(n.location), "global": loc_values(n.global_location)}
            for n in root.descendants
        ]
    )


def tree_children_rebuild():
    assy = Compound()
    assy.children = [Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)]
    cylinder = Pos(0, 4, 0) * Cylinder(1, 3)
    cylinder.parent = assy
    return assy


def tree_children_detach():
    assy = Compound(children=[Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1), Pos(0, 4, 0) * Cylinder(1, 3)])
    assy.children[1].parent = None
    return assy


CASES = {
    "joints_rigid": joints_rigid,
    "joints_revolute": joints_revolute,
    "joints_revolute_reference": joints_revolute_reference,
    "joints_linear": joints_linear,
    "joints_cylindrical": joints_cylindrical,
    "joints_ball": joints_ball,
    "joints_ball_out_of_range": joints_ball_out_of_range,
    "joints_rigid_to_others": joints_rigid_to_others,
    "joints_generic_args": joints_generic_args,
    "joints_angle_out_of_range": joints_angle_out_of_range,
    "joints_position_out_of_range": joints_position_out_of_range,
    "joints_wrong_type": joints_wrong_type,
    "joints_bad_angle_reference": joints_bad_angle_reference,
    "joints_moved_copy": joints_moved_copy,
    "joints_symbols": joints_symbols,
    "joints_symbol_rigid": joints_symbol_rigid,
    "joints_symbol_ball": joints_symbol_ball,
    "tree_navigation": tree_navigation,
    "tree_global_location": tree_global_location,
    "tree_children_rebuild": tree_children_rebuild,
    "tree_children_detach": tree_children_detach,
}
