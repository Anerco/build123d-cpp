"""Python twins of cases_integration.cpp."""

from build123d import Box, Cylinder, Location, Polyline, Pos, RigidJoint, Select, Sphere, make_brake_formed

from values import V


def brake_select():
    part = make_brake_formed(0.5, 2, Polyline((0, 0), (5, 6), (10, 1)))
    return V(
        {
            "new_edges": len(part.edges(Select.NEW)),
            "new_edge_length": sum(e.length for e in part.edges(Select.NEW)),
            "last_edges": len(part.edges(Select.LAST)),
            "new_faces": len(part.faces(Select.NEW)),
            "new_face_area": sum(f.area for f in part.faces(Select.NEW)),
            "last_faces": len(part.faces(Select.LAST)),
        }
    )


def joint_info(p):
    return {
        "keys": list(p.joints.keys()),
        "label": p.label,
        "location": p.joints["a"].location.position if "a" in p.joints else None,
    }


def joints_through_booleans():
    base = Box(4, 4, 4)
    base.label = "base"
    RigidJoint("a", base, Location((2, 0, 0)))
    separate = base + Pos(5, 0, 0) * Box(1, 1, 1)
    fused = base + Pos(2, 0, 0) * Box(2, 2, 2)
    cut = base - Cylinder(1, 4)
    common = base & Sphere(3)
    moved = Pos(0, 0, 10) * base
    return V([joint_info(p) for p in (separate, fused, cut, common, moved)])


CASES = {
    "integ_brake_select": brake_select,
    "integ_joints_through_booleans": joints_through_booleans,
}
