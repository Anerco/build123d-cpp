"""Python twins of cases_integration.cpp."""

from build123d import (
    Align,
    Axis,
    BasePartObject,
    Box,
    Cylinder,
    LinearJoint,
    Location,
    Plane,
    Polyline,
    Pos,
    RigidJoint,
    Select,
    Sphere,
    Unit,
    make_brake_formed,
)

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


def base_part_object_joints():
    part = Pos(1, 0, 0) * Box(2, 2, 2)
    part.label = "custom"
    RigidJoint("a", part, Location((2, 0, 0)))
    LinearJoint("l", part, Axis.X, linear_range=(0, 4))
    rotated = BasePartObject(part, rotation=(0, 0, 90))
    aligned = BasePartObject(part, align=Align.MIN)

    def info(p):
        return {
            "keys": list(p.joints.keys()),
            "label": p.label,
            "a": p.joints["a"].location.position,
            "l": p.joints["l"].location.position,
            "l_dir": Plane(p.joints["l"].location).z_dir,
        }

    return V([info(rotated), info(aligned)])


def unit_tables():
    import build123d  # pylint: disable=import-outside-toplevel

    per_meter = [build123d.UNITS_PER_METER[u] for u in (Unit.IN, Unit.FT, Unit.MC, Unit.MM, Unit.CM, Unit.M)]
    per_kilogram = [build123d.UNITS_PER_KILOGRAM[u] for u in (Unit.G, Unit.KG, Unit.LB)]
    return V([per_meter, per_kilogram, [build123d.G, build123d.KG, build123d.LB, build123d.G_PER_LB]])


CASES = {
    "integ_unit_tables": unit_tables,
    "integ_brake_select": brake_select,
    "integ_joints_through_booleans": joints_through_booleans,
    "integ_base_part_object_joints": base_part_object_joints,
}
