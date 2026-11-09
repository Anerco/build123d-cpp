"""Python twins of cases_integration.cpp."""

from build123d import (
    Align,
    Axis,
    BasePartObject,
    Box,
    Color,
    Compound,
    Cylinder,
    Edge,
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


def color_material_lookup():
    def c(color):
        return list(tuple(color)) if color is not None else None

    def m(material):
        return material.name if material is not None else None

    out = []
    assembly = Compound(children=[Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)])
    assembly.color = Color("red")
    out.append(c(assembly.children[0].color))
    assembly.color = Color("blue")
    out.append(c(assembly.children[0].color))
    out.append(c(assembly.children[1].color))
    out.append(c(assembly.children[0]._color))  # pylint: disable=protected-access
    inner = Compound(children=[Box(1, 1, 1)])
    outer = Compound(children=[inner])
    outer.color = Color("green")
    out.append(c(outer.children[0].children[0].color))
    out.append(c(outer.children[0]._color))  # pylint: disable=protected-access
    box = Box(1, 1, 1)
    box.material = "aluminum"
    out.append(c(box.color))
    out.append(m(box.material))
    box.color = Color("red")
    box.material = None
    out.append(c(box.color))
    metal = Compound(children=[Box(1, 1, 1)])
    metal.material = "Mild Steel"
    out.append(m(metal.children[0].material))
    out.append(c(metal.children[0].color))
    holder = Compound(children=[Box(1, 1, 1)])
    holder.color = Color("red")
    target = Box(2, 2, 2)
    holder.children[0].copy_attributes_to(target)
    out.append(c(target._color))  # pylint: disable=protected-access
    copper = Box(1, 1, 1)
    copper.material = "copper"
    obj = BasePartObject(copper)
    out.append(c(obj.color))
    out.append(m(obj.material))
    return V(out)


def property_setters():
    axis = Axis((0, 0, 0), (1, 0, 0))
    axis.direction = (0, 0, 2)
    axis.position = (1, 2, 3)
    try:
        axis.direction = (0, 0, 0)
        null_direction_rejected = False
    except Exception:  # pylint: disable=broad-exception-caught
        null_direction_rejected = True
    box = Box(1, 2, 3)
    box.position = (5, 0, 0)
    box.orientation = (0, 0, 90)
    bb = box.bounding_box()
    plane = Plane((0, 0, 0))
    plane.origin = (1, 1, 1)
    plane.x_dir = (1, 1, 0)
    edge = Edge.make_line((0, 0), (1, 0))
    edge.label = "kept"
    edge.wrapped = Edge.make_line((0, 0), (0, 2)).wrapped
    return V([axis.direction, axis.position, null_direction_rejected, box.position, box.orientation, bb.min, bb.max,
              plane.origin, plane.x_dir, plane.y_dir, plane.z_dir, edge.length, edge.label])


def version():
    import build123d  # pylint: disable=import-outside-toplevel
    from build123d import _version  # pylint: disable=import-outside-toplevel

    return V([build123d.__version__, list(_version.version_tuple)])


CASES = {
    "integ_version": version,
    "integ_property_setters": property_setters,
    "integ_color_material_lookup": color_material_lookup,
    "integ_unit_tables": unit_tables,
    "integ_brake_select": brake_select,
    "integ_joints_through_booleans": joints_through_booleans,
    "integ_base_part_object_joints": base_part_object_joints,
}
