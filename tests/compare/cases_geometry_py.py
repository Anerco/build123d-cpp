"""Python twins of cases_geometry.cpp."""

from build123d import Extrinsic, Location, Plane, Vector

from values import V


def plane_values(p):
    return V({"origin": p.origin, "x_dir": p.x_dir, "y_dir": p.y_dir, "z_dir": p.z_dir})


def location_values(loc):
    return V({"position": loc.position, "orientation": loc.orientation})


def location_set_position_orientation():
    loc = Location((1, 2, 3), (10, 20, 30))
    loc.position = (4, 5, 6)
    moved = location_values(loc)
    loc.orientation = (0, 45, 90)
    return V([moved, location_values(loc)])


CASES = {
    "plane_shift_origin": lambda: plane_values(Plane.XZ.shift_origin((1, 0, 2))),
    "plane_rotated_extrinsic": lambda: plane_values(Plane.XY.rotated((10, 20, 30), Extrinsic.ZYX)),
    "plane_location_between": lambda: location_values(
        Plane.XY.location_between(Plane((1, 2, 3), x_dir=(0, 1, 0), z_dir=(1, 0, 0)))
    ),
    "location_set_position_orientation": location_set_position_orientation,
}
