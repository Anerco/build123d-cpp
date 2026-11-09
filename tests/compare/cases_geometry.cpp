// Geometry helpers compared value by value with cases_geometry_py.py.
#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

V plane_values(const Plane& p) {
    return V::obj({{"origin", p.origin}, {"x_dir", p.x_dir}, {"y_dir", p.y_dir}, {"z_dir", p.z_dir}});
}

V location_values(const Location& l) {
    return V::obj({{"position", l.position()}, {"orientation", l.orientation()}});
}

}  // namespace

COMPARE_CASES {
    compare::add_value("plane_shift_origin", [] { return plane_values(Plane::XZ.shift_origin(Vector(1, 0, 2))); });
    compare::add_value("plane_rotated_extrinsic", [] {
        return plane_values(Plane::XY.rotated(Vector(10, 20, 30), Extrinsic::ZYX));
    });
    compare::add_value("plane_location_between", [] {
        return location_values(Plane::XY.location_between(Plane(Vector(1, 2, 3), Vector(0, 1, 0), Vector(1, 0, 0))));
    });
    compare::add_value("location_set_position_orientation", [] {
        Location loc(Vector(1, 2, 3), Vector(10, 20, 30));
        loc.set_position(Vector(4, 5, 6));
        const V moved = location_values(loc);
        loc.set_orientation(Vector(0, 45, 90));
        return V::list({moved, location_values(loc)});
    });
}
