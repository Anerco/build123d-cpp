// Features that span the work areas, compared with cases_integration_py.py.
#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

template <class L>
double total_length(const L& edges) {
    double s = 0;
    for (const auto& e : edges) s += e.length();
    return s;
}

template <class L>
double total_area(const L& faces) {
    double s = 0;
    for (const auto& f : faces) s += f.area();
    return s;
}

V joint_info(const Part& p) {
    return V::obj({{"keys", V::list_of(p.joints.keys())},
                   {"label", p.label},
                   {"location", p.joints.contains("a") ? V(p.joints["a"].location().position()) : V(nullptr)}});
}

}  // namespace

COMPARE_CASES {
    compare::add_value("integ_brake_select", [] {
        const Part part = make_brake_formed(0.5, 2, Polyline({{0, 0}, {5, 6}, {10, 1}}));
        return V::obj({{"new_edges", part.edges(Select::NEW).size()},
                       {"new_edge_length", total_length(part.edges(Select::NEW))},
                       {"last_edges", part.edges(Select::LAST).size()},
                       {"new_faces", part.faces(Select::NEW).size()},
                       {"new_face_area", total_area(part.faces(Select::NEW))},
                       {"last_faces", part.faces(Select::LAST).size()}});
    });
    compare::add_value("integ_joints_through_booleans", [] {
        Part base = Box(4, 4, 4);
        base.label = "base";
        RigidJoint("a", base, Location(Vector(2, 0, 0)));
        const Part separate = base + Pos(5, 0, 0) * Box(1, 1, 1);
        const Part fused = base + Pos(2, 0, 0) * Box(2, 2, 2);
        const Part cut = base - Cylinder(1, 4);
        const Part common = base & Sphere(3);
        const Part moved = Pos(0, 0, 10) * base;
        return V::list({joint_info(separate), joint_info(fused), joint_info(cut), joint_info(common), joint_info(moved)});
    });
}
