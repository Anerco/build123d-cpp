// Import/export, hidden-line views, meshes, serialization and viewers, compared with
// cases_io_py.py. Files are written to /tmp/opencode/scratch-io/compare/cpp (the Python
// side writes to .../py), and each case reports what both sides read back.
#include "case_registry.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

using namespace build123d;
using compare::V;

namespace {

/// An enum value's name without its class ("LINE"), as values.py reports Python enums.
template <class E>
std::string enum_name(E value) {
    const std::string s = to_string(value);
    return s.substr(s.find('.') + 1);
}

V edge_summary(const Edge& e) {
    return V::list({enum_name(e.geom_type()), e.length(), e.start_point(), e.end_point()});
}

V edges_summary(const ShapeList<Edge>& edges) {
    std::vector<V> items;
    for (const auto& e : edges) items.push_back(edge_summary(e));
    return V::list(items);
}

V viewport(const std::pair<ShapeList<Edge>, ShapeList<Edge>>& vh) {
    return V::obj({{"visible", edges_summary(vh.first)}, {"hidden", edges_summary(vh.second)}});
}

}  // namespace

COMPARE_CASES {
    // ---- project_to_viewport
    compare::add_value("io_hlr_box", [] { return viewport(Box(1, 2, 3).project_to_viewport(Vector(10, -10, 10))); });
    compare::add_value("io_hlr_holed_part", [] {
        const Part part = Box(10, 10, 10) - Cylinder(3, 10);
        return viewport(part.project_to_viewport(Vector(-20, 15, 30), Vector(0, 1, 0)));
    });
    compare::add_value("io_hlr_look_at", [] {
        const Part part = Box(10, 10, 10) - Pos(2, 0, 0) * Sphere(4);
        return viewport(part.project_to_viewport(Vector(0, -50, 0), Vector(0, 0, 1), Vector(5, 0, 0)));
    });
    compare::add_value("io_hlr_perspective", [] {
        const Part part = Cylinder(5, 10) + Pos(0, 0, 5) * Box(12, 3, 3);
        return viewport(part.project_to_viewport(Vector(30, -40, 25), Vector(0, 0, 1), std::nullopt, 60.0));
    });
    compare::add_value("io_hlr_solid", [] {
        const Solid s = Solid::make_torus(10, 2);
        return viewport(s.project_to_viewport(Vector(5, -10, 20)));
    });
    compare::add_value("io_hlr_face", [] {
        const Face f = Face::make_rect(4, 2, Plane::XZ);
        return viewport(f.project_to_viewport(Vector(1, -10, 3), Vector(0, 0, 1)));
    });
    compare::add_value("io_hlr_shell", [] {
        const Shell s = Shell::extrude(Wire::make_circle(3), Vector(0, 0, 4));
        return viewport(s.project_to_viewport(Vector(10, 10, 10)));
    });
    compare::add_value("io_hlr_edge", [] {
        const Edge e = Edge::make_circle(5, Plane::XY, 0, 270);
        return viewport(e.project_to_viewport(Vector(3, -20, 10)));
    });
    compare::add_value("io_hlr_wire", [] {
        const Wire w = Wire::make_polygon({{0, 0, 0}, {5, 0, 0}, {5, 5, 2}, {0, 5, 0}});
        return viewport(w.project_to_viewport(Vector(-7, -12, 9)));
    });
    compare::add_value("io_hlr_sketch", [] {
        const Sketch s = Circle(5) - Rectangle(3, 3);
        return viewport(s.project_to_viewport(Vector(0, -10, 10)));
    });
    compare::add_value("io_hlr_zero_focus", [] {
        return viewport(Box(2, 3, 4).project_to_viewport(Vector(10, 10, 10), Vector(0, 0, 1), Vector(1, 0, 0), 0.0));
    });
    compare::add_value("io_hlr_empty", [] { return viewport(Part().project_to_viewport(Vector(1, 1, 1))); });
}
