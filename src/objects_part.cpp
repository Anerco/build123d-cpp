#include <algorithm>
#include <array>
#include <cmath>
#include <map>

#include "build123d/objects.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

/// BasePartObject: align by bounding box, then rotate.
TopoDS_Shape base_part(Shape part, const Rotation& rotation, std::optional<Align3> align) {
    if (align) part.move(Location(part.bounding_box().to_align_offset(*align)));
    const Shape transformed = rotation != Rotation() ? part.moved(rotation) : part;
    if (transformed.wrapped().ShapeType() == TopAbs_COMPOUND) return transformed.wrapped();
    return make_compound({transformed.wrapped()});
}

Plane down_plane(const Vector& origin = Vector(0, 0, 0)) { return Plane(origin, std::nullopt, Vector(0, 0, -1)); }

// ---- incremental 3D convex hull (triangles, outward normals)
struct HullFace {
    std::array<std::size_t, 3> v;
    Vector normal;
    double offset;
};

std::vector<std::array<std::size_t, 3>> convex_hull_3d(const std::vector<Vector>& pts) {
    const std::size_t n = pts.size();
    if (n < 4) throw std::invalid_argument("ConvexPolyhedron needs at least 4 points");
    double scale = 0.0;
    for (const auto& p : pts) scale = std::max({scale, std::abs(p.X), std::abs(p.Y), std::abs(p.Z)});
    const double eps = 1e-9 * std::max(scale, 1.0);

    // initial tetrahedron
    std::size_t i0 = 0, i1 = n, i2 = n, i3 = n;
    for (std::size_t i = 1; i < n && i1 == n; ++i) {
        if ((pts[i] - pts[i0]).length() > eps) i1 = i;
    }
    for (std::size_t i = 1; i < n && i2 == n && i1 != n; ++i) {
        if ((pts[i1] - pts[i0]).cross(pts[i] - pts[i0]).length() > eps) i2 = i;
    }
    for (std::size_t i = 1; i < n && i3 == n && i2 != n; ++i) {
        const Vector nrm = (pts[i1] - pts[i0]).cross(pts[i2] - pts[i0]);
        if (std::abs(nrm.dot(pts[i] - pts[i0])) > eps * nrm.length()) i3 = i;
    }
    if (i3 == n) throw std::invalid_argument("ConvexPolyhedron points are coplanar");

    Vector centroid = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) / 4.0;
    std::vector<HullFace> faces;
    auto make_face = [&](std::size_t a, std::size_t b, std::size_t c) {
        Vector nrm = (pts[b] - pts[a]).cross(pts[c] - pts[a]);
        if (nrm.dot(pts[a] - centroid) < 0) {
            std::swap(b, c);
            nrm = -nrm;
        }
        nrm = nrm.normalized();
        faces.push_back({{a, b, c}, nrm, nrm.dot(pts[a])});
    };
    make_face(i0, i1, i2);
    make_face(i0, i1, i3);
    make_face(i0, i2, i3);
    make_face(i1, i2, i3);

    for (std::size_t p = 0; p < n; ++p) {
        if (p == i0 || p == i1 || p == i2 || p == i3) continue;
        std::vector<bool> visible(faces.size(), false);
        bool any = false;
        for (std::size_t f = 0; f < faces.size(); ++f) {
            if (faces[f].normal.dot(pts[p]) - faces[f].offset > eps) {
                visible[f] = true;
                any = true;
            }
        }
        if (!any) continue;
        // horizon: directed edges of visible faces whose reverse is not on a visible face
        std::map<std::pair<std::size_t, std::size_t>, int> edge_count;
        for (std::size_t f = 0; f < faces.size(); ++f) {
            if (!visible[f]) continue;
            for (int k = 0; k < 3; ++k) edge_count[{faces[f].v[k], faces[f].v[(k + 1) % 3]}]++;
        }
        std::vector<std::pair<std::size_t, std::size_t>> horizon;
        for (const auto& [e, count] : edge_count) {
            if (edge_count.find({e.second, e.first}) == edge_count.end()) horizon.push_back(e);
        }
        std::vector<HullFace> kept;
        for (std::size_t f = 0; f < faces.size(); ++f) {
            if (!visible[f]) kept.push_back(faces[f]);
        }
        faces = kept;
        for (const auto& e : horizon) {
            const Vector nrm = (pts[e.second] - pts[e.first]).cross(pts[p] - pts[e.first]).normalized();
            faces.push_back({{e.first, e.second, p}, nrm, nrm.dot(pts[e.first])});
        }
    }
    std::vector<std::array<std::size_t, 3>> out;
    for (const auto& f : faces) out.push_back(f.v);
    return out;
}

TopoDS_Shape fused(const Solid& a, const std::vector<Solid>& others) {
    std::vector<TopoDS_Shape> tools;
    for (const auto& o : others) tools.push_back(o.wrapped());
    return bool_op(BoolOp::FUSE, {a.wrapped()}, tools);
}

}  // namespace

Box::Box(double length, double width, double height, const Rotation& rotation, Align3 align)
    : Part(base_part(Solid::make_box(length, width, height), rotation, align)) {}

Box::Box(double length, double width, double height, Align3 align) : Box(length, width, height, Rotation(), align) {}

Cylinder::Cylinder(double radius, double height, double arc_size, const Rotation& rotation, Align3 align)
    : Part(base_part(Solid::make_cylinder(radius, height, Plane::XY, arc_size), rotation, align)) {}

Cylinder::Cylinder(double radius, double height, Align3 align)
    : Cylinder(radius, height, 360.0, Rotation(), align) {}

Cylinder::Cylinder(double radius, double height, const Rotation& rotation, Align3 align)
    : Cylinder(radius, height, 360.0, rotation, align) {}

Cone::Cone(double bottom_radius, double top_radius, double height, double arc_size, const Rotation& rotation,
           Align3 align)
    : Part(base_part(Solid::make_cone(bottom_radius, top_radius, height, Plane::XY, arc_size), rotation, align)) {}

Cone::Cone(double bottom_radius, double top_radius, double height, Align3 align)
    : Cone(bottom_radius, top_radius, height, 360.0, Rotation(), align) {}

Cone::Cone(double bottom_radius, double top_radius, double height, const Rotation& rotation, Align3 align)
    : Cone(bottom_radius, top_radius, height, 360.0, rotation, align) {}

Sphere::Sphere(double radius, double arc_size1, double arc_size2, double arc_size3, const Rotation& rotation,
               Align3 align)
    : Part(base_part(Solid::make_sphere(radius, Plane::XY, arc_size1, arc_size2, arc_size3), rotation, align)) {}

Sphere::Sphere(double radius, Align3 align) : Sphere(radius, -90.0, 90.0, 360.0, Rotation(), align) {}

Sphere::Sphere(double radius, const Rotation& rotation, Align3 align)
    : Sphere(radius, -90.0, 90.0, 360.0, rotation, align) {}

Torus::Torus(double major_radius, double minor_radius, double minor_start_angle, double minor_end_angle,
             double major_angle, const Rotation& rotation, Align3 align)
    : Part(base_part(Solid::make_torus(major_radius, minor_radius, Plane::XY, minor_start_angle, minor_end_angle,
                                       major_angle),
                     rotation, align)) {}

Torus::Torus(double major_radius, double minor_radius, Align3 align)
    : Torus(major_radius, minor_radius, 0.0, 360.0, 360.0, Rotation(), align) {}

Torus::Torus(double major_radius, double minor_radius, const Rotation& rotation, Align3 align)
    : Torus(major_radius, minor_radius, 0.0, 360.0, 360.0, rotation, align) {}

namespace {
Solid wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax, double zmax) {
    if (xsize <= 0 || ysize <= 0 || zsize <= 0) {
        throw std::invalid_argument("xsize, ysize & zsize must all be greater than zero");
    }
    return Solid::make_wedge(xsize, ysize, zsize, xmin, zmin, xmax, zmax);
}
}  // namespace

Wedge::Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax, double zmax,
             const Rotation& rotation, Align3 align)
    : Part(base_part(wedge(xsize, ysize, zsize, xmin, zmin, xmax, zmax), rotation, align)) {}

Wedge::Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax, double zmax,
             Align3 align)
    : Wedge(xsize, ysize, zsize, xmin, zmin, xmax, zmax, Rotation(), align) {}

Hole::Hole(double radius, double depth)
    : Part(base_part(Solid::make_cylinder(radius, 2 * depth, down_plane()), Rotation(), Align3(Align::CENTER))) {}

CounterBoreHole::CounterBoreHole(double radius, double counter_bore_radius, double counter_bore_depth, double depth)
    : Part(base_part(Shape(fused(Solid::make_cylinder(radius, depth, down_plane()),
                                 {Solid::make_cylinder(counter_bore_radius, counter_bore_depth + depth,
                                                       Plane(Vector(0, 0, -counter_bore_depth)))})),
                     Rotation(), std::nullopt)) {}

CounterSinkHole::CounterSinkHole(double radius, double counter_sink_radius, double depth, double counter_sink_angle)
    : Part(base_part(
          Shape(fused(Solid::make_cylinder(radius, depth, down_plane()),
                      {Solid::make_cone(counter_sink_radius, 0.0,
                                        counter_sink_radius / std::tan(counter_sink_angle / 2.0 * DEG2RAD), down_plane()),
                       Solid::make_cylinder(counter_sink_radius, depth)})),
          Rotation(), std::nullopt)) {}

namespace {
Solid convex_polyhedron(const std::vector<Vector>& points) {
    std::vector<Face> faces;
    for (const auto& tri : convex_hull_3d(points)) {
        faces.emplace_back(Wire::make_polygon({points[tri[0]], points[tri[1]], points[tri[2]]}));
    }
    return Solid(Shell(faces)).clean();
}
}  // namespace

ConvexPolyhedron::ConvexPolyhedron(const std::vector<Vector>& points, const Rotation& rotation, Align3 align)
    : Part(base_part(convex_polyhedron(points), rotation,
                     (align.x == Align::NONE && align.y == Align::NONE && align.z == Align::NONE)
                         ? std::nullopt
                         : std::optional<Align3>(align))) {}

}  // namespace build123d
