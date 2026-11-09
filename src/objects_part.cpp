#include <algorithm>
#include <array>
#include <cmath>
#include <map>

#include "build123d/objects.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

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

Box::Box(double length_in, double width_in, double height, const Rotation& rotation, Align3 align)
    : BasePartObject(Solid::make_box(length_in, width_in, height), rotation, align),
      length(length_in),
      width(width_in),
      box_height(height) {}

Box::Box(double length, double width, double height, Align3 align) : Box(length, width, height, Rotation(), align) {}

Cylinder::Cylinder(double radius_in, double height, double arc_size_in, const Rotation& rotation, Align3 align_in)
    : BasePartObject(Solid::make_cylinder(radius_in, height, Plane::XY, arc_size_in), rotation, align_in),
      radius(radius_in),
      cylinder_height(height),
      arc_size(arc_size_in),
      align(align_in) {}

Cylinder::Cylinder(double radius, double height, Align3 align)
    : Cylinder(radius, height, 360.0, Rotation(), align) {}

Cylinder::Cylinder(double radius, double height, const Rotation& rotation, Align3 align)
    : Cylinder(radius, height, 360.0, rotation, align) {}

Cone::Cone(double bottom, double top, double height, double arc_size_in, const Rotation& rotation, Align3 align_in)
    : BasePartObject(Solid::make_cone(bottom, top, height, Plane::XY, arc_size_in), rotation, align_in),
      bottom_radius(bottom),
      top_radius(top),
      cone_height(height),
      arc_size(arc_size_in),
      align(align_in) {}

Cone::Cone(double bottom_radius, double top_radius, double height, Align3 align)
    : Cone(bottom_radius, top_radius, height, 360.0, Rotation(), align) {}

Cone::Cone(double bottom_radius, double top_radius, double height, const Rotation& rotation, Align3 align)
    : Cone(bottom_radius, top_radius, height, 360.0, rotation, align) {}

Sphere::Sphere(double radius_in, double a1, double a2, double a3, const Rotation& rotation, Align3 align_in)
    : BasePartObject(Solid::make_sphere(radius_in, Plane::XY, a1, a2, a3), rotation, align_in),
      radius(radius_in),
      arc_size1(a1),
      arc_size2(a2),
      arc_size3(a3),
      align(align_in) {}

Sphere::Sphere(double radius, Align3 align) : Sphere(radius, -90.0, 90.0, 360.0, Rotation(), align) {}

Sphere::Sphere(double radius, const Rotation& rotation, Align3 align)
    : Sphere(radius, -90.0, 90.0, 360.0, rotation, align) {}

Torus::Torus(double major, double minor, double minor_start, double minor_end, double major_angle_in,
             const Rotation& rotation, Align3 align_in)
    : BasePartObject(Solid::make_torus(major, minor, Plane::XY, minor_start, minor_end, major_angle_in), rotation,
                     align_in),
      major_radius(major),
      minor_radius(minor),
      minor_start_angle(minor_start),
      minor_end_angle(minor_end),
      major_angle(major_angle_in),
      align(align_in) {}

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

Wedge::Wedge(double xs, double ys, double zs, double x0, double z0, double x1, double z1, const Rotation& rotation,
             Align3 align_in)
    : BasePartObject(wedge(xs, ys, zs, x0, z0, x1, z1), rotation, align_in),
      xsize(xs),
      ysize(ys),
      zsize(zs),
      xmin(x0),
      zmin(z0),
      xmax(x1),
      zmax(z1),
      align(align_in) {}

Wedge::Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax, double zmax,
             Align3 align)
    : Wedge(xsize, ysize, zsize, xmin, zmin, xmax, zmax, Rotation(), align) {}

namespace {
/// Hole depths come from the builder when not given (`_through_hole_depth`), so algebra
/// mode needs a depth.
double required_depth(std::optional<double> depth) {
    if (!depth) throw std::invalid_argument("No depth provided");
    return *depth;
}
}  // namespace

Hole::Hole(double radius_in, std::optional<double> depth)
    : radius(radius_in), hole_depth(2 * required_depth(depth)) {
    assign(Solid::make_cylinder(radius_in, hole_depth, down_plane()), Rotation(), Align3(Align::CENTER));
}

CounterBoreHole::CounterBoreHole(double radius_in, double cb_radius, double cb_depth, std::optional<double> depth)
    : radius(radius_in), counter_bore_radius(cb_radius), counter_bore_depth(cb_depth), hole_depth(required_depth(depth)) {
    assign(Shape(fused(Solid::make_cylinder(radius_in, hole_depth, down_plane()),
                       {Solid::make_cylinder(cb_radius, cb_depth + hole_depth, Plane(Vector(0, 0, -cb_depth)))})),
           Rotation(), std::nullopt);
}

CounterSinkHole::CounterSinkHole(double radius_in, double cs_radius, std::optional<double> depth, double cs_angle)
    : radius(radius_in), counter_sink_radius(cs_radius), hole_depth(required_depth(depth)), counter_sink_angle(cs_angle) {
    const double cone_height = cs_radius / std::tan(cs_angle / 2.0 * DEG2RAD);
    assign(Shape(fused(Solid::make_cylinder(radius_in, hole_depth, down_plane()),
                       {Solid::make_cone(cs_radius, 0.0, cone_height, down_plane()),
                        Solid::make_cylinder(cs_radius, hole_depth)})),
           Rotation(), std::nullopt);
}

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
    : BasePartObject(convex_polyhedron(points), rotation,
                     (align.x == Align::NONE && align.y == Align::NONE && align.z == Align::NONE)
                         ? std::nullopt
                         : std::optional<Align3>(align)) {}

}  // namespace build123d
