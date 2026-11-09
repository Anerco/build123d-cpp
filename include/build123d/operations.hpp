// build123d-cpp: algebra-mode operations. Mirrors operations_generic.py,
// operations_part.py and operations_sketch.py. Optional keyword arguments of the
// Python API are grouped in option structs, e.g. extrude(sk, 10, {.both = true}).
#pragma once

#include <optional>
#include <type_traits>
#include <variant>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

// ---------------------------------------------------------------------------
// Part operations
// ---------------------------------------------------------------------------

struct ExtrudeOptions {
    /// Extrusion direction (normalized); defaults to each face's normal.
    std::optional<Vector> dir;
    /// Extrude in both directions.
    bool both = false;
    /// Taper angle in degrees.
    double taper = 0.0;
    bool clean = true;
};

/// Extrude the faces of a Face/Sketch by `amount` (sign controls direction).
Part extrude(const Shape& to_extrude, double amount, const ExtrudeOptions& options = {});
Part extrude(const std::vector<Face>& to_extrude, double amount,
             const ExtrudeOptions& options = {});
/// Extrude until a surface of `target` (Until::NEXT, LAST, PREVIOUS or FIRST).
Part extrude(const Shape& to_extrude, Until until, const Shape& target,
             const ExtrudeOptions& options = {});
Part extrude(const std::vector<Face>& to_extrude, Until until, const Shape& target,
             const ExtrudeOptions& options = {});

/// Revolve the faces of a Face/Sketch about an axis by `revolution_arc` degrees.
Part revolve(const Shape& profiles, const Axis& axis = Axis::Z, double revolution_arc = 360.0,
             bool clean = true);
Part revolve(const std::vector<Face>& profiles, const Axis& axis = Axis::Z,
             double revolution_arc = 360.0, bool clean = true);
/// Revolve the faces of several Faces/Sketches.
Part revolve(const std::vector<Shape>& profiles, const Axis& axis = Axis::Z,
             double revolution_arc = 360.0, bool clean = true);
template <class S, std::enable_if_t<std::is_base_of_v<Shape, S> && !std::is_same_v<S, Shape> &&
                                        !std::is_same_v<S, Face>,
                                    int> = 0>
Part revolve(const std::vector<S>& profiles, const Axis& axis = Axis::Z, double revolution_arc = 360.0,
             bool clean = true) {
    return revolve(std::vector<Shape>(profiles.begin(), profiles.end()), axis, revolution_arc, clean);
}

/// Loft through Faces/Sketches (holes are lofted too); a Vertex may start or end.
Part loft(const std::vector<Shape>& sections, bool ruled = false, bool clean = true);
/// Loft through the elements of one Sketch/Compound, in order.
Part loft(const Shape& sections, bool ruled = false, bool clean = true);
template <class S, std::enable_if_t<std::is_base_of_v<Shape, S> && !std::is_same_v<S, Shape>, int> = 0>
Part loft(const std::vector<S>& sections, bool ruled = false, bool clean = true) {
    return loft(std::vector<Shape>(sections.begin(), sections.end()), ruled, clean);
}

/// Cross section(s) of a part by plane(s), offset by `height` along the plane normal.
Sketch section(const Shape& obj, const Plane& section_by = Plane::XZ, double height = 0.0,
               bool clean = true);
Sketch section(const Shape& obj, const std::vector<Plane>& section_by, double height = 0.0,
               bool clean = true);

/// Thicken (possibly non-planar) faces along their normals.
Part thicken(const Shape& to_thicken, double amount,
             std::optional<Vector> normal_override = std::nullopt, bool both = false,
             bool clean = true);
Part thicken(const std::vector<Face>& to_thicken, double amount,
             std::optional<Vector> normal_override = std::nullopt, bool both = false,
             bool clean = true);

/// Draft angle (degrees) applied to faces of a single solid.
Part draft(const std::vector<Face>& faces, const Plane& neutral_plane, double angle);
Part draft(const Face& faces, const Plane& neutral_plane, double angle);

/// Workplane at `distance` along projection_dir with a projected x direction.
Plane project_workplane(const Vector& origin, const Vector& x_dir, const Vector& projection_dir,
                        double distance);

/// Sheet metal part bent along `line` (an Edge, Wire or Curve): `thickness` thick, offset to
/// `side`, as wide as `station_widths` (one width, or one per vertex of the line).
Part make_brake_formed(double thickness, double station_widths, const Mixin1D& line,
                       Side side = Side::LEFT, Kind kind = Kind::ARC, bool clean = true);
Part make_brake_formed(double thickness, const std::vector<double>& station_widths, const Mixin1D& line,
                       Side side = Side::LEFT, Kind kind = Kind::ARC, bool clean = true);
Part make_brake_formed(double thickness, double station_widths, const Curve& line,
                       Side side = Side::LEFT, Kind kind = Kind::ARC, bool clean = true);
Part make_brake_formed(double thickness, const std::vector<double>& station_widths, const Curve& line,
                       Side side = Side::LEFT, Kind kind = Kind::ARC, bool clean = true);

// ---------------------------------------------------------------------------
// Sketch operations
// ---------------------------------------------------------------------------

/// Face from edges that combine into a single closed wire.
Sketch make_face(const Shape& edges);
Sketch make_face(const std::vector<Edge>& edges);
/// Face from the edges of several Edges, Wires and Curves.
Sketch make_face(const std::vector<Shape>& edges);
template <class S, std::enable_if_t<std::is_base_of_v<Shape, S> && !std::is_same_v<S, Shape> &&
                                        !std::is_same_v<S, Edge>,
                                    int> = 0>
Sketch make_face(const std::vector<S>& edges) {
    return make_face(std::vector<Shape>(edges.begin(), edges.end()));
}
/// Face from the convex hull of the edges.
Sketch make_hull(const Shape& edges);
Sketch make_hull(const std::vector<Edge>& edges);
/// Replace an edge of a face (taken from a Sketch/Face, e.g. sketch.edges()[0]) by the arc of
/// the largest circle that fits against it and its two neighbouring edges ("rounds off" the
/// end); `invert` makes the arc concave.
Sketch full_round(const Edge& edge, bool invert = false, int voronoi_point_count = 100);
/// Faces made by sweeping a line of `line_width` along the curves.
Sketch trace(const Shape& lines, double line_width = 1.0);
Sketch trace(const std::vector<Edge>& lines, double line_width = 1.0);
/// Trace several Edges, Wires and Curves.
Sketch trace(const std::vector<Shape>& lines, double line_width = 1.0);
template <class S, std::enable_if_t<std::is_base_of_v<Shape, S> && !std::is_same_v<S, Shape> &&
                                        !std::is_same_v<S, Edge>,
                                    int> = 0>
Sketch trace(const std::vector<S>& lines, double line_width = 1.0) {
    return trace(std::vector<Shape>(lines.begin(), lines.end()), line_width);
}

// ---------------------------------------------------------------------------
// Generic operations
// ---------------------------------------------------------------------------

/// 3D fillet of edges extracted from a part (part.edges()...).
Part fillet(const std::vector<Edge>& edges, double radius);
Part fillet(const Edge& edge, double radius);
/// 2D/1D fillet of vertices extracted from a sketch or curve.
/// Returns a Compound that converts to Sketch or Curve.
Compound fillet(const std::vector<Vertex>& vertices, double radius);
Compound fillet(const Vertex& vertex, double radius);

/// 3D chamfer of edges. `angle` (degrees) or `length2` make it asymmetric;
/// `reference` selects the face where `length` is measured.
Part chamfer(const std::vector<Edge>& edges, double length,
             std::optional<double> length2 = std::nullopt,
             std::optional<double> angle = std::nullopt,
             const std::optional<Face>& reference = std::nullopt);
Part chamfer(const Edge& edge, double length, std::optional<double> length2 = std::nullopt,
             std::optional<double> angle = std::nullopt,
             const std::optional<Face>& reference = std::nullopt);
/// 2D/1D chamfer of vertices; returns a Compound that converts to Sketch or Curve.
Compound chamfer(const std::vector<Vertex>& vertices, double length,
                 std::optional<double> length2 = std::nullopt,
                 std::optional<double> angle = std::nullopt,
                 const std::optional<Edge>& reference = std::nullopt);
Compound chamfer(const Vertex& vertex, double length, std::optional<double> length2 = std::nullopt,
                 std::optional<double> angle = std::nullopt,
                 const std::optional<Edge>& reference = std::nullopt);

struct OffsetOptions {
    /// Faces of a solid to leave open (hollowing).
    std::vector<Face> openings;
    Kind kind = Kind::ARC;
    /// For 1D objects: which side to offset.
    Side side = Side::BOTH;
    /// For 1D objects with side != BOTH: close the offset.
    bool closed = true;
    /// Repair degenerate edges shorter than this.
    std::optional<double> min_edge_length;
};

namespace detail {
TopoDS_Shape mirror_impl(const std::vector<const Shape*>& objs, const Plane& about);
TopoDS_Shape scale_impl(const std::vector<const Shape*>& objs, double by,
                        const std::optional<Vector>& about);
TopoDS_Shape scale_impl(const std::vector<const Shape*>& objs, const Vector& by,
                        const std::optional<Vector>& about);
TopoDS_Shape split_impl(const std::vector<const Shape*>& objs, const Plane& bisect_by, Keep keep);
TopoDS_Shape split_impl(const std::vector<const Shape*>& objs, const Shape& bisect_by, Keep keep);
TopoDS_Shape offset_impl(const std::vector<const Shape*>& objs, double amount,
                         const OffsetOptions& options);
TopoDS_Shape bounding_box_impl(const std::vector<const Shape*>& objs);

template <class T>
std::vector<const Shape*> one(const T& obj) {
    return {&obj};
}
template <class R>
R wrap_result(const TopoDS_Shape& shape) {
    return R(shape);
}
}  // namespace detail

/// Mirror copies of objects about a plane (the originals are not included).
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> mirror(const T& obj, const Plane& about = Plane::XZ) {
    return detail::wrap_result<algebra_t<T>>(detail::mirror_impl(detail::one(obj), about));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> mirror(const std::vector<T>& objs, const Plane& about = Plane::XZ) {
    return detail::wrap_result<algebra_t<T>>(detail::mirror_impl(detail::shape_ptrs(objs), about));
}

/// Scale objects (uniformly or per axis) about `about`, or each object's location.
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> scale(const T& obj, double by = 1, std::optional<Vector> about = std::nullopt) {
    return detail::wrap_result<algebra_t<T>>(detail::scale_impl(detail::one(obj), by, about));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> scale(const T& obj, const Vector& by, std::optional<Vector> about = std::nullopt) {
    return detail::wrap_result<algebra_t<T>>(detail::scale_impl(detail::one(obj), by, about));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> scale(const std::vector<T>& objs, double by = 1,
                   std::optional<Vector> about = std::nullopt) {
    return detail::wrap_result<algebra_t<T>>(detail::scale_impl(detail::shape_ptrs(objs), by, about));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> scale(const std::vector<T>& objs, const Vector& by,
                   std::optional<Vector> about = std::nullopt) {
    return detail::wrap_result<algebra_t<T>>(detail::scale_impl(detail::shape_ptrs(objs), by, about));
}

/// Split objects by a plane and keep the TOP, BOTTOM or BOTH sides.
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> split(const T& obj, const Plane& bisect_by = Plane::XZ, Keep keep = Keep::TOP) {
    return detail::wrap_result<algebra_t<T>>(detail::split_impl(detail::one(obj), bisect_by, keep));
}
/// Split objects by a face or shell.
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> split(const T& obj, const Shape& bisect_by, Keep keep = Keep::TOP) {
    return detail::wrap_result<algebra_t<T>>(detail::split_impl(detail::one(obj), bisect_by, keep));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> split(const std::vector<T>& objs, const Plane& bisect_by = Plane::XZ,
                   Keep keep = Keep::TOP) {
    return detail::wrap_result<algebra_t<T>>(
        detail::split_impl(detail::shape_ptrs(objs), bisect_by, keep));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> split(const std::vector<T>& objs, const Shape& bisect_by, Keep keep = Keep::TOP) {
    return detail::wrap_result<algebra_t<T>>(
        detail::split_impl(detail::shape_ptrs(objs), bisect_by, keep));
}

/// Offset edges/wires (2D), faces (2D) or solids (3D, hollowing with openings).
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> offset(const T& obj, double amount = 0, const OffsetOptions& options = {}) {
    return detail::wrap_result<algebra_t<T>>(detail::offset_impl(detail::one(obj), amount, options));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
algebra_t<T> offset(const std::vector<T>& objs, double amount = 0, const OffsetOptions& options = {}) {
    return detail::wrap_result<algebra_t<T>>(
        detail::offset_impl(detail::shape_ptrs(objs), amount, options));
}

template <class T>
using bounding_box_t = std::conditional_t<std::decay_t<T>::kDim == 2, Sketch, Part>;

/// Bounding rectangle (2D objects) or box (otherwise) of objects.
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
bounding_box_t<T> bounding_box(const T& obj) {
    return detail::wrap_result<bounding_box_t<T>>(detail::bounding_box_impl(detail::one(obj)));
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
bounding_box_t<T> bounding_box(const std::vector<T>& objs) {
    return detail::wrap_result<bounding_box_t<T>>(
        detail::bounding_box_impl(detail::shape_ptrs(objs)));
}

struct SweepOptions {
    /// Sweep through several sections (their outer wires) along one path.
    bool multisection = false;
    bool is_frenet = false;
    Transition transition = Transition::TRANSFORMED;
    /// Fixed binormal direction.
    std::optional<Vector> normal;
    /// Auxiliary curve guiding the profile rotation.
    std::optional<Wire> binormal;
    bool clean = true;
};

namespace detail {
TopoDS_Shape sweep_impl(const std::vector<const Shape*>& sections, const Shape& path,
                        const SweepOptions& options);
/// One sections argument: a Compound's children are the sections, as in build123d.
TopoDS_Shape sweep_one_impl(const Shape& sections, const Shape& path, const SweepOptions& options);
/// The wire of a path given as edges ("Unable to build path from edges").
Wire sweep_path_wire(const std::vector<Edge>& path);
}  // namespace detail

// ---------------------------------------------------------------------------
// project
// ---------------------------------------------------------------------------

namespace detail {
struct ProjectOutput {
    std::vector<TopoDS_Shape> shapes;  ///< projected faces
    std::vector<Vector> points;        ///< projected points, in workplane coordinates
};
ProjectOutput project_impl(const std::vector<const Shape*>& objects, const std::vector<Vector>& points,
                           const Plane& workplane);
template <class T>
using project_t = std::conditional_t<std::decay_t<T>::kDim == 2, Sketch,
                                     std::conditional_t<std::decay_t<T>::kDim == 1, Curve, Compound>>;
template <class T>
inline constexpr bool is_projectable_v =
    is_shape_v<T> && !std::is_same_v<std::decay_t<T>, Shape> && !std::is_same_v<std::decay_t<T>, Vertex>;
}  // namespace detail

/// Project faces onto `workplane` along its normal (algebra-mode project()). As in build123d
/// without a builder, edges and wires give nothing, other shapes are ignored and `target` is
/// not used.
template <class T, std::enable_if_t<detail::is_projectable_v<T>, int> = 0>
detail::project_t<T> project(const T& objects, const Plane& workplane,
                             const std::optional<Shape>& target = std::nullopt) {
    return detail::project_t<T>(
        detail::make_compound_topods(detail::project_impl(detail::one(objects), {}, workplane).shapes));
}
template <class T, std::enable_if_t<detail::is_projectable_v<T>, int> = 0>
detail::project_t<T> project(const std::vector<T>& objects, const Plane& workplane,
                             const std::optional<Shape>& target = std::nullopt) {
    return detail::project_t<T>(
        detail::make_compound_topods(detail::project_impl(detail::shape_ptrs(objects), {}, workplane).shapes));
}
/// Project points onto `workplane`: the points in the workplane's local coordinates.
std::vector<Vector> project(const Vector& point, const Plane& workplane,
                            const std::optional<Shape>& target = std::nullopt);
std::vector<Vector> project(const std::vector<Vector>& points, const Plane& workplane,
                            const std::optional<Shape>& target = std::nullopt);
std::vector<Vector> project(const Vertex& point, const Plane& workplane,
                            const std::optional<Shape>& target = std::nullopt);
std::vector<Vector> project(const std::vector<Vertex>& points, const Plane& workplane,
                            const std::optional<Shape>& target = std::nullopt);
/// Project a mix of shapes: projected points when there are vertices, else the projected
/// faces (Python's Curve | Sketch | Compound | ShapeList[Vector]).
std::variant<Compound, std::vector<Vector>> project(const std::vector<Shape>& objects, const Plane& workplane,
                                                    const std::optional<Shape>& target = std::nullopt);
std::variant<Compound, std::vector<Vector>> project(const Shape& object, const Plane& workplane,
                                                    const std::optional<Shape>& target = std::nullopt);

template <class S>
using sweep_t = std::conditional_t<
    std::decay_t<S>::kDim == 1, Sketch,
    std::conditional_t<(std::decay_t<S>::kDim < 0), Compound, Part>>;

/// Sweep faces (-> Part) or curves (-> Sketch) along a path (Edge, Wire or Curve).
template <class S, std::enable_if_t<detail::is_shape_v<S>, int> = 0>
sweep_t<S> sweep(const S& sections, const Shape& path, const SweepOptions& options = {}) {
    return detail::wrap_result<sweep_t<S>>(detail::sweep_one_impl(sections, path, options));
}
template <class S, std::enable_if_t<detail::is_shape_v<S>, int> = 0>
sweep_t<S> sweep(const std::vector<S>& sections, const Shape& path,
                 const SweepOptions& options = {}) {
    return detail::wrap_result<sweep_t<S>>(
        detail::sweep_impl(detail::shape_ptrs(sections), path, options));
}
/// Sweep along a path given as edges.
template <class S, std::enable_if_t<detail::is_shape_v<S>, int> = 0>
sweep_t<S> sweep(const S& sections, const std::vector<Edge>& path, const SweepOptions& options = {}) {
    return sweep(sections, detail::sweep_path_wire(path), options);
}
template <class S, std::enable_if_t<detail::is_shape_v<S>, int> = 0>
sweep_t<S> sweep(const std::vector<S>& sections, const std::vector<Edge>& path,
                 const SweepOptions& options = {}) {
    return sweep(sections, detail::sweep_path_wire(path), options);
}

}  // namespace build123d
