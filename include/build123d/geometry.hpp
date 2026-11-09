// build123d-cpp: geometry primitives (Vector, Axis, Plane, Location, BoundBox, Color).
// Mirrors build123d/geometry.py.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include <Bnd_Box.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Shape.hxx>

#include "build123d/enums.hpp"

namespace build123d {

inline constexpr double TOLERANCE = 1e-6;
inline constexpr double TOL = 1e-2;
inline constexpr double PI = 3.14159265358979323846;
inline constexpr double DEG2RAD = PI / 180.0;
inline constexpr double RAD2DEG = 180.0 / PI;

class Axis;
class Plane;
class Location;
class BoundBox;
class Shape;
class Vertex;
class Edge;
class Face;
template <class T>
class ShapeList;

// ---------------------------------------------------------------------------
// Vector
// ---------------------------------------------------------------------------

/// A 3D vector / point. Brace-initializable: Vector v = {1, 2} or {1, 2, 3}.
class Vector {
public:
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;

    constexpr Vector() = default;
    constexpr Vector(double x, double y, double z = 0.0) : X(x), Y(y), Z(z) {}
    explicit Vector(const gp_Vec& v) : X(v.X()), Y(v.Y()), Z(v.Z()) {}
    explicit Vector(const gp_Pnt& p) : X(p.X()), Y(p.Y()), Z(p.Z()) {}
    explicit Vector(const gp_Dir& d) : X(d.X()), Y(d.Y()), Z(d.Z()) {}
    explicit Vector(const gp_XYZ& v) : X(v.X()), Y(v.Y()), Z(v.Z()) {}
    /// A Vertex converts to its position, as in build123d where Vertex is VectorLike.
    Vector(const Vertex& v);  // NOLINT(google-explicit-constructor)

    double length() const { return std::sqrt(X * X + Y * Y + Z * Z); }
    double dot(const Vector& o) const { return X * o.X + Y * o.Y + Z * o.Z; }
    Vector cross(const Vector& o) const {
        return {Y * o.Z - Z * o.Y, Z * o.X - X * o.Z, X * o.Y - Y * o.X};
    }
    Vector normalized() const;
    Vector reverse() const { return {-X, -Y, -Z}; }
    Vector add(const Vector& o) const { return *this + o; }
    Vector sub(const Vector& o) const { return *this - o; }
    Vector multiply(double s) const { return *this * s; }
    /// Returns itself; lets Vector be used where a "center" is expected.
    Vector center() const { return *this; }

    /// Unsigned angle in degrees.
    double get_angle(const Vector& other) const;
    /// Signed angle in degrees about `normal` (defaults to -Z like build123d).
    double get_signed_angle(const Vector& other, std::optional<Vector> normal = std::nullopt) const;
    /// Rotate about the direction of `axis` (the axis position is ignored) by degrees.
    Vector rotate(const Axis& axis, double angle) const;
    Vector project_to_line(const Vector& line) const;
    Vector project_to_plane(const Plane& plane) const;
    double distance_to_plane(const Plane& plane) const;
    double signed_distance_from_plane(const Plane& plane) const;
    /// Apply a transformation as a point (default) or as a direction.
    Vector transform(const gp_Trsf& trsf, bool is_direction = false) const;

    /// The other point if it is this point (build123d's Vector.intersect and &);
    /// std::nullopt for Python's None.
    std::optional<Vector> intersect(const Vector& vector) const;
    std::optional<Vector> intersect(const Location& location) const;
    std::optional<Vector> intersect(const Axis& axis) const;
    std::optional<Vector> intersect(const Plane& plane) const;
    /// Shape.intersect(this point) (defined with the topology).
    ShapeList<Shape> intersect(const Shape& shape) const;

    gp_Vec wrapped() const { return gp_Vec(X, Y, Z); }
    gp_Vec to_vec() const { return gp_Vec(X, Y, Z); }
    gp_Pnt to_pnt() const { return gp_Pnt(X, Y, Z); }
    gp_Dir to_dir() const;
    gp_XYZ to_xyz() const { return gp_XYZ(X, Y, Z); }
    std::array<double, 3> to_tuple() const { return {X, Y, Z}; }

    Vector operator+(const Vector& o) const { return {X + o.X, Y + o.Y, Z + o.Z}; }
    Vector operator-(const Vector& o) const { return {X - o.X, Y - o.Y, Z - o.Z}; }
    Vector operator*(double s) const { return {X * s, Y * s, Z * s}; }
    Vector operator/(double s) const { return {X / s, Y / s, Z / s}; }
    Vector operator-() const { return {-X, -Y, -Z}; }
    Vector& operator+=(const Vector& o) { X += o.X; Y += o.Y; Z += o.Z; return *this; }
    Vector& operator-=(const Vector& o) { X -= o.X; Y -= o.Y; Z -= o.Z; return *this; }
    Vector& operator*=(double s) { X *= s; Y *= s; Z *= s; return *this; }
    Vector& operator/=(double s) { X /= s; Y /= s; Z /= s; return *this; }
    friend Vector operator*(double s, const Vector& v) { return v * s; }

    /// Equality after rounding to 5 decimal places, like build123d.
    bool operator==(const Vector& o) const;
    bool operator!=(const Vector& o) const { return !(*this == o); }

    std::string to_string() const;
};

std::ostream& operator<<(std::ostream& os, const Vector& v);

// ---------------------------------------------------------------------------
// Axis
// ---------------------------------------------------------------------------

/// A positioned direction. Axis::X, Axis::Y and Axis::Z are predefined.
class Axis {
public:
    Vector position{0, 0, 0};
    Vector direction{0, 0, 1};

    Axis(const Vector& origin, const Vector& direction);
    explicit Axis(const gp_Ax1& ax1);
    explicit Axis(const Location& location);
    /// Axis from the start point and tangent of an Edge.
    explicit Axis(const Edge& edge);
    static Axis from_end_point(const Vector& origin, const Vector& end_point);

    static const Axis X;
    static const Axis Y;
    static const Axis Z;

    gp_Ax1 wrapped() const;
    Location location() const;
    Axis located(const Location& new_location) const;

    bool is_coaxial(const Axis& other, double angular_tolerance = 1e-5,
                    double linear_tolerance = 1e-5) const;
    bool is_normal(const Axis& other, double angular_tolerance = 1e-5) const;
    bool is_opposite(const Axis& other, double angular_tolerance = 1e-5) const;
    bool is_parallel(const Axis& other, double angular_tolerance = 1e-5) const;
    bool is_skew(const Axis& other, double tolerance = 1e-5) const;
    /// Angle in degrees between the directions.
    double angle_between(const Axis& other) const;
    Axis reverse() const;
    Axis operator-() const { return reverse(); }

    /// Intersection with another axis: this Axis if coaxial, the crossing point, or
    /// std::nullopt (skew or parallel), as build123d's Vector | Axis | None.
    std::optional<std::variant<Vector, Axis>> intersect(const Axis& other) const;
    /// The point itself if it lies on the axis.
    std::optional<Vector> intersect(const Vector& point) const;
    /// The location if it lies on the axis with its z along it, its position if it only
    /// lies on the axis, else std::nullopt.
    std::optional<std::variant<Vector, Location>> intersect(const Location& location) const;
    /// Plane::intersect(Axis): this Axis if it lies in the plane, else the crossing point.
    std::optional<std::variant<Vector, Axis>> intersect(const Plane& plane) const;
    /// Shape.intersect(this axis) (defined with the topology).
    ShapeList<Shape> intersect(const Shape& shape) const;

    bool operator==(const Axis& o) const;
    bool operator!=(const Axis& o) const { return !(*this == o); }
    std::string to_string() const;

private:
    struct raw_tag {};
    constexpr Axis(Vector p, Vector d, raw_tag) : position(p), direction(d) {}
};

std::ostream& operator<<(std::ostream& os, const Axis& a);

// ---------------------------------------------------------------------------
// Location / Rotation / Pos
// ---------------------------------------------------------------------------

/// A rigid transformation (position + orientation).
class Location {
public:
    Location() = default;
    explicit Location(const gp_Trsf& trsf) : trsf_(trsf) {}
    explicit Location(const TopLoc_Location& loc) : trsf_(loc.Transformation()) {}
    explicit Location(const Vector& position);
    /// Position plus Euler angles in degrees.
    Location(const Vector& position, const Vector& rotation, Intrinsic ordering = Intrinsic::XYZ);
    Location(const Vector& position, const Vector& rotation, Extrinsic ordering);
    /// Position plus a rotation of `angle` degrees about `direction` (through the origin).
    Location(const Vector& position, const Vector& direction, double angle);
    /// Position plus a rotation of `angle` degrees about +Z.
    Location(const Vector& position, double angle);
    explicit Location(const Plane& plane);
    Location(const Plane& plane, const Vector& plane_offset);

    Vector position() const;
    /// Intrinsic XYZ Euler angles in degrees.
    Vector orientation() const;
    void set_position(const Vector& value);
    void set_orientation(const Vector& rotation);

    Axis x_axis() const;
    Axis y_axis() const;
    Axis z_axis() const;

    Location inverse() const;
    Location operator*(const Location& other) const;
    Location pow(int exponent) const;
    /// Flip the orientation without changing the position.
    Location operator-() const;
    Location mirror(const Plane& mirror_plane) const;
    Vector center() const { return position(); }

    /// build123d's Location.intersect and &: the point if it is at this position.
    std::optional<Vector> intersect(const Vector& vector) const;
    /// This location if equal, its position if only the positions match, else nullopt.
    std::optional<std::variant<Vector, Location>> intersect(const Location& location) const;
    /// Axis::intersect(this location).
    std::optional<std::variant<Vector, Location>> intersect(const Axis& axis) const;
    /// Plane::intersect(this location).
    std::optional<std::variant<Vector, Location>> intersect(const Plane& plane) const;
    /// Shape.intersect(this location) (defined with the topology).
    ShapeList<Shape> intersect(const Shape& shape) const;

    TopLoc_Location wrapped() const { return TopLoc_Location(trsf_); }
    const gp_Trsf& transformation() const { return trsf_; }

    bool operator==(const Location& o) const;
    bool operator!=(const Location& o) const { return !(*this == o); }
    std::string to_string() const;

protected:
    gp_Trsf trsf_;
};

std::ostream& operator<<(std::ostream& os, const Location& l);

/// A pure rotation, Euler angles in degrees (Intrinsic XYZ by default). Alias: Rot.
class Rotation : public Location {
public:
    Rotation() = default;
    explicit Rotation(double X);
    Rotation(double X, double Y, double Z = 0.0, Intrinsic ordering = Intrinsic::XYZ);
    Rotation(double X, double Y, double Z, Extrinsic ordering);
    Rotation(const Axis& axis, double angle);
};
using Rot = Rotation;

/// A pure translation.
class Pos : public Location {
public:
    Pos() = default;
    explicit Pos(double X) : Location(Vector(X, 0, 0)) {}
    Pos(double X, double Y, double Z = 0.0) : Location(Vector(X, Y, Z)) {}
    template <class V, std::enable_if_t<std::is_same_v<V, Vector> || std::is_same_v<V, Vertex>, int> = 0>
    explicit Pos(const V& v) : Location(Vector(v)) {}
};

// ---------------------------------------------------------------------------
// Plane
// ---------------------------------------------------------------------------

/// A coordinate system: origin plus orthonormal right-handed x/y/z directions.
class Plane {
public:
    Vector origin{0, 0, 0};
    Vector x_dir{1, 0, 0};
    Vector y_dir{0, 1, 0};
    Vector z_dir{0, 0, 1};

    /// Plane(origin, x_dir = None, z_dir = (0, 0, 1)), as in build123d.
    explicit Plane(const Vector& origin = Vector(0, 0, 0),
                   std::optional<Vector> x_dir = std::nullopt,
                   const Vector& z_dir = Vector(0, 0, 1));
    explicit Plane(const gp_Pln& pln);
    explicit Plane(const gp_Ax3& ax3);
    explicit Plane(const Location& location);
    /// Plane through the center of a planar face, normal to it.
    explicit Plane(const Face& face, std::optional<Vector> x_dir = std::nullopt);
    /// Plane with z_dir along the axis.
    explicit Plane(const Axis& axis, std::optional<Vector> x_dir = std::nullopt);
    /// Plane(origin, x_dir, y_dir=...) form.
    static Plane from_x_y(const Vector& origin, const Vector& x_dir, const Vector& y_dir);
    /// Plane through three points.
    static Plane from_points(const Vector& p0, const Vector& p1, const Vector& p2);

    static const Plane XY;
    static const Plane YZ;
    static const Plane ZX;
    static const Plane XZ;
    static const Plane YX;
    static const Plane ZY;
    static const Plane front;
    static const Plane back;
    static const Plane left;
    static const Plane right;
    static const Plane top;
    static const Plane bottom;
    static const Plane isometric;

    /// Move along z_dir.
    Plane offset(double amount) const;
    Plane reverse() const;
    Plane operator-() const { return reverse(); }
    /// Rotate about the plane's own axes (Euler degrees); origin unchanged.
    Plane rotated(const Vector& rotation, Intrinsic ordering = Intrinsic::XYZ) const;
    Plane rotated(const Vector& rotation, Extrinsic ordering) const;
    /// Relocate by a location expressed in this plane's local coordinates.
    Plane moved(const Location& loc) const;
    Plane& move(const Location& loc);
    Plane shift_origin(const Vector& locator) const;
    Plane shift_origin(const Axis& locator) const;
    void set_x_dir(const Vector& direction);

    /// Location mapping plane-local coordinates to global.
    Location location() const { return Location(*this); }
    gp_Pln wrapped() const;
    gp_Ax3 to_gp_ax3() const;
    gp_Ax2 to_gp_ax2() const;
    /// Global -> local.
    gp_Trsf forward_transform() const;
    /// Local -> global.
    gp_Trsf reverse_transform() const;
    Vector to_local_coords(const Vector& v) const;
    Vector from_local_coords(const Vector& v) const;
    BoundBox to_local_coords(const BoundBox& box) const;
    /// Reposition a shape into this plane's local coordinates (defined in topology.hpp).
    template <class S>
    S to_local_coords(const S& shape) const;
    template <class S>
    S from_local_coords(const S& shape) const;
    Location location_between(const Plane& other) const;

    bool contains(const Vector& point, double tolerance = TOLERANCE) const;
    bool contains(const Axis& axis, double tolerance = TOLERANCE) const;
    /// The axis if it lies in the plane, the crossing point, or std::nullopt (parallel).
    std::optional<std::variant<Vector, Axis>> intersect(const Axis& axis) const;
    /// This plane if coplanar (same z_dir), the line of intersection, or std::nullopt.
    std::optional<std::variant<Axis, Plane>> intersect(const Plane& other) const;
    /// The point if it lies in the plane.
    std::optional<Vector> intersect(const Vector& point) const;
    /// The location if it lies in the plane with the plane's z_dir, its position if it
    /// only lies in the plane, else std::nullopt.
    std::optional<std::variant<Vector, Location>> intersect(const Location& location) const;
    /// Shape.intersect(this plane) (defined with the topology).
    ShapeList<Shape> intersect(const Shape& shape) const;

    bool operator==(const Plane& o) const;
    bool operator!=(const Plane& o) const { return !(*this == o); }
    std::string to_string() const;

private:
    struct raw_tag {};
    Plane(const Vector& o, const Vector& x, const Vector& z, raw_tag) { init(o, x, z); }
    void init(const Vector& origin, const Vector& x_dir, const Vector& z_dir);
    void init(const gp_Ax3& ax3);

    // The coordinate system the plane was built with. Like build123d's stored gp_Pln it is
    // used as is (bit for bit) unless the public fields have been changed since.
    gp_Ax3 ax3_;
};

std::ostream& operator<<(std::ostream& os, const Plane& p);

/// Plane * Location -> Location in the plane's coordinate system.
Location operator*(const Plane& plane, const Location& loc);
/// Plane * Plane -> Location (as build123d).
Location operator*(const Plane& plane, const Plane& other);
/// Location * Plane -> the plane moved in its local coordinates.
Plane operator*(const Location& loc, const Plane& plane);
std::vector<Location> operator*(const Plane& plane, const std::vector<Location>& locs);
std::vector<Location> operator*(const Location& loc, const std::vector<Location>& locs);

// ---- build123d's & between geometry objects (the same as intersect)
inline std::optional<Vector> operator&(const Vector& a, const Vector& b) { return a.intersect(b); }
inline std::optional<Vector> operator&(const Vector& a, const Location& b) { return a.intersect(b); }
inline std::optional<Vector> operator&(const Vector& a, const Axis& b) { return a.intersect(b); }
inline std::optional<Vector> operator&(const Vector& a, const Plane& b) { return a.intersect(b); }
inline std::optional<std::variant<Vector, Axis>> operator&(const Axis& a, const Axis& b) { return a.intersect(b); }
inline std::optional<Vector> operator&(const Axis& a, const Vector& b) { return a.intersect(b); }
inline std::optional<std::variant<Vector, Location>> operator&(const Axis& a, const Location& b) {
    return a.intersect(b);
}
inline std::optional<std::variant<Vector, Axis>> operator&(const Axis& a, const Plane& b) { return a.intersect(b); }
inline std::optional<Vector> operator&(const Location& a, const Vector& b) { return a.intersect(b); }
inline std::optional<std::variant<Vector, Location>> operator&(const Location& a, const Location& b) {
    return a.intersect(b);
}
inline std::optional<std::variant<Vector, Location>> operator&(const Location& a, const Axis& b) {
    return a.intersect(b);
}
inline std::optional<std::variant<Vector, Location>> operator&(const Location& a, const Plane& b) {
    return a.intersect(b);
}
inline std::optional<std::variant<Vector, Axis>> operator&(const Plane& a, const Axis& b) { return a.intersect(b); }
inline std::optional<std::variant<Axis, Plane>> operator&(const Plane& a, const Plane& b) { return a.intersect(b); }
inline std::optional<Vector> operator&(const Plane& a, const Vector& b) { return a.intersect(b); }
inline std::optional<std::variant<Vector, Location>> operator&(const Plane& a, const Location& b) {
    return a.intersect(b);
}

// ---------------------------------------------------------------------------
// BoundBox
// ---------------------------------------------------------------------------

/// Axis-aligned bounding box.
class BoundBox {
public:
    Vector min{0, 0, 0};
    Vector max{0, 0, 0};
    Vector size{0, 0, 0};

    BoundBox() = default;
    explicit BoundBox(const Bnd_Box& box);
    /// Bounding box of a TopoDS_Shape; `optimal` gives an exact (slower) box.
    static BoundBox from_topo_ds(const TopoDS_Shape& shape,
                                 std::optional<double> tolerance = std::nullopt,
                                 bool optimal = true);

    bool is_void() const { return void_; }
    double diagonal() const;
    /// Length, area or volume depending on the number of non-degenerate axes.
    double measure() const;
    Vector center() const { return (min + max) / 2.0; }
    BoundBox add(const Vector& point, std::optional<double> tol = std::nullopt) const;
    BoundBox add(const BoundBox& other, std::optional<double> tol = std::nullopt) const;

    bool covers(const BoundBox& other, double tolerance = TOLERANCE) const;
    bool covered_by(const BoundBox& other, double tolerance = TOLERANCE) const;
    bool contains(const BoundBox& other, double tolerance = TOLERANCE) const;
    bool within(const BoundBox& other, double tolerance = TOLERANCE) const;
    bool intersects(const BoundBox& other, double tolerance = TOLERANCE) const;
    bool disjoint(const BoundBox& other, double tolerance = TOLERANCE) const;

    /// Offset needed to give the box the requested alignment.
    Vector to_align_offset(const Align3& align) const;
    Vector to_align_offset(const Align2& align) const;

    const Bnd_Box& wrapped() const { return box_; }
    std::string to_string() const;

private:
    Bnd_Box box_;
    bool void_ = true;
};

std::ostream& operator<<(std::ostream& os, const BoundBox& b);

/// Amount to move an object with the given extents to achieve the alignment.
Vector to_align_offset(const Vector& min_point, const Vector& max_point, const Align3& align,
                       std::optional<Vector> center = std::nullopt);

// ---------------------------------------------------------------------------
// Color
// ---------------------------------------------------------------------------

/// RGBA color in the range [0, 1].
class Color {
public:
    double red = 1.0;
    double green = 1.0;
    double blue = 0.0;
    double alpha = 1.0;

    Color() = default;
    Color(double r, double g, double b, double a = 1.0) : red(r), green(g), blue(b), alpha(a) {}
    /// A CSS/OCCT color name ("red", "SteelBlue", ...) or hex "#RRGGBB[AA]".
    explicit Color(const std::string& name, double alpha = 1.0);
    explicit Color(const char* name, double alpha = 1.0) : Color(std::string(name), alpha) {}
    /// 0xRRGGBB
    static Color from_hex(std::uint32_t rgb, double alpha = 1.0);

    Quantity_ColorRGBA wrapped() const;
    std::array<double, 4> to_tuple() const { return {red, green, blue, alpha}; }
    std::string to_string() const;
};

std::ostream& operator<<(std::ostream& os, const Color& c);

}  // namespace build123d
