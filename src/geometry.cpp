#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <cstdio>
#include <sstream>
#include <unordered_map>

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom_ElementarySurface.hxx>
#include <TopoDS_Face.hxx>
#include <GeomAPI_IntCS.hxx>
#include <GeomAPI_IntSS.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <gp.hxx>
#include <gp_Lin.hxx>
#include <gp_Quaternion.hxx>
#include <Quantity_Color.hxx>

#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d {

using detail::py_round;

namespace {

std::string fmt(double v) {
    if (std::abs(v) < TOLERANCE) v = 0.0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

std::array<double, 3> rounded_key(const Vector& v) {
    return {py_round(v.X, 5), py_round(v.Y, 5), py_round(v.Z, 5)};
}

std::array<double, 4> quaternion_key(const gp_Quaternion& q) {
    std::array<double, 4> c = {q.X(), q.Y(), q.Z(), q.W()};
    for (double value : c) {
        if (std::abs(value) > TOLERANCE) {
            if (value < 0) {
                for (double& x : c) x = -x;
            }
            break;
        }
    }
    for (double& x : c) x = py_round(x, 5);
    return c;
}

std::size_t hash_combine(std::size_t seed, double value) {
    if (value == 0.0) value = 0.0;  // -0.0 == 0.0, so they must hash alike
    return seed ^ (std::hash<double>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

template <class C>
std::size_t hash_values(std::size_t seed, const C& values) {
    for (double v : values) seed = hash_combine(seed, v);
    return seed;
}

Vector round14(const Vector& v) {
    return {py_round(v.X, 14), py_round(v.Y, 14), py_round(v.Z, 14)};
}

}  // namespace

// ---------------------------------------------------------------------------
// Vector
// ---------------------------------------------------------------------------

Vector Vector::normalized() const {
    const double len = length();
    if (len <= gp::Resolution()) throw std::invalid_argument("Cannot normalize a null Vector");
    return {X / len, Y / len, Z / len};
}

double Vector::reciprocal(double s) {
    if (s == 0.0) throw std::domain_error("float division by zero");
    return 1.0 / s;
}

gp_Dir Vector::to_dir() const {
    if (length() <= gp::Resolution()) throw std::invalid_argument("Cannot make a direction from a null Vector");
    return gp_Dir(X, Y, Z);
}

double Vector::get_angle(const Vector& other) const {
    return detail::occt("Vector.get_angle", [&] { return to_vec().Angle(other.to_vec()) * RAD2DEG; });
}

double Vector::get_signed_angle(const Vector& other, std::optional<Vector> normal) const {
    const gp_Vec ref = normal ? normal->to_vec() : gp_Vec(0, 0, -1);
    return detail::occt("Vector.get_signed_angle",
                        [&] { return to_vec().AngleWithRef(other.to_vec(), ref) * RAD2DEG; });
}

Vector Vector::rotate(const Axis& axis, double angle) const {
    return Vector(to_vec().Rotated(axis.wrapped(), angle * DEG2RAD));
}

Vector Vector::project_to_line(const Vector& line) const {
    const double len = line.length();
    return line * (dot(line) / (len * len));
}

Vector Vector::project_to_plane(const Plane& plane) const {
    const Vector& base = plane.origin;
    const Vector& normal = plane.z_dir;
    const double len = normal.length();
    return *this - normal * ((*this - base).dot(normal) / (len * len));
}

double Vector::distance_to_plane(const Plane& plane) const {
    return plane.wrapped().Distance(to_pnt());
}

double Vector::signed_distance_from_plane(const Plane& plane) const {
    return (*this - plane.origin).dot(plane.z_dir);
}

Vector Vector::transform(const gp_Trsf& trsf, bool is_direction) const {
    if (!is_direction) return Vector(to_pnt().Transformed(trsf));
    return Vector(to_dir().Transformed(trsf));
}

bool Vector::operator==(const Vector& o) const { return rounded_key(*this) == rounded_key(o); }

std::size_t Vector::hash() const { return hash_values(0, rounded_key(*this)); }

std::string Vector::to_string() const {
    return "Vector(" + fmt(X) + ", " + fmt(Y) + ", " + fmt(Z) + ")";
}

std::ostream& operator<<(std::ostream& os, const Vector& v) { return os << v.to_string(); }

// ---------------------------------------------------------------------------
// Axis
// ---------------------------------------------------------------------------

const Axis Axis::X{Vector(0, 0, 0), Vector(1, 0, 0), Axis::raw_tag{}};
const Axis Axis::Y{Vector(0, 0, 0), Vector(0, 1, 0), Axis::raw_tag{}};
const Axis Axis::Z{Vector(0, 0, 0), Vector(0, 0, 1), Axis::raw_tag{}};

Axis::Axis(const Vector& origin, const Vector& dir) : position(origin) {
    if (dir.length() <= gp::Resolution()) throw std::invalid_argument("Invalid Axis parameters: null direction");
    direction = Vector(gp_Dir(dir.normalized().to_xyz()));
}

void Axis::set_direction(const Vector& new_direction) { direction = Vector(new_direction.to_dir()); }

Axis::Axis(const gp_Ax1& ax1)
    : position(Vector(ax1.Location())), direction(Vector(ax1.Direction())) {}

Axis::Axis(const Location& location) {
    const gp_Ax1 z(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    *this = Axis(z.Transformed(location.transformation()));
}

Axis Axis::from_end_point(const Vector& origin, const Vector& end_point) {
    return Axis(origin, end_point - origin);
}

gp_Ax1 Axis::wrapped() const { return gp_Ax1(position.to_pnt(), direction.to_dir()); }

Location Axis::location() const { return Location(Plane(position, std::nullopt, direction)); }

Axis Axis::located(const Location& new_location) const {
    return Axis(wrapped().Transformed(new_location.transformation()));
}

bool Axis::is_coaxial(const Axis& other, double angular_tolerance, double linear_tolerance) const {
    return wrapped().IsCoaxial(other.wrapped(), angular_tolerance * (PI / 180.0), linear_tolerance);
}
bool Axis::is_normal(const Axis& other, double angular_tolerance) const {
    return wrapped().IsNormal(other.wrapped(), angular_tolerance * (PI / 180.0));
}
bool Axis::is_opposite(const Axis& other, double angular_tolerance) const {
    return wrapped().IsOpposite(other.wrapped(), angular_tolerance * (PI / 180.0));
}
bool Axis::is_parallel(const Axis& other, double angular_tolerance) const {
    return wrapped().IsParallel(other.wrapped(), angular_tolerance * (PI / 180.0));
}

bool Axis::is_skew(const Axis& other, double tolerance) const {
    if (is_parallel(other, tolerance)) {
        const Vector parallel_offset = (position - other.position).cross(direction);
        return parallel_offset.length() > tolerance;
    }
    const double coplanarity = (position - other.position).dot(direction.cross(other.direction));
    return std::abs(coplanarity) > tolerance;
}

double Axis::angle_between(const Axis& other) const {
    return wrapped().Angle(other.wrapped()) * RAD2DEG;
}

Axis Axis::reverse() const { return Axis(wrapped().Reversed()); }

namespace {

/// numpy.linalg.lstsq(A, b, rcond=None) for a 3x3 system: the minimum-norm least-squares
/// solution, through a (one-sided Jacobi) singular value decomposition with singular
/// values below eps * 3 * max(singular value) treated as zero.
std::array<double, 3> lstsq3(const std::array<std::array<double, 3>, 3>& a, const std::array<double, 3>& b) {
    // columns of A, and V = identity
    double u[3][3], v[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            u[j][i] = a[i][j];  // u[column][row]
            v[j][i] = i == j ? 1.0 : 0.0;
        }
    }
    const double eps = std::numeric_limits<double>::epsilon();
    for (int sweep = 0; sweep < 60; ++sweep) {
        bool rotated = false;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                double alpha = 0, beta = 0, gamma = 0;
                for (int k = 0; k < 3; ++k) {
                    alpha += u[p][k] * u[p][k];
                    beta += u[q][k] * u[q][k];
                    gamma += u[p][k] * u[q][k];
                }
                if (gamma == 0.0 || std::abs(gamma) <= eps * std::sqrt(alpha * beta)) continue;
                rotated = true;
                const double zeta = (beta - alpha) / (2 * gamma);
                const double t = (zeta >= 0 ? 1.0 : -1.0) / (std::abs(zeta) + std::sqrt(1 + zeta * zeta));
                const double c = 1 / std::sqrt(1 + t * t), s = c * t;
                for (int k = 0; k < 3; ++k) {
                    const double up = u[p][k], uq = u[q][k];
                    u[p][k] = c * up - s * uq;
                    u[q][k] = s * up + c * uq;
                    const double vp = v[p][k], vq = v[q][k];
                    v[p][k] = c * vp - s * vq;
                    v[q][k] = s * vp + c * vq;
                }
            }
        }
        if (!rotated) break;
    }
    double sigma[3], sigma_max = 0;
    for (int j = 0; j < 3; ++j) {
        sigma[j] = std::sqrt(u[j][0] * u[j][0] + u[j][1] * u[j][1] + u[j][2] * u[j][2]);
        sigma_max = std::max(sigma_max, sigma[j]);
    }
    const double cutoff = eps * 3 * sigma_max;
    std::array<double, 3> x{0, 0, 0};
    for (int j = 0; j < 3; ++j) {
        if (sigma[j] <= cutoff) continue;
        // x += v_j (u_j . b) / sigma_j with u_j the normalized column
        const double ub = (u[j][0] * b[0] + u[j][1] * b[1] + u[j][2] * b[2]) / (sigma[j] * sigma[j]);
        for (int k = 0; k < 3; ++k) x[k] += v[j][k] * ub;
    }
    return x;
}

}  // namespace

std::optional<std::variant<Vector, Axis>> Axis::intersect(const Axis& other) const {
    if (is_coaxial(other)) return *this;
    if (is_skew(other)) return std::nullopt;
    // Solve p1 + t1*d1 = p2 + t2*d2 in the basis (d1, -d2, d1 x d2) by least squares.
    const Vector& d1 = direction;
    const Vector d2 = -other.direction;
    const Vector d3 = direction.cross(other.direction);
    const Vector rhs = other.position - position;
    const std::array<std::array<double, 3>, 3> system = {
        {{d1.X, d2.X, d3.X}, {d1.Y, d2.Y, d3.Y}, {d1.Z, d2.Z, d3.Z}}};
    const double t1 = lstsq3(system, {rhs.X, rhs.Y, rhs.Z})[0];
    return position + d1 * t1;
}

std::optional<Vector> Axis::intersect(const Vector& point) const {
    const Vector vec_to_point = point - position;
    const double projected_length = vec_to_point.dot(direction);
    const Vector projected = direction * projected_length + position;
    if (point == projected) return point;
    return std::nullopt;
}

std::optional<std::variant<Vector, Location>> Axis::intersect(const Location& location) const {
    const Vector location_dir = Plane(location).z_dir;
    if (!intersect(location.position())) return std::nullopt;
    if (location_dir == direction) return location;
    return location.position();
}

std::optional<std::variant<Vector, Axis>> Axis::intersect(const Plane& plane) const { return plane.intersect(*this); }

// ---- Vector.intersect

std::optional<Vector> Vector::intersect(const Vector& vector) const {
    if (*this == vector) return vector;
    return std::nullopt;
}
std::optional<Vector> Vector::intersect(const Location& location) const { return location.intersect(*this); }
std::optional<Vector> Vector::intersect(const Axis& axis) const { return axis.intersect(*this); }
std::optional<Vector> Vector::intersect(const Plane& plane) const { return plane.intersect(*this); }

std::size_t Axis::hash() const { return hash_values(hash_values(1, rounded_key(position)), rounded_key(direction)); }

bool Axis::operator==(const Axis& o) const {
    return position == o.position && direction == o.direction;
}

std::string Axis::to_string() const {
    return "Axis(position=" + position.to_string() + ", direction=" + direction.to_string() + ")";
}

std::ostream& operator<<(std::ostream& os, const Axis& a) { return os << a.to_string(); }

// ---------------------------------------------------------------------------
// Location
// ---------------------------------------------------------------------------

Location::Location(const Vector& position) { trsf_.SetTranslationPart(position.to_vec()); }

Location::Location(const Vector& position, const Vector& rotation, Intrinsic ordering) {
    trsf_ = detail::euler_trsf(rotation, detail::to_occ(ordering));
    trsf_.SetTranslationPart(position.to_vec());
}

Location::Location(const Vector& position, const Vector& rotation, Extrinsic ordering) {
    trsf_ = detail::euler_trsf(rotation, detail::to_occ(ordering));
    trsf_.SetTranslationPart(position.to_vec());
}

Location::Location(const Vector& position, const Vector& direction, double angle) {
    trsf_.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), direction.to_dir()), angle * DEG2RAD);
    trsf_.SetTranslationPart(position.to_vec());
}

Location::Location(const Vector& position, double angle) {
    trsf_.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), angle * DEG2RAD);
    trsf_.SetTranslationPart(position.to_vec());
}

Location::Location(const Plane& plane) {
    // Rebuilt from the plane's directions, as build123d does (not its stored gp_Pln).
    trsf_.SetTransformation(gp_Ax3(plane.origin.to_pnt(), plane.z_dir.to_dir(), plane.x_dir.to_dir()));
    trsf_.Invert();
}

Location::Location(const Plane& plane, const Vector& plane_offset)
    : Location(Location(plane) * Location(plane_offset)) {}

Vector Location::position() const { return Vector(trsf_.TranslationPart()); }

Vector Location::orientation() const {
    double a = 0, b = 0, c = 0;
    trsf_.GetRotation().GetEulerAngles(gp_Intrinsic_XYZ, a, b, c);
    return {a * RAD2DEG, b * RAD2DEG, c * RAD2DEG};
}

void Location::set_position(const Vector& value) {
    gp_Trsf position;
    position.SetTranslationPart(value.to_vec());
    gp_Trsf orientation;
    orientation.SetRotation(trsf_.GetRotation());
    trsf_ = position * orientation;
}

void Location::set_orientation(const Vector& rotation) {
    gp_Trsf position;
    position.SetTranslationPart(gp_Vec(trsf_.TranslationPart()));
    const gp_Trsf orientation = detail::euler_trsf(rotation, gp_Intrinsic_XYZ);
    trsf_ = position * orientation;
}

Axis Location::x_axis() const {
    const Plane p(*this);
    return Axis(p.origin, p.x_dir);
}
Axis Location::y_axis() const {
    const Plane p(*this);
    return Axis(p.origin, p.y_dir);
}
Axis Location::z_axis() const {
    const Plane p(*this);
    return Axis(p.origin, p.z_dir);
}

Location Location::inverse() const { return Location(trsf_.Inverted()); }

Location Location::operator*(const Location& other) const { return Location(trsf_ * other.trsf_); }

Location Location::pow(int exponent) const { return Location(trsf_.Powered(exponent)); }

Location Location::operator-() const { return Location(-Plane(*this)); }

Location Location::mirror(const Plane& mirror_plane) const {
    auto mirror_dir = [&](const Vector& v) {
        return v - mirror_plane.z_dir * (2.0 * v.dot(mirror_plane.z_dir));
    };
    const Vector pos = position();
    const double distance = (pos - mirror_plane.origin).dot(mirror_plane.z_dir);
    const Vector new_pos = pos - mirror_plane.z_dir * (2.0 * distance);
    const Plane loc_plane(*this);
    return Location(Plane(new_pos, mirror_dir(loc_plane.x_dir), mirror_dir(loc_plane.z_dir)));
}

std::optional<Vector> Location::intersect(const Vector& vector) const {
    if (position() == vector) return vector;
    return std::nullopt;
}

std::optional<std::variant<Vector, Location>> Location::intersect(const Location& location) const {
    if (*this == location) return *this;
    if (position() == location.position()) return position();
    return std::nullopt;
}

std::optional<std::variant<Vector, Location>> Location::intersect(const Axis& axis) const {
    return axis.intersect(*this);
}

std::optional<std::variant<Vector, Location>> Location::intersect(const Plane& plane) const {
    return plane.intersect(*this);
}

std::size_t Location::hash() const {
    return hash_values(hash_values(2, rounded_key(position())), quaternion_key(trsf_.GetRotation()));
}

bool Location::operator==(const Location& o) const {
    return rounded_key(position()) == rounded_key(o.position()) &&
           quaternion_key(trsf_.GetRotation()) == quaternion_key(o.trsf_.GetRotation());
}

std::string Location::to_string() const {
    return "Location(position=" + position().to_string() +
           ", orientation=" + orientation().to_string() + ")";
}

std::ostream& operator<<(std::ostream& os, const Location& l) { return os << l.to_string(); }

Rotation::Rotation(double X) : Rotation(X, 0.0, 0.0) {}

Rotation::Rotation(double X, double Y, double Z, Intrinsic ordering)
    : Location(Vector(0, 0, 0), Vector(X, Y, Z), ordering) {}

Rotation::Rotation(double X, double Y, double Z, Extrinsic ordering)
    : Location(Vector(0, 0, 0), Vector(X, Y, Z), ordering) {}

Rotation::Rotation(const Vector& rotation, Intrinsic ordering) : Location(Vector(0, 0, 0), rotation, ordering) {}

Rotation::Rotation(const Vector& rotation, Extrinsic ordering) : Location(Vector(0, 0, 0), rotation, ordering) {}

Rotation::Rotation(const Axis& axis, double angle) {
    if (angle != 0.0) trsf_.SetRotation(axis.wrapped(), angle * DEG2RAD);
}

// ---------------------------------------------------------------------------
// Plane
// ---------------------------------------------------------------------------

// Named planes: origin, x_dir, z_dir.
const Plane Plane::XY{Vector(0, 0, 0), Vector(1, 0, 0), Vector(0, 0, 1), Plane::raw_tag{}};
const Plane Plane::YZ{Vector(0, 0, 0), Vector(0, 1, 0), Vector(1, 0, 0), Plane::raw_tag{}};
const Plane Plane::ZX{Vector(0, 0, 0), Vector(0, 0, 1), Vector(0, 1, 0), Plane::raw_tag{}};
const Plane Plane::XZ{Vector(0, 0, 0), Vector(1, 0, 0), Vector(0, -1, 0), Plane::raw_tag{}};
const Plane Plane::YX{Vector(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, -1), Plane::raw_tag{}};
const Plane Plane::ZY{Vector(0, 0, 0), Vector(0, 0, 1), Vector(-1, 0, 0), Plane::raw_tag{}};
const Plane Plane::front{Vector(0, 0, 0), Vector(1, 0, 0), Vector(0, -1, 0), Plane::raw_tag{}};
const Plane Plane::back{Vector(0, 0, 0), Vector(-1, 0, 0), Vector(0, 1, 0), Plane::raw_tag{}};
const Plane Plane::left{Vector(0, 0, 0), Vector(0, -1, 0), Vector(-1, 0, 0), Plane::raw_tag{}};
const Plane Plane::right{Vector(0, 0, 0), Vector(0, 1, 0), Vector(1, 0, 0), Plane::raw_tag{}};
const Plane Plane::top{Vector(0, 0, 0), Vector(1, 0, 0), Vector(0, 0, 1), Plane::raw_tag{}};
const Plane Plane::bottom{Vector(0, 0, 0), Vector(1, 0, 0), Vector(0, 0, -1), Plane::raw_tag{}};
// build123d passes these positionally, which normalizes them once more than keywords do.
const Plane Plane::isometric{
    Vector(0, 0, 0), Vector(1 / std::sqrt(2.0), 1 / std::sqrt(2.0), 0).normalized(),
    Vector(1 / std::sqrt(3.0), -1 / std::sqrt(3.0), 1 / std::sqrt(3.0)).normalized()};

void Plane::init(const gp_Ax3& ax3_in) {
    gp_Ax3 ax3 = ax3_in;
    if (!ax3.Direct()) {
        warn("Trying to set a left-handed plane");
        ax3 = gp_Ax3(gp_Ax2(ax3.Location(), ax3.Direction(), ax3.XDirection()));
    }
    ax3_ = ax3;
    origin = Vector(ax3.Location());
    x_dir = Vector(ax3.XDirection());
    y_dir = Vector(ax3.YDirection());
    z_dir = Vector(ax3.Direction());
}

void Plane::init(const Vector& o, const Vector& x, const Vector& z) {
    detail::occt("Plane", [&] {
        init(gp_Ax3(o.to_pnt(), z.to_dir(), x.to_dir()));
        return 0;
    });
}

Plane::Plane(const Vector& o, std::optional<Vector> x, const Vector& z) {
    if (z.length() == 0.0) throw std::invalid_argument("z_dir must be non null");
    const Vector zn = z.normalized();
    Vector xn;
    if (!x) {
        const gp_Ax3 ax3(o.to_pnt(), zn.to_dir());
        xn = Vector(ax3.XDirection()).normalized();
    } else {
        if (x->length() == 0.0) throw std::invalid_argument("x_dir must be non null");
        xn = x->normalized();
    }
    init(o, xn, zn);
}

Plane::Plane(const gp_Pln& pln) { init(pln.Position()); }

Plane::Plane(const gp_Ax3& ax3) { init(ax3); }

Plane::Plane(const Location& location) {
    // As build123d: the directions of an XY face moved to the location.
    TopoDS_Face face = BRepBuilderAPI_MakeFace(Plane::XY.wrapped(), -1.0, 1.0, -1.0, 1.0).Face();
    face.Move(location.wrapped());
    const Handle(Geom_ElementarySurface) surface =
        Handle(Geom_ElementarySurface)::DownCast(BRep_Tool::Surface(face));
    const Vector x = round14(Vector(surface->Position().XDirection()));
    const Vector z = round14(detail::face_normal_at_origin_projection(face));
    init(location.position(), x.normalized(), z.normalized());
}

Plane::Plane(const Axis& axis, std::optional<Vector> x) : Plane(axis.position, x, axis.direction) {}

Plane Plane::from_x_y(const Vector& o, const Vector& x, const Vector& y) {
    if (x.length() == 0.0) throw std::invalid_argument("x_dir must be non null");
    if (y.length() == 0.0) throw std::invalid_argument("y_dir must be non null");
    const Vector xn = x.normalized();
    const Vector yn = y.normalized();
    const Vector z = xn.cross(yn);
    if (z.length() == 0.0) throw std::invalid_argument("x_dir and y_dir must not be parallel");
    const Vector zn = z.normalized();
    const Vector y2 = zn.cross(xn).normalized();
    const Vector x2 = y2.cross(zn).normalized();
    Plane p;
    p.init(o, x2, zn);
    return p;
}

Plane Plane::from_points(const Vector& p0, const Vector& p1, const Vector& p2) {
    const Vector x = p1 - p0;
    const Vector z = x.cross(p2 - p0);
    return Plane(p0, x, z);
}

Plane Plane::offset(double amount) const { return Plane(origin + z_dir * amount, x_dir, z_dir); }

Plane Plane::reverse() const { return Plane(origin, x_dir, -z_dir); }

Plane Plane::rotated(const Vector& rotation, Intrinsic ordering) const {
    const gp_Trsf rot = detail::euler_trsf(rotation, detail::to_occ(ordering));
    gp_Ax2 ax = to_gp_ax2().Transformed(rot);
    ax.SetLocation(origin.to_pnt());
    return Plane(gp_Ax3(ax));
}

Plane Plane::rotated(const Vector& rotation, Extrinsic ordering) const {
    const gp_Trsf rot = detail::euler_trsf(rotation, detail::to_occ(ordering));
    gp_Ax2 ax = to_gp_ax2().Transformed(rot);
    ax.SetLocation(origin.to_pnt());
    return Plane(gp_Ax3(ax));
}

Plane Plane::moved(const Location& loc) const { return Plane(location() * loc); }

Plane Plane::moved(const Plane& plane) const { return moved(plane.location()); }

Plane& Plane::move(const Location& loc) {
    *this = moved(loc);
    return *this;
}

Plane& Plane::move(const Plane& plane) { return move(plane.location()); }

Plane Plane::shift_origin(const Vector& locator) const {
    if (!contains(locator)) {
        throw std::invalid_argument(locator.to_string() + " is not located within plane");
    }
    return Plane(locator, x_dir, z_dir);
}

Plane Plane::shift_origin(const Axis& locator) const {
    const auto intersection = intersect(locator);
    if (!intersection || !std::holds_alternative<Vector>(*intersection)) {
        throw std::invalid_argument(locator.to_string() + " doesn't intersect the plane");
    }
    return Plane(std::get<Vector>(*intersection), x_dir, z_dir);
}

void Plane::set_x_dir(const Vector& direction) {
    gp_Ax2 ax2 = to_gp_ax2();
    ax2.SetXDirection(direction.to_dir());
    init(gp_Ax3(ax2));
}

void Plane::set_y_dir(const Vector& direction) {
    gp_Ax2 ax2 = to_gp_ax2();
    ax2.SetYDirection(direction.to_dir());
    init(gp_Ax3(ax2));
}

Vector Plane::get_topods_face_normal(const TopoDS_Face& face) { return detail::face_normal_at_origin_projection(face); }

gp_Pln Plane::wrapped() const { return gp_Pln(to_gp_ax3()); }

gp_Ax3 Plane::to_gp_ax3() const {
    const auto same = [](const Vector& v, const gp_XYZ& p) { return v.X == p.X() && v.Y == p.Y() && v.Z == p.Z(); };
    if (same(x_dir, ax3_.XDirection().XYZ()) && same(y_dir, ax3_.YDirection().XYZ()) &&
        same(z_dir, ax3_.Direction().XYZ())) {
        if (same(origin, ax3_.Location().XYZ())) return ax3_;
        gp_Ax3 moved = ax3_;
        moved.SetLocation(origin.to_pnt());
        return moved;
    }
    return gp_Ax3(origin.to_pnt(), z_dir.to_dir(), x_dir.to_dir());
}

gp_Ax2 Plane::to_gp_ax2() const { return to_gp_ax3().Ax2(); }

gp_Trsf Plane::forward_transform() const {
    gp_Trsf t;
    t.SetTransformation(gp_Ax3(), to_gp_ax3());
    return t;
}

gp_Trsf Plane::reverse_transform() const {
    gp_Trsf t;
    t.SetTransformation(to_gp_ax3(), gp_Ax3());
    return t;
}

Vector Plane::to_local_coords(const Vector& v) const { return v.transform(forward_transform()); }

Vector Plane::from_local_coords(const Vector& v) const { return v.transform(reverse_transform()); }

BoundBox Plane::to_local_coords(const BoundBox& box) const {
    // as build123d: Bnd_Box(min, max) of the moved corners, unsorted
    return BoundBox(Bnd_Box(to_local_coords(box.min).to_pnt(), to_local_coords(box.max).to_pnt()));
}

BoundBox Plane::from_local_coords(const BoundBox& box) const {
    return BoundBox(Bnd_Box(from_local_coords(box.min).to_pnt(), from_local_coords(box.max).to_pnt()));
}

Location Plane::location_between(const Plane& other) const {
    gp_Trsf t;
    t.SetTransformation(to_gp_ax3(), other.to_gp_ax3());
    return Location(t);
}

bool Plane::contains(const Vector& point, double tolerance) const {
    return wrapped().Contains(point.to_pnt(), tolerance);
}

bool Plane::contains(const Axis& axis, double tolerance) const {
    return wrapped().Contains(gp_Lin(axis.position.to_pnt(), axis.direction.to_dir()), tolerance,
                              tolerance);
}

std::optional<std::variant<Vector, Axis>> Plane::intersect(const Axis& axis) const {
    if (contains(axis)) return axis;
    Handle(Geom_Line) line = new Geom_Line(axis.wrapped());
    Handle(Geom_Plane) plane = new Geom_Plane(to_gp_ax3());
    GeomAPI_IntCS calc(line, plane);
    if (calc.IsDone() && calc.NbPoints() == 1) return Vector(calc.Point(1));
    return std::nullopt;
}

std::optional<std::variant<Axis, Plane>> Plane::intersect(const Plane& other) const {
    if (contains(other.origin) && z_dir == other.z_dir) return *this;
    Handle(Geom_Plane) s1 = new Geom_Plane(wrapped());
    Handle(Geom_Plane) s2 = new Geom_Plane(other.wrapped());
    GeomAPI_IntSS inter(s1, s2, TOLERANCE);
    if (inter.IsDone() && inter.NbLines() > 0) {
        Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(inter.Line(1));
        if (!line.IsNull()) return Axis(line->Position());
    }
    return std::nullopt;
}

std::optional<Vector> Plane::intersect(const Vector& point) const {
    if (contains(point)) return point;
    return std::nullopt;
}

std::optional<std::variant<Vector, Location>> Plane::intersect(const Location& location) const {
    const Plane location_plane(location);
    if (!contains(location_plane.origin)) return std::nullopt;
    if (z_dir == location_plane.z_dir) return location;
    return location_plane.origin;
}

std::size_t Plane::hash() const {
    return hash_values(hash_values(hash_values(3, rounded_key(origin)), rounded_key(x_dir)), rounded_key(z_dir));
}

bool Plane::operator==(const Plane& o) const {
    return origin == o.origin && x_dir == o.x_dir && z_dir == o.z_dir;
}

std::string Plane::to_string() const {
    return "Plane(origin=" + origin.to_string() + ", x_dir=" + x_dir.to_string() +
           ", z_dir=" + z_dir.to_string() + ")";
}

std::ostream& operator<<(std::ostream& os, const Plane& p) { return os << p.to_string(); }

Location operator*(const Plane& plane, const Location& loc) { return Location(plane) * loc; }

Location operator*(const Plane& plane, const Plane& other) {
    return Location(plane) * other.location();
}

Plane operator*(const Location& loc, const Plane& plane) { return plane.moved(loc); }

std::vector<Location> operator*(const Plane& plane, const std::vector<Location>& locs) {
    std::vector<Location> out;
    out.reserve(locs.size());
    for (const auto& l : locs) out.push_back(Location(plane) * l);
    return out;
}

std::vector<Location> operator*(const Location& loc, const std::vector<Location>& locs) {
    std::vector<Location> out;
    out.reserve(locs.size());
    for (const auto& l : locs) out.push_back(loc * l);
    return out;
}

std::vector<Location> operator*(const Plane& plane, const std::vector<Plane>& planes) {
    std::vector<Location> out;
    out.reserve(planes.size());
    for (const auto& p : planes) out.push_back(Location(plane) * p.location());
    return out;
}

std::vector<Plane> operator*(const Location& loc, const std::vector<Plane>& planes) {
    std::vector<Plane> out;
    out.reserve(planes.size());
    for (const auto& p : planes) out.push_back(p.moved(loc));
    return out;
}

std::vector<Plane> operator*(const std::vector<Location>& locs, const Plane& plane) {
    std::vector<Plane> out;
    out.reserve(locs.size());
    for (const auto& l : locs) out.push_back(plane.moved(l));
    return out;
}

std::vector<Plane> operator*(const std::vector<Plane>& planes, const Plane& plane) {
    std::vector<Plane> out;
    out.reserve(planes.size());
    for (const auto& p : planes) out.push_back(plane.moved(p));
    return out;
}

// ---------------------------------------------------------------------------
// BoundBox
// ---------------------------------------------------------------------------

BoundBox::BoundBox(const Bnd_Box& box) : box_(box), void_(box.IsVoid()) {
    if (!void_) {
        double x0, y0, z0, x1, y1, z1;
        box.Get(x0, y0, z0, x1, y1, z1);
        min = Vector(x0, y0, z0);
        max = Vector(x1, y1, z1);
        size = Vector(x1 - x0, y1 - y0, z1 - z0);
    }
}

BoundBox BoundBox::from_topo_ds(const TopoDS_Shape& shape, std::optional<double> /*tolerance*/,
                                bool optimal) {
    Bnd_Box box;
    if (!shape.IsNull()) {
        BRepTools::Clean(shape);
        if (optimal) {
            BRepBndLib::AddOptimal(shape, box);
        } else {
            BRepBndLib::Add(shape, box, true);
        }
    }
    return BoundBox(box);
}

double BoundBox::diagonal() const { return void_ ? 0.0 : std::sqrt(box_.SquareExtent()); }

double BoundBox::measure() const {
    double result = 1.0;
    for (double s : {size.X, size.Y, size.Z}) {
        if (s > TOLERANCE) result *= s;
    }
    return result;
}

BoundBox BoundBox::add(const Vector& point, std::optional<double> tol) const {
    Bnd_Box tmp;
    tmp.SetGap(tol.value_or(TOL));
    if (!void_) tmp.Add(box_);
    tmp.Update(point.X, point.Y, point.Z);
    return BoundBox(tmp);
}

BoundBox BoundBox::add(const BoundBox& other, std::optional<double> tol) const {
    Bnd_Box tmp;
    tmp.SetGap(tol.value_or(TOL));
    if (!void_) tmp.Add(box_);
    if (!other.void_) tmp.Add(other.box_);
    return BoundBox(tmp);
}

bool BoundBox::covers(const BoundBox& other, double tolerance) const {
    if (void_ || other.void_) return false;
    const double smin[3] = {min.X, min.Y, min.Z}, smax[3] = {max.X, max.Y, max.Z};
    const double omin[3] = {other.min.X, other.min.Y, other.min.Z};
    const double omax[3] = {other.max.X, other.max.Y, other.max.Z};
    for (int i = 0; i < 3; ++i) {
        if (!(smin[i] <= omin[i] + tolerance && omax[i] <= smax[i] + tolerance)) return false;
    }
    return true;
}

bool BoundBox::covered_by(const BoundBox& other, double tolerance) const {
    return other.covers(*this, tolerance);
}

namespace {

/// BoundBox._interior_axes: the (self_min, self_max, other_min, other_max) of each axis
/// relevant to an interior comparison; std::nullopt when a degenerate axis of both boxes
/// is at different coordinates.
std::optional<std::vector<std::array<double, 4>>> interior_axes(const BoundBox& self, const BoundBox& other,
                                                                 double tolerance) {
    const double smin[3] = {self.min.X, self.min.Y, self.min.Z}, smax[3] = {self.max.X, self.max.Y, self.max.Z};
    const double omin[3] = {other.min.X, other.min.Y, other.min.Z};
    const double omax[3] = {other.max.X, other.max.Y, other.max.Z};
    std::vector<std::array<double, 4>> axes;
    for (int i = 0; i < 3; ++i) {
        const bool self_degenerate = smax[i] - smin[i] <= tolerance;
        const bool other_degenerate = omax[i] - omin[i] <= tolerance;
        if (self_degenerate && other_degenerate) {
            const double self_center = (smin[i] + smax[i]) / 2;
            const double other_center = (omin[i] + omax[i]) / 2;
            if (std::abs(self_center - other_center) > tolerance) return std::nullopt;
        } else {
            axes.push_back({smin[i], smax[i], omin[i], omax[i]});
        }
    }
    return axes;
}

}  // namespace

bool BoundBox::has_interior_overlap(const BoundBox& other, double tolerance) const {
    if (void_ || other.void_) return false;
    const auto axes = interior_axes(*this, other, tolerance);
    if (!axes) return false;
    for (const auto& [self_min, self_max, other_min, other_max] : *axes) {
        if (!(std::max(self_min, other_min) < std::min(self_max, other_max) - tolerance)) return false;
    }
    return true;
}

bool BoundBox::contains(const BoundBox& other, double tolerance) const {
    return covers(other, tolerance) && has_interior_overlap(other, tolerance);
}

bool BoundBox::contains_properly(const BoundBox& other, double tolerance) const {
    if (void_ || other.void_) return false;
    const auto axes = interior_axes(*this, other, tolerance);
    if (!axes || axes->empty()) return false;
    for (const auto& [self_min, self_max, other_min, other_max] : *axes) {
        if (!(self_min + tolerance < other_min && other_max < self_max - tolerance)) return false;
    }
    return true;
}

bool BoundBox::touches(const BoundBox& other, double tolerance) const {
    return intersects(other, tolerance) && !has_interior_overlap(other, tolerance);
}

bool BoundBox::overlaps(const BoundBox& other, double tolerance) const {
    return has_interior_overlap(other, tolerance) && !contains(other, tolerance) && !other.contains(*this, tolerance);
}

std::optional<BoundBox> BoundBox::find_outside_box_2d(const BoundBox& bb1, const BoundBox& bb2) {
    if (bb1.min.X < bb2.min.X && bb1.max.X > bb2.max.X && bb1.min.Y < bb2.min.Y && bb1.max.Y > bb2.max.Y) {
        return bb1;
    }
    if (bb2.min.X < bb1.min.X && bb2.max.X > bb1.max.X && bb2.min.Y < bb1.min.Y && bb2.max.Y > bb1.max.Y) {
        return bb2;
    }
    return std::nullopt;
}

bool BoundBox::within(const BoundBox& other, double tolerance) const {
    return other.contains(*this, tolerance);
}

bool BoundBox::intersects(const BoundBox& other, double tolerance) const {
    if (void_ || other.void_) return false;
    return box_.Distance(other.box_) <= tolerance;
}

bool BoundBox::disjoint(const BoundBox& other, double tolerance) const {
    return !intersects(other, tolerance);
}

Vector BoundBox::to_align_offset(const Align3& align) const {
    return build123d::to_align_offset(min, max, align);
}

Vector BoundBox::to_align_offset(const Align2& align) const {
    return build123d::to_align_offset(min, max, Align3(align, Align::NONE));
}

std::string BoundBox::to_string() const {
    using detail::py_repr;
    return "bbox: " + py_repr(min.X) + " <= x <= " + py_repr(max.X) + ", " + py_repr(min.Y) + " <= y <= " +
           py_repr(max.Y) + ", " + py_repr(min.Z) + " <= z <= " + py_repr(max.Z);
}

std::ostream& operator<<(std::ostream& os, const BoundBox& b) { return os << b.to_string(); }

Vector to_align_offset(const Vector& min_point, const Vector& max_point, const Align3& align,
                       std::optional<Vector> center) {
    const Vector c = center ? *center : (min_point + max_point) / 2.0;
    const double mins[3] = {min_point.X, min_point.Y, min_point.Z};
    const double maxs[3] = {max_point.X, max_point.Y, max_point.Z};
    const double cs[3] = {c.X, c.Y, c.Z};
    double out[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        switch (align[i]) {
            case Align::MIN: out[i] = -mins[i]; break;
            case Align::CENTER: out[i] = -cs[i]; break;
            case Align::MAX: out[i] = -maxs[i]; break;
            case Align::NONE: out[i] = 0.0; break;
        }
    }
    return {out[0], out[1], out[2]};
}

// ---------------------------------------------------------------------------
// Color
// ---------------------------------------------------------------------------

namespace {

const std::unordered_map<std::string, std::uint32_t>& css_colors() {
    static const std::unordered_map<std::string, std::uint32_t> table = {
        {"aliceblue", 0xf0f8ff}, {"antiquewhite", 0xfaebd7}, {"aqua", 0x00ffff},
        {"aquamarine", 0x7fffd4}, {"azure", 0xf0ffff}, {"beige", 0xf5f5dc},
        {"bisque", 0xffe4c4}, {"black", 0x000000}, {"blanchedalmond", 0xffebcd},
        {"blue", 0x0000ff}, {"blueviolet", 0x8a2be2}, {"brown", 0xa52a2a},
        {"burlywood", 0xdeb887}, {"cadetblue", 0x5f9ea0}, {"chartreuse", 0x7fff00},
        {"chocolate", 0xd2691e}, {"coral", 0xff7f50}, {"cornflowerblue", 0x6495ed},
        {"cornsilk", 0xfff8dc}, {"crimson", 0xdc143c}, {"cyan", 0x00ffff},
        {"darkblue", 0x00008b}, {"darkcyan", 0x008b8b}, {"darkgoldenrod", 0xb8860b},
        {"darkgray", 0xa9a9a9}, {"darkgrey", 0xa9a9a9}, {"darkgreen", 0x006400},
        {"darkkhaki", 0xbdb76b}, {"darkmagenta", 0x8b008b}, {"darkolivegreen", 0x556b2f},
        {"darkorange", 0xff8c00}, {"darkorchid", 0x9932cc}, {"darkred", 0x8b0000},
        {"darksalmon", 0xe9967a}, {"darkseagreen", 0x8fbc8f}, {"darkslateblue", 0x483d8b},
        {"darkslategray", 0x2f4f4f}, {"darkslategrey", 0x2f4f4f}, {"darkturquoise", 0x00ced1},
        {"darkviolet", 0x9400d3}, {"deeppink", 0xff1493}, {"deepskyblue", 0x00bfff},
        {"dimgray", 0x696969}, {"dimgrey", 0x696969}, {"dodgerblue", 0x1e90ff},
        {"firebrick", 0xb22222}, {"floralwhite", 0xfffaf0}, {"forestgreen", 0x228b22},
        {"fuchsia", 0xff00ff}, {"gainsboro", 0xdcdcdc}, {"ghostwhite", 0xf8f8ff},
        {"gold", 0xffd700}, {"goldenrod", 0xdaa520}, {"gray", 0x808080}, {"grey", 0x808080},
        {"green", 0x008000}, {"greenyellow", 0xadff2f}, {"honeydew", 0xf0fff0},
        {"hotpink", 0xff69b4}, {"indianred", 0xcd5c5c}, {"indigo", 0x4b0082},
        {"ivory", 0xfffff0}, {"khaki", 0xf0e68c}, {"lavender", 0xe6e6fa},
        {"lavenderblush", 0xfff0f5}, {"lawngreen", 0x7cfc00}, {"lemonchiffon", 0xfffacd},
        {"lightblue", 0xadd8e6}, {"lightcoral", 0xf08080}, {"lightcyan", 0xe0ffff},
        {"lightgoldenrodyellow", 0xfafad2}, {"lightgray", 0xd3d3d3}, {"lightgrey", 0xd3d3d3},
        {"lightgreen", 0x90ee90}, {"lightpink", 0xffb6c1}, {"lightsalmon", 0xffa07a},
        {"lightseagreen", 0x20b2aa}, {"lightskyblue", 0x87cefa}, {"lightslategray", 0x778899},
        {"lightslategrey", 0x778899}, {"lightsteelblue", 0xb0c4de}, {"lightyellow", 0xffffe0},
        {"lime", 0x00ff00}, {"limegreen", 0x32cd32}, {"linen", 0xfaf0e6},
        {"magenta", 0xff00ff}, {"maroon", 0x800000}, {"mediumaquamarine", 0x66cdaa},
        {"mediumblue", 0x0000cd}, {"mediumorchid", 0xba55d3}, {"mediumpurple", 0x9370db},
        {"mediumseagreen", 0x3cb371}, {"mediumslateblue", 0x7b68ee},
        {"mediumspringgreen", 0x00fa9a}, {"mediumturquoise", 0x48d1cc},
        {"mediumvioletred", 0xc71585}, {"midnightblue", 0x191970}, {"mintcream", 0xf5fffa},
        {"mistyrose", 0xffe4e1}, {"moccasin", 0xffe4b5}, {"navajowhite", 0xffdead},
        {"navy", 0x000080}, {"oldlace", 0xfdf5e6}, {"olive", 0x808000},
        {"olivedrab", 0x6b8e23}, {"orange", 0xffa500}, {"orangered", 0xff4500},
        {"orchid", 0xda70d6}, {"palegoldenrod", 0xeee8aa}, {"palegreen", 0x98fb98},
        {"paleturquoise", 0xafeeee}, {"palevioletred", 0xdb7093}, {"papayawhip", 0xffefd5},
        {"peachpuff", 0xffdab9}, {"peru", 0xcd853f}, {"pink", 0xffc0cb}, {"plum", 0xdda0dd},
        {"powderblue", 0xb0e0e6}, {"purple", 0x800080},
        {"red", 0xff0000}, {"rosybrown", 0xbc8f8f}, {"royalblue", 0x4169e1},
        {"saddlebrown", 0x8b4513}, {"salmon", 0xfa8072}, {"sandybrown", 0xf4a460},
        {"seagreen", 0x2e8b57}, {"seashell", 0xfff5ee}, {"sienna", 0xa0522d},
        {"silver", 0xc0c0c0}, {"skyblue", 0x87ceeb}, {"slateblue", 0x6a5acd},
        {"slategray", 0x708090}, {"slategrey", 0x708090}, {"snow", 0xfffafa},
        {"springgreen", 0x00ff7f}, {"steelblue", 0x4682b4}, {"tan", 0xd2b48c},
        {"teal", 0x008080}, {"thistle", 0xd8bfd8}, {"tomato", 0xff6347},
        {"turquoise", 0x40e0d0}, {"violet", 0xee82ee}, {"wheat", 0xf5deb3},
        {"white", 0xffffff}, {"whitesmoke", 0xf5f5f5}, {"yellow", 0xffff00},
        {"yellowgreen", 0x9acd32}};
    return table;
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    throw std::invalid_argument(std::string("Invalid hex digit '") + c + "'");
}

/// webcolors' rgb_to_name for CSS3: the name of a color code, preferring "cyan",
/// "magenta" and the "gray" spellings among the aliases, as webcolors does.
std::optional<std::string> css_name_of(std::uint32_t code) {
    static const std::unordered_map<std::uint32_t, std::string> names = [] {
        std::unordered_map<std::uint32_t, std::string> out;
        const auto preferred = [](const std::string& a, const std::string& b) {
            if (a == "cyan" || a == "magenta") return a;
            if (b == "cyan" || b == "magenta") return b;
            return a.find("grey") == std::string::npos ? a : b;
        };
        for (const auto& [name, value] : css_colors()) {
            const auto it = out.find(value);
            out[value] = it == out.end() ? name : preferred(it->second, name);
        }
        return out;
    }();
    const auto it = names.find(code);
    if (it == names.end()) return std::nullopt;
    return it->second;
}

/// Python's str.strip(): leading and trailing whitespace removed.
std::string strip(const std::string& text) {
    const char* space = " \t\n\r\f\v";
    const auto first = text.find_first_not_of(space);
    if (first == std::string::npos) return "";
    return text.substr(first, text.find_last_not_of(space) - first + 1);
}

/// colorsys.hls_to_rgb.
std::array<double, 3> hls_to_rgb(double h, double l, double s) {
    constexpr double one_third = 1.0 / 3.0, one_sixth = 1.0 / 6.0, two_third = 2.0 / 3.0;
    if (s == 0.0) return {l, l, l};
    const double m2 = l <= 0.5 ? l * (1.0 + s) : l + s - (l * s);
    const double m1 = 2.0 * l - m2;
    const auto v = [&](double hue) {
        hue = detail::py_float_mod(hue, 1.0);
        if (hue < one_sixth) return m1 + (m2 - m1) * hue * 6.0;
        if (hue < 0.5) return m2;
        if (hue < two_third) return m1 + (m2 - m1) * (two_third - hue) * 6.0;
        return m1;
    };
    return {v(h + one_third), v(h), v(h - one_third)};
}

/// colorsys.rgb_to_hls (the hue).
double rgb_to_hue(double r, double g, double b) {
    const double maxc = std::max({r, g, b}), minc = std::min({r, g, b});
    if (minc == maxc) return 0.0;
    const double rangec = maxc - minc;
    const double rc = (maxc - r) / rangec, gc = (maxc - g) / rangec, bc = (maxc - b) / rangec;
    double h;
    if (r == maxc) {
        h = bc - gc;
    } else if (g == maxc) {
        h = 2.0 + rc - bc;
    } else {
        h = 4.0 + gc - rc;
    }
    return detail::py_float_mod(h / 6.0, 1.0);
}

/// Quantity_Color's range check (build123d builds one in Color.__init__).
void check_color(double r, double g, double b) {
    detail::occt("Color", [&] { return Quantity_Color(r, g, b, Quantity_TOC_sRGB); });
}

}  // namespace

Color::Color(double r, double g, double b, double a) : red(r), green(g), blue(b), alpha(a) { check_color(r, g, b); }

Color::Color(double r) : red(r) { check_color(red, green, blue); }

Color::Color(double r, double g) : red(r), green(g) { check_color(red, green, blue); }

Color::Color(const Quantity_ColorRGBA& color) {
    color.GetRGB().Values(red, green, blue, Quantity_TOC_sRGB);
    alpha = color.Alpha();
}

Color Color::from_code(long long color_code, int alpha_code) {
    // Color._rgb_from_int: divmod by 256**2 and 256 (floor division)
    const auto floor_div = [](long long a, long long b) {
        long long q = a / b;
        if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
        return q;
    };
    const long long red_code = floor_div(color_code, 256 * 256);
    const long long remainder = color_code - red_code * 256 * 256;
    const long long green_code = floor_div(remainder, 256);
    const long long blue_code = remainder - green_code * 256;
    // alpha 1 stands for opaque (build123d special-cases the default 1.0)
    const double a = alpha_code == 1 ? 1.0 : alpha_code / 255.0;
    return Color(red_code / 255.0, green_code / 255.0, blue_code / 255.0, a);
}

std::vector<Color> Color::categorical_set(int color_count, double starting_hue, double alpha) {
    return categorical_set(color_count, starting_hue, std::vector<double>(std::max(color_count, 0), alpha));
}

std::vector<Color> Color::categorical_set(int color_count, double starting_hue, const std::vector<double>& alphas) {
    if (!(0.0 <= starting_hue && starting_hue <= 1.0)) {
        throw std::invalid_argument("Starting hue must be within range 0.0–1.0");
    }
    if (static_cast<long long>(alphas.size()) != color_count) {
        throw std::invalid_argument("Number of alpha values must match color_count");
    }
    if (color_count < 0) throw std::invalid_argument("Number of samples must be non-negative");
    // np.linspace(starting_hue, starting_hue + 1.0, color_count, endpoint=False)
    const double delta = (starting_hue + 1.0) - starting_hue;
    const double step = delta / color_count;
    std::vector<Color> colors;
    for (int i = 0; i < color_count; ++i) {
        const double hue = i * step + starting_hue;
        const auto [r, g, b] = hls_to_rgb(detail::py_float_mod(hue, 1.0), 0.55, 0.9);
        colors.emplace_back(r, g, b, alphas[static_cast<std::size_t>(i)]);
    }
    return colors;
}

std::vector<Color> Color::categorical_set(int color_count, int starting_color, double alpha) {
    return categorical_set(color_count, starting_color, std::vector<double>(std::max(color_count, 0), alpha));
}

std::vector<Color> Color::categorical_set(int color_count, int starting_color, const std::vector<double>& alphas) {
    if (starting_color < 0) throw std::invalid_argument("Starting color integer must be non-negative");
    const auto rgb = Color(starting_color).to_tuple();
    return categorical_set(color_count, rgb_to_hue(rgb[0], rgb[1], rgb[2]), alphas);
}

Color::Color(const std::string& name_in, double a) : alpha(a) {
    std::string name = strip(name_in);
    if (name.find('#') != std::string::npos) {
        // the alpha of a hex color goes through two hex digits, as build123d
        char hex_alpha[32];
        const long long alpha_code = static_cast<long long>(alpha * 255);  // int() truncates
        std::snprintf(hex_alpha, sizeof(hex_alpha), alpha_code < 0 ? "-%llx" : "%llx",
                      static_cast<unsigned long long>(std::llabs(alpha_code)));
        std::string hex_a = hex_alpha;
        if (name.size() == 5) {
            hex_a = std::string(2, name[4]);
            name = name.substr(0, 4);
        } else if (name.size() == 9) {
            hex_a = name.substr(7, 2);
            name = name.substr(0, 7);
        } else if (name.size() != 4 && name.size() != 7) {
            throw std::invalid_argument('"' + name + "\" is not a valid hexadecimal color value.");
        }
        int alpha_value = 0;
        try {
            std::size_t used = 0;
            alpha_value = std::stoi(hex_a, &used, 16);
            if (used != hex_a.size()) throw std::invalid_argument(hex_a);
        } catch (const std::exception&) {
            throw std::invalid_argument("Invald alpha hex string: " + hex_a);
        }
        alpha = alpha_value / 255.0;
        // webcolors.hex_to_rgb: "#rgb" or "#rrggbb"
        const bool valid = name[0] == '#' && std::all_of(name.begin() + 1, name.end(), [](char c) {
                               return std::isxdigit(static_cast<unsigned char>(c)) != 0;
                           });
        if (!valid) throw std::invalid_argument('"' + name + "\" is not a valid hexadecimal color value.");
        std::string h = name.substr(1);
        if (h.size() == 3) h = {h[0], h[0], h[1], h[1], h[2], h[2]};
        const auto byte = [&](std::size_t i) { return hex_digit(h[i]) * 16 + hex_digit(h[i + 1]); };
        red = byte(0) / 255.0;
        green = byte(2) / 255.0;
        blue = byte(4) / 255.0;
        check_color(red, green, blue);
        return;
    }
    std::string lower;
    for (char c : name) lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    const auto& table = css_colors();
    const auto it = table.find(lower);
    if (it != table.end()) {
        const std::uint32_t v = it->second;
        red = ((v >> 16) & 0xff) / 255.0;
        green = ((v >> 8) & 0xff) / 255.0;
        blue = (v & 0xff) / 255.0;
        return;
    }
    // OCCT/X11 names: build123d takes Red()/Green()/Blue() of the OCCT color, which are its
    // linear RGB values, as the sRGB values of the new color
    Quantity_Color occ_color;
    if (Quantity_Color::ColorFromName(name.c_str(), occ_color)) {
        red = occ_color.Red();
        green = occ_color.Green();
        blue = occ_color.Blue();
        check_color(red, green, blue);
        return;
    }
    throw std::invalid_argument("'" + name + "' is not defined as a named color in CSS3 or OCCT/X11");
}

std::size_t Color::hash() const { return hash_values(4, std::array<double, 4>{red, green, blue, alpha}); }

Color Color::from_hex(std::uint32_t rgb, double a) {
    return Color(((rgb >> 16) & 0xff) / 255.0, ((rgb >> 8) & 0xff) / 255.0, (rgb & 0xff) / 255.0, a);
}

Quantity_ColorRGBA Color::wrapped() const {
    return Quantity_ColorRGBA(Quantity_Color(red, green, blue, Quantity_TOC_sRGB),
                              static_cast<float>(alpha));
}

std::array<double, 4> Color::to_tuple() const {
    const Quantity_ColorRGBA color = wrapped();
    double r, g, b;
    color.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
    return {py_round(r, 7), py_round(g, 7), py_round(b, 7), py_round(color.Alpha(), 7)};
}

namespace {

std::string tuple_repr(const std::array<double, 4>& values) {
    std::string out = "(";
    for (std::size_t i = 0; i < values.size(); ++i) out += (i ? ", " : "") + detail::py_repr(values[i]);
    return out + ")";
}

}  // namespace

std::string Color::to_string() const { return "Color" + tuple_repr(to_tuple()); }

std::string Color::str() const {
    const Quantity_ColorRGBA color = wrapped();
    double r, g, b;
    color.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
    const auto code = [](double c) { return static_cast<std::uint32_t>(std::nearbyint(c * 255)); };
    const auto css = (r >= 0 && g >= 0 && b >= 0 && code(r) < 256 && code(g) < 256 && code(b) < 256)
                         ? css_name_of((code(r) << 16) | (code(g) << 8) | code(b))
                         : std::nullopt;
    std::string name, qualifier;
    if (css) {
        name = *css;
        qualifier = "is";
    } else {
        name = Quantity_Color::StringName(color.GetRGB().Name());
        qualifier = "near";
    }
    for (char& c : name) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return "Color: " + tuple_repr(to_tuple()) + " " + qualifier + " '" + name + "'";
}

std::ostream& operator<<(std::ostream& os, const Color& c) { return os << c.to_string(); }

// ---------------------------------------------------------------------------
// Enum names
// ---------------------------------------------------------------------------

std::string to_string(Align v) {
    switch (v) {
        case Align::MIN: return "Align.MIN";
        case Align::CENTER: return "Align.CENTER";
        case Align::MAX: return "Align.MAX";
        case Align::NONE: return "Align.NONE";
    }
    return "Align.?";
}

std::string to_string(GeomType v) {
    static const char* names[] = {"PLANE", "CYLINDER", "CONE", "SPHERE", "TORUS", "BEZIER",
                                  "BSPLINE", "REVOLUTION", "EXTRUSION", "OFFSET", "LINE",
                                  "CIRCLE", "ELLIPSE", "HYPERBOLA", "PARABOLA", "OTHER"};
    return std::string("GeomType.") + names[static_cast<int>(v)];
}

std::string to_string(ShapeType v) {
    static const char* names[] = {"Vertex", "Edge", "Wire", "Face", "Shell",
                                  "Solid", "CompSolid", "Compound", "None"};
    return names[static_cast<int>(v)];
}

}  // namespace build123d
