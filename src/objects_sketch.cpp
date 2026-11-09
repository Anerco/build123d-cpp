#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

#include "build123d/objects.hpp"
#include "build123d/text.hpp"
#include "internal.hpp"
#include "py_compat.hpp"

namespace build123d {

using namespace detail;

namespace {

Face slot_face(double half_length, double height) {
    const Wire spine(std::vector<Edge>{Edge::make_line(Vector(-half_length, 0, 0), Vector(0, 0, 0)),
                                       Edge::make_line(Vector(0, 0, 0), Vector(half_length, 0, 0))});
    return Face(spine.offset_2d(height / 2));
}

Face circle_face(double radius) { return Face(Wire::make_circle(radius)); }

// ---- triangle solver: a port of trianglesolver.solve (the package build123d uses), with
// sides a, b, c opposite angles A, B, C in radians
struct Tri {
    double a, b, c, A, B, C;
};

/// Python's assert (AssertionError).
void tri_assert(bool condition, const char* message = "") {
    if (!condition) throw std::invalid_argument(message);
}

/// math.acos, math.asin and math.sqrt raise for arguments outside their domain.
double py_acos(double x) {
    if (x < -1.0 || x > 1.0) throw std::invalid_argument("math domain error");
    return std::acos(x);
}
double py_asin(double x) {
    if (x < -1.0 || x > 1.0) throw std::invalid_argument("math domain error");
    return std::asin(x);
}
double py_sqrt(double x) {
    if (x < 0.0) throw std::invalid_argument("math domain error");
    return std::sqrt(x);
}
double sq(double x) { return std::pow(x, 2.0); }  // Python's x**2

/// (d, e, f, D, E, F) from three angles and side f.
Tri aaas(double D, double E, double F, double f) {
    const double d = f * std::sin(D) / std::sin(F);
    const double e = f * std::sin(E) / std::sin(F);
    return {d, e, f, D, E, F};
}

Tri sss(double d, double e, double f) {
    tri_assert(d + e > f && e + f > d && f + d > e);
    const double F = py_acos((sq(d) + sq(e) - sq(f)) / (2 * d * e));
    const double E = py_acos((sq(d) + sq(f) - sq(e)) / (2 * d * f));
    const double D = PI - F - E;
    return {d, e, f, D, E, F};
}

Tri sas(double d, double e, double F) { return sss(d, e, py_sqrt(sq(d) + sq(e) - 2 * d * e * std::cos(F))); }

/// Two sides and the angle opposite the first (ssa_flag 'forbid': an ambiguous case raises).
Tri ssa(double d, double e, double D) {
    const double sinE = std::sin(D) * e / d;
    double E;
    if (std::abs(sinE - 1) <= 1e-9 * std::max(std::abs(sinE), 1.0)) {  // math.isclose(sinE, 1)
        E = PI / 2;  // a right triangle: the solution is unique
    } else {
        tri_assert(sinE < 1, "No such triangle");
        const double E_acute = py_asin(sinE);
        const double E_obtuse = PI - E_acute;
        const bool acute_is_valid = 0 < (PI - D - E_acute) && (PI - D - E_acute) < PI;
        const bool obtuse_is_valid = 0 < (PI - D - E_obtuse) && (PI - D - E_obtuse) < PI;
        if (acute_is_valid && obtuse_is_valid) throw std::invalid_argument("Two different triangles fit this description");
        if (!acute_is_valid && !obtuse_is_valid) throw std::invalid_argument("No such triangle");
        E = acute_is_valid ? E_acute : E_obtuse;
    }
    const double F = PI - D - E;
    const Tri t = aaas(E, F, D, d);  // returns (e, f, d, E, F, D)
    return {t.c, t.a, t.b, t.C, t.A, t.B};
}

Tri solve_triangle(const TriangleParams& p) {
    const auto rad = [](std::optional<double> d) -> std::optional<double> {
        return d ? std::optional<double>(*d * DEG2RAD) : std::nullopt;
    };
    const std::optional<double> a = p.a, b = p.b, c = p.c;
    std::optional<double> A = rad(p.A), B = rad(p.B), C = rad(p.C);
    const int sides = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
    const int angles = (A ? 1 : 0) + (B ? 1 : 0) + (C ? 1 : 0);
    // build123d's check, then trianglesolver's
    if (sides == 0 || sides + angles != 3) throw std::invalid_argument("One length and two other values must be provided");
    for (const auto& x : {a, b, c, A, B, C}) tri_assert(!x || *x > 0);
    for (const auto& x : {A, B, C}) tri_assert(!x || *x < PI);

    if (sides == 3) return sss(*a, *b, *c);
    if (sides == 2) {
        // ssa cases
        if (a && A && b) return ssa(*a, *b, *A);
        if (a && A && c) {
            const Tri t = ssa(*a, *c, *A);  // (a, c, b, A, C, B)
            return {t.a, t.c, t.b, t.A, t.C, t.B};
        }
        if (b && B && a) {
            const Tri t = ssa(*b, *a, *B);  // (b, a, c, B, A, C)
            return {t.b, t.a, t.c, t.B, t.A, t.C};
        }
        if (b && B && c) {
            const Tri t = ssa(*b, *c, *B);  // (b, c, a, B, C, A)
            return {t.c, t.a, t.b, t.C, t.A, t.B};
        }
        if (c && C && a) {
            const Tri t = ssa(*c, *a, *C);  // (c, a, b, C, A, B)
            return {t.b, t.c, t.a, t.B, t.C, t.A};
        }
        if (c && C && b) {
            const Tri t = ssa(*c, *b, *C);  // (c, b, a, C, B, A)
            return {t.c, t.b, t.a, t.C, t.B, t.A};
        }
        // sas cases
        if (a && b && C) return sas(*a, *b, *C);
        if (b && c && A) {
            const Tri t = sas(*b, *c, *A);  // (b, c, a, B, C, A)
            return {t.c, t.a, t.b, t.C, t.A, t.B};
        }
        if (c && a && B) {
            const Tri t = sas(*c, *a, *B);  // (c, a, b, C, A, B)
            return {t.b, t.c, t.a, t.B, t.C, t.A};
        }
        throw std::invalid_argument("Oops, this code should never run");
    }
    // one side and two angles: find the third angle, then solve
    if (!A) {
        A = PI - *B - *C;
    } else if (!B) {
        B = PI - *A - *C;
    } else {
        C = PI - *A - *B;
    }
    tri_assert(*A > 0 && *B > 0 && *C > 0);
    if (c) return aaas(*A, *B, *C, *c);
    if (a) {
        const Tri t = aaas(*B, *C, *A, *a);  // (b, c, a, B, C, A)
        return {t.c, t.a, t.b, t.C, t.A, t.B};
    }
    const Tri t = aaas(*C, *A, *B, *b);  // (c, a, b, C, A, B)
    return {t.b, t.c, t.a, t.B, t.C, t.A};
}

}  // namespace

Circle::Circle(double r, double arc, Align2 align_in)
    : BaseSketchObject(arc == 360.0 ? circle_face(r)
                                    : Face::revolve(Edge::make_line(Vector(r, 0), Vector(0, 0)), arc, Axis::Z),
                       0.0, align_in),
      radius(r),
      arc_size(arc),
      align(align_in) {}

Circle::Circle(double r, Align2 align) : Circle(r, 360.0, align) {}

Ellipse::Ellipse(double xr, double yr, double rotation, Align2 align_in)
    : BaseSketchObject(Face(Wire::make_ellipse(xr, yr)), rotation, align_in),
      x_radius(xr),
      y_radius(yr),
      align(align_in) {}

Ellipse::Ellipse(double xr, double yr, Align2 align) : Ellipse(xr, yr, 0.0, align) {}

Rectangle::Rectangle(double w, double h, double rotation, Align2 align_in)
    : BaseSketchObject(Face::make_rect(w, h), rotation, align_in), width(w), rectangle_height(h), align(align_in) {}

Rectangle::Rectangle(double w, double h, Align2 align) : Rectangle(w, h, 0.0, align) {}

namespace {
Face rounded_rect(double w, double h, double r) {
    if (w <= 2 * r || h <= 2 * r) throw std::invalid_argument("width and height must be > 2*radius");
    const Face face = Face::make_rect(w, h);
    const auto v = face.vertices();
    return face.fillet_2d(r, std::vector<Vertex>(v.begin(), v.end()));
}
}  // namespace

RectangleRounded::RectangleRounded(double w, double h, double r, double rotation, Align2 align_in)
    : BaseSketchObject(rounded_rect(w, h, r), rotation, align_in),
      width(w),
      rectangle_height(h),
      radius(r),
      align(align_in) {}

RectangleRounded::RectangleRounded(double w, double h, double r, Align2 align)
    : RectangleRounded(w, h, r, 0.0, align) {}

Polygon::Polygon(const std::vector<Vector>& pts_in, double rotation, Align2 align_in)
    : BaseSketchObject(Face(Wire::make_polygon(pts_in)), rotation, align_in), pts(pts_in), align(align_in) {}

Polygon::Polygon(const std::vector<Vector>& pts, Align2 align) : Polygon(pts, 0.0, align) {}

namespace {
Face regular_polygon(double rad, int side_count, double rotation, Align2 align) {
    std::vector<Vector> pts;
    for (int i = 0; i <= side_count; ++i) {
        const double a = i * 2 * PI / side_count + rotation * DEG2RAD;
        pts.emplace_back(rad * std::cos(a), rad * std::sin(a));
    }
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    for (const auto& p : pts) {
        xmin = std::min(xmin, p.X);
        xmax = std::max(xmax, p.X);
        ymin = std::min(ymin, p.Y);
        ymax = std::max(ymax, p.Y);
    }
    const Vector offset =
        to_align_offset(Vector(xmin, ymin), Vector(xmax, ymax), Align3(align, Align::NONE), Vector(0, 0, 0));
    for (auto& p : pts) p += offset;
    return Face(Wire::make_polygon(pts));
}
}  // namespace

RegularPolygon::RegularPolygon(double r, int sides, bool major_radius, double rotation, Align2 align_in)
    : align(align_in) {
    if (sides < 3) throw std::invalid_argument("RegularPolygon must have at least three sides, not " + std::to_string(sides));
    radius = major_radius ? r : r / std::cos(PI / sides);
    apothem = radius * std::cos(PI / sides);
    side_count = sides;
    assign(regular_polygon(radius, sides, rotation, align_in), 0.0, std::nullopt);
}

RegularPolygon::RegularPolygon(double r, int sides, Align2 align) : RegularPolygon(r, sides, true, 0.0, align) {}

namespace {
Face trapezoid(double width, double height, double left, std::optional<double> right_in) {
    const double right = (right_in && *right_in != 0.0) ? *right_in : left;
    const double reduction_left = left == 90 ? 0 : height / std::tan(left * DEG2RAD);
    const double reduction_right = right == 90 ? 0 : height / std::tan(right * DEG2RAD);
    double top_left = width / 2, top_right = width / 2, bot_left = width / 2, bot_right = width / 2;
    if (reduction_left > 0) {
        top_left -= reduction_left;
    } else {
        bot_left += reduction_left;
    }
    if (reduction_right > 0) {
        top_right -= reduction_right;
    } else {
        bot_right += reduction_right;
    }
    if (bot_left + bot_right < 0) throw std::invalid_argument("Trapezoid bottom invalid - change angles");
    if (top_left + top_right < 0) throw std::invalid_argument("Trapezoid top invalid - change angles");
    return Face(Wire::make_polygon({Vector(-bot_left, -height / 2), Vector(bot_right, -height / 2),
                                    Vector(top_right, height / 2), Vector(-top_left, height / 2),
                                    Vector(-bot_left, -height / 2)}));
}
}  // namespace

Trapezoid::Trapezoid(double width_in, double height, double left, std::optional<double> right, double rotation,
                     Align2 align_in)
    : BaseSketchObject(trapezoid(width_in, height, left, right), rotation, align_in),
      width(width_in),
      trapezoid_height(height),
      left_side_angle(left),
      right_side_angle((right && *right != 0.0) ? *right : left),
      align(align_in) {}

Trapezoid::Trapezoid(double width, double height, double left_side_angle, Align2 align)
    : Trapezoid(width, height, left_side_angle, std::nullopt, 0.0, align) {}

Triangle::Triangle(const TriangleParams& params, Align2 align, double rotation) {
    const Tri t = solve_triangle(params);
    a = t.a;
    b = t.b;
    c = t.c;
    A = t.A * RAD2DEG;
    B = t.B * RAD2DEG;
    C = t.C * RAD2DEG;
    Face triangle(Wire::make_polygon({Vector(0, 0), Vector(a, 0), Vector(c, 0).rotate(Axis::Z, B)}));
    Vector centroid(0, 0, 0);
    for (const auto& v : triangle.vertices()) centroid += Vector(v);
    triangle.move(Location(-(centroid / 3.0)));
    const bool has_align = !(align.x == Align::NONE && align.y == Align::NONE);
    assign(triangle, rotation, has_align ? std::optional<Align2>(align) : std::nullopt);

    const auto es = edges();
    auto find_edge = [&](double len, const std::vector<Edge>& exclude) {
        for (const auto& e : es) {
            const bool excluded = std::any_of(exclude.begin(), exclude.end(), [&](const Edge& x) { return x.is_same(e); });
            if (!excluded && (len < 0 || std::abs(e.length() - len) < TOLERANCE)) return e;
        }
        throw std::runtime_error("Triangle: unable to identify edges");
    };
    edge_a = find_edge(a, {});
    edge_b = find_edge(b, {edge_a});
    edge_c = find_edge(-1, {edge_a, edge_b});
    auto common_vertex = [](const Edge& e1, const Edge& e2) {
        for (const auto& v1 : e1.vertices()) {
            for (const auto& v2 : e2.vertices()) {
                if (v1.is_same(v2)) return v1;
            }
        }
        throw std::runtime_error("Triangle: edges share no vertex");
    };
    vertex_A = common_vertex(edge_b, edge_c);
    vertex_B = common_vertex(edge_a, edge_c);
    vertex_C = common_vertex(edge_a, edge_b);
}

SlotArc::SlotArc(const Shape& arc_in, double height, double rotation)
    : BaseSketchObject(Face(as_wire(arc_in).offset_2d(height / 2)), rotation, std::nullopt),
      arc(arc_in),
      slot_height(height) {}

namespace {
Face slot_center_point(const Vector& center, const Vector& point, double height) {
    const Vector half_line = point - center;
    if (half_line.length() <= 0) {
        throw std::invalid_argument("Distance between center and point must be greater than 0");
    }
    const auto wires = Wire::combine(
        std::vector<Edge>{Edge::make_line(point, center), Edge::make_line(center, center - half_line)});
    return Face(wires.at(0).offset_2d(height / 2));
}
}  // namespace

SlotCenterPoint::SlotCenterPoint(const Vector& center, const Vector& point_in, double height, double rotation)
    : BaseSketchObject(slot_center_point(center, point_in, height), rotation, std::nullopt),
      slot_center(center),
      point(point_in),
      slot_height(height) {}

namespace {
Face slot_center_to_center(double center_separation, double height) {
    if (center_separation < 0) {
        throw std::invalid_argument("Requires center_separation > 0. Got: center_separation=" +
                                    std::to_string(center_separation));
    }
    return center_separation > 0 ? slot_face(center_separation / 2, height) : circle_face(height / 2);
}
}  // namespace

SlotCenterToCenter::SlotCenterToCenter(double separation, double height, double rotation)
    : BaseSketchObject(slot_center_to_center(separation, height), rotation, std::nullopt),
      center_separation(separation),
      slot_height(height) {}

namespace {
Face slot_overall(double width, double height) {
    if (width < height) throw std::invalid_argument("Slot requires that width > height");
    return width > height ? slot_face(width / 2 - height / 2, height) : circle_face(width / 2);
}
}  // namespace

SlotOverall::SlotOverall(double width_in, double height, double rotation, Align2 align)
    : BaseSketchObject(slot_overall(width_in, height), rotation, align), width(width_in), slot_height(height) {}

SlotOverall::SlotOverall(double width, double height, Align2 align) : SlotOverall(width, height, 0.0, align) {}

namespace {
Face superellipse(double width, double height, double order, int point_count) {
    if (order <= 0) throw std::invalid_argument("Order must be greater than zero.");
    Edge top_right;
    if (order == 1) {
        top_right = Edge::make_line(Vector(width / 2, 0), Vector(0, height / 2));
    } else if (order == 2) {
        top_right = Edge::make_ellipse(width / 2, height / 2, Plane::XY, 0, 90);
    } else {
        std::vector<Vector> pts{Vector(width / 2, 0.0)};
        for (int i = 1; i < point_count - 1; ++i) {
            const double t = i * PI / point_count / 2;
            const double ct = std::cos(t), st = std::sin(t);
            pts.emplace_back(std::pow(std::abs(ct), 2 / order) * width * std::copysign(1.0, ct) / 2,
                             std::pow(std::abs(st), 2 / order) * height * std::copysign(1.0, st) / 2);
        }
        pts.emplace_back(0.0, height / 2);
        top_right = Edge::make_spline(pts);
    }
    const Edge top_left = top_right.mirror(Plane::YZ).reversed();
    return Face(Wire(std::vector<Edge>{top_right, top_left, top_left.mirror(Plane::XZ).reversed(),
                                       top_right.mirror(Plane::XZ).reversed()}));
}
}  // namespace

Superellipse::Superellipse(double width_in, double height, double order_in, int point_count_in, double rotation,
                           Align2 align_in)
    : BaseSketchObject(superellipse(width_in, height, order_in, point_count_in), rotation, align_in),
      width(width_in),
      height_(height),
      order(order_in),
      point_count(point_count_in),
      align(align_in) {}

namespace {

/// Text's single_line_width: 4% of the font size by default, else above 0.
double text_line_width(double font_size, std::optional<double> single_line_width) {
    if (!single_line_width) return 0.04 * font_size;
    if (*single_line_width <= 0.0) {
        throw std::invalid_argument("single_line_width (" + detail::py_float_repr(*single_line_width) +
                                    ") must be greater than 0");
    }
    return *single_line_width;
}

}  // namespace

Text::Text(const std::string& text, double size, const TextOptions& o)
    : BaseSketchObject(Compound::make_text(text, size, o.font, o.font_path, o.font_style, o.text_align, o.align,
                                           o.position_on_path, o.path, text_line_width(size, o.single_line_width)),
                       o.rotation, std::nullopt),
      txt(text),
      font_size(size),
      font(o.font),
      font_path(o.font_path),
      font_style(o.font_style),
      text_align(o.text_align),
      align(o.align),
      text_path(o.path),
      position_on_path(o.position_on_path),
      single_line_width(text_line_width(size, o.single_line_width)) {
    // The font that was used
    std::tie(font, font_path, std::ignore) = Compound::resolve_font(o.font, o.font_path, o.font_style);
}

}  // namespace build123d
