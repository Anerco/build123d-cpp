#include <algorithm>
#include <array>
#include <cmath>

#include "build123d/objects.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

/// BaseSketchObject: align by bounding box, rotate about Z and collect the faces.
TopoDS_Shape base_sketch(Shape obj, double rotation, std::optional<Align2> align) {
    if (align) obj.move(Location(obj.bounding_box().to_align_offset(*align)));
    const ShapeList<Face> faces = rotation != 0.0 ? obj.moved(Rotation(0, 0, rotation)).faces() : obj.faces();
    std::vector<TopoDS_Shape> topods;
    for (const auto& f : faces) topods.push_back(f.wrapped());
    return make_compound(topods);
}

Face slot_face(double half_length, double height) {
    const Wire spine(std::vector<Edge>{Edge::make_line(Vector(-half_length, 0, 0), Vector(0, 0, 0)),
                                       Edge::make_line(Vector(0, 0, 0), Vector(half_length, 0, 0))});
    return Face(spine.offset_2d(height / 2));
}

Face circle_face(double radius) { return Face(Wire::make_circle(radius)); }

// ---- triangle solver (sides a, b, c opposite angles A, B, C; radians)
struct Tri {
    double a, b, c, A, B, C;
};

double clamp1(double x) { return std::max(-1.0, std::min(1.0, x)); }

Tri solve_sss(double a, double b, double c) {
    if (a + b <= c || a + c <= b || b + c <= a) throw std::invalid_argument("Triangle sides violate the triangle inequality");
    const double A = std::acos(clamp1((b * b + c * c - a * a) / (2 * b * c)));
    const double B = std::acos(clamp1((a * a + c * c - b * b) / (2 * a * c)));
    return {a, b, c, A, B, PI - A - B};
}

Tri solve_triangle(const TriangleParams& p) {
    const auto rad = [](std::optional<double> d) -> std::optional<double> {
        return d ? std::optional<double>(*d * DEG2RAD) : std::nullopt;
    };
    std::optional<double> s[3] = {p.a, p.b, p.c};
    std::optional<double> ang[3] = {rad(p.A), rad(p.B), rad(p.C)};
    int sides = 0, angles = 0;
    for (int i = 0; i < 3; ++i) {
        sides += s[i] ? 1 : 0;
        angles += ang[i] ? 1 : 0;
    }
    if (sides == 0 || sides + angles != 3) throw std::invalid_argument("One length and two other values must be provided");
    auto result = [&](std::array<double, 3> sd, std::array<double, 3> an) { return Tri{sd[0], sd[1], sd[2], an[0], an[1], an[2]}; };
    if (sides == 3) return solve_sss(*s[0], *s[1], *s[2]);
    if (sides == 2) {
        int missing = 0;
        while (s[missing]) ++missing;
        const int i = (missing + 1) % 3, j = (missing + 2) % 3;
        if (ang[missing]) {
            // SAS: the included angle is opposite the missing side
            const double x = std::sqrt(*s[i] * *s[i] + *s[j] * *s[j] - 2 * *s[i] * *s[j] * std::cos(*ang[missing]));
            std::array<double, 3> sd{};
            sd[missing] = x;
            sd[i] = *s[i];
            sd[j] = *s[j];
            return solve_sss(sd[0], sd[1], sd[2]);
        }
        // SSA: one known angle opposite a known side
        const int k = ang[i] ? i : j;     // known angle index (side k known)
        const int other = k == i ? j : i;  // the other known side
        const double sin_other = *s[other] * std::sin(*ang[k]) / *s[k];
        if (sin_other > 1.0 + 1e-12) throw std::invalid_argument("No triangle satisfies the given values");
        std::array<double, 3> an{};
        an[k] = *ang[k];
        an[other] = std::asin(clamp1(sin_other));
        an[missing] = PI - an[k] - an[other];
        if (an[missing] <= 0) throw std::invalid_argument("No triangle satisfies the given values");
        std::array<double, 3> sd{};
        sd[k] = *s[k];
        sd[other] = *s[other];
        sd[missing] = *s[k] * std::sin(an[missing]) / std::sin(an[k]);
        return result(sd, an);
    }
    // one side and two angles
    std::array<double, 3> an{};
    int missing = -1;
    for (int i = 0; i < 3; ++i) {
        if (ang[i]) {
            an[static_cast<std::size_t>(i)] = *ang[i];
        } else {
            missing = i;
        }
    }
    an[static_cast<std::size_t>(missing)] = PI - an[(missing + 1) % 3] - an[(missing + 2) % 3];
    if (an[static_cast<std::size_t>(missing)] <= 0) throw std::invalid_argument("Angles sum to 180 degrees or more");
    int known = 0;
    while (!s[known]) ++known;
    const double ratio = *s[known] / std::sin(an[static_cast<std::size_t>(known)]);
    return result({ratio * std::sin(an[0]), ratio * std::sin(an[1]), ratio * std::sin(an[2])}, an);
}

}  // namespace

Circle::Circle(double r, double arc, Align2 align)
    : Sketch(base_sketch(arc == 360.0 ? circle_face(r)
                                      : Face::revolve(Edge::make_line(Vector(r, 0), Vector(0, 0)), arc, Axis::Z),
                         0.0, align)),
      radius(r),
      arc_size(arc) {}

Circle::Circle(double r, Align2 align) : Circle(r, 360.0, align) {}

Ellipse::Ellipse(double xr, double yr, double rotation, Align2 align)
    : Sketch(base_sketch(Face(Wire::make_ellipse(xr, yr)), rotation, align)), x_radius(xr), y_radius(yr) {}

Ellipse::Ellipse(double xr, double yr, Align2 align) : Ellipse(xr, yr, 0.0, align) {}

Rectangle::Rectangle(double w, double h, double rotation, Align2 align)
    : Sketch(base_sketch(Face::make_rect(w, h), rotation, align)), width(w), rectangle_height(h) {}

Rectangle::Rectangle(double w, double h, Align2 align) : Rectangle(w, h, 0.0, align) {}

namespace {
Face rounded_rect(double w, double h, double r) {
    if (w <= 2 * r || h <= 2 * r) throw std::invalid_argument("width and height must be > 2*radius");
    const Face face = Face::make_rect(w, h);
    const auto v = face.vertices();
    return face.fillet_2d(r, std::vector<Vertex>(v.begin(), v.end()));
}
}  // namespace

RectangleRounded::RectangleRounded(double w, double h, double r, double rotation, Align2 align)
    : Sketch(base_sketch(rounded_rect(w, h, r), rotation, align)), width(w), rectangle_height(h), radius(r) {}

RectangleRounded::RectangleRounded(double w, double h, double r, Align2 align)
    : RectangleRounded(w, h, r, 0.0, align) {}

Polygon::Polygon(const std::vector<Vector>& pts, double rotation, Align2 align)
    : Sketch(base_sketch(Face(Wire::make_polygon(pts)), rotation, align)) {}

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

RegularPolygon::RegularPolygon(double r, int sides, bool major_radius, double rotation, Align2 align)
    : Sketch() {
    if (sides < 3) throw std::invalid_argument("RegularPolygon must have at least three sides, not " + std::to_string(sides));
    radius = major_radius ? r : r / std::cos(PI / sides);
    apothem = radius * std::cos(PI / sides);
    side_count = sides;
    obj_ = base_sketch(regular_polygon(radius, sides, rotation, align), 0.0, std::nullopt);
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

Trapezoid::Trapezoid(double width, double height, double left_side_angle, std::optional<double> right_side_angle,
                     double rotation, Align2 align)
    : Sketch(base_sketch(trapezoid(width, height, left_side_angle, right_side_angle), rotation, align)) {}

Trapezoid::Trapezoid(double width, double height, double left_side_angle, Align2 align)
    : Trapezoid(width, height, left_side_angle, std::nullopt, 0.0, align) {}

Triangle::Triangle(const TriangleParams& params, Align2 align, double rotation) : Sketch() {
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
    obj_ = base_sketch(triangle, rotation, has_align ? std::optional<Align2>(align) : std::nullopt);

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

SlotArc::SlotArc(const Shape& arc, double height, double rotation)
    : Sketch(base_sketch(Face(as_wire(arc).offset_2d(height / 2)), rotation, std::nullopt)) {}

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

SlotCenterPoint::SlotCenterPoint(const Vector& center, const Vector& point, double height, double rotation)
    : Sketch(base_sketch(slot_center_point(center, point, height), rotation, std::nullopt)) {}

SlotCenterToCenter::SlotCenterToCenter(double center_separation, double height, double rotation)
    : Sketch(base_sketch(center_separation > 0 ? slot_face(center_separation / 2, height) : circle_face(height / 2),
                         rotation, std::nullopt)) {
    if (center_separation < 0) throw std::invalid_argument("Requires center_separation > 0");
}

namespace {
Face slot_overall(double width, double height) {
    if (width < height) throw std::invalid_argument("Slot requires that width > height");
    return width > height ? slot_face(width / 2 - height / 2, height) : circle_face(width / 2);
}
}  // namespace

SlotOverall::SlotOverall(double width, double height, double rotation, Align2 align)
    : Sketch(base_sketch(slot_overall(width, height), rotation, align)) {}

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

Superellipse::Superellipse(double width, double height, double order, int point_count, double rotation, Align2 align)
    : Sketch(base_sketch(superellipse(width, height, order, point_count), rotation, align)) {}

Text::Text(const std::string& text, double size, const TextOptions& o)
    : Sketch(base_sketch(Compound::make_text(text, size, o.font, o.font_path, o.font_style, o.text_align, o.align,
                                             o.position_on_path, o.path),
                         o.rotation, std::nullopt)),
      txt(text),
      font_size(size) {}

}  // namespace build123d
