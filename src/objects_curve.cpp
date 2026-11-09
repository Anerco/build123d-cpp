#include <cmath>

#include "build123d/objects.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

const std::vector<Vector>& require_count(const std::vector<Vector>& pts, std::size_t n, const char* what) {
    if (pts.size() != n) throw std::invalid_argument(what);
    return pts;
}

double py_mod(double a, double b) {
    double r = std::fmod(a, b);
    if (r < 0) r += b;
    return r;
}

Edge jern_arc(const Vector& start, const Vector& tangent, double radius, double arc_size) {
    const Vector start_tangent = tangent.normalized();
    Vector arc_normal(0, 0, 1);
    if (std::abs(start_tangent.dot(arc_normal)) > TOLERANCE) {
        arc_normal = start_tangent.cross(arc_normal);
        if (arc_normal.length() <= TOLERANCE) arc_normal = start_tangent.cross(Vector(1, 0, 0));
        arc_normal = arc_normal.normalized();
    }
    const double direction = std::copysign(1.0, arc_size);
    const Axis axis(start, arc_normal);
    const Vector center = start + start_tangent.rotate(axis, direction * 90) * std::abs(radius);
    const Vector end_of_arc = center + (start - center).rotate(axis, arc_size);
    if (std::abs(arc_size) >= 360) {
        return Edge::make_circle(radius, Plane(center, start - center, arc_normal));
    }
    return Edge::make_tangent_arc(start, start_tangent, end_of_arc);
}

Edge sagitta_arc(const Vector& start, const Vector& end, double sagitta) {
    const Vector mid = (end + start) * 0.5;
    const Vector sv = ((end - start).normalized() * std::abs(sagitta)).rotate(Axis::Z, sagitta > 0 ? 90 : -90);
    return Edge::make_three_point_arc(start, mid + sv, end);
}

Edge polar_line(const Vector& start, double length, double angle, const Vector& dir, LengthMode mode) {
    Vector length_vector;
    switch (mode) {
        case LengthMode::DIAGONAL: length_vector = dir * length; break;
        case LengthMode::HORIZONTAL: length_vector = dir * std::abs(length / std::cos(angle * DEG2RAD)); break;
        case LengthMode::VERTICAL: length_vector = dir * std::abs(length / std::sin(angle * DEG2RAD)); break;
    }
    return Edge::make_line(start, start + length_vector);
}

/// The fillet arc's endpoints ordered (on the incoming segment, on the outgoing segment).
std::pair<Vector, Vector> arc_ends(const Edge& arc, const Edge& incoming) {
    const Vector p0 = arc.position_at(0), p1 = arc.position_at(1);
    if (incoming.distance_to(p0) <= incoming.distance_to(p1)) return {p0, p1};
    return {p1, p0};
}

Wire fillet_polyline(std::vector<Vector> points, const std::vector<double>* radii, double radius, bool close) {
    if (points.size() >= 2 && (points.front() - points.back()).length() < TOLERANCE) {
        close = true;
        points.pop_back();
    }
    if (points.size() < 2) throw std::invalid_argument("FilletPolyline requires two or more pts");
    std::vector<double> radius_list;
    if (radii == nullptr) {
        radius_list.assign(points.size(), radius);
    } else {
        radius_list = *radii;
        const std::size_t expected = points.size() - (close ? 0 : 2);
        if (radius_list.size() != expected) {
            throw std::invalid_argument("radius list length (" + std::to_string(radius_list.size()) +
                                        ") must match angle count (" + std::to_string(expected) + ")");
        }
    }
    for (double r : radius_list) {
        if (r < 0) throw std::invalid_argument("radius " + std::to_string(r) + " must be non-negative");
    }
    std::vector<Edge> lines;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) lines.push_back(Edge::make_line(points[i], points[i + 1]));
    if (close && (lines.front().position_at(0) - lines.back().position_at(1)).length() > 1e-5) {
        lines.push_back(Edge::make_line(lines.back().position_at(1), lines.front().position_at(0)));
    }
    const std::size_t n = points.size();
    if (!close && n == 2) return Wire(lines);

    // Fillet arc (or nothing for radius 0) at each corner, with its incoming/outgoing segments.
    struct Corner {
        std::optional<Edge> arc;
        Vector start, end;  // where the arc meets the incoming/outgoing segment
        Vector vertex;
    };
    std::vector<Corner> corners;
    const std::size_t first = close ? 0 : 1, last = close ? n : n - 1;
    for (std::size_t i = first; i < last; ++i) {
        const Vector prev = points[(i + n - 1) % n], here = points[i], next = points[(i + 1) % n];
        const double r = radius_list[close ? i : i - 1];
        Corner c{std::nullopt, here, here, here};
        if (r != 0.0) {
            const Edge in = Edge::make_line(prev, here), out = Edge::make_line(here, next);
            const Face triangle(Wire(std::vector<Edge>{in, out, Edge::make_line(next, prev)}));
            // the face's own corner vertex (Face::fillet_2d matches vertices by identity)
            const Vertex corner_vertex = triangle.vertices().sort_by_distance(here)[0];
            const Face filleted = triangle.fillet_2d(r, {corner_vertex});
            const auto arcs = filleted.edges().filter_by(GeomType::CIRCLE);
            if (arcs.empty()) throw std::invalid_argument("FilletPolyline: fillet failed, try a smaller radius");
            const auto ends = arc_ends(arcs[0], in);
            c.arc = arcs[0];
            c.start = ends.first;
            c.end = ends.second;
        }
        corners.push_back(c);
    }
    std::vector<Edge> edges;
    if (close) {
        for (std::size_t i = 0; i < corners.size(); ++i) {
            const Corner& prev = corners[(i + corners.size() - 1) % corners.size()];
            edges.push_back(Edge::make_line(prev.end, corners[i].start));
        }
    } else {
        edges.push_back(Edge::make_line(points.front(), corners.front().start));
        for (std::size_t i = 0; i + 1 < corners.size(); ++i) {
            edges.push_back(Edge::make_line(corners[i].end, corners[i + 1].start));
        }
        edges.push_back(Edge::make_line(corners.back().end, points.back()));
    }
    for (const auto& c : corners) {
        if (c.arc) edges.push_back(*c.arc);
    }
    return Wire(edges);
}

}  // namespace

Line::Line(const Vector& start, const Vector& end) : Edge(Edge::make_line(start, end)) {}

Line::Line(const std::vector<Vector>& pts)
    : Line(require_count(pts, 2, "Line requires two pts")[0], pts[1]) {}

Polyline::Polyline(const std::vector<Vector>& pts, bool close) {
    if (pts.size() < 2) throw std::invalid_argument("Polyline requires two or more pts");
    std::vector<Edge> edges;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) edges.push_back(Edge::make_line(pts[i], pts[i + 1]));
    if (close && (edges.front().position_at(0) - edges.back().position_at(1)).length() > 1e-5) {
        edges.push_back(Edge::make_line(edges.back().position_at(1), edges.front().position_at(0)));
    }
    obj_ = Wire::combine(edges).at(0).wrapped();
}

Spline::Spline(const std::vector<Vector>& pts, const std::vector<Vector>& tangents,
               const std::vector<double>& tangent_scalars, bool periodic) {
    std::vector<Vector> scaled;
    if (!tangents.empty()) {
        std::vector<double> scalars = tangent_scalars;
        if (scalars.empty()) scalars.assign(tangents.size(), 1.0);
        for (std::size_t i = 0; i < tangents.size() && i < scalars.size(); ++i) {
            scaled.push_back(tangents[i].normalized() * scalars[i]);
        }
    }
    obj_ = Edge::make_spline(pts, scaled, periodic, {}, tangent_scalars.empty()).wrapped();
}

Bezier::Bezier(const std::vector<Vector>& cntl_pnts, const std::vector<double>& weights)
    : Edge(Edge::make_bezier(cntl_pnts, weights)) {}

CenterArc::CenterArc(const Vector& center, double radius, double start_angle, double arc_size) {
    const AngularDirection direction =
        arc_size > 0 ? AngularDirection::COUNTER_CLOCKWISE : AngularDirection::CLOCKWISE;
    const double normalized = py_mod(arc_size + 360.0, 360.0);
    const double end_angle = start_angle + normalized;
    const double start = normalized == 360.0 ? end_angle : start_angle;
    obj_ = Edge::make_circle(radius, Plane(center), start, end_angle, direction).wrapped();
}

ThreePointArc::ThreePointArc(const Vector& point1, const Vector& point2, const Vector& point3)
    : Edge(Edge::make_three_point_arc(point1, point2, point3)) {}

ThreePointArc::ThreePointArc(const std::vector<Vector>& pts)
    : ThreePointArc(require_count(pts, 3, "ThreePointArc requires three points")[0], pts[1], pts[2]) {}

RadiusArc::RadiusArc(const Vector& start_point, const Vector& end_point, double radius, bool short_sagitta) {
    const double length = (end_point - start_point).length() / 2.0;
    const double disc = radius * radius - length * length;
    if (disc < 0) throw std::invalid_argument("Arc radius is not large enough to reach the end point.");
    const double sagitta =
        short_sagitta ? std::abs(radius) - std::sqrt(disc) : -std::abs(radius) - std::sqrt(disc);
    obj_ = sagitta_arc(start_point, end_point, radius > 0 ? sagitta : -sagitta).wrapped();
}

SagittaArc::SagittaArc(const Vector& start_point, const Vector& end_point, double sagitta)
    : Edge(sagitta_arc(start_point, end_point, sagitta)) {}

TangentArc::TangentArc(const Vector& start, const Vector& end, const Vector& tangent, bool tangent_from_first)
    : Edge(tangent_from_first ? Edge::make_tangent_arc(start, tangent.normalized(), end)
                              : Edge::make_tangent_arc(end, tangent.normalized(), start)) {}

TangentArc::TangentArc(const std::vector<Vector>& pts, const Vector& tangent, bool tangent_from_first)
    : TangentArc(require_count(pts, 2, "tangent_arc requires two points")[0], pts[1], tangent,
                 tangent_from_first) {}

JernArc::JernArc(const Vector& start_in, const Vector& tangent, double radius, double arc_size)
    : Edge(jern_arc(start_in, tangent, radius, arc_size)), start(start_in) {
    center_point = arc_center();
    end_of_arc = position_at(1);
}

EllipticalCenterArc::EllipticalCenterArc(const Vector& center, double x_radius, double y_radius,
                                         double start_angle, double arc_size, double rotation) {
    const AngularDirection direction =
        arc_size >= 0 ? AngularDirection::COUNTER_CLOCKWISE : AngularDirection::CLOCKWISE;
    obj_ = Edge::make_ellipse(x_radius, y_radius, Plane::XY, start_angle, start_angle + arc_size, direction)
               .rotate(Axis::Z, rotation)
               .translate(center)
               .wrapped();
}

BSpline::BSpline(const std::vector<Vector>& control_points, const std::vector<double>& knots, int degree,
                 const std::vector<double>& weights, bool periodic)
    : Edge(Edge::make_bspline(control_points, knots, degree, weights, periodic)) {}

EllipticalStartArc::EllipticalStartArc(const Vector& start_pnt, const Vector& start_tangent, double x_radius,
                                       double y_radius, double arc_size, std::optional<double> start_angle,
                                       std::optional<Vector> major_axis_dir) {
    const auto proj_to_plane = [](const Vector& v, const Vector& normal) {
        const Vector n = normal.normalized();
        return v - n * v.dot(n);
    };
    const Vector tangent = start_tangent.normalized();
    const Vector pln_normal(0, 0, 1);
    Vector pln_x_dir, pln_origin;
    double start_degrees;
    if (start_angle) {
        const double angle = *start_angle * DEG2RAD;
        const Vector pln_tangent = proj_to_plane(tangent, pln_normal).normalized();
        const double a_radius = -x_radius * std::sin(angle);
        const double b_radius = y_radius * std::cos(angle);
        const Vector x_dir = (pln_tangent * a_radius - pln_normal.cross(pln_tangent) * b_radius) /
                             (a_radius * a_radius + b_radius * b_radius);
        pln_x_dir = x_dir.normalized();
        const Vector pln_y_dir = pln_normal.cross(pln_x_dir);
        pln_origin = start_pnt - pln_x_dir * (x_radius * std::cos(angle)) - pln_y_dir * (y_radius * std::sin(angle));
        start_degrees = *start_angle;
    } else if (major_axis_dir) {
        pln_x_dir = proj_to_plane(*major_axis_dir, pln_normal).normalized();
        const Vector pln_y_dir = pln_normal.cross(pln_x_dir);
        const Vector pln_tangent = proj_to_plane(tangent, pln_normal);
        const double pln_x_radius = pln_tangent.dot(pln_x_dir);
        const double pln_y_radius = pln_tangent.dot(pln_y_dir);
        const double angle = std::atan2(-(pln_x_radius / x_radius), pln_y_radius / y_radius);
        pln_origin = start_pnt - pln_x_dir * (x_radius * std::cos(angle)) - pln_y_dir * (y_radius * std::sin(angle));
        start_degrees = angle * RAD2DEG;
    } else {
        throw std::invalid_argument("Either start_angle or major_axis_dir must be provided");
    }
    const Plane pln(pln_origin, pln_x_dir, pln_normal);
    const AngularDirection direction =
        arc_size >= 0 ? AngularDirection::COUNTER_CLOCKWISE : AngularDirection::CLOCKWISE;
    obj_ = Edge::make_ellipse(x_radius, y_radius, pln, start_degrees, start_degrees + arc_size, direction).wrapped();
}

ParabolicCenterArc::ParabolicCenterArc(const Vector& vertex, double focal_length, double start_angle,
                                       double arc_size, double rotation) {
    const AngularDirection direction =
        arc_size >= 0 ? AngularDirection::COUNTER_CLOCKWISE : AngularDirection::CLOCKWISE;
    obj_ = Edge::make_parabola(focal_length, Plane::XY, start_angle, start_angle + arc_size, direction)
               .rotate(Axis::Z, rotation)
               .translate(vertex)
               .wrapped();
}

HyperbolicCenterArc::HyperbolicCenterArc(const Vector& center, double x_radius, double y_radius,
                                         double start_angle, double arc_size, double rotation) {
    const AngularDirection direction =
        arc_size >= 0 ? AngularDirection::COUNTER_CLOCKWISE : AngularDirection::CLOCKWISE;
    obj_ = Edge::make_hyperbola(x_radius, y_radius, Plane::XY, start_angle, start_angle + arc_size, direction)
               .rotate(Axis::Z, rotation)
               .translate(center)
               .wrapped();
}

Helix::Helix(double pitch, double height, double radius, const Vector& center, const Vector& direction,
             double cone_angle, bool lefthand)
    : Edge(Edge::make_helix(pitch, height, radius, center, direction, cone_angle, lefthand)) {}

PolarLine::PolarLine(const Vector& start, double length, double angle, LengthMode length_mode)
    : Edge(polar_line(start, length, angle, Vector(1, 0, 0).rotate(Axis::Z, angle), length_mode)) {}

PolarLine::PolarLine(const Vector& start, double length, const Vector& direction, LengthMode length_mode)
    : Edge(polar_line(start, length, Vector(1, 0, 0).get_angle(direction.normalized()), direction.normalized(),
                      length_mode)) {}

FilletPolyline::FilletPolyline(const std::vector<Vector>& pts, double radius, bool close)
    : Wire(fillet_polyline(pts, nullptr, radius, close)) {}

FilletPolyline::FilletPolyline(const std::vector<Vector>& pts, const std::vector<double>& radius, bool close)
    : Wire(fillet_polyline(pts, &radius, 0.0, close)) {}

IntersectingLine::IntersectingLine(const Vector& start, const Vector& direction, const Shape& other) {
    const Vector dir = direction.normalized();
    const Axis axis(start, dir);
    double best = -1.0;
    for (const auto& edge : other.edges()) {
        for (const auto& p : edge.find_intersection_points(axis)) {
            const double along = (p - start).dot(dir);
            const double dist = (start - p).length();
            if (along < -TOLERANCE || dist <= TOLERANCE) continue;
            if (best < 0 || dist < best) best = dist;
        }
    }
    if (best < 0) throw std::invalid_argument("No intersections found");
    obj_ = Edge::make_line(start, start + dir * best).wrapped();
}

}  // namespace build123d
