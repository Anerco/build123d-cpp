// Curve objects of objects_curve.py that solve for their shape: BlendCurve,
// DoubleTangentArc, Airfoil, ConstrainedArcs and ConstrainedLines.
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

#include "build123d/objects.hpp"
#include "internal.hpp"
#include "nelder_mead.hpp"
#include "py_compat.hpp"

namespace build123d {

using namespace detail;

namespace {

Edge blend_curve(const Edge& curve0, const Edge& curve1, ContinuityLevel continuity,
                 const std::optional<std::pair<Vector, Vector>>& end_points,
                 const std::optional<std::pair<double, double>>& tangent_scalars) {
    const std::pair<double, double> tan_scalars = tangent_scalars.value_or(std::make_pair(1.0, 1.0));

    // Find the vertices that will be connected using closest if None
    std::pair<Vector, Vector> end_pnts;
    if (end_points) {
        end_pnts = *end_points;
    } else {
        double best = std::numeric_limits<double>::infinity();
        for (const auto& v0 : curve0.vertices()) {
            for (const auto& v1 : curve1.vertices()) {
                const double d = v0.distance_to(v1);
                if (d < best) {
                    best = d;
                    end_pnts = {Vector(v0), Vector(v1)};
                }
            }
        }
    }

    // Find the Edge parameter that matches the end points
    const Edge* curves[2] = {&curve0, &curve1};
    const Vector given[2] = {end_pnts.first, end_pnts.second};
    double end_params[2] = {0, 0};
    for (int i = 0; i < 2; ++i) {
        const Vector curve_start_pnt = curves[i]->position_at(0);
        const Vector curve_end_pnt = curves[i]->position_at(1);
        if ((given[i] - curve_start_pnt).length() < TOLERANCE) {
            end_params[i] = 0;
        } else if ((given[i] - curve_end_pnt).length() < TOLERANCE) {
            end_params[i] = 1;
        } else {
            throw std::invalid_argument("end_points must be at either the start or end of a curve");
        }
    }

    const Vector start_pos = curve0.position_at(end_params[0]);
    const Vector end_pos = curve1.position_at(end_params[1]);
    // derivative_at instead of tangent_at gives the curves' natural "speed"
    const Vector start_deriv = curve0.derivative_at(end_params[0], 1) * tan_scalars.first;
    const Vector end_deriv = curve1.derivative_at(end_params[1], 1) * tan_scalars.second;

    if (continuity == ContinuityLevel::C0) return Line(start_pos, end_pos);
    if (continuity == ContinuityLevel::C1) {
        const Vector cntl_pnt1 = start_pos + start_deriv / 3;
        const Vector cntl_pnt4 = end_pos - end_deriv / 3;
        return Bezier({start_pos, cntl_pnt1, cntl_pnt4, end_pos});  // degree-3 Bezier
    }
    // C2
    const Vector start_curv = curve0.derivative_at(end_params[0], 2);
    const Vector end_curv = curve1.derivative_at(end_params[1], 2);
    const Vector cntl_pnt1 = start_pos + start_deriv / 5;
    const Vector cntl_pnt2 = start_pos + (2 * start_deriv) / 5 + start_curv / 20;
    const Vector cntl_pnt4 = end_pos - end_deriv / 5;
    const Vector cntl_pnt3 = end_pos - (2 * end_deriv) / 5 + end_curv / 20;
    return Bezier({start_pos, cntl_pnt1, cntl_pnt2, cntl_pnt3, cntl_pnt4, end_pos});  // degree-5 Bezier
}

Edge double_tangent_arc(const Vector& pnt, const Vector& tangent, const Shape& other, Keep keep) {
    if (keep != Keep::TOP && keep != Keep::BOTTOM) {
        throw std::invalid_argument("Only the TOP or BOTTOM options are supported");
    }
    if (other.is_null()) throw std::invalid_argument("other must be an Edge or a Wire");
    const auto type = other.wrapped().ShapeType();
    // Python needs other.tangent_at, which a Curve doesn't have.
    if (type != TopAbs_EDGE && type != TopAbs_WIRE) throw std::invalid_argument("other must be an Edge or a Wire");
    const Mixin1D other_1d(other.wrapped());

    const Vector arc_pt = pnt;
    const Vector arc_tangent = tangent.normalized();
    std::vector<Shape> other_edges;
    for (const auto& e : other.edges()) other_edges.push_back(e);
    const auto coplane = Edge::make_line(arc_pt, arc_pt + arc_tangent).common_plane(other_edges);
    if (!coplane) throw std::invalid_argument("DoubleTangentArc only works on a single plane");
    const Plane workplane = -*coplane;  // flip to help with TOP/BOTTOM
    const Axis rotation_axis(Vector(0, 0, 0), workplane.z_dir);
    // Protect against massive circles that are effectively straight lines
    const double max_size = 10 * other.bounding_box().add(arc_pt).diagonal();

    std::vector<Vector> arc_centers;
    for (double angle : {90.0, -90.0}) {
        const Vector perpendicular_bisector = arc_tangent.rotate(rotation_axis, angle);
        // Function to be minimized
        auto func = [&](const std::vector<double>& radius) {
            const Vector center = arc_pt + perpendicular_bisector * radius[0];
            const double separation = other.distance_to(center);
            return std::abs(separation - radius[0]);
        };
        NelderMeadOptions options;
        options.bounds = std::vector<std::pair<double, double>>{{0.0, max_size}};
        options.xatol = TOLERANCE;
        options.fatol = TOLERANCE;
        const NelderMeadResult result = minimize_nelder_mead(func, {0.0}, options);
        const double arc_radius = result.x[0];
        const Vector arc_center = arc_pt + perpendicular_bisector * arc_radius;

        // Check for matching tangents
        const Edge circle = Edge::make_circle(arc_radius, Plane(arc_center, std::nullopt, rotation_axis.direction));
        const auto [dist, p1, p2] = other.distance_to_with_closest_points(circle);
        if (dist > TOLERANCE) continue;  // if they aren't touching
        const Axis other_axis(p1, other_1d.tangent_at(p1));
        const Axis circle_axis(p2, circle.tangent_at(p2));
        if (other_axis.is_parallel(circle_axis, 0.05)) arc_centers.push_back(arc_center);
    }
    if (arc_centers.empty()) throw std::runtime_error("No double tangent arcs found");

    // If there are multiple solutions, select the desired one
    const Vector center = keep == Keep::TOP ? arc_centers.front() : arc_centers.back();
    const Vector p1 = std::get<1>(other.distance_to_with_closest_points(center));
    return TangentArc(arc_pt, p1, arc_tangent);
}

double py_float(const std::string& text) {
    std::size_t used = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &used);
    } catch (const std::exception&) {
        used = 0;
    }
    if (used != text.size() || text.empty()) {
        throw std::invalid_argument("could not convert string to float: '" + text + "'");
    }
    return value;
}

std::string strip(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

/// s[i] (an IndexError when out of range).
std::string char_at(const std::string& s, std::size_t i) {
    if (i >= s.size()) throw std::out_of_range("string index out of range");
    return s.substr(i, 1);
}

Wire airfoil_profile(const std::string& airfoil_code, int n_points, bool finite_te, std::vector<Vector>& camber_points,
                     double& m, double& p, double& t) {
    std::tie(m, p, t) = Airfoil::parse_naca4(airfoil_code);
    if (n_points < 0) throw std::invalid_argument("Number of samples must be non-negative.");
    const std::size_t n = static_cast<std::size_t>(n_points);

    // Cosine-spaced x values for better nose resolution (np.linspace(0, pi, n))
    std::vector<double> beta(n);
    if (n > 1) {
        const double step = PI / static_cast<double>(n - 1);
        for (std::size_t i = 0; i < n; ++i) beta[i] = static_cast<double>(i) * step + 0.0;
        beta[n - 1] = PI;
    } else if (n == 1) {
        beta[0] = 0.0;
    }
    std::vector<double> x(n);
    for (std::size_t i = 0; i < n; ++i) x[i] = (1 - std::cos(beta[i])) / 2;

    // Thickness distribution
    const double a0 = 0.2969, a1 = -0.1260, a2 = -0.3516, a3 = 0.2843;
    const double a4 = finite_te ? -0.1015 : -0.1036;
    std::vector<double> yt(n), yc(n, 0.0), dyc_dx(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const double xi = x[i];
        yt[i] = 5 * t * (a0 * std::sqrt(xi) + a1 * xi + a2 * (xi * xi) + a3 * std::pow(xi, 3) + a4 * std::pow(xi, 4));
    }

    // Camber line and slope
    if (!(m == 0 || p == 0 || p == 1)) {
        for (std::size_t i = 0; i < n; ++i) {
            const double xi = x[i];
            if (xi < p) {
                yc[i] = m / (p * p) * (2 * p * xi - xi * xi);
                dyc_dx[i] = 2 * m / (p * p) * (p - xi);
            } else {
                yc[i] = m / ((1 - p) * (1 - p)) * ((1 - 2 * p) + 2 * p * xi - xi * xi);
                dyc_dx[i] = 2 * m / ((1 - p) * (1 - p)) * (p - xi);
            }
        }
    }

    camber_points.clear();
    std::vector<Vector> upper_pnts, lower_pnts;
    for (std::size_t i = 0; i < n; ++i) {
        const double theta = std::atan(dyc_dx[i]);
        camber_points.emplace_back(x[i], yc[i]);
        // Upper and lower surfaces
        upper_pnts.emplace_back(x[i] - yt[i] * std::sin(theta), yc[i] + yt[i] * std::cos(theta));
        lower_pnts.emplace_back(x[i] + yt[i] * std::sin(theta), yc[i] - yt[i] * std::cos(theta));
    }

    // dict.fromkeys(upper[::-1] + lower): unique by Vector's rounded key, first one wins
    std::vector<Vector> unique_points;
    std::set<std::array<double, 3>> seen;
    auto add_unique = [&](const Vector& v) {
        const std::array<double, 3> key = {py_round(v.X, 5) + 0.0, py_round(v.Y, 5) + 0.0, py_round(v.Z, 5) + 0.0};
        if (seen.insert(key).second) unique_points.push_back(v);
    };
    for (auto it = upper_pnts.rbegin(); it != upper_pnts.rend(); ++it) add_unique(*it);
    for (const auto& v : lower_pnts) add_unique(v);

    const Edge surface = Edge::make_spline(unique_points, {}, !finite_te);
    if (finite_te) {
        const Edge trailing_edge = Edge::make_line(surface.position_at(0), surface.position_at(1));
        return Wire(std::vector<Edge>{surface, trailing_edge});
    }
    return Wire(std::vector<Edge>{surface});
}

Curve select_edges(const ShapeList<Edge>& solutions, const EdgeSelector& selector) {
    const ShapeList<Edge> selected = selector ? ShapeList<Edge>(selector(solutions)) : solutions;
    if (selected.empty()) throw std::invalid_argument("selector must return an Edge or list of Edges, not None");
    return Curve(selected);
}

}  // namespace

BlendCurve::BlendCurve(const Edge& curve0, const Edge& curve1, ContinuityLevel continuity,
                       std::optional<std::pair<Vector, Vector>> end_points,
                       std::optional<std::pair<double, double>> tangent_scalars)
    : BaseEdgeObject(blend_curve(curve0, curve1, continuity, end_points, tangent_scalars)) {}

DoubleTangentArc::DoubleTangentArc(const Vector& pnt, const Vector& tangent, const Shape& other, Keep keep)
    : BaseEdgeObject(double_tangent_arc(pnt, tangent, other, keep)) {}

std::tuple<double, double, double> Airfoil::parse_naca4(const std::string& value) {
    std::string s = value;
    for (std::size_t pos; (pos = s.find("NACA")) != std::string::npos;) s.erase(pos, 4);
    s = strip(s);
    double m, p, t;
    const auto dot = s.find('.');
    if (dot != std::string::npos) {
        const std::string int_part = s.substr(0, dot), frac_part = s.substr(dot + 1);
        m = static_cast<double>(py_int(char_at(int_part, 0))) / 100;
        p = static_cast<double>(py_int(char_at(int_part, 1))) / 10;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%02lld", py_int(int_part.substr(2)));
        t = py_float(std::string(buf) + "." + frac_part) / 100;
    } else {
        m = static_cast<double>(py_int(char_at(s, 0))) / 100;
        p = static_cast<double>(py_int(char_at(s, 1))) / 10;
        t = static_cast<double>(py_int(s.substr(2))) / 100;
    }
    return {m, p, t};
}

std::tuple<double, double, double> Airfoil::parse_naca4(double value) { return parse_naca4(py_float_repr(value)); }

Airfoil::Airfoil(const std::string& airfoil_code, int n_points, bool finite_te_in)
    : code(airfoil_code), max_camber(0), camber_pos(0), thickness(0), finite_te(finite_te_in) {
    const Wire profile =
        airfoil_profile(airfoil_code, n_points, finite_te_in, camber_points_, max_camber, camber_pos, thickness);
    obj_ = profile.wrapped();
}

Airfoil::Airfoil(double airfoil_code, int n_points, bool finite_te_in)
    : Airfoil(py_float_repr(airfoil_code), n_points, finite_te_in) {}

Edge Airfoil::camber_line() const { return Edge::make_spline(camber_points_); }

ConstrainedArcs::ConstrainedArcs(const TangencyTarget& tangency_one, const ConstrainedArcOptions& options,
                                 const EdgeSelector& selector)
    : BaseCurveObject(select_edges(Edge::make_constrained_arcs(tangency_one, options), selector)) {}

ConstrainedArcs::ConstrainedArcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                 const ConstrainedArcOptions& options, const EdgeSelector& selector)
    : BaseCurveObject(select_edges(Edge::make_constrained_arcs(tangency_one, tangency_two, options), selector)) {}

ConstrainedArcs::ConstrainedArcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                 const TangencyTarget& tangency_three, const ConstrainedArcOptions& options,
                                 const EdgeSelector& selector)
    : BaseCurveObject(
          select_edges(Edge::make_constrained_arcs(tangency_one, tangency_two, tangency_three, options), selector)) {}

ConstrainedLines::ConstrainedLines(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                   const ConstrainedLineOptions& options, const EdgeSelector& selector)
    : BaseCurveObject(select_edges(Edge::make_constrained_lines(tangency_one, tangency_two, options), selector)) {}

}  // namespace build123d
