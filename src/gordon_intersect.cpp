// Gordon surfaces, part 4: B-spline/B-spline intersection
// (ocp_gordon/internal/intersect_bsplines.py).
#include <algorithm>
#include <cmath>
#include <limits>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "gordon_internal.hpp"

namespace build123d::gordon {

namespace {

gp_Pnt min_coords(const gp_Pnt& p1, const gp_Pnt& p2) {
    return gp_Pnt(std::min(p1.X(), p2.X()), std::min(p1.Y(), p2.Y()), std::min(p1.Z(), p2.Z()));
}

gp_Pnt max_coords(const gp_Pnt& p1, const gp_Pnt& p2) {
    return gp_Pnt(std::max(p1.X(), p2.X()), std::max(p1.Y(), p2.Y()), std::max(p1.Z(), p2.Z()));
}

/// Python's min/max of two floats (the first unless the second compares less/greater).
double py_min(double a, double b) { return b < a ? b : a; }
double py_max(double a, double b) { return b > a ? b : a; }

struct Intervall {
    double min;
    double max;
    bool operator==(const Intervall& other) const {
        const double eps = 1e-15;
        return std::abs(min - other.min) < eps && std::abs(max - other.max) < eps;
    }
};

/// The parameter range and the bounding box of the control points of a B-spline.
struct BoundingBox {
    Intervall range;
    gp_Pnt low;
    gp_Pnt high;

    explicit BoundingBox(const Handle(Geom_BSplineCurve)& curve)
        : range{curve->FirstParameter(), curve->LastParameter()} {
        // numpy's min and max over the poles (NaN propagates)
        const gp_Pnt& first = curve->Pole(1);
        double lo[3] = {first.X(), first.Y(), first.Z()};
        double hi[3] = {first.X(), first.Y(), first.Z()};
        for (int i = 2; i <= curve->NbPoles(); ++i) {
            const gp_Pnt& p = curve->Pole(i);
            const double c[3] = {p.X(), p.Y(), p.Z()};
            for (int k = 0; k < 3; ++k) {
                if (std::isnan(c[k]) || std::isnan(lo[k])) {
                    lo[k] = hi[k] = std::numeric_limits<double>::quiet_NaN();
                } else {
                    lo[k] = std::min(lo[k], c[k]);
                    hi[k] = std::max(hi[k], c[k]);
                }
            }
        }
        low = gp_Pnt(lo[0], lo[1], lo[2]);
        high = gp_Pnt(hi[0], hi[1], hi[2]);
    }

    bool intersects(const BoundingBox& other, double eps) const {
        const gp_Pnt min_c = max_coords(low, other.low);
        const gp_Pnt max_c = min_coords(high, other.high);
        return (min_c.X() < max_c.X() + eps) && (min_c.Y() < max_c.Y() + eps) && (min_c.Z() < max_c.Z() + eps);
    }

    BoundingBox& merge(const BoundingBox& other) {
        range.max = other.range.max;
        high = max_coords(high, other.high);
        low = min_coords(low, other.low);
        return *this;
    }

    bool operator==(const BoundingBox& other) const { return range == other.range; }
};

double curvature(const Handle(Geom_BSplineCurve)& curve) {
    const double len_curve = curve->Pole(1).Distance(curve->Pole(curve->NbPoles()));
    double total = 0.0;
    for (int i = 1; i < curve->NbPoles(); ++i) total += curve->Pole(i).Distance(curve->Pole(i + 1));
    if (std::abs(len_curve) < 1e-15) return total < 1e-15 ? 1.0 : 1e9;
    return py_max(total / len_curve, 1.0);
}

struct BoundingBoxPair {
    BoundingBox b1;
    BoundingBox b2;
};

Handle(Geom_BSplineCurve) trim_curve(const Handle(Geom_BSplineCurve)& curve, double first, double last) {
    Handle(Geom_BSplineCurve) new_curve = clone_bspline(curve);
    new_curve->Segment(first, last);
    return new_curve;
}

/// The squared distance between two curves at normalized parameters, and its gradient.
class CurveCurveDistanceObjective : public MultipleVarFunctionWithGradient {
public:
    CurveCurveDistanceObjective(const Handle(Geom_Curve)& c1, const Handle(Geom_Curve)& c2) : m_c1(c1), m_c2(c2) {}

    double get_u_param(double x0) const {
        const double umin = m_c1->FirstParameter(), umax = m_c1->LastParameter();
        return x0 * (umax - umin) + umin;
    }
    double get_v_param(double x1) const {
        const double vmin = m_c2->FirstParameter(), vmax = m_c2->LastParameter();
        return x1 * (vmax - vmin) + vmin;
    }
    double d_get_u_param() const { return 1 * (m_c1->LastParameter() - m_c1->FirstParameter()); }
    double d_get_v_param() const { return 1 * (m_c2->LastParameter() - m_c2->FirstParameter()); }

    bool values(const std::vector<double>& x, double& f, std::vector<double>& g) const override {
        const double u = get_u_param(x[0]);
        const double v = get_v_param(x[1]);
        gp_Pnt p1(0, 0, 0), p2(0, 0, 0);
        gp_Vec d1_vec(0, 0, 0), d2_vec(0, 0, 0);
        m_c1->D1(u, p1, d1_vec);
        m_c2->D1(v, p2, d2_vec);
        const gp_Vec diff(p2, p1);  // p1 - p2
        f = diff.SquareMagnitude();
        g[0] = 2.0 * diff.Dot(d1_vec) * d_get_u_param();
        g[1] = -2.0 * diff.Dot(d2_vec) * d_get_v_param();
        return true;
    }

private:
    Handle(Geom_Curve) m_c1;
    Handle(Geom_Curve) m_c2;
};

double find_split_point(const Handle(Geom_BSplineCurve)& curve) {
    const double mid_point = 0.5 * (curve->FirstParameter() + curve->LastParameter());
    if (curve->IsCN(curve->Degree() - 1)) return mid_point;
    // split at the inner knot of the highest multiplicity
    int max_inner_mult_index = 2;
    for (int i = 3; i < curve->NbKnots(); ++i) {
        if (curve->Multiplicity(i) > curve->Multiplicity(max_inner_mult_index)) max_inner_mult_index = i;
    }
    if (curve->Multiplicity(max_inner_mult_index) > 1) return curve->Knot(max_inner_mult_index);
    return mid_point;
}

/// The pairs of nearly straight pieces whose control point boxes intersect.
std::vector<BoundingBoxPair> get_ranges_of_intersection(const Handle(Geom_BSplineCurve)& curve1,
                                                        const Handle(Geom_BSplineCurve)& curve2, double tolerance) {
    const BoundingBox h1(curve1);
    const BoundingBox h2(curve2);
    if (!h1.intersects(h2, tolerance)) return {};
    const double c1_curvature = curvature(curve1);
    const double c2_curvature = curvature(curve2);
    const double max_curvature = 1.0005;
    if (c1_curvature <= max_curvature && c2_curvature <= max_curvature) return {BoundingBoxPair{h1, h2}};
    const double curve1_mid_parm = find_split_point(curve1);
    const double curve2_mid_parm = find_split_point(curve2);
    std::vector<BoundingBoxPair> results;
    const auto extend = [&](const std::vector<BoundingBoxPair>& more) { results.insert(results.end(), more.begin(), more.end()); };
    if (c1_curvature > max_curvature && c2_curvature > max_curvature) {
        const auto c11 = trim_curve(curve1, curve1->FirstParameter(), curve1_mid_parm);
        const auto c12 = trim_curve(curve1, curve1_mid_parm, curve1->LastParameter());
        const auto c21 = trim_curve(curve2, curve2->FirstParameter(), curve2_mid_parm);
        const auto c22 = trim_curve(curve2, curve2_mid_parm, curve2->LastParameter());
        extend(get_ranges_of_intersection(c11, c21, tolerance));
        extend(get_ranges_of_intersection(c11, c22, tolerance));
        extend(get_ranges_of_intersection(c12, c21, tolerance));
        extend(get_ranges_of_intersection(c12, c22, tolerance));
    } else if (c1_curvature <= max_curvature && max_curvature < c2_curvature) {
        const auto c21 = trim_curve(curve2, curve2->FirstParameter(), curve2_mid_parm);
        const auto c22 = trim_curve(curve2, curve2_mid_parm, curve2->LastParameter());
        extend(get_ranges_of_intersection(curve1, c21, tolerance));
        extend(get_ranges_of_intersection(curve1, c22, tolerance));
    } else if (c2_curvature <= max_curvature && max_curvature < c1_curvature) {
        const auto c11 = trim_curve(curve1, curve1->FirstParameter(), curve1_mid_parm);
        const auto c12 = trim_curve(curve1, curve1_mid_parm, curve1->LastParameter());
        extend(get_ranges_of_intersection(c11, curve2, tolerance));
        extend(get_ranges_of_intersection(c12, curve2, tolerance));
    }
    return results;
}

/// Merge each run of adjacent boxes (unless they meet at an avoided parameter).
void replace_adjacent_in_list(const std::vector<double>& avoid_u_values, std::vector<BoundingBox>& list_obj) {
    const auto is_adjacent = [&](const BoundingBox& b1, const BoundingBox& b2) {
        const double eps = 1e-15;
        if (!avoid_u_values.empty()) {
            double smallest = std::numeric_limits<double>::infinity();
            bool first = true;
            for (double u : avoid_u_values) {
                const double d = py_min(std::abs(b1.range.max - u), std::abs(b2.range.min - u));
                smallest = first ? d : py_min(smallest, d);
                first = false;
            }
            if (smallest < eps) return false;
        }
        return std::abs(b1.range.max - b2.range.min) < eps;
    };
    std::size_t i = 0;
    while (i < list_obj.size()) {
        const std::size_t next_i = i + 1;
        if (next_i >= list_obj.size()) break;
        if (is_adjacent(list_obj[i], list_obj[next_i])) {
            BoundingBox merged_val = list_obj[i];
            merged_val.merge(list_obj[next_i]);
            list_obj.erase(list_obj.begin() + static_cast<std::ptrdiff_t>(i), list_obj.begin() + static_cast<std::ptrdiff_t>(i + 2));
            list_obj.insert(list_obj.begin() + static_cast<std::ptrdiff_t>(i), merged_val);
        } else {
            i = next_i;
        }
    }
}

/// is_point_on_line_segment: the parameter of the projection on the segment, and whether the
/// point may lie on a curve this straight.
std::pair<double, bool> is_point_on_line_segment(const gp_Pnt& point, const gp_Pnt& segment_start, const gp_Pnt& segment_end,
                                                 double max_curvature = 1.0005) {
    const gp_Vec vec_segment(segment_start, segment_end);
    const gp_Vec vec_point_from_start(segment_start, point);
    const double line_length = vec_segment.Magnitude();
    if (line_length < 1e-9) return {0, vec_point_from_start.SquareMagnitude() < 1e-9};
    const double t = vec_point_from_start.Dot(vec_segment) / (line_length * line_length);
    bool possible_intersect = true;
    // the largest distance between a curve this straight and its chord
    const double rel_sagitta = 0.6123724 * std::sqrt(max_curvature - 1);
    const double max_distance = 1.2 * rel_sagitta * line_length;  // 1.2 for margin
    if (t * line_length < -max_distance || t * line_length > line_length + max_distance) possible_intersect = false;
    gp_Pnt closest_point_on_line(segment_start.XYZ());
    closest_point_on_line.BaryCenter(1 - t, segment_end, t);
    const double distance_to_line = point.Distance(closest_point_on_line);
    if (distance_to_line > max_distance) possible_intersect = false;
    return {t, possible_intersect};
}

/// line_line_intersection_3d: the parameters of the closest points of two segments, and whether
/// curves this straight may intersect.
std::tuple<double, double, bool> line_line_intersection_3d(const gp_Pnt& p1_start, const gp_Pnt& p1_end,
                                                           const gp_Pnt& p2_start, const gp_Pnt& p2_end,
                                                           double max_curvature = 1.0005) {
    const gp_Vec u(p1_start, p1_end);
    const gp_Vec v(p2_start, p2_end);
    const gp_Vec w(p1_start, p2_start);
    const double a = u.Dot(u);
    const double b = u.Dot(v);
    const double c = v.Dot(v);
    const double d = u.Dot(w);
    const double e = v.Dot(w);
    const double denom = a * c - b * b;
    double t, s;
    if (std::abs(denom) < 1e-12) {
        // parallel or collinear lines
        t = 0.5;
        s = 0.5;
    } else {
        t = (c * d - b * e) / denom;
        s = (b * d - a * e) / denom;
    }
    bool possible_intersect = true;
    const double p1_mag = std::sqrt(a);
    const double p2_mag = std::sqrt(c);
    const double rel_sagitta = 0.6123724 * std::sqrt(max_curvature - 1);
    const double max_distance = 1.2 * rel_sagitta * (p1_mag + p2_mag);  // 1.2 for margin
    if (t * p1_mag < -max_distance || t * p1_mag > p1_mag + max_distance) possible_intersect = false;
    if (s * p2_mag < -max_distance || s * p2_mag > p2_mag + max_distance) possible_intersect = false;
    gp_Pnt closest_p1(p1_start.XYZ());
    closest_p1.BaryCenter(1 - t, p1_end, t);
    gp_Pnt closest_p2(p2_start.XYZ());
    closest_p2.BaryCenter(1 - s, p2_end, s);
    if (closest_p1.Distance(closest_p2) > max_distance) possible_intersect = false;
    return {t, s, possible_intersect};
}

}  // namespace

std::vector<BSplineIntersection> intersect_bsplines(const Handle(Geom_BSplineCurve)& curve1,
                                                    const Handle(Geom_BSplineCurve)& curve2, double tolerance) {
    const std::vector<BoundingBoxPair> hulls = get_ranges_of_intersection(curve1, curve2, tolerance);
    std::vector<BoundingBox> curve1_ints, curve2_ints;
    for (const auto& hull : hulls) {
        curve1_ints.push_back(hull.b1);
        curve2_ints.push_back(hull.b2);
    }
    const auto by_start = [](const BoundingBox& a, const BoundingBox& b) { return a.range.min < b.range.min; };
    std::stable_sort(curve1_ints.begin(), curve1_ints.end(), by_start);
    std::stable_sort(curve2_ints.begin(), curve2_ints.end(), by_start);
    // remove consecutive duplicates
    const auto unique = [](const std::vector<BoundingBox>& ints) {
        std::vector<BoundingBox> out;
        for (std::size_t i = 0; i < ints.size(); ++i) {
            if (i == 0 || !(ints[i] == ints[i - 1])) out.push_back(ints[i]);
        }
        return out;
    };
    curve1_ints = unique(curve1_ints);
    curve2_ints = unique(curve2_ints);
    // the inner knots of multiplicity > 1 are not merged across
    const auto create_avoid_u_values = [](const Handle(Geom_BSplineCurve)& curve) {
        std::vector<double> avoid_u_values;
        for (int i = 2; i < curve->NbKnots(); ++i) {
            if (curve->Multiplicity(i) > 1) avoid_u_values.push_back(curve->Knot(i));
        }
        return avoid_u_values;
    };
    replace_adjacent_in_list(create_avoid_u_values(curve1), curve1_ints);
    replace_adjacent_in_list(create_avoid_u_values(curve2), curve2_ints);
    std::vector<BoundingBoxPair> intersection_candidates;
    for (const auto& b1 : curve1_ints) {
        for (const auto& b2 : curve2_ints) {
            if (b1.intersects(b2, tolerance)) intersection_candidates.push_back(BoundingBoxPair{b1, b2});
        }
    }
    std::vector<BSplineIntersection> results;
    const auto append_result = [&](const BSplineIntersection& result) {
        for (const auto& r : results) {
            if (std::abs(r.parm_on_curve1 - result.parm_on_curve1) < tolerance &&
                std::abs(r.parm_on_curve2 - result.parm_on_curve2) < tolerance) {
                return;
            }
        }
        results.push_back(result);
    };
    const double tolerance_distance = py_max(1e-10, py_min(1e-5, tolerance));
    for (const auto& boxes : intersection_candidates) {
        const Handle(Geom_BSplineCurve) c1 = trim_curve(curve1, boxes.b1.range.min, boxes.b1.range.max);
        const Handle(Geom_BSplineCurve) c2 = trim_curve(curve2, boxes.b2.range.min, boxes.b2.range.max);
        // is a point of the first curve (at param1) on the second curve?
        const auto check_point_on_curve2 = [&](const Handle(Geom_BSplineCurve)& cx1, double param1,
                                               const Handle(Geom_BSplineCurve)& cx2) -> std::optional<std::pair<double, double>> {
            const gp_Pnt point = cx1->Value(param1);
            double v = cx2->FirstParameter();
            const gp_Pnt cx2_first_point = cx2->Value(v);
            if (point.Distance(cx2_first_point) < tolerance_distance) return std::make_pair(param1, v);
            v = cx2->LastParameter();
            const gp_Pnt cx2_last_point = cx2->Value(v);
            if (point.Distance(cx2_last_point) < tolerance_distance) return std::make_pair(param1, v);
            const auto [t, possible_intersect] =
                is_point_on_line_segment(point, cx2_first_point, cx2_last_point, py_max(curvature(cx1), curvature(cx2)));
            (void)t;
            if (!possible_intersect) return std::nullopt;
            const ScalarResult res = minimize_scalar_bounded(
                [&](double vv) { return cx1->Value(param1).SquareDistance(cx2->Value(vv)); }, cx2->FirstParameter(),
                cx2->LastParameter(), 0.1 * tolerance_distance * tolerance_distance);
            if (res.success && res.fun < tolerance_distance * tolerance_distance) return std::make_pair(param1, res.x);
            return std::nullopt;
        };
        // the ends of each piece on the other piece
        if (const auto uv = check_point_on_curve2(c1, c1->FirstParameter(), c2)) {
            append_result({uv->first, uv->second});
            continue;
        }
        if (const auto uv = check_point_on_curve2(c1, c1->LastParameter(), c2)) {
            append_result({uv->first, uv->second});
            continue;
        }
        if (const auto vu = check_point_on_curve2(c2, c2->FirstParameter(), c1)) {
            append_result({vu->second, vu->first});
            continue;
        }
        if (const auto vu = check_point_on_curve2(c2, c2->LastParameter(), c1)) {
            append_result({vu->second, vu->first});
            continue;
        }
        // the closest points of the two pieces, from those of their chords
        const CurveCurveDistanceObjective obj(c1, c2);
        const gp_Pnt p1_start = c1->Value(c1->FirstParameter());
        const gp_Pnt p1_end = c1->Value(c1->LastParameter());
        const gp_Pnt p2_start = c2->Value(c2->FirstParameter());
        const gp_Pnt p2_end = c2->Value(c2->LastParameter());
        const auto [t_closest, s_closest, possible_intersect] =
            line_line_intersection_3d(p1_start, p1_end, p2_start, p2_end, py_max(curvature(c1), curvature(c2)));
        if (!possible_intersect) continue;
        std::vector<double> guess{t_closest, s_closest};
        for (int i = 0; i < 2; ++i) {
            if (math_bfgs(obj, guess, tolerance_distance * tolerance_distance)) break;
        }
        const double u = obj.get_u_param(guess[0]);
        const double v = obj.get_v_param(guess[1]);
        if (u < py_min(c1->FirstParameter(), c1->LastParameter()) - tolerance_distance ||
            u > py_max(c1->FirstParameter(), c1->LastParameter()) + tolerance_distance) {
            continue;
        }
        if (v < py_min(c2->FirstParameter(), c2->LastParameter()) - tolerance_distance ||
            v > py_max(c2->FirstParameter(), c2->LastParameter()) + tolerance_distance) {
            continue;
        }
        const double distance = c1->Value(u).Distance(c2->Value(v));
        if (distance < tolerance) append_result({u, v});
    }
    return results;
}

}  // namespace build123d::gordon
