// Gordon surfaces, part 1: the B-spline helpers of ocp_gordon
// (ocp_gordon/internal/bspline_algorithms.py and misc.clone_bspline*).
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include <BSplCLib.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomConvert.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_Conic.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <TColgp_HArray1OfPnt2d.hxx>
#include <math_Matrix.hxx>

#include "gordon_internal.hpp"

namespace build123d::gordon {

Handle(Geom_BSplineCurve) clone_bspline(const Handle(Geom_BSplineCurve)& spline) {
    return Handle(Geom_BSplineCurve)::DownCast(spline->Copy());
}

Handle(Geom_BSplineSurface) clone_bspline_surface(const Handle(Geom_BSplineSurface)& surface) {
    return Handle(Geom_BSplineSurface)::DownCast(surface->Copy());
}

std::vector<double> linspace_with_breaks(double umin, double umax, int n_values, const std::vector<double>& breaks) {
    if (n_values < 2) return n_values == 1 ? std::vector<double>{umin} : std::vector<double>{};
    const double du = (umax - umin) / (n_values - 1);
    std::vector<double> result;
    for (int i = 0; i < n_values; ++i) result.push_back(umin + i * du);
    const double eps = 0.3;
    for (double breakpoint : breaks) {
        int found_pos = -1;
        for (std::size_t i = 0; i < result.size(); ++i) {
            if (std::abs(result[i] - breakpoint) < du * eps) {
                result[i] = breakpoint;  // replace with the exact breakpoint
                found_pos = static_cast<int>(i);
                break;
            }
        }
        if (found_pos != -1) continue;
        // no point very close: insert the breakpoint next to the closest one
        int closest_idx = -1;
        double min_dist = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < result.size(); ++i) {
            const double dist = std::abs(result[i] - breakpoint);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = static_cast<int>(i);
            }
        }
        if (closest_idx != -1) {
            const auto pos = result.begin() + closest_idx;
            if (result[static_cast<std::size_t>(closest_idx)] > breakpoint) {
                result.insert(pos, breakpoint);
            } else {
                result.insert(pos + 1, breakpoint);
            }
        } else {
            result.insert(std::lower_bound(result.begin(), result.end(), breakpoint), breakpoint);
        }
    }
    return result;
}

bool is_u_dir_closed(const TColgp_Array2OfPnt& points, double tolerance) {
    for (int v_idx = points.LowerCol(); v_idx <= points.UpperCol(); ++v_idx) {
        if (!points.Value(points.LowerRow(), v_idx).IsEqual(points.Value(points.UpperRow(), v_idx), tolerance)) {
            return false;
        }
    }
    return true;
}

bool is_v_dir_closed(const TColgp_Array2OfPnt& points, double tolerance) {
    for (int u_idx = points.LowerRow(); u_idx <= points.UpperRow(); ++u_idx) {
        if (!points.Value(u_idx, points.LowerCol()).IsEqual(points.Value(u_idx, points.UpperCol()), tolerance)) {
            return false;
        }
    }
    return true;
}

std::vector<double> get_kink_parameters(const Handle(Geom_BSplineCurve)& curve) {
    if (curve.IsNull()) throw std::runtime_error("Null Pointer curve");
    const double eps = 1e-8;
    std::vector<double> kinks;
    for (int knot_index = 2; knot_index < curve->NbKnots(); ++knot_index) {
        if (curve->Multiplicity(knot_index) == curve->Degree()) {
            const double knot = curve->Knot(knot_index);
            // the tangents on either side of the knot
            const gp_Vec tangent1 = curve->DN(knot + eps, 1);
            const gp_Vec tangent2 = curve->DN(knot - eps, 1);
            const double angle = tangent1.Angle(tangent2);
            const double angle_tolerance = 1e-4;  // radians
            if (!(std::abs(angle) < angle_tolerance || std::abs(angle - PI) < angle_tolerance)) {
                kinks.push_back(knot);
            }
        }
    }
    return kinks;
}

std::vector<double> compute_params_bspline_curve(const TColgp_Array1OfPnt& points, double alpha) {
    const int n_points = points.Length();
    if (n_points < 2) return n_points == 1 ? std::vector<double>{0.0, 1.0} : std::vector<double>{};
    // the (1-based) points as indexed by the Python
    const auto point = [&](int i) -> const gp_Pnt& { return points(points.Lower() + i - 1); };
    std::vector<double> chord_lengths;
    double total_length = 0.0;
    for (int i = 1; i < n_points; ++i) {
        const gp_Pnt& p1 = point(i);
        const gp_Pnt& p2 = point(i + 1);
        const double dx = p2.X() - p1.X();
        const double dy = p2.Y() - p1.Y();
        const double dz = p2.Z() - p1.Z();
        const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
        chord_lengths.push_back(std::pow(length, alpha));
        total_length += chord_lengths.back();
    }
    if (total_length == 0.0) throw std::invalid_argument("float division by zero");
    std::vector<double> params{0.0};
    double current_length = 0.0;
    for (double length : chord_lengths) {
        current_length += length;
        params.push_back(current_length / total_length);
    }
    return params;
}

Handle(TColgp_HArray1OfPnt) pnt_array2_get_column(const TColgp_Array2OfPnt& matrix, int col_index) {
    Handle(TColgp_HArray1OfPnt) col_vector = new TColgp_HArray1OfPnt(matrix.LowerRow(), matrix.UpperRow());
    for (int row_idx = matrix.LowerRow(); row_idx <= matrix.UpperRow(); ++row_idx) {
        col_vector->SetValue(row_idx, matrix.Value(row_idx, col_index));
    }
    return col_vector;
}

Handle(TColgp_HArray1OfPnt) pnt_array2_get_row(const TColgp_Array2OfPnt& matrix, int row_index) {
    Handle(TColgp_HArray1OfPnt) row_vector = new TColgp_HArray1OfPnt(matrix.LowerCol(), matrix.UpperCol());
    for (int col_idx = matrix.LowerCol(); col_idx <= matrix.UpperCol(); ++col_idx) {
        row_vector->SetValue(col_idx, matrix.Value(row_index, col_idx));
    }
    return row_vector;
}

std::pair<std::vector<double>, std::vector<double>> compute_params_bspline_surf(const TColgp_Array2OfPnt& points,
                                                                                double alpha) {
    // the parameters of each column (u direction), averaged
    std::vector<double> params_u(static_cast<std::size_t>(points.ColLength()), 0.0);
    for (int v_idx = points.LowerCol(); v_idx <= points.UpperCol(); ++v_idx) {
        const auto points_u_line = pnt_array2_get_column(points, v_idx);
        const auto parameters_u_line = compute_params_bspline_curve(points_u_line->Array1(), alpha);
        for (std::size_t i = 0; i < parameters_u_line.size(); ++i) params_u[i] += parameters_u_line[i];
    }
    for (double& p : params_u) p /= static_cast<double>(points.RowLength());
    // the parameters of each row (v direction), averaged
    std::vector<double> params_v(static_cast<std::size_t>(points.RowLength()), 0.0);
    for (int u_idx = points.LowerRow(); u_idx <= points.UpperRow(); ++u_idx) {
        const auto points_v_line = pnt_array2_get_row(points, u_idx);
        const auto parameters_v_line = compute_params_bspline_curve(points_v_line->Array1(), alpha);
        for (std::size_t i = 0; i < parameters_v_line.size(); ++i) params_v[i] += parameters_v_line[i];
    }
    for (double& p : params_v) p /= static_cast<double>(points.ColLength());
    return {params_u, params_v};
}

void match_degree(const std::vector<Handle(Geom_BSplineCurve)>& bsplines) {
    if (bsplines.empty()) return;
    int max_degree = bsplines[0]->Degree();
    for (const auto& spline : bsplines) max_degree = std::max(max_degree, spline->Degree());
    for (const auto& spline : bsplines) {
        if (spline->Degree() < max_degree) spline->IncreaseDegree(max_degree);
    }
}

namespace {

int knot_multiplicity(const Handle(Geom_BSplineCurve)& spline, double knot, double tol) {
    for (int i = 1; i <= spline->NbKnots(); ++i) {
        if (std::abs(spline->Knot(i) - knot) < tol) return spline->Multiplicity(i);
    }
    return 0;
}

Handle(Geom_BSplineCurve) insert_knots(const Handle(Geom_BSplineCurve)& spline, const std::vector<double>& knots,
                                       const std::vector<int>& multiplicities, double tol) {
    Handle(Geom_BSplineCurve) new_spline = clone_bspline(spline);
    for (std::size_t k = 0; k < knots.size(); ++k) {
        bool knot_exists = false;
        for (int i = 1; i <= new_spline->NbKnots(); ++i) {
            if (std::abs(new_spline->Knot(i) - knots[k]) < tol) {
                knot_exists = true;
                new_spline->IncreaseMultiplicity(i, multiplicities[k]);
                break;
            }
        }
        if (!knot_exists) new_spline->InsertKnot(knots[k], multiplicities[k], tol, false);
    }
    return new_spline;
}

}  // namespace

std::vector<Handle(Geom_BSplineCurve)> create_common_knots_vector_curve(
    const std::vector<Handle(Geom_BSplineCurve)>& splines_vector, double tol) {
    if (splines_vector.empty()) return {};
    // every knot of every spline (Python's sorted set)
    std::set<double> all_knots;
    for (const auto& spline : splines_vector) {
        for (int i = 1; i <= spline->NbKnots(); ++i) all_knots.insert(spline->Knot(i));
    }
    std::vector<double> common_knots;
    std::vector<int> multiplicities;
    for (double knot : all_knots) {
        int max_mult = 0;
        for (const auto& spline : splines_vector) max_mult = std::max(max_mult, knot_multiplicity(spline, knot, tol));
        common_knots.push_back(knot);
        multiplicities.push_back(max_mult);
    }
    std::vector<Handle(Geom_BSplineCurve)> result;
    for (const auto& spline : splines_vector) result.push_back(insert_knots(spline, common_knots, multiplicities, tol));
    return result;
}

void reparametrize_bspline(const Handle(Geom_BSplineCurve)& spline, double umin, double umax, double /*tol*/) {
    TColStd_Array1OfReal knots = spline->Knots();
    BSplCLib::Reparametrize(umin, umax, knots);
    spline->SetKnots(knots);
}

namespace {

Handle(Geom_BSplineCurve) convert_to_bspline(const Handle(Geom_Curve)& curve) {
    const auto is_conic = [](const Handle(Geom_Curve)& c) {
        if (c->IsKind(STANDARD_TYPE(Geom_Conic))) return true;
        if (c->IsKind(STANDARD_TYPE(Geom_TrimmedCurve))) {
            return Handle(Geom_TrimmedCurve)::DownCast(c)->BasisCurve()->IsKind(STANDARD_TYPE(Geom_Conic));
        }
        return false;
    };
    if (is_conic(curve)) {
        // a full circle or ellipse (size 200) gives 99 poles, 33 knots, degree 5 and an
        // error of 1.3e-7; a trimmed one has fewer poles
        double curve_size = 0.0;
        const gp_Pnt curve_start_pnt = curve->Value(curve->FirstParameter());
        for (int i = 1; i < 3; ++i) {
            const double u = (1 - i / 4.0) * curve->FirstParameter() + (i / 4.0) * curve->LastParameter();
            curve_size = std::max(curve_size, curve_start_pnt.Distance(curve->Value(u)));
        }
        const double tol = Precision::Approximation() * curve_size / 200;
        GeomConvert_ApproxCurve approx(curve, tol, GeomAbs_C2, 64, 5);
        if (approx.HasResult() && approx.MaxError() < tol) return approx.Curve();
    }
    return GeomConvert::CurveToBSplineCurve(curve);
}

}  // namespace

std::vector<Handle(Geom_BSplineCurve)> to_bsplines(const std::vector<Handle(Geom_Curve)>& curves) {
    std::vector<Handle(Geom_BSplineCurve)> result;
    for (const auto& curve : curves) {
        const Handle(Geom_BSplineCurve) bspline = Handle(Geom_BSplineCurve)::DownCast(curve);
        result.push_back(bspline.IsNull() ? convert_to_bspline(curve) : clone_bspline(bspline));
    }
    return result;
}

std::vector<Handle(Geom_BSplineCurve)> to_bsplines(const std::vector<Handle(Geom_BSplineCurve)>& curves) {
    std::vector<Handle(Geom_BSplineCurve)> result;
    for (const auto& curve : curves) result.push_back(clone_bspline(curve));
    return result;
}

double scale(const Handle(Geom_BSplineCurve)& spline) {
    double scale_val = 0.0;
    if (spline->NbPoles() > 0) {
        const gp_Pnt first_ctrl_pnt = spline->Pole(1);
        for (int i = 2; i <= spline->NbPoles(); ++i) {
            scale_val = std::max(scale_val, first_ctrl_pnt.Distance(spline->Pole(i)));
        }
    }
    return scale_val > 0 ? scale_val : 1.0;
}

double scale(const std::vector<Handle(Geom_BSplineCurve)>& splines_vector) {
    double max_scale = 0.0;
    for (const auto& spline : splines_vector) max_scale = std::max(scale(spline), max_scale);
    return max_scale > 0 ? max_scale : 1.0;
}

double scale(const TColgp_Array2OfPnt& points) {
    double the_scale = 0.0;
    for (int u_idx = points.LowerRow(); u_idx <= points.UpperRow(); ++u_idx) {
        const gp_Pnt& p_first = points.Value(u_idx, points.LowerCol());
        for (int v_idx = points.LowerCol() + 1; v_idx <= points.UpperCol(); ++v_idx) {
            the_scale = std::max(the_scale, p_first.Distance(points.Value(u_idx, v_idx)));
        }
    }
    return the_scale;
}

ApproxResult reparametrize_bspline_continuously_approx(const Handle(Geom_BSplineCurve)& spline,
                                                       const std::vector<double>& old_parameters,
                                                       const std::vector<double>& new_parameters, int n_control_pnts) {
    const auto find_index_with_tolerance = [](const std::vector<double>& data, double target, double tolerance) {
        for (std::size_t i = 0; i < data.size(); ++i) {
            if (std::abs(data[i] - target) < tolerance) return static_cast<int>(i);
        }
        return -1;
    };
    if (old_parameters.size() != new_parameters.size()) throw std::runtime_error("parameter sizes dont match");
    // a 1D curve mapping the new parameters to the old ones
    Handle(TColgp_HArray1OfPnt2d) old_parameters_pnts = new TColgp_HArray1OfPnt2d(1, static_cast<int>(old_parameters.size()));
    for (std::size_t i = 0; i < old_parameters.size(); ++i) {
        old_parameters_pnts->SetValue(static_cast<int>(i) + 1, gp_Pnt2d(old_parameters[i], 0));
    }
    const Handle(TColStd_HArray1OfReal) new_parameters_array = to_array(new_parameters);
    Geom2dAPI_Interpolate interpolation_object(old_parameters_pnts, new_parameters_array, false, 1e-6);
    interpolation_object.Perform();
    if (!interpolation_object.IsDone()) throw std::runtime_error("Cannot reparametrize");
    const Handle(Geom2d_BSplineCurve) reparametrizing_spline = interpolation_object.Curve();

    std::vector<double> breaks(new_parameters.begin() + 1, new_parameters.end() - 1);
    const double par_tol = 1e-10;
    // the kinks, in the new parametrization, are not breaks
    std::vector<double> kinks = get_kink_parameters(spline);
    for (double& kink : kinks) {
        kink = Geom2dAPI_ProjectPointOnCurve(gp_Pnt2d(kink, 0.0), reparametrizing_spline).LowerDistanceParameter();
    }
    std::vector<double> new_breaks;
    for (double b : breaks) {
        const bool is_kink_nearby =
            std::any_of(kinks.begin(), kinks.end(), [&](double k) { return std::abs(b - k) < par_tol; });
        if (!is_kink_nearby) new_breaks.push_back(b);
    }
    breaks = new_breaks;
    // the approximation parameters, with the breaks and the kinks
    std::vector<double> parameters =
        linspace_with_breaks(new_parameters.front(), new_parameters.back(), std::max(101, n_control_pnts * 2), breaks);
    for (double kink : kinks) parameters.insert(std::lower_bound(parameters.begin(), parameters.end(), kink), kink);
    // the points to approximate
    Handle(TColgp_HArray1OfPnt) points = new TColgp_HArray1OfPnt(1, static_cast<int>(parameters.size()));
    for (std::size_t i = 0; i < parameters.size(); ++i) {
        const double old_parameter = reparametrizing_spline->Value(parameters[i]).X();
        points->SetValue(static_cast<int>(i) + 1, spline->Value(old_parameter));
    }
    const bool make_continuous =
        spline->IsClosed() &&
        spline->DN(spline->FirstParameter(), 1).Angle(spline->DN(spline->LastParameter(), 1)) < 6.0 / 180.0 * PI;
    BSplineApproxInterp approximation_obj(points->Array1(), n_control_pnts, 3, make_continuous);
    // the breaks and the ends are interpolated
    std::vector<double> breaks_for_interpolation{new_parameters.front()};
    breaks_for_interpolation.insert(breaks_for_interpolation.end(), breaks.begin(), breaks.end());
    breaks_for_interpolation.push_back(new_parameters.back());
    for (double the_break : breaks_for_interpolation) {
        const int idx = find_index_with_tolerance(parameters, the_break, par_tol);
        if (idx != -1) approximation_obj.interpolate_point(idx);
    }
    // the kinks are interpolated as kinks
    for (double kink : kinks) {
        const int idx = find_index_with_tolerance(parameters, kink, par_tol);
        if (idx != -1) approximation_obj.interpolate_point(idx, true);
    }
    return approximation_obj.fit_curve_optimal(parameters);
}

std::vector<double> knots_from_curve_parameters(std::vector<double>& params, int degree, bool closed_curve) {
    if (params.size() < 2) throw std::invalid_argument("Parameters must contain two or more elements.");
    int n_cp = static_cast<int>(params.size());
    // the first and last parameters of a closed curve are the same point
    if (closed_curve) n_cp += degree - 1;
    const int n_inner_knots = n_cp - degree + 1;
    std::vector<double> inner_knots(static_cast<std::size_t>(n_inner_knots), 0.0);
    inner_knots.front() = params.front();
    inner_knots.back() = params.back();
    std::vector<double> knots;
    if (closed_curve && degree % 2 == 0) {
        const int m = static_cast<int>(params.size()) - 2;
        std::vector<double> dparm(static_cast<std::size_t>(m + 1), 0.0);
        for (int iparm = 0; iparm <= m; ++iparm) dparm[iparm] = params[iparm + 1] - params[iparm];
        inner_knots[1] = inner_knots[0] + 0.5 * (dparm[0] + dparm[m]);
        for (int iparm = 1; iparm < m; ++iparm) {
            inner_knots[iparm + 1] = inner_knots[iparm] + 0.5 * (dparm[iparm - 1] + dparm[iparm]);
        }
        // shift the parameters
        for (double& p : params) p += dparm[m] / 2.0;
    } else if (closed_curve) {
        if (inner_knots.size() != params.size()) {
            throw std::invalid_argument("Inner knots size must match parameters size for closed curves");
        }
        inner_knots = params;
    } else {
        // averaging
        for (int j = 1; j < static_cast<int>(params.size()) - degree; ++j) {
            double sum_val = 0.0;
            for (int i = j; i < j + degree; ++i) sum_val += params[i];
            inner_knots[j] = sum_val / static_cast<double>(degree);
        }
    }
    if (closed_curve) {
        const double offset = inner_knots[0] - inner_knots[n_inner_knots - 1];
        for (int iknot = 0; iknot < degree; ++iknot) knots.push_back(offset + inner_knots[n_inner_knots - degree - 1 + iknot]);
        for (int iknot = 0; iknot < n_inner_knots; ++iknot) knots.push_back(inner_knots[iknot]);
        for (int iknot = 0; iknot < degree; ++iknot) knots.push_back(-offset + inner_knots[iknot + 1]);
    } else {
        for (int iknot = 0; iknot < degree; ++iknot) knots.push_back(inner_knots[0]);
        for (int iknot = 0; iknot < n_inner_knots; ++iknot) knots.push_back(inner_knots[iknot]);
        for (int iknot = 0; iknot < degree; ++iknot) knots.push_back(inner_knots[n_inner_knots - 1]);
    }
    if (closed_curve && degree <= 1) {
        knots.front() = knots[1];
        knots.back() = knots[knots.size() - 2];
    }
    return knots;
}

Matrix bspline_basis_mat(int degree, const TColStd_Array1OfReal& knots, const TColStd_Array1OfReal& params,
                         int deriv_order) {
    const int n_cp = knots.Length() - degree - 1;
    const int n_params = params.Length();
    Matrix basis_mat(n_params, n_cp);
    math_Matrix bspl_basis(1, deriv_order + 1, 1, degree + 1);
    bspl_basis.Init(0.0);
    for (int i_param = 1; i_param <= n_params; ++i_param) {
        const double param_val = params(params.Lower() + i_param - 1);
        int span_idx = 0;
        double new_u = 0.0;
        BSplCLib::LocateParameter(degree, knots, param_val, false, degree + 1, knots.Length() - degree, span_idx, new_u);
        int first_non_zero = 0;
        BSplCLib::EvalBsplineBasis(deriv_order, degree + 1, knots, param_val, first_non_zero, bspl_basis);
        for (int j = 0; j <= degree; ++j) {
            const int col_idx = span_idx - degree - 1 + j;
            if (0 <= col_idx && col_idx < n_cp) basis_mat(i_param - 1, col_idx) = bspl_basis.Value(deriv_order + 1, j + 1);
        }
    }
    return basis_mat;
}

double max_distance_of_bounding_box(const TColgp_Array1OfPnt& points) {
    double max_distance = 0.0;
    for (int i = points.Lower(); i < points.Upper(); ++i) {
        for (int j = i + 1; j <= points.Upper(); ++j) max_distance = std::max(max_distance, points(i).Distance(points(j)));
    }
    return max_distance;
}

bool is_closed(const TColgp_Array1OfPnt& points, bool c2_continuous) {
    const double error = 1e-6 * max_distance_of_bounding_box(points);
    return points(points.Lower()).IsEqual(points(points.Upper()), error) && c2_continuous;
}

Handle(TColStd_HArray1OfReal) to_array(const std::vector<double>& vector) {
    Handle(TColStd_HArray1OfReal) array = new TColStd_HArray1OfReal(1, static_cast<int>(vector.size()));
    for (std::size_t i = 0; i < vector.size(); ++i) array->SetValue(static_cast<int>(i) + 1, vector[i]);
    return array;
}

Handle(Geom_BSplineSurface) flip_surface(const Handle(Geom_BSplineSurface)& surface) {
    Handle(Geom_BSplineSurface) result = clone_bspline_surface(surface);
    result->ExchangeUV();
    return result;
}

void insert_knot_with_multiplicity(double knot, int count, int degree, std::vector<double>& knots, std::vector<int>& mults,
                                   double tol) {
    if (!(knots.front() - tol <= knot && knot <= knots.back() + tol)) throw std::runtime_error("knot out of range");
    int found_pos = -1;
    for (std::size_t i = 0; i < knots.size(); ++i) {
        if (std::abs(knots[i] - knot) < tol) {
            found_pos = static_cast<int>(i);
            break;
        }
    }
    if (found_pos != -1) {
        // the knot exists: increase its multiplicity, at most to the degree
        mults[found_pos] = std::min(mults[found_pos] + count, degree);
    } else {
        std::size_t insert_idx = 0;
        while (insert_idx < knots.size() && knots[insert_idx] < knot) ++insert_idx;
        knots.insert(knots.begin() + static_cast<std::ptrdiff_t>(insert_idx), knot);
        mults.insert(mults.begin() + static_cast<std::ptrdiff_t>(insert_idx), std::min(count, degree));
    }
}

}  // namespace build123d::gordon
