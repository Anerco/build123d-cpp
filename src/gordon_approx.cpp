// Gordon surfaces, part 3: approximation, interpolation and skinning
// (ocp_gordon/internal/bspline_approx_interp.py, points_to_bspline_interpolation.py and
// curves_to_surface.py).
#include <algorithm>
#include <cmath>
#include <string>

#include <BSplCLib.hxx>
#include <GeomConvert.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Vec.hxx>

#include "gordon_internal.hpp"

namespace build123d::gordon {

namespace {

TColStd_Array1OfReal real_array(const std::vector<double>& values) {
    TColStd_Array1OfReal array(1, static_cast<int>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) array.SetValue(static_cast<int>(i) + 1, values[i]);
    return array;
}

TColStd_Array1OfInteger int_array(const std::vector<int>& values) {
    TColStd_Array1OfInteger array(1, static_cast<int>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) array.SetValue(static_cast<int>(i) + 1, values[i]);
    return array;
}

bool contains(const std::vector<int>& values, int value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

// ---------------------------------------------------------------------------
// BSplineApproxInterp
// ---------------------------------------------------------------------------

BSplineApproxInterp::BSplineApproxInterp(const TColgp_Array1OfPnt& points, int n_control_points, int degree,
                                         bool continuous_if_closed)
    : m_pnts(1, points.Length()), m_degree(degree), m_ncp(n_control_points), m_c2_continuous(continuous_if_closed) {
    for (int i = points.Lower(); i <= points.Upper(); ++i) m_pnts.SetValue(i - points.Lower() + 1, points(i));
    for (int i = 0; i < points.Length(); ++i) m_index_of_approximated.push_back(i);
}

void BSplineApproxInterp::interpolate_point(int point_index, bool with_kink) {
    const auto it = std::find(m_index_of_approximated.begin(), m_index_of_approximated.end(), point_index);
    if (it == m_index_of_approximated.end()) {
        throw std::runtime_error("Invalid index in BSplineApproxInterp::interpolate_point");
    }
    m_index_of_approximated.erase(it);
    m_index_of_interpolated.push_back(point_index);
    if (with_kink) m_index_of_kinks.push_back(point_index);
}

bool BSplineApproxInterp::is_closed() const {
    if (!m_c2_continuous) return false;
    // the diagonal of the points' bounding box
    double max_distance = 0.0;
    if (m_pnts.Length() > 0) {
        gp_XYZ low = m_pnts(m_pnts.Lower()).XYZ(), high = low;
        for (int i = m_pnts.Lower(); i <= m_pnts.Upper(); ++i) {
            const gp_XYZ& p = m_pnts(i).XYZ();
            low.SetCoord(std::min(low.X(), p.X()), std::min(low.Y(), p.Y()), std::min(low.Z(), p.Z()));
            high.SetCoord(std::max(high.X(), p.X()), std::max(high.Y(), p.Y()), std::max(high.Z(), p.Z()));
        }
        const gp_XYZ delta = high - low;
        max_distance = std::sqrt(delta.X() * delta.X() + delta.Y() * delta.Y() + delta.Z() * delta.Z());
    }
    const double error_val = 1e-12 * max_distance;
    return m_pnts(m_pnts.Lower()).IsEqual(m_pnts(m_pnts.Upper()), error_val);
}

bool BSplineApproxInterp::first_and_last_interpolated() const {
    return contains(m_index_of_interpolated, 0) && contains(m_index_of_interpolated, m_pnts.Length() - 1);
}

std::vector<double> BSplineApproxInterp::compute_parameters(double alpha) const {
    double sum_len = 0.0;
    const int n_points = m_pnts.Length();
    std::vector<double> params(static_cast<std::size_t>(n_points), 0.0);
    // the cumulative (chord length)^alpha
    for (int i = 1; i < n_points; ++i) {
        const gp_Pnt& p1 = m_pnts(m_pnts.Lower() + i - 1);
        const gp_Pnt& p2 = m_pnts(m_pnts.Lower() + i);
        const double len2 = p1.SquareDistance(p2);
        sum_len += std::pow(len2, alpha / 2.0);
        params[static_cast<std::size_t>(i)] = sum_len;
    }
    const double tmax = params[static_cast<std::size_t>(n_points - 1)];
    if (tmax < 1e-10) {
        // all the points are the same
        for (int i = 0; i < n_points; ++i) {
            params[static_cast<std::size_t>(i)] = n_points > 1 ? static_cast<double>(i) / (n_points - 1) : 0.0;
        }
    } else {
        for (int i = 1; i < n_points; ++i) params[static_cast<std::size_t>(i)] /= tmax;
    }
    if (n_points > 0) params[static_cast<std::size_t>(n_points - 1)] = 1.0;
    return params;
}

std::pair<std::vector<double>, std::vector<int>> BSplineApproxInterp::compute_knots(int ncp,
                                                                                    const std::vector<double>& params) const {
    const int order = m_degree + 1;
    if (ncp < order) throw std::runtime_error("Number of control points too small!");
    const double umin = *std::min_element(params.begin(), params.end());
    const double umax = *std::max_element(params.begin(), params.end());
    std::vector<double> knots(static_cast<std::size_t>(ncp - m_degree + 1), 0.0);
    std::vector<int> mults(static_cast<std::size_t>(ncp - m_degree + 1), 0);
    // fixed knot positions, clamped at the ends
    knots[0] = umin;
    mults[0] = order;
    const int n = ncp - order;
    for (int i = 1; i <= n; ++i) {
        knots[static_cast<std::size_t>(i)] = umin + (umax - umin) * static_cast<double>(i) / static_cast<double>(n + 1);
        mults[static_cast<std::size_t>(i)] = 1;
    }
    knots[static_cast<std::size_t>(n + 1)] = umax;
    mults[static_cast<std::size_t>(n + 1)] = order;
    // a kink is a knot of multiplicity degree
    for (int kink_idx : m_index_of_kinks) {
        const double knot_val = params[static_cast<std::size_t>(kink_idx)];
        insert_knot_with_multiplicity(knot_val, m_degree, m_degree, knots, mults, 1e-4);
    }
    return {knots, mults};
}

ApproxResult BSplineApproxInterp::fit_curve_optimal(std::vector<double>& initial_params, int max_iter) {
    std::vector<double> computed;
    std::vector<double>* params = &initial_params;
    if (initial_params.empty()) {
        computed = compute_parameters(0.5);
        params = &computed;
    }
    if (static_cast<int>(params->size()) != m_pnts.Length()) {
        throw std::runtime_error("Number of parameters don't match number of points");
    }
    const auto [knots_list, mults_list] = compute_knots(m_ncp, *params);
    const TColStd_Array1OfReal knots_array = real_array(knots_list);
    const TColStd_Array1OfInteger mults_array = int_array(mults_list);
    int iteration = 0;
    ApproxResult result = solve(*params, knots_array, mults_array);
    double old_error = result.error * 2.0;
    // optimize the parameters by projection as long as the error decreases
    while (result.error > 0 && (old_error - result.error) / std::max(result.error, 1e-6) > 1e-3 && iteration < max_iter &&
           !result.curve.IsNull()) {
        old_error = result.error;
        optimize_parameters(result.curve, *params);
        result = solve(*params, knots_array, mults_array);
        iteration += 1;
    }
    return result;
}

std::pair<double, double> BSplineApproxInterp::project_on_curve(const gp_Pnt& pnt, const Handle(Geom_Curve)& curve,
                                                                double initial_param) const {
    const int max_iter = 10;  // maximum number of iterations
    const double eps = 1.0e-6;  // accuracy of the arc length parameter
    double t_new = initial_param;
    double t = t_new;
    gp_Pnt current_point;
    gp_Vec tangent_vec, second_deriv_vec, diff;
    for (int i = 0; i < max_iter; ++i) {
        t = t_new;
        curve->D2(t, current_point, tangent_vec, second_deriv_vec);
        diff = gp_Vec(pnt, current_point);
        // Newton on the derivative of the squared distance
        const double df = diff.Dot(tangent_vec);
        const double d2f = diff.Dot(second_deriv_vec) + tangent_vec.SquareMagnitude();
        if (std::abs(d2f) < 1e-12) break;
        const double dt = -df / d2f;
        if (std::abs(dt) < eps) break;
        t_new = t + dt;
        if (t_new < curve->FirstParameter() || t_new > curve->LastParameter()) break;
    }
    return {t, diff.Magnitude()};
}

Matrix BSplineApproxInterp::get_continuity_matrix(int n_ctr_pnts, int contin_cons, const std::vector<double>& params,
                                                  const TColStd_Array1OfReal& flat_knots) const {
    Matrix continuity_entries(contin_cons, n_ctr_pnts);
    TColStd_Array1OfReal continuity_params1(1, 1), continuity_params2(1, 1);
    continuity_params1.SetValue(1, params.front());
    continuity_params2.SetValue(1, params.back());
    const auto set_row = [&](int row, int deriv_order) {
        const Matrix diff_1 = bspline_basis_mat(m_degree, flat_knots, continuity_params1, deriv_order);
        const Matrix diff_2 = bspline_basis_mat(m_degree, flat_knots, continuity_params2, deriv_order);
        for (int j = 0; j < n_ctr_pnts; ++j) continuity_entries(row, j) = diff_1(0, j) - diff_2(0, j);
    };
    // C1 and C2 conditions
    set_row(0, 1);
    set_row(1, 2);
    // the closing condition, unless the ends are interpolated
    if (!first_and_last_interpolated()) set_row(2, 0);
    return continuity_entries;
}

ApproxResult BSplineApproxInterp::solve(const std::vector<double>& params, const TColStd_Array1OfReal& knots,
                                        const TColStd_Array1OfInteger& mults) const {
    const int n_flat_knots = BSplCLib::KnotSequenceLength(mults, m_degree, false);
    TColStd_Array1OfReal flat_knots_array(1, n_flat_knots);
    BSplCLib::KnotSequence(knots, mults, flat_knots_array);
    const int n_approximated = static_cast<int>(m_index_of_approximated.size());
    const int n_interpolated = static_cast<int>(m_index_of_interpolated.size());
    int n_continuity_conditions = 0;
    const bool make_closed = is_closed();
    if (make_closed) {
        // C0, C1 and C2 continuity; C0 is already given by interpolated end points
        n_continuity_conditions = 3;
        if (first_and_last_interpolated()) n_continuity_conditions -= 1;
    }
    const int n_ctr_pnts = flat_knots_array.Length() - m_degree - 1;
    if (n_ctr_pnts < n_interpolated + n_continuity_conditions || n_ctr_pnts < m_degree + 1 + n_continuity_conditions) {
        throw std::runtime_error("Too few control points for curve interpolation!");
    }
    if (n_approximated == 0 && n_ctr_pnts != n_interpolated + n_continuity_conditions) {
        throw std::runtime_error("Wrong number of control points for curve interpolation!");
    }
    // the least-squares system with the interpolation conditions as Lagrange multipliers
    const int n_vars = n_ctr_pnts + n_interpolated + n_continuity_conditions;
    Matrix lhs(n_vars, n_vars);
    Matrix rhs(n_vars, 3);
    const auto point_of = [&](int idx) -> const gp_Pnt& { return m_pnts(m_pnts.Lower() + idx); };
    if (n_approximated > 0) {
        TColStd_Array1OfReal app_params_array(1, n_approximated);
        Matrix b(n_approximated, 3);
        for (int i = 0; i < n_approximated; ++i) {
            const int idx = m_index_of_approximated[static_cast<std::size_t>(i)];
            app_params_array.SetValue(i + 1, params[static_cast<std::size_t>(idx)]);
            const gp_Pnt& p = point_of(idx);
            b(i, 0) = p.X();
            b(i, 1) = p.Y();
            b(i, 2) = p.Z();
        }
        const Matrix a = bspline_basis_mat(m_degree, flat_knots_array, app_params_array);
        const Matrix at = transpose(a);
        const Matrix ata = matmul(at, a);
        const Matrix atb = matmul(at, b);
        for (int i = 0; i < n_ctr_pnts; ++i) {
            for (int j = 0; j < n_ctr_pnts; ++j) lhs(i, j) = ata(i, j);
            for (int j = 0; j < 3; ++j) rhs(i, j) = atb(i, j);
        }
    }
    if (n_interpolated + n_continuity_conditions > 0) {
        Matrix d(n_interpolated + n_continuity_conditions, 3);
        if (n_interpolated > 0) {
            TColStd_Array1OfReal interp_params_array(1, n_interpolated);
            for (int i = 0; i < n_interpolated; ++i) {
                const int idx = m_index_of_interpolated[static_cast<std::size_t>(i)];
                interp_params_array.SetValue(i + 1, params[static_cast<std::size_t>(idx)]);
                const gp_Pnt& p = point_of(idx);
                d(i, 0) = p.X();
                d(i, 1) = p.Y();
                d(i, 2) = p.Z();
            }
            const Matrix c = bspline_basis_mat(m_degree, flat_knots_array, interp_params_array);
            for (int i = 0; i < n_interpolated; ++i) {
                for (int j = 0; j < n_ctr_pnts; ++j) {
                    lhs(j, n_ctr_pnts + i) = c(i, j);
                    lhs(n_ctr_pnts + i, j) = c(i, j);
                }
            }
        }
        if (make_closed) {
            const Matrix continuity_entries =
                get_continuity_matrix(n_ctr_pnts, n_continuity_conditions, params, flat_knots_array);
            const int start_row_lhs = n_ctr_pnts + n_interpolated;
            for (int i = 0; i < n_continuity_conditions; ++i) {
                for (int j = 0; j < n_ctr_pnts; ++j) {
                    lhs(start_row_lhs + i, j) = continuity_entries(i, j);
                    lhs(j, start_row_lhs + i) = continuity_entries(i, j);
                }
            }
        }
        for (int i = 0; i < n_interpolated + n_continuity_conditions; ++i) {
            for (int j = 0; j < 3; ++j) rhs(n_ctr_pnts + i, j) = d(i, j);
        }
    }
    for (int i = 0; i < n_vars; ++i) {
        for (int j = 0; j < n_vars; ++j) lhs(i, j) = lhs(i, j) + 1e-15 * (i == j ? 1.0 : 0.0);
    }
    const auto cp_full = linalg_solve(lhs, rhs);
    if (!cp_full) throw std::runtime_error("Singular Matrix");
    TColgp_Array1OfPnt poles(1, n_ctr_pnts);
    for (int i = 0; i < n_ctr_pnts; ++i) poles.SetValue(i + 1, gp_Pnt((*cp_full)(i, 0), (*cp_full)(i, 1), (*cp_full)(i, 2)));
    const Handle(Geom_BSplineCurve) result_curve = new Geom_BSplineCurve(poles, knots, mults, m_degree, false);
    // the largest error of the approximated points
    double max_error = 0.0;
    for (int idx : m_index_of_approximated) {
        const double error_val = result_curve->Value(params[static_cast<std::size_t>(idx)]).Distance(point_of(idx));
        max_error = std::max(max_error, error_val);
    }
    return {result_curve, max_error};
}

void BSplineApproxInterp::optimize_parameters(const Handle(Geom_Curve)& curve, std::vector<double>& params) const {
    // the approximated points move to the parameters of their projections
    for (int idx : m_index_of_approximated) {
        const auto [parameter, error] =
            project_on_curve(m_pnts(m_pnts.Lower() + idx), curve, params[static_cast<std::size_t>(idx)]);
        (void)error;
        params[static_cast<std::size_t>(idx)] = parameter;
    }
}

// ---------------------------------------------------------------------------
// PointsToBSplineInterpolation
// ---------------------------------------------------------------------------

PointsToBSplineInterpolation::PointsToBSplineInterpolation(const Handle(TColgp_HArray1OfPnt)& points,
                                                           std::optional<std::vector<double>> parameters, int max_degree,
                                                           bool continuous_if_closed)
    : m_pnts(points), m_degree(max_degree), m_c2_continuous(continuous_if_closed) {
    m_params = parameters ? *parameters : compute_params_bspline_curve(points->Array1());
    if (m_degree < 1) throw std::invalid_argument("Degree must be larger than 1 in PointsToBSplineInterpolation!");
    if (m_pnts.IsNull()) throw std::invalid_argument("No points given in PointsToBSplineInterpolation");
    if (m_pnts->Length() < 2) throw std::invalid_argument("Too few points in PointsToBSplineInterpolation");
    if (static_cast<int>(m_params.size()) != m_pnts->Length()) {
        throw std::invalid_argument("Number of parameters " + std::to_string(m_params.size()) + " and points " +
                                    std::to_string(m_pnts->Length()) + " don't match in PointsToBSplineInterpolation");
    }
}

int PointsToBSplineInterpolation::degree() const {
    int max_degree = m_pnts->Length() - 1;
    if (is_closed()) max_degree -= 1;
    const int degree = std::min(max_degree, m_degree);
    if (degree <= 0) throw std::invalid_argument("Invalid degree computed");
    return degree;
}

bool PointsToBSplineInterpolation::is_closed() const { return gordon::is_closed(m_pnts->Array1(), m_c2_continuous); }

bool PointsToBSplineInterpolation::needs_shifting() const { return (degree() % 2) == 0 && is_closed(); }

Handle(Geom_BSplineCurve) PointsToBSplineInterpolation::curve() const {
    const int degree = this->degree();
    std::vector<double> params = m_params;
    std::vector<double> knots = knots_from_curve_parameters(params, degree, is_closed());
    // a closed curve's last point is its first
    if (is_closed()) params.pop_back();
    const Handle(TColStd_HArray1OfReal) occ_knots = to_array(knots);
    const Handle(TColStd_HArray1OfReal) occ_params = to_array(params);
    const Matrix bspl_mat = bspline_basis_mat(degree, occ_knots->Array1(), occ_params->Array1());
    // the interpolation system
    const int n_params = static_cast<int>(params.size());
    Matrix lhs(n_params, n_params);
    for (int i = 0; i < n_params; ++i) {
        for (int j = 0; j < n_params; ++j) lhs(i, j) = bspl_mat(i, j);
    }
    if (is_closed()) {
        // the wrapped control points of a periodic curve
        for (int j = 0; j < degree; ++j) {
            for (int i = 0; i < n_params; ++i) lhs(i, j) += bspl_mat(i, n_params + j);
        }
    }
    Matrix rhs_pnts(n_params, 3);
    for (int i = 0; i < n_params; ++i) {
        const gp_Pnt& p = m_pnts->Value(i + 1);
        rhs_pnts(i, 0) = p.X();
        rhs_pnts(i, 1) = p.Y();
        rhs_pnts(i, 2) = p.Z();
    }
    const auto cp_coords = linalg_solve(lhs, rhs_pnts);
    if (!cp_coords) throw std::invalid_argument("Singular Matrix in linear system solution: Singular matrix");
    int n_ctrl_pnts = is_closed() ? static_cast<int>(m_params.size()) + degree - 1 : n_params;
    if (needs_shifting()) n_ctrl_pnts += 1;
    TColgp_Array1OfPnt poles(1, n_ctrl_pnts);
    for (int i = 0; i < n_params; ++i) poles.SetValue(i + 1, gp_Pnt((*cp_coords)(i, 0), (*cp_coords)(i, 1), (*cp_coords)(i, 2)));
    if (is_closed()) {
        // only `degree` points are wrapped
        for (int i = 0; i < degree; ++i) {
            poles.SetValue(n_params + i + 1, gp_Pnt((*cp_coords)(i, 0), (*cp_coords)(i, 1), (*cp_coords)(i, 2)));
        }
    }
    if (needs_shifting()) {
        // an additional knot and pole for the shifted parameters
        const int deg = degree;
        knots.push_back(knots.back() + knots[static_cast<std::size_t>(2 * deg + 1)] - knots[static_cast<std::size_t>(2 * deg)]);
        poles.SetValue(n_params + degree + 1, poles(degree + 1));
        for (double& knot : knots) knot -= params[0];
    }
    const Handle(TColStd_HArray1OfReal) shifted_knots = to_array(knots);
    const int knots_len = BSplCLib::KnotsLength(shifted_knots->Array1());
    TColStd_Array1OfReal occ_unique_knots(1, knots_len);
    TColStd_Array1OfInteger occ_multiplicities(1, knots_len);
    BSplCLib::Knots(shifted_knots->Array1(), occ_unique_knots, occ_multiplicities);
    Handle(Geom_BSplineCurve) result = new Geom_BSplineCurve(poles, occ_unique_knots, occ_multiplicities, degree, false);
    if (is_closed()) {
        // clamp to the first and last parameters
        const Handle(Geom_TrimmedCurve) c = new Geom_TrimmedCurve(result, m_params.front(), m_params.back());
        result = GeomConvert::CurveToBSplineCurve(c);
    }
    return result;
}

// ---------------------------------------------------------------------------
// CurvesToSurface
// ---------------------------------------------------------------------------

namespace {

/// curves_to_surface.clamp_bspline: a periodic curve made non-periodic and clamped.
Handle(Geom_BSplineCurve) clamp_bspline(const Handle(Geom_BSplineCurve)& curve) {
    if (!curve->IsPeriodic()) return {};
    curve->SetNotPeriodic();
    const double first_param = curve->FirstParameter();
    const double last_param = curve->LastParameter();
    const Handle(Geom_TrimmedCurve) trimmed_curve = new Geom_TrimmedCurve(curve, first_param, last_param);
    return GeomConvert::CurveToBSplineCurve(trimmed_curve);
}

}  // namespace

CurvesToSurface::CurvesToSurface(const std::vector<Handle(Geom_BSplineCurve)>& curves, std::vector<double> parameters,
                                 bool continuous_if_closed, double tolerance)
    : m_parameters(std::move(parameters)), m_continuous_if_closed(continuous_if_closed), m_tolerance(tolerance) {
    for (const auto& curve : curves) m_input_curves.push_back(GeomConvert::CurveToBSplineCurve(curve));
    match_degree(m_input_curves);
    if (m_parameters.empty()) calculate_parameters();
    if (m_compatible_splines.empty()) {
        m_compatible_splines = create_common_knots_vector_curve(m_input_curves, m_tolerance);
    }
}

Handle(Geom_BSplineSurface) CurvesToSurface::surface() {
    if (!m_has_performed) perform();
    if (m_skinned_surface.IsNull()) throw std::runtime_error("Surface could not be created.");
    return m_skinned_surface;
}

void CurvesToSurface::calculate_parameters() {
    if (m_input_curves.empty()) return;
    if (m_compatible_splines.empty()) {
        m_compatible_splines = create_common_knots_vector_curve(m_input_curves, m_tolerance);
    }
    // the parameters of the control points along the v direction
    const Handle(Geom_BSplineCurve)& first_curve = m_compatible_splines[0];
    const int num_poles_u = first_curve->NbPoles();
    const int num_splines = static_cast<int>(m_compatible_splines.size());
    TColgp_Array2OfPnt control_points_matrix(1, num_poles_u, 1, num_splines);
    for (int spline_idx = 1; spline_idx <= num_splines; ++spline_idx) {
        const auto& spline = m_compatible_splines[static_cast<std::size_t>(spline_idx - 1)];
        for (int pole_idx = 1; pole_idx <= num_poles_u; ++pole_idx) {
            control_points_matrix.SetValue(pole_idx, spline_idx, spline->Pole(pole_idx));
        }
    }
    m_parameters = compute_params_bspline_surf(control_points_matrix, 0.5).second;
}

void CurvesToSurface::perform() {
    if (m_input_curves.size() < 2) return;
    if (m_parameters.size() != m_input_curves.size()) {
        throw std::runtime_error("The amount of given parameters has to be equal to the amount of given B-splines!");
    }
    // closed in v if the first and last curves are the same
    const double tolerance = scale(m_input_curves) * REL_TOL_CLOSED;
    const bool make_closed = m_continuous_if_closed && !m_input_curves.empty() &&
                             m_input_curves.front()->IsEqual(m_input_curves.back(), tolerance);
    const int n_curves = static_cast<int>(m_input_curves.size());
    if (m_compatible_splines.empty()) {
        m_compatible_splines = create_common_knots_vector_curve(m_input_curves, m_tolerance);
    }
    const Handle(Geom_BSplineCurve)& first_curve = m_compatible_splines[0];
    const int num_control_points_u = first_curve->NbPoles();
    int degree_v = 0;
    const int degree_u = first_curve->Degree();
    std::optional<TColStd_Array1OfReal> knots_v;
    std::optional<TColStd_Array1OfInteger> mults_v;
    TColgp_Array2OfPnt cp_surf;
    // interpolate each row of control points in the v direction
    for (int cp_u_idx = 1; cp_u_idx <= num_control_points_u; ++cp_u_idx) {
        Handle(TColgp_HArray1OfPnt) interp_points_v_dir = new TColgp_HArray1OfPnt(1, n_curves);
        for (int cp_v_idx = 1; cp_v_idx <= n_curves; ++cp_v_idx) {
            interp_points_v_dir->SetValue(cp_v_idx, m_compatible_splines[static_cast<std::size_t>(cp_v_idx - 1)]->Pole(cp_u_idx));
        }
        const PointsToBSplineInterpolation interpolator(interp_points_v_dir, m_parameters, m_max_degree, make_closed);
        Handle(Geom_BSplineCurve) interp_spline = interpolator.curve();
        if (interp_spline.IsNull()) throw std::runtime_error("Interpolation failed for U index " + std::to_string(cp_u_idx));
        if (make_closed) {
            const Handle(Geom_BSplineCurve) clamped_spline = clamp_bspline(interp_spline);
            if (!clamped_spline.IsNull()) interp_spline = clamped_spline;
        }
        if (cp_u_idx == 1) {
            // the v knots of the surface
            degree_v = interp_spline->Degree();
            knots_v.emplace(1, interp_spline->NbKnots());
            mults_v.emplace(1, interp_spline->NbKnots());
            for (int i = 1; i <= interp_spline->NbKnots(); ++i) {
                knots_v->SetValue(i, interp_spline->Knot(i));
                mults_v->SetValue(i, interp_spline->Multiplicity(i));
            }
            cp_surf.Resize(1, num_control_points_u, 1, interp_spline->NbPoles(), false);
        }
        if (interp_spline->NbPoles() > cp_surf.UpperCol()) throw std::runtime_error("Standard_OutOfRange");
        for (int i = 1; i <= interp_spline->NbPoles(); ++i) cp_surf.SetValue(cp_u_idx, i, interp_spline->Pole(i));
        if (cp_u_idx > 1 && degree_v != interp_spline->Degree()) {
            throw std::runtime_error("Inconsistent degree_v found at U index " + std::to_string(cp_u_idx) + ". Expected " +
                                     std::to_string(degree_v) + ", got " + std::to_string(interp_spline->Degree()) + ".");
        }
    }
    // the u knots are the compatible splines' knots
    TColStd_Array1OfReal knots_u(1, first_curve->NbKnots());
    TColStd_Array1OfInteger mults_u(1, first_curve->NbKnots());
    for (int i = 1; i <= first_curve->NbKnots(); ++i) {
        knots_u.SetValue(i, first_curve->Knot(i));
        mults_u.SetValue(i, first_curve->Multiplicity(i));
    }
    if (!knots_v || !mults_v) throw std::runtime_error("Failed to obtain V direction knots and multiplicities.");
    m_skinned_surface = new Geom_BSplineSurface(cp_surf, knots_u, *knots_v, mults_u, *mults_v, degree_u, degree_v);
    m_has_performed = true;
}

}  // namespace build123d::gordon
