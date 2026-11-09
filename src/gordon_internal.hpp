// Gordon surfaces (Face::make_gordon_surface): a C++ port of the ocp_gordon package
// (ocp_gordon/internal/*.py, itself a Python port of TiGL's curve network interpolation)
// with the numpy/scipy routines it relies on. Internal to the library.
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_HArray1OfPnt.hxx>

#include "build123d/geometry.hpp"

namespace build123d::gordon {

// BSplineAlgorithms constants
inline constexpr double REL_TOL_CLOSED = 1e-8;
inline constexpr double PAR_CHECK_TOL = 1e-5;

/// A dense row-major matrix of doubles (numpy's 2-D float64 arrays).
class Matrix {
public:
    Matrix() = default;
    Matrix(int rows, int cols, double value = 0.0)
        : rows_(rows), cols_(cols), data_(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), value) {}
    double& operator()(int i, int j) { return data_[index(i, j)]; }
    double operator()(int i, int j) const { return data_[index(i, j)]; }
    int rows() const { return rows_; }
    int cols() const { return cols_; }
    static Matrix identity(int n) {
        Matrix m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }

private:
    std::size_t index(int i, int j) const {
        return static_cast<std::size_t>(i) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(j);
    }
    int rows_ = 0;
    int cols_ = 0;
    std::vector<double> data_;
};

// ---- numpy.linalg / scipy.optimize (gordon_optimize.cpp)

/// numpy.linalg.solve: LU with partial pivoting (LAPACK gesv); std::nullopt for a singular matrix
/// (numpy's LinAlgError).
std::optional<Matrix> linalg_solve(Matrix a, Matrix b);
/// numpy.linalg.inv.
std::optional<Matrix> linalg_inv(const Matrix& a);
/// numpy.linalg.eigh of a symmetric 2x2 matrix (LAPACK syevd: eigenvalues ascending, the
/// eigenvectors in the columns).
std::optional<std::pair<std::array<double, 2>, Matrix>> linalg_eigh_2x2(const Matrix& a);
/// Does numpy.linalg.cholesky succeed (is the symmetric matrix positive definite)?
bool linalg_cholesky_ok(const Matrix& a);
/// a @ b.
Matrix matmul(const Matrix& a, const Matrix& b);
Matrix transpose(const Matrix& a);

/// A function of several variables with its gradient: returns f(x) and fills grad.
using FunctionWithGradient = std::function<double(const std::vector<double>& x, std::vector<double>& grad)>;

struct OptimizeResult {
    std::vector<double> x;
    double fun = 0.0;
    bool success = false;
};

/// scipy.optimize.minimize(fun, x0, jac=True, method="BFGS", tol=gtol,
/// options={"hess_inv0": hess_inv0}).
OptimizeResult minimize_bfgs(const FunctionWithGradient& fun, const std::vector<double>& x0, double gtol,
                             const std::optional<Matrix>& hess_inv0);

struct ScalarResult {
    double x = 0.0;
    double fun = 0.0;
    bool success = false;
};

/// scipy.optimize.minimize_scalar(func, bounds=(x1, x2), method="Bounded",
/// options={"xatol": xatol}).
ScalarResult minimize_scalar_bounded(const std::function<double(double)>& func, double x1, double x2, double xatol,
                                     int maxiter = 500);

// ---- misc.py

Handle(Geom_BSplineCurve) clone_bspline(const Handle(Geom_BSplineCurve)& spline);
Handle(Geom_BSplineSurface) clone_bspline_surface(const Handle(Geom_BSplineSurface)& surface);

/// misc.math_MultipleVarFunctionWithGradient: Values(X, F, G) for X(1..2).
class MultipleVarFunctionWithGradient {
public:
    virtual ~MultipleVarFunctionWithGradient() = default;
    virtual int nb_variables() const { return 2; }
    /// Fills f and g (1-based math_Vector order in the Python: g[0] is G(1)).
    virtual bool values(const std::vector<double>& x, double& f, std::vector<double>& g) const = 0;
};

/// misc.math_BFGS: minimizes from x (updated in place) with scipy's BFGS started from an
/// estimated inverse Hessian; returns res.success.
bool math_bfgs(const MultipleVarFunctionWithGradient& func, std::vector<double>& x, double tolerance);

// ---- bspline_algorithms.py (gordon_bspline.cpp)

struct ApproxResult {
    Handle(Geom_BSplineCurve) curve;
    double error = 0.0;
};

std::vector<double> linspace_with_breaks(double umin, double umax, int n_values, const std::vector<double>& breaks);
bool is_u_dir_closed(const TColgp_Array2OfPnt& points, double tolerance);
bool is_v_dir_closed(const TColgp_Array2OfPnt& points, double tolerance);
std::vector<double> get_kink_parameters(const Handle(Geom_BSplineCurve)& curve);
std::vector<double> compute_params_bspline_curve(const TColgp_Array1OfPnt& points, double alpha = 0.5);
std::pair<std::vector<double>, std::vector<double>> compute_params_bspline_surf(const TColgp_Array2OfPnt& points,
                                                                                double alpha = 0.5);
void match_degree(const std::vector<Handle(Geom_BSplineCurve)>& bsplines);
std::vector<Handle(Geom_BSplineCurve)> create_common_knots_vector_curve(
    const std::vector<Handle(Geom_BSplineCurve)>& splines_vector, double tol);
void reparametrize_bspline(const Handle(Geom_BSplineCurve)& spline, double umin, double umax, double tol = 1e-15);
std::vector<Handle(Geom_BSplineCurve)> to_bsplines(const std::vector<Handle(Geom_Curve)>& curves);
std::vector<Handle(Geom_BSplineCurve)> to_bsplines(const std::vector<Handle(Geom_BSplineCurve)>& curves);
/// BSplineAlgorithms.scale of a curve, a list of curves and an Array2 of points.
double scale(const Handle(Geom_BSplineCurve)& spline);
double scale(const std::vector<Handle(Geom_BSplineCurve)>& splines_vector);
double scale(const TColgp_Array2OfPnt& points);
ApproxResult reparametrize_bspline_continuously_approx(const Handle(Geom_BSplineCurve)& spline,
                                                       const std::vector<double>& old_parameters,
                                                       const std::vector<double>& new_parameters, int n_control_pnts);
/// The knots for interpolating at `params`; may shift `params` (closed, even degree) like the Python.
std::vector<double> knots_from_curve_parameters(std::vector<double>& params, int degree, bool closed_curve);
Matrix bspline_basis_mat(int degree, const TColStd_Array1OfReal& knots, const TColStd_Array1OfReal& params,
                         int deriv_order = 0);
double max_distance_of_bounding_box(const TColgp_Array1OfPnt& points);
bool is_closed(const TColgp_Array1OfPnt& points, bool c2_continuous = false);
Handle(TColStd_HArray1OfReal) to_array(const std::vector<double>& vector);
Handle(TColgp_HArray1OfPnt) pnt_array2_get_column(const TColgp_Array2OfPnt& matrix, int col_index);
Handle(TColgp_HArray1OfPnt) pnt_array2_get_row(const TColgp_Array2OfPnt& matrix, int row_index);
Handle(Geom_BSplineSurface) flip_surface(const Handle(Geom_BSplineSurface)& surface);
void insert_knot_with_multiplicity(double knot, int count, int degree, std::vector<double>& knots,
                                   std::vector<int>& mults, double tol = 1e-5);

// ---- bspline_approx_interp.py (gordon_approx.cpp)

/// Approximation of points by a B-spline with some points interpolated exactly.
class BSplineApproxInterp {
public:
    BSplineApproxInterp(const TColgp_Array1OfPnt& points, int n_control_points, int degree = 3,
                        bool continuous_if_closed = false);
    void interpolate_point(int point_index, bool with_kink = false);
    /// Fits with parameters optimized by projection; `params` (empty: computed) is updated.
    ApproxResult fit_curve_optimal(std::vector<double>& params, int max_iter = 10);

private:
    bool is_closed() const;
    bool first_and_last_interpolated() const;
    std::vector<double> compute_parameters(double alpha) const;
    std::pair<std::vector<double>, std::vector<int>> compute_knots(int ncp, const std::vector<double>& params) const;
    std::pair<double, double> project_on_curve(const gp_Pnt& pnt, const Handle(Geom_Curve)& curve,
                                               double initial_param) const;
    Matrix get_continuity_matrix(int n_ctr_pnts, int contin_cons, const std::vector<double>& params,
                                 const TColStd_Array1OfReal& flat_knots) const;
    ApproxResult solve(const std::vector<double>& params, const TColStd_Array1OfReal& knots,
                       const TColStd_Array1OfInteger& mults) const;
    void optimize_parameters(const Handle(Geom_Curve)& curve, std::vector<double>& params) const;

    TColgp_Array1OfPnt m_pnts;
    std::vector<int> m_index_of_approximated;
    std::vector<int> m_index_of_interpolated;
    std::vector<int> m_index_of_kinks;
    int m_degree;
    int m_ncp;
    bool m_c2_continuous;
};

// ---- points_to_bspline_interpolation.py, curves_to_surface.py (gordon_approx.cpp)

/// Interpolation of points by a B-spline at given parameters.
class PointsToBSplineInterpolation {
public:
    PointsToBSplineInterpolation(const Handle(TColgp_HArray1OfPnt)& points, std::optional<std::vector<double>> parameters,
                                 int max_degree = 3, bool continuous_if_closed = false);
    Handle(Geom_BSplineCurve) curve() const;

private:
    int degree() const;
    bool is_closed() const;
    bool needs_shifting() const;

    Handle(TColgp_HArray1OfPnt) m_pnts;
    std::vector<double> m_params;
    int m_degree;
    bool m_c2_continuous;
};

/// Skinning of compatible B-splines into a surface (interpolation of their poles).
class CurvesToSurface {
public:
    CurvesToSurface(const std::vector<Handle(Geom_BSplineCurve)>& curves, std::vector<double> parameters,
                    bool continuous_if_closed = false, double tolerance = 1e-14);
    Handle(Geom_BSplineSurface) surface();

private:
    void calculate_parameters();
    void perform();

    std::vector<double> m_parameters;
    bool m_continuous_if_closed;
    double m_tolerance;
    int m_max_degree = 3;
    bool m_has_performed = false;
    Handle(Geom_BSplineSurface) m_skinned_surface;
    std::vector<Handle(Geom_BSplineCurve)> m_input_curves;
    std::vector<Handle(Geom_BSplineCurve)> m_compatible_splines;
};

// ---- intersect_bsplines.py (gordon_intersect.cpp)

struct BSplineIntersection {
    double parm_on_curve1;
    double parm_on_curve2;
};

/// IntersectBSplines: the intersections of two B-splines within `tolerance`.
std::vector<BSplineIntersection> intersect_bsplines(const Handle(Geom_BSplineCurve)& curve1,
                                                    const Handle(Geom_BSplineCurve)& curve2, double tolerance = 1e-5);

// ---- interpolate_curve_network.py (gordon.cpp)

/// interpolate_curve_network: the Gordon surface of profiles (u curves) and guides (v curves).
Handle(Geom_BSplineSurface) interpolate_curve_network(const std::vector<Handle(Geom_Curve)>& ucurves,
                                                      const std::vector<Handle(Geom_Curve)>& vcurves,
                                                      double tolerance = 3e-4);

}  // namespace build123d::gordon
