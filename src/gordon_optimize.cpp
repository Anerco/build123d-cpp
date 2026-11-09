// Gordon surfaces, part 2: the numpy/scipy routines ocp_gordon calls, ported from numpy
// (linalg.solve/inv/eigh/cholesky through LAPACK gesv/syevd/potrf) and scipy 1.18
// (optimize._optimize._minimize_bfgs with _linesearch.line_search_wolfe1/2 and
// _dcsrch.DCSRCH, optimize._optimize._minimize_scalar_bounded), and misc.math_BFGS.
#include <cmath>
#include <limits>
#include <string>
#include <tuple>

#include "gordon_internal.hpp"

namespace build123d::gordon {

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
/// LAPACK's dlamch('S') and dlamch('E') (eps of rounding).
constexpr double kSafeMin = std::numeric_limits<double>::min();
constexpr double kEps = std::numeric_limits<double>::epsilon() * 0.5;

/// Python's min(a, b) and max(a, b): the first argument unless the second compares less/greater.
double py_min(double a, double b) { return b < a ? b : a; }
double py_max(double a, double b) { return b > a ? b : a; }

/// numpy's sign, maximum and clip (NaN propagates).
double np_sign(double x) {
    if (std::isnan(x)) return kNan;
    return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0);
}
double np_maximum(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return kNan;
    return a > b ? a : b;
}
double np_minimum(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return kNan;
    return a < b ? a : b;
}
double np_clip(double x, double lo, double hi) { return np_minimum(np_maximum(x, lo), hi); }

double dot(const std::vector<double>& a, const std::vector<double>& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

/// xk + s * pk
std::vector<double> axpy(const std::vector<double>& xk, double s, const std::vector<double>& pk) {
    std::vector<double> out(xk.size());
    for (std::size_t i = 0; i < xk.size(); ++i) out[i] = xk[i] + s * pk[i];
    return out;
}

/// numpy.amax(numpy.abs(x)) (NaN propagates).
double norm_inf(const std::vector<double>& x) {
    double m = -kInf;
    for (double v : x) {
        if (std::isnan(v)) return kNan;
        m = std::max(m, std::abs(v));
    }
    return m;
}

double norm2(const std::vector<double>& x) { return std::sqrt(dot(x, x)); }

bool starts_with(const std::string& text, const char* prefix) { return text.rfind(prefix, 0) == 0; }

}  // namespace

// ---------------------------------------------------------------------------
// numpy.linalg
// ---------------------------------------------------------------------------

std::optional<Matrix> linalg_solve(Matrix a, Matrix b) {
    const int n = a.rows();
    if (a.cols() != n || b.rows() != n) throw std::invalid_argument("linalg_solve: incompatible shapes");
    // LU factorization with partial pivoting (getrf)
    for (int j = 0; j < n; ++j) {
        int p = j;
        double max_abs = std::abs(a(j, j));
        for (int i = j + 1; i < n; ++i) {
            if (std::abs(a(i, j)) > max_abs) {
                max_abs = std::abs(a(i, j));
                p = i;
            }
        }
        if (a(p, j) == 0.0) return std::nullopt;  // singular: numpy's LinAlgError
        if (p != j) {
            for (int k = 0; k < n; ++k) std::swap(a(p, k), a(j, k));
            for (int k = 0; k < b.cols(); ++k) std::swap(b(p, k), b(j, k));
        }
        const double pivot = a(j, j);
        if (std::abs(pivot) >= kSafeMin) {
            const double reciprocal = 1.0 / pivot;
            for (int i = j + 1; i < n; ++i) a(i, j) *= reciprocal;
        } else {
            for (int i = j + 1; i < n; ++i) a(i, j) /= pivot;
        }
        for (int i = j + 1; i < n; ++i) {
            const double l = a(i, j);
            for (int k = j + 1; k < n; ++k) a(i, k) -= l * a(j, k);
        }
    }
    // forward and back substitution (getrs)
    for (int c = 0; c < b.cols(); ++c) {
        for (int i = 0; i < n; ++i) {
            for (int k = 0; k < i; ++k) b(i, c) -= a(i, k) * b(k, c);
        }
        for (int i = n - 1; i >= 0; --i) {
            for (int k = n - 1; k > i; --k) b(i, c) -= a(i, k) * b(k, c);
            b(i, c) /= a(i, i);
        }
    }
    return b;
}

std::optional<Matrix> linalg_inv(const Matrix& a) { return linalg_solve(a, Matrix::identity(a.rows())); }

std::optional<std::pair<std::array<double, 2>, Matrix>> linalg_eigh_2x2(const Matrix& a) {
    // syevd -> steqr on the (already tridiagonal) lower triangle: d = diagonal, e = a(1, 0)
    double d1 = a(0, 0), d2 = a(1, 1), e = a(1, 0);
    Matrix z = Matrix::identity(2);
    const double tst = std::abs(e);
    const bool split = tst == 0.0 || tst <= (std::sqrt(std::abs(d1)) * std::sqrt(std::abs(d2))) * kEps;
    const double anorm = std::isnan(d1) || std::isnan(d2) || std::isnan(e)
                             ? kNan
                             : std::max({std::abs(d1), std::abs(d2), std::abs(e)});
    if (!split && anorm != 0.0 && !(e * e <= (kEps * kEps * std::abs(d1)) * std::abs(d2) + kSafeMin)) {
        // dlaev2: the eigenvalues rt1 (larger in magnitude), rt2 and the eigenvector (cs1, sn1) of rt1
        const double sm = d1 + d2;
        const double df = d1 - d2;
        const double adf = std::abs(df);
        const double tb = e + e;
        const double ab = std::abs(tb);
        double acmx, acmn;
        if (std::abs(d1) > std::abs(d2)) {
            acmx = d1;
            acmn = d2;
        } else {
            acmx = d2;
            acmn = d1;
        }
        double rt;
        if (adf > ab) {
            rt = adf * std::sqrt(1.0 + (ab / adf) * (ab / adf));
        } else if (adf < ab) {
            rt = ab * std::sqrt(1.0 + (adf / ab) * (adf / ab));
        } else {
            rt = ab * std::sqrt(2.0);
        }
        double rt1, rt2;
        int sgn1;
        if (sm < 0.0) {
            rt1 = 0.5 * (sm - rt);
            sgn1 = -1;
            rt2 = (acmx / rt1) * acmn - (e / rt1) * e;
        } else if (sm > 0.0) {
            rt1 = 0.5 * (sm + rt);
            sgn1 = 1;
            rt2 = (acmx / rt1) * acmn - (e / rt1) * e;
        } else {
            rt1 = 0.5 * rt;
            rt2 = -0.5 * rt;
            sgn1 = 1;
        }
        double cs;
        int sgn2;
        if (df >= 0.0) {
            cs = df + rt;
            sgn2 = 1;
        } else {
            cs = df - rt;
            sgn2 = -1;
        }
        double cs1, sn1;
        if (std::abs(cs) > ab) {
            const double ct = -tb / cs;
            sn1 = 1.0 / std::sqrt(1.0 + ct * ct);
            cs1 = ct * sn1;
        } else if (ab == 0.0) {
            cs1 = 1.0;
            sn1 = 0.0;
        } else {
            const double tn = -cs / tb;
            cs1 = 1.0 / std::sqrt(1.0 + tn * tn);
            sn1 = tn * cs1;
        }
        if (sgn1 == sgn2) {
            const double tn = cs1;
            cs1 = -sn1;
            sn1 = tn;
        }
        // dlasr: the plane rotation applied to the identity
        z(0, 0) = cs1;
        z(1, 0) = sn1;
        z(0, 1) = -sn1;
        z(1, 1) = cs1;
        d1 = rt1;
        d2 = rt2;
    }
    // eigenvalues in increasing order
    if (d2 < d1) {
        std::swap(d1, d2);
        std::swap(z(0, 0), z(0, 1));
        std::swap(z(1, 0), z(1, 1));
    }
    return std::make_pair(std::array<double, 2>{d1, d2}, z);
}

bool linalg_cholesky_ok(const Matrix& a) {
    const int n = a.rows();
    Matrix l(n, n);
    for (int j = 0; j < n; ++j) {
        double ajj = a(j, j);
        for (int k = 0; k < j; ++k) ajj -= l(j, k) * l(j, k);
        if (ajj <= 0.0 || std::isnan(ajj)) return false;
        l(j, j) = std::sqrt(ajj);
        for (int i = j + 1; i < n; ++i) {
            double s = a(i, j);
            for (int k = 0; k < j; ++k) s -= l(i, k) * l(j, k);
            l(i, j) = s / l(j, j);
        }
    }
    return true;
}

Matrix matmul(const Matrix& a, const Matrix& b) {
    Matrix out(a.rows(), b.cols());
    for (int i = 0; i < a.rows(); ++i) {
        for (int j = 0; j < b.cols(); ++j) {
            double s = 0.0;
            for (int k = 0; k < a.cols(); ++k) s += a(i, k) * b(k, j);
            out(i, j) = s;
        }
    }
    return out;
}

Matrix transpose(const Matrix& a) {
    Matrix out(a.cols(), a.rows());
    for (int i = 0; i < a.rows(); ++i) {
        for (int j = 0; j < a.cols(); ++j) out(j, i) = a(i, j);
    }
    return out;
}

// ---------------------------------------------------------------------------
// scipy.optimize: BFGS and its line searches
// ---------------------------------------------------------------------------

namespace {

/// scipy's ScalarFunction over MemoizeJac: the value and the gradient of the last x.
class ScalarFunction {
public:
    explicit ScalarFunction(const FunctionWithGradient& fun) : fun_(fun) {}
    double fun(const std::vector<double>& x) {
        update(x);
        return f_;
    }
    std::vector<double> grad(const std::vector<double>& x) {
        update(x);
        return g_;
    }

private:
    void update(const std::vector<double>& x) {
        if (has_value_ && x == x_) return;
        x_ = x;
        g_.assign(x.size(), 0.0);
        f_ = fun_(x_, g_);
        has_value_ = true;
    }
    const FunctionWithGradient& fun_;
    bool has_value_ = false;
    std::vector<double> x_;
    double f_ = 0.0;
    std::vector<double> g_;
};

using ScalarFn = std::function<double(double)>;

/// _dcsrch.dcstep: a safeguarded step for the line search (MINPACK-2).
std::tuple<double, double, double, double, double, double, double, bool> dcstep(double stx, double fx, double dx,
                                                                                double sty, double fy, double dy,
                                                                                double stp, double fp, double dp,
                                                                                bool brackt, double stpmin,
                                                                                double stpmax) {
    const double sgnd = np_sign(dp) * np_sign(dx);
    const auto max3 = [](double a, double b, double c) { return py_max(py_max(a, b), c); };
    double stpf;
    if (fp > fx) {
        // a higher function value: the minimum is bracketed
        const double theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;
        const double s = max3(std::abs(theta), std::abs(dx), std::abs(dp));
        double gamma = s * std::sqrt((theta / s) * (theta / s) - (dx / s) * (dp / s));
        if (stp < stx) gamma *= -1;
        const double p = (gamma - dx) + theta;
        const double q = ((gamma - dx) + gamma) + dp;
        const double r = p / q;
        const double stpc = stx + r * (stp - stx);
        const double stpq = stx + ((dx / ((fx - fp) / (stp - stx) + dx)) / 2.0) * (stp - stx);
        if (std::abs(stpc - stx) <= std::abs(stpq - stx)) {
            stpf = stpc;
        } else {
            stpf = stpc + (stpq - stpc) / 2.0;
        }
        brackt = true;
    } else if (sgnd < 0.0) {
        // derivatives of opposite sign: the minimum is bracketed
        const double theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        const double s = max3(std::abs(theta), std::abs(dx), std::abs(dp));
        double gamma = s * std::sqrt((theta / s) * (theta / s) - (dx / s) * (dp / s));
        if (stp > stx) gamma *= -1;
        const double p = (gamma - dp) + theta;
        const double q = ((gamma - dp) + gamma) + dx;
        const double r = p / q;
        const double stpc = stp + r * (stx - stp);
        const double stpq = stp + (dp / (dp - dx)) * (stx - stp);
        if (std::abs(stpc - stp) > std::abs(stpq - stp)) {
            stpf = stpc;
        } else {
            stpf = stpq;
        }
        brackt = true;
    } else if (std::abs(dp) < std::abs(dx)) {
        // derivatives of the same sign decreasing in magnitude
        const double theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        const double s = max3(std::abs(theta), std::abs(dx), std::abs(dp));
        double gamma = s * std::sqrt(py_max(0, (theta / s) * (theta / s) - (dx / s) * (dp / s)));
        if (stp > stx) gamma = -gamma;
        const double p = (gamma - dp) + theta;
        const double q = (gamma + (dx - dp)) + gamma;
        const double r = p / q;
        double stpc;
        if (r < 0 && gamma != 0) {
            stpc = stp + r * (stx - stp);
        } else if (stp > stx) {
            stpc = stpmax;
        } else {
            stpc = stpmin;
        }
        const double stpq = stp + (dp / (dp - dx)) * (stx - stp);
        if (brackt) {
            // the cubic step if it is closer to stp than the secant step, kept within the bracket
            if (std::abs(stpc - stp) < std::abs(stpq - stp)) {
                stpf = stpc;
            } else {
                stpf = stpq;
            }
            if (stp > stx) {
                stpf = py_min(stp + 0.66 * (sty - stp), stpf);
            } else {
                stpf = py_max(stp + 0.66 * (sty - stp), stpf);
            }
        } else {
            // the step farthest from stp, within the limits
            if (std::abs(stpc - stp) > std::abs(stpq - stp)) {
                stpf = stpc;
            } else {
                stpf = stpq;
            }
            stpf = np_clip(stpf, stpmin, stpmax);
        }
    } else {
        // derivatives of the same sign not decreasing in magnitude
        if (brackt) {
            const double theta = 3.0 * (fp - fy) / (sty - stp) + dy + dp;
            const double s = max3(std::abs(theta), std::abs(dy), std::abs(dp));
            double gamma = s * std::sqrt((theta / s) * (theta / s) - (dy / s) * (dp / s));
            if (stp > sty) gamma = -gamma;
            const double p = (gamma - dp) + theta;
            const double q = ((gamma - dp) + gamma) + dy;
            const double r = p / q;
            const double stpc = stp + r * (sty - stp);
            stpf = stpc;
        } else if (stp > stx) {
            stpf = stpmax;
        } else {
            stpf = stpmin;
        }
    }
    // the interval containing the minimizer
    if (fp > fx) {
        sty = stp;
        fy = fp;
        dy = dp;
    } else {
        if (sgnd < 0) {
            sty = stx;
            fy = fx;
            dy = dx;
        }
        stx = stp;
        fx = fp;
        dx = dp;
    }
    stp = stpf;
    return {stx, fx, dx, sty, fy, dy, stp, brackt};
}

/// _dcsrch.DCSRCH: the More'-Thuente line search of MINPACK-2.
class Dcsrch {
public:
    Dcsrch(ScalarFn phi, ScalarFn derphi, double ftol, double gtol, double xtol, double stpmin, double stpmax)
        : phi_(std::move(phi)),
          derphi_(std::move(derphi)),
          ftol_(ftol),
          gtol_(gtol),
          xtol_(xtol),
          stpmin_(stpmin),
          stpmax_(stpmax) {}

    /// (stp or nothing when failed, phi1, phi0, task)
    std::tuple<std::optional<double>, double, double, std::string> operator()(double alpha1, double phi0, double derphi0,
                                                                              int maxiter = 100) {
        double phi1 = phi0;
        double derphi1 = derphi0;
        std::string task = "START";
        std::optional<double> result;
        bool broke = false;
        for (int i = 0; i < maxiter; ++i) {
            double stp;
            std::tie(stp, phi1, derphi1, task) = iterate(alpha1, phi1, derphi1, task);
            if (!std::isfinite(stp)) {
                task = "WARN";
                result.reset();
                broke = true;
                break;
            }
            result = stp;
            if (starts_with(task, "FG")) {
                alpha1 = stp;
                phi1 = phi_(stp);
                derphi1 = derphi_(stp);
            } else {
                broke = true;
                break;
            }
        }
        if (!broke) {
            // maxiter reached, the line search did not converge
            result.reset();
            task = "WARNING: dcsrch did not converge within max iterations";
        }
        if (starts_with(task, "ERROR") || starts_with(task, "WARN")) result.reset();  // failed
        return {result, phi1, phi0, task};
    }

private:
    std::tuple<double, double, double, std::string> iterate(double stp, double f, double g, std::string task) {
        const double p5 = 0.5, p66 = 0.66, xtrapl = 1.1, xtrapu = 4.0;
        if (starts_with(task, "START")) {
            if (stp < stpmin_) task = "ERROR: STP .LT. STPMIN";
            if (stp > stpmax_) task = "ERROR: STP .GT. STPMAX";
            if (g >= 0) task = "ERROR: INITIAL G .GE. ZERO";
            if (ftol_ < 0) task = "ERROR: FTOL .LT. ZERO";
            if (gtol_ < 0) task = "ERROR: GTOL .LT. ZERO";
            if (xtol_ < 0) task = "ERROR: XTOL .LT. ZERO";
            if (stpmin_ < 0) task = "ERROR: STPMIN .LT. ZERO";
            if (stpmax_ < stpmin_) task = "ERROR: STPMAX .LT. STPMIN";
            if (starts_with(task, "ERROR")) return {stp, f, g, task};
            // the initial state
            brackt_ = false;
            stage_ = 1;
            finit_ = f;
            ginit_ = g;
            gtest_ = ftol_ * ginit_;
            width_ = stpmax_ - stpmin_;
            width1_ = width_ / p5;
            stx_ = 0.0;
            fx_ = finit_;
            gx_ = ginit_;
            sty_ = 0.0;
            fy_ = finit_;
            gy_ = ginit_;
            stmin_ = 0;
            stmax_ = stp + xtrapu * stp;
            return {stp, f, g, "FG"};
        }
        // the test value for the sufficient decrease
        const double ftest = finit_ + stp * gtest_;
        if (stage_ == 1 && f <= ftest && g >= 0) stage_ = 2;
        // warnings
        if (brackt_ && (stp <= stmin_ || stp >= stmax_)) task = "WARNING: ROUNDING ERRORS PREVENT PROGRESS";
        if (brackt_ && stmax_ - stmin_ <= xtol_ * stmax_) task = "WARNING: XTOL TEST SATISFIED";
        if (stp == stpmax_ && f <= ftest && g <= gtest_) task = "WARNING: STP = STPMAX";
        if (stp == stpmin_ && (f > ftest || g >= gtest_)) task = "WARNING: STP = STPMIN";
        // convergence
        if (f <= ftest && std::abs(g) <= gtol_ * -ginit_) task = "CONVERGENCE";
        if (starts_with(task, "WARN") || starts_with(task, "CONV")) return {stp, f, g, task};
        if (stage_ == 1 && f <= fx_ && f > ftest) {
            // the modified function values
            const double fm = f - stp * gtest_;
            double fxm = fx_ - stx_ * gtest_;
            double fym = fy_ - sty_ * gtest_;
            const double gm = g - gtest_;
            double gxm = gx_ - gtest_;
            double gym = gy_ - gtest_;
            std::tie(stx_, fxm, gxm, sty_, fym, gym, stp, brackt_) =
                dcstep(stx_, fxm, gxm, sty_, fym, gym, stp, fm, gm, brackt_, stmin_, stmax_);
            fx_ = fxm + stx_ * gtest_;
            fy_ = fym + sty_ * gtest_;
            gx_ = gxm + gtest_;
            gy_ = gym + gtest_;
        } else {
            std::tie(stx_, fx_, gx_, sty_, fy_, gy_, stp, brackt_) =
                dcstep(stx_, fx_, gx_, sty_, fy_, gy_, stp, f, g, brackt_, stmin_, stmax_);
        }
        // a bisection step if the interval does not shrink enough
        if (brackt_) {
            if (std::abs(sty_ - stx_) >= p66 * width1_) stp = stx_ + p5 * (sty_ - stx_);
            width1_ = width_;
            width_ = std::abs(sty_ - stx_);
        }
        // the minimum and maximum steps allowed
        if (brackt_) {
            stmin_ = py_min(stx_, sty_);
            stmax_ = py_max(stx_, sty_);
        } else {
            stmin_ = stp + xtrapl * (stp - stx_);
            stmax_ = stp + xtrapu * (stp - stx_);
        }
        stp = np_clip(stp, stpmin_, stpmax_);
        // the best step so far if no further progress is possible
        if ((brackt_ && (stp <= stmin_ || stp >= stmax_)) || (brackt_ && stmax_ - stmin_ <= xtol_ * stmax_)) {
            stp = stx_;
        }
        return {stp, f, g, "FG"};
    }

    ScalarFn phi_;
    ScalarFn derphi_;
    double ftol_, gtol_, xtol_, stpmin_, stpmax_;
    bool brackt_ = false;
    int stage_ = 0;
    double ginit_ = 0, gtest_ = 0, gx_ = 0, gy_ = 0, finit_ = 0, fx_ = 0, fy_ = 0, stx_ = 0, sty_ = 0, stmin_ = 0,
           stmax_ = 0, width_ = 0, width1_ = 0;
};

struct LineSearch {
    std::optional<double> alpha;
    double fval = 0.0;
    double old_fval = 0.0;
    std::optional<std::vector<double>> gradient;
};

/// _linesearch.line_search_wolfe1 (scalar_search_wolfe1 with DCSRCH).
LineSearch line_search_wolfe1(ScalarFunction& sf, const std::vector<double>& xk, const std::vector<double>& pk,
                              const std::vector<double>& gfk, double old_fval, double old_old_fval, double c1,
                              double c2, double amax, double amin, double xtol = 1e-14) {
    std::vector<double> gval = gfk;
    const ScalarFn phi = [&](double s) { return sf.fun(axpy(xk, s, pk)); };
    const ScalarFn derphi = [&](double s) {
        gval = sf.grad(axpy(xk, s, pk));
        return dot(gval, pk);
    };
    const double derphi0 = dot(gfk, pk);
    // scalar_search_wolfe1
    double alpha1;
    if (derphi0 != 0) {
        alpha1 = py_min(1.0, 1.01 * 2 * (old_fval - old_old_fval) / derphi0);
        if (alpha1 < 0) alpha1 = 1.0;
    } else {
        alpha1 = 1.0;
    }
    Dcsrch dcsrch(phi, derphi, c1, c2, xtol, amin, amax);
    const auto [stp, phi1, phi0, task] = dcsrch(alpha1, old_fval, derphi0, 100);
    return {stp, phi1, phi0, gval};
}

/// _linesearch._cubicmin: the minimizer of the cubic through (a, fa), (b, fb), (c, fc) with
/// slope fpa at a; nothing on a floating-point error (numpy errstate raise) or a non-finite result.
std::optional<double> cubicmin(double a, double fa, double fpa, double b, double fb, double c, double fc) {
    const auto finite = [](std::initializer_list<double> values) {
        for (double v : values) {
            if (!std::isfinite(v)) return false;
        }
        return true;
    };
    const double cc = fpa;
    const double db = b - a;
    const double dc = c - a;
    const double denom = (db * dc) * (db * dc) * (db - dc);
    const double d00 = dc * dc, d01 = -(db * db), d10 = -std::pow(dc, 3), d11 = std::pow(db, 3);
    const double v0 = fb - fa - cc * db, v1 = fc - fa - cc * dc;
    double aa = d00 * v0 + d01 * v1;
    double bb = d10 * v0 + d11 * v1;
    if (!finite({db, dc, denom, d00, d01, d10, d11, v0, v1, aa, bb}) || denom == 0.0) return std::nullopt;
    aa /= denom;
    bb /= denom;
    const double radical = bb * bb - 3 * aa * cc;
    if (!finite({aa, bb, radical}) || radical < 0 || aa == 0.0) return std::nullopt;
    const double xmin = a + (-bb + std::sqrt(radical)) / (3 * aa);
    if (!std::isfinite(xmin)) return std::nullopt;
    return xmin;
}

/// _linesearch._quadmin: the minimizer of the quadratic through (a, fa), (b, fb) with slope fpa at a.
std::optional<double> quadmin(double a, double fa, double fpa, double b, double fb) {
    const double d = fa;
    const double c = fpa;
    const double db = b - a * 1.0;
    const double denom = db * db;
    if (!std::isfinite(db) || !std::isfinite(denom) || denom == 0.0) return std::nullopt;
    const double bb = (fb - d - c * db) / denom;
    if (!std::isfinite(bb) || bb == 0.0) return std::nullopt;
    const double xmin = a - c / (2.0 * bb);
    if (!std::isfinite(xmin)) return std::nullopt;
    return xmin;
}

struct ScalarSearch {
    std::optional<double> alpha_star;
    double phi_star = 0.0;
    double phi0 = 0.0;
    std::optional<double> derphi_star;
};

/// _linesearch._zoom (no extra condition).
std::tuple<std::optional<double>, double, std::optional<double>> zoom(double a_lo, double a_hi, double phi_lo,
                                                                     double phi_hi, double derphi_lo, const ScalarFn& phi,
                                                                     const ScalarFn& derphi, double phi0,
                                                                     double derphi0, double c1, double c2) {
    const int maxiter = 10;
    int i = 0;
    const double delta1 = 0.2;  // cubic interpolant check
    const double delta2 = 0.1;  // quadratic interpolant check
    double phi_rec = phi0;
    double a_rec = 0;
    while (true) {
        const double dalpha = a_hi - a_lo;
        double a, b;
        if (dalpha < 0) {
            a = a_hi;
            b = a_lo;
        } else {
            a = a_lo;
            b = a_hi;
        }
        // cubic interpolation, else quadratic, else bisection
        std::optional<double> a_j;
        double cchk = 0.0;
        if (i > 0) {
            cchk = delta1 * dalpha;
            a_j = cubicmin(a_lo, phi_lo, derphi_lo, a_hi, phi_hi, a_rec, phi_rec);
        }
        if (i == 0 || !a_j || *a_j > b - cchk || *a_j < a + cchk) {
            const double qchk = delta2 * dalpha;
            a_j = quadmin(a_lo, phi_lo, derphi_lo, a_hi, phi_hi);
            if (!a_j || *a_j > b - qchk || *a_j < a + qchk) a_j = a_lo + 0.5 * dalpha;
        }
        const double phi_aj = phi(*a_j);
        if (phi_aj > phi0 + c1 * *a_j * derphi0 || phi_aj >= phi_lo) {
            phi_rec = phi_hi;
            a_rec = a_hi;
            a_hi = *a_j;
            phi_hi = phi_aj;
        } else {
            const double derphi_aj = derphi(*a_j);
            if (std::abs(derphi_aj) <= -c2 * derphi0) return {*a_j, phi_aj, derphi_aj};
            if (derphi_aj * (a_hi - a_lo) >= 0) {
                phi_rec = phi_hi;
                a_rec = a_hi;
                a_hi = a_lo;
                phi_hi = phi_lo;
            } else {
                phi_rec = phi_lo;
                a_rec = a_lo;
            }
            a_lo = *a_j;
            phi_lo = phi_aj;
            derphi_lo = derphi_aj;
        }
        i += 1;
        if (i > maxiter) return {std::nullopt, 0.0, std::nullopt};  // failed to find a conforming step
    }
}

/// _linesearch.scalar_search_wolfe2 (no extra condition).
ScalarSearch scalar_search_wolfe2(const ScalarFn& phi, const ScalarFn& derphi, double phi0, double old_phi0,
                                  double derphi0, double c1, double c2, std::optional<double> amax, int maxiter = 10) {
    double alpha0 = 0;
    double alpha1 = derphi0 != 0 ? py_min(1.0, 1.01 * 2 * (phi0 - old_phi0) / derphi0) : 1.0;
    if (alpha1 < 0) alpha1 = 1.0;
    if (amax) alpha1 = py_min(alpha1, *amax);
    double phi_a1 = phi(alpha1);
    double phi_a0 = phi0;
    double derphi_a0 = derphi0;
    for (int i = 0; i < maxiter; ++i) {
        if (alpha1 == 0 || (amax && alpha0 > *amax)) {
            // rounding errors prevent progress, or no solution up to amax
            return {std::nullopt, phi0, old_phi0, std::nullopt};
        }
        const bool not_first_iteration = i > 0;
        if (phi_a1 > phi0 + c1 * alpha1 * derphi0 || (phi_a1 >= phi_a0 && not_first_iteration)) {
            const auto [alpha_star, phi_star, derphi_star] =
                zoom(alpha0, alpha1, phi_a0, phi_a1, derphi_a0, phi, derphi, phi0, derphi0, c1, c2);
            return {alpha_star, phi_star, phi0, derphi_star};
        }
        const double derphi_a1 = derphi(alpha1);
        if (std::abs(derphi_a1) <= -c2 * derphi0) return {alpha1, phi_a1, phi0, derphi_a1};
        if (derphi_a1 >= 0) {
            const auto [alpha_star, phi_star, derphi_star] =
                zoom(alpha1, alpha0, phi_a1, phi_a0, derphi_a1, phi, derphi, phi0, derphi0, c1, c2);
            return {alpha_star, phi_star, phi0, derphi_star};
        }
        double alpha2 = 2 * alpha1;  // increase by a factor of two on each iteration
        if (amax) alpha2 = py_min(alpha2, *amax);
        alpha0 = alpha1;
        alpha1 = alpha2;
        phi_a0 = phi_a1;
        phi_a1 = phi(alpha1);
        derphi_a0 = derphi_a1;
    }
    // stopping test maxiter reached
    return {alpha1, phi_a1, phi0, std::nullopt};
}

/// _linesearch.line_search_wolfe2.
LineSearch line_search_wolfe2(ScalarFunction& sf, const std::vector<double>& xk, const std::vector<double>& pk,
                              const std::vector<double>& gfk, double old_fval, double old_old_fval, double c1,
                              double c2, std::optional<double> amax) {
    std::optional<std::vector<double>> gval;
    const ScalarFn phi = [&](double alpha) { return sf.fun(axpy(xk, alpha, pk)); };
    const ScalarFn derphi = [&](double alpha) {
        gval = sf.grad(axpy(xk, alpha, pk));
        return dot(*gval, pk);
    };
    const double derphi0 = dot(gfk, pk);
    const ScalarSearch search = scalar_search_wolfe2(phi, derphi, old_fval, old_old_fval, derphi0, c1, c2, amax);
    LineSearch result{search.alpha_star, search.phi_star, search.phi0, std::nullopt};
    // the gradient at the step is the last one computed
    if (search.derphi_star) result.gradient = gval;
    return result;
}

void check_positive_definite(const Matrix& hk) {
    const int n = hk.rows();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!(hk(i, j) == hk(j, i))) throw std::invalid_argument("'hess_inv0' matrix isn't positive definite.");
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!std::isfinite(hk(i, j))) throw std::invalid_argument("array must not contain infs or NaNs");
        }
    }
    if (!linalg_cholesky_ok(hk)) throw std::invalid_argument("'hess_inv0' matrix isn't positive definite.");
}

}  // namespace

OptimizeResult minimize_bfgs(const FunctionWithGradient& fun, const std::vector<double>& x0, double gtol,
                             const std::optional<Matrix>& hess_inv0) {
    const double c1 = 1e-4, c2 = 0.9, xrtol = 0.0;
    if (hess_inv0) check_positive_definite(*hess_inv0);
    const int n = static_cast<int>(x0.size());
    const int maxiter = n * 200;
    ScalarFunction sf(fun);
    double old_fval = sf.fun(x0);
    std::vector<double> gfk = sf.grad(x0);
    int k = 0;
    const Matrix identity = Matrix::identity(n);
    Matrix hk = hess_inv0 ? *hess_inv0 : identity;
    // the initial step guess to dx ~ 1
    double old_old_fval = old_fval + norm2(gfk) / 2;
    std::vector<double> xk = x0;
    int warnflag = 0;
    double gnorm = norm_inf(gfk);
    while (gnorm > gtol && k < maxiter) {
        std::vector<double> pk(static_cast<std::size_t>(n), 0.0);
        for (int i = 0; i < n; ++i) {
            double s = 0.0;
            for (int j = 0; j < n; ++j) s += hk(i, j) * gfk[static_cast<std::size_t>(j)];
            pk[static_cast<std::size_t>(i)] = -s;
        }
        // _line_search_wolfe12: wolfe1, then wolfe2 if it failed
        LineSearch ls = line_search_wolfe1(sf, xk, pk, gfk, old_fval, old_old_fval, c1, c2, 1e100, 1e-100);
        if (!ls.alpha) ls = line_search_wolfe2(sf, xk, pk, gfk, old_fval, old_old_fval, c1, c2, 1e100);
        if (!ls.alpha) {
            // the line search failed to find a better solution
            warnflag = 2;
            break;
        }
        const double alpha_k = *ls.alpha;
        old_fval = ls.fval;
        old_old_fval = ls.old_fval;
        std::vector<double> sk(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) sk[static_cast<std::size_t>(i)] = alpha_k * pk[static_cast<std::size_t>(i)];
        std::vector<double> xkp1(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            xkp1[static_cast<std::size_t>(i)] = xk[static_cast<std::size_t>(i)] + sk[static_cast<std::size_t>(i)];
        }
        xk = xkp1;
        const std::vector<double> gfkp1 = ls.gradient ? *ls.gradient : sf.grad(xkp1);
        std::vector<double> yk(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            yk[static_cast<std::size_t>(i)] = gfkp1[static_cast<std::size_t>(i)] - gfk[static_cast<std::size_t>(i)];
        }
        gfk = gfkp1;
        k += 1;
        gnorm = norm_inf(gfk);
        if (gnorm <= gtol) break;
        if (alpha_k * norm2(pk) <= xrtol * (xrtol + norm2(xk))) break;
        if (!std::isfinite(old_fval)) {
            // +-Inf found as the optimal value, or something went wrong
            warnflag = 2;
            break;
        }
        const double rhok_inv = dot(yk, sk);
        const double rhok = rhok_inv == 0.0 ? 1000.0 : 1.0 / rhok_inv;
        Matrix a1(n, n), a2(n, n), ss(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                const auto ui = static_cast<std::size_t>(i), uj = static_cast<std::size_t>(j);
                a1(i, j) = identity(i, j) - sk[ui] * yk[uj] * rhok;
                a2(i, j) = identity(i, j) - yk[ui] * sk[uj] * rhok;
                ss(i, j) = rhok * sk[ui] * sk[uj];
            }
        }
        const Matrix updated = matmul(a1, matmul(hk, a2));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) hk(i, j) = updated(i, j) + ss(i, j);
        }
    }
    const double fval = old_fval;
    if (warnflag != 2) {
        if (k >= maxiter) {
            warnflag = 1;
        } else {
            bool nan_x = false;
            for (double v : xk) nan_x = nan_x || std::isnan(v);
            if (std::isnan(gnorm) || std::isnan(fval) || nan_x) warnflag = 3;
        }
    }
    return {xk, fval, warnflag == 0};
}

ScalarResult minimize_scalar_bounded(const std::function<double(double)>& func, double x1, double x2, double xatol,
                                     int maxiter) {
    const int maxfun = maxiter;
    if (!(std::isfinite(x1) && std::isfinite(x2))) throw std::invalid_argument("Optimization bounds must be finite scalars.");
    if (x1 > x2) throw std::invalid_argument("The lower bound exceeds the upper bound.");
    int flag = 0;
    const double sqrt_eps = std::sqrt(2.2e-16);
    const double golden_mean = 0.5 * (3.0 - std::sqrt(5.0));
    double a = x1, b = x2;
    double fulc = a + golden_mean * (b - a);
    double nfc = fulc, xf = fulc;
    double rat = 0.0, e = 0.0;
    double x = xf;
    double fx = func(x);
    int num = 1;
    double fu = kInf;
    double ffulc = fx, fnfc = fx;
    double xm = 0.5 * (a + b);
    double tol1 = sqrt_eps * std::abs(xf) + xatol / 3.0;
    double tol2 = 2.0 * tol1;
    while (std::abs(xf - xm) > (tol2 - 0.5 * (b - a))) {
        bool golden = true;
        // check for a parabolic fit
        if (std::abs(e) > tol1) {
            golden = false;
            double r = (xf - nfc) * (fx - ffulc);
            double q = (xf - fulc) * (fx - fnfc);
            double p = (xf - fulc) * q - (xf - nfc) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = std::abs(q);
            r = e;
            e = rat;
            // is the parabola acceptable?
            if (std::abs(p) < std::abs(0.5 * q * r) && p > q * (a - xf) && p < q * (b - xf)) {
                rat = (p + 0.0) / q;
                x = xf + rat;
                if ((x - a) < tol2 || (b - x) < tol2) {
                    const double si = np_sign(xm - xf) + ((xm - xf) == 0 ? 1.0 : 0.0);
                    rat = tol1 * si;
                }
            } else {
                golden = true;
            }
        }
        if (golden) {
            // a golden-section step
            e = xf >= xm ? a - xf : b - xf;
            rat = golden_mean * e;
        }
        const double si = np_sign(rat) + (rat == 0 ? 1.0 : 0.0);
        x = xf + si * np_maximum(std::abs(rat), tol1);
        fu = func(x);
        num += 1;
        if (fu <= fx) {
            if (x >= xf) {
                a = xf;
            } else {
                b = xf;
            }
            fulc = nfc;
            ffulc = fnfc;
            nfc = xf;
            fnfc = fx;
            xf = x;
            fx = fu;
        } else {
            if (x < xf) {
                a = x;
            } else {
                b = x;
            }
            if (fu <= fnfc || nfc == xf) {
                fulc = nfc;
                ffulc = fnfc;
                nfc = x;
                fnfc = fu;
            } else if (fu <= ffulc || fulc == xf || fulc == nfc) {
                fulc = x;
                ffulc = fu;
            }
        }
        xm = 0.5 * (a + b);
        tol1 = sqrt_eps * std::abs(xf) + xatol / 3.0;
        tol2 = 2.0 * tol1;
        if (num >= maxfun) {
            flag = 1;
            break;
        }
    }
    if (std::isnan(xf) || std::isnan(fx) || std::isnan(fu)) flag = 2;
    return {xf, fx, flag == 0};
}

// ---------------------------------------------------------------------------
// misc.math_BFGS
// ---------------------------------------------------------------------------

namespace {

/// misc.math_BFGS.estimate_initial_inv_hessian: a regularized, symmetric positive definite
/// inverse of the central-difference Hessian at x0.
Matrix estimate_initial_inv_hessian(const FunctionWithGradient& f, const std::vector<double>& x0, double rel_eps = 1e-8,
                                    double reg = 1e-8) {
    const int n = static_cast<int>(x0.size());
    if (n != 2) throw std::runtime_error("This routine expects 2 parameters; adapt if you have more.");
    Matrix h(n, n);
    // central differences for the Jacobian of the gradient
    for (int j = 0; j < n; ++j) {
        const auto uj = static_cast<std::size_t>(j);
        const double eps = rel_eps * py_max(1.0, std::abs(x0[uj]));
        std::vector<double> plus(x0), minus(x0);
        for (int k = 0; k < n; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            const double step = k == j ? eps : 0.0;
            plus[uk] = x0[uk] + step;
            minus[uk] = x0[uk] - step;
        }
        std::vector<double> g_plus(static_cast<std::size_t>(n), 0.0), g_minus(static_cast<std::size_t>(n), 0.0);
        (void)f(plus, g_plus);
        (void)f(minus, g_minus);
        for (int i = 0; i < n; ++i) {
            const auto ui = static_cast<std::size_t>(i);
            h(i, j) = (g_plus[ui] - g_minus[ui]) / (2.0 * eps);
        }
    }
    // symmetrize
    Matrix hs(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) hs(i, j) = 0.5 * (h(i, j) + h(j, i));
    }
    h = hs;
    // regularize with a multiple of the identity scaled like the diagonal
    const double max_diag = std::isnan(h(0, 0)) || std::isnan(h(1, 1)) ? kNan : std::max(std::abs(h(0, 0)), std::abs(h(1, 1)));
    const double diag_scale = py_max(max_diag, 1.0);
    Matrix h_reg(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) h_reg(i, j) = h(i, j) + (i == j ? 1.0 : 0.0) * (reg * diag_scale);
    }
    const auto avg_curv = [&] { return py_max(1e-8, (std::abs(h(0, 0)) + std::abs(h(1, 1))) / 2.0); };
    const auto scaled_identity = [&](double factor) {
        Matrix m(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) m(i, j) = (i == j ? 1.0 : 0.0) * factor;
        }
        return m;
    };
    std::optional<Matrix> inv = linalg_inv(h_reg);
    Matrix inv_h = inv ? *inv : scaled_identity(1.0 / avg_curv());
    Matrix inv_sym(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) inv_sym(i, j) = 0.5 * (inv_h(i, j) + inv_h(j, i));
    }
    // project onto the symmetric positive definite matrices
    const auto eig = linalg_eigh_2x2(inv_sym);
    if (eig) {
        const auto& [eigvals, eigvecs] = *eig;
        Matrix diag(n, n);
        for (int i = 0; i < n; ++i) diag(i, i) = np_maximum(eigvals[static_cast<std::size_t>(i)], rel_eps);
        const Matrix spd = matmul(matmul(eigvecs, diag), transpose(eigvecs));
        Matrix spd_sym(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) spd_sym(i, j) = 0.5 * (spd(i, j) + spd(j, i));
        }
        if (linalg_cholesky_ok(spd_sym)) return spd_sym;
    }
    // the projection failed: fall back to a scaled identity
    return scaled_identity(1.0 / avg_curv());
}

}  // namespace

bool math_bfgs(const MultipleVarFunctionWithGradient& func, std::vector<double>& x, double tolerance) {
    const int nb_variables = func.nb_variables();
    const FunctionWithGradient f = [&](const std::vector<double>& args, std::vector<double>& grad) {
        double value = 0.0;
        std::vector<double> g(static_cast<std::size_t>(nb_variables), 0.0);
        if (!func.values(args, value, g)) throw std::runtime_error("function cannot evaluate");
        grad = g;
        return value;
    };
    const std::vector<double> x0 = x;
    const Matrix h0 = estimate_initial_inv_hessian(f, x0);
    const OptimizeResult res = minimize_bfgs(f, x0, tolerance, h0);
    x = res.x;
    return res.success;
}

}  // namespace build123d::gordon
