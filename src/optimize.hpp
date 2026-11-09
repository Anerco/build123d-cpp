// Ports of the scipy.optimize minimizers build123d uses (scipy 1.18, _optimize.py):
// minimize(method="Nelder-Mead") (_minimize_neldermead, used by DoubleTangentArc) and
// minimize_scalar(method="bounded") (_minimize_scalar_bounded, used by param_at_point).
#pragma once

#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace build123d::detail {

struct NelderMeadOptions {
    /// Bounds (lower, upper) per variable; the simplex is clipped to them, as in scipy.
    std::optional<std::vector<std::pair<double, double>>> bounds;
    /// minimize(tol=...) sets both xatol and fatol for Nelder-Mead.
    double xatol = 1e-4;
    double fatol = 1e-4;
    std::optional<int> maxiter;
    std::optional<int> maxfev;
    bool adaptive = false;
};

struct NelderMeadResult {
    std::vector<double> x;
    double fun = 0.0;
    int nit = 0;
    int nfev = 0;
    /// 0 success, 1 too many function evaluations, 2 too many iterations.
    int status = 0;
    bool success = false;
};

NelderMeadResult minimize_nelder_mead(const std::function<double(const std::vector<double>&)>& func,
                                      const std::vector<double>& x0, const NelderMeadOptions& options = {});

struct ScalarResult {
    double x = 0.0;
    double fun = 0.0;
    int nfev = 0;
    /// 0 success, 1 maximum number of function calls reached, 2 NaN result.
    int status = 0;
};

/// minimize_scalar(func, bounds=(x1, x2), method="bounded", options={"xatol": xatol}).
ScalarResult minimize_scalar_bounded(const std::function<double(double)>& func, double x1, double x2,
                                     double xatol = 1e-5, int maxiter = 500);

}  // namespace build123d::detail
