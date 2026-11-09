// A port of scipy.optimize.minimize(method="Nelder-Mead") (scipy 1.18,
// scipy/optimize/_optimize.py: _minimize_neldermead), used by DoubleTangentArc.
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

}  // namespace build123d::detail
