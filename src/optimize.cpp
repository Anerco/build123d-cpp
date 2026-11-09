// Ports of scipy.optimize._optimize._minimize_neldermead and _minimize_scalar_bounded
// (scipy 1.18), step by step: the same initial points, update order and stopping tests.
#include "optimize.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "build123d/warnings.hpp"

namespace build123d::detail {

namespace {

using Point = std::vector<double>;

struct MaxFuncCallError {};

/// np.clip(x, lower, upper)
Point clip(Point x, const std::vector<double>& lower, const std::vector<double>& upper) {
    for (std::size_t i = 0; i < x.size(); ++i) x[i] = std::min(std::max(x[i], lower[i]), upper[i]);
    return x;
}

/// np.max, which propagates NaN.
double nan_max(const std::vector<double>& values) {
    double m = -std::numeric_limits<double>::infinity();
    for (double v : values) {
        if (std::isnan(v)) return v;
        m = std::max(m, v);
    }
    return m;
}

/// np.argsort for the (small) simplex: equal values keep their order.
std::vector<std::size_t> argsort(const std::vector<double>& values) {
    std::vector<std::size_t> ind(values.size());
    std::iota(ind.begin(), ind.end(), 0);
    std::stable_sort(ind.begin(), ind.end(), [&](std::size_t a, std::size_t b) { return values[a] < values[b]; });
    return ind;
}

}  // namespace

NelderMeadResult minimize_nelder_mead(const std::function<double(const std::vector<double>&)>& user_func,
                                      const std::vector<double>& x0_in, const NelderMeadOptions& options) {
    Point x0 = x0_in;
    const std::size_t N = x0.size();
    if (N == 0) throw std::invalid_argument("x0 must not be empty");

    double rho, chi, psi, sigma;
    if (options.adaptive) {
        const double dim = static_cast<double>(N);
        rho = 1;
        chi = 1 + 2 / dim;
        psi = 0.75 - 1 / (2 * dim);
        sigma = 1 - 1 / dim;
    } else {
        rho = 1;
        chi = 2;
        psi = 0.5;
        sigma = 0.5;
    }
    const double nonzdelt = 0.05;
    const double zdelt = 0.00025;

    std::vector<double> lower_bound, upper_bound;
    const bool has_bounds = options.bounds.has_value();
    if (has_bounds) {
        if (options.bounds->size() != N) {
            throw std::invalid_argument("The number of bounds is not compatible with the length of `x0`.");
        }
        for (const auto& b : *options.bounds) {
            lower_bound.push_back(b.first);
            upper_bound.push_back(b.second);
        }
        for (std::size_t i = 0; i < N; ++i) {
            if (lower_bound[i] > upper_bound[i]) {
                throw std::invalid_argument("Nelder Mead - one of the lower bounds is greater than an upper bound.");
            }
        }
        for (std::size_t i = 0; i < N; ++i) {
            if (lower_bound[i] > x0[i] || x0[i] > upper_bound[i]) {
                warn("Initial guess is not within the specified bounds");
                break;
            }
        }
        x0 = clip(x0, lower_bound, upper_bound);
    }

    std::vector<Point> sim(N + 1, Point(N));
    sim[0] = x0;
    for (std::size_t k = 0; k < N; ++k) {
        Point y = x0;
        if (y[k] != 0) {
            y[k] = (1 + nonzdelt) * y[k];
        } else {
            y[k] = zdelt;
        }
        sim[k + 1] = y;
    }

    // If neither are set, then set both to default
    const double inf = std::numeric_limits<double>::infinity();
    double maxiter, maxfun;
    if (!options.maxiter && !options.maxfev) {
        maxiter = static_cast<double>(N * 200);
        maxfun = static_cast<double>(N * 200);
    } else if (!options.maxiter) {
        maxfun = *options.maxfev;
        maxiter = maxfun == inf ? static_cast<double>(N * 200) : inf;
    } else if (!options.maxfev) {
        maxiter = *options.maxiter;
        maxfun = maxiter == inf ? static_cast<double>(N * 200) : inf;
    } else {
        maxiter = *options.maxiter;
        maxfun = *options.maxfev;
    }

    if (has_bounds) {
        // Reflect entries above an upper bound into the interior, then clip.
        for (auto& vertex : sim) {
            for (std::size_t i = 0; i < N; ++i) {
                if (vertex[i] > upper_bound[i]) vertex[i] = 2 * upper_bound[i] - vertex[i];
            }
            vertex = clip(vertex, lower_bound, upper_bound);
        }
    }

    std::vector<double> fsim(N + 1, inf);
    int fcalls = 0;
    auto func = [&](const Point& x) {
        if (fcalls >= maxfun) throw MaxFuncCallError{};
        ++fcalls;
        return user_func(x);
    };
    auto sort_simplex = [&] {
        const auto ind = argsort(fsim);
        std::vector<Point> sorted_sim;
        std::vector<double> sorted_fsim;
        for (std::size_t i : ind) {
            sorted_sim.push_back(sim[i]);
            sorted_fsim.push_back(fsim[i]);
        }
        sim = std::move(sorted_sim);
        fsim = std::move(sorted_fsim);
    };

    try {
        for (std::size_t k = 0; k < N + 1; ++k) fsim[k] = func(sim[k]);
    } catch (const MaxFuncCallError&) {
    }
    sort_simplex();
    sort_simplex();

    int iterations = 1;
    while (fcalls < maxfun && iterations < maxiter) {
        try {
            std::vector<double> dx, df;
            for (std::size_t j = 1; j < N + 1; ++j) {
                for (std::size_t i = 0; i < N; ++i) dx.push_back(std::abs(sim[j][i] - sim[0][i]));
                df.push_back(std::abs(fsim[0] - fsim[j]));
            }
            if (nan_max(dx) <= options.xatol && nan_max(df) <= options.fatol) break;

            Point xbar = sim[0];
            for (std::size_t j = 1; j < N; ++j) {
                for (std::size_t i = 0; i < N; ++i) xbar[i] += sim[j][i];
            }
            for (std::size_t i = 0; i < N; ++i) xbar[i] /= static_cast<double>(N);

            const Point& worst = sim[N];
            Point xr(N);
            for (std::size_t i = 0; i < N; ++i) xr[i] = (1 + rho) * xbar[i] - rho * worst[i];
            if (has_bounds) xr = clip(xr, lower_bound, upper_bound);
            const double fxr = func(xr);
            bool doshrink = false;

            if (fxr < fsim[0]) {
                Point xe(N);
                for (std::size_t i = 0; i < N; ++i) xe[i] = (1 + rho * chi) * xbar[i] - rho * chi * worst[i];
                if (has_bounds) xe = clip(xe, lower_bound, upper_bound);
                const double fxe = func(xe);
                if (fxe < fxr) {
                    sim[N] = xe;
                    fsim[N] = fxe;
                } else {
                    sim[N] = xr;
                    fsim[N] = fxr;
                }
            } else {  // fsim[0] <= fxr
                if (fxr < fsim[N - 1]) {
                    sim[N] = xr;
                    fsim[N] = fxr;
                } else {  // fxr >= fsim[-2]
                    // Perform contraction
                    if (fxr < fsim[N]) {
                        Point xc(N);
                        for (std::size_t i = 0; i < N; ++i) xc[i] = (1 + psi * rho) * xbar[i] - psi * rho * worst[i];
                        if (has_bounds) xc = clip(xc, lower_bound, upper_bound);
                        const double fxc = func(xc);
                        if (fxc <= fxr) {
                            sim[N] = xc;
                            fsim[N] = fxc;
                        } else {
                            doshrink = true;
                        }
                    } else {
                        // Perform an inside contraction
                        Point xcc(N);
                        for (std::size_t i = 0; i < N; ++i) xcc[i] = (1 - psi) * xbar[i] + psi * worst[i];
                        if (has_bounds) xcc = clip(xcc, lower_bound, upper_bound);
                        const double fxcc = func(xcc);
                        if (fxcc < fsim[N]) {
                            sim[N] = xcc;
                            fsim[N] = fxcc;
                        } else {
                            doshrink = true;
                        }
                    }
                    if (doshrink) {
                        for (std::size_t j = 1; j < N + 1; ++j) {
                            for (std::size_t i = 0; i < N; ++i) sim[j][i] = sim[0][i] + sigma * (sim[j][i] - sim[0][i]);
                            if (has_bounds) sim[j] = clip(sim[j], lower_bound, upper_bound);
                            fsim[j] = func(sim[j]);
                        }
                    }
                }
            }
            iterations += 1;
        } catch (const MaxFuncCallError&) {
        }
        sort_simplex();
    }

    NelderMeadResult result;
    result.x = sim[0];
    result.fun = -nan_max([&] {  // np.min, which propagates NaN
        std::vector<double> negated;
        for (double f : fsim) negated.push_back(-f);
        return negated;
    }());
    result.nit = iterations;
    result.nfev = fcalls;
    if (fcalls >= maxfun) {
        result.status = 1;
    } else if (iterations >= maxiter) {
        result.status = 2;
    } else {
        result.status = 0;
    }
    result.success = result.status == 0;
    return result;
}

namespace {
double np_sign(double v) { return v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0); }
}  // namespace

ScalarResult minimize_scalar_bounded(const std::function<double(double)>& func, double x1, double x2, double xatol,
                                     int maxiter) {
    const int maxfun = maxiter;
    if (!std::isfinite(x1) || !std::isfinite(x2)) {
        throw std::invalid_argument("Optimization bounds must be finite scalars.");
    }
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
    double fu = std::numeric_limits<double>::infinity();

    double ffulc = fx, fnfc = fx;
    double xm = 0.5 * (a + b);
    double tol1 = sqrt_eps * std::abs(xf) + xatol / 3.0;
    double tol2 = 2.0 * tol1;

    while (std::abs(xf - xm) > (tol2 - 0.5 * (b - a))) {
        bool golden = true;
        // Check for parabolic fit
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
            // Check for acceptability of parabola
            if ((std::abs(p) < std::abs(0.5 * q * r)) && (p > q * (a - xf)) && (p < q * (b - xf))) {
                rat = (p + 0.0) / q;
                x = xf + rat;
                if (((x - a) < tol2) || ((b - x) < tol2)) {
                    const double si = np_sign(xm - xf) + ((xm - xf) == 0 ? 1.0 : 0.0);
                    rat = tol1 * si;
                }
            } else {  // do a golden-section step
                golden = true;
            }
        }
        if (golden) {  // do a golden-section step
            if (xf >= xm) {
                e = a - xf;
            } else {
                e = b - xf;
            }
            rat = golden_mean * e;
        }

        const double si = np_sign(rat) + (rat == 0 ? 1.0 : 0.0);
        x = xf + si * std::max(std::abs(rat), tol1);
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
            if ((fu <= fnfc) || (nfc == xf)) {
                fulc = nfc;
                ffulc = fnfc;
                nfc = x;
                fnfc = fu;
            } else if ((fu <= ffulc) || (fulc == xf) || (fulc == nfc)) {
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

    ScalarResult result;
    result.x = xf;
    result.fun = fx;
    result.nfev = num;
    result.status = flag;
    return result;
}

}  // namespace build123d::detail
