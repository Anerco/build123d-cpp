// Python-compatible number formatting, so that written files match Python's byte for byte.
#pragma once

#include <string>

namespace build123d::pyfmt {

/// repr()/str() of a Python float: the shortest string that round-trips ("0.1", "1e-05",
/// "1e+16", "3.0", "-0.0", "inf", "nan").
std::string repr(double x);

/// Python's round(x, ndigits) for floats: correctly rounded, ties to even on the exact
/// binary value, keeping the sign of zero.
double round(double x, int ndigits);

/// format(x, ".{precision}g") (identical to printf's %g for finite values).
std::string format_g(double x, int precision);

/// format(x, ".{precision}f").
std::string format_f(double x, int precision);

/// format(x, ".{precision}e").
std::string format_e(double x, int precision);

}  // namespace build123d::pyfmt
