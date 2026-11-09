#include "pyformat.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace build123d::pyfmt {

std::string repr(double x) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x > 0 ? "inf" : "-inf";
    if (x == 0.0) return std::signbit(x) ? "-0.0" : "0.0";
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::scientific);
    std::string s(buf, r.ptr);
    std::string sign;
    if (s[0] == '-') {
        sign = "-";
        s.erase(0, 1);
    }
    const auto epos = s.find('e');
    const std::string mantissa = s.substr(0, epos);
    const int exp10 = std::stoi(s.substr(epos + 1));
    std::string digits;
    for (char c : mantissa) {
        if (c != '.') digits += c;
    }
    while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
    // value = 0.<digits> * 10^decpt
    const int decpt = exp10 + 1;
    const int n = static_cast<int>(digits.size());
    std::string out;
    if (decpt <= -4 || decpt > 16) {
        out = digits.substr(0, 1);
        if (n > 1) out += "." + digits.substr(1);
        const int e = decpt - 1;
        char ebuf[16];
        std::snprintf(ebuf, sizeof(ebuf), "e%c%02d", e < 0 ? '-' : '+', std::abs(e));
        out += ebuf;
    } else if (decpt <= 0) {
        out = "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
    } else if (decpt >= n) {
        out = digits + std::string(static_cast<std::size_t>(decpt - n), '0') + ".0";
    } else {
        out = digits.substr(0, static_cast<std::size_t>(decpt)) + "." + digits.substr(static_cast<std::size_t>(decpt));
    }
    return sign + out;
}

double round(double x, int ndigits) {
    if (!std::isfinite(x) || x == 0.0) return x;
    // Python (float_round_impl): values whose rounding can't change are returned as is.
    // 308 + 15 digits cover every double; past them nothing changes.
    if (ndigits > 323) return x;
    if (ndigits < -308) return std::copysign(0.0, x);
    char buf[512];
    if (ndigits >= 0) {
        const auto r = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::fixed, ndigits);
        if (r.ec != std::errc()) throw std::runtime_error("pyfmt::round: formatting failed");
        double y = 0.0;
        std::from_chars(buf, r.ptr, y);
        return y == 0.0 ? std::copysign(0.0, x) : y;
    }
    // Negative ndigits: round to tens, hundreds, ... via the exact decimal digits.
    const double scale = std::pow(10.0, -ndigits);
    const auto r = std::to_chars(buf, buf + sizeof(buf), x / scale, std::chars_format::fixed, 0);
    double y = 0.0;
    std::from_chars(buf, r.ptr, y);
    y *= scale;
    return y == 0.0 ? std::copysign(0.0, x) : y;
}

std::string format_g(double x, int precision) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x > 0 ? "inf" : "-inf";
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%.*g", precision, x);
    return buf;
}

std::string format_f(double x, int precision) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x > 0 ? "inf" : "-inf";
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%.*f", precision, x);
    return buf;
}

std::string format_e(double x, int precision) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x > 0 ? "inf" : "-inf";
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%.*e", precision, x);
    return buf;
}

}  // namespace build123d::pyfmt
