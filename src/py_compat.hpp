// Python semantics the ports need: float %, repr(float) and int(str).
#pragma once

#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace build123d::detail {

/// Python's float % (the result has the sign of the divisor).
inline double py_mod(double a, double b) {
    double mod = std::fmod(a, b);
    if (mod != 0.0) {
        if ((b < 0) != (mod < 0)) mod += b;
    } else {
        mod = std::copysign(0.0, b);
    }
    return mod;
}

/// Python's repr(float) / str(float): the shortest digits that round-trip, in fixed notation
/// unless the decimal exponent is below -4 or above 16, with ".0" added to whole numbers.
inline std::string py_float_repr(double value) {
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value > 0 ? "inf" : "-inf";
    if (value == 0.0) return std::signbit(value) ? "-0.0" : "0.0";
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::scientific);
    std::string sci(buf, res.ptr);
    std::string sign;
    if (sci[0] == '-') {
        sign = "-";
        sci = sci.substr(1);
    }
    const auto e = sci.find('e');
    std::string mantissa = sci.substr(0, e);
    const int exponent = std::stoi(sci.substr(e + 1));
    std::string digits;
    for (char c : mantissa) {
        if (c != '.') digits += c;
    }
    const int ndigits = static_cast<int>(digits.size());
    const int decpt = exponent + 1;
    std::string out;
    if (decpt <= -4 || decpt > 16) {
        out = digits.substr(0, 1);
        if (ndigits > 1) out += "." + digits.substr(1);
        const int exp = decpt - 1;
        char ebuf[16];
        std::snprintf(ebuf, sizeof(ebuf), "e%c%02d", exp < 0 ? '-' : '+', std::abs(exp));
        out += ebuf;
    } else if (decpt <= 0) {
        out = "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
    } else if (decpt >= ndigits) {
        out = digits + std::string(static_cast<std::size_t>(decpt - ndigits), '0') + ".0";
    } else {
        out = digits.substr(0, static_cast<std::size_t>(decpt)) + "." + digits.substr(static_cast<std::size_t>(decpt));
    }
    return sign + out;
}

/// Python's int(str) for decimal integers (surrounding whitespace, a sign and single
/// underscores between digits are accepted); ValueError otherwise.
inline long long py_int(const std::string& text) {
    std::size_t b = 0, e = text.size();
    while (b < e && std::isspace(static_cast<unsigned char>(text[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(text[e - 1]))) --e;
    std::string s = text.substr(b, e - b);
    bool negative = false;
    if (!s.empty() && (s[0] == '+' || s[0] == '-')) {
        negative = s[0] == '-';
        s = s.substr(1);
    }
    std::string digits;
    bool ok = !s.empty() && std::isdigit(static_cast<unsigned char>(s.front())) &&
              std::isdigit(static_cast<unsigned char>(s.back()));
    for (std::size_t i = 0; ok && i < s.size(); ++i) {
        if (std::isdigit(static_cast<unsigned char>(s[i]))) {
            digits += s[i];
        } else if (s[i] == '_' && i + 1 < s.size() && std::isdigit(static_cast<unsigned char>(s[i + 1]))) {
            continue;
        } else {
            ok = false;
        }
    }
    if (!ok) throw std::invalid_argument("invalid literal for int() with base 10: '" + text + "'");
    const long long value = std::stoll(digits);
    return negative ? -value : value;
}

}  // namespace build123d::detail
