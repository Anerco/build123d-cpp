// Port of the parts of svgelements 1.9.6 that ocpsvg uses (see svgdoc.hpp). The structure
// follows svgelements: lengths with units, colors, Matrix and its transform parser, path
// segments and arcs (center parameterization), the path-data lexer, the basic shapes with
// render()/reify()/segments(), and SVG.parse with its value cascade, CSS and <use> inlining.
// Lengths with units inside transforms are resolved when the transform is parsed (svgelements
// keeps them symbolic until render()), which only differs in rounding.
#include "svgdoc.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <regex>
#include <stdexcept>

#include "pyformat.hpp"
#include "xml_reader.hpp"

namespace build123d::svgdoc {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kTau = 6.283185307179586;
constexpr double kError = 1e-12;

/// Python's ValueError raised inside svgelements: SVG.parse catches it around shapes.
struct ValueError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

[[noreturn]] void fail(const std::string& what) { throw std::invalid_argument("SVG import: " + what); }

/// math.radians / math.degrees
double radians(double x) { return x * (kPi / 180.0); }
double degrees(double x) { return x * (180.0 / kPi); }

/// Python's float % float.
double py_mod(double x, double y) {
    double mod = std::fmod(x, y);
    if (mod != 0.0) {
        if ((y < 0) != (mod < 0)) mod += y;
    } else {
        mod = std::copysign(0.0, y);
    }
    return mod;
}

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }

std::string strip(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && is_space(s[b])) ++b;
    while (e > b && is_space(s[e - 1])) --e;
    return s.substr(b, e - b);
}

std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
        const auto p = s.find(sep, start);
        if (p == std::string::npos) {
            out.push_back(s.substr(start));
            return out;
        }
        out.push_back(s.substr(start, p - start));
        start = p + 1;
    }
}

bool ends_with(const std::string& s, const char* suffix) {
    const std::size_t n = std::strlen(suffix);
    return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

/// PATTERN_FLOAT = [-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)? matched at `pos`: the length of
/// the match, 0 if there is none.
std::size_t match_float(const std::string& s, std::size_t pos) {
    std::size_t i = pos;
    auto digit = [&s](std::size_t k) { return k < s.size() && std::isdigit(static_cast<unsigned char>(s[k])); };
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    const std::size_t ds = i;
    while (digit(i)) ++i;
    const std::size_t d1 = i - ds;
    if (i < s.size() && s[i] == '.' && digit(i + 1)) {
        ++i;
        while (digit(i)) ++i;
    } else if (d1 == 0) {
        return 0;
    }
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        std::size_t j = i + 1;
        if (j < s.size() && (s[j] == '+' || s[j] == '-')) ++j;
        if (digit(j)) {
            while (digit(j)) ++j;
            i = j;
        }
    }
    return i - pos;
}

double to_float(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

/// REGEX_FLOAT.findall
std::vector<double> find_floats(const std::string& s) {
    std::vector<double> out;
    std::size_t pos = 0;
    while (pos < s.size()) {
        const std::size_t n = match_float(s, pos);
        if (n) {
            out.push_back(to_float(s.substr(pos, n)));
            pos += n;
        } else {
            ++pos;
        }
    }
    return out;
}

/// Python's float(text), or nothing where it raises ValueError.
std::optional<double> py_float(const std::string& text) {
    std::string s = strip(text);
    if (s.empty()) return std::nullopt;
    std::string clean;
    for (std::size_t i = 0; i < s.size(); ++i) {
        // underscores are allowed between digits
        if (s[i] == '_' && i > 0 && i + 1 < s.size() && std::isdigit(static_cast<unsigned char>(s[i - 1])) &&
            std::isdigit(static_cast<unsigned char>(s[i + 1]))) {
            continue;
        }
        clean += s[i];
    }
    const std::string l = lower(clean);
    if (l.find("0x") != std::string::npos || l.find('_') != std::string::npos) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(clean.c_str(), &end);
    if (end != clean.c_str() + clean.size()) return std::nullopt;
    if (std::isnan(v) && l.find("nan(") != std::string::npos) return std::nullopt;
    return v;
}

// ---------------------------------------------------------------------------
// Length
// ---------------------------------------------------------------------------

struct Length {
    bool none = false;  // amount is None
    double amount = 0.0;
    std::string units;
};

/// Length(text): the first REGEX_LENGTH match (amount 0 without units if there is none).
Length parse_length(const std::string& s) {
    std::size_t pos = 0;
    while (pos < s.size()) {
        const std::size_t n = match_float(s, pos);
        if (n) {
            std::size_t e = pos + n;
            while (e < s.size() && (std::isalpha(static_cast<unsigned char>(s[e])) || s[e] == '%')) ++e;
            return Length{false, to_float(s.substr(pos, n)), s.substr(pos + n, e - pos - n)};
        }
        ++pos;
    }
    return Length{false, 0.0, ""};
}

/// Length(values.get(key[, default])).
Length length_attr(const std::map<std::string, std::string>& values, const std::string& key,
                   const char* default_text) {
    const auto it = values.find(key);
    if (it != values.end()) return parse_length(it->second);
    if (default_text == nullptr) return Length{true, 0.0, ""};
    return parse_length(default_text);
}

/// A value as svgelements keeps it: None, a float, or a Length it couldn't resolve yet.
struct LV {
    bool none = false;
    bool resolved = true;
    double v = 0.0;
    Length len;
};

LV lv_float(double v) {
    LV r;
    r.v = v;
    return r;
}
LV lv_length(const Length& l) {
    LV r;
    r.resolved = false;
    r.len = l;
    return r;
}
LV lv_none() {
    LV r;
    r.none = true;
    return r;
}

struct Viewbox {
    std::optional<double> x, y, width, height;
};

/// Length.value(ppi, relative_length, viewbox).
LV length_value(const Length& l, std::optional<double> ppi = std::nullopt,
                std::optional<double> relative_length = std::nullopt, const Viewbox* viewbox = nullptr) {
    if (l.none) return lv_none();
    const std::string& u = l.units;
    if (u == "%") {
        if (!relative_length) return lv_length(l);
        return lv_float(l.amount / 100.0 * *relative_length);
    }
    if (u == "mm") return ppi ? lv_float(l.amount * *ppi * 0.0393701) : lv_length(l);
    if (u == "cm") return ppi ? lv_float(l.amount * *ppi * 0.393701) : lv_length(l);
    if (u == "in") return ppi ? lv_float(l.amount * *ppi) : lv_length(l);
    if (u == "px" || u.empty()) return lv_float(l.amount);
    if (u == "pt") return lv_float(l.amount * 4.0 / 3.0);
    if (u == "pc") return lv_float(l.amount * 16.0);
    if (u == "em" || u == "ex") return lv_length(l);
    if (u == "vw" || u == "vh" || u == "vmin" || u == "vmax") {
        if (viewbox == nullptr) return lv_length(l);
        const auto& dim = u == "vw" ? viewbox->width : viewbox->height;  // vmin/vmax use the height
        if (!dim) fail("can't resolve '" + u + "' without a complete viewBox");
        return lv_float(l.amount * *dim / 100.0);
    }
    return lv_float(l.amount);  // float(self)
}

/// float(value): a Length gives its amount (pt and pc converted).
double float_of(const LV& x) {
    if (x.none) fail("missing length");
    if (x.resolved) return x.v;
    if (x.len.units == "pt") return x.len.amount * 4.0 / 3.0;
    if (x.len.units == "pc") return x.len.amount * 16.0;
    return x.len.amount;
}

/// Length.str: "%.12f" without trailing zeros.
std::string length_str(double s) {
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%.12f", s);
    std::string out = buf;
    if (out.find('.') != std::string::npos) {
        while (!out.empty() && out.back() == '0') out.pop_back();
        if (!out.empty() && out.back() == '.') out.pop_back();
    }
    return out;
}

/// Length(value) of a float or Length (through their str()).
Length rewrap(const LV& x) {
    if (x.none) return Length{true, 0.0, ""};
    if (x.resolved) return parse_length(pyfmt::repr(x.v));
    return parse_length(length_str(x.len.amount) + x.len.units);
}

/// The relative lengths and resolution a shape is rendered with.
struct RenderInfo {
    double ppi = 96.0;
    LV width = lv_none();
    LV height = lv_none();
    /// Only check the transform for the errors svgelements raises while parsing it (lengths
    /// it can't resolve stay symbolic there).
    bool lenient = false;
};

/// shape.render(ppi, width, height) of one of its lengths: a float.
double render_length(const LV& x, const RenderInfo& info, bool vertical, const char* what) {
    if (x.none) fail(std::string("missing ") + what);
    if (x.resolved) return x.v;
    const LV& rel = vertical ? info.height : info.width;
    std::optional<double> relative;
    if (rel.resolved && !rel.none) relative = rel.v;
    const LV r = length_value(x.len, info.ppi, relative);
    if (!r.resolved) {
        if (info.lenient) return 0.0;
        fail(std::string("can't resolve the length of ") + what + " ('" + x.len.units + "' units)");
    }
    return r.v;
}

// ---------------------------------------------------------------------------
// Color
// ---------------------------------------------------------------------------

const std::pair<const char*, unsigned long long> kColorNames[] = {
#include "svg_colors.inc"
};

std::int64_t crimp(double v) {
    if (v > 255) return 255;
    if (v < 0) return 0;
    return static_cast<std::int64_t>(v);
}

std::int64_t rgb_to_int(double r, double g, double b, double opacity = 1.0) {
    if (opacity > 1) opacity = 1.0;
    if (opacity < 0) opacity = 0;
    const double rounded = std::nearbyint(opacity * 255.0);
    if (std::isnan(rounded)) throw ValueError("cannot convert float NaN to integer");
    const std::int64_t a = crimp(rounded);
    return (crimp(r) << 24) | (crimp(g) << 16) | (crimp(b) << 8) | a;
}

std::int64_t hsl_to_int(double h, double s, double l, double opacity) {
    auto hue_2_rgb = [](double v1, double v2, double vh) {
        if (vh < 0) vh += 1;
        if (vh > 1) vh -= 1;
        if (6.0 * vh < 1.0) return v1 + (v2 - v1) * 6.0 * vh;
        if (2.0 * vh < 1) return v2;
        if (3 * vh < 2.0) return v1 + (v2 - v1) * ((2.0 / 3.0) - vh) * 6.0;
        return v1;
    };
    double r, g, b;
    if (s == 0.0) {
        r = 255.0 * l;
        g = 255.0 * l;
        b = 255.0 * l;
    } else {
        const double v2 = l < 0.5 ? l * (1.0 + s) : (l + s) - (s * l);
        const double v1 = 2 * l - v2;
        r = 255.0 * hue_2_rgb(v1, v2, h + (1.0 / 3.0));
        g = 255.0 * hue_2_rgb(v1, v2, h);
        b = 255.0 * hue_2_rgb(v1, v2, h - (1.0 / 3.0));
    }
    return rgb_to_int(r, g, b, opacity);
}

/// Python's int(text) for a matched number: ValueError unless it is an integer literal.
double py_int(const std::string& text) {
    std::size_t i = 0;
    if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
    if (i == text.size()) throw ValueError("invalid literal for int() with base 10: '" + text + "'");
    for (std::size_t k = i; k < text.size(); ++k) {
        if (!std::isdigit(static_cast<unsigned char>(text[k]))) {
            throw ValueError("invalid literal for int() with base 10: '" + text + "'");
        }
    }
    return std::strtod(text.c_str(), nullptr);
}

/// Angle.parse: radians.
double angle_parse(const std::string& text) {
    const std::string s = lower(text);
    auto number = [](const std::string& t) {
        const auto v = py_float(t);
        if (!v) throw ValueError("could not convert string to float: '" + t + "'");
        return *v;
    };
    if (ends_with(s, "deg")) return kTau * number(s.substr(0, s.size() - 3)) / 360.0;
    if (ends_with(s, "grad")) return kTau * number(s.substr(0, s.size() - 4)) / 400.0;
    if (ends_with(s, "rad")) return number(s.substr(0, s.size() - 3));
    if (ends_with(s, "turn")) return kTau * number(s.substr(0, s.size() - 4));
    if (ends_with(s, "%")) return kTau * (number(s.substr(0, s.size() - 1)) / 100.0);
    return kTau * number(s) / 360.0;
}

#define B3D_FLOAT "[-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?"

const std::regex& re_rgb() {
    static const std::regex r("rgba?\\(\\s*(" B3D_FLOAT ")\\s*,\\s*(" B3D_FLOAT ")\\s*,\\s*(" B3D_FLOAT
                              ")\\s*(?:,\\s*(" B3D_FLOAT ")\\s*)?\\)");
    return r;
}
const std::regex& re_rgb_percent() {
    static const std::regex r("rgba?\\(\\s*(" B3D_FLOAT ")%\\s*,\\s*(" B3D_FLOAT ")%\\s*,\\s*(" B3D_FLOAT
                              ")%\\s*(?:,\\s*(" B3D_FLOAT ")\\s*)?\\)");
    return r;
}
const std::regex& re_hsl() {
    static const std::regex r("hsla?\\(\\s*(" B3D_FLOAT ")\\s*,\\s*(" B3D_FLOAT ")%\\s*,\\s*(" B3D_FLOAT
                              ")%\\s*(?:,\\s*(" B3D_FLOAT ")\\s*)?\\)");
    return r;
}
const std::regex& re_coord_pair() {
    static const std::regex r("(" B3D_FLOAT ")(?:\\s*,\\s*|\\s+|(?=-))(" B3D_FLOAT ")");
    return r;
}
const std::regex& re_transform_template() {
    static const std::regex r(
        "(matrix|translate|translatex|translatey|scale|scalex|scaley|rotate|skew|skewx|skewy)[\\s\\t\\n]*\\(([^)]+)\\)");
    return r;
}
const std::regex& re_transform_parameter() {
    static const std::regex r("(" B3D_FLOAT
                              ")[\\s\\t\\n]*(cm|mm|Q|in|pt|pc|px|em|cx|ch|rem|vw|vh|vmin|vmax|deg|grad|rad|turn|%)?");
    return r;
}

#undef B3D_FLOAT

/// re.match: a match anchored at the start of the text.
bool re_match(const std::string& text, const std::regex& re, std::smatch& m) {
    return std::regex_search(text, m, re, std::regex_constants::match_continuous);
}

/// Color.parse
std::optional<std::int64_t> color_parse(const std::string& cs) {
    if (cs == "none") return std::nullopt;
    {
        // REGEX_COLOR_HEX: ^#?([0-9A-Fa-f]{3,8})$
        const std::size_t first = (!cs.empty() && cs[0] == '#') ? 1 : 0;
        const std::size_t n = cs.size() - first;
        bool hex = n >= 3 && n <= 8;
        for (std::size_t i = first; hex && i < cs.size(); ++i) hex = std::isxdigit(static_cast<unsigned char>(cs[i])) != 0;
        if (hex) {
            const std::string h = cs.substr(first);
            if (h.size() == 8) return static_cast<std::int64_t>(std::stoull(h, nullptr, 16));
            if (h.size() == 6) return static_cast<std::int64_t>(std::stoull(h + "FF", nullptr, 16));
            if (h.size() == 4) {
                const std::string e{h[0], h[0], h[1], h[1], h[2], h[2], h[3], h[3]};
                return static_cast<std::int64_t>(std::stoull(e, nullptr, 16));
            }
            if (h.size() == 3) {
                const std::string e{h[0], h[0], h[1], h[1], h[2], h[2], 'F', 'F'};
                return static_cast<std::int64_t>(std::stoull(e, nullptr, 16));
            }
            return rgb_to_int(0, 0, 0);
        }
    }
    std::smatch m;
    if (re_match(cs, re_rgb(), m)) {
        const double opacity = m[4].matched ? to_float(m[4].str()) : 1.0;
        return rgb_to_int(py_int(m[1].str()), py_int(m[2].str()), py_int(m[3].str()), opacity);
    }
    if (re_match(cs, re_rgb_percent(), m)) {
        const double ratio = 255.0 / 100.0;
        const double r = std::nearbyint(to_float(m[1].str()) * ratio);
        const double g = std::nearbyint(to_float(m[2].str()) * ratio);
        const double b = std::nearbyint(to_float(m[3].str()) * ratio);
        const double opacity = m[4].matched ? to_float(m[4].str()) : 1.0;
        return rgb_to_int(r, g, b, opacity);
    }
    if (re_match(cs, re_hsl(), m)) {
        const double h = angle_parse(m[1].str()) / kTau;
        double s = to_float(m[2].str()) / 100.0;
        if (s > 1) s = 1.0;
        if (s < 0) s = 0.0;
        double l = to_float(m[3].str()) / 100.0;
        if (l > 1) l = 1.0;
        if (l < 0) l = 0.0;
        const double opacity = m[4].matched ? to_float(m[4].str()) : 1.0;
        return hsl_to_int(h, s, l, opacity);
    }
    // parse_color_lookup
    std::string v;
    for (char c : cs) {
        if (c != ' ') v += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    for (const auto& [name, value] : kColorNames) {
        if (v == name) return static_cast<std::int64_t>(value);
    }
    // int(v)
    const std::string t = strip(v);
    std::size_t i = (!t.empty() && (t[0] == '+' || t[0] == '-')) ? 1 : 0;
    bool integer = i < t.size();
    std::string digits;
    for (; integer && i < t.size(); ++i) {
        if (std::isdigit(static_cast<unsigned char>(t[i]))) {
            digits += t[i];
        } else if (t[i] == '_' && i > 0 && std::isdigit(static_cast<unsigned char>(t[i - 1])) && i + 1 < t.size() &&
                   std::isdigit(static_cast<unsigned char>(t[i + 1]))) {
            continue;
        } else {
            integer = false;
        }
    }
    if (integer && !digits.empty()) {
        const std::int64_t mag = static_cast<std::int64_t>(std::stoull(digits.substr(0, 18)));
        return t[0] == '-' ? -mag : mag;
    }
    return rgb_to_int(0, 0, 0);
}

/// Color.opacity = opacity (on a color that has a value)
void set_opacity(Color& c, double opacity) {
    const double rounded = std::nearbyint(opacity * 255.0);
    if (std::isnan(rounded)) return;  // round(nan): ValueError, which svgelements ignores here
    if (std::isinf(rounded)) fail("cannot convert float infinity to integer");
    const std::int64_t a = crimp(crimp(rounded));
    *c.value &= ~static_cast<std::int64_t>(0xFF);
    *c.value |= a;
}

// ---------------------------------------------------------------------------
// Points and matrices
// ---------------------------------------------------------------------------

/// Point.distance(p1, p2)
double point_distance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    dx *= dx;
    dy *= dy;
    return std::sqrt(dx + dy);
}

/// Point.angle(p1, p2), p1.angle_to(p2)
double point_angle(const Point& p1, const Point& p2) { return std::atan2(p2.y - p1.y, p2.x - p1.x); }

/// p1.distance_to(p2) = abs(p1 - p2)
double distance_to(const Point& p1, const Point& p2) { return std::hypot(p1.x - p2.x, p1.y - p2.y); }

/// Point.orientation(p, q, r)
int orientation(const Point& p, const Point& q, const Point& r) {
    const double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;
    return val > 0 ? 1 : 2;
}

Matrix m_translate(double tx, double ty) { return Matrix{1.0, 0.0, 0.0, 1.0, tx, ty}; }
Matrix m_scale(double sx, double sy) { return Matrix{sx, 0, 0, sy, 0, 0}; }
Matrix m_rotate(double angle) {
    const double ct = std::cos(angle);
    const double st = std::sin(angle);
    return Matrix{ct, st, -st, ct, 0.0, 0.0};
}
Matrix m_skew(double angle_a, double angle_b) {
    const double aa = std::tan(angle_a);
    const double bb = std::tan(angle_b);
    return Matrix{1.0, bb, aa, 1.0, 0.0, 0.0};
}

void pre_cat(Matrix& self, const Matrix& mx) { self = Matrix::multiply(mx, self); }
void post_cat(Matrix& self, const Matrix& mx) { self = Matrix::multiply(self, mx); }
void pre_translate(Matrix& self, double tx, double ty = 0.0) { pre_cat(self, m_translate(tx, ty)); }
void post_translate(Matrix& self, double tx, double ty = 0.0) { post_cat(self, m_translate(tx, ty)); }

void pre_scale(Matrix& self, double sx, double sy, double x, double y) {
    if (x == 0 && y == 0) {
        pre_cat(self, m_scale(sx, sy));
    } else {
        pre_translate(self, x, y);
        pre_scale(self, sx, sy, 0.0, 0.0);
        pre_translate(self, -x, -y);
    }
}

void pre_rotate(Matrix& self, double angle, double x, double y) {
    if (x == 0 && y == 0) {
        pre_cat(self, m_rotate(angle));
    } else {
        pre_translate(self, x, y);
        pre_rotate(self, angle, 0.0, 0.0);
        pre_translate(self, -x, -y);
    }
}

void pre_skew(Matrix& self, double angle_a, double angle_b, double x, double y) {
    if (x == 0 && y == 0) {
        pre_cat(self, m_skew(angle_a, angle_b));
    } else {
        pre_translate(self, x, y);
        pre_skew(self, angle_a, angle_b, 0.0, 0.0);
        pre_translate(self, -x, -y);
    }
}

void post_rotate(Matrix& self, double angle, double x, double y) {
    if (x == 0 && y == 0) {
        post_cat(self, m_rotate(angle));
    } else {
        Matrix matrix;
        post_translate(matrix, -x, -y);
        post_cat(matrix, m_rotate(angle));
        post_translate(matrix, x, y);
        post_cat(self, matrix);
    }
}

/// Matrix(transform_str) with its lengths rendered (ppi, width, height).
Matrix parse_transform(const std::string& transform_str, const RenderInfo& info) {
    Matrix self;
    if (transform_str.empty()) return self;
    const std::string text = lower(transform_str);
    auto length = [&info](const std::string& param, bool vertical) {
        const LV v = length_value(parse_length(param));
        return render_length(v, info, vertical, "a transform");
    };
    auto number = [](const std::string& param) {
        const auto v = py_float(param);
        if (!v) throw ValueError("could not convert string to float: '" + param + "'");
        return *v;
    };
    for (auto it = std::sregex_iterator(text.begin(), text.end(), re_transform_template());
         it != std::sregex_iterator(); ++it) {
        const std::string name = (*it)[1].str();
        const std::string inner = (*it)[2].str();
        std::vector<std::string> params;
        for (auto p = std::sregex_iterator(inner.begin(), inner.end(), re_transform_parameter());
             p != std::sregex_iterator(); ++p) {
            params.push_back((*p)[1].str() + ((*p)[2].matched ? (*p)[2].str() : std::string()));
        }
        const std::size_t n = params.size();
        if (name == "matrix") {
            std::vector<double> v;
            for (const auto& p : params) v.push_back(number(p));
            if (v.empty()) {
                pre_cat(self, Matrix());
            } else if (v.size() < 6) {
                fail("matrix() needs 6 values");  // TypeError/IndexError in svgelements
            } else {
                pre_cat(self, Matrix{v[0], v[1], v[2], v[3], v[4], v[5]});
            }
        } else if (name == "translate") {
            if (n == 0) continue;
            const double x = length(params[0], false);
            if (n >= 2) {
                pre_translate(self, x, length(params[1], true));
            } else {
                pre_translate(self, x);
            }
        } else if (name == "translatex") {
            if (n == 0) fail("translatex() needs a value");
            pre_translate(self, length(params[0], false), 0);
        } else if (name == "translatey") {
            if (n == 0) fail("translatey() needs a value");
            pre_translate(self, 0, length(params[0], true));
        } else if (name == "scale") {
            std::vector<double> v;
            for (const auto& p : params) v.push_back(number(p));
            if (v.size() > 4) fail("scale() takes at most 4 values");
            const double sx = v.size() > 0 ? v[0] : 1.0;
            const double sy = v.size() > 1 ? v[1] : sx;
            const double x = v.size() > 2 ? v[2] : 0.0;
            const double y = v.size() > 3 ? v[3] : 0.0;
            pre_scale(self, sx, sy, x, y);
        } else if (name == "scalex") {
            if (n == 0) fail("scalex() needs a value");
            pre_scale(self, number(params[0]), 1, 0.0, 0.0);
        } else if (name == "scaley") {
            if (n == 0) fail("scaley() needs a value");
            pre_scale(self, 1, number(params[0]), 0.0, 0.0);
        } else if (name == "rotate") {
            if (n == 0) fail("rotate() needs an angle");
            const double angle = angle_parse(params[0]);
            if (n < 2) {
                pre_rotate(self, angle, 0.0, 0.0);
                continue;
            }
            const double x = length(params[1], false);
            const double y = n >= 3 ? length(params[2], true) : 0.0;
            pre_rotate(self, angle, x, y);
        } else if (name == "skew") {
            if (n == 0) fail("skew() needs an angle");
            const double angle_a = angle_parse(params[0]);
            if (n < 2) continue;
            const double angle_b = angle_parse(params[1]);
            if (n < 3) {
                pre_skew(self, angle_a, angle_b, 0.0, 0.0);
                continue;
            }
            const double x = length(params[2], false);
            const double y = n >= 4 ? length(params[3], true) : 0.0;
            pre_skew(self, angle_a, angle_b, x, y);
        } else if (name == "skewx" || name == "skewy") {
            if (n == 0) fail(name + "() needs an angle");
            const double angle = angle_parse(params[0]);
            double x = 0.0, y = 0.0;
            if (n >= 2) x = length(params[1], false);
            if (n >= 3) y = length(params[2], true);
            if (name == "skewx") {
                pre_skew(self, angle, 0, x, y);
            } else {
                pre_skew(self, 0.0, angle, x, y);
            }
        }
    }
    return self;
}

/// Viewbox.viewbox_transform: the transform string of an <svg> viewport.
std::string viewbox_transform(double e_x, double e_y, double e_width, double e_height, const Viewbox& vb,
                              const std::optional<std::string>& aspect) {
    if (!vb.x || !vb.y || !vb.width || !vb.height) return "";
    std::string align = "xMidyMid";
    std::string meet_or_slice = "meet";
    if (aspect) {
        const auto parts = split(*aspect, ' ');
        align = parts[0];
        meet_or_slice = parts.size() > 1 ? parts[1] : "meet";
    }
    if (*vb.width == 0 || *vb.height == 0) throw std::domain_error("float division by zero");
    double scale_x = e_width / *vb.width;
    double scale_y = e_height / *vb.height;
    if (align != "none" && meet_or_slice == "meet") {
        scale_x = scale_y = std::min(scale_x, scale_y);
    } else if (align != "none" && meet_or_slice == "slice") {
        scale_x = scale_y = std::max(scale_x, scale_y);
    }
    double translate_x = e_x - (*vb.x * scale_x);
    double translate_y = e_y - (*vb.y * scale_y);
    align = lower(align);
    if (align.find("xmid") != std::string::npos) translate_x += (e_width - *vb.width * scale_x) / 2.0;
    if (align.find("xmax") != std::string::npos) translate_x += e_width - *vb.width * scale_x;
    if (align.find("ymid") != std::string::npos) translate_y += (e_height - *vb.height * scale_y) / 2.0;
    if (align.find("ymax") != std::string::npos) translate_y += e_height - *vb.height * scale_y;
    if (translate_x == 0 && translate_y == 0) {
        if (scale_x == 1 && scale_y == 1) return "";
        return "scale(" + length_str(scale_x) + ", " + length_str(scale_y) + ")";
    }
    if (scale_x == 1 && scale_y == 1) return "translate(" + length_str(translate_x) + ", " + length_str(translate_y) + ")";
    return "translate(" + length_str(translate_x) + ", " + length_str(translate_y) + ") scale(" + length_str(scale_x) +
           ", " + length_str(scale_y) + ")";
}

// ---------------------------------------------------------------------------
// Arcs
// ---------------------------------------------------------------------------

constexpr double kTau14 = kTau / 4.0;
constexpr double kTau34 = 3 * kTau14;

/// Arc.point_at_t
Point point_at_t(const Segment& arc, double t) {
    const double rotation = arc.rotation();
    const double a = arc.rx();
    const double b = arc.ry();
    const double cx = arc.center.x;
    const double cy = arc.center.y;
    const double cos_rot = std::cos(rotation);
    const double sin_rot = std::sin(rotation);
    const double cos_t = std::cos(t);
    const double sin_t = std::sin(t);
    const double px = cx + a * cos_t * cos_rot - b * sin_t * sin_rot;
    const double py = cy + a * cos_t * sin_rot + b * sin_t * cos_rot;
    return Point{px, py};
}

/// Arc.point_at_angle
Point point_at_angle(const Segment& arc, double angle) {
    angle -= arc.rotation();
    const double a = arc.rx();
    const double b = arc.ry();
    if (a == b) return point_at_t(arc, angle);
    double t = std::atan2(a * std::tan(angle), b);
    const double m = std::fmod(std::fabs(angle), kTau);
    if (kTau34 >= m && m > kTau14) t += kTau / 2.0;
    return point_at_t(arc, t);
}

/// Arc.t_at_point
double t_at_point(const Segment& arc, const Point& p) {
    double angle = point_angle(arc.center, p);
    angle -= arc.rotation();
    const double a = arc.rx();
    const double b = arc.ry();
    double t = std::atan2(a * std::tan(angle), b);
    const double m = std::fmod(std::fabs(angle), kTau);
    if (kTau34 >= m && m > kTau14) t += kTau / 2.0;
    return t;
}

/// Arc(start, rx, ry, rotation, large_arc, sweep, end): Arc._svg_parameterize.
Segment svg_arc(const std::optional<Point>& start_opt, double rx, double ry, std::optional<double> rotation,
                bool large_arc_flag, bool sweep_flag, const std::optional<Point>& end_opt) {
    if (!start_opt || !end_opt) fail("an arc needs a current point");
    const Point start = *start_opt;
    const Point end = *end_opt;
    Segment s;
    s.kind = SegKind::ARC;
    s.start = start;
    s.end = end;
    if (same_point(start, end) || rx == 0 || ry == 0) {
        s.sweep = 0;
        s.prx = start;
        s.pry = start;
        s.center = start;
        return s;
    }
    if (!rotation) fail("an arc needs a rotation");
    const double cosr = std::cos(radians(*rotation));
    const double sinr = std::sin(radians(*rotation));
    const double dx = (start.x - end.x) / 2;
    const double dy = (start.y - end.y) / 2;
    const double x1prim = cosr * dx + sinr * dy;
    const double x1prim_sq = x1prim * x1prim;
    const double y1prim = -sinr * dx + cosr * dy;
    const double y1prim_sq = y1prim * y1prim;
    double rx_sq = rx * rx;
    double ry_sq = ry * ry;
    const double radius_check = (x1prim_sq / rx_sq) + (y1prim_sq / ry_sq);
    if (radius_check > 1) {
        rx *= std::sqrt(radius_check);
        ry *= std::sqrt(radius_check);
        rx_sq = rx * rx;
        ry_sq = ry * ry;
    }
    const double t1 = rx_sq * y1prim_sq;
    const double t2 = ry_sq * x1prim_sq;
    double c = std::sqrt(std::fabs((rx_sq * ry_sq - t1 - t2) / (t1 + t2)));
    if (large_arc_flag == sweep_flag) c = -c;
    const double cxprim = c * rx * y1prim / ry;
    const double cyprim = -c * ry * x1prim / rx;
    const Point center{(cosr * cxprim - sinr * cyprim) + ((start.x + end.x) / 2),
                       (sinr * cxprim + cosr * cyprim) + ((start.y + end.y) / 2)};
    const double ux = (x1prim - cxprim) / rx;
    const double uy = (y1prim - cyprim) / ry;
    const double vx = (-x1prim - cxprim) / rx;
    const double vy = (-y1prim - cyprim) / ry;
    const double n = std::sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));
    const double p = ux * vx + uy * vy;
    double d = p / n;
    if (d > 1.0) {
        d = 1.0;
    } else if (d < -1.0) {
        d = -1.0;
    }
    double delta = degrees(std::acos(d));
    if ((ux * vy - uy * vx) < 0) delta = -delta;
    delta = py_mod(delta, 360);
    if (!sweep_flag) delta -= 360;
    Matrix rotate_matrix;
    post_rotate(rotate_matrix, kTau * *rotation / 360.0, center.x, center.y);
    s.center = center;
    s.prx = rotate_matrix.apply(Point{center.x + rx, center.y});
    s.pry = rotate_matrix.apply(Point{center.x, center.y + ry});
    s.sweep = kTau * delta / 360.0;
    return s;
}

/// Arc(start, end, rx=rx, ry=ry) of the corners of a rounded rectangle.
Segment corner_arc(const Point& start, const Point& end, double rx, double ry) {
    Segment s;
    s.kind = SegKind::ARC;
    s.start = start;
    s.end = end;
    s.sweep = kTau / 4.0;
    Point center{start.x, end.y};
    const bool cw = orientation(start, center, end) == 1;
    if (!cw) center = Point{end.x, start.y};
    s.center = center;
    s.prx = Point{center.x + rx, center.y};
    s.pry = Point{center.x, center.y + ry};
    return s;
}

Segment make_segment(SegKind kind, const std::optional<Point>& start, const std::optional<Point>& end) {
    Segment s;
    s.kind = kind;
    s.start = start;
    s.end = end;
    return s;
}

// ---------------------------------------------------------------------------
// Path building (svgelements.Path) and the path-data lexer (SVGLexicalParser)
// ---------------------------------------------------------------------------

/// A coordinate argument: a point, the inline close marker ("z"), or None.
struct Coord {
    enum Kind { NONE, POINT, Z } kind = NONE;
    Point p;
};

class PathBuilder {
public:
    std::vector<Segment> segs;

    std::optional<Point> current_point() const {
        if (segs.empty()) return std::nullopt;
        return segs.back().end;
    }

    std::optional<Point> z_point() const {
        std::optional<Point> end_pos;
        for (auto it = segs.rbegin(); it != segs.rend(); ++it) {
            if (it->kind == SegKind::MOVE) {
                end_pos = it->end;
                break;
            }
        }
        if (!end_pos && !segs.empty()) end_pos = segs[0].end;
        return end_pos;
    }

    std::optional<Point> smooth_point() const {
        if (segs.empty()) return std::nullopt;
        const std::optional<Point> start_pos = current_point();
        const Segment& last = segs.back();
        std::optional<Point> previous;
        if (last.kind == SegKind::QUAD) previous = last.control;
        if (last.kind == SegKind::CUBIC) previous = last.control2;
        if (last.kind == SegKind::QUAD || last.kind == SegKind::CUBIC) {
            if (!start_pos || !previous) fail("can't reflect a missing control point");
            return Point{start_pos->x + (start_pos->x - previous->x), start_pos->y + (start_pos->y - previous->y)};
        }
        return start_pos;
    }

    void append(Segment value) {
        const long index = static_cast<long>(segs.size()) - 1;
        const bool close = value.kind == SegKind::CLOSE;
        segs.push_back(std::move(value));
        validate_connection(index);
        if (close) validate_close(static_cast<std::size_t>(index + 1));
    }

    std::optional<Point> target(const Coord& c) const {
        if (c.kind == Coord::Z) return z_point();
        if (c.kind == Coord::POINT) return c.p;
        return std::nullopt;
    }

    void move(const Coord& c) { append(make_segment(SegKind::MOVE, current_point(), target(c))); }
    void line(const Coord& c) { append(make_segment(SegKind::LINE, current_point(), target(c))); }

    void vertical(std::optional<double> y, bool relative) {
        const auto start_pos = current_point();
        if (!start_pos || !y) fail("bad vertical line in path data");
        append(make_segment(SegKind::LINE, start_pos,
                            Point{start_pos->x, relative ? start_pos->y + *y : *y}));
    }

    void horizontal(std::optional<double> x, bool relative) {
        const auto start_pos = current_point();
        if (!start_pos || !x) fail("bad horizontal line in path data");
        append(make_segment(SegKind::LINE, start_pos,
                            Point{relative ? start_pos->x + *x : *x, start_pos->y}));
    }

    void smooth_quad(const Coord& end) {
        Segment s = make_segment(SegKind::QUAD, current_point(), std::nullopt);
        s.control = smooth_point();
        s.end = target(end);
        append(std::move(s));
    }

    void quad(const Coord& control, const Coord& end) {
        Segment s = make_segment(SegKind::QUAD, current_point(), std::nullopt);
        if (control.kind == Coord::Z) {
            const auto z = z_point();
            s.control = z;
            s.end = z;
            append(std::move(s));
            return;
        }
        s.control = target(control);
        s.end = target(end);
        append(std::move(s));
    }

    void smooth_cubic(const Coord& control2, const Coord& end) {
        Segment s = make_segment(SegKind::CUBIC, current_point(), std::nullopt);
        s.control1 = smooth_point();
        if (control2.kind == Coord::Z) {
            const auto z = z_point();
            s.control2 = z;
            s.end = z;
            append(std::move(s));
            return;
        }
        s.control2 = target(control2);
        s.end = target(end);
        append(std::move(s));
    }

    void cubic(const Coord& c1, const Coord& c2, const Coord& end) {
        Segment s = make_segment(SegKind::CUBIC, current_point(), std::nullopt);
        if (c1.kind == Coord::Z) {
            const auto z = z_point();
            s.control1 = z;
            s.control2 = z;
            s.end = z;
            append(std::move(s));
            return;
        }
        s.control1 = target(c1);
        if (c2.kind == Coord::Z) {
            const auto z = z_point();
            s.control2 = z;
            s.end = z;
            append(std::move(s));
            return;
        }
        s.control2 = target(c2);
        s.end = target(end);
        append(std::move(s));
    }

    void arc(std::optional<double> rx, std::optional<double> ry, std::optional<double> rotation,
             std::optional<bool> large_arc, std::optional<bool> sweep, const Coord& end) {
        if (!rx || !ry) fail("bad arc radius in path data");
        double rxv = *rx, ryv = *ry;
        if (rxv < 0) rxv = std::fabs(rxv);
        if (ryv < 0) ryv = std::fabs(ryv);
        append(svg_arc(current_point(), rxv, ryv, rotation, large_arc.value_or(false), sweep.value_or(false),
                       target(end)));
    }

    void closed() { append(make_segment(SegKind::CLOSE, current_point(), z_point())); }

private:
    void validate_connection(long index) {
        if (index < 0 || static_cast<std::size_t>(index + 1) >= segs.size()) return;
        Segment& first = segs[static_cast<std::size_t>(index)];
        Segment& second = segs[static_cast<std::size_t>(index + 1)];
        if (first.end && !second.start) {
            second.start = first.end;
        } else if (!first.end && second.start) {
            first.end = second.start;
        } else if (first.end && second.start && !same_point(*first.end, *second.start)) {
            second.start = first.end;
        }
    }

    void validate_close(std::size_t index) {
        for (std::size_t i = index + 1; i-- > 0;) {
            if (segs[i].kind == SegKind::MOVE) {
                segs[index].end = segs[i].end;
                return;
            }
        }
        segs[index].end = segs[0].end;
    }
};

class Lexer {
public:
    Lexer(PathBuilder& parser, const std::string& pathd) : parser_(parser), d_(pathd), limit_(pathd.size()) {}

    void parse() {
        while (true) {
            const auto cmd = command();
            if (!cmd) return;
            const char c = *cmd;
            if (c == 'z' || c == 'Z') {
                if (more()) throw ValueError("unexpected value after close");
                parser_.closed();
                inline_close_ = false;
                continue;
            }
            switch (c) {
                case 'm':
                case 'M': {
                    if (!more()) throw ValueError("moveto without coordinates");
                    parser_.move(c == 'm' ? rcoord() : coord());
                    while (more()) parser_.line(c == 'm' ? rcoord() : coord());
                    break;
                }
                case 'l':
                case 'L':
                case 't':
                case 'T':
                    while (true) {
                        Coord p = c == 'l' || c == 't' ? rcoord() : coord();
                        p = or_close(p);
                        if (c == 'l' || c == 'L') {
                            parser_.line(p);
                        } else {
                            parser_.smooth_quad(p);
                        }
                        if (!more()) break;
                    }
                    break;
                case 'h':
                case 'H':
                    while (true) {
                        parser_.horizontal(number(), c == 'h');
                        if (!more()) break;
                    }
                    break;
                case 'v':
                    while (true) {
                        parser_.vertical(number(), true);
                        if (!more()) break;
                    }
                    break;
                case 'V':
                    while (more()) parser_.vertical(number(), false);
                    break;
                case 'c':
                case 'C':
                    while (true) {
                        const bool rel = c == 'c';
                        Coord c1 = rel ? rcoord() : coord();
                        Coord c2 = rel ? rcoord() : coord();
                        Coord c3 = rel ? rcoord() : coord();
                        c1 = or_close(c1);
                        c2 = or_close(c2);
                        c3 = or_close(c3);
                        parser_.cubic(c1, c2, c3);
                        if (!more()) break;
                    }
                    break;
                case 'q':
                case 'Q':
                case 's':
                case 'S':
                    while (true) {
                        const bool rel = c == 'q' || c == 's';
                        Coord c1 = rel ? rcoord() : coord();
                        Coord c2 = rel ? rcoord() : coord();
                        c1 = or_close(c1);
                        c2 = or_close(c2);
                        if (c == 'q' || c == 'Q') {
                            parser_.quad(c1, c2);
                        } else {
                            parser_.smooth_cubic(c1, c2);
                        }
                        if (!more()) break;
                    }
                    break;
                case 'a':
                case 'A':
                    while (more()) {
                        const auto rx = number();
                        const auto ry = number();
                        const auto rotation = number();
                        const auto large = flag();
                        const auto sweep = flag();
                        Coord p = c == 'a' ? rcoord() : coord();
                        if (c == 'a' && !sweep) throw ValueError("arc without a sweep flag");
                        p = or_close(p);
                        parser_.arc(rx, ry, rotation, large, sweep, p);
                    }
                    break;
                default: break;
            }
        }
    }

private:
    PathBuilder& parser_;
    const std::string& d_;
    std::size_t pos_ = 0;
    std::size_t limit_;
    bool inline_close_ = false;

    static bool is_command(char c) { return std::strchr("MmZzLlHhVvCcSsQqTtAa", c) != nullptr && c != '\0'; }
    static bool is_skip(char c) { return c == ' ' || c == ',' || c == '\t' || c == '\n' || c == '\f' || c == '\r'; }

    std::size_t skip_length() const {
        std::size_t n = 0;
        while (pos_ + n < limit_ && is_skip(d_[pos_ + n])) ++n;
        return n;
    }

    std::optional<char> command() {
        while (pos_ < limit_) {
            const char c = d_[pos_];
            if (is_command(c)) {
                ++pos_;
                return c;
            }
            if (is_skip(c)) {
                pos_ += skip_length();
                continue;
            }
            return std::nullopt;
        }
        return std::nullopt;
    }

    enum class Tok { NONE, FLOAT, CLOSE, SKIP };
    Tok num_token(std::size_t& len) const {
        len = match_float(d_, pos_);
        if (len) return Tok::FLOAT;
        const char c = d_[pos_];
        if (c == 'z' || c == 'Z') {
            len = 1;
            return Tok::CLOSE;
        }
        if (is_skip(c)) {
            len = skip_length();
            return Tok::SKIP;
        }
        return Tok::NONE;
    }

    bool more() {
        while (pos_ < limit_) {
            std::size_t len = 0;
            const Tok t = num_token(len);
            if (t == Tok::NONE) return false;
            if (t == Tok::CLOSE) {
                inline_close_ = true;
                return false;
            }
            if (t == Tok::SKIP) {
                pos_ += len;
                continue;
            }
            return true;
        }
        return false;
    }

    std::optional<double> number() {
        while (pos_ < limit_) {
            std::size_t len = 0;
            const Tok t = num_token(len);
            if (t == Tok::NONE) break;
            if (t == Tok::CLOSE) {
                inline_close_ = true;
                return std::nullopt;
            }
            const std::size_t start = pos_;
            pos_ += len;
            if (t == Tok::SKIP) continue;
            return to_float(d_.substr(start, len));
        }
        return std::nullopt;
    }

    std::optional<bool> flag() {
        while (pos_ < limit_) {
            const char c = d_[pos_];
            if (c == '0' || c == '1') {
                ++pos_;
                return c == '1';
            }
            if (is_skip(c)) {
                pos_ += skip_length();
                continue;
            }
            break;
        }
        return std::nullopt;
    }

    Coord coord() {
        const auto x = number();
        if (!x) return Coord{};
        const auto y = number();
        if (!y) throw ValueError("coordinate without y value");
        return Coord{Coord::POINT, Point{*x, *y}};
    }

    Coord rcoord() {
        Coord position = coord();
        if (position.kind == Coord::NONE) return position;
        const auto current = parser_.current_point();
        if (!current) return position;
        position.p = Point{position.p.x + current->x, position.p.y + current->y};
        return position;
    }

    /// A missing coordinate is the inline close, else an error.
    Coord or_close(const Coord& c) const {
        if (c.kind != Coord::NONE) return c;
        if (!inline_close_) throw ValueError("missing coordinate in path data");
        return Coord{Coord::Z, {}};
    }
};

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------

using Values = std::map<std::string, std::string>;

const std::string* get(const Values& values, const std::string& key) {
    const auto it = values.find(key);
    return it == values.end() ? nullptr : &it->second;
}

/// A shape as svgelements builds it in SVG.parse: created from values, rendered, reified.
struct ParsedShape {
    ShapeElement element;
    // Rect
    double x = 0, y = 0, width = 0, height = 0, rx = 0, ry = 0;
    // SimpleLine
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    // Polyline / Polygon
    std::vector<Point> points;
    // Path
    std::vector<Segment> segments;
    bool parse_error = false;
};

/// GraphicObject.property_by_values: fill and stroke with their opacities.
void graphic_properties(ShapeElement& e, const Values& values) {
    auto color = [&values](const char* key, const char* opacity_key, const char* opacity_alias) {
        Color c;
        const std::string* text = get(values, key);
        if (text == nullptr) fail(std::string("no ") + key);  // never: the root sets fill and stroke
        c.value = color_parse(*text);
        const std::string* opacity = get(values, opacity_key);
        if (opacity == nullptr) opacity = get(values, opacity_alias);
        if (opacity != nullptr && c.value) {
            const auto v = py_float(*opacity);
            if (v) set_opacity(c, *v);
        }
        return c;
    };
    e.stroke = color("stroke", "stroke-opacity", "stroke_opacity");
    e.fill = color("fill", "fill-opacity", "fill_opacity");
}

/// Shape(values): the element's values, transform, fill and stroke, and its geometry.
ParsedShape create_shape(ShapeKind kind, const Values& values, const RenderInfo& info) {
    ParsedShape s;
    s.element.kind = kind;
    s.element.values = values;
    const std::string* transform = get(values, "transform");
    s.element.transform = parse_transform(transform ? *transform : std::string(), info);
    graphic_properties(s.element, values);
    switch (kind) {
        case ShapeKind::PATH: {
            const std::string* d = get(values, "d");
            if (d == nullptr) fail("<path> without 'd' attribute");  // TypeError in svgelements
            PathBuilder builder;
            try {
                Lexer(builder, *d).parse();
            } catch (const ValueError&) {
                s.parse_error = true;  // svgelements keeps the path read so far
            }
            s.segments = std::move(builder.segs);
            break;
        }
        case ShapeKind::RECT: {
            s.x = render_length(length_value(length_attr(values, "x", "0")), info, false, "x");
            s.y = render_length(length_value(length_attr(values, "y", "0")), info, true, "y");
            s.width = render_length(length_value(length_attr(values, "width", "1")), info, false, "width");
            s.height = render_length(length_value(length_attr(values, "height", "1")), info, true, "height");
            // Rect._validate_rect: percentages of rx/ry are of the rectangle's size
            const LV rx = length_value(length_attr(values, "rx", nullptr));
            const LV ry = length_value(length_attr(values, "ry", nullptr));
            auto corner = [&](const LV& r, double size, bool vertical) {
                if (!r.resolved && r.len.units == "%") return r.len.amount / 100.0 * size;
                return render_length(r, info, vertical, "a corner radius");
            };
            double rxv = 0, ryv = 0;
            if (rx.none && ry.none) {
                rxv = ryv = 0;
            } else if (!rx.none && ry.none) {
                rxv = corner(rx, s.width, false);
                ryv = rxv;
            } else if (rx.none && !ry.none) {
                ryv = corner(ry, s.height, true);
                rxv = ryv;
            } else {
                rxv = corner(rx, s.width, false);
                ryv = corner(ry, s.height, true);
            }
            if (rxv == 0 || ryv == 0) {
                rxv = ryv = 0;
            } else {
                rxv = std::min(rxv, s.width / 2.0);
                ryv = std::min(ryv, s.height / 2.0);
            }
            s.rx = rxv;
            s.ry = ryv;
            break;
        }
        case ShapeKind::CIRCLE:
        case ShapeKind::ELLIPSE: {
            LV cx = length_value(length_attr(values, "cx", nullptr));
            LV cy = length_value(length_attr(values, "cy", nullptr));
            LV rx = length_value(length_attr(values, "rx", nullptr));
            LV ry = length_value(length_attr(values, "ry", nullptr));
            const LV r = length_value(length_attr(values, "r", nullptr));
            if (!r.none) {
                rx = r;
                ry = r;
            } else {
                if (rx.none) rx = lv_float(1);
                if (ry.none) ry = lv_float(1);
            }
            if (const std::string* center = get(values, "center")) {
                std::smatch m;
                if (!std::regex_search(*center, m, re_coord_pair())) fail("bad center '" + *center + "'");
                cx = lv_float(to_float(m[1].str()));
                cy = lv_float(to_float(m[2].str()));
            }
            if (cx.none) cx = lv_float(0);
            if (cy.none) cy = lv_float(0);
            s.element.cx = render_length(cx, info, false, "cx");
            s.element.cy = render_length(cy, info, true, "cy");
            s.element.rx = render_length(rx, info, false, "rx");
            s.element.ry = render_length(ry, info, true, "ry");
            break;
        }
        case ShapeKind::LINE: {
            s.x1 = render_length(length_value(length_attr(values, "x1", "0")), info, false, "x1");
            s.y1 = render_length(length_value(length_attr(values, "y1", "0")), info, true, "y1");
            s.x2 = render_length(length_value(length_attr(values, "x2", "0")), info, false, "x2");
            s.y2 = render_length(length_value(length_attr(values, "y2", "0")), info, true, "y2");
            break;
        }
        case ShapeKind::POLYLINE:
        case ShapeKind::POLYGON: {
            if (const std::string* points = get(values, "points")) {
                for (auto it = std::sregex_iterator(points->begin(), points->end(), re_coord_pair());
                     it != std::sregex_iterator(); ++it) {
                    s.points.push_back(Point{to_float((*it)[1].str()), to_float((*it)[2].str())});
                }
            }
            break;
        }
    }
    return s;
}

/// shape.reify()
void reify(ParsedShape& s) {
    Matrix& t = s.element.transform;
    switch (s.element.kind) {
        case ShapeKind::PATH:
            for (auto& seg : s.segments) seg.transform(t);
            t = Matrix();
            break;
        case ShapeKind::RECT:
        case ShapeKind::CIRCLE:
        case ShapeKind::ELLIPSE: {
            const double scale_x = t.a;
            const double scale_y = t.d;
            if (scale_x * scale_y < 0) return;  // no reification of flipped values
            const double translate_x = t.e;
            const double translate_y = t.f;
            if (t.b == 0 && t.c == 0 && scale_x != 0 && scale_y != 0) {
                double& x = s.element.kind == ShapeKind::RECT ? s.x : s.element.cx;
                double& y = s.element.kind == ShapeKind::RECT ? s.y : s.element.cy;
                double& rx = s.element.kind == ShapeKind::RECT ? s.rx : s.element.rx;
                double& ry = s.element.kind == ShapeKind::RECT ? s.ry : s.element.ry;
                x *= scale_x;
                y *= scale_y;
                x += translate_x;
                y += translate_y;
                post_cat(t, m_translate(-translate_x, -translate_y));
                rx = scale_x * rx;
                ry = scale_y * ry;
                if (s.element.kind == ShapeKind::RECT) {
                    s.width = scale_x * s.width;
                    s.height = scale_y * s.height;
                }
                post_cat(t, m_scale(1.0 / scale_x, 1.0 / scale_y));
            }
            break;
        }
        case ShapeKind::LINE: {
            const Point p1 = t.apply(Point{s.x1, s.y1});
            s.x1 = p1.x;
            s.y1 = p1.y;
            const Point p2 = t.apply(Point{s.x2, s.y2});
            s.x2 = p2.x;
            s.y2 = p2.y;
            t = Matrix();
            break;
        }
        case ShapeKind::POLYLINE:
        case ShapeKind::POLYGON:
            for (auto& p : s.points) p = t.apply(p);
            t = Matrix();
            break;
    }
}

/// shape.is_degenerate() (with the transform applied, as SVG.parse calls it)
bool is_degenerate(const ParsedShape& s) {
    switch (s.element.kind) {
        case ShapeKind::RECT: return s.width == 0 || s.height == 0;
        case ShapeKind::CIRCLE:
        case ShapeKind::ELLIPSE: {
            const Matrix& t = s.element.transform;
            const Point origin = t.apply(Point{0, 0});
            const double rx = distance_to(origin, t.apply(Point{s.element.rx, 0}));
            const double ry = distance_to(origin, t.apply(Point{0, s.element.ry}));
            return rx == 0 || ry == 0;
        }
        case ShapeKind::POLYLINE:
        case ShapeKind::POLYGON: return s.points.empty();
        default: return false;
    }
}

/// shape.segments(transformed=False)
std::vector<Segment> untransformed_segments(const ParsedShape& s) {
    std::vector<Segment> out;
    switch (s.element.kind) {
        case ShapeKind::PATH: return s.segments;
        case ShapeKind::RECT: {
            const double x = s.x, y = s.y, width = s.width, height = s.height;
            if (is_degenerate(s)) return out;
            double rx = s.rx, ry = s.ry;
            if ((rx < 0 && 0 < width) || (ry < 0 && 0 < height)) {
                rx = 0;
                ry = 0;
            }
            if (rx == ry && ry == 0) {
                out.push_back(make_segment(SegKind::MOVE, std::nullopt, Point{x, y}));
                out.push_back(make_segment(SegKind::LINE, Point{x, y}, Point{x + width, y}));
                out.push_back(make_segment(SegKind::LINE, Point{x + width, y}, Point{x + width, y + height}));
                out.push_back(make_segment(SegKind::LINE, Point{x + width, y + height}, Point{x, y + height}));
                out.push_back(make_segment(SegKind::CLOSE, Point{x, y + height}, Point{x, y}));
            } else {
                out.push_back(make_segment(SegKind::MOVE, std::nullopt, Point{x + rx, y}));
                out.push_back(make_segment(SegKind::LINE, Point{x + rx, y}, Point{x + width - rx, y}));
                out.push_back(corner_arc(Point{x + width - rx, y}, Point{x + width, y + ry}, rx, ry));
                out.push_back(make_segment(SegKind::LINE, Point{x + width, y + ry}, Point{x + width, y + height - ry}));
                out.push_back(corner_arc(Point{x + width, y + height - ry}, Point{x + width - rx, y + height}, rx, ry));
                out.push_back(make_segment(SegKind::LINE, Point{x + width - rx, y + height}, Point{x + rx, y + height}));
                out.push_back(corner_arc(Point{x + rx, y + height}, Point{x, y + height - ry}, rx, ry));
                out.push_back(make_segment(SegKind::LINE, Point{x, y + height - ry}, Point{x, y + ry}));
                out.push_back(corner_arc(Point{x, y + ry}, Point{x + rx, y}, rx, ry));
                out.push_back(make_segment(SegKind::CLOSE, Point{x + rx, y}, Point{x + rx, y}));
            }
            return out;
        }
        case ShapeKind::LINE: {
            const Point start{s.x1, s.y1};
            const Point end{s.x2, s.y2};
            out.push_back(make_segment(SegKind::MOVE, std::nullopt, start));
            out.push_back(make_segment(SegKind::LINE, start, end));
            return out;
        }
        case ShapeKind::POLYLINE:
        case ShapeKind::POLYGON: {
            if (s.points.empty()) return out;
            out.push_back(make_segment(SegKind::MOVE, std::nullopt, s.points[0]));
            Point last = s.points[0];
            for (std::size_t i = 1; i < s.points.size(); ++i) {
                out.push_back(make_segment(SegKind::LINE, last, s.points[i]));
                last = s.points[i];
            }
            if (s.element.kind == ShapeKind::POLYGON) out.push_back(make_segment(SegKind::CLOSE, last, s.points[0]));
            return out;
        }
        default: return out;
    }
}

// ---------------------------------------------------------------------------
// CSS
// ---------------------------------------------------------------------------

/// re.sub(REGEX_CSS_COMMENT, "", text): /* ... */ and // to the end of the line.
std::string strip_css_comments(const std::string& text) {
    std::string out;
    std::size_t i = 0;
    while (i < text.size()) {
        if (text.compare(i, 2, "/*") == 0) {
            const auto e = text.find("*/", i + 2);
            if (e != std::string::npos) {
                i = e + 2;
                continue;
            }
        }
        if (text.compare(i, 2, "//") == 0) {
            const auto e = text.find('\n', i);
            i = e == std::string::npos ? text.size() : e;
            continue;
        }
        out += text[i++];
    }
    return out;
}

/// re.findall(REGEX_CSS_STYLE, text): ([^{]+)\s*\{\s*([^}]+)\s*\}
std::vector<std::pair<std::string, std::string>> css_rules(const std::string& text) {
    std::vector<std::pair<std::string, std::string>> out;
    std::size_t pos = 0;
    const std::size_t n = text.size();
    while (pos < n) {
        const std::size_t brace = text.find('{', pos);
        if (brace == std::string::npos) break;
        if (brace == pos) {
            ++pos;
            continue;
        }
        std::size_t m = brace + 1;
        while (m < n && is_space(text[m])) ++m;
        std::size_t body = m;
        if (m < n && text[m] == '}') {
            if (m == brace + 1) {  // "{}": no match here
                ++pos;
                continue;
            }
            body = m - 1;  // the last blank is the body
        }
        const std::size_t close = text.find('}', std::max(body, m));
        if (close == std::string::npos) break;  // nothing later can match either
        out.emplace_back(text.substr(pos, brace - pos), text.substr(body, close - body));
        pos = close + 1;
    }
    return out;
}

// ---------------------------------------------------------------------------
// SVG.parse
// ---------------------------------------------------------------------------

constexpr const char* kSvgNamespace = "{http://www.w3.org/2000/svg";

class SvgParser {
public:
    SvgParser(bool parse_display_none, double ppi) : parse_display_none_(parse_display_none), ppi_(ppi) {}

    Document run(const xml::Node& root) {
        index_ids(root);
        values_ = {{"color", "black"}, {"fill", "black"}, {"stroke", "none"}};
        try {
            semiparse(root, true, 0);
        } catch (const Terminated&) {
        }
        if (!result_) fail("the document has no visible <svg> element");
        if (!result_is_svg_) fail("the document's root element is not <svg>");
        return finish();
    }

private:
    struct Terminated {};
    struct Context {
        bool reachable = false;
    };
    struct SvgResult {
        Values values;
        LV width, height;
        std::optional<Viewbox> viewbox;
    };

    bool parse_display_none_;
    double ppi_;
    std::map<std::string, const xml::Node*> defs_;
    Values values_;
    std::optional<Context> context_;
    std::vector<std::pair<std::optional<Context>, Values>> stack_;
    std::map<std::string, std::string> styles_;
    LV width_ = lv_none();
    LV height_ = lv_none();
    bool root_set_ = false;
    std::optional<SvgResult> result_;
    bool result_is_svg_ = false;
    std::vector<ShapeElement> shapes_;

    void index_ids(const xml::Node& node) {
        if (const std::string* id = node.get("id")) defs_[*id] = &node;
        for (const auto& child : node.children) index_ids(*child);
    }

    void semiparse(const xml::Node& node, bool own_namespaces, int depth) {
        if (depth > 256) fail("<use> elements nest too deeply");
        if (own_namespaces) {
            for (const auto& [prefix, uri] : node.namespaces) {
                if (prefix != "d") values_[prefix] = uri;
            }
        }
        std::string tag = node.tag;
        if (tag.compare(0, std::strlen(kSvgNamespace), kSvgNamespace) == 0) tag = tag.substr(28);
        start(node, tag);
        for (const auto& child : node.children) semiparse(*child, true, depth + 1);
        if (tag == "use") {
            const std::string* url = node.get("{http://www.w3.org/1999/xlink}href");
            if (const std::string* href = node.get("href")) url = href;
            if (url != nullptr) {
                const auto it = defs_.find(url->empty() ? std::string() : url->substr(1));
                if (it != defs_.end()) semiparse(*it->second, false, depth + 1);
            }
        }
        end(node, tag);
    }

    static bool display_none(const Values& values) {
        const std::string* d = get(values, "display");
        return d != nullptr && lower(*d) == "none";
    }

    RenderInfo render_info() const {
        RenderInfo info;
        info.ppi = ppi_;
        info.width = width_;
        info.height = height_;
        return info;
    }

    void start(const xml::Node& node, const std::string& tag) {
        stack_.emplace_back(context_, values_);
        if (!parse_display_none_ && display_none(values_)) return;
        Values values = values_;
        for (const char* key : {"preserveAspectRatio", "viewBox", "id", "class", "clip-path"}) values.erase(key);
        Values attributes;
        for (const auto& [k, v] : node.attrib) attributes[k] = v;
        attributes["tag"] = tag;
        // style rules, then the style attribute
        std::string style;
        if (const auto it = styles_.find("*"); it != styles_.end()) style += it->second;
        if (const auto it = styles_.find(tag); it != styles_.end()) style += it->second;
        auto add_style = [&style, this](const std::string& selector) {
            const auto it = styles_.find(selector);
            if (it == styles_.end()) return;
            if (!style.empty()) style += ";";
            style += it->second;
        };
        if (const std::string* id = get(attributes, "id")) add_style("#" + *id);
        if (const std::string* cls = get(attributes, "class")) {
            for (const auto& svg_class : split(*cls, ' ')) {
                add_style("." + svg_class);
                add_style(tag + "." + svg_class);
            }
        }
        if (const std::string* s = get(attributes, "style")) {
            if (!style.empty()) style += ";";
            style += *s;
        }
        for (const auto& equate : split(style, ';')) {
            const auto equal_item = split(equate, ':');
            if (equal_item.size() == 2) attributes[strip(equal_item[0])] = strip(equal_item[1]);
        }
        for (const char* key : {"fill", "stroke"}) {
            const auto it = attributes.find(key);
            if (it != attributes.end() && it->second == "currentColor") {
                const std::string* color = get(attributes, "color");
                it->second = color ? *color : values["color"];
            }
        }
        if (const auto it = attributes.find("transform"); it != attributes.end()) {
            if (const std::string* inherited = get(values, "transform")) it->second = *inherited + " " + it->second;
        }
        for (const auto& [k, v] : attributes) values[k] = v;
        values_ = values;
        if (!parse_display_none_ && display_none(values_)) return;

        if (tag == "svg") {
            check_transform();
            start_svg();
        } else if (tag == "g") {
            check_transform();
            open_context(true);
        } else if (tag == "defs" || tag == "clipPath" || tag == "pattern") {
            if (tag == "defs") check_transform();
            context_ = Context{false};
        } else if (tag == "use") {
            start_use();
        } else if (tag == "path" || tag == "circle" || tag == "ellipse" || tag == "line" || tag == "polyline" ||
                   tag == "polygon" || tag == "rect") {
            if (!start_shape(tag)) return;
        } else if (tag == "image") {
            // an Image is not a Shape: ocpsvg ignores it
        } else if (tag == "style" || tag == "text" || tag == "desc" || tag == "title" || tag == "tspan") {
            return;
        }
        if (!root_set_) {
            root_set_ = true;
            if (tag != "svg") {
                result_ = SvgResult{};
                result_is_svg_ = false;
            }
        }
    }

    void end(const xml::Node& node, const std::string& tag) {
        if (display_none(values_)) {
            pop();
            return;
        }
        if (tag == "text" || tag == "tspan") check_transform();
        if (tag == "style") {
            const std::string text = strip(strip_css_comments(node.text));
            for (const auto& [key_raw, value_raw] : css_rules(text)) {
                const std::string value = strip(value_raw);
                for (const auto& selector : split(strip(key_raw), ',')) {
                    const std::string sel = strip(selector);
                    auto it = styles_.find(sel);
                    if (it == styles_.end()) {
                        styles_[sel] = value;
                    } else {
                        if (!ends_with(it->second, ";")) it->second += ";";
                        it->second += value;
                    }
                }
            }
        }
        pop();
    }

    void pop() {
        context_ = stack_.back().first;
        values_ = stack_.back().second;
        stack_.pop_back();
    }

    /// Append a container to the current context and make it the context.
    void open_context(bool rendered) {
        const bool reachable = rendered && (!context_ || context_->reachable);
        context_ = Context{reachable};
    }

    void start_svg() {
        SvgResult s;
        s.values = values_;
        s.width = length_value(length_attr(values_, "width", "100%"));
        s.height = length_value(length_attr(values_, "height", "100%"));
        LV x = length_value(length_attr(values_, "x", "0"));
        LV y = length_value(length_attr(values_, "y", "0"));
        if (const std::string* vb = get(values_, "viewBox")) {
            const auto dims = find_floats(*vb);
            Viewbox v;
            if (dims.size() > 0) v.x = dims[0];
            if (dims.size() > 1) v.y = dims[1];
            if (dims.size() > 2) v.width = dims[2];
            if (dims.size() > 3) v.height = dims[3];
            s.viewbox = v;
        }
        if (width_.none) width_ = s.viewbox ? (s.viewbox->width ? lv_float(*s.viewbox->width) : lv_none()) : lv_float(1000);
        if (height_.none) {
            height_ = s.viewbox ? (s.viewbox->height ? lv_float(*s.viewbox->height) : lv_none()) : lv_float(1000);
        }
        // SVG.render
        auto rel = [](const LV& v) -> std::optional<double> {
            if (v.none || !v.resolved) return std::nullopt;
            return v.v;
        };
        const Viewbox* vbp = s.viewbox ? &*s.viewbox : nullptr;
        s.width = length_value(rewrap(s.width), ppi_, rel(width_), vbp);
        s.height = length_value(rewrap(s.height), ppi_, rel(height_), vbp);
        x = length_value(rewrap(x), ppi_, rel(width_), vbp);
        y = length_value(rewrap(y), ppi_, rel(height_), vbp);
        // height, width = s.width, s.height (sic)
        width_ = s.height;
        height_ = s.width;
        const bool first = !root_set_;
        if (s.viewbox) {
            auto is_zero = [](const LV& v) { return v.resolved ? v.v == 0 : v.len.amount == 0; };
            if (is_zero(s.height) || is_zero(s.width)) terminate(std::move(s));
            const Viewbox& vb = *s.viewbox;
            std::string viewport_transform;
            if (vb.x && vb.y && vb.width && vb.height) {
                // a zero-sized viewBox disables rendering (ZeroDivisionError in svgelements)
                if (*vb.width == 0 || *vb.height == 0) terminate(std::move(s));
                if (!s.width.resolved || !s.height.resolved || !x.resolved || !y.resolved) {
                    fail("can't resolve the size of an <svg> element");  // ValueError in viewbox_transform
                }
                const std::string* par = get(values_, "preserveAspectRatio");
                viewport_transform = viewbox_transform(x.v, y.v, s.width.v, s.height.v, vb,
                                                       par ? std::optional<std::string>(*par) : std::nullopt);
            }
            if (const std::string* t = get(values_, "transform")) {
                values_["transform"] = *t + " " + viewport_transform;
            } else {
                values_["transform"] = viewport_transform;
            }
            values_["viewport_transform"] = values_["transform"];
            width_ = s.viewbox->width ? lv_float(*s.viewbox->width) : lv_none();
            height_ = s.viewbox->height ? lv_float(*s.viewbox->height) : lv_none();
        }
        if (!context_) stack_.back() = {context_, values_};
        open_context(true);
        if (first) {
            root_set_ = true;
            result_ = std::move(s);
            result_is_svg_ = true;
        }
    }

    [[noreturn]] void terminate(SvgResult s) {
        // SVG.parse returns this <svg> element: nothing else is read
        result_ = std::move(s);
        result_is_svg_ = true;
        shapes_.clear();
        throw Terminated{};
    }

    void start_use() {
        // Use.property_by_values: x and y become a translation
        const LV x = length_value(length_attr(values_, "x", "0"));
        const LV y = length_value(length_attr(values_, "y", "0"));
        auto nonzero = [](const LV& v) { return v.resolved ? v.v != 0 : v.len.amount != 0; };
        auto text = [](const LV& v) {
            return v.resolved ? pyfmt::repr(v.v) : length_str(v.len.amount) + v.len.units;
        };
        if (nonzero(x) || nonzero(y)) {
            const std::string t = "translate(" + text(x) + ", " + text(y) + ")";
            if (const std::string* current = get(values_, "transform")) {
                values_["transform"] = *current + " " + t;
            } else {
                values_["transform"] = t;
            }
        }
        check_transform();
        for (const char* key : {"x", "y", "width", "height"}) values_.erase(key);
        open_context(true);
    }

    /// Transformable(values) parses the transform, raising what svgelements raises.
    void check_transform() const {
        if (const std::string* t = get(values_, "transform")) {
            RenderInfo info = render_info();
            info.lenient = true;
            parse_transform(*t, info);
        }
    }

    /// A shape element; false where svgelements skips the rest of its start handling.
    bool start_shape(const std::string& tag) {
        ShapeKind kind = ShapeKind::PATH;
        if (tag == "circle") kind = ShapeKind::CIRCLE;
        if (tag == "ellipse") kind = ShapeKind::ELLIPSE;
        if (tag == "line") kind = ShapeKind::LINE;
        if (tag == "polyline") kind = ShapeKind::POLYLINE;
        if (tag == "polygon") kind = ShapeKind::POLYGON;
        if (tag == "rect") kind = ShapeKind::RECT;
        ParsedShape s;
        try {
            s = create_shape(kind, values_, render_info());
        } catch (const ValueError&) {
            return false;  // the shape couldn't be created: skipped
        }
        reify(s);
        if (is_degenerate(s)) return false;
        if (!context_ || context_->reachable) collect(std::move(s));
        return !s.parse_error;
    }

    /// ocpsvg's find_shapes_svg_in_document for one shape.
    void collect(ParsedShape s) {
        if (!parse_display_none_) {
            const std::string* visibility = get(s.element.values, "visibility");
            if (visibility != nullptr && (*visibility == "hidden" || *visibility == "collapse")) return;
        }
        ShapeElement& e = s.element;
        if (e.kind == ShapeKind::CIRCLE || e.kind == ShapeKind::ELLIPSE) {
            if (e.rx == 0 || e.ry == 0) return;  // Path(element) is empty
            shapes_.push_back(std::move(e));
            return;
        }
        // svgelements.Path(element).reify()
        e.path = untransformed_segments(s);
        if (e.path.empty()) return;
        for (auto& seg : e.path) seg.transform(e.transform);
        shapes_.push_back(std::move(e));
    }

    Document finish() {
        Document doc;
        doc.shapes = std::move(shapes_);
        const SvgResult& s = *result_;
        doc.values = s.values;
        doc.width = s.width.none ? 0.0 : float_of(s.width);
        doc.height = s.height.none ? 0.0 : float_of(s.height);
        if (s.viewbox) {
            if (!s.viewbox->x || !s.viewbox->y || !s.viewbox->width || !s.viewbox->height) {
                fail("the viewBox needs 4 numbers");
            }
            doc.has_viewbox = true;
            doc.vb_x = *s.viewbox->x;
            doc.vb_y = *s.viewbox->y;
            doc.vb_width = *s.viewbox->width;
            doc.vb_height = *s.viewbox->height;
        }
        return doc;
    }
};

}  // namespace

bool same_point(const Point& a, const Point& b) {
    return std::fabs(a.x - b.x) <= kError && std::fabs(a.y - b.y) <= kError;
}

Point Matrix::apply(const Point& p) const { return Point{p.x * a + p.y * c + 1 * e, p.x * b + p.y * d + 1 * f}; }

Matrix Matrix::multiply(const Matrix& m, const Matrix& s) {
    const double r00 = s.a * m.a + s.c * m.b + s.e * 0;
    const double r01 = s.a * m.c + s.c * m.d + s.e * 0;
    const double r02 = s.a * m.e + s.c * m.f + s.e * 1;
    const double r10 = s.b * m.a + s.d * m.b + s.f * 0;
    const double r11 = s.b * m.c + s.d * m.d + s.f * 0;
    const double r12 = s.b * m.e + s.d * m.f + s.f * 1;
    return Matrix{r00, r10, r01, r11, r02, r12};
}

void Segment::transform(const Matrix& m) {
    if (start) start = m.apply(*start);
    if (kind == SegKind::QUAD && control) control = m.apply(*control);
    if (kind == SegKind::CUBIC) {
        if (control1) control1 = m.apply(*control1);
        if (control2) control2 = m.apply(*control2);
    }
    if (kind == SegKind::ARC) {
        center = m.apply(center);
        prx = m.apply(prx);
        pry = m.apply(pry);
    }
    if (end) end = m.apply(*end);
    if (kind == SegKind::ARC && m.determinant() < 0) sweep = -sweep;
}

double Segment::rx() const { return point_distance(center, prx); }
double Segment::ry() const { return point_distance(center, pry); }
double Segment::rotation() const { return point_angle(center, prx); }

double Segment::theta() const {
    if (!start) fail("an arc without a start point");
    const double start_t = t_at_point(*this, point_at_angle(*this, point_angle(center, *start)));
    double v = start_t * 360.0 / kTau;
    while (v < 0) v += 360.0;
    return v;
}

double Segment::delta() const { return sweep * 360.0 / kTau; }

Document parse(const std::string& text, bool ignore_visibility, double ppi) {
    const auto root = xml::parse(text);
    return SvgParser(ignore_visibility, ppi).run(*root);
}

}  // namespace build123d::svgdoc
