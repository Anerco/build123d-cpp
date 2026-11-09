// Port of the parts of svgpathtools (path.py) that build123d uses.
#include "svgpath.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "build123d/warnings.hpp"
#include "pyformat.hpp"

namespace build123d::svgpath {

// ---------------------------------------------------------------------------
// Complex arithmetic
// ---------------------------------------------------------------------------

Cx py_add(Cx a, Cx b) { return {a.re + b.re, a.im + b.im}; }
Cx py_sub(Cx a, Cx b) { return {a.re - b.re, a.im - b.im}; }
Cx py_mul(Cx a, Cx b) { return {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re}; }

Cx py_div(Cx a, Cx b) {
    // CPython's _Py_c_quot
    const double abs_breal = b.re < 0 ? -b.re : b.re;
    const double abs_bimag = b.im < 0 ? -b.im : b.im;
    if (abs_breal >= abs_bimag) {
        if (abs_breal == 0.0) throw std::domain_error("complex division by zero");
        const double ratio = b.im / b.re;
        const double denom = b.re + b.im * ratio;
        return {(a.re + a.im * ratio) / denom, (a.im - a.re * ratio) / denom};
    }
    if (abs_bimag >= abs_breal) {
        const double ratio = b.re / b.im;
        const double denom = b.re * ratio + b.im;
        return {(a.re * ratio + a.im) / denom, (a.im * ratio - a.re) / denom};
    }
    return {std::nan(""), std::nan("")};
}

Cx np_mul(Cx a, Cx b) { return {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re}; }

Cx np_div(Cx a, Cx b) {
    // NumPy's complex divide loop
    const double in2r_abs = std::fabs(b.re);
    const double in2i_abs = std::fabs(b.im);
    if (in2r_abs >= in2i_abs) {
        if (in2r_abs == 0 && in2i_abs == 0) return {a.re / in2r_abs, a.im / in2r_abs};
        const double rat = b.im / b.re;
        const double scl = 1.0 / (b.re + b.im * rat);
        return {(a.re + a.im * rat) * scl, (a.im - a.re * rat) * scl};
    }
    const double rat = b.re / b.im;
    const double scl = 1.0 / (b.im + b.re * rat);
    return {(a.re * rat + a.im) * scl, (a.im * rat - a.re) * scl};
}

namespace {

constexpr double kPi = 3.141592653589793;
double radians(double x) { return x * (kPi / 180.0); }
double degrees(double x) { return x * (180.0 / kPi); }
double clip(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }
/// float + complex (Python: the float becomes x + 0j)
Cx f_plus_c(double x, Cx c) { return {x + c.re, 0.0 + c.im}; }
/// 1j * x for a float x (Python's complex multiply)
Cx j_times(double x) { return py_mul({0.0, 1.0}, {x, 0.0}); }

void parameterize(Arc& arc, bool autoscale_radius) {
    double rx = arc.radius.re;
    double ry = arc.radius.im;
    double rx_sqd = rx * rx;
    double ry_sqd = ry * ry;

    // z' = rot_matrix**(-1) * (start - end) / 2
    const Cx zp1 = np_div(np_mul(np_div({1.0, 0.0}, arc.rot_matrix), py_sub(arc.start, arc.end)), {2.0, 0.0});
    const double x1p = zp1.re, y1p = zp1.im;
    const double x1p_sqd = x1p * x1p;
    const double y1p_sqd = y1p * y1p;

    // Correct out of range radii
    const double radius_check = (x1p_sqd / rx_sqd) + (y1p_sqd / ry_sqd);
    if (radius_check > 1) {
        if (!autoscale_radius) throw std::invalid_argument("No such elliptic arc exists.");
        rx *= std::sqrt(radius_check);
        ry *= std::sqrt(radius_check);
        arc.radius = f_plus_c(rx, j_times(ry));
        rx_sqd = rx * rx;
        ry_sqd = ry * ry;
    }

    const double tmp = rx_sqd * y1p_sqd + ry_sqd * x1p_sqd;
    const double radicand = (rx_sqd * ry_sqd - tmp) / tmp;
    // np.isclose(radicand, 0) gives the integer 0, whose negation is still +0
    const bool radical_is_zero = std::fabs(radicand) <= 1e-08;
    const double radical = radical_is_zero ? 0.0 : std::sqrt(radicand);

    // rx*y1p/ry - 1j*ry*x1p/rx
    const Cx b = py_div(py_mul(j_times(ry), {x1p, 0.0}), {rx, 0.0});
    const Cx w{rx * y1p / ry - b.re, 0.0 - b.im};
    const double factor = radical_is_zero ? 0.0 : (arc.large_arc == arc.sweep ? -radical : radical);
    const Cx cp = np_mul({factor, 0.0}, w);

    arc.center = py_add(np_mul(arc.rot_matrix, cp), py_div(py_add(arc.start, arc.end), {2.0, 0.0}));

    const Cx u1i = py_div(j_times(y1p - cp.im), {ry, 0.0});
    const Cx u2i = py_div(j_times(-y1p - cp.im), {ry, 0.0});
    Cx u1 = f_plus_c((x1p - cp.re) / rx, u1i);
    Cx u2 = f_plus_c((-x1p - cp.re) / rx, u2i);
    u1 = {clip(u1.re, -1, 1), clip(u1.im, -1, 1)};
    u2 = {clip(u2.re, -1, 1), clip(u2.im, -1, 1)};

    if (u1.im > 0) {
        arc.theta = degrees(std::acos(u1.re));
    } else if (u1.im < 0) {
        arc.theta = -degrees(std::acos(u1.re));
    } else {
        arc.theta = u1.re > 0 ? 0.0 : 180.0;
    }

    const double det_uv = u1.re * u2.im - u1.im * u2.re;
    double acosand = u1.re * u2.re + u1.im * u2.im;
    acosand = clip(acosand, -1, 1) + clip(0.0, -1, 1);
    if (det_uv > 0) {
        arc.delta = degrees(std::acos(acosand));
    } else if (det_uv < 0) {
        arc.delta = -degrees(std::acos(acosand));
    } else {
        arc.delta = (u1.re * u2.re + u1.im * u2.im > 0) ? 0.0 : 180.0;
    }

    if (!arc.sweep && arc.delta >= 0) {
        arc.delta -= 360;
    } else if (arc.large_arc && arc.delta <= 0) {
        arc.delta += 360;
    }
}

}  // namespace

Arc make_arc(Cx start, Cx radius, double rotation, bool large_arc, bool sweep, Cx end, bool autoscale_radius) {
    if (start == end) throw std::runtime_error("svgpathtools.Arc: start and end must differ");
    if (radius.re == 0 || radius.im == 0) throw std::runtime_error("svgpathtools.Arc: radius can't be zero");
    Arc arc;
    arc.start = start;
    arc.radius = f_plus_c(std::fabs(radius.re), j_times(std::fabs(radius.im)));
    arc.rotation = rotation;
    arc.large_arc = large_arc;
    arc.sweep = sweep;
    arc.end = end;
    arc.phi = radians(rotation);
    const Cx z = j_times(arc.phi);
    arc.rot_matrix = {std::exp(z.re) * std::cos(z.im), std::exp(z.re) * std::sin(z.im)};
    parameterize(arc, autoscale_radius);
    return arc;
}

// ---------------------------------------------------------------------------
// Segments
// ---------------------------------------------------------------------------

Segment Segment::line(Cx start, Cx end) {
    Segment s;
    s.type = SegmentType::LINE;
    s.start = start;
    s.end = end;
    return s;
}

Segment Segment::quadratic(Cx start, Cx control, Cx end) {
    Segment s;
    s.type = SegmentType::QUADRATIC;
    s.start = start;
    s.c1 = control;
    s.end = end;
    return s;
}

Segment Segment::cubic(Cx start, Cx control1, Cx control2, Cx end) {
    Segment s;
    s.type = SegmentType::CUBIC;
    s.start = start;
    s.c1 = control1;
    s.c2 = control2;
    s.end = end;
    return s;
}

Segment Segment::make(const Arc& arc) {
    Segment s;
    s.type = SegmentType::ARC;
    s.start = arc.start;
    s.end = arc.end;
    s.arc = arc;
    return s;
}

const char* Segment::class_name() const {
    switch (type) {
        case SegmentType::LINE: return "Line";
        case SegmentType::QUADRATIC: return "QuadraticBezier";
        case SegmentType::CUBIC: return "CubicBezier";
        case SegmentType::ARC: return "Arc";
    }
    return "?";
}

std::string path_d(const Path& path) {
    if (path.empty()) return "";
    auto pt = [](Cx p) { return pyfmt::repr(p.re) + "," + pyfmt::repr(p.im); };
    std::string out;
    auto part = [&out](const std::string& s) {
        if (!out.empty()) out += ' ';
        out += s;
    };
    bool have_pos = false;
    Cx current_pos;
    for (const Segment& segment : path) {
        if (!have_pos || current_pos != segment.start) part("M " + pt(segment.start));
        switch (segment.type) {
            case SegmentType::LINE: part("L " + pt(segment.end)); break;
            case SegmentType::CUBIC: part("C " + pt(segment.c1) + " " + pt(segment.c2) + " " + pt(segment.end)); break;
            case SegmentType::QUADRATIC: part("Q " + pt(segment.c1) + " " + pt(segment.end)); break;
            case SegmentType::ARC:
                part("A " + pyfmt::repr(segment.arc.radius.re) + "," + pyfmt::repr(segment.arc.radius.im) + " " +
                     pyfmt::repr(segment.arc.rotation) + " " + (segment.arc.large_arc ? "1" : "0") + "," +
                     (segment.arc.sweep ? "1" : "0") + " " + pt(segment.end));
                break;
        }
        current_pos = segment.end;
        have_pos = true;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

namespace {

bool is_command(char c) { return std::string("MmZzLlHhVvCcSsQqTtAa").find(c) != std::string::npos; }

/// FLOAT_RE = [-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)? matched at `pos`; the match length or 0.
std::size_t match_float(const std::string& s, std::size_t pos) {
    std::size_t i = pos;
    auto digit = [&s](std::size_t k) { return k < s.size() && std::isdigit(static_cast<unsigned char>(s[k])); };
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    const std::size_t digits_start = i;
    while (digit(i)) ++i;
    const std::size_t d1 = i - digits_start;
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

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }

std::vector<std::string> tokenize_path(const std::string& pathdef) {
    // COMMAND_RE.split: the chunks between command letters, and the letters themselves
    std::vector<std::string> tokens;
    char command = 0;
    std::size_t i = 0;
    auto chunk_tokens = [&](const std::string& x) {
        if (command == 'A' || command == 'a') {
            // _tokenize_arc_args: the flags may be written without separators
            std::size_t pos = 0;
            int field = 0;
            while (true) {
                while (pos < x.size() && (is_space(x[pos]) || x[pos] == ',')) ++pos;
                if (pos >= x.size()) return;
                std::size_t len = 0;
                if (field % 7 == 3 || field % 7 == 4) {
                    if (x[pos] == '0' || x[pos] == '1') len = 1;
                }
                if (len == 0) len = match_float(x, pos);
                if (len == 0) return;
                tokens.push_back(x.substr(pos, len));
                pos += len;
                ++field;
            }
        } else {
            std::size_t pos = 0;
            while (pos < x.size()) {
                const std::size_t len = match_float(x, pos);
                if (len) {
                    tokens.push_back(x.substr(pos, len));
                    pos += len;
                } else {
                    ++pos;
                }
            }
        }
    };
    while (i <= pathdef.size()) {
        std::size_t j = i;
        while (j < pathdef.size() && !is_command(pathdef[j])) ++j;
        chunk_tokens(pathdef.substr(i, j - i));
        if (j >= pathdef.size()) break;
        command = pathdef[j];
        tokens.emplace_back(1, pathdef[j]);
        i = j + 1;
    }
    return tokens;
}

double to_float(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

/// float(x) + float(y) * 1j
Cx point(double x, double y) { return f_plus_c(x, py_mul({y, 0.0}, {0.0, 1.0})); }

}  // namespace

Path parse_path(const std::string& pathdef, Cx current_pos) {
    std::vector<std::string> elements = tokenize_path(pathdef);
    std::reverse(elements.begin(), elements.end());
    Path segments;
    bool have_start = false;
    Cx start_pos;
    char command = 0, last_command = 0;
    bool absolute = true;
    auto pop = [&elements]() {
        std::string v = elements.back();
        elements.pop_back();
        return to_float(v);
    };
    auto num_args = [](char c) {
        switch (c) {
            case 'M': case 'L': case 'T': return 2;
            case 'H': case 'V': return 1;
            case 'C': return 6;
            case 'S': case 'Q': return 4;
            case 'A': return 7;
            default: return 0;
        }
    };
    while (!elements.empty()) {
        const std::string& top = elements.back();
        if (top.size() == 1 && is_command(top[0])) {
            last_command = command;
            command = top[0];
            elements.pop_back();
            absolute = std::isupper(static_cast<unsigned char>(command)) != 0;
            command = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));
        } else {
            if (command == 0) throw std::invalid_argument("Unallowed implicit command in " + pathdef);
            last_command = command;
        }
        if (static_cast<int>(elements.size()) < num_args(command)) {
            throw std::invalid_argument(std::string("Invalid path string: command '") + command + "' expects " +
                                        std::to_string(num_args(command)) + " values");
        }
        switch (command) {
            case 'M': {
                const double x = pop(), y = pop();
                const Cx pos = point(x, y);
                current_pos = absolute ? pos : py_add(current_pos, pos);
                start_pos = current_pos;
                have_start = true;
                command = 'L';
                break;
            }
            case 'Z': {
                if (!have_start) throw std::invalid_argument("Close path before any moveto in " + pathdef);
                if (!(current_pos == start_pos)) segments.push_back(Segment::line(current_pos, start_pos));
                current_pos = start_pos;
                command = 0;
                break;
            }
            case 'L': {
                const double x = pop(), y = pop();
                Cx pos = point(x, y);
                if (!absolute) pos = py_add(pos, current_pos);
                segments.push_back(Segment::line(current_pos, pos));
                current_pos = pos;
                break;
            }
            case 'H': {
                const double x = pop();
                Cx pos = f_plus_c(x, j_times(current_pos.im));
                if (!absolute) pos = py_add(pos, {current_pos.re, 0.0});
                segments.push_back(Segment::line(current_pos, pos));
                current_pos = pos;
                break;
            }
            case 'V': {
                const double y = pop();
                Cx pos = f_plus_c(current_pos.re, py_mul({y, 0.0}, {0.0, 1.0}));
                if (!absolute) pos = py_add(pos, j_times(current_pos.im));
                segments.push_back(Segment::line(current_pos, pos));
                current_pos = pos;
                break;
            }
            case 'C': {
                const double a = pop(), b = pop(), c = pop(), d = pop(), e = pop(), f = pop();
                Cx control1 = point(a, b), control2 = point(c, d), end = point(e, f);
                if (!absolute) {
                    control1 = py_add(control1, current_pos);
                    control2 = py_add(control2, current_pos);
                    end = py_add(end, current_pos);
                }
                segments.push_back(Segment::cubic(current_pos, control1, control2, end));
                current_pos = end;
                break;
            }
            case 'S': {
                Cx control1 = current_pos;
                if (last_command == 'C' || last_command == 'S') {
                    control1 = py_sub(py_add(current_pos, current_pos), segments.back().c2);
                }
                const double c = pop(), d = pop(), e = pop(), f = pop();
                Cx control2 = point(c, d), end = point(e, f);
                if (!absolute) {
                    control2 = py_add(control2, current_pos);
                    end = py_add(end, current_pos);
                }
                segments.push_back(Segment::cubic(current_pos, control1, control2, end));
                current_pos = end;
                break;
            }
            case 'Q': {
                const double a = pop(), b = pop(), e = pop(), f = pop();
                Cx control = point(a, b), end = point(e, f);
                if (!absolute) {
                    control = py_add(control, current_pos);
                    end = py_add(end, current_pos);
                }
                segments.push_back(Segment::quadratic(current_pos, control, end));
                current_pos = end;
                break;
            }
            case 'T': {
                Cx control = current_pos;
                if (last_command == 'Q' || last_command == 'T') {
                    control = py_sub(py_add(current_pos, current_pos), segments.back().c1);
                }
                const double e = pop(), f = pop();
                Cx end = point(e, f);
                if (!absolute) end = py_add(end, current_pos);
                segments.push_back(Segment::quadratic(current_pos, control, end));
                current_pos = end;
                break;
            }
            case 'A': {
                const double rx = pop(), ry = pop();
                const Cx radius = point(rx, ry);
                const double rotation = pop();
                const double large = pop();
                const double sweep = pop();
                const double e = pop(), f = pop();
                Cx end = point(e, f);
                if (!absolute) end = py_add(end, current_pos);
                if (radius.re == 0 || radius.im == 0) {
                    warn("Replacing degenerate (zero radius) Arc with a Line");
                    segments.push_back(Segment::line(current_pos, end));
                } else {
                    segments.push_back(Segment::make(make_arc(current_pos, radius, rotation, large != 0, sweep != 0, end)));
                }
                current_pos = end;
                break;
            }
            default: throw std::invalid_argument("Invalid path command in " + pathdef);
        }
    }
    return segments;
}

}  // namespace build123d::svgpath
