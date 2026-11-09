// The parts of svgpathtools (path.py, parser.py) that build123d uses: path segments with
// svgpathtools' arc parameterization, Path.d() and path-data parsing. svgpathtools mixes
// Python and NumPy complex arithmetic; both are reproduced so results match bit for bit.
#pragma once

#include <string>
#include <vector>

namespace build123d::svgpath {

/// A complex number (a point x + iy).
struct Cx {
    double re = 0.0;
    double im = 0.0;
    bool operator==(const Cx& o) const { return re == o.re && im == o.im; }
    bool operator!=(const Cx& o) const { return !(*this == o); }
};

// CPython's complex arithmetic (complexobject.c)
Cx py_add(Cx a, Cx b);
Cx py_sub(Cx a, Cx b);
Cx py_mul(Cx a, Cx b);
Cx py_div(Cx a, Cx b);
// NumPy's complex128 arithmetic
Cx np_mul(Cx a, Cx b);
Cx np_div(Cx a, Cx b);

/// svgpathtools.Arc: an elliptical arc in SVG endpoint parameterization, with the derived
/// center, theta and delta (degrees).
struct Arc {
    Cx start;
    Cx radius;
    double rotation = 0.0;
    bool large_arc = false;
    bool sweep = false;
    Cx end;
    // derived
    double phi = 0.0;
    Cx rot_matrix;
    Cx center;
    double theta = 0.0;
    double delta = 0.0;
};
/// Arc(start, radius, rotation, large_arc, sweep, end, autoscale_radius): raises
/// std::runtime_error if start == end or a radius is 0 (Python's AssertionError) and
/// std::invalid_argument if no such arc exists and autoscale_radius is false.
Arc make_arc(Cx start, Cx radius, double rotation, bool large_arc, bool sweep, Cx end, bool autoscale_radius = true);

enum class SegmentType { LINE, QUADRATIC, CUBIC, ARC };

/// One path segment: Line(start, end), QuadraticBezier(start, control=c1, end),
/// CubicBezier(start, c1, c2, end) or an Arc.
struct Segment {
    SegmentType type = SegmentType::LINE;
    Cx start;
    Cx c1;
    Cx c2;
    Cx end;
    Arc arc;

    static Segment line(Cx start, Cx end);
    static Segment quadratic(Cx start, Cx control, Cx end);
    static Segment cubic(Cx start, Cx control1, Cx control2, Cx end);
    static Segment make(const Arc& arc);
    /// svgpathtools' class name ("Line", "QuadraticBezier", "CubicBezier", "Arc").
    const char* class_name() const;
};

using Path = std::vector<Segment>;

/// Path.d(): "M x,y L x,y C ... Q ... A rx,ry rot large,sweep x,y", new subpaths where a
/// segment doesn't start at the previous one's end. Numbers as Python's str(float).
std::string path_d(const Path& path);

/// svgpathtools.parser.parse_path (and Path.__init__ from a string): the segments of SVG
/// path data, with `current_pos` as the initial point.
Path parse_path(const std::string& pathdef, Cx current_pos = Cx{});

}  // namespace build123d::svgpath
