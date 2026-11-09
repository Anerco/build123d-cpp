// The parts of svgelements (svgelements.py, version 1.9.6) that ocpsvg uses to read SVG
// documents: SVG.parse with its style cascade, units, transforms and viewBox, the basic
// shapes, paths and arcs, reified as ocpsvg asks, and ocpsvg's walk over the parsed document
// (find_shapes_svg_in_document).
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace build123d::svgdoc {

struct Point {
    double x = 0.0;
    double y = 0.0;
};
/// svgelements' Point equality: both coordinates within 1e-12.
bool same_point(const Point& a, const Point& b);

/// svgelements.Matrix: x' = a x + c y + e, y' = b x + d y + f.
struct Matrix {
    double a = 1, b = 0, c = 0, d = 1, e = 0, f = 0;
    /// Matrix.point_in_matrix_space
    Point apply(const Point& p) const;
    bool is_identity() const { return a == 1 && b == 0 && c == 0 && d == 1 && e == 0 && f == 0; }
    double determinant() const { return a * d - c * b; }
    /// Matrix.matrix_multiply(m, s): first m, then s.
    static Matrix multiply(const Matrix& m, const Matrix& s);
};

enum class SegKind { MOVE, LINE, CLOSE, QUAD, CUBIC, ARC };

/// A path segment. Arcs are in svgelements' native form: the center, the points prx and pry
/// at the ends of the two radii, and the sweep in radians.
struct Segment {
    SegKind kind = SegKind::LINE;
    std::optional<Point> start;
    std::optional<Point> end;
    std::optional<Point> control;   // QUAD
    std::optional<Point> control1;  // CUBIC
    std::optional<Point> control2;  // CUBIC
    Point center;                   // ARC
    Point prx;       // ARC
    Point pry;       // ARC
    double sweep = 0.0;

    /// segment *= matrix
    void transform(const Matrix& m);
    // Arc properties
    double rx() const;
    double ry() const;
    /// Arc.get_rotation(): the angle from the center to prx (radians).
    double rotation() const;
    /// Arc.theta: the start parameter in positive degrees.
    double theta() const;
    /// Arc.delta: the sweep in degrees.
    double delta() const;
};

/// svgelements.Color: an RGBA integer, or no value for "none".
struct Color {
    std::optional<std::int64_t> value;
    int red() const { return static_cast<int>((*value >> 24) & 0xFF); }
    int green() const { return static_cast<int>((*value >> 16) & 0xFF); }
    int blue() const { return static_cast<int>((*value >> 8) & 0xFF); }
    int alpha() const { return static_cast<int>(*value & 0xFF); }
};

enum class ShapeKind { PATH, RECT, CIRCLE, ELLIPSE, LINE, POLYLINE, POLYGON };

/// A shape of the document, as ocpsvg uses it.
struct ShapeElement {
    ShapeKind kind = ShapeKind::PATH;
    /// The element's values: its attributes and style, with the inherited ones.
    std::map<std::string, std::string> values;
    Color fill;
    Color stroke;
    /// The transform left after reification.
    Matrix transform;
    /// Circle and ellipse geometry after reification.
    double cx = 0, cy = 0, rx = 0, ry = 0;
    /// svgelements.Path(element).reify(): the shape's segments with the remaining transform
    /// applied (not used for circles and ellipses).
    std::vector<Segment> path;
};

struct Document {
    /// The visible shapes, in document order, that ocpsvg imports.
    std::vector<ShapeElement> shapes;
    /// The values of the parsed <svg> element.
    std::map<std::string, std::string> values;
    /// float(svg.width), float(svg.height)
    double width = 0.0;
    double height = 0.0;
    bool has_viewbox = false;
    double vb_x = 0, vb_y = 0, vb_width = 0, vb_height = 0;
};

/// svgelements.SVG.parse(text, parse_display_none=ignore_visibility, ppi=ppi) and ocpsvg's
/// walk over its result. Raises std::invalid_argument for documents svgelements or ocpsvg
/// can't read.
Document parse(const std::string& text, bool ignore_visibility, double ppi);

}  // namespace build123d::svgdoc
