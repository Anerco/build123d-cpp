// build123d-cpp: algebra-mode objects. Mirrors objects_curve.py, objects_sketch.py and
// objects_part.py. Curve objects are Edges or Wires, sketch objects are Sketches and part
// objects are Parts, so they combine with the algebra operators directly.
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

// ---------------------------------------------------------------------------
// 1D objects
// ---------------------------------------------------------------------------

/// Straight line between two points.
class Line : public Edge {
public:
    Line(const Vector& start, const Vector& end);
    explicit Line(const std::vector<Vector>& pts);
};

/// Connected straight lines through the points.
class Polyline : public Wire {
public:
    explicit Polyline(const std::vector<Vector>& pts, bool close = false);
};

/// Interpolating spline, optionally with start/end (or per point) tangents.
class Spline : public Edge {
public:
    explicit Spline(const std::vector<Vector>& pts, const std::vector<Vector>& tangents = {},
                    const std::vector<double>& tangent_scalars = {}, bool periodic = false);
};

/// Bezier curve through control points, optionally rational with weights.
class Bezier : public Edge {
public:
    explicit Bezier(const std::vector<Vector>& cntl_pnts, const std::vector<double>& weights = {});
};

/// Exact B-spline from control points, knots (repeat a value for multiplicity), a degree
/// and optional weights. Unlike Spline it does not interpolate the points.
class BSpline : public Edge {
public:
    BSpline(const std::vector<Vector>& control_points, const std::vector<double>& knots, int degree,
            const std::vector<double>& weights = {}, bool periodic = false);
};

/// Circular arc from a center, radius, start angle and (signed) arc size in degrees.
class CenterArc : public Edge {
public:
    CenterArc(const Vector& center, double radius, double start_angle, double arc_size);
};

/// Circular arc through three points.
class ThreePointArc : public Edge {
public:
    ThreePointArc(const Vector& point1, const Vector& point2, const Vector& point3);
    explicit ThreePointArc(const std::vector<Vector>& pts);
};

/// Circular arc between two points with the given radius (negative flips the side).
class RadiusArc : public Edge {
public:
    RadiusArc(const Vector& start_point, const Vector& end_point, double radius,
              bool short_sagitta = true);
};

/// Circular arc between two points with the given sagitta (arc height).
class SagittaArc : public Edge {
public:
    SagittaArc(const Vector& start_point, const Vector& end_point, double sagitta);
};

/// Circular arc between two points, tangent to `tangent` at the first (or last) point.
class TangentArc : public Edge {
public:
    TangentArc(const Vector& start, const Vector& end, const Vector& tangent,
               bool tangent_from_first = true);
    TangentArc(const std::vector<Vector>& pts, const Vector& tangent,
               bool tangent_from_first = true);
};

/// Circular arc from a start point and tangent, with a radius and (signed) arc size.
class JernArc : public Edge {
public:
    JernArc(const Vector& start, const Vector& tangent, double radius, double arc_size);
    Vector start;
    Vector end_of_arc;
    Vector center_point;
};

/// Elliptical arc around a center.
class EllipticalCenterArc : public Edge {
public:
    EllipticalCenterArc(const Vector& center, double x_radius, double y_radius,
                        double start_angle = 0.0, double arc_size = 90.0, double rotation = 0.0);
};

/// Elliptical arc from a start point and tangent; give either the start angle on the
/// ellipse (degrees) or the direction of its major axis.
class EllipticalStartArc : public Edge {
public:
    EllipticalStartArc(const Vector& start_pnt, const Vector& start_tangent, double x_radius, double y_radius,
                       double arc_size, std::optional<double> start_angle = std::nullopt,
                       std::optional<Vector> major_axis_dir = std::nullopt);
};

/// Parabolic arc from its vertex; angles are curve parameters in degrees.
class ParabolicCenterArc : public Edge {
public:
    ParabolicCenterArc(const Vector& vertex, double focal_length, double start_angle = 0.0,
                       double arc_size = 90.0, double rotation = 0.0);
};

/// Hyperbolic arc around a center.
class HyperbolicCenterArc : public Edge {
public:
    HyperbolicCenterArc(const Vector& center, double x_radius, double y_radius, double start_angle = 0.0,
                        double arc_size = 90.0, double rotation = 0.0);
};

/// Helix (conical if cone_angle != 0).
class Helix : public Edge {
public:
    Helix(double pitch, double height, double radius, const Vector& center = Vector(0, 0, 0),
          const Vector& direction = Vector(0, 0, 1), double cone_angle = 0.0,
          bool lefthand = false);
};

/// Straight line from a start point, a length and an angle (degrees) or direction.
class PolarLine : public Edge {
public:
    PolarLine(const Vector& start, double length, double angle,
              LengthMode length_mode = LengthMode::DIAGONAL);
    PolarLine(const Vector& start, double length, const Vector& direction,
              LengthMode length_mode = LengthMode::DIAGONAL);
};

/// Polyline with filleted corners (radius 0 keeps a sharp corner).
class FilletPolyline : public Wire {
public:
    FilletPolyline(const std::vector<Vector>& pts, double radius, bool close = false);
    FilletPolyline(const std::vector<Vector>& pts, const std::vector<double>& radius,
                   bool close = false);
};

/// Line from a point along a direction until it meets another curve.
class IntersectingLine : public Edge {
public:
    IntersectingLine(const Vector& start, const Vector& direction, const Shape& other);
};

// ---------------------------------------------------------------------------
// 2D objects
// ---------------------------------------------------------------------------

/// Circle (a sector if arc_size < 360).
class Circle : public Sketch {
public:
    explicit Circle(double radius, double arc_size = 360.0, Align2 align = Align::CENTER);
    Circle(double radius, Align2 align);
    double radius;
    double arc_size;
};

class Ellipse : public Sketch {
public:
    Ellipse(double x_radius, double y_radius, double rotation = 0.0, Align2 align = Align::CENTER);
    Ellipse(double x_radius, double y_radius, Align2 align);
    double x_radius;
    double y_radius;
};

class Rectangle : public Sketch {
public:
    Rectangle(double width, double height, double rotation = 0.0, Align2 align = Align::CENTER);
    Rectangle(double width, double height, Align2 align);
    double width;
    double rectangle_height;
};

class RectangleRounded : public Sketch {
public:
    RectangleRounded(double width, double height, double radius, double rotation = 0.0,
                     Align2 align = Align::CENTER);
    RectangleRounded(double width, double height, double radius, Align2 align);
    double width;
    double rectangle_height;
    double radius;
};

/// Polygon through the points (counter-clockwise points give an upward normal).
class Polygon : public Sketch {
public:
    explicit Polygon(const std::vector<Vector>& pts, double rotation = 0.0,
                     Align2 align = Align::NONE);
    Polygon(const std::vector<Vector>& pts, Align2 align);
};

class RegularPolygon : public Sketch {
public:
    RegularPolygon(double radius, int side_count, bool major_radius = true, double rotation = 0.0,
                   Align2 align = Align::CENTER);
    RegularPolygon(double radius, int side_count, Align2 align);
    double radius;   ///< circumscribed radius
    double apothem;  ///< inscribed radius
    int side_count;
};

class Trapezoid : public Sketch {
public:
    Trapezoid(double width, double height, double left_side_angle,
              std::optional<double> right_side_angle = std::nullopt, double rotation = 0.0,
              Align2 align = Align::CENTER);
    Trapezoid(double width, double height, double left_side_angle, Align2 align);
};

/// Any three of the sides a, b, c and angles A, B, C (one must be a side).
struct TriangleParams {
    std::optional<double> a;
    std::optional<double> b;
    std::optional<double> c;
    std::optional<double> A;
    std::optional<double> B;
    std::optional<double> C;
};

/// Triangle from sides and/or angles: Triangle({.a = 3, .b = 4, .c = 5}).
class Triangle : public Sketch {
public:
    explicit Triangle(const TriangleParams& params, Align2 align = Align::NONE,
                      double rotation = 0.0);
    double a, b, c;  ///< side lengths
    double A, B, C;  ///< interior angles in degrees
    Edge edge_a, edge_b, edge_c;
    Vertex vertex_A, vertex_B, vertex_C;
};

/// Slot following an arc (edge or wire); height is the slot width.
class SlotArc : public Sketch {
public:
    SlotArc(const Shape& arc, double height, double rotation = 0.0);
};

class SlotCenterPoint : public Sketch {
public:
    SlotCenterPoint(const Vector& center, const Vector& point, double height,
                    double rotation = 0.0);
};

class SlotCenterToCenter : public Sketch {
public:
    SlotCenterToCenter(double center_separation, double height, double rotation = 0.0);
};

class SlotOverall : public Sketch {
public:
    SlotOverall(double width, double height, double rotation = 0.0, Align2 align = Align::CENTER);
    SlotOverall(double width, double height, Align2 align);
};

class Superellipse : public Sketch {
public:
    Superellipse(double width, double height, double order = 4.0, int point_count = 16,
                 double rotation = 0.0, Align2 align = Align::CENTER);
};

/// Options for Text, matching build123d's keyword arguments.
struct TextOptions {
    std::string font = "Arial";
    std::string font_path;
    FontStyle font_style = FontStyle::REGULAR;
    std::pair<TextAlign, TextAlign> text_align = {TextAlign::CENTER, TextAlign::CENTER};
    std::optional<Align2> align;
    std::optional<Wire> path;
    double position_on_path = 0.0;
    double rotation = 0.0;
};

/// Text as faces: Text("Hello", 10, {.font_style = FontStyle::BOLD}).
class Text : public Sketch {
public:
    Text(const std::string& txt, double font_size, const TextOptions& options = {});
    std::string txt;
    double font_size;
};

/// Font family names known to the font manager.
std::vector<std::string> available_fonts();

// ---------------------------------------------------------------------------
// 3D objects
// ---------------------------------------------------------------------------

class Box : public Part {
public:
    Box(double length, double width, double height, const Rotation& rotation = Rotation(),
        Align3 align = Align::CENTER);
    Box(double length, double width, double height, Align3 align);
};

class Cylinder : public Part {
public:
    Cylinder(double radius, double height, double arc_size = 360.0,
             const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Cylinder(double radius, double height, Align3 align);
    Cylinder(double radius, double height, const Rotation& rotation, Align3 align = Align::CENTER);
};

class Cone : public Part {
public:
    Cone(double bottom_radius, double top_radius, double height, double arc_size = 360.0,
         const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Cone(double bottom_radius, double top_radius, double height, Align3 align);
    Cone(double bottom_radius, double top_radius, double height, const Rotation& rotation,
         Align3 align = Align::CENTER);
};

class Sphere : public Part {
public:
    explicit Sphere(double radius, double arc_size1 = -90.0, double arc_size2 = 90.0,
                    double arc_size3 = 360.0, const Rotation& rotation = Rotation(),
                    Align3 align = Align::CENTER);
    Sphere(double radius, Align3 align);
    Sphere(double radius, const Rotation& rotation, Align3 align = Align::CENTER);
};

class Torus : public Part {
public:
    Torus(double major_radius, double minor_radius, double minor_start_angle = 0.0,
          double minor_end_angle = 360.0, double major_angle = 360.0,
          const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Torus(double major_radius, double minor_radius, Align3 align);
    Torus(double major_radius, double minor_radius, const Rotation& rotation,
          Align3 align = Align::CENTER);
};

class Wedge : public Part {
public:
    Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax,
          double zmax, const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax,
          double zmax, Align3 align);
};

/// Hole tool: subtract it, e.g. part - Pos(x, y, top) * Hole(2, 10).
class Hole : public Part {
public:
    Hole(double radius, double depth);
};

/// Counter bore hole tool, top at the origin, going down -Z.
class CounterBoreHole : public Part {
public:
    CounterBoreHole(double radius, double counter_bore_radius, double counter_bore_depth,
                    double depth);
};

/// Counter sink hole tool, top at the origin, going down -Z.
class CounterSinkHole : public Part {
public:
    CounterSinkHole(double radius, double counter_sink_radius, double depth,
                    double counter_sink_angle = 82.0);
};

/// Convex solid enclosing the points.
class ConvexPolyhedron : public Part {
public:
    explicit ConvexPolyhedron(const std::vector<Vector>& points,
                              const Rotation& rotation = Rotation(), Align3 align = Align::NONE);
};

}  // namespace build123d
