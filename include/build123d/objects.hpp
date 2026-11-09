// build123d-cpp: algebra-mode objects. Mirrors objects_curve.py, objects_sketch.py and
// objects_part.py. Curve objects are Edges or Wires, sketch objects are Sketches and part
// objects are Parts, so they combine with the algebra operators directly.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

// ---------------------------------------------------------------------------
// Base classes for objects (Python's Base*Object): derive from them to write
// custom objects, as in build123d.
// ---------------------------------------------------------------------------

/// Base of curve objects that are Curves (Python's BaseCurveObject).
class BaseCurveObject : public Curve {
public:
    BaseCurveObject() = default;
    explicit BaseCurveObject(const Curve& curve);
};

/// Base of curve objects that are Wires (Python's BaseLineObject).
class BaseLineObject : public Wire {
public:
    BaseLineObject() = default;
    explicit BaseLineObject(const Wire& curve);
};

/// Base of curve objects that are Edges (Python's BaseEdgeObject).
class BaseEdgeObject : public Edge {
public:
    BaseEdgeObject() = default;
    explicit BaseEdgeObject(const Edge& curve);
};

/// Base of sketch objects (Python's BaseSketchObject): moves `obj` so that its bounding box
/// has the given alignment, rotates it by `rotation` degrees about Z and keeps its faces.
class BaseSketchObject : public Sketch {
public:
    explicit BaseSketchObject(const Shape& obj, double rotation = 0.0,
                              std::optional<Align2> align = std::nullopt);
    double rotation = 0.0;

protected:
    BaseSketchObject() = default;
    /// The constructor's work, for objects that compute their shape before building it.
    void assign(const Shape& obj, double rotation, std::optional<Align2> align);
};

/// Base of part objects (Python's BasePartObject): moves `part` so that its bounding box has
/// the given alignment, then rotates it.
class BasePartObject : public Part {
public:
    explicit BasePartObject(const Shape& part, const Rotation& rotation = Rotation(),
                            std::optional<Align3> align = std::nullopt);
    Rotation rotation;

protected:
    BasePartObject() = default;
    void assign(const Shape& part, const Rotation& rotation, std::optional<Align3> align);
};

// ---------------------------------------------------------------------------
// 1D objects
// ---------------------------------------------------------------------------

/// What an arc's arc_size or PolarLine's length can be instead of a number: a Shape, Axis,
/// Location, Plane or point. The curve is trimmed where it first meets it
/// (Edge::trim_to_other).
class Limit {
public:
    Limit(const Shape& shape);        // NOLINT(google-explicit-constructor)
    Limit(const Axis& axis);          // NOLINT
    Limit(const Location& location);  // NOLINT
    Limit(const Plane& plane);        // NOLINT
    Limit(const Vector& point);       // NOLINT
    Limit(double x, double y, double z = 0.0);
    /// edge.trim_to_other(the limit)
    std::optional<Edge> trim(const Edge& edge) const;
    std::string to_string() const;

    std::variant<Shape, Axis, Location, Plane, Vector> value;
};

/// Straight line between two points.
class Line : public BaseEdgeObject {
public:
    Line(const Vector& start, const Vector& end);
    explicit Line(const std::vector<Vector>& pts);
};

/// Connected straight lines through the points.
class Polyline : public BaseLineObject {
public:
    explicit Polyline(const std::vector<Vector>& pts, bool close = false);
};

/// Interpolating spline, optionally with start/end (or per point) tangents.
class Spline : public BaseEdgeObject {
public:
    explicit Spline(const std::vector<Vector>& pts, const std::vector<Vector>& tangents = {},
                    const std::vector<double>& tangent_scalars = {}, bool periodic = false);
};

/// Bezier curve through control points, optionally rational with weights.
class Bezier : public BaseEdgeObject {
public:
    explicit Bezier(const std::vector<Vector>& cntl_pnts, const std::vector<double>& weights = {});
};

/// Exact B-spline from control points, knots (repeat a value for multiplicity), a degree
/// and optional weights. Unlike Spline it does not interpolate the points.
class BSpline : public BaseEdgeObject {
public:
    BSpline(const std::vector<Vector>& control_points, const std::vector<double>& knots, int degree,
            const std::vector<double>& weights = {}, bool periodic = false);
};

/// Circular arc from a center, radius, start angle and (signed) arc size in degrees, or an
/// arc limit: the arc then runs from the start to the limit, the shorter way round.
class CenterArc : public BaseEdgeObject {
public:
    CenterArc(const Vector& center, double radius, double start_angle, double arc_size);
    CenterArc(const Vector& center, double radius, double start_angle, const Limit& arc_size);
};

/// Circular arc through three points.
class ThreePointArc : public BaseEdgeObject {
public:
    ThreePointArc(const Vector& point1, const Vector& point2, const Vector& point3);
    explicit ThreePointArc(const std::vector<Vector>& pts);
};

/// Circular arc between two points with the given radius (negative flips the side).
class RadiusArc : public BaseEdgeObject {
public:
    RadiusArc(const Vector& start_point, const Vector& end_point, double radius,
              bool short_sagitta = true);
};

/// Circular arc between two points with the given sagitta (arc height).
class SagittaArc : public BaseEdgeObject {
public:
    SagittaArc(const Vector& start_point, const Vector& end_point, double sagitta);
};

/// Circular arc between two points, tangent to `tangent` at the first (or last) point.
class TangentArc : public BaseEdgeObject {
public:
    TangentArc(const Vector& start, const Vector& end, const Vector& tangent,
               bool tangent_from_first = true);
    TangentArc(const std::vector<Vector>& pts, const Vector& tangent,
               bool tangent_from_first = true);
};

/// Circular arc from a start point and tangent, with a radius and (signed) arc size or an
/// arc limit.
class JernArc : public BaseEdgeObject {
public:
    JernArc(const Vector& start, const Vector& tangent, double radius, double arc_size);
    JernArc(const Vector& start, const Vector& tangent, double radius, const Limit& arc_size);
    Vector start;
    Vector end_of_arc;
    Vector center_point;
};

/// Elliptical arc around a center.
class EllipticalCenterArc : public BaseEdgeObject {
public:
    EllipticalCenterArc(const Vector& center, double x_radius, double y_radius,
                        double start_angle = 0.0, double arc_size = 90.0, double rotation = 0.0);
    EllipticalCenterArc(const Vector& center, double x_radius, double y_radius, double start_angle,
                        const Limit& arc_size, double rotation = 0.0);
};

/// Elliptical arc from a start point and tangent; give either the start angle on the
/// ellipse (degrees) or the direction of its major axis.
class EllipticalStartArc : public BaseEdgeObject {
public:
    EllipticalStartArc(const Vector& start_pnt, const Vector& start_tangent, double x_radius, double y_radius,
                       double arc_size, std::optional<double> start_angle = std::nullopt,
                       std::optional<Vector> major_axis_dir = std::nullopt);
};

/// Parabolic arc from its vertex; angles are curve parameters in degrees.
class ParabolicCenterArc : public BaseEdgeObject {
public:
    ParabolicCenterArc(const Vector& vertex, double focal_length, double start_angle = 0.0,
                       double arc_size = 90.0, double rotation = 0.0);
    ParabolicCenterArc(const Vector& vertex, double focal_length, double start_angle,
                       const Limit& arc_size, double rotation = 0.0);
};

/// Hyperbolic arc around a center.
class HyperbolicCenterArc : public BaseEdgeObject {
public:
    HyperbolicCenterArc(const Vector& center, double x_radius, double y_radius, double start_angle = 0.0,
                        double arc_size = 90.0, double rotation = 0.0);
    HyperbolicCenterArc(const Vector& center, double x_radius, double y_radius, double start_angle,
                        const Limit& arc_size, double rotation = 0.0);
};

/// Helix (conical if cone_angle != 0).
class Helix : public BaseEdgeObject {
public:
    Helix(double pitch, double height, double radius, const Vector& center = Vector(0, 0, 0),
          const Vector& direction = Vector(0, 0, 1), double cone_angle = 0.0,
          bool lefthand = false);
};

/// Straight line from a start point, a length and an angle (degrees) or direction. The
/// length can be a limit instead: the line then runs until it meets it.
class PolarLine : public BaseEdgeObject {
public:
    PolarLine(const Vector& start, double length, double angle,
              LengthMode length_mode = LengthMode::DIAGONAL);
    PolarLine(const Vector& start, double length, const Vector& direction,
              LengthMode length_mode = LengthMode::DIAGONAL);
    PolarLine(const Vector& start, const Limit& length, double angle,
              LengthMode length_mode = LengthMode::DIAGONAL);
    PolarLine(const Vector& start, const Limit& length, const Vector& direction,
              LengthMode length_mode = LengthMode::DIAGONAL);
};

/// Polyline with filleted corners (radius 0 keeps a sharp corner).
class FilletPolyline : public BaseLineObject {
public:
    FilletPolyline(const std::vector<Vector>& pts, double radius, bool close = false);
    FilletPolyline(const std::vector<Vector>& pts, const std::vector<double>& radius,
                   bool close = false);
};

/// Line from a point along a direction until it meets another curve.
class IntersectingLine : public BaseEdgeObject {
public:
    IntersectingLine(const Vector& start, const Vector& direction, const Shape& other);
};

/// Smooth Bezier transition between two edges: a line (C0), a cubic (C1) or a quintic
/// (C2) matching the end positions, derivatives and curvatures. `end_points` picks the
/// ends to join (default: the closest pair); `tangent_scalars` scale the end derivatives.
class BlendCurve : public BaseEdgeObject {
public:
    BlendCurve(const Edge& curve0, const Edge& curve1, ContinuityLevel continuity = ContinuityLevel::C2,
               std::optional<std::pair<Vector, Vector>> end_points = std::nullopt,
               std::optional<std::pair<double, double>> tangent_scalars = std::nullopt);
};

/// Arc from a point and tangent that is also tangent to another curve (an Edge or a Wire);
/// keep picks Keep::TOP or Keep::BOTTOM when there are two. Contains a solver.
class DoubleTangentArc : public BaseEdgeObject {
public:
    DoubleTangentArc(const Vector& pnt, const Vector& tangent, const Shape& other, Keep keep = Keep::TOP);
};

/// NACA 4-digit (or fractional, e.g. "2213.323") airfoil profile of unit chord.
class Airfoil : public BaseLineObject {
public:
    explicit Airfoil(const std::string& airfoil_code, int n_points = 50, bool finite_te = false);
    /// The code as a number, like Python's Airfoil(2412).
    explicit Airfoil(double airfoil_code, int n_points = 50, bool finite_te = false);
    /// (max camber, its position, thickness) as fractions of the chord.
    static std::tuple<double, double, double> parse_naca4(const std::string& value);
    static std::tuple<double, double, double> parse_naca4(double value);
    /// The camber line of the airfoil.
    Edge camber_line() const;

    std::string code;   ///< NACA code string (e.g. "2412")
    double max_camber;  ///< maximum camber as fraction of chord
    double camber_pos;  ///< chordwise position of max camber (0-1)
    double thickness;   ///< maximum thickness as fraction of chord
    bool finite_te;     ///< if true, the trailing edge is finite

private:
    std::vector<Vector> camber_points_;
};

/// What a ConstrainedArcs/ConstrainedLines selector returns: one Edge or a list of them.
class EdgeSelection : public ShapeList<Edge> {
public:
    EdgeSelection(const Edge& edge) : ShapeList<Edge>{edge} {}                    // NOLINT
    EdgeSelection(const ShapeList<Edge>& edges) : ShapeList<Edge>(edges) {}       // NOLINT
    EdgeSelection(const std::vector<Edge>& edges) : ShapeList<Edge>(edges) {}     // NOLINT
};
/// Chooses some of the solutions, e.g. [](const ShapeList<Edge>& arcs) { return arcs[0]; }.
using EdgeSelector = std::function<EdgeSelection(const ShapeList<Edge>&)>;

/// Arcs constrained by other objects (see Edge::make_constrained_arcs), filtered by an
/// optional selector. Always a Curve.
class ConstrainedArcs : public BaseCurveObject {
public:
    ConstrainedArcs(const TangencyTarget& tangency_one, const ConstrainedArcOptions& options,
                    const EdgeSelector& selector = {});
    ConstrainedArcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                    const ConstrainedArcOptions& options, const EdgeSelector& selector = {});
    ConstrainedArcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                    const TangencyTarget& tangency_three, const ConstrainedArcOptions& options = {},
                    const EdgeSelector& selector = {});
};

/// Lines constrained by other objects (see Edge::make_constrained_lines), filtered by an
/// optional selector. Always a Curve.
class ConstrainedLines : public BaseCurveObject {
public:
    ConstrainedLines(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                     const ConstrainedLineOptions& options = {}, const EdgeSelector& selector = {});
};

// ---------------------------------------------------------------------------
// 2D objects
// ---------------------------------------------------------------------------

/// Circle (a sector if arc_size < 360).
class Circle : public BaseSketchObject {
public:
    explicit Circle(double radius, double arc_size = 360.0, Align2 align = Align::CENTER);
    Circle(double radius, Align2 align);
    double radius;
    double arc_size;
    Align2 align;
};

class Ellipse : public BaseSketchObject {
public:
    Ellipse(double x_radius, double y_radius, double rotation = 0.0, Align2 align = Align::CENTER);
    Ellipse(double x_radius, double y_radius, Align2 align);
    double x_radius;
    double y_radius;
    Align2 align;
};

class Rectangle : public BaseSketchObject {
public:
    Rectangle(double width, double height, double rotation = 0.0, Align2 align = Align::CENTER);
    Rectangle(double width, double height, Align2 align);
    double width;
    double rectangle_height;
    Align2 align;
};

class RectangleRounded : public BaseSketchObject {
public:
    RectangleRounded(double width, double height, double radius, double rotation = 0.0,
                     Align2 align = Align::CENTER);
    RectangleRounded(double width, double height, double radius, Align2 align);
    double width;
    double rectangle_height;
    double radius;
    Align2 align;
};

/// Polygon through the points (counter-clockwise points give an upward normal).
class Polygon : public BaseSketchObject {
public:
    explicit Polygon(const std::vector<Vector>& pts, double rotation = 0.0,
                     Align2 align = Align::NONE);
    Polygon(const std::vector<Vector>& pts, Align2 align);
    std::vector<Vector> pts;
    Align2 align;
};

class RegularPolygon : public BaseSketchObject {
public:
    RegularPolygon(double radius, int side_count, bool major_radius = true, double rotation = 0.0,
                   Align2 align = Align::CENTER);
    RegularPolygon(double radius, int side_count, Align2 align);
    double radius;   ///< circumscribed radius
    double apothem;  ///< inscribed radius
    int side_count;
    Align2 align;
};

class Trapezoid : public BaseSketchObject {
public:
    Trapezoid(double width, double height, double left_side_angle,
              std::optional<double> right_side_angle = std::nullopt, double rotation = 0.0,
              Align2 align = Align::CENTER);
    Trapezoid(double width, double height, double left_side_angle, Align2 align);
    double width;
    double trapezoid_height;
    double left_side_angle;
    double right_side_angle;
    Align2 align;
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
class Triangle : public BaseSketchObject {
public:
    explicit Triangle(const TriangleParams& params, Align2 align = Align::NONE,
                      double rotation = 0.0);
    double a, b, c;  ///< side lengths
    double A, B, C;  ///< interior angles in degrees
    Edge edge_a, edge_b, edge_c;
    Vertex vertex_A, vertex_B, vertex_C;
};

/// Slot following an arc (edge or wire); height is the slot width.
class SlotArc : public BaseSketchObject {
public:
    SlotArc(const Shape& arc, double height, double rotation = 0.0);
    Shape arc;
    double slot_height;
};

class SlotCenterPoint : public BaseSketchObject {
public:
    SlotCenterPoint(const Vector& center, const Vector& point, double height,
                    double rotation = 0.0);
    Vector slot_center;
    Vector point;
    double slot_height;
};

class SlotCenterToCenter : public BaseSketchObject {
public:
    SlotCenterToCenter(double center_separation, double height, double rotation = 0.0);
    double center_separation;
    double slot_height;
};

class SlotOverall : public BaseSketchObject {
public:
    SlotOverall(double width, double height, double rotation = 0.0, Align2 align = Align::CENTER);
    SlotOverall(double width, double height, Align2 align);
    double width;
    double slot_height;
};

class Superellipse : public BaseSketchObject {
public:
    Superellipse(double width, double height, double order = 4.0, int point_count = 16,
                 double rotation = 0.0, Align2 align = Align::CENTER);
    double width;
    double height_;
    double order;
    int point_count;
    Align2 align;
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
class Text : public BaseSketchObject {
public:
    Text(const std::string& txt, double font_size, const TextOptions& options = {});
    std::string txt;
    double font_size;
    std::string font;
    std::string font_path;
    FontStyle font_style;
    std::pair<TextAlign, TextAlign> text_align;
    std::optional<Align2> align;
    std::optional<Wire> text_path;
    double position_on_path;
};

/// Font family names known to the font manager.
std::vector<std::string> available_fonts();

// ---------------------------------------------------------------------------
// 3D objects
// ---------------------------------------------------------------------------

class Box : public BasePartObject {
public:
    Box(double length, double width, double height, const Rotation& rotation = Rotation(),
        Align3 align = Align::CENTER);
    Box(double length, double width, double height, Align3 align);
    double length;
    double width;
    double box_height;
};

class Cylinder : public BasePartObject {
public:
    Cylinder(double radius, double height, double arc_size = 360.0,
             const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Cylinder(double radius, double height, Align3 align);
    Cylinder(double radius, double height, const Rotation& rotation, Align3 align = Align::CENTER);
    double radius;
    double cylinder_height;
    double arc_size;
    Align3 align;
};

class Cone : public BasePartObject {
public:
    Cone(double bottom_radius, double top_radius, double height, double arc_size = 360.0,
         const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Cone(double bottom_radius, double top_radius, double height, Align3 align);
    Cone(double bottom_radius, double top_radius, double height, const Rotation& rotation,
         Align3 align = Align::CENTER);
    double bottom_radius;
    double top_radius;
    double cone_height;
    double arc_size;
    Align3 align;
};

class Sphere : public BasePartObject {
public:
    explicit Sphere(double radius, double arc_size1 = -90.0, double arc_size2 = 90.0,
                    double arc_size3 = 360.0, const Rotation& rotation = Rotation(),
                    Align3 align = Align::CENTER);
    Sphere(double radius, Align3 align);
    Sphere(double radius, const Rotation& rotation, Align3 align = Align::CENTER);
    double radius;
    double arc_size1;
    double arc_size2;
    double arc_size3;
    Align3 align;
};

class Torus : public BasePartObject {
public:
    Torus(double major_radius, double minor_radius, double minor_start_angle = 0.0,
          double minor_end_angle = 360.0, double major_angle = 360.0,
          const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Torus(double major_radius, double minor_radius, Align3 align);
    Torus(double major_radius, double minor_radius, const Rotation& rotation,
          Align3 align = Align::CENTER);
    double major_radius;
    double minor_radius;
    double minor_start_angle;
    double minor_end_angle;
    double major_angle;
    Align3 align;
};

class Wedge : public BasePartObject {
public:
    Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax,
          double zmax, const Rotation& rotation = Rotation(), Align3 align = Align::CENTER);
    Wedge(double xsize, double ysize, double zsize, double xmin, double zmin, double xmax,
          double zmax, Align3 align);
    double xsize;
    double ysize;
    double zsize;
    double xmin;
    double zmin;
    double xmax;
    double zmax;
    Align3 align;
};

/// Hole tool: subtract it, e.g. part - Pos(x, y, top) * Hole(2, 10). A depth is required:
/// "through the part" (no depth) needs a builder, as in build123d's algebra mode.
class Hole : public BasePartObject {
public:
    explicit Hole(double radius, std::optional<double> depth = std::nullopt);
    double radius;
    double hole_depth;  ///< twice the depth, as in build123d
};

/// Counter bore hole tool, top at the origin, going down -Z.
class CounterBoreHole : public BasePartObject {
public:
    CounterBoreHole(double radius, double counter_bore_radius, double counter_bore_depth,
                    std::optional<double> depth = std::nullopt);
    double radius;
    double counter_bore_radius;
    double counter_bore_depth;
    double hole_depth;
};

/// Counter sink hole tool, top at the origin, going down -Z.
class CounterSinkHole : public BasePartObject {
public:
    CounterSinkHole(double radius, double counter_sink_radius,
                    std::optional<double> depth = std::nullopt, double counter_sink_angle = 82.0);
    double radius;
    double counter_sink_radius;
    double hole_depth;
    double counter_sink_angle;
};

/// Convex solid enclosing the points.
class ConvexPolyhedron : public BasePartObject {
public:
    explicit ConvexPolyhedron(const std::vector<Vector>& points,
                              const Rotation& rotation = Rotation(), Align3 align = Align::NONE);
};

}  // namespace build123d
