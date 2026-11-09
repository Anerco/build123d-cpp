// build123d-cpp: a C++ port of the build123d algebra API on OpenCASCADE.
// Enumerations mirror build123d/build_enums.py.
#pragma once

#include <stdexcept>
#include <string>

namespace build123d {

/// Alignment of an object's bounding box relative to the origin.
enum class Align { MIN, CENTER, MAX, NONE };

/// Underlying geometry of an Edge or Face.
enum class GeomType {
    PLANE, CYLINDER, CONE, SPHERE, TORUS, BEZIER, BSPLINE, REVOLUTION,
    EXTRUSION, OFFSET, LINE, CIRCLE, ELLIPSE, HYPERBOLA, PARABOLA, OTHER
};

/// Sorting / grouping criteria for ShapeList.
enum class SortBy { LENGTH, RADIUS, AREA, VOLUME, DISTANCE };

/// Which part(s) of a split to keep.
enum class Keep { ALL, TOP, BOTTOM, BOTH, INSIDE, OUTSIDE };

/// Offset corner transition type.
enum class Kind { ARC, INTERSECTION, TANGENT };

/// Side of a 1D object to offset.
enum class Side { LEFT, RIGHT, BOTH };

/// Profile handling at C1 discontinuities when sweeping.
enum class Transition { RIGHT, ROUND, TRANSFORMED };

/// Limit for extrude(..., Until, target).
enum class Until { NEXT, LAST, PREVIOUS, FIRST };

/// How the center of an object is computed.
enum class CenterOf { GEOMETRY, MASS, BOUNDING_BOX };

enum class FontStyle { REGULAR, BOLD, ITALIC, BOLDITALIC };

enum class TextAlign { BOTTOM, CENTER, LEFT, RIGHT, TOP, TOPFIRSTLINE };

enum class AngularDirection { CLOCKWISE, COUNTER_CLOCKWISE };

/// Interpretation of a position along a 1D object.
enum class PositionMode { LENGTH, PARAMETER };

/// Moving-frame calculation used by location_at.
enum class FrameMethod { FRENET, CORRECTED };

/// How PolarLine's length is measured.
enum class LengthMode { DIAGONAL, HORIZONTAL, VERTICAL };

/// Euler angle orderings (rotating frame).
enum class Intrinsic { XYZ, XZY, YZX, YXZ, ZXY, ZYX, XYX, XZX, YZY, YXY, ZXZ, ZYZ };

/// Euler angle orderings (fixed frame).
enum class Extrinsic { XYZ, XZY, YZX, YXZ, ZXY, ZYX, XYX, XZX, YZY, YXY, ZXZ, ZYZ };

// <windows.h> defines IN as an empty macro.
#if defined(IN)
#pragma push_macro("IN")
#undef IN
#define BUILD123D_RESTORE_IN
#endif

/// Length and mass units (length units for the exporters, mass units for Shape::mass()).
enum class Unit { MC, MM, CM, M, IN, FT, G, KG, LB, INCH = IN };

/// Length units, in millimeters: Box(2 * IN, 1 * IN, 5 * MM).
inline constexpr double MC = 0.001;
inline constexpr double MM = 1.0;
inline constexpr double CM = 10.0 * MM;
inline constexpr double M = 1000.0 * MM;
inline constexpr double IN = 25.4 * MM;
inline constexpr double INCH = IN;
inline constexpr double FT = 12.0 * IN;
inline constexpr double THOU = IN / 1000.0;

/// Mass units, in grams.
inline constexpr double G = 1.0;
inline constexpr double KG = 1000.0 * G;
inline constexpr double G_PER_LB = 453.59237;
inline constexpr double LB = G_PER_LB * G;

/// build123d's UNITS_PER_METER: how many of a length unit make a meter.
inline double units_per_meter(Unit unit) {
    switch (unit) {
        case Unit::IN: return M / IN;
        case Unit::FT: return M / FT;
        case Unit::MC: return M / MC;
        case Unit::MM: return M / MM;
        case Unit::CM: return M / CM;
        case Unit::M: return 1.0;
        default: break;
    }
    throw std::invalid_argument("Not a length unit");
}

/// build123d's UNITS_PER_KILOGRAM: how many of a mass unit make a kilogram.
inline double units_per_kilogram(Unit unit) {
    switch (unit) {
        case Unit::G: return KG / G;
        case Unit::KG: return 1.0;
        case Unit::LB: return KG / LB;
        default: break;
    }
    throw std::invalid_argument("Not a mass unit");
}

#if defined(BUILD123D_RESTORE_IN)
#pragma pop_macro("IN")
#undef BUILD123D_RESTORE_IN
#endif

/// STEP geometric precision.
enum class PrecisionMode { LEAST = -1, AVERAGE = 0, GREATEST = 1, SESSION = 2 };

/// Topological shape types.
enum class ShapeType { VERTEX, EDGE, WIRE, FACE, SHELL, SOLID, COMPSOLID, COMPOUND, NONE };

/// Approximation of curves when converting shapes (e.g. to arcs or splines).
enum class ApproxOption { ARC, NONE, SPLINE };

/// Continuity between connected edges.
enum class ContinuityLevel { C0 = 0, C1 = 1, C2 = 2 };

/// Local shape of a vertex, edge or face.
enum class Convexity { CONVEX, CONCAVE, SMOOTH, SADDLE };

/// Arrow head style for drafting objects.
enum class HeadType { STRAIGHT, CURVED, FILLETED };

/// Which arcs a constrained-arc solver returns.
enum class Sagitta { SHORT = 0, LONG = -1, BOTH = 1 };

/// 3MF mesh object types.
enum class MeshType { OTHER, MODEL, SUPPORT, SOLIDSUPPORT };

/// How drafting objects display numbers.
enum class NumberDisplay { DECIMAL, FRACTION };

/// Paper sizes for TechnicalDrawing.
enum class PageSize { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, LETTER, LEGAL, LEDGER };

/// Qualifier of a constraint for the constrained line and arc solvers (GccEnt_Position).
enum class Tangency { UNQUALIFIED = 0, ENCLOSING = 1, ENCLOSED = 2, OUTSIDE = 3 };

/// Which sub-shapes a selector returns: all of them, those of the last operation, or
/// those the last operation created.
enum class Select { ALL, LAST, NEW };

std::string to_string(Align v);
std::string to_string(GeomType v);
std::string to_string(ShapeType v);

/// Alignment for 2D objects; a single Align applies to both axes.
struct Align2 {
    Align x = Align::CENTER;
    Align y = Align::CENTER;
    constexpr Align2() = default;
    constexpr Align2(Align a) : x(a), y(a) {}  // NOLINT: implicit like build123d's tuplify
    constexpr Align2(Align ax, Align ay) : x(ax), y(ay) {}
    constexpr Align operator[](int i) const { return i == 0 ? x : y; }
};

/// Alignment for 3D objects; a single Align applies to all axes.
struct Align3 {
    Align x = Align::CENTER;
    Align y = Align::CENTER;
    Align z = Align::CENTER;
    constexpr Align3() = default;
    constexpr Align3(Align a) : x(a), y(a), z(a) {}  // NOLINT
    constexpr Align3(Align ax, Align ay, Align az) : x(ax), y(ay), z(az) {}
    constexpr explicit Align3(Align2 a, Align az = Align::NONE) : x(a.x), y(a.y), z(az) {}
    constexpr Align operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};

}  // namespace build123d
