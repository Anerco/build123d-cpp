// Unit tests for the objects, operations, text and drafting API (the C++ side of what
// tests/compare/cases_objects*.{cpp,py} compare with Python).
#include <build123d/build123d.hpp>

#include <type_traits>

#include "test_framework.hpp"

using namespace build123d;

// ---------------------------------------------------------------------------
// Base classes: the objects derive from them as in Python, and keep the algebra types
// ---------------------------------------------------------------------------

static_assert(std::is_base_of_v<BaseEdgeObject, Line>);
static_assert(std::is_base_of_v<BaseEdgeObject, JernArc>);
static_assert(std::is_base_of_v<BaseLineObject, Polyline>);
static_assert(std::is_base_of_v<BaseLineObject, FilletPolyline>);
static_assert(std::is_base_of_v<BaseSketchObject, Circle>);
static_assert(std::is_base_of_v<BaseSketchObject, Text>);
static_assert(std::is_base_of_v<BasePartObject, Box>);
static_assert(std::is_base_of_v<BasePartObject, CounterSinkHole>);
static_assert(std::is_same_v<decltype(Circle(1) + Rectangle(1, 1)), Sketch>);
static_assert(std::is_same_v<decltype(Pos(1, 0) * Circle(1)), Sketch>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) - Hole(0.1, 1)), Part>);
static_assert(std::is_same_v<decltype(Pos(1, 0) * BasePartObject(Box(1, 1, 1))), Part>);
static_assert(std::is_same_v<decltype(Line({0, 0}, {1, 0}) + Line({1, 0}, {1, 1})), Curve>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1).rotation), Rotation>);
static_assert(std::is_same_v<decltype(Circle(1).rotation), double>);

namespace {

class TiltedSquare : public BaseSketchObject {
public:
    explicit TiltedSquare(double size, double rotation = 0) : BaseSketchObject(Rectangle(size, size), rotation) {}
};

}  // namespace

TEST(objects_base_classes) {
    const TiltedSquare sq(2, 45);
    CHECK_NEAR(sq.area(), 4, 1e-9);
    CHECK_NEAR(sq.rotation, 45, 0);
    CHECK_NEAR(sq.bounding_box().size.X, 2 * std::sqrt(2.0), 1e-9);
    const Sketch moved = Pos(5, 0) * sq;
    CHECK_NEAR(moved.center().X, 5, 1e-9);

    // BasePartObject keeps the label of the part it wraps, like Python
    Part p = Box(1, 2, 3);
    p.label = "block";
    const BasePartObject based(p, Rotation(0, 0, 90));
    CHECK(based.label == "block");
    CHECK_NEAR(based.bounding_box().size.X, 2, 1e-9);

    const BaseEdgeObject e(Edge::make_line({0, 0}, {3, 4}));
    CHECK_NEAR(e.length(), 5, 1e-12);
    const BaseCurveObject empty_curve{Curve()};
    CHECK(empty_curve.is_null());
}

TEST(objects_hole_depth_required) {
    // Without a builder there is no part to go through, as in Python's algebra mode.
    CHECK_THROWS(Hole(1), std::invalid_argument);
    CHECK_THROWS(CounterBoreHole(1, 2, 1), std::invalid_argument);
    CHECK_THROWS(CounterSinkHole(1, 2), std::invalid_argument);
    CHECK_NEAR(Hole(1, 2).hole_depth, 4, 0);
}

// ---------------------------------------------------------------------------
// Curve objects with solvers
// ---------------------------------------------------------------------------

static_assert(std::is_base_of_v<BaseCurveObject, ConstrainedArcs>);
static_assert(std::is_base_of_v<BaseCurveObject, ConstrainedLines>);
static_assert(std::is_base_of_v<BaseEdgeObject, BlendCurve>);
static_assert(std::is_base_of_v<BaseEdgeObject, DoubleTangentArc>);
static_assert(std::is_base_of_v<BaseLineObject, Airfoil>);
static_assert(std::is_convertible_v<Line, TangencyTarget>);
static_assert(std::is_convertible_v<Axis, TangencyTarget>);
static_assert(std::is_convertible_v<Vertex, TangencyTarget>);
static_assert(std::is_convertible_v<Vector, TangencyTarget>);
static_assert(!std::is_convertible_v<Curve, TangencyTarget>);
static_assert(std::is_convertible_v<Edge, EdgeSelection>);
static_assert(std::is_convertible_v<ShapeList<Edge>, EdgeSelection>);

TEST(objects_tangency_targets) {
    const TangencyTarget edge = Line({0, 0}, {1, 0});
    CHECK(edge.kind() == TangencyTarget::Kind::EDGE && !edge.qualified());
    const TangencyTarget qualified(Edge::make_circle(1), Tangency::ENCLOSING);
    CHECK(qualified.qualified() && qualified.tangency() == Tangency::ENCLOSING);
    const TangencyTarget point = {1, 2};
    CHECK(point.kind() == TangencyTarget::Kind::POINT && point.point() == Vector(1, 2));
    const TangencyTarget vertex = Vertex(1, 2, 3);
    CHECK(vertex.kind() == TangencyTarget::Kind::VERTEX);
    CHECK_THROWS(vertex.edge(), std::invalid_argument);
}

TEST(objects_constrained_errors) {
    // Python's ValueError/TypeError -> invalid_argument, RuntimeError -> runtime_error
    CHECK_THROWS(Edge::make_constrained_arcs(Vector(0, 0), Vector(10, 0), {.radius = 2}), std::runtime_error);
    CHECK_THROWS(Edge::make_constrained_arcs(Vector(0, 0), Vector(1, 0), {.radius = -1}), std::invalid_argument);
    CHECK_THROWS(Edge::make_constrained_lines(Edge::make_circle(1), Axis::Z, {.angle = 1}), std::invalid_argument);
    // the selector may return one edge or several
    const ConstrainedLines all(CenterArc({-5, 0}, 4, 0, 360), CenterArc({5, 0}, 3, 0, 360));
    CHECK(all.edges().size() == 4);
    const ConstrainedLines one(CenterArc({-5, 0}, 4, 0, 360), CenterArc({5, 0}, 3, 0, 360), {},
                               [](const ShapeList<Edge>& lines) { return lines[0]; });
    CHECK(one.edges().size() == 1);
    CHECK_THROWS(DoubleTangentArc({0, 0}, {1, 0}, Curve(Line({0, 5}, {10, 5}))), std::invalid_argument);
    const DoubleTangentArc up({0, 0}, {0, 1}, Line({0, 5}, {10, 5}));
    CHECK((up.position_at(1) - Vector(5, 5)).length() < 1e-5);
}

TEST(objects_airfoil) {
    const Airfoil a("2412", 40);
    CHECK(a.code == "2412");
    CHECK_NEAR(a.thickness, 0.12, 1e-15);
    CHECK(a.edges().size() == 1 && a.edges()[0].is_closed());
    const Airfoil finite("2412", 40, true);
    CHECK(finite.edges().size() == 2);
    CHECK_THROWS(Airfoil("2"), std::out_of_range);
    CHECK_THROWS(Airfoil("24x2"), std::invalid_argument);
}
