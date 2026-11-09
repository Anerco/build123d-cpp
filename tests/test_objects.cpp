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
