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

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------

static_assert(std::is_convertible_v<Line, Limit>);
static_assert(std::is_convertible_v<Box, Limit>);
static_assert(std::is_convertible_v<Axis, Limit>);
static_assert(std::is_convertible_v<Pos, Limit>);
static_assert(std::is_convertible_v<Plane, Limit>);
static_assert(std::is_convertible_v<Vector, Limit>);
static_assert(!std::is_convertible_v<double, Limit>);
static_assert(std::is_same_v<decltype(Edge().trim_to_other(Axis::X)), std::optional<Edge>>);

TEST(objects_limits) {
    // a number is still an angle, a shape a limit
    const CenterArc numeric({0, 0}, 1, 0, 90);
    CHECK_NEAR(numeric.length(), PI / 2, 1e-9);
    const CenterArc limited({1, 0}, 1, 0, Line({1, 0}, {2, 1}));
    CHECK_NEAR(limited.length(), PI / 4, 1e-6);
    const PolarLine to_axis({2, 0}, Axis::Y, 135);
    CHECK_NEAR(to_axis.position_at(1).X, 0, 1e-9);
    // no intersection: nothing, and the objects throw invalid_argument (Python's ValueError)
    CHECK(!Line({0, 0}, {1, 0}).trim_to_other(Vector(5, 5)));
    CHECK_THROWS(CenterArc({1, 0}, 1, 0, {5, 0}), std::invalid_argument);
    CHECK_THROWS(PolarLine({0, 0}, Axis::Y, 0.0), std::invalid_argument);
    CHECK_THROWS(JernArc({1, 0}, {0, 1}, 1, {5, 0}), std::invalid_argument);
    // trim_infinite only changes infinite lines
    CHECK_NEAR(Edge(Axis::X).trim_infinite(3).length(), 6, 1e-9);
    CHECK_NEAR(Line({0, 0}, {1, 0}).trim_infinite(3).length(), 1, 1e-12);
}

// ---------------------------------------------------------------------------
// Projection
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(Wire().project_to_shape(Box(1, 1, 1), Vector(0, 0, 1))), ShapeList<Wire>>);
static_assert(std::is_same_v<decltype(Edge().project_to_shape(Box(1, 1, 1), Vector(0, 0, 1))), ShapeList<Edge>>);
static_assert(std::is_same_v<decltype(Edge().project(Face(), Vector(0, 0, 1))), Mixin1D>);
static_assert(std::is_same_v<decltype(Edge().project(Face(), Vector(0, 0, 1), false)), ShapeList<Mixin1D>>);
static_assert(std::is_same_v<decltype(project(Face(), Plane::XY)), Sketch>);
static_assert(std::is_same_v<decltype(project(std::vector<Face>{}, Plane::XY)), Sketch>);
static_assert(std::is_same_v<decltype(project(Edge(), Plane::XY)), Curve>);
static_assert(std::is_same_v<decltype(project(Vector(), Plane::XY)), std::vector<Vector>>);
static_assert(std::is_same_v<decltype(project(Vertex(), Plane::XY)), std::vector<Vector>>);
static_assert(std::is_same_v<decltype(project(std::vector<Shape>{}, Plane::XY)),
                             std::variant<Compound, std::vector<Vector>>>);

TEST(objects_projection) {
    const Wire circle = Wire::make_circle(3, Plane::XY.offset(10));
    const Part target = Box(10, 10, 4) - Pos(0, 0, 2) * Box(5, 5, 2);
    const auto down = circle.project_to_shape(target, Vector(0, 0, -1));
    CHECK(down.size() == 2);
    CHECK(down[0].bounding_box().max.Z > down[1].bounding_box().max.Z);  // sorted by distance
    const auto conical = circle.project_to_shape(target, std::nullopt, Vector(0, 0, 20));
    CHECK(conical.size() == 4);  // the cone from the center hits the outer walls, in four pieces
    CHECK_THROWS(circle.project_to_shape(target), std::invalid_argument);
    const Mixin1D closest = Edge::make_circle(1, Plane::XY.offset(5)).project(Face::make_rect(10, 10), Vector(0, 0, -1));
    CHECK(closest.shape_type() == ShapeType::WIRE);
    CHECK_NEAR(closest.position_at(0).Z, 0, 1e-9);
    CHECK_THROWS(project(std::vector<Face>{}, Plane::XY), std::invalid_argument);
    CHECK(project(Vertex(1, 2, 3), Plane::XZ)[0] == Vector(1, 3, 0));
}

// ---------------------------------------------------------------------------
// full_round, make_brake_formed
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(full_round(Edge())), Sketch>);
static_assert(std::is_same_v<decltype(make_brake_formed(1.0, 1.0, Edge())), Part>);
static_assert(std::is_same_v<decltype(make_brake_formed(1.0, std::vector<double>{1.0}, Wire())), Part>);
static_assert(std::is_same_v<decltype(make_brake_formed(1.0, 1.0, Curve())), Part>);

TEST(objects_full_round_and_brake_formed) {
    const Sketch rounded = full_round((Rectangle(34, 10).edges() << Axis::X)[0]);
    CHECK(rounded.faces().size() == 1);
    CHECK(rounded.edges().filter_by(GeomType::CIRCLE).size() == 1);
    CHECK_THROWS(full_round(Edge::make_line({0, 0}, {1, 0})), std::invalid_argument);

    const FilletPolyline outline({{0, 0}, {5, 6}, {10, 1}}, 1);
    const Part sheet = make_brake_formed(0.5, {1, 2, 3, 4}, outline);
    CHECK(sheet.solids().size() == 1);
    CHECK_NEAR(sheet.bounding_box().max.Z, 4, 1e-2);
    CHECK_NEAR(make_brake_formed(0.5, 1, Curve() + outline).volume(), make_brake_formed(0.5, 1, outline).volume(), 1e-9);
    CHECK_THROWS(make_brake_formed(0.5, std::vector<double>{1, 2}, outline), std::invalid_argument);
    CHECK_THROWS(make_brake_formed(0.5, 1, Wire()), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// pack
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(pack(std::vector<Box>{}, 1.0)), ShapeList<Part>>);
static_assert(std::is_same_v<decltype(pack(std::vector<Shape>{}, 1.0)), ShapeList<Shape>>);
static_assert(std::is_same_v<decltype(pack(ShapeList<Face>{}, 1.0, true)), ShapeList<Face>>);

TEST(objects_pack) {
    const auto packed = pack(std::vector<Box>{Box(10, 2, 1), Box(1, 5, 1), Box(1, 5, 1)}, 1);
    CHECK(packed.size() == 3);
    CHECK(packed[1].bounding_box().min == Vector(0, 3, -0.5));
    CHECK(packed[2].bounding_box().min == Vector(2, 3, -0.5));
    CHECK(pack(std::vector<Part>{Pos(0, 0, 5) * Box(1, 1, 1)}, 0, true)[0].bounding_box().min == Vector(0, 0, 0));
    CHECK_THROWS(pack(std::vector<Part>{}, 1), std::invalid_argument);
    CHECK_THROWS(pack(std::vector<Box>{Box(2, 2, 2), Box(2, 2, 2)}, -1.5), std::runtime_error);
    const auto positions = detail::pack2d({{2, 10}, {3, 2}});
    CHECK(positions == (std::vector<std::pair<double, double>>{{0, 0}, {2, 0}}));
}

// ---------------------------------------------------------------------------
// Operation overloads (lists wherever build123d takes them)
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(extrude(std::vector<Face>{}, Until::NEXT, Part())), Part>);
static_assert(std::is_same_v<decltype(revolve(std::vector<Sketch>{})), Part>);
static_assert(std::is_same_v<decltype(revolve(std::vector<Shape>{})), Part>);
static_assert(std::is_same_v<decltype(thicken(std::vector<Face>{}, 1.0)), Part>);
static_assert(std::is_same_v<decltype(draft(Face(), Plane::XY, 5.0)), Part>);
static_assert(std::is_same_v<decltype(make_face(std::vector<Wire>{})), Sketch>);
static_assert(std::is_same_v<decltype(make_face(std::vector<Shape>{})), Sketch>);
static_assert(std::is_same_v<decltype(trace(std::vector<Curve>{})), Sketch>);
static_assert(std::is_same_v<decltype(scale(std::vector<Part>{}, Vector(1, 2, 3))), Part>);
static_assert(std::is_same_v<decltype(scale(Box(1, 1, 1))), Part>);
static_assert(std::is_same_v<decltype(split(std::vector<Part>{}, Face())), Part>);
static_assert(std::is_same_v<decltype(offset(Rectangle(1, 1))), Sketch>);
static_assert(std::is_same_v<decltype(sweep(Circle(1), std::vector<Edge>{})), Part>);
static_assert(std::is_same_v<decltype(sweep(std::vector<Edge>{}, std::vector<Edge>{})), Sketch>);

TEST(objects_operation_overloads) {
    // Empty inputs behave as in build123d: an empty Sketch gives an empty Part, an empty list of
    // profiles raises
    CHECK(extrude(Sketch(), 1).solids().empty());
    CHECK(revolve(Sketch()).solids().empty());
    CHECK_THROWS(revolve(std::vector<Face>{}), std::invalid_argument);
    CHECK(thicken(std::vector<Face>{}, 1).solids().empty());
    CHECK_THROWS(trace(std::vector<Edge>{}), std::out_of_range);
    const std::vector<Edge> path{Edge::make_line({0, 0, 0}, {0, 0, 5}), Edge::make_line({0, 0, 5}, {3, 0, 8})};
    CHECK(sweep(Circle(1), path).solids().size() == 1);
    CHECK(sweep(Circle(1), {path[0], path[1]}).solids().size() == 1);
    const std::vector<Edge> broken{Edge::make_line({0, 0, 0}, {0, 0, 5}), Edge::make_line({0, 1, 6}, {3, 0, 8})};
    CHECK_THROWS(sweep(Circle(1), broken), std::invalid_argument);
    CHECK_THROWS(offset(Rectangle(10, 2) - Circle(0.8), -0.3), std::invalid_argument);
    CHECK_NEAR(scale(std::vector<Part>{Box(2, 2, 2)}, Vector(1, 2, 3)).volume(), 48, 1e-9);
}

// ---------------------------------------------------------------------------
// Text and fonts
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(available_fonts()), std::vector<FontInfo>>);
static_assert(std::is_same_v<decltype(FontManager().register_font("")), std::vector<std::string>>);
static_assert(std::is_same_v<decltype(Compound::resolve_font()),
                             std::tuple<std::string, std::string, Handle(Font_SystemFont)>>);

TEST(objects_text_fonts) {
    // The bundled single-line font is found (in the source tree here) and registered
    CHECK(!FontManager::bundled_path().empty());
    const auto [name, path, system_font] = Compound::resolve_font("singleline");
    CHECK(name == "Relief SingleLine CAD");
    CHECK(path.find("reliefsingleline/ReliefSingleLineCAD-Regular.ttf") != std::string::npos);
    CHECK(system_font->IsSingleStrokeFont());
    const auto fonts = available_fonts();
    CHECK(std::is_sorted(fonts.begin(), fonts.end(), [](const FontInfo& a, const FontInfo& b) { return a.name < b.name; }));
    CHECK(std::find(fonts.begin(), fonts.end(), FontInfo{"Relief SingleLine CAD", {FontStyle::REGULAR}}) != fonts.end());
    CHECK(FontInfo({"Arial", {FontStyle::REGULAR}}).repr() == "Font(name='Arial', styles=('REGULAR',))");

    // Single-line text: wires from make_text, outlined faces from Text
    const Compound strokes = Compound::make_text("Hi", 5, "singleline");
    CHECK(strokes.faces().empty() && !strokes.wires().empty());
    const Text text("Hi", 5, {.font = "singleline"});
    CHECK_NEAR(text.single_line_width, 0.2, 1e-12);
    CHECK(!text.faces().empty());
    CHECK(text.font == "Relief SingleLine CAD");
    CHECK_THROWS(Text("Hi", 5, {.single_line_width = 0}), std::invalid_argument);
    CHECK_THROWS(FontManager().register_font("/nonexistent/font.ttf"), std::runtime_error);
}
