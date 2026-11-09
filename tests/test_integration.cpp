// Unit tests for C++-only details of features that span the work areas.
#include <build123d/build123d.hpp>

#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>

#include "test_framework.hpp"

using namespace build123d;

TEST(geom_adaptors) {
    const Edge line = Edge::make_line(Vector(0, 0, 0), Vector(3, 4, 0));
    CHECK(line.geom_adaptor()->GetType() == GeomAbs_Line);
    CHECK_NEAR(GCPnts_AbscissaPoint::Length(*line.geom_adaptor()), 5, 1e-9);
    const Wire wire = Polyline({{0, 0}, {1, 0}, {1, 2}});
    CHECK_NEAR(GCPnts_AbscissaPoint::Length(*wire.geom_adaptor()), 3, 1e-9);
    CHECK(!Handle(Geom_Plane)::DownCast(Rectangle(1, 2).face().geom_adaptor()).IsNull());
    CHECK_THROWS(Edge().geom_adaptor(), std::invalid_argument);
    CHECK_THROWS(Wire().geom_adaptor(), std::invalid_argument);
}

TEST(dxf_metric_units) {
    CHECK(ExportDXF::METRIC_UNITS.count(Unit::MM) == 1);
    CHECK(ExportDXF::METRIC_UNITS.count(Unit::IN) == 0);
    CHECK(ExportDXF::METRIC_UNITS.size() == 3);
}

// ---------------------------------------------------------------------------
// The README's claims: "Static types" and the "From Python to C++" table
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Box(1, 1, 1)), Part>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Line({0, 0}, {1, 0})), Curve>);
static_assert(std::is_same_v<decltype(Line({0, 0}, {2, 2}) & Line({0, 2}, {2, 0})), Compound>);
static_assert(std::is_same_v<decltype(Circle(1) & Circle(2)), Compound>);
static_assert(!std::is_convertible_v<Part, Sketch> && !std::is_convertible_v<Sketch, Part>);
static_assert(!std::is_convertible_v<Part, Curve> && !std::is_convertible_v<Curve, Part>);
static_assert(!std::is_convertible_v<Sketch, Curve> && !std::is_convertible_v<Curve, Sketch>);
static_assert(std::is_convertible_v<Compound, Sketch> && std::is_convertible_v<Compound, Curve> &&
              std::is_convertible_v<Compound, Part>);
static_assert(std::is_same_v<decltype(fillet(Rectangle(2, 2).vertices(), 0.5)), Compound>);
static_assert(std::is_same_v<decltype(chamfer(Rectangle(2, 2).vertices(), 0.5)), Compound>);
static_assert(std::is_same_v<decltype(Axis::Z.intersect(Plane::XY)), std::optional<std::variant<Vector, Axis>>>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1).intersect(Box(1, 1, 1))), ShapeList<Shape>>);

TEST(readme_python_to_cpp_table) {
    CHECK(Box(1, 2, 3, Align::MIN).bounding_box().min == Vector(0, 0, 0));
    CHECK(Box(1, 2, 3, {Align::MIN, Align::CENTER, Align::MAX}).bounding_box().max == Vector(1, 1, 0));

    const Sketch disk = Circle(1);
    CHECK_NEAR(extrude(disk, 5, {.both = true, .taper = 10}).bounding_box().size.Z, 10, 1e-6);
    CHECK_NEAR(extrude(disk, Until::NEXT, Pos(0, 0, 10) * Box(5, 5, 1)).bounding_box().max.Z, 9.5, 1e-6);
    const Part block = Box(4, 4, 4);
    const Part shelled = offset(block, -1, {.openings = {(block.faces() > Axis::Z).last()}});
    CHECK_NEAR(shelled.volume(), 4 * 4 * 4 - 2 * 2 * 3, 1e-6);
    const Part swept = sweep(std::vector<Sketch>{Circle(1), Pos(0, 0, 10) * Circle(2)},
                             Edge::make_line({0, 0, 0}, {0, 0, 10}), {.multisection = true});
    CHECK_NEAR(swept.volume(), M_PI * 10 * (1 + 2 + 4) / 3, 1e-6);
    const Triangle triangle({.a = 3, .b = 4, .c = 5});
    CHECK_NEAR(triangle.area(), 6, 1e-9);
    const CenterArc to_line({0, 0}, 1, 0, Edge::make_line({-2, 0.5}, {2, 0.5}));
    CHECK_NEAR(to_line.position_at(1).Y, 0.5, 1e-6);
    [[maybe_unused]] const auto bold = [] { return Text("hi", 10, {.font_style = FontStyle::BOLD}); };

    const Curve curve = Line({0, 0}, {2, 0});
    CHECK(curve.position_at(0.5) == Vector(1, 0, 0));
    CHECK((curve % 0.5) == Vector(1, 0, 0));
    CHECK((curve ^ 0.5).position() == Vector(1, 0, 0));
    CHECK_NEAR(Box(1, 2, 3).volume(), 6, 1e-9);
    CHECK_NEAR(Edge::make_line({0, 0}, {3, 4}).length(), 5, 1e-9);
    CHECK_NEAR(Vertex(1, 2, 3).X(), 1, 1e-12);

    const ShapeList<Face> faces = Box(1, 2, 3).faces();
    CHECK(faces[-1].is_same(faces[5]) && faces.last().is_same(faces[5]));
    CHECK(faces.slice(1, 3).size() == 2);
    const ShapeList<Edge> edges = Box(1, 2, 3).edges();
    CHECK(edges.filter_by([](const Edge& e) { return e.length() > 1; }).size() == 8);
    const Sketch ring = Circle(2) - Circle(1);
    CHECK(ring.edges().group_by([](const Edge& e) { return e.radius(); }).size() == 2);
    CHECK((edges > Axis::Z).size() == 12 && (edges >> Axis::Z).size() == 4 && (edges << Axis::Z).size() == 4 &&
          (edges | Axis::Z).size() == 4);

    const Part joined = Box(10, 10, 10) + Cylinder(2, 20);
    CHECK(joined.edges(Select::NEW).size() == 2);
    CHECK((GridLocations(5, 5, 2, 2) * Circle(1)).size() == 4);
    CHECK(Plane({0, 0, 5}, std::nullopt, {1, 0, 0}).z_dir == Vector(1, 0, 0));
    CHECK(Plane::from_x_y({0, 0, 0}, {1, 0, 0}, {0, 0, 1}).z_dir == Vector(0, -1, 0));
    CHECK(Rot(0, 0, 45).orientation() == Vector(0, 0, 45));
    const Vector point(1, 2, 3);
    CHECK(Pos(point).position() == point);
    const auto hit = Axis::Z.intersect(Plane::XY);
    CHECK(hit && std::holds_alternative<Vector>(*hit));
    CHECK(Box(1, 1, 1).intersect(Pos(5, 0, 0) * Box(1, 1, 1)).empty());
    auto [top, bottom] = Box(2, 2, 2).split(Plane::XY, Keep::BOTH);
    CHECK(top.size() == 1 && bottom.size() == 1);

    Compound assembly = Compound::make_assembly({Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)});
    Compound same;
    same.children = {Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)};
    CHECK(assembly.children.size() == 2 && same.children.size() == 2);
    assembly.children.push_back(Pos(0, 5, 0) * Box(1, 1, 1));
    CHECK(assembly.children[2].parent() == &assembly);
    CHECK(assembly.children[1].global_location().position() == Vector(3, 0, 0));

    Part base = Box(10, 10, 10);
    Part lid = Box(10, 10, 1);
    RevoluteJoint("j", base, Axis({5, 0, 5}, {0, 1, 0}));
    RigidJoint("k", lid, Location({5, 0, 0}));
    base.joints["j"].connect_to(lid.joints["k"], {.angle = 90});
    CHECK(lid.location().position() != Vector(0, 0, 0));
    {
        SkipClean skip;
        CHECK((Box(1, 1, 1) + Pos(1, 0, 0) * Box(1, 1, 1)).faces().size() == 10);
    }
    CHECK((Box(1, 1, 1) + Pos(1, 0, 0) * Box(1, 1, 1)).faces().size() == 6);
    CHECK(!shape_to_html(block).empty());

    std::vector<std::string> warnings;
    const WarningHandler previous = set_warning_handler([&](const std::string& m) { warnings.push_back(m); });
    warn("from the README");
    set_warning_handler(previous);
    CHECK(warnings == std::vector<std::string>{"from the README"});

    CHECK_THROWS(Vector().normalized(), std::invalid_argument);
    CHECK_THROWS(faces[6], std::out_of_range);
    CHECK_THROWS(Vector(1, 0, 0) / 0.0, std::domain_error);

    const Shape untyped(block.solids()[0].wrapped());
    CHECK_NEAR(cast_shape<Solid>(untyped).volume(), 64, 1e-9);
    CHECK(std::visit([](const auto& typed) { return typed.shape_type(); }, cast_shape(untyped)) == ShapeType::SOLID);
}

TEST(color_material_properties) {
    Compound assembly = Compound::make_assembly({Box(1, 1, 1)});
    assembly.color = "red";
    const std::optional<Color> inherited = assembly.children[0].color;
    CHECK(inherited && *inherited == Color("red"));
    auto snapshot = assembly.children[0].color;  // a value of its own
    snapshot = Color("blue");
    CHECK(assembly.children[0].color == Color("red"));
    CHECK(snapshot == Color("blue"));

    Part part = Box(1, 1, 1);
    CHECK(part.color == std::nullopt);
    part.color = assembly.color;
    CHECK(part.color.raw() == Color("red"));
    part.color.set_raw(std::nullopt);
    CHECK(!part.color);

    part.material = std::string("aluminum");
    CHECK(part.material->name() == "Alu_G6061_T6");
    CHECK(part.color.has_value());  // the material's look
    const Part copy = part;
    CHECK(copy.material->name() == "Alu_G6061_T6" && copy.color == part.color);
    part.material.set_raw(std::nullopt);
    CHECK(!part.material && part.color.has_value());
}
