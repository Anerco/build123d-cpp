// Unit tests for build123d-cpp. Geometry results are compared with Python build123d by
// tests/compare; these tests cover the C++ API itself. More tests live in the other
// tests/test_*.cpp files. Run: build123d_tests [test-name ...]
#include <build123d/build123d.hpp>

#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopoDS.hxx>

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "test_framework.hpp"

using namespace build123d;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Static types of the algebra
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(Box(1, 1, 1) + Sphere(1)), Part>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) - Sphere(1)), Part>);
static_assert(std::is_same_v<decltype(Circle(1) - Rectangle(1, 1)), Sketch>);
static_assert(std::is_same_v<decltype(Line({0, 0}, {1, 0}) + Line({1, 0}, {1, 1})), Curve>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Rectangle(1, 1)), Sketch>);
static_assert(std::is_same_v<decltype(Pos(1, 0) * Box(1, 1, 1)), Part>);
static_assert(std::is_same_v<decltype(Plane::XZ * Circle(1)), Sketch>);
static_assert(std::is_same_v<decltype(Rot(0, 0, 45) * Edge::make_line({0, 0}, {1, 0})), Edge>);
static_assert(std::is_same_v<decltype(GridLocations(1, 1, 2, 2) * Circle(1)), ShapeList<Sketch>>);
static_assert(std::is_same_v<decltype(extrude(Circle(1), 1)), Part>);
static_assert(std::is_same_v<decltype(sweep(Line({0, 0}, {1, 0}), Line({0, 0}, {0, 0, 1}))), Sketch>);
static_assert(!std::is_constructible_v<Part, Sketch>);
static_assert(!std::is_constructible_v<Sketch, Curve>);

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

TEST(vector_math) {
    const Vector a(1, 2, 3), b(4, 5, 6);
    CHECK(a + b == Vector(5, 7, 9));
    CHECK(b - a == Vector(3, 3, 3));
    CHECK(a * 2 == Vector(2, 4, 6));
    CHECK(2 * a == Vector(2, 4, 6));
    CHECK_NEAR(a.dot(b), 32, 1e-12);
    CHECK(a.cross(b) == Vector(-3, 6, -3));
    CHECK_NEAR(Vector(3, 4).length(), 5, 1e-12);
    // Equality rounds to 5 decimals like build123d
    CHECK(Vector(1, 2, 3) == Vector(1.000001, 2, 3));
    CHECK(Vector(1, 2, 3) != Vector(1.0001, 2, 3));
    CHECK_THROWS(Vector(0, 0, 0).normalized(), std::invalid_argument);
    CHECK_NEAR(Vector(1, 0, 0).get_angle(Vector(0, 1, 0)), 90, 1e-9);
    // The signed angle is measured about -Z by default
    CHECK_NEAR(Vector(1, 0, 0).get_signed_angle(Vector(0, 1, 0)), -90, 1e-9);
    CHECK(Vector(1, 0, 0).rotate(Axis::Z, 90) == Vector(0, 1, 0));
    CHECK(Vector(Vertex(1, 2, 3)) == Vector(1, 2, 3));
}

TEST(units) {
    CHECK_NEAR(IN, 25.4, 0);
    CHECK_NEAR(INCH, 25.4, 0);
    CHECK_NEAR(FT, 304.8, 1e-12);
    CHECK_NEAR(2 * CM, 20, 0);
    CHECK_NEAR(THOU, 0.0254, 1e-15);
}

TEST(axis) {
    const Axis a({0, 0, 0}, {0, 0, 5});
    CHECK(a.direction == Vector(0, 0, 1));
    CHECK(a.is_parallel(Axis::Z));
    CHECK(a.is_opposite(-Axis::Z));
    CHECK(Axis::X.is_normal(Axis::Y));
    const auto hit = Axis({0, 1, 0}, {1, 0, 0}).intersect(Axis({3, 0, 0}, {0, 1, 0}));
    CHECK(hit && std::get<Vector>(*hit) == Vector(3, 1, 0));
    CHECK(!Axis::X.intersect(Axis({0, 1, 0}, {1, 0, 0})).has_value());
}

TEST(location_math) {
    const Location loc = Pos(1, 2, 3) * Rot(0, 0, 90);
    CHECK(loc.position() == Vector(1, 2, 3));
    CHECK(loc.orientation() == Vector(0, 0, 90));
    // x of the rotated frame points along +Y
    CHECK((loc * Pos(1, 0, 0)).position() == Vector(1, 3, 3));
    CHECK(loc * loc.inverse() == Location());
    CHECK(Rot(0, 0, 90).pow(2) == Rot(0, 0, 180));
    CHECK(Location({1, 2, 3}, {10, 20, 30}).orientation() == Vector(10, 20, 30));
    CHECK(Rot(Axis::X, 90) == Rot(90, 0, 0));
}

TEST(plane_math) {
    const Plane p = Plane::XZ.offset(5);
    CHECK(p.origin == Vector(0, -5, 0));
    CHECK(p.y_dir == Vector(0, 0, 1));
    const Vector local(1, 2, 3);
    CHECK(p.to_local_coords(p.from_local_coords(local)) == local);
    CHECK(p.from_local_coords(Vector(1, 2, 0)) == Vector(1, -5, 2));
    CHECK_NEAR(Plane::isometric.z_dir.dot(Plane::isometric.x_dir), 0, 1e-12);
    CHECK_NEAR(Plane::isometric.x_dir.length(), 1, 1e-15);
    // Plane * Location gives a Location in the plane's coordinates ...
    CHECK((Plane::XZ * Pos(1, 2)).position() == Vector(1, 0, 2));
    // ... and Location * Plane moves the plane in its own coordinates
    CHECK((Pos(0, 0, 3) * Plane::XZ).origin == Vector(0, -3, 0));
    CHECK(Plane(Location({1, 2, 3}, {0, 0, 90})).x_dir == Vector(0, 1, 0));
    CHECK(Plane::from_points({0, 0, 0}, {1, 0, 0}, {0, 1, 0}).z_dir == Vector(0, 0, 1));
    // Intersections
    CHECK(!Plane::XY.intersect(Plane::XY.offset(1)).has_value());
    const auto line = Plane::XY.intersect(Plane::XZ);
    CHECK(line && std::get<Axis>(*line).is_parallel(Axis::X));
    const auto point = Plane::XY.intersect(Axis({0, 0, 5}, {0, 0, -1}));
    CHECK(point && std::get<Vector>(*point) == Vector(0, 0, 0));
}

TEST(plane_fields_stay_authoritative) {
    // The public fields may be edited; the OCCT views follow them.
    Plane q = Plane::XY;
    q.origin = Vector(1, 2, 3);
    CHECK(Vector(q.to_gp_ax3().Location()) == Vector(1, 2, 3));
    CHECK(Location(q).position() == Vector(1, 2, 3));
    q.x_dir = Vector(0, 1, 0);
    q.y_dir = Vector(-1, 0, 0);
    CHECK(Vector(q.to_gp_ax3().XDirection()) == Vector(0, 1, 0));
    CHECK((q * Rectangle(4, 2)).bounding_box().size == Vector(2, 4, 0));
}

// ---------------------------------------------------------------------------
// Topology and selectors
// ---------------------------------------------------------------------------

TEST(shape_list_indexing) {
    const Part box = Box(1, 2, 3);
    const auto faces = box.faces().sort_by(Axis::Z);
    CHECK(faces.size() == 6);
    CHECK(faces[-1].center().Z > 1.4);
    CHECK(faces[0].center().Z < -1.4);
    CHECK(faces.first().is_same(faces[0]));
    CHECK(faces.last().is_same(faces[5]));
    CHECK_THROWS(faces[6], std::out_of_range);
    CHECK_THROWS(faces[-7], std::out_of_range);
    CHECK(faces.slice(1, -1).size() == 4);
    CHECK(faces.slice(-2).size() == 2);
}

TEST(shape_list_selectors) {
    const Part box = Box(1, 2, 3);
    const auto edges = box.edges();
    const auto groups = edges.group_by(Axis::Z);
    CHECK(groups.size() == 3);
    CHECK(groups.keys == (std::vector<double>{-1.5, 0, 1.5}));
    CHECK(groups[-1].size() == 4);
    CHECK(groups.group(0.0).size() == 4);
    CHECK(groups.group_for(groups[0][0]).size() == 4);
    CHECK((edges >> Axis::Z).size() == 4);
    CHECK((edges << Axis::Z).size() == 4);
    CHECK((edges | Axis::Z).size() == 4);
    CHECK((box.faces() | Plane::XY).size() == 2);
    CHECK((box.faces() | GeomType::PLANE).size() == 6);
    CHECK(edges.filter_by([](const Edge& e) { return e.length() > 2.5; }).size() == 4);
    CHECK(edges.group_by(SortBy::LENGTH).keys == (std::vector<double>{1, 2, 3}));
    CHECK((edges - (edges | Axis::Z)).size() == 8);
    CHECK((edges & (edges | Axis::Z)).size() == 4);
    CHECK((box.faces() > Axis::X)[0].center().X < 0);
    CHECK((box.faces() < Axis::X)[0].center().X > 0);
    CHECK(box.faces().filter_by_position(Axis::Z, -1.5, 0, {true, false}).size() == 1);
    CHECK_NEAR(box.faces().sort_by(SortBy::AREA).last().area(), 2.0 * 3.0, 1e-9);
    CHECK_THROWS(box.edge(), std::invalid_argument);
    CHECK(Box(1, 1, 1).solid().volume() > 0.99);
}

TEST(topo_parent) {
    const Part box = Box(10, 10, 10);
    const Edge e = box.edges()[0];
    CHECK(e.topo_parent() && e.topo_parent()->is_same(box));
    // Sub-shapes of sub-shapes keep the outermost parent
    const Vertex v = box.faces()[0].vertices()[0];
    CHECK(v.topo_parent() && v.topo_parent()->is_same(box));
    const Edge free_edge = Edge::make_line({0, 0, 0}, {1, 0, 0});
    CHECK(!free_edge.topo_parent());
    CHECK_THROWS(fillet(free_edge, 1), std::invalid_argument);
    const Part rounded = fillet(box.edges().filter_by(Axis::Z), 1);
    CHECK(rounded.faces().size() == 10);
    CHECK_THROWS(fillet(box.edges(), 6), std::invalid_argument);
}

TEST(skip_clean) {
    const Part a = Box(1, 1, 1);
    const Part b = Pos(1, 0, 0) * Box(1, 1, 1);
    CHECK((a + b).faces().size() == 6);
    {
        const SkipClean skip;
        CHECK((a + b).faces().size() == 10);
    }
    CHECK((a + b).faces().size() == 6);
}

TEST(algebra) {
    const Part plate = Box(10, 10, 2) - GridLocations(5, 5, 2, 2) * Cylinder(1, 2);
    CHECK_NEAR(plate.volume(), 200 - 4 * PI * 2, 1e-6);
    Part acc;
    for (int i = 0; i < 3; ++i) acc += Pos(i * 2.0, 0, 0) * Box(1, 1, 1);
    CHECK(acc.solids().size() == 3);
    const Sketch s = Rectangle(2, 2) & Box(1, 1, 1);
    CHECK_NEAR(s.area(), 1, 1e-9);
    const Sketch rounded = fillet(Rectangle(4, 4).vertices(), 1);
    CHECK_NEAR(rounded.area(), 12 + PI, 1e-9);
    CHECK_THROWS(Part(Compound(Circle(1).wrapped())), std::invalid_argument);
    CHECK_THROWS(Sketch() - Circle(1), std::invalid_argument);
}

TEST(curves) {
    const Line line({0, 0}, {10, 0});
    CHECK(line.position_at(0.5) == Vector(5, 0, 0));
    CHECK((line % 0.5) == Vector(1, 0, 0));
    CHECK((line ^ 1).position() == Vector(10, 0, 0));
    const Curve curve = line + Line({10, 0}, {10, 10});
    CHECK_NEAR(curve.length(), 20, 1e-9);
    CHECK(curve.position_at(0.75) == Vector(10, 5, 0));
    CHECK(curve.wires().size() == 1);
    CHECK_NEAR(CenterArc({0, 0}, 2, 0, 90).length(), PI, 1e-9);
    CHECK(Polyline({{0, 0}, {1, 0}, {1, 1}}, true).is_closed());
}

TEST(locations) {
    CHECK((GridLocations(10, 20, 3, 2) * Box(1, 1, 1)).size() == 6);
    CHECK(GridLocations(10, 20, 3, 2)[0].position() == Vector(-10, -10, 0));
    const PolarLocations polar(10, 4);
    CHECK(polar[1].position() == Vector(0, 10, 0));
    CHECK(polar[1].orientation() == Vector(0, 0, 90));
    CHECK(HexLocations(1, 2, 2).size() == 4);
    CHECK(Locations({Vector(1, 2), Vector(3, 4)}).size() == 2);
}

TEST(errors_are_std_exceptions) {
    CHECK_THROWS(Edge::make_line({0, 0, 0}, {0, 0, 0}), std::runtime_error);
    CHECK_THROWS(Wire(std::vector<Edge>{Line({0, 0}, {1, 0}), Line({5, 5}, {6, 5})}), std::invalid_argument);
    CHECK_THROWS(RegularPolygon(1, 2), std::invalid_argument);
    CHECK_THROWS(Plane(Vector(0, 0, 0), std::nullopt, Vector(0, 0, 0)), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Import / export
// ---------------------------------------------------------------------------

namespace {

struct TempDir {
    fs::path path;
    TempDir() {
        path = fs::temp_directory_path() /
               ("build123d_tests_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    std::string file(const std::string& name) const { return (path / name).string(); }
};

}  // namespace

TEST(io_round_trips) {
    const TempDir dir;
    const Part part = Box(10, 20, 30) - Cylinder(4, 30);

    CHECK(export_brep(part, dir.file("part.brep")));
    CHECK_NEAR(import_brep(dir.file("part.brep")).volume(), part.volume(), 1e-9 * part.volume());

    CHECK(export_step(part, dir.file("part.step")));
    const Shape step = import_step(dir.file("part.step"));  // a Solid, as in Python
    CHECK_NEAR(step.volume(), part.volume(), 1e-6 * part.volume());
    CHECK(step.faces().size() == part.faces().size());

    CHECK(export_stl(part, dir.file("part.stl")));
    const Face mesh = import_stl(dir.file("part.stl"));
    TopLoc_Location loc;
    const auto triangulation = BRep_Tool::Triangulation(TopoDS::Face(mesh.wrapped()), loc);
    CHECK(!triangulation.IsNull() && triangulation->NbTriangles() > 0);
    CHECK(mesh.bounding_box().size == part.bounding_box().size);

    CHECK(export_gltf(part, dir.file("part.glb"), Unit::MM, true));
    CHECK(fs::file_size(dir.path / "part.glb") > 0);
    CHECK(export_gltf(part, dir.file("part.gltf")));
    CHECK(fs::exists(dir.path / "part.bin"));

    CHECK_THROWS(import_step(dir.file("missing.step")), std::runtime_error);
    CHECK_THROWS(export_step(part, dir.file("no/such/dir/part.step")), std::runtime_error);
}

TEST(step_keeps_narrowing_cones) {
    // A cone narrowing along its axis and a circular edge's chamfer: conical faces of a
    // negative semi-angle, which OCCT 8.0.0's writer dropped without its shape processing.
    const TempDir dir;
    const Part cone = Cone(10, 5, 3);
    const Part c = Cylinder(10, 3);
    const Part chamfered = chamfer(c.edges().group_by(Axis::Z)[0], 0.6);
    for (const auto& [part, name] : {std::pair{cone, "cone.step"}, std::pair{chamfered, "chamfer.step"}}) {
        CHECK(export_step(part, dir.file(name)));
        const Shape back = import_step(dir.file(name));
        CHECK(back.faces().size() == part.faces().size());
        CHECK(back.solids().size() == 1);
        CHECK_NEAR(back.volume(), part.volume(), 1e-6 * part.volume());
    }
}

TEST(step_assembly_labels_and_colors) {
    const TempDir dir;
    Part red = Box(1, 1, 1);
    red.label = "red box";
    red.color = Color("red");
    Part blue = Pos(3, 0, 0) * Sphere(0.5);
    blue.label = "blue ball";
    blue.color = Color(0, 0, 1);
    const Compound assembly = Compound::make_assembly({red, blue}, "assembly");
    CHECK(export_step(assembly, dir.file("assembly.step")));

    // Labels come back cleaned as in Python's import_step: " .()" become "_"
    const Compound back = cast_shape<Compound>(import_step(dir.file("assembly.step")));
    CHECK(back.label == "assembly");
    CHECK(back.children.size() == 2);
    std::set<std::string> labels;
    for (const auto& child : back.children) {
        labels.insert(child.label);
        CHECK(child.color.has_value());
        if (child.label == "red_box") {
            CHECK_NEAR(child.color->red, 1, 1e-3);
            CHECK_NEAR(child.color->blue, 0, 1e-3);
            CHECK_NEAR(child.volume(), 1, 1e-6);
        } else {
            CHECK_NEAR(child.color->blue, 1, 1e-3);
            CHECK_NEAR(child.volume(), 4.0 / 3.0 * PI * 0.125, 1e-6);
            CHECK(child.bounding_box().center() == Vector(3, 0, 0));
        }
    }
    CHECK(labels == (std::set<std::string>{"red_box", "blue_ball"}));
}

TEST(warning_handler) {
    std::vector<std::string> seen;
    const WarningHandler previous = set_warning_handler([&](const std::string& m) { seen.push_back(m); });
    warn("first");
    set_warning_handler({});
    warn("silenced");
    set_warning_handler(previous);
    CHECK(seen == std::vector<std::string>{"first"});
}

int main(int argc, char** argv) {
    // The operation cache's gate (scripts/opcache_gate.py) runs the tests with it on.
    if (const char* dir = std::getenv("BUILD123D_OPCACHE_DIR")) enable_operation_cache({.folder = dir, .min_ms = 0});
    int failed = 0, run = 0;
    const std::set<std::string> only(argv + 1, argv + argc);
    for (const auto& test : testing::registry()) {
        if (!only.empty() && only.count(test.name) == 0) continue;
        ++run;
        try {
            test.body();
            std::cout << "ok      " << test.name << "\n";
        } catch (const Standard_Failure& e) {
            // Before OCCT 8, Standard_Failure doesn't derive from std::exception.
            ++failed;
#if OCC_VERSION_HEX >= 0x080000
            std::cout << "FAILED  " << test.name << ": OCCT " << e.what() << "\n";
#else
            std::cout << "FAILED  " << test.name << ": OCCT " << e.DynamicType()->Name() << "\n";
#endif
        } catch (const std::exception& e) {
            ++failed;
            std::cout << "FAILED  " << test.name << ": " << e.what() << "\n";
        }
    }
    std::cout << "\n" << run - failed << "/" << run << " tests passed\n";
    return failed ? 1 : 0;
}
