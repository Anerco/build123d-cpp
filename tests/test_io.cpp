// Unit tests for the C++ side of import/export, hidden-line views, meshes, serialization and
// viewers. Results are compared with Python build123d in tests/compare/cases_io.cpp.
#include <build123d/build123d.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "test_framework.hpp"

using namespace build123d;
namespace fs = std::filesystem;

namespace {

/// A fresh scratch directory under the system temp directory.
struct IoTempDir {
    fs::path path;
    explicit IoTempDir(const std::string& name) {
        path = fs::temp_directory_path() / ("build123d_test_io_" + name);
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~IoTempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    std::string file(const std::string& name) const { return (path / name).string(); }
};

}  // namespace

static_assert(std::is_same_v<decltype(Box(1, 1, 1).project_to_viewport(Vector(1, 1, 1))),
                             std::pair<ShapeList<Edge>, ShapeList<Edge>>>);

TEST(io_project_to_viewport) {
    const auto [visible, hidden] = Box(1, 2, 3).project_to_viewport(Vector(10, -10, 10));
    CHECK(visible.size() == 9);
    CHECK(hidden.size() == 3);
    // Every edge lies in the viewport plane
    for (const auto& e : visible) CHECK_NEAR(e.start_point().Z, 0.0, 1e-9);
    const auto [v2, h2] = Edge::make_line({0, 0, 0}, {1, 0, 0}).project_to_viewport(Vector(0, -5, 0));
    CHECK(v2.size() == 1);
    CHECK(h2.empty());
    CHECK_THROWS(Part().project_to_viewport(Vector(1, 1, 1)), std::invalid_argument);
}

TEST(io_json_values) {
    const Json j = Json::parse(R"({"a": [1, 2.5, "x"], "b": {"c": null}, "d": true})");
    CHECK(j["a"].at(0).type() == Json::Type::INT);
    CHECK(j["a"].at(1).type() == Json::Type::FLOAT);
    CHECK(j["a"].at(1).as_double() == 2.5);
    CHECK(j["a"].at(2).as_string() == "x");
    CHECK(j["b"]["c"].is_null());
    CHECK(j["d"].as_bool());
    CHECK(j.contains("a") && !j.contains("z"));
    CHECK_THROWS(j["z"].is_null(), std::out_of_range);
    CHECK_THROWS(j["a"].at(5), std::out_of_range);
    CHECK_THROWS(j["d"].as_string(), std::invalid_argument);
    CHECK_THROWS(Json::parse("{\"a\": }"), std::invalid_argument);
    CHECK_THROWS(Json::parse("[1] 2"), std::invalid_argument);
    Json built;
    built["x"] = 1;
    built["y"].push_back(2.0);
    CHECK(built.dump() == R"({"x": 1, "y": [2.0]})");
    CHECK(Json::parse(built.dump()) == built);
}

static_assert(std::is_same_v<decltype(cast_shape<Solid>(Shape())), Solid>);
static_assert(std::is_same_v<decltype(cast_shape(Shape())), AnyShape>);
static_assert(std::is_same_v<decltype(from_bytes<Part>(Bytes())), Part>);

TEST(io_cast_shape) {
    Shape s(Box(1, 2, 3).solid().wrapped());
    s.label = "box";
    s.color = Color("red");
    const Solid solid = cast_shape<Solid>(s);
    CHECK(solid.label == "box" && solid.color && solid.color->red == 1.0);
    CHECK_NEAR(solid.volume(), 6.0, 1e-9);
    const Part part = cast_shape<Part>(s);
    CHECK(part.wrapped().ShapeType() == TopAbs_COMPOUND && part.label == "box");
    CHECK(std::holds_alternative<Solid>(cast_shape(s)));
    CHECK(std::holds_alternative<Compound>(cast_shape(Shape())));
    CHECK_THROWS(cast_shape<Face>(s), std::invalid_argument);
    CHECK_THROWS(cast_shape<Sketch>(s), std::invalid_argument);
    CHECK_THROWS(cast_shape<Curve>(Shape(Box(1, 1, 1).wrapped())), std::invalid_argument);
    const Mixin1D m = cast_shape<Mixin1D>(Shape(Edge::make_line({0, 0}, {1, 0}).wrapped()));
    CHECK_NEAR(m.length(), 1.0, 1e-12);
    CHECK(cast_shape<Sketch>(Shape((Circle(1) + Pos(3, 0) * Circle(1)).wrapped())).faces().size() == 2);
}

TEST(io_persistence) {
    // Payloads as Python's persistence.py
    const Part box = Box(1, 2, 3);
    const Bytes payload = serialize_shape(box.wrapped());
    CHECK(!payload.empty());
    CHECK_NEAR(Shape(deserialize_shape(payload)).volume(), 6.0, 1e-9);
    CHECK(serialize_shape(TopoDS_Shape()).empty());
    CHECK(deserialize_shape(Bytes()).IsNull());
    CHECK_THROWS(deserialize_shape(Bytes{1, 2, 3}), std::runtime_error);
    const Bytes loc_bytes = serialize_location(Location(Vector(1, 2, 3), Vector(0, 0, 90)).wrapped());
    CHECK(loc_bytes.size() == 28);
    const Location loc(deserialize_location(loc_bytes));
    CHECK(loc.position() == Vector(1, 2, 3));
    CHECK(loc.orientation() == Vector(0, 0, 90));
    CHECK_THROWS(deserialize_location(Bytes(10)), std::invalid_argument);

    // Whole objects, as pickle
    Compound assembly = Compound::make_assembly({Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)}, "asm");
    assembly.children[0].label = "kid";
    assembly.children[1].color = Color(0.1, 0.2, 0.3, 0.5);
    const Bytes data = to_bytes(assembly);
    const Compound copy = from_bytes<Compound>(data);
    CHECK(copy.label == "asm");
    CHECK(copy.children.size() == 2);
    CHECK(copy.children[0].label == "kid");
    CHECK(copy.children[1].color && copy.children[1].color->alpha == 0.5);
    CHECK_NEAR(copy.volume(), assembly.volume(), 1e-9);
    CHECK(from_bytes<Part>(to_bytes(box)).solids().size() == 1);
    CHECK_THROWS(from_bytes<Face>(to_bytes(box)), std::invalid_argument);
    const Face face = box.faces()[0];
    CHECK(from_bytes<Face>(to_bytes(face)).topo_parent().has_value());
    const Location l2 = from_bytes<Location>(to_bytes(Location(Vector(4, 5, 6))));
    CHECK(l2.position() == Vector(4, 5, 6));
    CHECK_THROWS(from_bytes<Location>(data), std::invalid_argument);
    CHECK_THROWS(shape_from_bytes(Bytes{1, 2, 3}), std::invalid_argument);
    Bytes truncated(data.begin(), data.begin() + 40);
    CHECK_THROWS(shape_from_bytes(truncated), std::invalid_argument);
}

TEST(io_geom_encoder) {
    const Location loc(Vector(1, 2, 3), Vector(10, 20, 30));
    const Geometry g = GeomEncoder::loads(GeomEncoder::dumps(loc));
    CHECK(std::holds_alternative<Location>(g));
    CHECK(std::get<Location>(g) == loc);
    const Plane p = std::get<Plane>(GeomEncoder::loads(GeomEncoder::dumps(Plane::XZ)));
    CHECK(p == Plane::XZ);
    const Axis a = std::get<Axis>(GeomEncoder::geometry_hook(Json::parse(GeomEncoder::dumps(Axis::Y))));
    CHECK(a == Axis::Y);
    const Color c = std::get<Color>(GeomEncoder::loads(R"({"Color": [0.1, 0.2, 0.3]})"));
    CHECK(c.alpha == 1.0 && c.blue == 0.3);
    CHECK(std::get<Vector>(GeomEncoder::loads(R"({"Vector": [1, 2, 3]})")) == Vector(1, 2, 3));
    CHECK_THROWS(GeomEncoder::loads(R"({"Vector": [1, 2, 3], "Axis": 1})"), std::invalid_argument);
    CHECK_THROWS(GeomEncoder::loads(R"({"Foo": [1]})"), std::out_of_range);
}
