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
