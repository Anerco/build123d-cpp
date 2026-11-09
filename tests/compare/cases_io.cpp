// Import/export, hidden-line views, meshes, serialization and viewers, compared with
// cases_io_py.py. Files are written to /tmp/opencode/scratch-io/compare/cpp (the Python
// side writes to .../py), and each case reports what both sides read back.
#include "case_registry.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace build123d;
using compare::V;

namespace {

namespace fs = std::filesystem;

/// Where the C++ side writes files that the Python side reads (C++ runs first).
std::string exchange_dir(const std::string& side) {
    const fs::path dir = fs::temp_directory_path() / "build123d_compare_io" / side;
    fs::create_directories(dir);
    return dir.string();
}
std::string cpp_file(const std::string& name) { return (fs::path(exchange_dir("cpp")) / name).string(); }

/// Files written by Python build123d, committed in tests/compare/data (make_io_fixtures.py).
std::string fixture(const std::string& name) {
    return (fs::path(__FILE__).parent_path() / "data" / name).string();
}

Bytes read_bytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Can't read " + path);
    return Bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void write_bytes(const std::string& path, const Bytes& data) {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
}

std::string hex(const Bytes& data) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    for (const auto b : data) {
        out += digits[b >> 4];
        out += digits[b & 15];
    }
    return out;
}

/// An enum value's name without its class ("LINE"), as values.py reports Python enums.
template <class E>
std::string enum_name(E value) {
    const std::string s = to_string(value);
    return s.substr(s.find('.') + 1);
}

V edge_summary(const Edge& e) {
    return V::list({enum_name(e.geom_type()), e.length(), e.start_point(), e.end_point()});
}

V edges_summary(const ShapeList<Edge>& edges) {
    std::vector<V> items;
    for (const auto& e : edges) items.push_back(edge_summary(e));
    return V::list(items);
}

V viewport(const std::pair<ShapeList<Edge>, ShapeList<Edge>>& vh) {
    return V::obj({{"visible", edges_summary(vh.first)}, {"hidden", edges_summary(vh.second)}});
}

}  // namespace

COMPARE_CASES {
    // ---- project_to_viewport
    compare::add_value("io_hlr_box", [] { return viewport(Box(1, 2, 3).project_to_viewport(Vector(10, -10, 10))); });
    compare::add_value("io_hlr_holed_part", [] {
        const Part part = Box(10, 10, 10) - Cylinder(3, 10);
        return viewport(part.project_to_viewport(Vector(-20, 15, 30), Vector(0, 1, 0)));
    });
    compare::add_value("io_hlr_look_at", [] {
        const Part part = Box(10, 10, 10) - Pos(2, 0, 0) * Sphere(4);
        return viewport(part.project_to_viewport(Vector(0, -50, 0), Vector(0, 0, 1), Vector(5, 0, 0)));
    });
    compare::add_value("io_hlr_perspective", [] {
        const Part part = Cylinder(5, 10) + Pos(0, 0, 5) * Box(12, 3, 3);
        return viewport(part.project_to_viewport(Vector(30, -40, 25), Vector(0, 0, 1), std::nullopt, 60.0));
    });
    compare::add_value("io_hlr_solid", [] {
        const Solid s = Solid::make_torus(10, 2);
        return viewport(s.project_to_viewport(Vector(5, -10, 20)));
    });
    compare::add_value("io_hlr_face", [] {
        const Face f = Face::make_rect(4, 2, Plane::XZ);
        return viewport(f.project_to_viewport(Vector(1, -10, 3), Vector(0, 0, 1)));
    });
    compare::add_value("io_hlr_shell", [] {
        const Shell s = Shell::extrude(Wire::make_circle(3), Vector(0, 0, 4));
        return viewport(s.project_to_viewport(Vector(10, 10, 10)));
    });
    compare::add_value("io_hlr_edge", [] {
        const Edge e = Edge::make_circle(5, Plane::XY, 0, 270);
        return viewport(e.project_to_viewport(Vector(3, -20, 10)));
    });
    compare::add_value("io_hlr_wire", [] {
        const Wire w = Wire::make_polygon({{0, 0, 0}, {5, 0, 0}, {5, 5, 2}, {0, 5, 0}});
        return viewport(w.project_to_viewport(Vector(-7, -12, 9)));
    });
    compare::add_value("io_hlr_sketch", [] {
        const Sketch s = Circle(5) - Rectangle(3, 3);
        return viewport(s.project_to_viewport(Vector(0, -10, 10)));
    });
    compare::add_value("io_hlr_zero_focus", [] {
        return viewport(Box(2, 3, 4).project_to_viewport(Vector(10, 10, 10), Vector(0, 0, 1), Vector(1, 0, 0), 0.0));
    });
    compare::add_value("io_hlr_empty", [] { return viewport(Part().project_to_viewport(Vector(1, 1, 1))); });

    // ---- GeomEncoder / JSON
    compare::add_value("io_json_encode", [] {
        Json data = Json::object({
            {"a", GeomEncoder::encode(Axis(Vector(1, 2, 3), Vector(0, 1, 0)))},
            {"c", GeomEncoder::encode(Color("red"))},
            {"c2", GeomEncoder::encode(Color(0.1, 0.2, 0.3, 0.4))},
            {"l", GeomEncoder::encode(Location(Vector(1, 2, 3), Vector(10, 20, 30)))},
            {"p", GeomEncoder::encode(Plane::XZ)},
            {"v", GeomEncoder::encode(Vector(1, 2.5, -0.0))},
            {"list", Json::array({1, 2.0, nullptr, true, "x\xc3\xa9\n\x7f\xf0\x9f\x98\x80\"\\/"})},
            {"nested", Json::object({{"empty", Json::object()}, {"e2", Json::array()}})},
        });
        return V::list({data.dump(), data.dump(2), data.dump(0)});
    });
    compare::add_value("io_json_floats", [] {
        Json floats = Json::array({0.1, 1e-05, 1e16, 1e15, 123456789012345678.0, 5e-324, -0.0, 1.0, 100.0, 0.0001,
                                   1.5e-7, 2.5, 1.0 / 3.0, 1e22, 1.7976931348623157e308, -2.5e-10, 12345.678,
                                   std::nan(""), HUGE_VAL, -HUGE_VAL, 0, -7, 9007199254740993LL});
        return V(floats.dump());
    });
    compare::add_value("io_json_dumps_each", [] {
        return V::list({GeomEncoder::dumps(Axis::Z), GeomEncoder::dumps(Color(0.5, 0.25, 1)),
                        GeomEncoder::dumps(Location(Vector(1, 2, 3), Vector(0, 0, 90))),
                        GeomEncoder::dumps(Plane::isometric, 4), GeomEncoder::dumps(Vector(1e-7, 3, 1e300))});
    });
    compare::add_value("io_json_decode", [] {
        std::vector<V> out;
        for (const char* text : {R"({"Axis": [[1.0, 2.0, 3.0], [0.0, 1.0, 0.0]]})", R"({"Color": [0.1, 0.2, 0.3]})",
                                 R"({"Location": [[1, 2, 3], [10, 20, 30]]})",
                                 R"({"Plane": [[0, 0, 1], [1, 0, 0], [0, -1, 0]]})", R"({"Vector": [1, 2]})"}) {
            const Geometry g = GeomEncoder::loads(text);
            out.push_back(V(GeomEncoder::encode(g).dump()));
        }
        return V::list(out);
    });
    compare::add_value("io_json_hook_two_keys", [] {
        GeomEncoder::loads(R"({"Vector": [1, 2, 3], "Axis": [[0, 0, 0], [0, 0, 1]]})");
        return V("decoded");
    });
    compare::add_value("io_json_hook_unknown", [] {
        GeomEncoder::loads(R"({"Foo": [1]})");
        return V("decoded");
    });
    compare::add_value("io_json_roundtrip", [] {
        const std::string text =
            "{\"b\": [1, 2.50, -0.0, 1E3, 1e-7, true, false, null, {\"x\": \"\\u00e9\\ud83d\\ude00\\n\\/\"}],"
            " \"a\": {}, \"b\": 3, \"nan\": NaN, \"inf\": -Infinity}";
        return V(Json::parse(text).dump());
    });
    compare::add_value("io_json_malformed", [] {
        Json::parse("[1, 2,]");
        return V("parsed");
    });

    // ---- persistence: BinTools payloads and locations, exchanged both ways
    compare::add_value("io_persist_shape_bytes", [] {
        return V::list({hex(serialize_shape(Box(1, 2, 3).wrapped())),
                        hex(serialize_shape((Box(4, 4, 4) - Cylinder(1, 4)).wrapped())),
                        hex(serialize_shape(Edge::make_circle(2).wrapped()))});
    });
    compare::add("io_persist_shape_to_py", [] {
        // Python reads this file with persistence.deserialize_shape
        const Part part = Box(10, 10, 10) - Pos(2, 0, 0) * Sphere(4) + Pos(0, 0, 8) * Cylinder(2, 6);
        write_bytes(cpp_file("persist_shape.bin"), serialize_shape(part.wrapped()));
        return part;
    });
    compare::add("io_persist_shape_from_py", [] {
        return Shape(deserialize_shape(read_bytes(fixture("persist_py_shape.bin"))));
    });
    compare::add("io_persist_meshed_from_py", [] {
        // A payload with triangulations
        const Shape s(deserialize_shape(read_bytes(fixture("persist_py_meshed.bin"))));
        return s;
    });
    compare::add_value("io_persist_location_bytes", [] {
        return V::list({hex(serialize_location(Location(Vector(1, 2, 3), Vector(10, 20, 30)).wrapped())),
                        hex(serialize_location(Location(Vector(-1.5, 1e-3, 7), Vector(0, 90, 45)).wrapped())),
                        hex(serialize_location(Location().wrapped()))});
    });
    compare::add_value("io_persist_location_from_py", [] {
        const Location loc(deserialize_location(read_bytes(fixture("persist_py_location.bin"))));
        return V::obj({{"position", loc.position()}, {"orientation", loc.orientation()}});
    });
    compare::add_value("io_persist_location_round_trip", [] {
        const Location loc(deserialize_location(serialize_location(Location(Vector(1, 2, 3), Vector(10, 20, 30)).wrapped())));
        return V::obj({{"position", loc.position()}, {"orientation", loc.orientation()}});
    });
}
