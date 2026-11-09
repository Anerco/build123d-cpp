// Import/export, hidden-line views, meshes, serialization and viewers, compared with
// cases_io_py.py. Files are written to build123d_compare_io/cpp in the system's temporary
// directory (the Python side writes to .../py), and each case reports what both sides read back.
#include "case_registry.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
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

std::string read_text(const std::string& path) {
    const Bytes b = read_bytes(path);
    return std::string(b.begin(), b.end());
}

const char* type_name(const Shape& s) {
    if (s.is_null()) return "Compound";
    switch (s.wrapped().ShapeType()) {
        case TopAbs_VERTEX: return "Vertex";
        case TopAbs_EDGE: return "Edge";
        case TopAbs_WIRE: return "Wire";
        case TopAbs_FACE: return "Face";
        case TopAbs_SHELL: return "Shell";
        case TopAbs_SOLID: return "Solid";
        default: return "Compound";
    }
}

V color_value(const std::optional<Color>& c) {
    if (!c) return V();
    return V::list({c->red, c->green, c->blue, c->alpha});
}

/// An imported tree: type, label, color, location, size and children.
V shape_tree(const Shape& s) {
    std::vector<V> kids;
    for (const auto& c : s.children) kids.push_back(shape_tree(c));
    const Location loc = s.is_null() ? Location() : s.location();
    return V::obj({{"type", type_name(s)},
                   {"label", s.label},
                   {"color", color_value(s.color)},
                   {"position", loc.position()},
                   {"orientation", loc.orientation()},
                   {"volume", s.is_null() ? 0.0 : s.volume()},
                   {"faces", static_cast<double>(s.is_null() ? 0 : s.faces().size())},
                   {"children", V::list(kids)}});
}

/// An OBJ file with its "f" lines sorted and its normals as numbers: OCCT 8.0.0 (here) and 8.0.1
/// (Python) order the triangles of some faces differently, which changes the float32 vertex
/// normals OCCT averages in the last digits. Vertices, UVs and the triangles are the same.
V obj_canonical(const std::string& text) {
    std::istringstream in(text);
    std::string line, others;
    std::vector<std::string> faces;
    std::vector<V> normals;
    while (std::getline(in, line)) {
        if (line.rfind("f ", 0) == 0) {
            faces.push_back(line);
        } else if (line.rfind("vn ", 0) == 0) {
            std::istringstream nums(line.substr(3));
            double x, y, z;
            nums >> x >> y >> z;
            normals.push_back(V::list({x, y, z}));
        } else {
            others += line + "\n";
        }
    }
    std::sort(faces.begin(), faces.end());
    std::string f;
    for (const auto& x : faces) f += x + "\n";
    return V::list({others, f, V::list(normals)});
}

/// A STEP file as its exact header and the sorted list of its entities with the #N references
/// blanked out: OCCT writes the color styles of assemblies in a different order on every run.
V step_canonical(const std::string& text) {
    const auto data_pos = text.find("DATA;\n");
    const std::string header = text.substr(0, data_pos);
    std::string data = text.substr(data_pos + 6);
    std::vector<std::string> entities;
    std::string current;
    for (std::size_t i = 0; i < data.size(); ++i) {
        const char c = data[i];
        if (c == '\n') {
            if (!current.empty() && current.back() == ';') {
                entities.push_back(current);
                current.clear();
            }
            while (i + 1 < data.size() && data[i + 1] == ' ') ++i;  // continuation indent
            continue;
        }
        if (c == '#' && i + 1 < data.size() && std::isdigit(static_cast<unsigned char>(data[i + 1]))) {
            current += '#';
            while (i + 1 < data.size() && std::isdigit(static_cast<unsigned char>(data[i + 1]))) ++i;
            continue;
        }
        current += c;
    }
    if (!current.empty()) entities.push_back(current);
    std::sort(entities.begin(), entities.end());
    return V::list({header, V::list_of(entities)});
}

V json_value(const Json& j) {
    switch (j.type()) {
        case Json::Type::NUL: return V();
        case Json::Type::BOOL: return V(j.as_bool());
        case Json::Type::INT:
        case Json::Type::FLOAT: return V(j.as_double());
        case Json::Type::STRING: return V(j.as_string());
        case Json::Type::ARRAY: {
            std::vector<V> items;
            for (const auto& x : j.as_array()) items.push_back(json_value(x));
            return V::list(items);
        }
        case Json::Type::OBJECT: {
            std::vector<std::pair<std::string, V>> fields;
            for (const auto& [k, v] : j.as_object()) fields.emplace_back(k, json_value(v));
            return V::obj(fields);
        }
    }
    return V();
}

/// The JSON chunk of a .glb file.
Json glb_json(const Bytes& glb) {
    auto u32 = [&](std::size_t at) {
        return static_cast<std::uint32_t>(glb[at]) | static_cast<std::uint32_t>(glb[at + 1]) << 8 |
               static_cast<std::uint32_t>(glb[at + 2]) << 16 | static_cast<std::uint32_t>(glb[at + 3]) << 24;
    };
    const std::uint32_t length = u32(12);
    return Json::parse(std::string(glb.begin() + 20, glb.begin() + 20 + length));
}

/// A nested assembly with labels and colors (the same as make_io_fixtures.py's).
Compound sample_assembly() {
    Part red = Box(2, 2, 2);
    red.label = "red box (v1.0)";
    red.color = Color("red");
    Part ball = Pos(4, 0, 5) * Sphere(1);
    ball.label = "ball\tone";
    ball.color = Color(0.2, 0.4, 0.6, 0.8);
    // (Not a moved assembly: Python deep-copies those, which perturbs colors and geometry)
    Compound inner = Compound::make_assembly({ball, Pos(0, 4, 5) * Cylinder(1, 2)}, "inner.asm");
    inner.color = Color("green");
    return Compound::make_assembly({red, inner}, "outer");
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
    // ---- STEP
    compare::add_value("io_step_text_box", [] {
        Part box = Box(1, 2, 3);
        box.label = "my box";
        export_step(box, cpp_file("box.step"), Unit::MM, true, PrecisionMode::AVERAGE, "2024-01-01T00:00:00");
        return V(read_text(cpp_file("box.step")));
    });
    compare::add_value("io_step_text_assembly", [] {
        export_step(sample_assembly(), cpp_file("assembly.step"), Unit::MM, true, PrecisionMode::AVERAGE,
                    "2024-01-01T00:00:00");
        return step_canonical(read_text(cpp_file("assembly.step")));
    });
    compare::add_value("io_step_text_options", [] {
        Part part = Cylinder(2, 5) - Box(1, 1, 5);
        part.color = Color(0.1, 0.5, 0.9);
        std::ostringstream stream;
        export_step(part, stream, Unit::IN, false, PrecisionMode::GREATEST, "2025-06-07T08:09:10");
        export_step(Edge::make_circle(3), cpp_file("circle.step"), Unit::M, true, PrecisionMode::LEAST,
                    "2025-06-07T08:09:10");
        return V::list({stream.str(), read_text(cpp_file("circle.step"))});
    });
    compare::add_value("io_step_import_py_assembly", [] { return shape_tree(import_step(fixture("py_assembly.step"))); });
    compare::add_value("io_step_import_cpp_assembly", [] {
        export_step(sample_assembly(), cpp_file("assembly2.step"));
        return shape_tree(import_step(cpp_file("assembly2.step")));
    });
    compare::add_value("io_step_import_face_colors", [] { return shape_tree(import_step(fixture("py_face_colors.step"))); });
    compare::add_value("io_step_import_inches", [] { return shape_tree(import_step(fixture("py_cone_in.step"))); });
    compare::add_value("io_step_import_single_solid", [] {
        Solid s = Solid::make_box(1, 2, 3);
        s.label = "solid one";
        s.color = Color("red");
        export_step(s, cpp_file("solid.step"));
        return shape_tree(import_step(cpp_file("solid.step")));
    });
    compare::add_value("io_step_import_missing", [] { return shape_tree(import_step(cpp_file("missing.step"))); });
    compare::add_value("io_step_empty", [] { return V(export_step(Part(), cpp_file("empty.step"))); });
    compare::add_value("io_step_missing_dir", [] { return V(export_step(Box(1, 1, 1), cpp_file("no/dir/x.step"))); });

    // ---- BREP
    compare::add_value("io_brep_text", [] {
        const Part part = Rot(10, 20, 30) * (Box(3, 3, 3) - Sphere(1.8));
        export_brep(part, cpp_file("part.brep"));
        std::ostringstream stream;
        export_brep(part, stream);
        return V::list({read_text(cpp_file("part.brep")), stream.str()});
    });
    compare::add_value("io_brep_import", [] {
        export_brep(Pos(1, 2, 3) * Face::make_rect(2, 3), cpp_file("face.brep"));
        return shape_tree(import_brep(cpp_file("face.brep")));
    });
    compare::add_value("io_brep_missing_dir", [] { return V(export_brep(Box(1, 1, 1), cpp_file("no/dir/x.brep"))); });
    compare::add_value("io_brep_import_missing", [] { return shape_tree(import_brep(cpp_file("missing.brep"))); });

    // ---- STL
    compare::add_value("io_stl_ascii", [] {
        export_stl(Box(1, 2, 3), cpp_file("box.stl"), 1e-3, 0.1, true);
        return V(read_text(cpp_file("box.stl")));
    });
    compare::add_value("io_stl_binary", [] {
        export_stl(Cylinder(2, 3), cpp_file("cyl.stl"), 0.05, 0.3);
        return V(hex(read_bytes(cpp_file("cyl.stl"))));
    });
    compare::add_value("io_stl_import_units", [] {
        export_stl(Box(1, 2, 3), cpp_file("unit_box.stl"));
        std::vector<V> out;
        for (Unit u : {Unit::MM, Unit::MC, Unit::CM, Unit::M, Unit::IN, Unit::FT}) {
            const Face f = import_stl(cpp_file("unit_box.stl"), u);
            const BoundBox bb = f.bounding_box();
            out.push_back(V::list({bb.min, bb.max}));
        }
        return V::list(out);
    });
    compare::add_value("io_stl_import_bad_unit", [] {
        export_stl(Box(1, 2, 3), cpp_file("unit_box2.stl"));
        import_stl(cpp_file("unit_box2.stl"), Unit::KG);
        return V("imported");
    });
    compare::add_value("io_stl_missing_dir", [] { return V(export_stl(Box(1, 1, 1), cpp_file("no/dir/x.stl"))); });

    // ---- OBJ
    compare::add_value("io_obj_uvs", [] {
        const Part part = Box(4, 3, 2) - Cylinder(1, 2);
        export_obj(part, cpp_file("part.obj"), 0.01, 0.2);
        return obj_canonical(read_text(cpp_file("part.obj")));
    });
    compare::add_value("io_obj_options", [] {
        const Part part = Sphere(2) + Pos(2, 0, 0) * Box(1, 1, 1);
        std::ostringstream no_uvs, no_atlas, gutter;
        export_obj(part, no_uvs, 0.05, 0.3, false);
        export_obj(part, no_atlas, 0.05, 0.3, true, false);
        export_obj(part, gutter, 0.05, 0.3, true, true, 0.02);
        return V::list({obj_canonical(no_uvs.str()), obj_canonical(no_atlas.str()), obj_canonical(gutter.str())});
    });
    compare::add_value("io_obj_sketch", [] {
        std::ostringstream out;
        export_obj(Circle(2) - Rectangle(1, 1), out, 0.01);
        return obj_canonical(out.str());
    });
    compare::add_value("io_obj_missing_dir", [] { return V(export_obj(Box(1, 1, 1), cpp_file("no/dir/x.obj"))); });

    // ---- glTF
    compare::add_value("io_gltf_assembly", [] {
        export_gltf(sample_assembly(), cpp_file("assembly.gltf"));
        return V::list({json_value(Json::parse(read_text(cpp_file("assembly.gltf")))),
                        static_cast<double>(read_bytes(cpp_file("assembly.bin")).size())});
    });
    compare::add_value("io_gltf_glb_options", [] {
        Part part = Rot(0, 0, 30) * (Cylinder(10, 5) - Box(4, 4, 5));
        part.label = "part";
        part.color = Color(0.25, 0.5, 0.75, 0.5);
        export_gltf(part, cpp_file("part.glb"), Unit::M, true, 0.01, 0.3);
        const Bytes glb = read_bytes(cpp_file("part.glb"));
        return V::list({json_value(glb_json(glb)), static_cast<double>(glb.size())});
    });
    compare::add_value("io_gltf_sketch_curve", [] {
        export_gltf(Circle(3) - Rectangle(1, 2), cpp_file("sketch.gltf"));
        return json_value(Json::parse(read_text(cpp_file("sketch.gltf"))));
    });
    compare::add_value("io_gltf_missing_dir", [] { return V(export_gltf(Box(1, 1, 1), cpp_file("no/dir/x.gltf"))); });

    compare::add_value("io_persist_location_round_trip", [] {
        const Location loc(deserialize_location(serialize_location(Location(Vector(1, 2, 3), Vector(10, 20, 30)).wrapped())));
        return V::obj({{"position", loc.position()}, {"orientation", loc.orientation()}});
    });
}

// ---- 2D export: SVG
namespace {

std::string svg_text(const ExportSVG& svg) { return svg.to_string(); }

}  // namespace

COMPARE_CASES {
    compare::add_value("io_svg_basic", [] {
        ExportSVG e;
        e.add_layer("hid", {.fill_color = std::nullopt, .line_color = Color(1, 0, 0), .line_type = LineType::HIDDEN});
        Sketch s = Circle(5) - Rectangle(2, 2);
        s.color = Color(0.1, 0.5, 0.9, 0.5);
        e.add_shape(s);
        e.add_shape(Line({0, 0}, {10, 5}), "hid");
        e.write(cpp_file("basic.svg"));
        return V(read_text(cpp_file("basic.svg")));
    });
    compare::add_value("io_svg_drawing", [] {
        const Part part = Box(10, 8, 4) - Pos(2, 0, 0) * Cylinder(2, 4) + Pos(-3, 0, 2) * Sphere(2);
        const Drawing view(part, {.look_from = Vector(1, -2, 1.5)});
        ExportSVG e({.unit = Unit::IN, .scale = 2, .margin = 5, .fit_to_stroke = false, .precision = 3,
                     .line_weight = 0.3});
        e.add_layer("hidden", {.line_color = Color("gray"), .line_weight = 0.2, .line_type = LineType::ISO_DOT});
        e.add_shape(view.visible_lines);
        e.add_shape(view.hidden_lines, "hidden");
        return V(svg_text(e));
    });
    compare::add_value("io_svg_viewport", [] {
        const Part part = Cylinder(5, 10) - Pos(0, 0, 5) * Box(3, 12, 4);
        const auto [visible, hidden] = part.project_to_viewport(Vector(20, -30, 25));
        ExportSVG e({.unit = Unit::CM, .dot_length = DotLength::QCAD_IMPERIAL});
        e.add_layer("hidden", {.line_type = LineType::DASHDOT2});
        e.add_layer("other", {.line_type = LineType::DIVIDE});
        e.add_shape(visible);
        e.add_shape(hidden, "hidden");
        e.add_shape(Edge::make_line({0, 0}, {1, 1}), "other");
        return V(svg_text(e));
    });
    compare::add_value("io_svg_curves", [] {
        ExportSVG e({.precision = 4, .dot_length = 0.05});
        e.add_layer("dots", {.line_type = LineType::DOT});
        e.add_shape(Edge::make_ellipse(5, 2, Plane::XY, 30, 250));
        e.add_shape(Edge::make_ellipse(3, 1, Plane::XY.rotated(Vector(0, 0, 25))));
        e.add_shape(Edge::make_circle(4, Plane::XY.offset(0).moved(Pos(10, 0)), 20, 300));
        e.add_shape(Edge::make_circle(2, Plane::XY.moved(Pos(-10, 0))));
        e.add_shape(Edge::make_circle(2, Plane::YX.moved(Pos(-10, 5)), 0, 120));
        e.add_shape(Edge::make_spline({{0, 0}, {3, 4}, {6, -1}, {9, 2}}), "dots");
        e.add_shape(Edge::make_bezier({{0, 5}, {2, 9}, {5, 5}}), "dots");
        e.add_shape(Wire::make_polygon({{0, -5}, {4, -5}, {4, -8}}, false), "dots");
        e.add_shape(Polyline({{-2, -2}, {-4, -6}, {-1, -7}}).edges()[0].reversed(), "dots");
        e.add_shape(Edge::make_helix(2, 4, 1).moved(Pos(15, 15)));
        return V(svg_text(e));
    });
    compare::add_value("io_svg_faces", [] {
        ExportSVG e({.fill_color = Color("yellow")});
        e.add_layer("holes", {.fill_color = std::nullopt});
        e.add_layer("wires", {.line_color = Color(0.0, 0.5, 0.0, 0.25)});
        Sketch holed = Rectangle(20, 10) - GridLocations(5, 3, 3, 2) * Circle(1) - Pos(8, 0) * Ellipse(1.5, 1);
        holed.color = Color("blue");
        e.add_shape(holed, "holes");
        e.add_shape(Pos(30, 0) * (Circle(4) + Pos(3, 0) * Rectangle(4, 4)), "", true);
        e.add_shape(Pos(0, 20) * SlotOverall(10, 3), "wires", true);
        Curve c = Pos(0, -20) * Curve(std::vector<Edge>(Wire::make_rect(6, 3).edges()));
        c.color = Color("red");
        e.add_shape(c, "wires");
        e.add_shape(Pos(20, 20) * Text("b3d", 6));
        return V(svg_text(e));
    });
    compare::add_value("io_svg_colors", [] {
        std::vector<V> out;
        for (const SvgColor& c : std::vector<SvgColor>{Color("red"), Color(0.2, 0.4, 0.6, 0.123456), "#0080FF",
                                                       std::array<double, 3>{255, 128, 0}, ColorIndex::GRAY,
                                                       ColorIndex::BLACK, std::nullopt, INHERIT_COLOR}) {
            ExportSVG e({.fill_color = c, .line_color = c});
            Sketch s = Rectangle(1, 1);
            s.color = Color(1, 1, 1);
            e.add_shape(s);
            e.add_shape(Edge::make_line({0, 0}, {1, 1}));
            out.push_back(V(svg_text(e)));
        }
        return V::list(out);
    });
    compare::add_value("io_svg_line_types", [] {
        std::vector<V> out;
        for (LineType lt : {LineType::CONTINUOUS, LineType::BORDER, LineType::CENTERX2, LineType::DOT2,
                            LineType::PHANTOM2, LineType::ISO_DASH, LineType::ISO_LONG_DASH_TRIPLE_DOT,
                            LineType::ISO_DOUBLE_DASH_TRIPLE_DOT}) {
            for (double dot : {0.0, 0.01, 0.2}) {
                ExportSVG e({.unit = Unit::IN, .scale = 0.5, .line_weight = 0.25, .line_type = lt, .dot_length = dot});
                e.add_shape(Edge::make_line({0, 0}, {1, 1}));
                out.push_back(V(svg_text(e)));
            }
        }
        return V::list(out);
    });
    compare::add_value("io_svg_non_planar", [] {
        ExportSVG e;
        e.add_shape(Box(1, 2, 3));
        return V(svg_text(e));
    });
    compare::add_value("io_svg_bad_unit", [] {
        ExportSVG e({.unit = Unit::M});
        return V("created");
    });
    compare::add_value("io_svg_duplicate_layer", [] {
        ExportSVG e;
        e.add_layer("a");
        e.add_layer("a");
        return V("added");
    });
    compare::add_value("io_svg_undefined_layer", [] {
        ExportSVG e;
        e.add_shape(Box(1, 1, 1), "nope");
        return V("added");
    });
    compare::add_value("io_svg_empty", [] {
        ExportSVG e;
        return V(svg_text(e));
    });
}

// ---- 2D export: DXF. The Python side reads the files written here and its own with ezdxf
// and reports their differences (cases_io_py.py dxf_diff); "same" means equivalent.
namespace {

/// The shapes of the DXF cases (cases_io_py.py dxf_shapes).
void add_dxf_shapes(ExportDXF& e, bool r12) {
    e.add_shape(Edge::make_line({0, 0}, {10, 5}), "L1");
    e.add_shape(Edge::make_circle(3));
    e.add_shape(Edge::make_circle(3, Plane::XY, 10, 100));
    e.add_shape(Edge::make_circle(3, Plane::XY.reverse(), 10, 100));
    e.add_shape(Pos(5, 5) * (Rectangle(4, 2) - Circle(0.5)), "L2");
    if (r12) return;
    e.add_shape(Edge::make_ellipse(4, 2));
    e.add_shape(Edge::make_ellipse(4, 2, Plane::XY.reverse(), 20, 200));
    e.add_shape(Edge::make_ellipse(3, 1, Plane::XY.rotated(Vector(0, 0, 30)).moved(Pos(-5, 2)), 0, 90));
    e.add_shape(Edge::make_spline({{0, 0}, {1, 2}, {3, 1}, {5, 4}}), "L2");
    e.add_shape(Edge::make_spline({{0, 0}, {1, 2}, {3, 1}}, {}, true), "L2");
    e.add_shape(Edge::make_bezier({{0, 0}, {1, 3}, {4, 0}}, {1, 2, 1}), "L2");
    e.add_shape(Edge::make_bezier({{0, -2}, {2, -5}, {4, -2}}), "L2");
}

V dxf_written(const std::string& name) { return V::list({name, "same"}); }

}  // namespace

COMPARE_CASES {
    for (const char* version : {"R12", "R2000", "R2004", "R2007", "R2010", "R2013", "R2018"}) {
        const std::string v = version;
        compare::add_value("io_dxf_version_" + v, [v] {
            ExportDXF e(v, Unit::MM, ColorIndex::BLUE, 0.35, LineType::CENTER);
            e.add_layer("L1", {.color = ColorIndex::RED, .line_weight = 0.5, .line_type = LineType::DASHED});
            e.add_layer("L2");
            add_dxf_shapes(e, v == "R12");
            e.write(cpp_file("version_" + v + ".dxf"));
            return dxf_written("version_" + v + ".dxf");
        });
    }
    compare::add_value("io_dxf_units", [] {
        std::vector<V> out;
        for (Unit u : {Unit::MC, Unit::MM, Unit::CM, Unit::M, Unit::IN, Unit::FT}) {
            ExportDXF e(ExportDXF::DXF2013, u, std::nullopt, std::nullopt, LineType::ISO_DASH_DOT);
            e.add_layer("dots", {.line_type = LineType::DOT2});
            e.add_shape(Edge::make_line({0, 0}, {1, 1}), "dots");
            const std::string name = "unit_" + std::to_string(static_cast<int>(u)) + ".dxf";
            e.write(cpp_file(name));
            out.push_back(dxf_written(name));
        }
        return V::list(out);
    });
    compare::add_value("io_dxf_binary", [] {
        std::vector<V> out;
        for (const char* v : {"R12", "R2004", "R2013"}) {
            ExportDXF e(v);
            e.add_layer("L1", {.color = ColorIndex::GREEN, .line_type = LineType::PHANTOM});
            e.add_layer("L2", {.line_weight = 0.13});
            add_dxf_shapes(e, std::string(v) == "R12");
            const std::string name = std::string("binary_") + v + ".dxf";
            e.write(cpp_file(name), false);
            out.push_back(dxf_written(name));
        }
        return V::list(out);
    });
    compare::add_value("io_dxf_text_hlr", [] {
        ExportDXF e(ExportDXF::DXF2018, Unit::IN);
        e.add_layer("hidden", {.line_type = LineType::HIDDEN});
        const Part part = Box(10, 8, 4) - Pos(2, 0, 0) * Cylinder(2, 4) + Pos(-3, 0, 2) * Sphere(2);
        const auto [visible, hidden] = part.project_to_viewport(Vector(20, -30, 25));
        e.add_shape(visible);
        e.add_shape(hidden, "hidden");
        e.add_shape(Pos(0, 20) * Text("b3d", 6));
        std::ostringstream stream;
        e.write(stream);
        std::ofstream(cpp_file("text_hlr.dxf"), std::ios::binary) << stream.str();
        return dxf_written("text_hlr.dxf");
    });
    compare::add_value("io_dxf_non_planar", [] {
        ExportDXF e;
        e.add_shape(Box(1, 2, 3));
        e.write(cpp_file("non_planar.dxf"));
        return dxf_written("non_planar.dxf");
    });
    compare::add_value("io_dxf_bad_unit", [] {
        ExportDXF e(ExportDXF::DXF2013, Unit::KG);
        return V("created");
    });
    compare::add_value("io_dxf_r12_ellipse", [] {
        ExportDXF e("R12");
        e.add_shape(Edge::make_ellipse(2, 1));
        return V("added");
    });
    compare::add_value("io_dxf_duplicate_layer", [] {
        ExportDXF e;
        e.add_layer("A");
        e.add_layer("a");
        return V("added");
    });
    compare::add_value("io_dxf_bad_version", [] {
        ExportDXF e("R14");
        return V("created");
    });
}

// ---- SVG import
namespace {

/// What import_svg returned: per shape its type, label, color, size and position.
V svg_shapes(const ShapeList<Shape>& shapes) {
    std::vector<V> out;
    for (const Shape& s : shapes) {
        double length = 0.0;
        std::vector<V> geom;
        const bool is_wire = s.wrapped().ShapeType() == TopAbs_WIRE;
        // Python's Wire.edges() follows the wire (BRepTools_WireExplorer)
        for (const Edge& e : is_wire ? Wire(s.wrapped()).edges() : s.edges()) {
            length += e.length();
            geom.push_back(V(enum_name(e.geom_type())));
        }
        const BoundBox bb = s.bounding_box();
        const bool face = s.wrapped().ShapeType() == TopAbs_FACE;
        V extra = face ? V(Face(s.wrapped()).normal_at())
                       : V::list({Wire(s.wrapped()).start_point(), Wire(s.wrapped()).end_point()});
        out.push_back(V::obj({{"type", type_name(s)},
                              {"label", s.label},
                              {"color", color_value(s.color)},
                              {"area", s.area()},
                              {"length", length},
                              {"edges", V::list(geom)},
                              {"bbox", V::list({bb.min, bb.max})},
                              {"position", s.location().position()},
                              {"normal_or_ends", extra}}));
    }
    return V::list(out);
}

}  // namespace

COMPARE_CASES {
    for (const char* name : {"svg_basic", "svg_inkscape", "svg_holes", "svg_edge"}) {
        const std::string n = name;
        compare::add_value("io_" + n + "_default", [n] { return svg_shapes(import_svg(fixture(n + ".svg"))); });
        compare::add_value("io_" + n + "_label_noalign", [n] {
            return svg_shapes(import_svg(fixture(n + ".svg"), {.align = Align::NONE, .label_by = "inkscape:label"}));
        });
        compare::add_value("io_" + n + "_noflip_center", [n] {
            return svg_shapes(import_svg(fixture(n + ".svg"),
                                         {.flip_y = false, .align = Align::CENTER, .ignore_visibility = true}));
        });
    }
    compare::add_value("io_svg_import_align_pair", [] {
        return svg_shapes(import_svg(fixture("svg_basic.svg"), {.align = Align2(Align::MAX, Align::CENTER), .label_by = "class"}));
    });
    compare::add_value("io_svg_import_stream", [] {
        std::ifstream in(fixture("svg_inkscape.svg"));
        return svg_shapes(import_svg(in, {.label_by = "tag"}));
    });
    compare::add_value("io_svg_import_missing", [] { return svg_shapes(import_svg(cpp_file("missing.svg"))); });
}

COMPARE_CASES {
    compare::add_value("io_svg_export_roundtrip", [] {
        return svg_shapes(import_svg(fixture("svg_export.svg"), {.label_by = "id"}));
    });
    compare::add_value("io_svg_export_roundtrip_layers", [] {
        return svg_shapes(import_svg(fixture("svg_export.svg"), {.flip_y = false, .label_by = "stroke-dasharray"}));
    });
    for (const char* name : {"svg_basic", "svg_inkscape", "svg_holes", "svg_edge", "svg_export"}) {
        const std::string n = name;
        compare::add_value("io_svg_buildline_" + n, [n] {
            const auto [code, builder] = import_svg_as_buildline_code(fixture(n + ".svg"));
            return V::list({code, builder});
        });
    }
    compare::add_value("io_svg_buildline_precision", [] {
        const auto [code, builder] = import_svg_as_buildline_code(fixture("svg_basic.svg"), 3);
        return V::list({code, builder});
    });
}

// ---- DXF import
#include "../../src/dxf_reader.hpp"

namespace {

/// What import_dxf returned: per shape its type, size, edges and bounding box (compounds
/// with many edges, i.e. text, are summarized).
V dxf_import_summary(const ShapeList<Shape>& shapes) {
    std::vector<V> out;
    for (const Shape& s : shapes) {
        const TopAbs_ShapeEnum t = s.wrapped().ShapeType();
        // Python's Wire.edges() follows the wire (BRepTools_WireExplorer)
        const ShapeList<Edge> edges = t == TopAbs_WIRE ? Wire(s.wrapped()).edges() : s.edges();
        const bool listed = t != TopAbs_COMPOUND || edges.size() <= 4;
        double length = 0.0;
        std::vector<V> edge_values;
        for (const Edge& e : edges) {
            length += e.length();
            if (listed) edge_values.push_back(V::list({enum_name(e.geom_type()), e.start_point(), e.end_point()}));
        }
        const BoundBox bb = s.bounding_box();
        out.push_back(V::obj({{"type", type_name(s)},
                              {"area", s.area()},
                              {"length", length},
                              {"edge_count", static_cast<double>(edges.size())},
                              {"edges", V::list(edge_values)},
                              {"faces", static_cast<double>(s.faces().size())},
                              {"vertices", static_cast<double>(s.vertices().size())},
                              {"bbox", V::list({bb.min, bb.max})}}));
    }
    return V::list(out);
}

ShapeList<Shape> import_dxf_stream(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Can't read " + path);
    return import_dxf(in);
}

}  // namespace

COMPARE_CASES {
    for (const char* name : {"dxf_entities", "dxf_hatch", "dxf_r12", "dxf_binary", "dxf_cp1252", "dxf_3dface",
                             "dxf_polyline3d"}) {
        const std::string n = name;
        compare::add_value("io_import_" + n, [n] { return dxf_import_summary(import_dxf(fixture(n + ".dxf"))); });
    }
    // A binary stream is read as Latin-1 text (build123d's BytesIO branch)
    for (const char* name : {"dxf_entities", "dxf_cp1252", "dxf_binary"}) {
        const std::string n = name;
        compare::add_value("io_import_" + n + "_stream",
                           [n] { return dxf_import_summary(import_dxf_stream(fixture(n + ".dxf"))); });
    }
    compare::add_value("io_dxf_crlf", [] {
        // the R12 fixture with \r\n line ends: universal newlines for files, not for streams
        std::string text = read_text(fixture("dxf_r12.dxf")), crlf;
        for (char c : text) crlf += c == '\n' ? std::string("\r\n") : std::string(1, c);
        std::ofstream(cpp_file("crlf.dxf"), std::ios::binary) << crlf;
        return V::list({dxf_import_summary(import_dxf(cpp_file("crlf.dxf"))), dxf_import_summary(import_dxf_stream(cpp_file("crlf.dxf")))});
    });
    compare::add_value("io_dxf_import_exported", [] {
        // ExportDXF's own files, read by both
        ExportDXF e(ExportDXF::DXF2013, Unit::MM, ColorIndex::BLUE, 0.35, LineType::CENTER);
        e.add_layer("L1", {.color = ColorIndex::RED, .line_weight = 0.5, .line_type = LineType::DASHED});
        e.add_layer("L2");
        add_dxf_shapes(e, false);
        e.write(cpp_file("export_import.dxf"));
        return dxf_import_summary(import_dxf(cpp_file("export_import.dxf")));
    });
    compare::add_value("io_dxf_import_exported_r12_binary", [] {
        ExportDXF e("R12");
        e.add_layer("L1");
        e.add_layer("L2");
        add_dxf_shapes(e, true);
        e.write(cpp_file("export_import_r12.dxf"), false);
        return dxf_import_summary(import_dxf(cpp_file("export_import_r12.dxf")));
    });
    compare::add_value("io_dxf_import_missing", [] { return dxf_import_summary(import_dxf(cpp_file("missing.dxf"))); });
    compare::add_value("io_dxf_import_not_dxf", [] { return dxf_import_summary(import_dxf(fixture("svg_basic.svg"))); });
    compare::add_value("io_dxf_import_no_eof", [] {
        std::ofstream(cpp_file("no_eof.dxf"), std::ios::binary)
            << "  0\nSECTION\n  2\nENTITIES\n  0\nLINE\n 10\n0\n 20\n0\n 30\n0\n 11\n1\n 21\n1\n 31\n0\n  0\nENDSEC\n";
        return dxf_import_summary(import_dxf(cpp_file("no_eof.dxf")));
    });
    compare::add_value("io_dxf_import_minimal", [] {
        // A hand-written file: no header, R12 style entities, an empty line value, comments
        std::ofstream(cpp_file("minimal.dxf"), std::ios::binary)
            << "999\ncomment\n  0\nSECTION\n  2\nENTITIES\n  0\nLINE\n  8\n\n 10\n1.5\n 20\n-2\n 11\n4\n 21\n2\n"
               "  0\nCIRCLE\n 10\n3\n 20\n3\n 40\n2\n  0\nPOINT\n 10\n7\n 20\n8\n 30\n9\n  0\nARC\n 10\n0\n 20\n0\n"
               " 40\n1\n 50\n10\n 51\n10\n  0\nENDSEC\n  0\nEOF\n";
        return dxf_import_summary(import_dxf(cpp_file("minimal.dxf")));
    });
    compare::add_value("io_dxf_plain_mtext", [] {
        std::vector<V> out;
        for (const char* text : {"Plain {\\C1;red} 50%%d", "x\\S1^2;y", "a\\Pb\\Nc\\Ld\\le", "\\\\back\\{\\}",
                                 "%%c%%p%%d%%%%%%x", "unterminated \\fArial", "^Jcaret^I^", "tail\\",
                                 "{\\H2.5x;\\fArial|b1;Bold}", "100%", "%%", "%%%", "\\A1;aligned", "\\S1#2;",
                                 "\\S;empty", "\\;semi", "\xc3\xa9\xe2\x82\xac\xc3\xbc^\xc3\xa9", "\\Sab", "%%C%%D"}) {
            out.push_back(V(dxf::fast_plain_mtext(text)));
        }
        return V::list(out);
    });
}

// ---- Mesher (3MF, STL). The Python side compares the files written here with its own
// (cases_io_py.py mesh_file_diff); "same" means equivalent.
namespace {

constexpr const char* kUserUuid = "12345678-1234-5678-1234-567812345678";

/// Python's Unit.name and MeshType.name.
std::string unit_name(Unit u) {
    switch (u) {
        case Unit::MC: return "MC";
        case Unit::MM: return "MM";
        case Unit::CM: return "CM";
        case Unit::M: return "M";
        case Unit::IN: return "IN";
        case Unit::FT: return "FT";
        default: return "?";
    }
}

std::string mesh_type_name(MeshType t) {
    switch (t) {
        case MeshType::OTHER: return "OTHER";
        case MeshType::MODEL: return "MODEL";
        case MeshType::SUPPORT: return "SUPPORT";
        case MeshType::SOLIDSUPPORT: return "SOLIDSUPPORT";
    }
    return "?";
}

V mesh_properties(const Mesher& m) {
    std::vector<V> out;
    for (const MeshProperties& p : m.get_mesh_properties()) {
        // lib3mf makes up random UUIDs for objects without one
        const V uuid = p.uuid == kUserUuid ? V(p.uuid) : V(static_cast<double>(p.uuid.size()));
        out.push_back(V::obj({{"name", p.name}, {"part_number", p.part_number},
                              {"type", mesh_type_name(p.type)}, {"uuid", uuid}}));
    }
    return V::list(out);
}

V mesher_state(const Mesher& m) {
    std::vector<V> triangles, vertices, meta;
    for (int c : m.triangle_counts()) triangles.push_back(V(static_cast<double>(c)));
    for (int c : m.vertex_counts()) vertices.push_back(V(static_cast<double>(c)));
    for (const MeshMetaData& d : m.get_meta_data()) meta.push_back(V::list({d.name_space, d.name, d.type, d.value}));
    return V::obj({{"unit", unit_name(m.model_unit())},
                   {"mesh_count", static_cast<double>(m.mesh_count())},
                   {"triangle_counts", V::list(triangles)},
                   {"vertex_counts", V::list(vertices)},
                   {"properties", mesh_properties(m)},
                   {"meta_data", V::list(meta)}});
}

/// Shapes read from a mesh file.
V mesh_shapes(const ShapeList<Shape>& shapes) {
    std::vector<V> out;
    for (const Shape& s : shapes) {
        const BoundBox bb = s.bounding_box();
        out.push_back(V::obj({{"type", type_name(s)},
                              {"label", s.label},
                              {"color", color_value(s.color)},
                              {"volume", s.volume()},
                              {"area", s.area()},
                              {"faces", static_cast<double>(s.faces().size())},
                              {"shells", static_cast<double>(s.shells().size())},
                              {"bbox", V::list({bb.min, bb.max})}}));
    }
    return V::list(out);
}

/// The Mesher of the file comparisons (cases_io_py.py sample_mesher).
Mesher sample_mesher(Unit unit = Unit::MM) {
    Mesher m(unit);
    Part box = Box(2, 3, 4);
    box.label = "box";
    box.color = Color("red");
    m.add_shape(box, 0.001, 0.1, MeshType::MODEL, "P-1", kUserUuid);
    m.add_shape(Pos(5, 0, 0) * Sphere(1), 0.5, 0.5, MeshType::SUPPORT);
    Part tube = Pos(0, 6, 0) * (Cylinder(2, 3) - Cylinder(1, 3));
    tube.color = Color(0.1, 0.2, 0.3, 0.4);
    m.add_shape(tube, 0.05, 0.2, MeshType::SOLIDSUPPORT, "T-2");
    m.add_meta_data("build123d", "note", "hello <&> \"world\"", "string", true);
    m.add_meta_data("other", "count", "3", "xs:integer", false);
    return m;
}

V mesh_file(const std::string& name) { return V::list({name, "same"}); }

}  // namespace

COMPARE_CASES {
    compare::add_value("io_mesher_state", [] {
        const Mesher m = sample_mesher();
        const MeshMetaDataValue v = m.get_meta_data_by_key("build123d", "note");
        return V::list({mesher_state(m), V::list({v.type, v.value})});
    });
    compare::add_value("io_mesher_3mf_file", [] {
        sample_mesher().write(cpp_file("sample.3mf"));
        return mesh_file("sample.3mf");
    });
    compare::add_value("io_mesher_stl_file", [] {
        sample_mesher().write(cpp_file("sample.stl"));
        return mesh_file("sample.stl");
    });
    compare::add_value("io_mesher_stream", [] {
        const Mesher m = sample_mesher(Unit::IN);
        std::ostringstream three_mf, stl;
        m.write_stream(three_mf, "3mf");
        m.write_stream(stl, "stl");
        std::ofstream(cpp_file("stream.3mf"), std::ios::binary) << three_mf.str();
        std::ofstream(cpp_file("stream.stl"), std::ios::binary) << stl.str();
        return V::list({mesh_file("stream.3mf"), mesh_file("stream.stl")});
    });
    compare::add_value("io_mesher_read_cpp", [] {
        // Python reads the C++ file, C++ reads it too
        sample_mesher(Unit::CM).write(cpp_file("read.3mf"));
        Mesher reader;
        const ShapeList<Shape> shapes = reader.read(cpp_file("read.3mf"));
        return V::list({mesh_shapes(shapes), mesher_state(reader)});
    });
    compare::add_value("io_mesher_read_py", [] {
        Mesher reader;
        const ShapeList<Shape> shapes = reader.read(fixture("py_mesher.3mf"));
        return V::list({mesh_shapes(shapes), mesher_state(reader)});
    });
    compare::add_value("io_mesher_read_stl", [] {
        Mesher reader, open_reader;
        return V::list({mesh_shapes(reader.read(fixture("py_mesher.stl"))), unit_name(reader.model_unit()),
                        mesh_shapes(open_reader.read(fixture("open_pyramid.stl")))});
    });
    compare::add_value("io_mesher_assemblies", [] {
        // Children of an assembly (taking its color when they have none), a labelled compound,
        // and the solids of a plain Part as separate meshes
        Part a = Box(1, 1, 1);
        a.label = "a";
        Part b = Pos(3, 0, 0) * Sphere(1);
        b.color = Color("blue");
        Compound assembly = Compound::make_assembly({a, b}, "asm");
        assembly.color = Color("green");
        Part labelled = Pos(0, 5, 0) * Box(1, 2, 3) + Pos(4, 5, 0) * Box(1, 1, 1);
        labelled.label = "labelled";
        const Part plain = Pos(0, -5, 0) * Box(1, 1, 1) + Pos(3, -5, 0) * Cylinder(1, 1);
        Mesher m;
        m.add_shape(std::vector<Shape>{assembly, labelled, plain}, 0.01, 0.2);
        m.write(cpp_file("assemblies.3mf"));
        Mesher reader;
        return V::list({mesher_state(m), mesh_file("assemblies.3mf"), mesh_shapes(reader.read(cpp_file("assemblies.3mf")))});
    });
    compare::add_value("io_mesher_degenerate", [] {
        Mesher m;
        m.add_shape(Edge::make_line({0, 0}, {1, 1}));
        m.add_shape(Box(1, 1, 1));
        return mesher_state(m);
    });
    compare::add_value("io_mesher_open_face", [] {
        Mesher m;
        m.add_shape(Face::make_rect(1, 1));
        return mesher_state(m);
    });
    compare::add_value("io_mesher_other_type", [] {
        // lib3mf reports meshes of type "other" as invalid
        Mesher m;
        m.add_shape(Box(1, 1, 1), 0.001, 0.1, MeshType::OTHER);
        return mesher_state(m);
    });
    compare::add_value("io_mesher_bad_unit", [] {
        const Mesher m(Unit::KG);
        return V("created");
    });
    compare::add_value("io_mesher_bad_extension", [] {
        sample_mesher().write(cpp_file("sample.obj"));
        return V("written");
    });
    compare::add_value("io_mesher_bad_read_extension", [] {
        Mesher m;
        m.read(fixture("py_mesher.3MF"));
        return V("read");
    });
    compare::add_value("io_mesher_bad_stream_type", [] {
        std::ostringstream out;
        sample_mesher().write_stream(out, "obj");
        return V("written");
    });
    compare::add_value("io_mesher_read_missing", [] {
        Mesher m;
        m.read(cpp_file("missing.3mf"));
        return V("read");
    });
}

// ---- detect_primitives (brep_from_stl.py) on meshes written by Python's Mesher
namespace {

Shape read_mesh(const std::string& name) {
    Mesher reader;
    return reader.read(fixture(name + ".stl"))[0];
}

V face_summary(const Face& f) {
    const BoundBox bb = f.bounding_box();
    return V::obj({{"type", enum_name(f.geom_type())},
                   {"area", f.area()},
                   {"center", f.center()},
                   {"bbox", V::list({bb.min, bb.max})}});
}

/// detect_primitives' result. The primitives whose code sorts last with equal keys (spheres,
/// "Pos(...)") are ordered by their fit residuals, which tie up to rounding noise of numpy's
/// least squares (LAPACK): that tail is compared sorted by its code.
V detect_summary(const ShapeList<Face>& primitives, const ShapeList<Face>& leftovers,
                 const std::vector<std::string>& code) {
    std::vector<std::string> names, bodies;
    for (const std::string& line : code) {
        const auto p = line.find(" = ");
        names.push_back(line.substr(0, p));
        bodies.push_back(line.substr(p + 3));
    }
    auto tail = [&bodies](std::size_t i) {
        return !(bodies[i].rfind("Plane.", 0) == 0 || bodies[i].rfind("Location", 0) == 0);
    };
    std::vector<std::size_t> order(bodies.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (tail(a) != tail(b)) return tail(b);
        return tail(a) && bodies[a] < bodies[b];
    });
    std::vector<V> faces, left, lines, name_values;
    for (const std::size_t i : order) {
        faces.push_back(face_summary(primitives[static_cast<std::ptrdiff_t>(i)]));
        lines.push_back(V(bodies[i]));
    }
    for (const std::string& name : names) name_values.push_back(V(name));
    for (const Face& f : leftovers) left.push_back(V(f.center()));
    return V::obj({{"primitives", V::list(faces)},
                   {"leftovers", V::list(left)},
                   {"names", V::list(name_values)},
                   {"code", V::list(lines)},
                   {"counts", V::list({static_cast<double>(primitives.size()), static_cast<double>(code.size())})}});
}

}  // namespace

COMPARE_CASES {
    for (const char* name : {"brep_cylinder", "brep_sphere", "brep_box", "brep_box_with_hole", "brep_tilted",
                             "brep_moved", "brep_cone"}) {
        const std::string n = name;
        compare::add_value("io_" + n + "_mesh", [n] {
            // the mesh both sides work on: its faces in order
            const Shape mesh = read_mesh(n);
            std::vector<V> centers;
            for (const Face& f : mesh.faces()) centers.push_back(V(f.center()));
            return V::obj({{"type", type_name(mesh)}, {"faces", static_cast<double>(mesh.faces().size())},
                           {"centers", V::list(centers)}});
        });
        compare::add_value("io_" + n + "_detect", [n] {
            const auto [primitives, leftovers, code] = detect_primitives(read_mesh(n));
            return detect_summary(primitives, leftovers, code);
        });
    }
}

COMPARE_CASES {
    // shapes that aren't triangle meshes
    compare::add_value("io_detect_primitives_brep_box", [] {
        const auto [primitives, leftovers, code] = detect_primitives(Pos(1, 2, 3) * Box(2, 3, 4));
        return detect_summary(primitives, leftovers, code);
    });
    compare::add_value("io_detect_primitives_brep_cylinder", [] {
        const auto [primitives, leftovers, code] = detect_primitives(Cylinder(1, 2));
        return detect_summary(primitives, leftovers, code);
    });
    compare::add_value("io_detect_primitives_empty", [] {
        const auto [primitives, leftovers, code] = detect_primitives(Compound());
        return detect_summary(primitives, leftovers, code);
    });
}

// ---- the color Python's material setter gives a shape (bd_materials PBR look)
#include "../../src/io_internal.hpp"
#include "../../src/pbr_colors.hpp"

COMPARE_CASES {
    // the look's color, computed (bd_materials + threejs_materials port)
    compare::add_value("io_material_pbr_colors", [] {
        std::vector<std::string> grades;
        for (const std::string& name : material_names()) grades.push_back(resolve_material(name).material.name);
        std::sort(grades.begin(), grades.end());
        grades.erase(std::unique(grades.begin(), grades.end()), grades.end());
        std::vector<V> out;
        for (const std::string& grade : grades) {
            const auto c = detail::python_color_tuple(resolve_material(grade).pbr().interpolated_color());
            out.push_back(V::list({grade, V::list({c[0], c[1], c[2], c[3]})}));
        }
        return V::list(out);
    });
    // the table the material setter reads (src/pbr_colors.inc)
    compare::add_value("io_material_pbr_table", [] {
        std::vector<V> out;
        for (const std::string& grade : detail::catalog_pbr_grades()) {
            const auto c = *detail::catalog_pbr_color(grade);
            out.push_back(V::list({grade, V::list({c[0], c[1], c[2], c[3]})}));
        }
        return V::list(out);
    });
}
