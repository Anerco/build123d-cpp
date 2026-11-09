// Unit tests for the C++ side of import/export, hidden-line views, meshes, serialization and
// viewers. Results are compared with Python build123d in tests/compare/cases_io.cpp.
#include <build123d/build123d.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
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

namespace {
// OCCT reports unreadable data on std::cout, as it does under Python; keep it out of the test output.
template <class F>
auto quiet_cout(F&& f) {
    std::ostringstream sink;
    std::streambuf* const old = std::cout.rdbuf(sink.rdbuf());
    struct Restore {
        std::streambuf* old;
        ~Restore() { std::cout.rdbuf(old); }
    } restore{old};
    return f();
}
}  // namespace

TEST(io_persistence) {
    // Payloads as Python's persistence.py
    const Part box = Box(1, 2, 3);
    const Bytes payload = serialize_shape(box.wrapped());
    CHECK(!payload.empty());
    CHECK_NEAR(Shape(deserialize_shape(payload)).volume(), 6.0, 1e-9);
    CHECK(serialize_shape(TopoDS_Shape()).empty());
    CHECK(deserialize_shape(Bytes()).IsNull());
    CHECK_THROWS(quiet_cout([] { return deserialize_shape(Bytes{1, 2, 3}); }), std::runtime_error);
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

static_assert(std::is_same_v<decltype(import_svg(std::string())), ShapeList<Shape>>);

TEST(io_import_svg) {
    const std::string svg = R"(<svg xmlns="http://www.w3.org/2000/svg" width="20" height="10">
        <rect id="a" width="10" height="5" fill="red"/>
        <path id="b" d="M0 0 L5 5" stroke="blue" fill="none"/></svg>)";
    std::istringstream in(svg);
    const ShapeList<Shape> shapes = import_svg(in);
    CHECK(shapes.size() == 2);
    CHECK(shapes[0].wrapped().ShapeType() == TopAbs_FACE);
    CHECK(shapes[0].label == "a");
    CHECK(shapes[0].color && shapes[0].color->red == 1.0 && shapes[0].color->green == 0.0);
    CHECK(shapes[1].wrapped().ShapeType() == TopAbs_WIRE);
    CHECK(shapes[1].color && shapes[1].color->blue == 1.0);
    CHECK_NEAR(cast_shape<Face>(shapes[0]).area(), 50.0, 1e-9);
    CHECK(shapes.faces().size() == 1);
    // y flipped, the viewbox aligned at the origin
    const BoundBox bb = shapes[0].bounding_box();
    CHECK_NEAR(bb.min.Y, 5.0, 1e-6);
    CHECK_NEAR(bb.max.Y, 10.0, 1e-6);
    std::istringstream in2(svg);
    const auto unflipped = import_svg(in2, {.flip_y = false, .align = Align::NONE, .label_by = "fill"});
    CHECK_NEAR(unflipped[0].bounding_box().max.Y, 5.0, 1e-6);
    CHECK(unflipped[0].label == "red");

    std::istringstream broken("<svg><rect");
    CHECK_THROWS(import_svg(broken), std::invalid_argument);
    std::istringstream no_d(R"(<svg xmlns="http://www.w3.org/2000/svg"><path/></svg>)");
    CHECK_THROWS(import_svg(no_d), std::invalid_argument);
    std::istringstream not_svg(R"(<g><rect width="1" height="1"/></g>)");
    CHECK_THROWS(import_svg(not_svg), std::invalid_argument);
    CHECK_THROWS(import_svg("/nonexistent/drawing.svg"), std::runtime_error);

    const IoTempDir dir("svg");
    {
        std::ofstream(dir.file("my_line.svg")) << R"(<svg><path d="M 0 0 L 10 0 A 5 5 0 0 1 20 0"/></svg>)";
    }
    const auto [code, builder] = import_svg_as_buildline_code(dir.file("my_line.svg"));
    CHECK(builder == "my_line");
    CHECK(code == "from build123d import *\nwith BuildLine() as my_line:\n    Line((0, 0),(10, 0))\n"
                  "    RadiusArc((10, 0), (20, 0), 5, True)");
}

static_assert(std::is_same_v<decltype(import_dxf(std::string())), ShapeList<Shape>>);

TEST(io_import_dxf) {
    const IoTempDir dir("dxf");
    ExportDXF exporter;
    exporter.add_shape(Edge::make_line({0, 0}, {10, 0}));
    exporter.add_shape(Edge::make_circle(2));
    exporter.add_shape(Pos(5, 5) * Rectangle(2, 2));
    exporter.write(dir.file("shapes.dxf"));
    const ShapeList<Shape> shapes = import_dxf(dir.file("shapes.dxf"));
    CHECK(shapes.size() == 6);
    CHECK(shapes[0].wrapped().ShapeType() == TopAbs_EDGE);
    CHECK_NEAR(cast_shape<Edge>(shapes[0]).length(), 10.0, 1e-9);
    CHECK(cast_shape<Edge>(shapes[1]).geom_type() == GeomType::CIRCLE);
    CHECK(shapes.edges().size() == 6);

    std::istringstream stream(
        "  0\nSECTION\n  2\nENTITIES\n  0\nPOINT\n 10\n1\n 20\n2\n 30\n3\n  0\nXLINE\n 10\n0\n 20\n0\n 30\n0\n"
        "  0\nENDSEC\n  0\nEOF\n");
    std::vector<std::string> warnings;
    const auto previous = set_warning_handler([&](const std::string& m) { warnings.push_back(m); });
    const ShapeList<Shape> points = import_dxf(stream);
    set_warning_handler(previous);
    CHECK(points.size() == 1);
    CHECK(points[0].wrapped().ShapeType() == TopAbs_VERTEX);
    CHECK(warnings == std::vector<std::string>{"Unable to convert XLINE"});

    std::istringstream broken("  0\nSECTION\n  2\nENTITIES\n");
    CHECK_THROWS(import_dxf(broken), std::invalid_argument);
    CHECK_THROWS(import_dxf(dir.file("missing.dxf")), std::runtime_error);
    {
        std::ofstream(dir.file("not.dxf")) << "hello\nworld\n";
    }
    CHECK_THROWS(import_dxf(dir.file("not.dxf")), std::runtime_error);
}

static_assert(!std::is_copy_constructible_v<Mesher>);
static_assert(std::is_move_constructible_v<Mesher>);

TEST(io_mesher) {
    const IoTempDir dir("mesher");
    Mesher m(Unit::CM);
    CHECK(m.model_unit() == Unit::CM);
    CHECK(m.library_version().find('.') != std::string::npos);
    Part box = Box(1, 2, 3);
    box.label = "box";
    m.add_shape(box);
    const Part ball = Pos(5, 0, 0) * Sphere(1);
    m.add_shape(ball.solids(), 0.1, 0.3, MeshType::SUPPORT);  // a ShapeList<Solid>
    CHECK(m.mesh_count() == 2);
    CHECK(m.triangle_counts().size() == 2 && m.triangle_counts()[0] == 12);
    CHECK(m.vertex_counts()[0] == 8);
    m.add_code_to_metadata();
    const auto meta = m.get_meta_data();
    CHECK(meta.size() == 1);
    CHECK(meta[0].name_space == "build123d" && meta[0].name == "test_io.cpp" && meta[0].type == "cpp");
    CHECK(meta[0].value.find("TEST(io_mesher)") != std::string::npos);
    CHECK(m.get_meta_data_by_key("build123d", "test_io.cpp").type == "cpp");
    CHECK_THROWS(m.get_meta_data_by_key("build123d", "nothing"), std::runtime_error);
    const auto properties = m.get_mesh_properties();
    CHECK(properties[0].name == "box" && properties[1].type == MeshType::SUPPORT);
    CHECK(properties[1].uuid.size() == 36);

    m.write(dir.file("m.3mf"));
    Mesher moved = std::move(m);
    moved.write(dir.file("m.stl"));
    CHECK_THROWS(moved.write(dir.file("m.obj")), std::invalid_argument);
    std::ostringstream stream;
    moved.write_stream(stream, "3mf");
    CHECK(stream.str().compare(0, 2, "PK") == 0);  // a zip archive

    Mesher reader;
    const ShapeList<Shape> shapes = reader.read(dir.file("m.3mf"));
    CHECK(shapes.size() == 2);
    CHECK(reader.model_unit() == Unit::CM);
    CHECK(shapes[0].label == "box");
    CHECK_NEAR(cast_shape<Solid>(shapes[0]).volume(), 6.0, 1e-6);
    CHECK_THROWS(Mesher().read(dir.file("missing.3mf")), std::runtime_error);
    CHECK_THROWS(Mesher(Unit::LB), std::invalid_argument);
    CHECK_THROWS(Mesher().add_shape(Face::make_rect(1, 1)), std::runtime_error);
}

static_assert(std::is_same_v<decltype(detect_primitives(Shape())),
                             std::tuple<ShapeList<Face>, ShapeList<Face>, std::vector<std::string>>>);

TEST(io_detect_primitives) {
    const IoTempDir dir("brep");
    Mesher mesher;
    mesher.add_shape(Box(2, 2, 1) - Cylinder(0.5, 1), 0.01, 1);
    mesher.write(dir.file("plate.stl"));
    Mesher reader;
    const Shape mesh = reader.read(dir.file("plate.stl"))[0];
    const auto [primitives, leftovers, code] = detect_primitives(mesh);
    CHECK(primitives.size() == code.size());
    CHECK(primitives.filter_by(GeomType::PLANE).size() == 6);
    CHECK(primitives.filter_by(GeomType::CYLINDER).size() == 1);
    CHECK(leftovers.empty());
    CHECK(code[0].rfind("r0 = Plane.XY.offset(-0.5) * Pos(-1, -1) * Rectangle(2, 2, align=Align.MIN)", 0) == 0);
    const ShapeList<Face> cylinders = primitives.filter_by(GeomType::CYLINDER);
    CHECK_NEAR(cylinders[0].area(), 2 * 3.141592653589793 * 0.5 * 1, 1e-2);
    CHECK_THROWS(detect_primitives(Cylinder(1, 2)), std::out_of_range);  // circular edges: one vertex
}

// ---- PBR materials ----------------------------------------------------------------------

namespace {

/// Sets an environment variable for the scope of the object.
struct ScopedEnv {
    std::string name;
    std::optional<std::string> previous;
    ScopedEnv(const std::string& n, const std::string& value) : name(n) {
        if (const char* old = std::getenv(n.c_str())) previous = old;
        set(value);
    }
    ~ScopedEnv() {
        if (previous) {
            set(*previous);
        } else {
#ifdef _WIN32
            _putenv_s(name.c_str(), "");
#else
            unsetenv(name.c_str());
#endif
        }
    }
    void set(const std::string& value) const {
#ifdef _WIN32
        _putenv_s(name.c_str(), value.c_str());
#else
        setenv(name.c_str(), value.c_str(), 1);
#endif
    }
};

/// Collects the warnings issued during the scope of the object.
struct ScopedWarnings {
    std::vector<std::string> messages;
    WarningHandler previous;
    ScopedWarnings() : previous(set_warning_handler([this](const std::string& m) { messages.push_back(m); })) {}
    ~ScopedWarnings() { set_warning_handler(previous); }
    ScopedWarnings(const ScopedWarnings&) = delete;
    ScopedWarnings& operator=(const ScopedWarnings&) = delete;
};

std::string read_all(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

static_assert(!std::is_constructible_v<ColorSpec, bool>);
static_assert(std::is_same_v<decltype(FinishedMaterial("oak").pbr()), PbrProperties>);
static_assert(std::is_same_v<decltype(pbr_preset("wood", "oak").interpolated_color()), Color>);

TEST(io_pbr_material_api) {
    // core's API
    const FinishedMaterial alu = "aluminum";
    CHECK(alu.name() == "Alu_G6061_T6");
    CHECK_NEAR(alu.material.density, 2700.0, 1e-12);
    CHECK(!alu.color);
    CHECK_THROWS(resolve_material("unobtainium"), std::invalid_argument);
    CHECK(material_names().size() == 156);
    Part box = Box(1, 1, 1);
    box.material = "oak";
    CHECK(box.material->name() == "Hardwood_OAK");

    // the override is one object, shared by copies (one glTF material, as in Python)
    const auto look = std::make_shared<const PbrProperties>(pbr_preset("metal", "copper"));
    const FinishedMaterial a(alu.material, {.pbr = look});
    const FinishedMaterial b = a;
    CHECK(a.pbr_ptr() == look && b.pbr_ptr() == look);
    CHECK(a.repr() == "FinishedMaterial(Alu_G6061_T6, pbr=<override>)");
    // a computed look is a new object at each call
    CHECK(alu.pbr_ptr() != alu.pbr_ptr());
    CHECK(alu.pbr().id == alu.pbr().id);
    FinishedMaterial c = a;
    c.set_pbr(nullptr);
    CHECK(!c.pbr_override() && c.pbr().id == alu.pbr().id);

    // options and their checks
    CHECK_THROWS(FinishedMaterial(alu.material, finishes::brushed(), {.pbr = look}), std::invalid_argument);
    CHECK_THROWS(FinishedMaterial(alu.material, finishes::brushed(), {.process = processes::cast()}), std::invalid_argument);
    CHECK_THROWS(registry::factory("aluminum")({.color = "red"}), std::invalid_argument);
    CHECK(registry::factory("Alu_G7075_T6")({.density = 3000}).material.density == 3000);
    CHECK(FinishSpec(finishes::brushed()).repr().rfind("AppliedFinish(", 0) == 0);
    CHECK(FinishSpec{finishes::brushed()}.repr().rfind("[AppliedFinish(", 0) == 0);
    std::vector<std::string> warnings;
    const WarningHandler previous = set_warning_handler([&](const std::string& m) { warnings.push_back(m); });
    const FinishedMaterial deprecated = plastics::pla({.process = processes::Process::FDM});
    set_warning_handler(previous);
    CHECK(warnings.size() == 1 && deprecated.process && deprecated.process->process == processes::Process::FDM);
    CHECK(metals::ALU_MATERIALS.at(metals::Alu::G7075_T6).name == "Alu_G7075_T6");
    CHECK(ALL_MATERIALS.size() == 95);

    // the examples of the docs (without the textures, textured looks keep the color, with a warning)
    const ScopedWarnings quiet;
    Part part = Box(1, 1, 1);
    part.material = metals::aluminum(metals::Alu::G7075_T6, {.finish = finishes::anodize("black")});
    part.material = metals::stainless({.finish = FinishSpec{finishes::brushed({2, 2}, 45), finishes::pvd("gold")}});
    part.material = plastics::petg({.color = "#336699", .process = processes::fdm(0.1, 90)});
    part.material = glass::soda_lime({.thickness_mm = 3, .opacity = 0.4});
    part.material = wood::oak({.scale = {2, 2}, .rotation = 30});
    part.material = metals::custom_metal("Inconel", 8440, {.family = "stainless", .tensile_strength = Range(900, 1100)});
    CHECK(part.material->material.tensile_strength->max == 1100);
    const FinishedMaterial m(alu.material, finishes::brushed(), {.color = "red"});
    const FinishedMaterial n = registry::factory("Alu_G7075_T6")({.density = 2800});
    CHECK(m.repr().find("color='red'") != std::string::npos && n.material.density == 2800);
}

TEST(io_pbr_missing_textures) {
    IoTempDir dir("pbr_textures");
    const ScopedEnv env("BUILD123D_TEXTURE_DIR", (dir.path / "none").string());
    // a texture directory beside the library or in the source tree would still be found
    if (!pbr_preset("wood", "oak").maps_dir->empty() &&
        pbr_preset("wood", "oak").maps_dir->find("$BUILD123D_TEXTURE_DIR") == std::string::npos) {
        std::cout << "  (textures installed: skipping the missing-texture checks)\n";
        return;
    }
    // looks without textures need none
    Part plain = Box(1, 2, 3);
    plain.material = glass::soda_lime({.opacity = 0.5});
    CHECK(export_gltf(plain, dir.file("glass.gltf")));
    CHECK(read_all(dir.file("glass.gltf")).find("KHR_materials_transmission") != std::string::npos);
    Part metal = Box(1, 2, 3);
    metal.material = metals::aluminum();
    CHECK(export_gltf(metal, dir.file("metal.glb"), Unit::MM, true));
    // textured looks: the error tells how to get the textures
    Part oak = Box(1, 2, 3);
    oak.material.set_raw(wood::oak());
    bool named = false;
    try {
        export_gltf(oak, dir.file("oak.gltf"));
    } catch (const std::runtime_error& e) {
        const std::string what = e.what();
        named = what.find("scripts/fetch-textures.sh") != std::string::npos &&
                what.find("BUILD123D_TEXTURE_DIR") != std::string::npos;
    }
    CHECK(named);
    CHECK_THROWS(wood::oak().pbr().interpolate_color(), std::runtime_error);
    // the material setter: a catalog material without choices of its own still gets its
    // grade's color; another textured one keeps the shape's color, with a warning
    Part assigned = Box(1, 2, 3);
    assigned.material = wood::oak();
    CHECK(assigned.color && assigned.color->to_tuple() == (std::array<double, 4>{0.4745, 0.3294, 0.2039, 1.0}));
    const ScopedWarnings warnings;
    Part scaled = Box(1, 2, 3);
    scaled.color = Color("red");
    scaled.material = wood::oak({.scale = {2.0, 2.0}});
    CHECK(scaled.color == Color("red"));
    CHECK(warnings.messages.size() == 1 && warnings.messages[0].find("fetch-textures.sh") != std::string::npos);
}
