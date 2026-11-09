// Objects, operations, text and drafting compared with cases_objects_py.py.
#include "case_registry.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "../../src/optimize.hpp"
#include "../../src/voronoi.hpp"

using namespace build123d;
using compare::V;

namespace {

/// The enum member's name, like Python's Enum.name ("MIN" for Align::MIN).
std::string name(Align a) {
    const std::string s = to_string(a);
    return s.substr(s.find('.') + 1);
}
V align_values(const Align2& a) { return V::list({name(a.x), name(a.y)}); }
V align_values(const Align3& a) { return V::list({name(a.x), name(a.y), name(a.z)}); }

/// A custom sketch object written like Python's custom objects (examples/custom_sketch_objects.py).
class Club : public BaseSketchObject {
public:
    explicit Club(double size, double rotation = 0, std::optional<Align2> align = std::nullopt)
        : BaseSketchObject(club(size), rotation, align), club_size(size) {}
    double club_size;

private:
    static Sketch club(double size) {
        return Pos(0, size / 3) * Circle(size / 4) + Pos(size / 4, 0) * Circle(size / 4) +
               Pos(-size / 4, 0) * Circle(size / 4) + Pos(0, -size / 3) * Rectangle(size / 6, size / 2);
    }
};

/// A custom part object.
class Bracket : public BasePartObject {
public:
    Bracket(double size, const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt)
        : BasePartObject(Box(size, size, size / 4, Align::MIN) + Box(size / 4, size, size, Align::MIN), rotation,
                         align) {}
};

V location_values(const Location& l) { return V::list({l.position(), l.orientation()}); }

V locations_values(const std::vector<Location>& locs) {
    std::vector<V> out;
    for (const auto& l : locs) out.push_back(location_values(l));
    return V::list(out);
}

/// Length and end points of each edge, in order.
V edges_values(const std::vector<Edge>& edges) {
    std::vector<V> out;
    for (const auto& e : edges) out.push_back(V::list({e.length(), e.position_at(0), e.position_at(1)}));
    return V::list(out);
}

V edge_values(const Edge& e) { return V::list({e.length(), e.position_at(0), e.position_at(1)}); }
V optional_edge_values(const std::optional<Edge>& e) { return e ? edge_values(*e) : V(nullptr); }

/// Edge count, length, bounding box and end points of a curve.
V curve_values(const Mixin1D& c) {
    const BoundBox bb = c.bounding_box();
    return V::list({static_cast<double>(c.edges().size()), c.length(), bb.min, bb.max, c.position_at(0),
                    c.position_at(1), c.shape_type() == ShapeType::WIRE});
}
template <class L>
V curves_values(const L& curves) {
    std::vector<V> out;
    for (const auto& c : curves) out.push_back(curve_values(c));
    return V::list(out);
}
V points_values(const std::vector<Vector>& pts) { return V::list_of(pts); }

/// Points along three edges, as full_round samples them.
std::vector<detail::Point2> sample_points(const std::vector<Edge>& edges, int n = 100) {
    std::vector<detail::Point2> pts;
    for (const auto& e : edges) {
        for (int i = 0; i <= n; ++i) {
            const Vector v = e.position_at(static_cast<double>(i) / n);
            pts.push_back({v.X, v.Y});
        }
    }
    return pts;
}

/// The Voronoi vertices sorted by their coordinates (rounded to 9 digits).
V voronoi_values(const std::vector<detail::Point2>& pts) {
    auto verts = detail::voronoi_vertices(pts);
    std::sort(verts.begin(), verts.end(), [](const detail::Point2& a, const detail::Point2& b) {
        const double ax = detail::round_digits(a[0], 9), bx = detail::round_digits(b[0], 9);
        if (ax != bx) return ax < bx;
        return detail::round_digits(a[1], 9) < detail::round_digits(b[1], 9);
    });
    std::vector<V> out;
    for (const auto& v : verts) out.push_back(V::list({v[0], v[1]}));
    return V::list(out);
}

V nelder_mead_values(const std::function<double(const std::vector<double>&)>& f, const std::vector<double>& x0,
                     detail::NelderMeadOptions options) {
    const auto r = detail::minimize_nelder_mead(f, x0, options);
    return V::obj({{"x", V::list_of(r.x)}, {"fun", r.fun}, {"nit", r.nit}, {"nfev", r.nfev}, {"status", r.status}});
}

/// [[min, max], ...] of the shapes' bounding boxes.
template <class T>
V bbox_values(const std::vector<T>& shapes) {
    std::vector<V> out;
    for (const auto& s : shapes) {
        const BoundBox bb = s.bounding_box();
        out.push_back(V::list({bb.min, bb.max}));
    }
    return V::list(out);
}

V pack2d_values(const std::vector<std::pair<double, double>>& sizes) {
    std::vector<V> out;
    for (const auto& [x, y] : detail::pack2d(sizes)) out.push_back(V::list({x, y}));
    return V::list(out);
}

// ---- fonts

const std::string kDejaVu = "/usr/share/fonts/truetype/dejavu/";

std::string bundled_font() { return FontManager::bundled_path() + "/reliefsingleline/ReliefSingleLineCAD-Regular.ttf"; }

/// A font path, from reliefsingleline/ on for the bundled font (installed elsewhere in Python).
std::string font_path_value(const std::string& path) {
    const auto i = path.find("reliefsingleline/");
    return i == std::string::npos ? path : path.substr(i);
}

V font_info_values(const std::vector<FontInfo>& fonts) {
    std::vector<V> out;
    for (const auto& f : fonts) {
        std::vector<V> styles;
        for (const auto s : f.styles) {
            styles.emplace_back(s == FontStyle::REGULAR ? "REGULAR"
                                : s == FontStyle::BOLD  ? "BOLD"
                                : s == FontStyle::ITALIC ? "ITALIC"
                                                         : "BOLDITALIC");
        }
        out.push_back(V::list({f.name, V::list(styles)}));
    }
    return V::list(out);
}

Font_FontAspect aspect_of(FontStyle s) {
    return s == FontStyle::REGULAR ? Font_FA_Regular
           : s == FontStyle::BOLD  ? Font_FA_Bold
           : s == FontStyle::ITALIC ? Font_FA_Italic
                                    : Font_FA_BoldItalic;
}

V system_font_values(const Handle(Font_SystemFont)& font, FontStyle style) {
    return V::list({std::string(font->FontName().ToCString()),
                    font_path_value(font->FontPath(aspect_of(style)).ToCString()), font->FontFaceId(aspect_of(style))});
}

V top_level_values(const Compound& compound) {
    std::vector<V> out;
    for (const auto& s : compound.get_top_level_shapes()) {
        const double length = s.shape_type() == ShapeType::WIRE   ? Wire(s.wrapped()).length()
                              : s.shape_type() == ShapeType::EDGE ? Edge(s.wrapped()).length()
                                                                  : 0.0;
        out.push_back(V::list({to_string(s.shape_type()), s.edges().size(), length}));
    }
    return V::list(out);
}

std::string read_bytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

/// A TrueType collection of font files (their table offsets moved behind the collection header).
void write_collection(const std::vector<std::string>& files, const std::string& path) {
    const auto u32 = [](const std::string& d, std::size_t p) {
        return (std::uint32_t(std::uint8_t(d[p])) << 24) | (std::uint32_t(std::uint8_t(d[p + 1])) << 16) |
               (std::uint32_t(std::uint8_t(d[p + 2])) << 8) | std::uint32_t(std::uint8_t(d[p + 3]));
    };
    const auto put32 = [](std::string& d, std::size_t p, std::uint32_t v) {
        for (int k = 0; k < 4; ++k) d[p + k] = char((v >> (24 - 8 * k)) & 0xFF);
    };
    std::string body;
    std::vector<std::uint32_t> starts;
    const std::size_t header = 12 + 4 * files.size();
    for (const auto& file : files) {
        std::string font = read_bytes(file);
        const auto start = static_cast<std::uint32_t>(header + body.size());
        const std::size_t num_tables = (std::uint8_t(font[4]) << 8) | std::uint8_t(font[5]);
        for (std::size_t i = 0; i < num_tables; ++i) put32(font, 12 + 16 * i + 8, u32(font, 12 + 16 * i + 8) + start);
        starts.push_back(start);
        body += font;
        while (body.size() % 4) body.push_back('\0');
    }
    std::string out(header, '\0');
    out.replace(0, 4, "ttcf");
    put32(out, 4, 0x00010000);
    put32(out, 8, static_cast<std::uint32_t>(files.size()));
    for (std::size_t i = 0; i < starts.size(); ++i) put32(out, 12 + 4 * i, starts[i]);
    std::ofstream(path, std::ios::binary) << out << body;
}

std::string temp_file(const std::string& name) { return (std::filesystem::temp_directory_path() / name).string(); }

V tri_values(const Triangle& t) {
    return V::list({t.a, t.b, t.c, t.A, t.B, t.C, t.area(), V::list({t.vertex_A, t.vertex_B, t.vertex_C})});
}

// ---- drafting: the settings of upstream test_drafting.py

Draft metric() {
    return Draft({.font_size = 3.0, .font = "Arial", .font_style = FontStyle::REGULAR, .head_type = HeadType::CURVED,
                  .arrow_length = 3.0, .line_width = 0.25, .unit = Unit::MM, .number_display = NumberDisplay::DECIMAL,
                  .display_units = true, .decimal_precision = 2, .fractional_precision = 64, .extension_gap = 2.0});
}

/// create_test_sketch's outer face.
Sketch test_sketch_outer() {
    const Polyline l1({{10, 20}, {-20, 20}, {-20, -20}, {10, -20}});
    return make_face(l1 + RadiusArc(l1.position_at(0), l1.position_at(1), 25));
}

Sketch test_sketch_shape() {
    const Sketch outer = test_sketch_outer();
    return outer - offset(outer, -0.5);
}

/// (width, length) pairs drawn with random.seed(123456) as in upstream test_pack.py.
const std::vector<std::pair<double, double>> random_boxes{
    {10, 1}, {6, 1},   {3, 2},   {9, 1},   {4, 8},   {16, 6}, {1, 17},  {5, 13},  {12, 2},  {6, 2},
    {15, 17}, {15, 15}, {17, 6}, {17, 18}, {2, 4},   {3, 4},  {17, 3},  {3, 5},   {11, 19}, {10, 15},
    {17, 20}, {11, 12}, {9, 8},  {4, 9},   {9, 3},   {4, 7},  {15, 13}, {2, 1},   {16, 4},  {14, 4},
    {6, 13}, {8, 1},   {20, 17}, {14, 4},  {20, 17}, {8, 17}, {20, 17}, {4, 13},  {9, 19},  {2, 12},
    {14, 4}, {2, 17},  {8, 2},   {7, 6},   {18, 8},  {12, 12}, {7, 3},  {17, 11}, {18, 17}, {9, 19}};
const std::vector<std::pair<double, double>> random_slots{
    {11, 4}, {2, 1},  {7, 6},   {2, 1},   {4, 1},   {3, 1},  {10, 8},  {2, 1},   {5, 4},   {9, 2},
    {17, 6}, {7, 4},  {2, 1},   {18, 1},  {6, 5},   {14, 9}, {13, 7},  {3, 1},   {7, 5},   {3, 1},
    {16, 9}, {18, 17}, {16, 12}, {16, 15}, {18, 4}, {7, 6},  {18, 13}, {19, 9},  {3, 1},   {5, 3},
    {4, 2},  {5, 1},  {18, 2},  {4, 3},   {4, 3},   {6, 2},  {12, 11}, {20, 2},  {11, 4},  {16, 3},
    {18, 8}, {12, 6}, {13, 12}, {10, 6},  {9, 4},   {5, 1},  {10, 9},  {10, 6},  {4, 3},   {5, 3}};

}  // namespace

COMPARE_CASES {
    // ---- base classes for custom objects
    compare::add("obj_base_sketch", [] {
        return BaseSketchObject(Rectangle(4, 2) + Pos(2, 0) * Circle(1), 30, Align2(Align::MIN, Align::MAX));
    });
    compare::add("obj_base_sketch_plain", [] { return BaseSketchObject(Rectangle(4, 2).face()); });
    compare::add("obj_base_part", [] {
        return BasePartObject(Solid::make_box(1, 2, 3), Rotation(10, 20, 30), Align3(Align::MIN, Align::CENTER, Align::MAX));
    });
    compare::add("obj_base_part_plain", [] { return BasePartObject(Pos(1, 2, 3) * Cylinder(1, 2)); });
    compare::add("obj_base_curve", [] {
        return BaseCurveObject(Curve(std::vector<Edge>{Line({0, 0}, {1, 0}), Line({1, 0}, {1, 1})}));
    });
    compare::add("obj_base_line", [] { return BaseLineObject(Polyline({{0, 0}, {1, 0}, {1, 1}})); });
    compare::add("obj_base_edge", [] { return BaseEdgeObject(Edge::make_line({0, 0}, {3, 4})); });
    compare::add("obj_custom_club", [] { return Club(10, 45, Align2(Align::MIN, Align::MIN)); });
    compare::add("obj_custom_bracket", [] { return Bracket(4, Rotation(0, 0, 30), Align3(Align::CENTER)); });
    compare::add_value("obj_custom_attrs", [] {
        const Club c(10, 45);
        const Bracket b(4, Rotation(0, 0, 30));
        return V::obj({{"club_rotation", c.rotation}, {"bracket_rotation", b.rotation.orientation()}});
    });

    // ---- object attributes (vars() of each object)
    compare::add_value("obj_attrs_sketch", [] {
        const Circle circle(2, 90, Align::MIN);
        const Ellipse ellipse(3, 2, 15);
        const Polygon polygon({{0, 0}, {1, 0}, {1, 1}}, 10, Align::CENTER);
        const Rectangle rect(3, 2, 5);
        const RectangleRounded rr(3, 2, 0.5);
        const RegularPolygon rp(3, 6, false, 20);
        const SlotArc sa(CenterArc({0, 0}, 5, 0, 90), 1, 10);
        const SlotCenterPoint scp({0, 0}, {2, 0}, 1, 5);
        const SlotCenterToCenter sctc(3, 1, 7);
        const SlotOverall so(4, 1, 8);
        const Superellipse se(4, 2, 3, 12, 9);
        const Trapezoid tz(5, 2, 60, 0.0, 4);
        const Trapezoid tz2(5, 2, 60, 75.0);
        const Triangle tri({.a = 3, .b = 4, .c = 5}, Align::NONE, 12);
        std::vector<V> pts;
        for (const auto& p : polygon.pts) pts.emplace_back(p);
        return V::obj({
            {"circle", V::list({circle.radius, circle.arc_size, align_values(circle.align), circle.rotation})},
            {"ellipse", V::list({ellipse.x_radius, ellipse.y_radius, align_values(ellipse.align), ellipse.rotation})},
            {"polygon", V::list({V::list(pts), align_values(polygon.align), polygon.rotation})},
            {"rect", V::list({rect.width, rect.rectangle_height, align_values(rect.align), rect.rotation})},
            {"rect_rounded", V::list({rr.width, rr.rectangle_height, rr.radius, align_values(rr.align), rr.rotation})},
            {"regular_polygon",
             V::list({rp.radius, rp.apothem, rp.side_count, align_values(rp.align), rp.rotation})},
            {"slot_arc", V::list({sa.arc.wrapped().ShapeType() == TopAbs_EDGE, sa.slot_height, sa.rotation})},
            {"slot_center_point", V::list({scp.slot_center, scp.point, scp.slot_height, scp.rotation})},
            {"slot_center_to_center", V::list({sctc.center_separation, sctc.slot_height, sctc.rotation})},
            {"slot_overall", V::list({so.width, so.slot_height, so.rotation})},
            {"superellipse",
             V::list({se.width, se.height_, se.order, se.point_count, align_values(se.align), se.rotation})},
            {"trapezoid", V::list({tz.width, tz.trapezoid_height, tz.left_side_angle, tz.right_side_angle,
                                   align_values(tz.align), tz.rotation, tz2.right_side_angle})},
            {"triangle", V::list({tri.a, tri.b, tri.c, tri.A, tri.B, tri.C, tri.rotation})},
        });
    });
    compare::add_value("obj_attrs_part", [] {
        const Box box(1, 2, 3, Rotation(10, 0, 0));
        const Cone cone(2, 1, 3, 270, Rotation(), Align::MIN);
        const ConvexPolyhedron cp({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, Rotation(0, 0, 5));
        const CounterBoreHole cbh(1, 2, 1, 5);
        const CounterSinkHole csh(1, 2, 5, 90);
        const Cylinder cyl(1, 2, 180, Rotation(), Align3(Align::MIN, Align::CENTER, Align::MAX));
        const Hole hole(1, 5);
        const Sphere sphere(2, -45, 60, 270);
        const Torus torus(3, 1, 10, 300, 270);
        const Wedge wedge(1, 2, 3, 0, 0, 1, 1);
        return V::obj({
            {"box", V::list({box.length, box.width, box.box_height, box.rotation.orientation()})},
            {"cone", V::list({cone.bottom_radius, cone.top_radius, cone.cone_height, cone.arc_size,
                              align_values(cone.align)})},
            {"convex_polyhedron", V::list({cp.rotation.orientation()})},
            {"counter_bore_hole",
             V::list({cbh.radius, cbh.counter_bore_radius, cbh.counter_bore_depth, cbh.hole_depth})},
            {"counter_sink_hole",
             V::list({csh.radius, csh.counter_sink_radius, csh.hole_depth, csh.counter_sink_angle})},
            {"cylinder", V::list({cyl.radius, cyl.cylinder_height, cyl.arc_size, align_values(cyl.align)})},
            {"hole", V::list({hole.radius, hole.hole_depth})},
            {"sphere", V::list({sphere.radius, sphere.arc_size1, sphere.arc_size2, sphere.arc_size3,
                                align_values(sphere.align)})},
            {"torus", V::list({torus.major_radius, torus.minor_radius, torus.minor_start_angle,
                               torus.minor_end_angle, torus.major_angle, align_values(torus.align)})},
            {"wedge", V::list({wedge.xsize, wedge.ysize, wedge.zsize, wedge.xmin, wedge.zmin, wedge.xmax,
                               wedge.zmax, align_values(wedge.align)})},
        });
    });
    compare::add("obj_hole_no_depth", [] { return Hole(1); });
    compare::add("obj_counter_bore_no_depth", [] { return CounterBoreHole(1, 2, 1); });
    compare::add("obj_counter_sink_no_depth", [] { return CounterSinkHole(1, 2); });

    // ---- Locations
    compare::add_value("obj_locations_faces", [] {
        return locations_values(Locations((Rot(10, 20, 30) * Box(1, 2, 3)).faces()));
    });
    compare::add_value("obj_locations_faces_explicit", [] {
        return locations_values(Locations((Pos(1, 0, 0) * Cylinder(1, 2)).faces()));
    });
    compare::add_value("obj_locations_vertices", [] {
        return locations_values(Locations((Pos(1, 2, 3) * Box(1, 2, 3)).vertices()));
    });
    compare::add_value("obj_locations_planes", [] {
        return locations_values(Locations(std::vector<Plane>{Plane::XY, Plane::XZ.offset(2), Plane::isometric}));
    });
    compare::add_value("obj_locations_axes", [] {
        return locations_values(Locations(Axis::X, Axis(Vector(1, 2, 3), Vector(0, 1, 1))));
    });
    compare::add_value("obj_locations_axis_list", [] {
        return locations_values(Locations(std::vector<Axis>{Axis::Y, Axis::Z}));
    });
    compare::add_value("obj_locations_mixed", [] {
        const auto face = Box(1, 2, 3).faces().sort_by(Axis::X).last();
        return locations_values(Locations(Vector(1, 2), Pos(3, 4) * Rot(0, 0, 30), Axis::Z, Plane::YZ,
                                          Vertex(1, 1, 1), face, std::vector<Vector>{{5, 5}, {6, 6}}));
    });
    compare::add("obj_locations_faces_shape", [] {
        const Part box = Box(4, 4, 4);
        return box + Locations(box.faces()) * Cylinder(0.5, 1);
    });
    compare::add("obj_locations_mixed_shape", [] {
        return Sketch() + Locations(Vector(0, 0), Pos(5, 0) * Rot(0, 0, 45), Plane::XY.offset(1),
                                    Axis(Vector(-5, 0, 0), Vector(0, 0, 1))) *
                              Rectangle(2, 1);
    });
    compare::add_value("obj_location_list_attrs", [] {
        const GridLocations grid(2, 3, 3, 2, Align2(Align::MIN, Align::MAX));
        const HexLocations hex(1, 3, 2, true, Align::MIN);
        const PolarLocations polar(3, 4, 10, 180, false, true);
        const Locations locs(Vector(1, 2), Vector(3, 4));
        return V::obj({
            {"grid", V::list({grid.x_spacing, grid.y_spacing, grid.x_count, grid.y_count, align_values(grid.align),
                              grid.size, grid.min, grid.max, static_cast<double>(grid.planes.size()),
                              locations_values(grid.local_locations())})},
            {"hex", V::list({hex.radius, hex.apothem, hex.diagonal, hex.x_count, hex.y_count, hex.major_radius,
                             align_values(hex.align), locations_values(hex.local_locations())})},
            {"polar", locations_values(polar.local_locations())},
            {"locations", locations_values(locs.local_locations())},
        });
    });

    // ---- scipy's Nelder-Mead (used by DoubleTangentArc)
    compare::add_value("obj_nelder_mead", [] {
        using Opt = detail::NelderMeadOptions;
        Opt bounded;
        bounded.bounds = std::vector<std::pair<double, double>>{{0.0, 10.0}};
        bounded.xatol = bounded.fatol = 1e-6;
        Opt sine;
        sine.bounds = std::vector<std::pair<double, double>>{{0.0, 3.0}};
        sine.xatol = sine.fatol = 1e-6;
        Opt near_top;
        near_top.bounds = std::vector<std::pair<double, double>>{{0.0, 10.0}};
        Opt box2;
        box2.bounds = std::vector<std::pair<double, double>>{{0.0, 0.5}, {-1.0, 1.0}};
        box2.xatol = box2.fatol = 1e-8;
        Opt capped;
        capped.maxiter = 15;
        auto rosen = [](const std::vector<double>& x) {
            return 100 * std::pow(x[1] - x[0] * x[0], 2) + std::pow(1 - x[0], 2);
        };
        return V::list({
            nelder_mead_values([](const std::vector<double>& x) { return (x[0] - 3) * (x[0] - 3) + 1; }, {0.0}, bounded),
            nelder_mead_values([](const std::vector<double>& x) { return std::abs(std::sin(x[0]) - 0.5); }, {0.0}, sine),
            nelder_mead_values([](const std::vector<double>& x) { return (x[0] - 5) * (x[0] - 5); }, {9.99}, near_top),
            nelder_mead_values(rosen, {-1.2, 1.0}, Opt{}),
            nelder_mead_values(rosen, {0.4, 0.4}, box2),
            nelder_mead_values(rosen, {-1.2, 1.0}, capped),
        });
    });

    compare::add_value("obj_minimize_scalar_bounded", [] {
        auto run = [](const std::function<double(double)>& f, double lo, double hi, double xatol) {
            const auto r = detail::minimize_scalar_bounded(f, lo, hi, xatol);
            return V::list({r.x, r.fun, r.nfev, r.status});
        };
        return V::list({
            run([](double x) { return (x - 0.3) * (x - 0.3); }, 0, 1, 5e-7),
            run([](double x) { return std::abs(std::sin(5 * x) - 0.2); }, 0, 1, 1e-5),
            run([](double x) { return std::cos(x); }, 0.5, 0.75, 5e-7),
            run([](double x) { return std::exp(x); }, -2, 3, 1e-5),
        });
    });

    // ---- BlendCurve
    compare::add("obj_blend_c0", [] {
        return BlendCurve(Line({0, 0}, {1, 0}), Line({2, 0}, {2, 1}), ContinuityLevel::C0);
    });
    compare::add("obj_blend_c2_auto", [] {
        const Edge m1 = Pos(-10, 3) * CenterArc({0, 0}, 5, -10, 200).reversed();
        const Edge m2 = Pos(5, -13) * Spline({{-3, 9}, {0, 0}, {3, 9}});
        return BlendCurve(m1, m2);
    });
    compare::add_value("obj_blend_c1_c2_scalars", [] {
        const Edge m1 = Pos(-10, 3) * CenterArc({0, 0}, 5, -10, 200).reversed();
        const Edge m2 = Pos(5, -13) * Spline({{-3, 9}, {0, 0}, {3, 9}});
        std::vector<V> out;
        for (auto c : {ContinuityLevel::C1, ContinuityLevel::C2}) {
            const BlendCurve bc(m1, m2, c, std::make_pair(m1.position_at(1), m2.position_at(0)),
                                std::make_pair(1.7, 0.8));
            out.push_back(V::list({bc.length(), bc.derivative_at(0, 1), bc.derivative_at(1, 1), bc.derivative_at(0, 2),
                                   bc.derivative_at(1, 2), bc.position_at(0.3)}));
        }
        return V::list(out);
    });
    compare::add("obj_blend_bad_end_point", [] {
        const Edge m1 = CenterArc({0, 0}, 5, -10, 200);
        return BlendCurve(m1, Line({10, 0}, {12, 3}), ContinuityLevel::C1,
                          std::make_pair(m1.position_at(0.5), Vector(10, 0)));
    });

    // ---- DoubleTangentArc
    compare::add("obj_double_tangent_line", [] { return DoubleTangentArc({0, 0}, {1, 0}, Line({0, 5}, {10, 5})); });
    compare::add("obj_double_tangent_bottom", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Line({0, 5}, {10, 5}), Keep::BOTTOM);
    });
    compare::add("obj_double_tangent_wire", [] {
        return DoubleTangentArc({0, 0}, {1, 1}, Polyline({{6, 0}, {6, 8}, {10, 12}}));
    });
    compare::add("obj_double_tangent_arc_target", [] {
        return DoubleTangentArc({0, 0}, {0, 1}, CenterArc({10, 0}, 3, 90, 180));
    });
    compare::add("obj_double_tangent_spline", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Spline({{-5, 8}, {0, 6}, {6, 9}}), Keep::BOTTOM);
    });
    compare::add("obj_double_tangent_curve_error", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Curve(Line({0, 5}, {10, 5})));
    });
    compare::add("obj_double_tangent_keep_error", [] {
        return DoubleTangentArc({0, 0}, {1, 0}, Line({0, 5}, {10, 5}), Keep::BOTH);
    });

    // ---- Airfoil
    compare::add("obj_airfoil_2412", [] { return Airfoil("2412"); });
    compare::add("obj_airfoil_finite", [] { return Airfoil("NACA 2412", 40, true); });
    compare::add("obj_airfoil_0012", [] { return Airfoil("0012", 60); });
    compare::add("obj_airfoil_fraction", [] { return Airfoil("2213.323", 30); });
    compare::add("obj_airfoil_number", [] { return Airfoil(4415.0, 25, true); });
    compare::add("obj_airfoil_camber", [] { return Airfoil("4412", 40).camber_line(); });
    compare::add_value("obj_airfoil_attrs", [] {
        const Airfoil a("2412", 40);
        const auto [m, p, t] = Airfoil::parse_naca4("NACA 2213.323");
        const auto [m2, p2, t2] = Airfoil::parse_naca4(6409.0);
        return V::list({a.code, a.max_camber, a.camber_pos, a.thickness, a.finite_te, m, p, t, m2, p2, t2});
    });
    compare::add("obj_airfoil_bad_code", [] { return Airfoil("2x12"); });

    // ---- ConstrainedArcs / Edge::make_constrained_arcs
    compare::add_value("obj_carcs_2tan_rad_lines", [] {
        const Line e1({-2, 0}, {2, 0}), e2({0, -2}, {0, 2});
        return V::list({edges_values(Edge::make_constrained_arcs(e1, e2, {.radius = 0.5, .sagitta = Sagitta::BOTH})),
                        edges_values(Edge::make_constrained_arcs(e1, e2, {.radius = 0.5})),
                        edges_values(Edge::make_constrained_arcs(TangencyTarget(e1, Tangency::UNQUALIFIED),
                                                                 TangencyTarget(e2, Tangency::UNQUALIFIED),
                                                                 {.radius = 0.5, .sagitta = Sagitta::LONG}))});
    });
    compare::add_value("obj_carcs_2tan_rad_arc_line", [] {
        return edges_values(Edge::make_constrained_arcs(CenterArc({0, 0}, 1, 0, 90), Line({1, 0}, {2, 0}),
                                                        {.radius = 0.5}));
    });
    compare::add_value("obj_carcs_2tan_rad_points", [] {
        return V::list({edges_values(Edge::make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), {.radius = 0.5})),
                        edges_values(Edge::make_constrained_arcs(Vertex(0, 0), Vertex(0, 0.5), {.radius = 0.5})),
                        edges_values(Edge::make_constrained_arcs({0, 0.5}, Line({0, 0}, {1, 0}), {.radius = 0.5}))});
    });
    compare::add("obj_carcs_2tan_rad_none", [] {
        return Curve(Edge::make_constrained_arcs(Vector(0, 0), Vector(10, 0), {.radius = 2}));
    });
    compare::add_value("obj_carcs_2tan_on", [] {
        const Edge c1 = PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL);
        const Line c2({4, -2}, {4, 2}), on({3, -2}, {3, 2});
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(c1, Tangency::UNQUALIFIED),
                                                     TangencyTarget(c2, Tangency::UNQUALIFIED), {.center_on = on})),
            edges_values(Edge::make_constrained_arcs(Vector(0, 3), Vector(5, 0), {.center_on = Line({0, -5}, {0, 5})})),
            edges_values(Edge::make_constrained_arcs(Line({-5, 3}, {5, 3}), Vector(5, 0), {.center_on = Axis::Y})),
            edges_values(Edge::make_constrained_arcs(Line({-10, 0}, {10, 0}), Line({0, -10}, {0, 10}),
                                                     {.center_on = Line({-10, 1}, {10, 1}), .sagitta = Sagitta::BOTH})),
            edges_values(Edge::make_constrained_arcs(Line({-10, 0}, {10, 0}), Line({0, -10}, {0, 10}),
                                                     {.center_on = Line({3, -10}, {3, 10}), .sagitta = Sagitta::LONG})),
            edges_values(Edge::make_constrained_arcs(Line({0, 0}, {0.01, 0}), Line({10, -10}, {10, 10}),
                                                     {.center_on = Line({5, -10}, {5, 10})})),
        });
    });
    compare::add("obj_carcs_2tan_on_none", [] {
        return Curve(Edge::make_constrained_arcs(Line({-5, 3}, {5, 3}), Line({-5, 0}, {5, 0}),
                                                 {.center_on = Line({-5, -1}, {5, -1})}));
    });
    compare::add("obj_carcs_2tan_on_trim", [] {
        const CenterArc c1({0, 20}, 20, -90, 90);
        const Line ln2(c1.position_at(1), {10, 20});
        const Line ln3({10, 10}, ln2.position_at(1));
        return ConstrainedArcs(c1, ln2, {.center_on = ln3});
    });
    compare::add_value("obj_carcs_tan_cen", [] {
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(PolarLine({0, 0}, 4, 60), Tangency::UNQUALIFIED),
                                                     {.center = Vector(2, 1)})),
            edges_values(Edge::make_constrained_arcs(Axis::X, {.center = Vector(2, 1, 5)})),
            edges_values(Edge::make_constrained_arcs(CenterArc({0, 0}, 1, 180, 5), {.center = Vector(2, 0)})),
            edges_values(Edge::make_constrained_arcs({-2.5, 1.5}, {.center = Vector(-2, 1)})),
            edges_values(Edge::make_constrained_arcs({-2.5, 1.5}, {.center = Vertex(-2, 1)})),
            edges_values(Edge::make_constrained_arcs({-2.5, 1.5}, {.center = Pos(1, 0) * Vertex(-2, 1)})),
            edges_values(Edge::make_constrained_arcs(Vector(0, 0), {.center = Vector(0, 0)})),
        });
    });
    compare::add_value("obj_carcs_tan_on_rad", [] {
        const Edge c1 = PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL);
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(c1, Tangency::UNQUALIFIED),
                                                     {.radius = 1, .center_on = Line({3, -2}, {3, 2})})),
            edges_values(Edge::make_constrained_arcs(c1, {.radius = 1, .center_on = Axis::X})),
            edges_values(Edge::make_constrained_arcs(Vector(1, 1), {.radius = 2, .center_on = Axis::X})),
        });
    });
    compare::add("obj_carcs_tan_on_rad_none", [] {
        return Curve(Edge::make_constrained_arcs(Line({0, 10}, {10, 10}), {.radius = 1, .center_on = Axis::X}));
    });
    compare::add_value("obj_carcs_3tan", [] {
        const Edge c5 = PolarLine({0, 0}, 4, 60), c6 = PolarLine({0, 0}, 4, 40);
        const CenterArc c7({0, 0}, 4, 0, 90);
        const CenterArc a1({0, 20}, 20, -90, 90), a2({5, 25}, 10, -90, 90);
        return V::list({
            edges_values(Edge::make_constrained_arcs(TangencyTarget(c5, Tangency::UNQUALIFIED),
                                                     TangencyTarget(c6, Tangency::UNQUALIFIED),
                                                     TangencyTarget(c7, Tangency::UNQUALIFIED))),
            edges_values(Edge::make_constrained_arcs(c5, c6, c7, {.sagitta = Sagitta::BOTH})),
            edges_values(Edge::make_constrained_arcs(Line({-1, 0}, {-1, 2}), Line({1, 0}, {1, 2}),
                                                     Line({-1, 0}, {-0.75, 0}))),
            edges_values(Edge::make_constrained_arcs(a1, a2, Line(a1.position_at(1), a2.position_at(1)))),
            edges_values(Edge::make_constrained_arcs(a1, a2, RadiusArc(a1.position_at(1), a2.position_at(1), 10))),
        });
    });
    compare::add("obj_carcs_3tan_parallel", [] {
        return Curve(Edge::make_constrained_arcs(Line({0, 0}, {10, 0}), Line({0, 2}, {10, 2}), Line({0, 5}, {10, 5})));
    });
    compare::add("obj_carcs_eggplant", [] {
        const CenterArc left({-2, 0}, 0.75, 0, 360), right({2, 0}, 1.0, 0, 360);
        const Edge bottom = Edge::make_constrained_arcs(TangencyTarget(right, Tangency::OUTSIDE),
                                                        TangencyTarget(left, Tangency::OUTSIDE), {.radius = 6})
                                .sort_by(Axis::Y)[0];
        const Edge top = Edge::make_constrained_arcs(TangencyTarget(right, Tangency::ENCLOSING),
                                                     TangencyTarget(left, Tangency::ENCLOSING), {.radius = 8})
                             .sort_by(Axis::Y)[-1];
        const ThreePointArc r(bottom.vertices().sort_by(Axis::X)[-1], right.position_at(0),
                              top.vertices().sort_by(Axis::X)[-1]);
        const ThreePointArc l(bottom.vertices().sort_by(Axis::X)[0], left.position_at(0.5),
                              top.vertices().sort_by(Axis::X)[0]);
        return make_face(std::vector<Edge>{l, top, r, bottom});
    });
    compare::add("obj_carcs_object_selector", [] {
        return ConstrainedArcs(Edge::make_circle(1.0), Axis::Y, {.radius = 1},
                               [](const ShapeList<Edge>& arcs) { return arcs.group_by(Axis::Y)[-1]; });
    });
    compare::add("obj_carcs_object_all", [] {
        return ConstrainedArcs(CenterArc({-5, 0}, 4, 0, 360), CenterArc({5, 0}, 3, 0, 360), {.radius = 10});
    });
    compare::add("obj_carcs_object_first", [] {
        return ConstrainedArcs(PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL), Line({4, -2}, {4, 2}),
                               {.center_on = Axis(Vector(3, 0), Vector(0, 1))},
                               [](const ShapeList<Edge>& arcs) { return arcs[0]; });
    });
    compare::add("obj_carcs_object_shortest", [] {
        return ConstrainedArcs(PolarLine({0, 0}, 4, 60), PolarLine({0, 0}, 4, 40), CenterArc({0, 0}, 4, 0, 90),
                               {.sagitta = Sagitta::BOTH},
                               [](const ShapeList<Edge>& arcs) { return arcs.sort_by(SortBy::LENGTH)[0]; });
    });
    compare::add("obj_carcs_object_center", [] {
        return ConstrainedArcs(Axis::Y, {.center = Vector(-2, 1)}, [](const ShapeList<Edge>& arcs) { return arcs[0]; });
    });
    compare::add("obj_carcs_object_tan_on_rad", [] {
        return ConstrainedArcs(PolarLine({0, 0}, 5, -20), {.radius = 1, .center_on = Line({3, -2}, {3, 2})});
    });
    compare::add("obj_carcs_object_empty_selection", [] {
        return ConstrainedArcs(Edge::make_circle(1.0), Axis::Y, {.radius = 1},
                               [](const ShapeList<Edge>& arcs) { return arcs.filter_by(GeomType::LINE); });
    });
    compare::add("obj_carcs_axis_tangency_error", [] {
        return Curve(Edge::make_constrained_arcs(TangencyTarget(Axis::X, Tangency::ENCLOSING),
                                                 CenterArc({0, 5}, 2, 0, 360), {.radius = 3}));
    });
    compare::add("obj_carcs_ambiguous_error", [] {
        return Curve(Edge::make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), {.radius = 0.5, .center = Vector(0, 0.25)}));
    });
    compare::add("obj_carcs_radius_error", [] {
        return Curve(Edge::make_constrained_arcs(Vector(0, 0), Vector(0, 0.5), {.radius = -0.5}));
    });
    compare::add("obj_carcs_center_on_face_error", [] {
        return Curve(Edge::make_constrained_arcs(PolarLine({0, 0}, 4, -20, LengthMode::HORIZONTAL),
                                                 {.radius = 1, .center_on = Vector(1, 1)}));
    });
    compare::add("obj_carcs_3tan_points_error", [] {
        return Curve(Edge::make_constrained_arcs(Line({0, 0}, {1, 0}), Line({0, 0}, {0, 1}), Vector(3, 3)));
    });

    // ---- ConstrainedLines / Edge::make_constrained_lines
    compare::add_value("obj_clines", [] {
        const Edge unit = Edge::make_circle(1.0);
        const Edge moved = unit.translate({3, 0, 0});
        return V::list({
            edges_values(Edge::make_constrained_lines(unit, moved)),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING), moved)),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING),
                                                      TangencyTarget(moved, Tangency::ENCLOSING))),
            edges_values(Edge::make_constrained_lines(unit, Vector(2, 0))),
            edges_values(Edge::make_constrained_lines(unit, {3, 0})),
            edges_values(Edge::make_constrained_lines(unit, unit)),
            edges_values(Edge::make_constrained_lines(unit, Axis::X, {.angle = 30})),
            edges_values(Edge::make_constrained_lines(unit, Axis::Y, {.angle = 30})),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING), Axis::Y, {.angle = 30})),
            edges_values(Edge::make_constrained_lines(TangencyTarget(unit, Tangency::ENCLOSING), Axis::Y,
                                                      {.direction = Vector(1, 1)})),
            edges_values(Edge::make_constrained_lines(unit, Axis::X, {.angle = 0})),
            edges_values(Edge::make_constrained_lines(unit, Axis(Vector(1, 0), Vector(0, 1)), {.angle = 0})),
            edges_values(Edge::make_constrained_lines(Vector(0, 6), CenterArc({-5, 0}, 4, 0, 360))),
            edges_values(Edge::make_constrained_lines(Axis::X, Edge::make_circle(1.0, Plane::XY.offset(0)).translate({0, 3}))),
        });
    });
    compare::add("obj_clines_none", [] { return Curve(Edge::make_constrained_lines(Edge::make_circle(1.0), Vector(0, 0))); });
    compare::add("obj_clines_vertical_ref_error", [] {
        return Curve(Edge::make_constrained_lines(Edge::make_circle(1.0), Axis::Z, {.angle = 1}));
    });
    compare::add("obj_clines_two_points_error", [] {
        return Curve(Edge::make_constrained_lines(Vector(0, 0), Vector(1, 1)));
    });
    compare::add("obj_clines_object_all", [] {
        return ConstrainedLines(CenterArc({-5, 0}, 4, 0, 360), CenterArc({5, 0}, 3, 0, 360));
    });
    compare::add("obj_clines_object_first", [] {
        return ConstrainedLines(CenterArc({-5, 0}, 4, 0, 360), {0, 6}, {},
                                [](const ShapeList<Edge>& lines) { return lines[0]; });
    });
    compare::add("obj_clines_object_closest", [] {
        return ConstrainedLines(CenterArc({-5, 0}, 4, 0, 360), Axis::Y, {.angle = 30},
                                [](const ShapeList<Edge>& lines) { return lines.sort_by_distance(Vector(0, 0))[0]; });
    });

    // ---- limits: Edge::trim_to_other, trim_infinite, arc_size/length limits
    compare::add_value("obj_trim_to_other", [] {
        const double h = std::sqrt(2.0) / 2;
        const std::vector<Edge> bases{CenterArc({0, 0}, 1, 0, 180), CenterArc({0, 0}, 1, 0, 360)};
        const Sketch rect = Rectangle(1, 1, Align2(Align::MAX, Align::MIN)).rotate(Axis::Z, -45);
        std::vector<V> out;
        for (const Edge& base : bases) {
            out.push_back(optional_edge_values(base.trim_to_other(Line({0, 0}, {1, 1}))));
            out.push_back(optional_edge_values(base.trim_to_other(CenterArc({0, 0}, 1, 45, 90))));
            out.push_back(optional_edge_values(base.trim_to_other(rect.wire())));
            out.push_back(optional_edge_values(base.trim_to_other(rect)));
            out.push_back(optional_edge_values(base.trim_to_other(Curve(std::vector<Edge>(rect.edges())))));
            out.push_back(optional_edge_values(
                base.trim_to_other(Box(1, 1, 1, Align3(Align::MAX, Align::MIN, Align::CENTER)).rotate(Axis::Z, -45))));
            out.push_back(optional_edge_values(base.trim_to_other(Axis(Vector(0, 0, 0), Vector(1, 1, 0)))));
            out.push_back(optional_edge_values(base.trim_to_other(Location(Vector(h, h), Vector(0, 0, 1)))));
            out.push_back(optional_edge_values(base.trim_to_other(Plane(Vector(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)))));
            out.push_back(optional_edge_values(base.trim_to_other(Vector(1, 0).rotate(Axis::Z, 45))));
            out.push_back(optional_edge_values(base.trim_to_other(Vector(h, h))));
            out.push_back(optional_edge_values(base.trim_to_other(Vector(0, 2))));
            out.push_back(optional_edge_values(base.trim_to_other(Vertex(h, h, 0))));
            out.push_back(optional_edge_values(base.trim_to_other(Pos(0, 0.5) * Circle(0.8))));
            out.push_back(optional_edge_values(base.trim_to_other(Line({-2, 0.5}, {2, 0.5}))));
            out.push_back(optional_edge_values(base.reversed().trim_to_other(Line({0, 0}, {1, 1}))));
        }
        return V::list(out);
    });
    compare::add_value("obj_trim_infinite", [] {
        return V::list({edge_values(Edge(Axis::X).trim_infinite(5)),
                        edge_values(Edge(Axis(Vector(1, 2, 3), Vector(0, 1, 1))).trim_infinite(2)),
                        edge_values(Line({0, 0}, {1, 1}).trim_infinite(5)), Edge(Axis::Y).is_infinite()});
    });
    compare::add_value("obj_center_arc_limits", [] {
        const CenterArc c1({1, 0}, 1, 0, Line({1, 0}, {2, 1}));
        const CenterArc c2({1, 0}, 1, 0, Line({1, 0}, {2, -1}));
        const CenterArc c3({0, 0}, 2, 30, Axis::Y);
        const CenterArc c4({0, 0}, 2, 30, Plane::XZ);
        const CenterArc c5({0, 0}, 2, 10, Pos(0, 3) * Circle(1.5));
        const CenterArc c6({0, 0}, 2, 10, Vector(0, -2));
        return V::list({edge_values(c1), c1 % 0, edge_values(c2), c2 % 0, edge_values(c3), edge_values(c4),
                        edge_values(c5), edge_values(c6)});
    });
    compare::add("obj_center_arc_limit_error", [] { return CenterArc({1, 0}, 1, 0, {5, 0}); });
    compare::add_value("obj_conic_arc_limits", [] {
        return V::list({
            edge_values(EllipticalCenterArc({0, 0}, 2, 1, 0, Line({0, 0}, {0, 2}))),
            edge_values(EllipticalCenterArc({0, 0}, 2, 1, 0, Line({0, 0}, {0, -2}))),
            edge_values(EllipticalCenterArc({1, 1}, 3, 1, 0, Axis(Vector(1, 1), Vector(1, 1)), 20)),
            edge_values(ParabolicCenterArc({0, 0}, 0.5, 0, Line({0, 1}, {5, 1}))),
            edge_values(ParabolicCenterArc({0, 0}, 0.5, 0, Line({1, -2}, {1, 2}))),
            edge_values(ParabolicCenterArc({0, 0}, 0.5, -30, Axis::X, 15)),
            edge_values(HyperbolicCenterArc({0, 0}, 2, 1, 0, Line({0, 1}, {10, 1}))),
            edge_values(HyperbolicCenterArc({0, 0}, 2, 1, 0, Line({3, -2}, {3, 2}))),
            edge_values(HyperbolicCenterArc({0, 0}, 1, 2, 0, Line({-5, 3}, {5, 3}), 10)),
        });
    });
    compare::add("obj_elliptical_limit_error", [] { return EllipticalCenterArc({0, 0}, 2, 1, 0, {0, 5}); });
    compare::add("obj_parabolic_limit_error", [] { return ParabolicCenterArc({0, 0}, 0.5, 0, {0, 5}); });
    compare::add("obj_hyperbolic_limit_error", [] { return HyperbolicCenterArc({0, 0}, 2, 1, 0, {0, 5}); });
    compare::add_value("obj_jern_arc_limits", [] {
        const JernArc j1({1, 0}, {0, 1}, 1, Line({1, 0}, {2, 1}));
        const JernArc j2({1, 0}, {0, 1}, 1, Line({1, 0}, {0, 1}));
        const JernArc j3({0, 0}, {1, 1}, 3, Axis::Y);
        const JernArc j4({0, 0, 1}, {0, 0, 1}, 2, Plane::XY.offset(3));
        return V::list({edge_values(j1), j1.center_point, j1.end_of_arc, j1.start, edge_values(j2), j2.center_point,
                        edge_values(j3), j3.center_point, edge_values(j4), j4.center_point});
    });
    compare::add("obj_jern_arc_limit_error", [] { return JernArc({1, 0}, {0, 1}, 1, {5, 0}); });
    compare::add_value("obj_polar_line_limits", [] {
        const double h = std::sqrt(2.0) / 2;
        const Sketch rect = Rectangle(1, 1, Align2(Align::MAX, Align::MIN)).rotate(Axis::Z, -45);
        const std::vector<Limit> limits{
            Line({0, 0}, {1, 1}),
            CenterArc({0, 0}, 1, 45, 90),
            rect.wire(),
            rect,
            Curve(std::vector<Edge>(rect.edges())),
            Box(1, 1, 1, Align3(Align::MAX, Align::MIN, Align::CENTER)).rotate(Axis::Z, -45),
            Axis(Vector(0, 0, 0), Vector(1, 1, 0)),
            Location(Vector(h, h), Vector(0, 0, 1)),
            Plane(Vector(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)),
            Vector(1, 0).rotate(Axis::Z, 45),
            Limit(h, h),
        };
        std::vector<V> out;
        for (const auto& limit : limits) out.push_back(edge_values(PolarLine({std::sqrt(2.0), 0}, limit, 135)));
        out.push_back(edge_values(PolarLine({2, 0}, Axis::Y, 135)));
        out.push_back(edge_values(PolarLine({0, 0}, Line({3, -1}, {3, 5}), Vector(1, 1))));
        return V::list(out);
    });
    compare::add("obj_polar_line_limit_point_error", [] { return PolarLine({std::sqrt(2.0), 0}, {0, 2}, 135); });
    compare::add("obj_polar_line_limit_shape_error", [] {
        return PolarLine({std::sqrt(2.0), 0}, Line({3, 0}, {3, 1}), 135);
    });
    compare::add("obj_polar_line_limit_behind_error", [] { return PolarLine({0, 0}, Axis::Y, 0.0); });
    compare::add("obj_polar_line_limit_location_aligned", [] {
        return PolarLine({0, 0}, Location(Plane(Vector(3, 0), std::nullopt, Vector(1, 0, 0))), 0.0);
    });

    // ---- projection
    compare::add_value("obj_wire_project_multiple", [] {
        const Part target = Box(10, 10, 4) - Pos(0, 0, 2) * Box(5, 5, 2);
        const Wire circle = Wire::make_circle(3, Plane::XY.offset(10));
        return V::list({curves_values(circle.project_to_shape(target, Vector(0, 0, -1))),
                        curves_values(circle.project_to_shape(target, Vector(0, 0, 1)))});
    });
    compare::add_value("obj_wire_project_conical", [] {
        const Part target = Box(10, 10, 4) - Pos(0, 0, 2) * Box(5, 5, 2);
        const Wire circle = Wire::make_circle(3, Plane::XY.offset(10));
        const Wire square = Wire::make_rect(4, 4, Plane::XY.offset(30));
        return V::list({curves_values(circle.project_to_shape(target, std::nullopt, Vector(0, 0, 20))),
                        curves_values(square.project_to_shape(Sphere(10), std::nullopt, Vector(0, 0, 40))),
                        curves_values(Wire::make_circle(2, Plane::XZ.offset(-30))
                                          .project_to_shape(Sphere(10), std::nullopt, Vector(0, -50, 0)))});
    });
    compare::add_value("obj_edge_project", [] {
        const auto projection = Edge::make_circle(1, Plane::XY.offset(-5)).project_to_shape(Solid::make_box(1, 1, 1), Vector(0, 0, 1));
        const auto conical = Line({-2, -2, 10}, {2, 2, 10}).project_to_shape(Sphere(5), std::nullopt, Vector(0, 0, 20));
        const auto reversed = Edge::make_circle(1, Plane::XY.offset(-5)).reversed().project_to_shape(
            Solid::make_box(1, 1, 1), Vector(0, 0, 1));
        return V::list({curves_values(projection), projection[0].arc_center(), curves_values(conical),
                        curves_values(reversed)});
    });
    compare::add("obj_project_to_shape_no_direction", [] {
        return Compound(Wire::make_circle(1).project_to_shape(Sphere(50)));
    });
    compare::add("obj_project_to_shape_both", [] {
        return Compound(Wire::make_circle(1).project_to_shape(Sphere(50), Vector(0, 0, 1), Vector(0, 0, 5)));
    });
    compare::add_value("obj_project_clean_wire", [] {
        const Edge center_arc = Edge::make_ellipse(45, 30, Plane::XY, 270 + 40, 270 - 40);
        const Face center_surface =
            -Face::extrude(center_arc, Vector(0, 0, 50)).moved(Location(Vector(0, 0, -25), Vector(0, 0, 180)));
        const Location tip_center_loc = center_surface.location_at(center_arc.position_at(1), Vector(1, 0, 0));
        const Vector normal = tip_center_loc.z_axis().direction;
        const Edge planar_tip_arc = Edge::make_circle(12.5, Plane::XY, 270, 450).located(tip_center_loc);
        return curves_values(planar_tip_arc.project_to_shape(center_surface, -normal));
    });
    compare::add_value("obj_mixin1d_project", [] {
        const Face rect = Face::make_rect(10, 10);
        const Edge circle = Edge::make_circle(1, Plane::XY.offset(5));
        const Face sphere_face = Sphere(5).faces()[0];
        const Wire square = Wire::make_rect(2, 2, Plane::XY.offset(8));
        return V::list({
            curve_values(circle.project(rect, Vector(0, 0, -1))),
            curves_values(circle.project(rect, Vector(0, 0, -1), false)),
            curve_values(square.project(sphere_face, Vector(0, 0, -1))),
            curves_values(square.project(sphere_face, Vector(0, 0, -1), false)),
            curves_values(Line({-1, 0, 8}, {1, 0, 8}).project(sphere_face, Vector(0, 0, -1), false)),
        });
    });
    compare::add("obj_project_faces", [] {
        return project(Box(4, 6, 8).rotate(Axis::X, 30).faces(), Plane::XY.offset(-10));
    });
    compare::add("obj_project_face", [] {
        return project(Plane::XZ.offset(-5) * Rot(0, 30, 0) * Rectangle(4, 3).face(), Plane::XY);
    });
    compare::add("obj_project_face_ignores_target", [] {
        return project(Circle(3).face(), Plane::XY.offset(7), Box(1, 1, 1));
    });
    compare::add("obj_project_edges", [] { return project(Line({0, 0, 0}, {5, 5, 0}), Plane::XY.offset(5)); });
    compare::add("obj_project_sketch_ignored", [] { return project(Circle(3), Plane::XY.offset(5)); });
    compare::add_value("obj_project_points", [] {
        return V::list({points_values(project(Vector(1, 2, 3), Plane::XY)),
                        points_values(project(Vertex(1, 2, 3), Plane::XZ)),
                        points_values(project(std::vector<Vector>{{1, 2, 3}, {4, 5, 6}}, Plane::XY.offset(2))),
                        points_values(project(std::vector<Vertex>{Vertex(1, 2, 3), Vertex(-1, 0, 2)},
                                              Plane(Vector(1, 1, 1), Vector(1, 0, 0), Vector(0, 1, 1))))});
    });
    compare::add_value("obj_project_mixed", [] {
        const auto with_points = project(std::vector<Shape>{Rectangle(2, 2).face(), Vertex(1, 2, 3)}, Plane::XY);
        const auto shapes = project(std::vector<Shape>{Rectangle(2, 2).face(), Line({0, 0}, {1, 1})}, Plane::XY.offset(1));
        const Compound& c = std::get<Compound>(shapes);
        return V::list({points_values(std::get<std::vector<Vector>>(with_points)), c.area(),
                        static_cast<double>(c.faces().size()), c.center()});
    });
    compare::add("obj_project_empty_error", [] { return project(std::vector<Face>{}, Plane::XY); });

    // ---- Voronoi vertices (scipy.spatial.Voronoi) and full_round
    compare::add_value("obj_voronoi_rect", [] {
        const auto edges = Rectangle(34, 10).edges().sort_by(Axis::X);
        return voronoi_values(sample_points({edges[0], edges[1], edges[2]}));
    });
    compare::add_value("obj_voronoi_trapezoid", [] {
        const auto edges = Trapezoid(0.5, 1, 82).edges().sort_by(Axis::Y);
        return voronoi_values(sample_points({edges[3], edges[1], edges[2]}));
    });
    compare::add_value("obj_voronoi_triangle", [] {
        const auto edges = Triangle({.a = 10, .b = 10, .c = 10}).edges().sort_by(Axis::X);
        return voronoi_values(sample_points({edges[0], edges[1], edges[2]}));
    });
    compare::add_value("obj_voronoi_arcs", [] {
        return voronoi_values(sample_points({CenterArc({0, 0}, 5, 0, 90), Line({0, 5}, {-6, 5}),
                                             Spline({{5, 0}, {6, -3}, {9, -4}})}, 40));
    });
    compare::add("obj_full_round_trap_top", [] {
        const Trapezoid trap(0.5, 1, 90 - 8);
        return full_round(trap.edges().sort_by(Axis::Y)[-1]);
    });
    compare::add("obj_full_round_trap_short", [] {
        const Trapezoid trap(0.5, 1, 90 - 8);
        return full_round(trap.edges().sort_by(SortBy::LENGTH)[0]);
    });
    compare::add("obj_full_round_trap_invert", [] {
        const Trapezoid trap(0.5, 1, 90 - 8);
        return full_round(trap.edges().sort_by(SortBy::LENGTH)[0], true);
    });
    compare::add("obj_full_round_trap_side", [] {
        const Trapezoid trap(0.5, 1, 90 - 8);
        return full_round(trap.edges().sort_by(Axis::X)[-1]);
    });
    compare::add("obj_full_round_rect", [] { return full_round((Rectangle(34, 10).edges() << Axis::X)[0]); });
    compare::add("obj_full_round_rect_invert", [] {
        return full_round((Rectangle(34, 10).edges() << Axis::X)[0], true);
    });
    compare::add("obj_full_round_rect_points", [] {
        return full_round((Rectangle(34, 10).edges() << Axis::X)[0], false, 37);
    });
    compare::add("obj_full_round_triangle", [] {
        return full_round(Triangle({.a = 10, .b = 10, .c = 10}).edges().sort_by(Axis::X)[0]);
    });
    compare::add_value("obj_full_round_flipped", [] {
        const Face flipped = -Face::make_rect(34, 10);
        const Sketch rounded = full_round((flipped.edges() << Axis::X)[0]);
        return V::list({flipped.normal_at(), rounded.face().normal_at(), rounded.area()});
    });
    compare::add("obj_full_round_slot_hole", [] {
        const Sketch s = Rectangle(30, 10) - Pos(5, 0) * Circle(2);
        return full_round((s.edges() << Axis::X)[0]);
    });
    compare::add("obj_full_round_no_parent", [] { return full_round(Edge::make_line({0, 0}, {1, 0})); });
    compare::add("obj_full_round_two_edges", [] {
        const Edge l1 = Edge::make_spline({{-1, 0}, {1, 0}}, {{0, -8}, {0, 8}}, false, {}, true);
        const Edge l2 = Edge::make_line(l1.position_at(0), l1.position_at(1));
        const Face face(Wire(std::vector<Edge>{l1, l2}));
        return full_round(face.edges()[0]);
    });

    // ---- make_brake_formed
    const auto outline = [] { return FilletPolyline({{0, 0}, {5, 6}, {10, 1}}, 1); };
    const auto zigzag = [] { return Polyline({{0, 0}, {5, 6}, {10, 1}}); };
    compare::add("obj_brake_fillet_polyline", [=] { return make_brake_formed(0.5, 1, outline()); });
    compare::add("obj_brake_widths", [=] { return make_brake_formed(0.5, {1, 2, 3, 4}, outline()); });
    compare::add("obj_brake_right", [=] { return make_brake_formed(0.5, 1, outline(), Side::RIGHT); });
    compare::add("obj_brake_intersection", [=] {
        return make_brake_formed(0.5, 1.5, zigzag(), Side::LEFT, Kind::INTERSECTION);
    });
    compare::add("obj_brake_tangent", [=] {
        return make_brake_formed(0.5, 1.5, zigzag(), Side::LEFT, Kind::TANGENT);
    });
    compare::add("obj_brake_polyline", [=] { return make_brake_formed(0.5, 2, zigzag()); });
    compare::add("obj_brake_polyline_widths", [=] { return make_brake_formed(0.5, {1, 2, 3}, zigzag()); });
    compare::add("obj_brake_arc", [] {
        return make_brake_formed(0.5, 1, Edge::make_circle(5, Plane::XY, 0, 60));
    });
    compare::add("obj_brake_straight", [] { return make_brake_formed(0.5, 1, Edge::make_line({0, 0}, {5, 0})); });
    compare::add("obj_brake_wrong_count", [=] { return make_brake_formed(0.5, {1, 2}, outline()); });
    compare::add("obj_brake_no_clean", [=] {
        return make_brake_formed(0.5, 1, outline(), Side::LEFT, Kind::ARC, false);
    });
    compare::add("obj_brake_curve", [=] { return make_brake_formed(0.5, 2, Curve() + zigzag()); });
    compare::add("obj_brake_curve_steps", [] {
        const Curve steps = Curve() + std::vector<Edge>{Line({0, 0}, {4, 0}), Line({4, 0}, {6, 3}), Line({6, 3}, {10, 3})};
        return make_brake_formed(0.3, 1.5, steps);
    });
    compare::add("obj_brake_xz", [=] { return make_brake_formed(0.5, 1, Wire(Plane::XZ * outline())); });
    compare::add("obj_brake_spline", [] { return make_brake_formed(0.4, 1, Spline({{0, 0}, {3, 2}, {6, -1}, {9, 0}})); });
    compare::add("obj_brake_mixed_right", [] {
        const Wire mixed(std::vector<Edge>{Edge::make_line({0, 0}, {4, 0}), Edge::make_tangent_arc({4, 0}, {1, 0}, {6, 2})});
        return make_brake_formed(0.4, {1, 2, 1}, mixed, Side::RIGHT);
    });

    // ---- pack
    compare::add_value("obj_pack2d_growth", [] { return pack2d_values({{2, 10}, {3, 2}}); });
    compare::add_value("obj_pack2d_ties", [] {
        return pack2d_values({{3, 3}, {2, 5}, {5, 2}, {3, 3}, {1, 1}, {4, 4}, {2, 2}, {5, 1}, {1, 5}, {0.5, 7}});
    });
    compare::add_value("obj_pack2d_random", [] { return pack2d_values(random_boxes); });
    compare::add_value("obj_pack_simple", [] {
        return bbox_values(pack(std::vector<Box>{Box(10, 2, 1), Box(1, 5, 1), Box(1, 5, 1)}, 1));
    });
    compare::add_value("obj_pack_random_boxes", [] {
        std::vector<Part> boxes;
        for (const auto& [w, l] : random_boxes) boxes.push_back(Box(w, l, 1));
        return bbox_values(pack(boxes, 1));
    });
    compare::add("obj_pack_random_boxes_part", [] {
        std::vector<Part> boxes;
        for (const auto& [w, l] : random_boxes) boxes.push_back(Box(w, l, 1));
        return Part() + pack(boxes, 1);
    });
    compare::add_value("obj_pack_random_slots", [] {
        std::vector<Sketch> slots;
        for (const auto& [w, h] : random_slots) slots.push_back(SlotOverall(w, h));
        return bbox_values(pack(slots, 1));
    });
    compare::add_value("obj_pack_align_z", [] {
        const std::vector<Shape> objects{Pos(5, 5, 5) * Box(1, 2, 3), Pos(-3, 0, -2) * Sphere(1), Circle(2),
                                         Edge::make_line({0, 0}, {3, 4})};
        return bbox_values(pack(objects, 0.5, true));
    });
    compare::add_value("obj_pack_vertices", [] { return bbox_values(pack(std::vector<Vertex>{Vertex(1, 2, 3), Vertex(4, 5, 6)}, 1)); });
    compare::add_value("obj_pack_empty", [] { return bbox_values(pack(std::vector<Part>{}, 1)); });
    compare::add_value("obj_pack_overlap", [] {
        return bbox_values(pack(std::vector<Box>{Box(2, 2, 2), Box(2, 2, 2), Box(3, 1, 1)}, -1.5));
    });
    compare::add_value("obj_pack_empty_part", [] { return bbox_values(pack(std::vector<Part>{Part()}, 1)); });

    // ---- operation signatures and inputs (lists, empty inputs)
    const auto z5 = [] { return Edge::make_line({0, 0, 0}, {0, 0, 5}); };
    compare::add("obj_ops_extrude_empty_sketch", [] { return extrude(Sketch(), 1); });
    compare::add("obj_ops_extrude_empty_list", [] { return extrude(std::vector<Face>{}, 1); });
    compare::add("obj_ops_extrude_tuple_both", [] {
        return extrude(std::vector<Face>{Face::make_rect(1, 1), Pos(3, 0) * Face::make_rect(1, 1)}, 1, {.both = true});
    });
    compare::add("obj_ops_extrude_list_until", [] {
        return extrude(std::vector<Face>{Face::make_rect(2, 2), Pos(5, 0) * Face::make_rect(1, 1)}, Until::NEXT,
                       Pos(0, 0, 5) * Box(20, 20, 1));
    });
    compare::add("obj_ops_extrude_list_until_last", [] {
        return extrude(std::vector<Face>{Face::make_rect(2, 2), Pos(5, 0) * Face::make_rect(1, 1)}, Until::LAST,
                       Pos(0, 0, 5) * Box(20, 20, 1) + Pos(0, 0, 9) * Box(20, 20, 1));
    });
    compare::add("obj_ops_extrude_empty_until", [] { return extrude(Sketch(), Until::NEXT, Shape()); });
    compare::add("obj_ops_extrude_until_no_target", [] {
        return extrude(Face::make_rect(1, 1), Until::NEXT, Shape());
    });
    compare::add("obj_ops_revolve_sketches", [] {
        return revolve(std::vector<Sketch>{Plane::XZ * (Pos(3, 0) * Rectangle(1, 1)), Plane::XZ * (Pos(6, 0) * Circle(0.5))});
    });
    compare::add("obj_ops_revolve_mixed", [] {
        return revolve(std::vector<Shape>{Plane::XZ * (Pos(3, 0) * Rectangle(1, 1)), (Plane::XZ * (Pos(6, 0) * Circle(0.5))).face()},
                       Axis::Z, 90);
    });
    compare::add("obj_ops_revolve_empty_sketch", [] { return revolve(Sketch()); });
    compare::add("obj_ops_revolve_empty_list", [] { return revolve(std::vector<Face>{}); });
    compare::add("obj_ops_thicken_list", [] {
        return thicken(std::vector<Face>{Face::make_rect(1, 1), Pos(3, 0) * Face::make_rect(1, 1)}, 1);
    });
    compare::add("obj_ops_thicken_list_both", [] {
        return thicken(std::vector<Face>{Face::make_rect(1, 1), Pos(3, 0) * Face::make_rect(1, 1)}, 0.5, std::nullopt, true);
    });
    compare::add("obj_ops_thicken_empty_sketch", [] { return thicken(Sketch(), 1); });
    compare::add("obj_ops_thicken_empty_list", [] { return thicken(std::vector<Face>{}, 1); });
    compare::add("obj_ops_draft_single_face", [] {
        return draft(Box(10, 10, 10).faces().filter_by(Axis::Z, true)[0], Plane::XY.offset(-5), 5);
    });
    compare::add("obj_ops_make_face_mixed", [] {
        return make_face(std::vector<Shape>{Wire::make_polygon({{0, 0}, {3, 0}, {3, 2}}, false), Curve() + Line({3, 2}, {0, 0})});
    });
    compare::add("obj_ops_make_face_curves", [] {
        return make_face(std::vector<Curve>{Curve() + Polyline({{0, 0}, {3, 0}, {3, 2}, {0, 0}})});
    });
    compare::add("obj_ops_make_face_wires", [] {
        return make_face(std::vector<Wire>{Wire::make_polygon({{0, 0}, {4, 0}}, false),
                                           Wire::make_polygon({{4, 0}, {2, 3}, {0, 0}}, false)});
    });
    compare::add("obj_ops_make_face_empty", [] { return make_face(std::vector<Edge>{}); });
    compare::add("obj_ops_trace_mixed", [] {
        return trace(std::vector<Shape>{Wire::make_polygon({{0, 0}, {3, 0}, {3, 2}}, false), Curve() + Line({5, 5}, {8, 5})}, 0.5);
    });
    compare::add("obj_ops_trace_wires", [] { return trace(std::vector<Wire>{Wire::make_polygon({{0, 0}, {3, 0}, {3, 2}}, false)}); });
    compare::add("obj_ops_trace_empty", [] { return trace(std::vector<Edge>{}); });
    compare::add("obj_ops_offset_default", [] { return offset(Rectangle(2, 2)); });
    compare::add("obj_ops_offset_hole_splits", [] { return offset(Rectangle(10, 2) - Circle(0.8), -0.3); });
    compare::add("obj_ops_offset_hole_grows", [] { return offset(Rectangle(10, 4) - Circle(0.8), -0.3); });
    compare::add("obj_ops_offset_hole_vanishes", [] { return offset(Rectangle(10, 4) - Circle(0.8), 1); });
    compare::add("obj_ops_offset_nested_compound", [] {
        return offset(Compound(std::vector<Shape>{Compound(std::vector<Shape>{Edge::make_line({0, 0}, {4, 0})}),
                                                  Edge::make_line({4, 0}, {4, 3})}),
                      0.5);
    });
    compare::add("obj_ops_offset_compound_wire_edge", [] {
        return offset(Compound(std::vector<Shape>{Wire::make_polygon({{0, 0}, {5, 0}, {5, 5}}, false),
                                                  Edge::make_line({5, 5}, {0, 5})}),
                      0.5);
    });
    compare::add("obj_ops_scale_default", [] { return scale(Box(2, 2, 2)); });
    compare::add("obj_ops_scale_list_vector", [] {
        return scale(std::vector<Part>{Box(2, 2, 2), Pos(5, 0, 0) * Box(2, 2, 2)}, Vector(1, 2, 3));
    });
    compare::add("obj_ops_scale_list_vector_about", [] {
        return scale(std::vector<Part>{Box(2, 2, 2), Pos(5, 0, 0) * Sphere(1)}, Vector(1, 2, 3), Vector(1, 1, 1));
    });
    compare::add("obj_ops_split_list_face", [] {
        return split(std::vector<Part>{Box(2, 2, 2), Pos(5, 0, 0) * Box(2, 2, 2)}, Face::make_rect(20, 20, Plane::XY.offset(0.5)));
    });
    compare::add("obj_ops_split_list_face_both", [] {
        return split(std::vector<Part>{Box(2, 2, 2), Pos(5, 0, 0) * Sphere(1)}, Face::make_rect(20, 20, Plane::XY.offset(0.25)),
                     Keep::BOTH);
    });
    compare::add("obj_ops_sweep_curve_children", [=] {
        return sweep(Curve() + std::vector<Edge>{Line({0, 0}, {1, 0}), Line({0, 2}, {1, 2})}, z5());
    });
    compare::add("obj_ops_sweep_curve_connected", [=] {
        return sweep(Curve() + std::vector<Edge>{Line({0, 0}, {1, 0}), Line({1, 0}, {1, 1})}, z5());
    });
    compare::add("obj_ops_sweep_path_edges", [] {
        return sweep(Circle(1), std::vector<Edge>{Edge::make_line({0, 0, 0}, {0, 0, 5}), Edge::make_line({0, 0, 5}, {3, 0, 8})});
    });
    compare::add("obj_ops_sweep_path_edges_list", [] {
        return sweep(std::vector<Sketch>{Circle(1), Pos(0, 0, 0) * Rectangle(0.5, 0.5)},
                     std::vector<Edge>{Edge::make_line({0, 0, 0}, {0, 0, 5}), Edge::make_line({0, 0, 5}, {3, 0, 8})});
    });
    compare::add("obj_ops_sweep_path_edges_bad", [] {
        return sweep(Circle(1), std::vector<Edge>{Edge::make_line({0, 0, 0}, {0, 0, 5}), Edge::make_line({0, 1, 6}, {3, 0, 8})});
    });
    compare::add("obj_ops_sweep_path_curve_bad", [] {
        return sweep(Circle(1), Curve() + std::vector<Edge>{Line({0, 0, 0}, {0, 0, 5}), Line({1, 1, 6}, {3, 0, 8})});
    });
    compare::add("obj_ops_sweep_mixed", [] {
        return sweep(std::vector<Shape>{Face::make_rect(1, 1), Edge::make_line({3, 0}, {4, 0})},
                     Edge::make_line({0, 0, 0}, {0, 0, 4}));
    });
    compare::add("obj_ops_sweep_mixed_holed", [] {
        return sweep(std::vector<Shape>{Rectangle(4, 4) - Circle(1), Edge::make_line({3, 0}, {4, 0})},
                     Edge::make_line({0, 0, 0}, {0, 0, 4}));
    });
    compare::add("obj_ops_sweep_compound_faces", [] {
        return sweep(Compound(std::vector<Shape>{Face::make_rect(1, 1), Pos(3, 0, 0) * Face::make_rect(1, 1)}),
                     Edge::make_line({0, 0, 0}, {0, 0, 4}));
    });
    compare::add("obj_ops_sweep_binormal_edge", [] {
        return sweep(Pos(5, 0, 0) * Rectangle(1, 1), Edge::make_helix(5, 10, 5),
                     {.binormal = Wire(Edge::make_line({0, 0, 0}, {0, 0, 10}))});
    });
    compare::add("obj_ops_sweep_empty_list", [=] { return sweep(std::vector<Face>{}, z5()); });
    compare::add("obj_ops_bounding_box_compound_faces", [] {
        return bounding_box(Compound(std::vector<Shape>{Pos(1, 2) * Face::make_rect(2, 1), Pos(-3, 0) * Circle(1).face()}));
    });
    compare::add("obj_ops_bounding_box_shapes", [] {
        return bounding_box(std::vector<Shape>{Pos(0, 0, 3) * Box(1, 2, 3), Pos(5, 5, 5) * Sphere(1)});
    });

    // ---- objects with edge and error inputs (both sides must agree on what fails)
    compare::add("obj_edge_line1", [] { return Line(std::vector<Vector>{{0, 0}}); });
    compare::add("obj_edge_line3", [] { return Line(std::vector<Vector>{{0, 0}, {1, 0}, {2, 0}}); });
    compare::add("obj_edge_line_same", [] { return Line({0, 0}, {0, 0}); });
    compare::add("obj_edge_polyline1", [] { return Polyline({{0, 0}}); });
    compare::add("obj_edge_polyline_close2", [] { return Polyline({{0, 0}, {1, 0}}, true); });
    compare::add("obj_edge_tpa2", [] { return ThreePointArc(std::vector<Vector>{{0, 0}, {1, 1}}); });
    compare::add("obj_edge_tpa_collinear", [] { return ThreePointArc({0, 0}, {1, 0}, {2, 0}); });
    compare::add("obj_edge_spline1", [] { return Spline({{0, 0}}); });
    compare::add("obj_edge_spline_tan1", [] { return Spline({{0, 0}, {1, 1}, {2, 0}}, {{1, 0}}); });
    compare::add("obj_edge_spline_tan3", [] { return Spline({{0, 0}, {1, 1}, {2, 0}}, {{1, 0}, {1, 1}, {1, 0}}); });
    compare::add("obj_edge_spline_scalars", [] { return Spline({{0, 0}, {1, 1}, {2, 0}}, {{1, 0}, {1, 0}}, {1}); });
    compare::add("obj_edge_tangent_arc3", [] { return TangentArc(std::vector<Vector>{{0, 0}, {1, 1}, {2, 0}}, {1, 0}); });
    compare::add("obj_edge_radius_arc_small", [] { return RadiusArc({0, 0}, {10, 0}, 2); });
    compare::add("obj_edge_radius_arc_neg", [] { return RadiusArc({0, 0}, {10, 0}, -8); });
    compare::add("obj_edge_sagitta_zero", [] { return SagittaArc({0, 0}, {10, 0}, 0); });
    compare::add("obj_edge_center_arc_zero", [] { return CenterArc({0, 0}, 5, 0, 0.0); });
    compare::add("obj_edge_center_arc_720", [] { return CenterArc({0, 0}, 5, 0, 720.0); });
    compare::add("obj_edge_jern_zero_radius", [] { return JernArc({0, 0}, {1, 0}, 0, 90.0); });
    compare::add("obj_edge_jern_neg", [] { return JernArc({0, 0}, {1, 0}, 2, -90.0); });
    compare::add("obj_edge_helix_neg_pitch", [] { return Helix(-2, 10, 3); });
    compare::add("obj_edge_helix_cone", [] { return Helix(2, 10, 3, {0, 0, 0}, {0, 0, 1}, 10, true); });
    compare::add("obj_edge_polar_neg", [] { return PolarLine({0, 0}, -5.0, 30.0); });
    compare::add("obj_edge_polar_vertical_0", [] { return PolarLine({0, 0}, 5.0, 0.0, LengthMode::VERTICAL); });
    compare::add("obj_edge_polar_vertical_180", [] { return PolarLine({0, 0}, 5.0, 180.0, LengthMode::VERTICAL); });
    compare::add("obj_edge_polar_horizontal_90", [] { return PolarLine({0, 0}, 5.0, 90.0, LengthMode::HORIZONTAL); });
    compare::add("obj_edge_polar_direction_vertical", [] {
        return PolarLine({0, 0}, 5.0, Vector(1, 0), LengthMode::VERTICAL);
    });
    compare::add("obj_edge_fillet_poly_mismatch", [] {
        return FilletPolyline({{0, 0}, {5, 0}, {5, 5}, {0, 5}}, std::vector<double>{1, 1, 1});
    });
    compare::add("obj_edge_fillet_poly_zero", [] { return FilletPolyline({{0, 0}, {5, 0}, {5, 5}}, 0.0); });
    compare::add("obj_edge_fillet_poly_two", [] { return FilletPolyline({{0, 0}, {5, 0}}, 1.0); });
    compare::add("obj_edge_fillet_poly_close_list", [] {
        return FilletPolyline({{0, 0}, {5, 0}, {5, 5}, {0, 5}}, std::vector<double>{1, 0.5, 0.25, 0.75}, true);
    });
    compare::add("obj_edge_reg_poly2", [] { return RegularPolygon(3, 2); });
    compare::add("obj_edge_reg_poly_minor", [] { return RegularPolygon(3, 5, false); });
    compare::add("obj_edge_trap_bad", [] { return Trapezoid(5, 2, 20); });
    compare::add("obj_edge_trap_obtuse", [] { return Trapezoid(5, 2, 120); });
    compare::add("obj_edge_tri_two", [] { return Triangle({.a = 3, .b = 4}); });
    compare::add("obj_edge_tri_bad", [] { return Triangle({.a = 1, .b = 2, .c = 5}); });
    compare::add("obj_edge_tri_aaa", [] { return Triangle({.A = 60, .B = 60, .C = 60}); });
    compare::add("obj_edge_tri_ssa_ambiguous", [] { return Triangle({.a = 3, .b = 4, .A = 40}); });
    compare::add_value("obj_tri_ssa_ab", [] { return tri_values(Triangle({.a = 5, .b = 3, .A = 70})); });
    compare::add_value("obj_tri_ssa_ac", [] { return tri_values(Triangle({.a = 5, .c = 3, .A = 70})); });
    compare::add_value("obj_tri_ssa_ba", [] { return tri_values(Triangle({.a = 3, .b = 5, .B = 70})); });
    compare::add_value("obj_tri_ssa_bc", [] { return tri_values(Triangle({.b = 5, .c = 3, .B = 70})); });
    compare::add_value("obj_tri_ssa_ca", [] { return tri_values(Triangle({.a = 3, .c = 5, .C = 70})); });
    compare::add_value("obj_tri_ssa_cb", [] { return tri_values(Triangle({.b = 3, .c = 5, .C = 70})); });
    compare::add_value("obj_tri_ssa_right", [] { return tri_values(Triangle({.a = 4, .b = 8, .A = 30})); });
    compare::add_value("obj_tri_ssa_obtuse_only", [] { return tri_values(Triangle({.a = 6, .b = 4, .A = 120})); });
    compare::add_value("obj_tri_ssa_none", [] { return tri_values(Triangle({.a = 2, .b = 8, .A = 60})); });
    compare::add_value("obj_tri_sas_abC", [] { return tri_values(Triangle({.a = 3, .b = 4, .C = 90})); });
    compare::add_value("obj_tri_sas_bcA", [] { return tri_values(Triangle({.b = 3, .c = 5, .A = 40})); });
    compare::add_value("obj_tri_sas_caB", [] { return tri_values(Triangle({.a = 2, .c = 7, .B = 110})); });
    compare::add_value("obj_tri_aas_c", [] { return tri_values(Triangle({.c = 5, .A = 40, .B = 60})); });
    compare::add_value("obj_tri_aas_a", [] { return tri_values(Triangle({.a = 5, .B = 40, .C = 60})); });
    compare::add_value("obj_tri_aas_b", [] { return tri_values(Triangle({.b = 5, .A = 30, .C = 100})); });
    compare::add_value("obj_tri_aas_bad", [] { return tri_values(Triangle({.a = 5, .A = 100, .B = 90})); });
    compare::add_value("obj_tri_sss", [] { return tri_values(Triangle({.a = 4, .b = 5, .c = 6})); });
    compare::add_value("obj_tri_angle_180", [] { return tri_values(Triangle({.a = 3, .b = 4, .C = 180})); });
    compare::add_value("obj_tri_zero_side", [] { return tri_values(Triangle({.a = 0, .b = 4, .C = 30})); });
    compare::add("obj_edge_slot_cp_same", [] { return SlotCenterPoint({0, 0}, {0, 0}, 2); });
    compare::add("obj_edge_slot_c2c_zero", [] { return SlotCenterToCenter(0, 2); });
    compare::add("obj_edge_slot_overall_narrow", [] { return SlotOverall(2, 3); });
    compare::add("obj_edge_superellipse_order1", [] { return Superellipse(4, 2, 1); });
    compare::add("obj_edge_rect_rounded_big", [] { return RectangleRounded(4, 2, 1); });
    compare::add("obj_edge_circle_half", [] { return Circle(2, 180); });
    compare::add("obj_edge_convex_3", [] { return ConvexPolyhedron({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}); });
    compare::add("obj_convex_cube", [] {
        std::vector<Vector> pts;
        for (double x : {0, 2}) for (double y : {0, 3}) for (double z : {0, 1}) pts.emplace_back(x, y, z);
        return ConvexPolyhedron(pts);
    });
    compare::add("obj_convex_interior", [] {
        return ConvexPolyhedron({{0, 0, 0}, {4, 0, 0}, {0, 4, 0}, {0, 0, 4}, {1, 1, 1}, {0.5, 0.2, 0.3}, {2, 0, 0}, {0, 2, 2}});
    });
    compare::add("obj_convex_icosahedron", [] {
        const double p = 1.618033988749895;
        std::vector<Vector> pts;
        for (double s1 : {-1, 1}) for (double s2 : {-1, 1}) pts.emplace_back(0, s1, s2 * p);
        for (double s1 : {-1, 1}) for (double s2 : {-1, 1}) pts.emplace_back(s1, s2 * p, 0);
        for (double s1 : {-1, 1}) for (double s2 : {-1, 1}) pts.emplace_back(s2 * p, 0, s1);
        return ConvexPolyhedron(pts);
    });
    compare::add("obj_convex_duplicates", [] {
        return ConvexPolyhedron({{0, 0, 0}, {0, 0, 0}, {3, 0, 0}, {0, 3, 0}, {0, 0, 3}, {3, 0, 0}});
    });
    compare::add("obj_convex_flat", [] { return ConvexPolyhedron({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}}); });
    compare::add("obj_convex_aligned", [] {
        return ConvexPolyhedron({{1, 2, 3}, {4, 1, 2}, {2, 5, 1}, {3, 3, 4}, {0, 1, 1}, {5, 4, 3}}, Rotation(10, 20, 30), Align::MIN);
    });
    compare::add("obj_edge_cylinder_arc", [] { return Cylinder(2, 5, 90); });
    compare::add("obj_edge_sphere_parts", [] { return Sphere(3, -45, 30, 200); });
    compare::add("obj_edge_torus_partial", [] { return Torus(5, 1, 30, 270, 180); });
    compare::add("obj_edge_wedge", [] { return Wedge(4, 3, 2, 1, 0.5, 3, 1.5); });
    compare::add("obj_edge_cone_point", [] { return Cone(3, 0, 4); });
    compare::add("obj_edge_text_empty", [] { return Text("", 5); });
    compare::add("obj_edge_text_lines", [] { return Text("a\nb", 5, {.text_align = {TextAlign::LEFT, TextAlign::TOPFIRSTLINE}}); });
    compare::add("obj_edge_bezier_weights", [] { return Bezier({{0, 0}, {1, 1}, {2, 0}}, {1, 2}); });
    compare::add("obj_edge_bspline_bad", [] { return BSpline({{0, 0}, {1, 1}, {2, 0}}, {0, 1}, 2); });
    compare::add("obj_edge_blend_c0", [] {
        return BlendCurve(Line({0, 0}, {1, 0}), Line({2, 1}, {3, 1}), ContinuityLevel::C0);
    });
    compare::add("obj_edge_ellstart_none", [] { return EllipticalStartArc({0, 0}, {1, 0}, 4, 2, 90); });
    compare::add("obj_edge_ellstart_dir", [] {
        return EllipticalStartArc({0, 0}, {1, 1}, 4, 2, 120, 30.0, Vector(0, 1));
    });

    // ---- drafting
    compare::add_value("obj_draft_numbers", [] {
        const Draft d2({.decimal_precision = 2});
        const Draft whole({.decimal_precision = -1});
        const Draft zero({.decimal_precision = 0});
        const Draft inch({.unit = Unit::IN});
        const Draft frac({.unit = Unit::IN, .number_display = NumberDisplay::FRACTION, .fractional_precision = 64});
        const Draft frac16({.unit = Unit::IN, .number_display = NumberDisplay::FRACTION, .fractional_precision = 16});
        const Draft no_units({.display_units = false, .decimal_precision = 3});
        const Draft cm({.unit = Unit::CM});
        return V::list({d2._number_with_units(3.141),
                        d2._number_with_units(3.149),
                        d2._number_with_units(0),
                        d2._number_with_units(3.14, 0.01),
                        d2._number_with_units(3.14, {0.01, 0}),
                        d2._number_with_units(2.675),
                        d2._number_with_units(0.125),
                        d2._number_with_units(0.375),
                        d2._number_with_units(-0.001),
                        d2._number_with_units(-12.345678),
                        d2._number_with_units(1e20),
                        d2._number_with_units(3.14, {}, false),
                        d2._number_with_units(3.14, 0.05, true),
                        whole._number_with_units(314.1),
                        whole._number_with_units(315.0),
                        whole._number_with_units(325.0),
                        zero._number_with_units(2.5),
                        zero._number_with_units(3.5),
                        zero._number_with_units(-0.4),
                        inch._number_with_units((5.0 / 8) * IN),
                        inch._number_with_units(10.0, {0.5, 0.25}),
                        frac._number_with_units((5.0 / 8) * IN),
                        frac._number_with_units(M_PI * IN),
                        frac16._number_with_units(M_PI * IN),
                        frac._number_with_units(2 * IN),
                        frac._number_with_units(IN / 128),
                        frac._number_with_units(3 * IN / 128),
                        frac._number_with_units(-1.5 * IN),
                        frac._number_with_units(0.999 * IN),
                        frac._number_with_units(2.5 * IN, 0.125 * IN),
                        no_units._number_with_units(7.0 / 3),
                        cm._number_with_units(12.5),
                        d2._round_to_str(1.005),
                        zero._round_to_str(-0.5)});
    });
    compare::add_value("obj_draft_labels", [] {
        const Draft d0({.decimal_precision = 0});
        const Draft d2({.decimal_precision = 2});
        const Edge arc1 = Edge::make_circle(100, Plane::XY, 0, 30);
        const Wire arc_wire(std::vector<Edge>{Edge::make_line({10, -5}, {10, 0}), Edge::make_circle(10, Plane::XY, 0, 60)});
        return V::list({d0._label_to_str(std::nullopt, arc1, true, {}),
                        d2._label_to_str(std::nullopt, arc_wire, true, {}),
                        d2._label_to_str(std::nullopt, Edge::make_circle(7, Plane::XY, 10, 123), true, {}),
                        d2._label_to_str("Given", arc1, true, 0.1),
                        d2._label_to_str(std::nullopt, Edge::make_line({0, 0}, {12.5, 0}), false, 0.1),
                        d2._label_to_str(std::nullopt, Polyline({{0, 0}, {3, 0}, {3, 4}}), false, {0.2, 0.1})});
    });
    compare::add_value("obj_draft_label_angle_line", [] {
        return V(Draft({.decimal_precision = 0})._label_to_str(std::nullopt, Edge::make_line({0, 0, 0}, {100, 0, 0}), true, 0));
    });
    compare::add_value("obj_draft_bad_precision", [] { return V(Draft({.fractional_precision = 37}).fractional_precision); });
    compare::add_value("obj_draft_zero_precision", [] { return V(Draft({.fractional_precision = 0}).fractional_precision); });
    compare::add_value("obj_draft_attrs", [] {
        const Draft d({.font_size = 4, .unit = Unit::IN, .number_display = NumberDisplay::FRACTION, .fractional_precision = 32});
        return V::list({d.font_size, d.font, "REGULAR", d.head_type == HeadType::CURVED ? "CURVED" : "?", d.arrow_length,
                        d.line_width, d.pad_around_text, d.unit == Unit::IN ? "IN" : "?",
                        d.number_display == NumberDisplay::FRACTION ? "FRACTION" : "?", d.display_units, d.decimal_precision,
                        d.fractional_precision, d.extension_gap, d.is_metric(), Draft().is_metric(),
                        Draft({.unit = Unit::M}).is_metric(), Draft::unit_LUT.at(true), Draft::unit_LUT.at(false)});
    });
    compare::add_value("obj_draft_paths", [] {
        std::vector<V> out;
        for (const PathDescriptor& path : std::vector<PathDescriptor>{
                 PathDescriptor({{0, 0, 0}, {100, 0, 0}}), PathDescriptor({{0, 0}, {5, 0}, {5, 5}, {0, 9}}),
                 Edge::make_line({1, 2}, {3, 4}), Polyline({{0, 0}, {1, 0}, {1, 1}})}) {
            const auto p = Draft::_process_path(path);
            std::visit([&](const auto& x) {
                out.push_back(V::list({x.shape_type() == ShapeType::WIRE ? "Wire" : "Edge", x.length(), x.position_at(0),
                                       x.position_at(1)}));
            }, p);
        }
        return V::list(out);
    });
    compare::add_value("obj_draft_sketch_location", [] {
        std::vector<V> out;
        const auto add = [&](const Mixin1D& path, double u, bool flip) {
            const Location loc = Draft::_sketch_location(path, u, flip);
            out.push_back(V::list({loc.position(), loc.orientation()}));
        };
        add(Edge::make_line({0, 0}, {10, 0}), 0.5, false);
        add(Edge::make_line({0, 0}, {10, 5}), 0.25, true);
        add(Edge::make_circle(5, Plane::XY, 0, 180), 0.5, false);
        add(Polyline({{0, 0}, {10, 0}, {10, 10}}), 0.75, true);
        add(Edge::make_line({0, 0}, {10, 0}), -0.4, false);
        add(Edge::make_line({0, 0}, {10, 0}), 1.3, true);
        return V::list(out);
    });
    compare::add("obj_draft_arrowhead_curved", [] { return ArrowHead(10, HeadType::CURVED); });
    compare::add("obj_draft_arrowhead_filleted", [] { return ArrowHead(10, HeadType::FILLETED); });
    compare::add("obj_draft_arrowhead_straight", [] { return ArrowHead(10, HeadType::STRAIGHT); });
    compare::add("obj_draft_arrowhead_rotated", [] { return ArrowHead(4, HeadType::FILLETED, 120); });
    compare::add("obj_draft_arrow_start", [] { return Arrow(3, Edge::make_line({0, 0}, {20, 0}), 0.5); });
    compare::add("obj_draft_arrow_end", [] { return Arrow(3, Edge::make_line({0, 0}, {20, 10}), 0.5, false); });
    compare::add("obj_draft_arrow_wire", [] {
        return Arrow(2, Polyline({{0, 0}, {10, 0}, {10, 10}}), 0.3, false, HeadType::STRAIGHT);
    });
    compare::add("obj_draft_arrow_arc", [] {
        return Arrow(2, Edge::make_circle(10, Plane::XY, 0, 120), 0.4, true, HeadType::FILLETED);
    });
    compare::add("obj_draft_dim_two_points", [] { return DimensionLine({{0, 0, 0}, {100, 0, 0}}, metric()); });
    compare::add("obj_draft_dim_three_points", [] { return DimensionLine({{0, 0, 0}, {50, 0, 0}, {50, 50, 0}}, metric()); });
    compare::add("obj_draft_dim_edge", [] { return DimensionLine(Edge::make_line({0, 0}, {100, 0}), metric()); });
    compare::add("obj_draft_dim_vertices", [] {
        return DimensionLine(std::vector<Vertex>(Edge::make_line({0, 0}, {100, 0}).vertices()), metric());
    });
    compare::add("obj_draft_dim_label", [] { return DimensionLine({{0, 0, 0}, {100, 0, 0}}, metric(), {.label = "Test"}); });
    compare::add("obj_draft_dim_outside_arrows", [] { return DimensionLine({{0, 0, 0}, {15, 0, 0}}, metric()); });
    compare::add("obj_draft_dim_outside_label", [] { return DimensionLine({{0, 0, 0}, {5, 0, 0}}, metric()); });
    compare::add("obj_draft_dim_single_outside", [] {
        return DimensionLine({{0, 0, 0}, {5, 0, 0}}, metric(), {.arrows = {false, true}});
    });
    compare::add("obj_draft_dim_first_arrow", [] {
        return DimensionLine({{0, 0, 0}, {40, 0, 0}}, metric(), {.arrows = {true, false}});
    });
    compare::add("obj_draft_dim_no_arrows", [] {
        return DimensionLine({{0, 0, 0}, {5, 0, 0}}, metric(), {.arrows = {false, false}});
    });
    compare::add("obj_draft_dim_vertical", [] { return DimensionLine({{0, 0}, {0, 100}}, Draft()); });
    compare::add("obj_draft_dim_reversed", [] { return DimensionLine({{60, 10}, {0, 10}}, metric()); });
    compare::add("obj_draft_dim_diagonal", [] { return DimensionLine({{0, 0}, {30, -40}}, metric(), {.tolerance = 0.05}); });
    compare::add("obj_draft_dim_tolerance_pair", [] {
        return DimensionLine({{0, 0}, {80, 0}}, metric(), {.tolerance = {0.1, 0.2}});
    });
    compare::add("obj_draft_dim_arc_angle", [] {
        return DimensionLine(Edge::make_circle(20, Plane::XY, 0, 75), metric(), {.label_angle = true});
    });
    compare::add("obj_draft_dim_wire", [] { return DimensionLine(Polyline({{0, 0}, {30, 0}, {30, 30}}), metric()); });
    compare::add("obj_draft_dim_imperial", [] {
        return DimensionLine({{0, 0}, {3 * IN, 0}}, Draft({.font_size = 5.0, .unit = Unit::IN, .number_display = NumberDisplay::FRACTION}));
    });
    compare::add("obj_draft_dim_avoid_sketch", [] {
        return DimensionLine({{45, 45}, {-45, -45}}, metric(), {.sketch = Rectangle(100, 100) - Rectangle(90, 90)});
    });
    compare::add("obj_draft_dim_padding", [] {
        return DimensionLine(Edge::make_line({0, 0}, {Text("Test", 3.0).bounding_box().size.X + 4, 0}), metric(),
                             {.label = "Test", .arrows = {false, true}});
    });
    compare::add("obj_draft_dim_min_shaft", [] {
        return DimensionLine(Edge::make_line({0, 0}, {Text("Test", 3.0).bounding_box().size.X + 7, 0}), metric(),
                             {.label = "Test"});
    });
    compare::add("obj_draft_dim_singleline", [] {
        return DimensionLine({{0, 0}, {50, 0}}, Draft({.font = "singleline", .head_type = HeadType::STRAIGHT}));
    });
    compare::add("obj_draft_ext_min_x", [] { return ExtensionLine(test_sketch_outer().edges().sort_by(Axis::X)[0], 10, metric()); });
    compare::add("obj_draft_ext_zero", [] { return ExtensionLine(test_sketch_outer().edges().sort_by(Axis::X)[0], 0, metric()); });
    compare::add("obj_draft_ext_sketch", [] {
        return ExtensionLine(test_sketch_outer().edges().sort_by(Axis::Y)[0], 10, metric(), {.sketch = test_sketch_shape()});
    });
    const auto diagonal = [] { return Edge::make_line({100, 100}, {200, 200}); };
    compare::add("obj_draft_ext_vertical_out", [=] {
        return ExtensionLine(diagonal(), -10, metric(), {.measurement_direction = Vector(0, 1, 0)});
    });
    compare::add("obj_draft_ext_vertical_in", [=] {
        return ExtensionLine(diagonal(), 10, metric(), {.measurement_direction = Vector(0, 1, 0)});
    });
    compare::add("obj_draft_ext_otherside", [=] {
        return ExtensionLine(diagonal(), 110, metric(), {.measurement_direction = Vector(0, 1, 0)});
    });
    compare::add("obj_draft_ext_vertical_line", [] {
        return ExtensionLine(Edge::make_line({100, 100}, {100, 200}), 10, metric(), {.measurement_direction = Vector(0, 1, 0)});
    });
    compare::add("obj_draft_ext_horizontal", [=] {
        return ExtensionLine(diagonal(), 10, metric(), {.measurement_direction = Vector(1, 0, 0)});
    });
    compare::add("obj_draft_ext_angled", [=] {
        return ExtensionLine(diagonal(), 10, metric(), {.measurement_direction = Vector(1, 1, 0)});
    });
    compare::add("obj_draft_ext_half_circle", [] {
        return ExtensionLine(Edge::make_circle(50, Plane::XY, 0, 180), -10, metric(), {.measurement_direction = Vector(1, 0, 0)});
    });
    compare::add("obj_draft_ext_full_circle", [] {
        return ExtensionLine(Edge::make_circle(50), 10, metric(), {.measurement_direction = Vector(0, 1, 0)});
    });
    compare::add("obj_draft_ext_vector_above", [] { return ExtensionLine(Edge::make_line({0, 0}, {40, 0}), Vector(0, 10), metric()); });
    compare::add("obj_draft_ext_vector_below", [] { return ExtensionLine(Edge::make_line({0, 0}, {40, 0}), Vector(0, -10), metric()); });
    compare::add("obj_draft_ext_vector_right", [] { return ExtensionLine(Edge::make_line({0, 0}, {0, 40}), Vector(10, 0), metric()); });
    compare::add("obj_draft_ext_vector_left", [] { return ExtensionLine(Edge::make_line({0, 0}, {0, 40}), Vector(-10, 0), metric()); });
    compare::add("obj_draft_ext_scalar_up", [] { return ExtensionLine(Edge::make_line({0, 0}, {40, 0}), -10, metric()); });
    compare::add("obj_draft_ext_vector_infers", [] { return ExtensionLine(Edge::make_line({0, 0}, {10, 10}), Vector(0, 5), metric()); });
    compare::add("obj_draft_ext_vector_measure", [] {
        return ExtensionLine(Edge::make_line({0, 0}, {10, 10}), Vector(0, 5), metric(), {.measurement_direction = Vector(1, 0, 0)});
    });
    compare::add("obj_draft_ext_vector_parallel", [] {
        return ExtensionLine(Edge::make_line({0, 0}, {10, 10}), Vector(5, 0), metric(), {.measurement_direction = Vector(1, 0, 0)});
    });
    compare::add("obj_draft_ext_vector_zero", [] { return ExtensionLine(Edge::make_line({0, 0}, {40, 0}), Vector(0, 0, 0), metric()); });
    compare::add("obj_draft_ext_vector_out_of_plane", [] {
        return ExtensionLine(Edge::make_line({0, 0}, {40, 0}), Vector(0, 10, 0.1), metric());
    });
    compare::add("obj_draft_ext_direction_out_of_plane", [] {
        return ExtensionLine(Edge::make_line({0, 0}, {40, 40}), Vector(0, 10), metric(), {.measurement_direction = Vector(1, 0, 0.1)});
    });
    compare::add("obj_draft_ext_direction_zero", [] {
        return ExtensionLine(Edge::make_line({0, 0}, {40, 40}), 10, metric(), {.measurement_direction = Vector(0, 0, 0)});
    });
    compare::add("obj_draft_ext_same_ends", [] { return ExtensionLine(Edge::make_circle(5), 3, metric()); });
    compare::add("obj_draft_ext_points_label", [] {
        return ExtensionLine({{0, 0}, {20, 0}, {20, 15}}, 8, metric(), {.label = "W", .arrows = {true, false}});
    });
    compare::add("obj_draft_ext_tolerance", [] {
        return ExtensionLine({{0, 0}, {60, 0}}, -12, metric(), {.tolerance = {0.1, 0.05}});
    });
    compare::add_value("obj_draft_dimension_values", [=] {
        const Draft d = metric();
        return V::list({DimensionLine({{0, 0, 0}, {100, 0, 0}}, d).dimension, DimensionLine({{0, 0, 0}, {15, 0, 0}}, d).dimension,
                        DimensionLine(Edge::make_circle(20, Plane::XY, 0, 75), d, {.label_angle = true}).dimension,
                        ExtensionLine(diagonal(), -10, d, {.measurement_direction = Vector(0, 1, 0)}).dimension,
                        ExtensionLine(diagonal(), 10, d, {.measurement_direction = Vector(1, 1, 0)}).dimension,
                        ExtensionLine(Edge::make_circle(50, Plane::XY, 0, 180), -10, d, {.measurement_direction = Vector(1, 0, 0)})
                            .dimension,
                        ExtensionLine(Edge::make_line({0, 0}, {10, 10}), Vector(0, 5), d).dimension});
    });
    compare::add("obj_draft_drawing_a4", [] { return TechnicalDrawing({.design_date = Date(2024, 3, 15)}); });
    compare::add("obj_draft_drawing_a3", [] {
        return TechnicalDrawing({.designed_by = "me", .design_date = Date(2023, 11, 9), .page_size = PageSize::A3,
                                 .title = "Bracket", .sub_title = "Left side", .drawing_number = "B-7", .sheet_number = 2,
                                 .drawing_scale = 0.5, .nominal_text_size = 8, .line_width = 0.35});
    });
    compare::add("obj_draft_drawing_letter", [] {
        return TechnicalDrawing({.design_date = Date(2020, 1, 2), .page_size = PageSize::LETTER, .sheet_number = 12});
    });
    compare::add_value("obj_draft_date", [] {
        return V::list({Date(2024, 3, 15).isoformat(), Date(1, 1, 1).isoformat(), Date(2024, 2, 29).isoformat()});
    });
    compare::add_value("obj_draft_bad_date", [] { return V(Date(2023, 2, 29).isoformat()); });
    compare::add_value("obj_draft_today", [] { return V(Date::today().isoformat()); });

    // ---- text and fonts (the cases that change the font database last)
    compare::add_value("obj_text_available_fonts", [] { return font_info_values(available_fonts()); });
    compare::add_value("obj_text_font_info_repr", [] {
        return V::list({FontInfo{"Arial", {FontStyle::REGULAR, FontStyle::BOLD, FontStyle::ITALIC, FontStyle::BOLDITALIC}}.repr(),
                        FontInfo{"O'Neil", {FontStyle::BOLD}}.repr(), FontInfo{"Say \"hi\"", {}}.repr(),
                        FontInfo{"Tab\tFont", {FontStyle::ITALIC, FontStyle::REGULAR}}.repr()});
    });
    compare::add_value("obj_text_resolve", [] {
        const std::vector<std::pair<std::string, FontStyle>> requests{
            {"Arial", FontStyle::REGULAR},       {"Arial", FontStyle::BOLD},
            {"Arial", FontStyle::ITALIC},        {"Arial", FontStyle::BOLDITALIC},
            {"singleline", FontStyle::REGULAR},  {"singleline", FontStyle::ITALIC},
            {"Relief SingleLine CAD", FontStyle::REGULAR}, {"DejaVu Sans Mono", FontStyle::BOLDITALIC},
            {"Liberation Serif", FontStyle::ITALIC}, {"nonexistent", FontStyle::REGULAR}};
        std::vector<V> out;
        for (const auto& [font, style] : requests) {
            const auto [name, path, system_font] = Compound::resolve_font(font, "", style);
            out.push_back(V::list({name, font_path_value(path)}));
        }
        return V::list(out);
    });
    compare::add("obj_text_singleline", [] { return Text("Hello", 10, {.font = "singleline"}); });
    compare::add("obj_text_singleline_width", [] {
        return Text("build123d", 6, {.font = "singleline", .single_line_width = 0.25});
    });
    compare::add("obj_text_singleline_bold", [] {
        return Text("Ab", 6, {.font = "singleline", .font_style = FontStyle::BOLD});
    });
    compare::add("obj_text_singleline_wires", [] { return Compound::make_text("Hello World", 10, "singleline"); });
    compare::add_value("obj_text_singleline_wire_types", [] {
        return top_level_values(Compound::make_text("Hello World", 10, "singleline"));
    });
    compare::add("obj_text_singleline_triad_x", [] {
        return Compound::make_text("X", 2.5, "singleline", "", FontStyle::REGULAR, {TextAlign::CENTER, TextAlign::CENTER},
                                   Align2(Align::MIN, Align::CENTER));
    });
    compare::add("obj_text_singleline_triad_z", [] {
        return Compound::make_text("Z", 2.5, "singleline", "", FontStyle::REGULAR, {TextAlign::CENTER, TextAlign::CENTER},
                                   Align2(Align::CENTER, Align::MIN));
    });
    compare::add("obj_text_singleline_joint", [] {
        return Compound::make_text("Y", 0.4, "singleline", "", FontStyle::REGULAR, {TextAlign::CENTER, TextAlign::CENTER},
                                   Align2(Align::CENTER, Align::CENTER));
    });
    compare::add("obj_text_singleline_outline", [] {
        return Compound::make_text("Wire", 8, "singleline", "", FontStyle::REGULAR, {TextAlign::CENTER, TextAlign::CENTER},
                                   std::nullopt, 0.0, std::nullopt, 0.5);
    });
    compare::add("obj_text_singleline_path", [] {
        return Text("Curved", 4, {.font = "singleline", .path = Wire(CenterArc({0, 0}, 20, 30, 120)), .position_on_path = 0.1});
    });
    compare::add("obj_text_path_edge", [] {
        return Text("Curved", 4, {.path = CenterArc({0, 0}, 20, 30, 120), .position_on_path = 0.1});
    });
    compare::add("obj_text_path_wire", [] {
        return Text("Curved", 4, {.path = Wire(CenterArc({0, 0}, 20, 30, 120)), .position_on_path = 0.1});
    });
    compare::add("obj_text_path_spline_edge", [] {
        return Text("Spline path", 3, {.path = Spline({{0, 0}, {10, 5}, {20, -3}, {30, 0}}), .position_on_path = 0.05});
    });
    compare::add("obj_text_path_polyline", [] {
        return Compound::make_text("Along", 3, "Arial", "", FontStyle::REGULAR, {TextAlign::CENTER, TextAlign::CENTER},
                                   std::nullopt, 0.2, Polyline({{0, 0}, {10, 0}, {15, 8}}));
    });
    compare::add("obj_text_singleline_wide", [] { return Text("Hi", 5, {.font = "singleline", .single_line_width = 3}); });
    compare::add("obj_text_singleline_too_wide", [] {
        return Text("ii", 5, {.font = "singleline", .single_line_width = 20});
    });
    compare::add("obj_text_bad_width", [] { return Text("Hi", 5, {.single_line_width = 0}); });
    compare::add("obj_text_bad_width_negative", [] {
        return Text("Hi", 5, {.font = "singleline", .single_line_width = -1.5});
    });
    compare::add("obj_text_italic", [] { return Text("Hi", 10, {.font_style = FontStyle::ITALIC}); });
    compare::add("obj_text_bolditalic", [] { return Text("Hi", 10, {.font_style = FontStyle::BOLDITALIC}); });
    compare::add("obj_text_mono_bolditalic", [] {
        return Text("Mono", 7, {.font = "DejaVu Sans Mono", .font_style = FontStyle::BOLDITALIC});
    });
    compare::add_value("obj_text_attrs", [] {
        const Text t("Hi", 8, {.font_style = FontStyle::BOLD, .text_align = {TextAlign::LEFT, TextAlign::BOTTOM},
                               .align = Align2(Align::MIN, Align::MAX), .position_on_path = 0.25, .rotation = 15});
        const Text s("Hey", 5, {.font = "singleline", .single_line_width = 0.3});
        return V::obj({{"txt", t.txt},
                       {"font_size", t.font_size},
                       {"font", t.font},
                       {"font_path", font_path_value(t.font_path)},
                       {"font_style", "BOLD"},
                       {"text_align", V::list({"LEFT", "BOTTOM"})},
                       {"align", align_values(*t.align)},
                       {"position_on_path", t.position_on_path},
                       {"single_line_width", t.single_line_width},
                       {"rotation", t.rotation},
                       {"singleline", V::list({s.font, font_path_value(s.font_path), s.single_line_width})}});
    });
    compare::add_value("obj_text_styles_attrs", [] {
        std::vector<V> out;
        for (const std::string font : {"Arial", "DejaVu Sans Mono", "singleline"}) {
            for (const auto style : {FontStyle::REGULAR, FontStyle::BOLD, FontStyle::ITALIC, FontStyle::BOLDITALIC}) {
                const Text t("Ab", 4, {.font = font, .font_style = style});
                out.push_back(V::list({t.font, font_path_value(t.font_path)}));
            }
        }
        return V::list(out);
    });
    compare::add("obj_text_align_error_h", [] {
        return Compound::make_text("x", 5, "Arial", "", FontStyle::REGULAR, {TextAlign::TOP, TextAlign::CENTER});
    });
    compare::add("obj_text_align_error_v", [] {
        return Compound::make_text("x", 5, "Arial", "", FontStyle::REGULAR, {TextAlign::LEFT, TextAlign::RIGHT});
    });
    compare::add("obj_text_font_path", [] { return Text("Hi", 10, {.font_path = kDejaVu + "DejaVuSans-Bold.ttf"}); });
    compare::add_value("obj_text_font_path_attrs", [] {
        const Text a("Hi", 6, {.font_path = kDejaVu + "DejaVuSans-Bold.ttf"});
        const Text b("Hi", 6, {.font = "DejaVu Sans Mono", .font_path = kDejaVu + "DejaVuSansMono.ttf"});
        const Text c("Hi", 6, {.font = "nothing", .font_path = kDejaVu + "DejaVuSerif.ttf", .font_style = FontStyle::BOLD});
        return V::list({V::list({a.font, a.font_path}), V::list({b.font, b.font_path}), V::list({c.font, c.font_path})});
    });
    compare::add_value("obj_text_check_find", [] {
        FontManager manager;
        const auto good = manager.check_font(bundled_font());
        const auto bad = manager.check_font("reliefsingleline/ReliefSingleLineCAD-Regular.ttf");
        const auto found = manager.find_font("Relief SingleLine CAD", FontStyle::REGULAR);
        const auto other = manager.find_font("build123d", FontStyle::REGULAR);
        return V::list({font_path_value(good->FontPath(Font_FA_Regular).ToCString()), bad.IsNull(),
                        std::string(found->FontName().ToCString()), found->IsSingleStrokeFont(),
                        std::string(other->FontName().ToCString()) != "build123d"});
    });
    compare::add_value("obj_text_register_fonts", [] {
        FontManager manager;
        std::vector<V> out;
        for (const auto& path : {bundled_font(), kDejaVu + "DejaVuSans-Bold.ttf",
                                 std::string("/usr/share/fonts/truetype/ubuntu/Ubuntu[wdth,wght].ttf"),
                                 std::string("/usr/share/fonts/truetype/ubuntu/UbuntuMono-Italic[wght].ttf"),
                                 std::string("/usr/share/fonts/opentype/urw-base35/URWBookman-DemiItalic.otf")}) {
            out.push_back(V::list_of(manager.register_font(path)));
        }
        out.push_back(system_font_values(manager.find_font("Ubuntu Condensed", FontStyle::BOLD), FontStyle::BOLD));
        out.push_back(system_font_values(manager.find_font("Ubuntu Mono Medium", FontStyle::ITALIC), FontStyle::ITALIC));
        return V::list(out);
    });
    compare::add_value("obj_text_register_single_stroke", [] {
        FontManager manager;
        const auto names = manager.register_font(kDejaVu + "DejaVuSansMono-Bold.ttf", true, true);
        const auto font = manager.find_font(names.at(0), FontStyle::BOLD);
        const V result = V::list({V::list_of(names), font->IsSingleStrokeFont(),
                                  Compound::make_text("A", 5, names.at(0), "", FontStyle::BOLD).edges().size()});
        manager.register_font(kDejaVu + "DejaVuSansMono-Bold.ttf", true, false);
        return result;
    });
    compare::add_value("obj_text_register_folder", [] {
        auto names = FontManager().register_folder(kDejaVu);
        std::sort(names.begin(), names.end());
        return V::list_of(names);
    });
    compare::add_value("obj_text_register_corrupt", [] {
        const std::string path = temp_file("build123d_corrupt.ttf");
        std::ofstream(path, std::ios::binary) << "not a font";
        return V::list_of(FontManager().register_font(path));
    });
    compare::add_value("obj_text_register_missing", [] {
        return V::list_of(FontManager().register_font("/tmp/build123d_missing_font.ttf"));
    });
    compare::add_value("obj_text_register_collection_as_ttf", [] {
        const std::string path = temp_file("build123d_collection_as.ttf");
        write_collection({bundled_font(), kDejaVu + "DejaVuSans-Bold.ttf"}, path);
        return V::list_of(FontManager().register_font(path));
    });
    compare::add_value("obj_text_available_fonts_after", [] { return font_info_values(available_fonts()); });
    compare::add_value("obj_text_system_fonts_rebuilt", [] {
        // As upstream test_register_system_fonts: the database rebuilt from build123d's own reading
        FontManager manager;
        manager.manager->RemoveFontAlias(TCollection_AsciiString("singleline"), TCollection_AsciiString("Relief SingleLine CAD"));
        manager.manager->ClearFontDataBase();
        manager.register_system_fonts();
        return font_info_values(FontManager().available_fonts());
    });
}
