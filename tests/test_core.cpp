// Unit tests for the core (geometry/topology) parity work: static types and C++-only
// behaviour. Results are compared with Python in tests/compare/cases_core*.
#include <build123d/build123d.hpp>

#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "test_framework.hpp"

using namespace build123d;

namespace {

template <class A, class B, class = void>
struct can_add : std::false_type {};
template <class A, class B>
struct can_add<A, B, std::void_t<decltype(std::declval<const A&>() + std::declval<const B&>())>> : std::true_type {};

template <class A, class B, class = void>
struct can_and : std::false_type {};
template <class A, class B>
struct can_and<A, B, std::void_t<decltype(std::declval<const A&>() & std::declval<const B&>())>> : std::true_type {};

}  // namespace

// ---- static result types of &
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Sphere(1)), Part>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & std::vector<Part>{}), Part>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Rectangle(1, 1)), Sketch>);
static_assert(std::is_same_v<decltype(Rectangle(1, 1) & Box(1, 1, 1)), Sketch>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Line({0, 0}, {1, 0})), Curve>);
static_assert(std::is_same_v<decltype(Line({0, 0}, {1, 0}) & Box(1, 1, 1)), Curve>);
static_assert(std::is_same_v<decltype(Line({0, 0}, {1, 0}) & Line({0, 0}, {1, 0})), Compound>);
static_assert(std::is_same_v<decltype(Rectangle(1, 1) & Circle(1)), Compound>);
static_assert(std::is_same_v<decltype(Rectangle(1, 1) & Line({0, 0}, {1, 0})), Compound>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Axis::Z), Curve>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Plane::XY), Sketch>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Vector(0, 0, 0)), Compound>);
static_assert(std::is_same_v<decltype(Box(1, 1, 1) & Pos(0, 0, 0)), Compound>);
static_assert(std::is_same_v<decltype(Axis::Z & Box(1, 1, 1)), ShapeList<Shape>>);
static_assert(std::is_same_v<decltype(Axis::Z & Axis::X), std::optional<std::variant<Vector, Axis>>>);
static_assert(std::is_same_v<decltype(Plane::XY & Plane::XZ), std::optional<std::variant<Axis, Plane>>>);
static_assert(std::is_same_v<decltype(Pos(1, 0, 0) & Plane::XY), std::optional<std::variant<Vector, Location>>>);
static_assert(std::is_same_v<decltype(Vector(1, 0, 0) & Axis::X), std::optional<Vector>>);

// ---- Vertex arithmetic is coordinate arithmetic, never a boolean
static_assert(std::is_same_v<decltype(Vertex() + Vertex()), Vertex>);
static_assert(std::is_same_v<decltype(Vertex() - Vertex()), Vertex>);
static_assert(std::is_same_v<decltype(Vertex() + Vector()), Vertex>);
static_assert(std::is_same_v<decltype(Vertex() - Vector()), Vertex>);

TEST(core_vertex_arithmetic) {
    const Vertex a(1, 2, 3), b(4, 5, 6);
    CHECK(Vector(a + b) == Vector(5, 7, 9));
    CHECK(Vector(b - a) == Vector(3, 3, 3));
    CHECK(Vector(a + Vector(0, 0, 15)) == Vector(1, 2, 18));
    CHECK(Vector(a - Vector(1, 1, 1)) == Vector(0, 1, 2));
    Vertex c = a;
    c += {0, 0, 1};
    c -= Vertex(1, 0, 0);
    CHECK(Vector(c) == Vector(0, 2, 4));
}

TEST(core_and_chains_a_list) {
    // a & [b, c] is a ∩ b ∩ c, not a COMMON with a tool group
    const Part a = Box(4, 4, 4);
    const Part chained = a & std::vector<Part>{Pos(1, 0, 0) * Box(4, 4, 4), Sphere(2.5)};
    const Part step = (a & (Pos(1, 0, 0) * Box(4, 4, 4))) & Sphere(2.5);
    CHECK_NEAR(chained.volume(), step.volume(), 1e-9);
    CHECK(chained.solids().size() == 1);
}

TEST(core_intersect_results) {
    // intersect returns a ShapeList; no intersection is an empty list (Python's None)
    const Part box = Box(2, 2, 2);
    CHECK(box.intersect(Pos(10, 0, 0) * Box(1, 1, 1)).empty());
    const auto hits = box.intersect(Axis::Z);
    CHECK(hits.size() == 1 && hits[0].shape_type() == ShapeType::EDGE);
    // several operands chain
    CHECK(box.intersect({IntersectArg(Pos(1, 0, 0) * Box(2, 2, 2)), IntersectArg(Plane::XY)}).size() == 1);
    CHECK_THROWS(Box(1, 1, 1) & Part(), std::invalid_argument);
    CHECK_THROWS(Part() & Box(1, 1, 1), std::invalid_argument);
}

TEST(core_geometry_intersections) {
    // coaxial axes give the Axis, an axis in a plane gives the Axis, coplanar planes the Plane
    const auto coaxial = Axis::X.intersect(Axis({5, 0, 0}, {1, 0, 0}));
    CHECK(coaxial && std::holds_alternative<Axis>(*coaxial));
    const auto in_plane = Plane::XY.intersect(Axis::X);
    CHECK(in_plane && std::holds_alternative<Axis>(*in_plane));
    const auto coplanar = Plane::XY.intersect(Plane(Vector(1, 2, 0)));
    CHECK(coplanar && std::holds_alternative<Plane>(*coplanar));
    const auto location = Plane::XY & Location(Vector(1, 1, 0));
    CHECK(location && std::holds_alternative<Location>(*location));
    CHECK_THROWS(Plane::XY.shift_origin(Axis::X), std::invalid_argument);
}

TEST(core_hash_containers) {
    // Shapes and geometry objects work as keys of the standard hash containers, with
    // build123d's equality (is_same for shapes, rounded values for geometry)
    const Part box = Box(1, 2, 3);
    std::unordered_set<Edge> edges;
    for (const auto& e : box.edges()) {
        edges.insert(e);
        edges.insert(e.reversed());
    }
    CHECK(edges.size() == 12);
    std::unordered_map<Vector, int> counts;
    for (const auto& v : box.vertices()) ++counts[Vector(v)];
    CHECK(counts.size() == 8);
    const std::unordered_set<Location> locations{Pos(1, 2, 3), Location(Vector(1, 2, 3)),
                                                 Location(Vector(1, 2, 3), Vector(0, 0, 360))};
    CHECK(locations.size() == 1);
    CHECK(std::hash<Shape>{}(box) == box.hash());
    CHECK(Color(1, 0, 0) == Color("red"));
}

TEST(core_split_unpacks) {
    const Part box = Box(2, 2, 2);
    const auto [top, bottom] = box.split(Plane::XY.offset(0.5), Keep::BOTH);
    CHECK(top.size() == 1 && bottom.size() == 1);
    CHECK_NEAR(top[0].volume(), 2.0, 1e-9);
    CHECK_NEAR(bottom[0].volume(), 6.0, 1e-9);
    const SplitResult kept = box.split(Plane::XY.offset(0.5), Keep::BOTH);
    CHECK(kept.size() == 2);  // tops then bottoms
    CHECK(box.split(Plane::XY.offset(0.5)).bottom.empty());
    CHECK(box.split(Plane::XY.offset(-5), Keep::TOP).size() == 1);
    CHECK(box.split(Plane::XY.offset(-5), Keep::BOTTOM).empty());  // build123d's None
    CHECK_THROWS(box.split(Plane::XY, Keep::INSIDE), std::invalid_argument);
}

// group_by keys: numbers (and bools) are rounded doubles, other keys keep their type
static_assert(std::is_same_v<decltype(ShapeList<Solid>().group_by([](const Solid& s) { return s.label; })),
                             GroupBy<Solid, std::string>>);
static_assert(std::is_same_v<decltype(ShapeList<Edge>().group_by([](const Edge& e) { return e.is_closed(); })),
                             GroupBy<Edge, double>>);
static_assert(std::is_same_v<decltype(ShapeList<Edge>().group_by(by_convexity)), GroupBy<Edge, Convexity>>);
static_assert(std::is_same_v<decltype(ShapeList<Edge>().group_by(Axis::Z)), GroupBy<Edge>>);

TEST(core_shapelist_semantics) {
    const Part box = Box(1, 2, 3);
    const auto faces = box.faces();
    // == compares as sets (is_same), like build123d
    CHECK(faces == faces.slice(std::nullopt, std::nullopt, -1));
    CHECK(faces == faces + faces);
    CHECK(faces != faces.slice(1));
    // - and & keep each shape once
    CHECK((faces + faces - faces.slice(0, 2)).size() == 4);
    CHECK(((faces + faces) & faces.slice(0, 2)).size() == 2);
    // expand drops null shapes
    ShapeList<Shape> with_null{Shape(), Vertex(1, 2, 3), Compound(std::vector<Shape>{Wire::make_rect(1, 1)})};
    CHECK(with_null.expand().size() == 5);
    CHECK_THROWS(faces.slice(0, 2, 0), std::invalid_argument);
    // a GroupBy of enum keys looks groups up by the enum
    const Part pocket = Box(10, 10, 10) - Pos(0, 0, 3) * Box(4, 4, 4);
    const auto groups = pocket.edges().group_by(by_convexity);
    CHECK(groups.group(Convexity::CONCAVE).size() == pocket.edges().filter_by(Convexity::CONCAVE).size());
    CHECK_THROWS(groups.group(Convexity::SADDLE), std::out_of_range);
}

// ---- geometry operators with lists of planes
static_assert(std::is_same_v<decltype(Pos(1, 0, 0) * std::vector<Plane>{}), std::vector<Plane>>);
static_assert(std::is_same_v<decltype(std::vector<Location>{} * Plane::XY), std::vector<Plane>>);
static_assert(std::is_same_v<decltype(std::vector<Plane>{} * Plane::XY), std::vector<Plane>>);
static_assert(std::is_same_v<decltype(Plane::XY * std::vector<Plane>{}), std::vector<Location>>);

TEST(core_geometry_helpers) {
    // Color(0) is black (build123d's Color(0) is white: it tests the color code for truth)
    CHECK(Color(0).to_tuple() == (std::array<double, 4>{0, 0, 0, 1}));
    CHECK(Color().to_tuple() == (std::array<double, 4>{1, 1, 1, 1}));
    CHECK_THROWS(Color(1.5, 0, 0), std::runtime_error);
    CHECK_THROWS(Vector(1, 2, 3) / 0.0, std::invalid_argument);
    // BoundBox predicates on void boxes are false
    CHECK(!BoundBox().contains_properly(Box(1, 1, 1).bounding_box()));
    CHECK(!BoundBox().touches(Box(1, 1, 1).bounding_box()));
    // a left-handed gp_Ax3 is made right-handed with a warning
    std::vector<std::string> messages;
    const auto previous = set_warning_handler([&](const std::string& m) { messages.push_back(m); });
    gp_Ax3 left(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
    left.YReverse();
    const Plane fixed(left);
    set_warning_handler(previous);
    CHECK(messages.size() == 1 && messages[0] == "Trying to set a left-handed plane");
    CHECK(fixed.z_dir == Vector(0, 0, 1) && fixed.y_dir == Vector(0, 1, 0));
}
