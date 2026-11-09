// Unit tests for the core (geometry/topology) parity work: static types and C++-only
// behaviour. Results are compared with Python in tests/compare/cases_core*.
#include <build123d/build123d.hpp>

#include <type_traits>
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
