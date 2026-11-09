// Unit tests for the C++ side of joints and the assembly tree: handle semantics, copies and
// moves, parent links. The geometry is compared with Python in tests/compare/cases_joints*.
#include <build123d/build123d.hpp>

#include <utility>
#include <vector>

#include "test_framework.hpp"

using namespace build123d;

TEST(joint_handles_share_the_joint) {
    Part part = Box(1, 1, 1);
    RigidJoint j("a", part, Location(Vector(1, 0, 0)));
    CHECK(part.joints.size() == 1);
    CHECK(part.joints["a"] == j);
    CHECK(&j.parent() == &part);
    j.set_relative_location(Location(Vector(2, 0, 0)));
    CHECK(part.joints.get<RigidJoint>("a").relative_location().position() == Vector(2, 0, 0));
    CHECK(part.joints["a"].is<RigidJoint>());
    CHECK(!part.joints["a"].is<BallJoint>());
    CHECK_THROWS(part.joints.get<BallJoint>("a"), std::invalid_argument);
    CHECK_THROWS(part.joints["missing"], std::out_of_range);
}

TEST(joints_follow_copies_and_moves) {
    Part part = Box(1, 1, 1);
    RigidJoint j("a", part);
    RevoluteJoint("b", part);
    CHECK((part.joints.keys() == std::vector<std::string>{"a", "b"}));

    Part copy = part;  // the copy gets its own joints
    CHECK(&copy.joints["a"].parent() == &copy);
    CHECK(copy.joints["a"] != j);
    CHECK(&j.parent() == &part);

    Part moved = std::move(part);  // moving keeps the same joints, bound to the new object
    CHECK(moved.joints["a"] == j);
    CHECK(&j.parent() == &moved);

    const Part placed = Pos(0, 0, 5) * moved;  // operations that copy the shape copy the joints
    CHECK(placed.joints.size() == 2);
    CHECK(&placed.joints["b"].parent() == &placed);
    CHECK(placed.joints["a"].location().position() == Vector(0, 0, 5));
}

TEST(joint_of_destroyed_part_is_detached) {
    std::optional<RigidJoint> handle;
    {
        Part part = Box(1, 1, 1);
        handle.emplace("a", part);
    }
    CHECK_THROWS(handle->parent(), std::runtime_error);
    CHECK_THROWS(handle->location(), std::runtime_error);
}

TEST(joint_connect_arguments) {
    Part base = Box(4, 4, 4);
    Part arm = Box(1, 1, 3);
    RevoluteJoint hinge("hinge", base, Axis::Z, std::nullopt, {0, 90});
    RigidJoint pin("pin", arm, Location(Vector(0, 0, -1.5)));
    // RevoluteJoint.connect_to only takes `angle`
    CHECK_THROWS(hinge.connect_to(pin, JointArgs{.position = 1}), std::invalid_argument);
    hinge.connect_to(pin, 45);
    CHECK_NEAR(*hinge.angle(), 45, 0);
    CHECK(pin.connected_to() == std::nullopt);
    CHECK(hinge.connected_to() == std::optional<Joint>(pin));
    // a RigidJoint passes keyword arguments on (and ignores those it doesn't use)
    pin.connect_to(hinge, JointArgs{.position = 3, .angle = 30});
    CHECK_NEAR(*hinge.angle(), 30, 0);
}

TEST(joint_reparent) {
    Part part = Pos(1, 0, 0) * Box(1, 1, 1);
    Part other = Pos(0, 5, 0) * Box(1, 1, 1);
    RigidJoint j("a", part, Location(Vector(2, 0, 0)));
    const Location before = j.location();
    j.reparent(other);
    CHECK(&j.parent() == &other);
    CHECK(other.joints.contains("a"));
    CHECK(!part.joints.contains("a"));
    CHECK(j.location().position() == before.position());
}

TEST(tree_parent_links) {
    Compound assy;
    for (int i = 0; i < 20; ++i) assy.children.push_back(Pos(i * 2.0, 0, 0) * Box(1, 1, 1));  // reallocates
    for (const auto& child : assy.children) CHECK(child.parent() == &assy);
    CHECK(assy.solids().size() == 20);  // the geometry was rebuilt from the children

    Compound copy = assy;
    for (const auto& child : copy.children) CHECK(child.parent() == &copy);

    const Compound moved = Pos(0, 0, 3) * assy;  // children come along, linked to the new shape
    CHECK(moved.children.size() == 20);
    CHECK(moved.children[0].parent() == &moved);
    CHECK(moved.children[0].global_location().position() == Vector(0, 0, 3));

    Shape detached = assy.children[0];  // a copy is a root
    CHECK(detached.parent() == nullptr);
    CHECK(detached.is_root());

    assy.children.erase(assy.children.begin());
    CHECK(assy.solids().size() == 19);
    assy.children.clear();
    CHECK(assy.solids().empty());
}

TEST(tree_nested_assignment_keeps_links) {
    Compound inner = Compound::make_assembly({Box(1, 1, 1), Sphere(1)}, "inner");
    Compound outer = Compound::make_assembly({inner, Cylinder(1, 2)}, "outer");
    const Shape& stored_inner = outer.children[0];
    CHECK(stored_inner.parent() == &outer);
    CHECK(stored_inner.children[0].parent() == &stored_inner);
    CHECK(&stored_inner.children[1].root() == &outer);
    CHECK(outer.tree_size() == 5);
    CHECK(outer.descendants().size() == 4);
}
