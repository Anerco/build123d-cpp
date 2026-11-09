// Joints and the assembly tree, compared with cases_joints_py.py.
#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

V loc_values(const Location& l) { return V::obj({{"position", l.position()}, {"orientation", l.orientation()}}); }

Part arm() { return Box(1, 2, 6); }

Part block() { return Pos(1, 2, 3) * Rot(10, 20, 30) * Box(10, 10, 10); }

// A labelled tree: root(a, sub(b, c)) with locations at every level.
Compound make_tree() {
    Part a = Pos(5, 0, 0) * Box(1, 1, 1);
    a.label = "a";
    Part b = Pos(0, 5, 0) * Sphere(1);
    b.label = "b";
    Part c = Rot(0, 0, 45) * Pos(0, 0, 5) * Cylinder(1, 2);
    c.label = "c";
    Compound sub = Compound::make_assembly({b, c}, "sub");
    sub.move(Location(Vector(10, 0, 0), Vector(0, 0, 90)));
    Compound root = Compound::make_assembly({a, sub}, "root");
    root.move(Location(Vector(0, 0, 7), Vector(30, 0, 0)));
    return root;
}

std::vector<std::string> labels(const std::vector<const Shape*>& nodes) {
    std::vector<std::string> out;
    for (const auto* n : nodes) out.push_back(n->label);
    return out;
}

}  // namespace

COMPARE_CASES {
    compare::add_value("joints_rigid", [] {
        Part base = block();
        Part other = arm();
        RigidJoint j1("a", base, Location(Vector(5, 0, 0), Vector(0, 90, 0)));
        RigidJoint j2("b", other, Location(Vector(0, 0, -3)));
        const Location rel = j1.relative_to(j2);
        j1.connect_to(j2);
        return V::obj({{"relative", loc_values(rel)},
                       {"other", loc_values(other.location())},
                       {"j1", loc_values(j1.location())},
                       {"j2", loc_values(j2.location())},
                       {"connected", j1.connected_to().has_value() && *j1.connected_to() == j2}});
    });
    compare::add_value("joints_revolute", [] {
        Part base = block();
        Part other = arm();
        RevoluteJoint hinge("h", base, Axis(Vector(5, 5, 0), Vector(0, 0, 1)), std::nullopt, {0, 270});
        RigidJoint pin("p", other, Location(Vector(0, 1, 0)));
        std::vector<V> out;
        for (double angle : {0.0, 45.0, 270.0}) {
            hinge.connect_to(pin, angle);
            out.push_back(loc_values(other.location()));
        }
        out.push_back(loc_values(hinge.location()));
        out.push_back(*hinge.angle());
        return V::list(out);
    });
    compare::add_value("joints_revolute_reference", [] {
        Part base = block();
        Part other = arm();
        RevoluteJoint hinge("h", base, Axis(Vector(0, 0, 0), Vector(1, 0, 0)), Vector(0, 1, 1), {0, 360});
        RigidJoint pin("p", other, Location(Vector(0, 0, 3), Vector(0, 90, 0)));
        hinge.connect_to(pin, 30);
        const V first = loc_values(other.location());
        pin.connect_to(hinge, 120);  // the other way round: base moves
        return V::list({first, loc_values(base.location()), hinge.angle_reference()});
    });
    compare::add_value("joints_linear", [] {
        Part base = block();
        Part slider = arm();
        Part pinned = Box(1, 1, 1);
        LinearJoint slide("s", base, Axis(Vector(0, 0, 5), Vector(1, 0, 0)), {-5, 5});
        RigidJoint s("r", slider, Location(Vector(0, 0, -3)));
        RevoluteJoint pin("p", pinned, Axis::Z, std::nullopt, {0, 180});
        slide.connect_to(s);  // default: middle of the range
        const V middle = loc_values(slider.location());
        slide.connect_to(s, 4);
        const V at4 = loc_values(slider.location());
        slide.connect_to(pin, -2, 135);
        return V::list({middle, at4, loc_values(pinned.location()), *slide.position(), *slide.angle()});
    });
    compare::add_value("joints_cylindrical", [] {
        Part base = block();
        Part screw = arm();
        CylindricalJoint hole("c", base, Axis(Vector(0, -5, 0), Vector(0, 1, 0)), std::nullopt, {-10, 10},
                              {0, 180});
        RigidJoint tip("t", screw, Location(Vector(0, 0, 3)));
        hole.connect_to(tip);  // defaults: middle of both ranges
        const V defaults = loc_values(screw.location());
        hole.connect_to(tip, -1, 90);
        return V::list({defaults, loc_values(screw.location()), *hole.position(), *hole.angle()});
    });
    compare::add_value("joints_ball", [] {
        Part base = block();
        Part ball = Sphere(1);
        BallJoint socket("b", base, Location(Vector(-5, 0, 0)), {{{0, 360}, {0, 360}, {0, 360}}});
        RigidJoint knob("k", ball, Location(Vector(0, 0, 1)));
        socket.connect_to(knob);
        const V defaults = loc_values(ball.location());
        socket.connect_to(knob, Rotation(10, 20, 30));
        return V::list({defaults, loc_values(ball.location()), loc_values(socket.location())});
    });
    compare::add_value("joints_ball_out_of_range", [] {
        Part base = block();
        Part ball = Sphere(1);
        BallJoint socket("b", base, Location(Vector(-5, 0, 0)), {{{0, 360}, {0, 360}, {0, 360}}}, Plane::XZ);
        RigidJoint knob("k", ball, Location(Vector(0, 0, 1)));
        socket.connect_to(knob, Rotation(10, 20, 30));
        return V(true);
    });
    compare::add_value("joints_rigid_to_others", [] {
        Part base = block();
        Part a = arm(), b = arm(), c = arm(), d = arm();
        RigidJoint r("r", base, Location(Vector(0, 0, 5)));
        RevoluteJoint rev("rev", a, Axis::X, std::nullopt, {0, 90});
        LinearJoint lin("lin", b, Axis::Y, {0, 4});
        CylindricalJoint cyl("cyl", c, Axis::Z, std::nullopt, {0, 2}, {0, 90});
        BallJoint ball("ball", d);
        r.connect_to(rev, 30);
        r.connect_to(lin, 3);
        r.connect_to(cyl, 1, 45);
        r.connect_to(ball, Rotation(5, 10, 15));
        return V::list({loc_values(a.location()), loc_values(b.location()), loc_values(c.location()),
                        loc_values(d.location())});
    });
    compare::add_value("joints_generic_args", [] {
        Part base = block();
        Part other = arm();
        RevoluteJoint("h", base, Axis::Y, std::nullopt, {0, 180});
        RigidJoint("p", other, Location(Vector(1, 0, 0)));
        base.joints["h"].connect_to(other.joints["p"], {.angle = 60});
        return V::obj({{"other", loc_values(other.location())},
                       {"keys", V::list_of(base.joints.keys())},
                       {"relative", loc_values(base.joints["h"].relative_to(other.joints["p"], {.angle = 10}))}});
    });
    compare::add_value("joints_angle_out_of_range", [] {
        Part base = block(), other = arm();
        RevoluteJoint hinge("h", base, Axis::Z, std::nullopt, {0, 90});
        RigidJoint pin("p", other);
        hinge.connect_to(pin, 120);
        return V(true);
    });
    compare::add_value("joints_position_out_of_range", [] {
        Part base = block(), other = arm();
        LinearJoint slide("s", base, Axis::Z, {0, 1});
        RigidJoint pin("p", other);
        slide.connect_to(pin, 2);
        return V(true);
    });
    compare::add_value("joints_wrong_type", [] {
        Part base = block(), other = arm();
        RevoluteJoint a("a", base);
        RevoluteJoint b("b", other);
        a.connect_to(b);
        return V(true);
    });
    compare::add_value("joints_bad_angle_reference", [] {
        Part base = block();
        RevoluteJoint("h", base, Axis::Z, Vector(0, 0, 1));
        return V(true);
    });
    compare::add_value("joints_moved_copy", [] {
        Part base = block();
        RigidJoint("a", base, Location(Vector(5, 0, 0)));
        LinearJoint("l", base, Axis::X, {0, 10});
        const Part moved = Pos(0, 0, 20) * Rot(0, 0, 90) * base;
        return V::obj({{"original", loc_values(base.joints["a"].location())},
                       {"moved", loc_values(moved.joints["a"].location())},
                       {"moved_axis", loc_values(moved.joints["l"].location())},
                       {"count", moved.joints.size()}});
    });
    compare::add("joints_symbols", [] {
        Part base = block();
        RevoluteJoint hinge("h", base, Axis(Vector(5, 5, 0), Vector(0, 0, 1)));
        LinearJoint slide("s", base, Axis(Vector(0, 0, 5), Vector(1, 0, 0)), {-5, 5});
        CylindricalJoint hole("c", base, Axis(Vector(0, -5, 0), Vector(0, 1, 0)), std::nullopt, {-10, 10});
        return Compound(std::vector<Shape>{hinge.symbol(), slide.symbol(), hole.symbol()});
    });
    compare::add("joints_symbol_rigid", [] {
        Part base = block();
        RigidJoint corner("r", base, Location(Vector(5, 5, 5), Vector(0, 0, 30)));
        return corner.symbol();
    });
    compare::add("joints_symbol_ball", [] {
        Part base = block();
        BallJoint socket("b", base, Location(Vector(-5, 0, 0)), {{{0, 360}, {0, 360}, {0, 360}}}, Plane::XZ);
        return socket.symbol();
    });
    compare::add_value("tree_navigation", [] {
        const Compound root = make_tree();
        const Shape& sub = root.children[1];
        const Shape& c = sub.children[1];
        return V::obj({{"depth", V::list({root.depth(), sub.depth(), c.depth()})},
                       {"height", V::list({root.height(), sub.height(), c.height()})},
                       {"size", V::list({root.tree_size(), sub.tree_size(), c.tree_size()})},
                       {"is_leaf", V::list({root.is_leaf(), sub.is_leaf(), c.is_leaf()})},
                       {"is_root", V::list({root.is_root(), sub.is_root(), c.is_root()})},
                       {"path", V::list_of(labels(c.path()))},
                       {"ancestors", V::list_of(labels(c.ancestors()))},
                       {"descendants", V::list_of(labels(root.descendants()))},
                       {"leaves", V::list_of(labels(root.leaves()))},
                       {"siblings", V::list_of(labels(c.siblings()))},
                       {"root", c.root().label},
                       {"parent", c.parent()->label}});
    });
    compare::add_value("tree_global_location", [] {
        const Compound root = make_tree();
        std::vector<V> out;
        for (const Shape* node : root.descendants()) {
            out.push_back(V::obj({{"label", node->label},
                                  {"location", loc_values(node->location())},
                                  {"global", loc_values(node->global_location())}}));
        }
        return V::list(out);
    });
    compare::add("tree_children_rebuild", [] {
        Compound assy;
        assy.children = std::vector<Shape>{Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)};
        assy.children.push_back(Pos(0, 4, 0) * Cylinder(1, 3));
        return assy;
    });
    compare::add("tree_children_detach", [] {
        Compound assy = Compound::make_assembly({Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1), Pos(0, 4, 0) * Cylinder(1, 3)});
        assy.children.erase(assy.children.begin() + 1);
        return assy;
    });
}
