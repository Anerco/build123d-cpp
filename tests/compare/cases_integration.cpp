// Features that span the work areas, compared with cases_integration_py.py.
#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

template <class L>
double total_length(const L& edges) {
    double s = 0;
    for (const auto& e : edges) s += e.length();
    return s;
}

template <class L>
double total_area(const L& faces) {
    double s = 0;
    for (const auto& f : faces) s += f.area();
    return s;
}

V joint_info(const Part& p) {
    return V::obj({{"keys", V::list_of(p.joints.keys())},
                   {"label", p.label},
                   {"location", p.joints.contains("a") ? V(p.joints["a"].location().position()) : V(nullptr)}});
}

}  // namespace

COMPARE_CASES {
    compare::add_value("integ_material_setter_looks", [] {
        std::vector<V> out;
        for (const FinishedMaterial& material :
             {FinishedMaterial("oak"), metals::aluminum(metals::Alu::G7075_T6, {.finish = finishes::anodize("red")}),
              plastics::petg({.color = "#336699", .process = processes::fdm(0.1)})}) {
            Part part = Box(1, 1, 1);
            part.material = material;  // colors the part with the material's look
            out.push_back(V::list_of(part.color->to_tuple()));
        }
        return V::list(out);
    });
    compare::add_value("integ_version", [] {
        return V::list({V(version), V::list({version_tuple[0], version_tuple[1], version_tuple[2]})});
    });
    compare::add_value("integ_property_setters", [] {
        Axis axis(Vector(0, 0, 0), Vector(1, 0, 0));
        axis.set_direction(Vector(0, 0, 2));
        axis.set_position(Vector(1, 2, 3));
        bool null_direction_rejected = false;
        try {
            axis.set_direction(Vector(0, 0, 0));
        } catch (const std::invalid_argument&) {
            null_direction_rejected = true;
        }
        Part box = Box(1, 2, 3);
        box.set_position(Vector(5, 0, 0));
        box.set_orientation(Vector(0, 0, 90));
        const BoundBox bb = box.bounding_box();
        Plane plane(Vector(0, 0, 0));
        plane.origin = Vector(1, 1, 1);
        plane.set_x_dir(Vector(1, 1, 0));
        Edge edge = Edge::make_line({0, 0}, {1, 0});
        edge.label = "kept";
        edge.set_wrapped(Edge::make_line({0, 0}, {0, 2}).wrapped());
        return V::list({V(axis.direction), V(axis.position), V(null_direction_rejected), V(box.position()),
                        V(box.orientation()), V(bb.min), V(bb.max), V(plane.origin), V(plane.x_dir), V(plane.y_dir),
                        V(plane.z_dir), V(edge.length()), V(edge.label)});
    });
    compare::add_value("integ_color_material_lookup", [] {
        const auto c = [](const std::optional<Color>& color) { return color ? V::list_of(color->to_tuple()) : V(); };
        const auto m = [](const std::optional<FinishedMaterial>& material) {
            return material ? V(material->name()) : V();
        };
        std::vector<V> out;
        Compound assembly = Compound::make_assembly({Box(1, 1, 1), Pos(3, 0, 0) * Sphere(1)});
        assembly.color = Color("red");
        out.push_back(c(assembly.children[0].color));  // the parent's
        assembly.color = Color("blue");
        out.push_back(c(assembly.children[0].color));  // kept from the first read
        out.push_back(c(assembly.children[1].color));
        out.push_back(c(assembly.children[0].color.raw()));
        Compound inner = Compound::make_assembly({Box(1, 1, 1)});
        Compound outer = Compound::make_assembly({inner});
        outer.color = Color("green");
        out.push_back(c(outer.children[0].children[0].color));  // two levels up
        out.push_back(c(outer.children[0].color.raw()));       // the level between keeps none
        Part box = Box(1, 1, 1);
        box.material = "aluminum";
        out.push_back(c(box.color));  // the material's look
        out.push_back(m(box.material));
        box.color = Color("red");
        box.material = std::nullopt;
        out.push_back(c(box.color));
        Compound metal = Compound::make_assembly({Box(1, 1, 1)});
        metal.material = "Mild Steel";
        out.push_back(m(metal.children[0].material));
        out.push_back(c(metal.children[0].color));
        Compound holder = Compound::make_assembly({Box(1, 1, 1)});
        holder.color = Color("red");
        Part target = Box(2, 2, 2);
        holder.children[0].copy_attributes_to(target);  // copies own values only
        out.push_back(c(target.color.raw()));
        Part copper = Box(1, 1, 1);
        copper.material = "copper";
        const BasePartObject object(copper);  // built with the part's material: its look's color
        out.push_back(c(object.color));
        out.push_back(m(object.material));
        return V::list(out);
    });
    compare::add_value("integ_unit_tables", [] {
        std::vector<V> per_meter, per_kilogram;
        for (const Unit u : {Unit::IN, Unit::FT, Unit::MC, Unit::MM, Unit::CM, Unit::M}) {
            per_meter.push_back(UNITS_PER_METER.at(u));
        }
        for (const Unit u : {Unit::G, Unit::KG, Unit::LB}) per_kilogram.push_back(UNITS_PER_KILOGRAM.at(u));
        return V::list({V::list(per_meter), V::list(per_kilogram), V::list({G, KG, LB, G_PER_LB})});
    });
    compare::add_value("integ_brake_select", [] {
        const Part part = make_brake_formed(0.5, 2, Polyline({{0, 0}, {5, 6}, {10, 1}}));
        return V::obj({{"new_edges", part.edges(Select::NEW).size()},
                       {"new_edge_length", total_length(part.edges(Select::NEW))},
                       {"last_edges", part.edges(Select::LAST).size()},
                       {"new_faces", part.faces(Select::NEW).size()},
                       {"new_face_area", total_area(part.faces(Select::NEW))},
                       {"last_faces", part.faces(Select::LAST).size()}});
    });
    compare::add_value("integ_joints_through_booleans", [] {
        Part base = Box(4, 4, 4);
        base.label = "base";
        RigidJoint("a", base, Location(Vector(2, 0, 0)));
        const Part separate = base + Pos(5, 0, 0) * Box(1, 1, 1);
        const Part fused = base + Pos(2, 0, 0) * Box(2, 2, 2);
        const Part cut = base - Cylinder(1, 4);
        const Part common = base & Sphere(3);
        const Part moved = Pos(0, 0, 10) * base;
        return V::list({joint_info(separate), joint_info(fused), joint_info(cut), joint_info(common), joint_info(moved)});
    });
    compare::add_value("integ_base_part_object_joints", [] {
        Part part = Pos(1, 0, 0) * Box(2, 2, 2);
        part.label = "custom";
        RigidJoint("a", part, Location(Vector(2, 0, 0)));
        LinearJoint("l", part, Axis::X, {0, 4});
        const BasePartObject rotated(part, Rotation(0, 0, 90));
        const BasePartObject aligned(part, Rotation(), Align3(Align::MIN));
        auto info = [](const Part& p) {
            return V::obj({{"keys", V::list_of(p.joints.keys())},
                           {"label", p.label},
                           {"a", p.joints["a"].location().position()},
                           {"l", p.joints["l"].location().position()},
                           {"l_dir", Plane(p.joints["l"].location()).z_dir}});
        };
        return V::list({info(rotated), info(aligned)});
    });
}
