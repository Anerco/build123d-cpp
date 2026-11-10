// Port of bd_warehouse's retaining_ring.py.
#include "bd_warehouse/retaining_ring.hpp"

#include "data.hpp"

namespace bd_warehouse {

using namespace build123d;

namespace {

/// _intersections: the intersections of a curve with an axis, raising if there are none.
ShapeList<Shape> intersections(const Shape& shape, const Axis& axis) {
    ShapeList<Shape> found = shape.intersect(axis);
    if (found.empty()) throw std::invalid_argument("curve and axis do not intersect");
    return found;
}

Vector point(const Shape& vertex) { return Vector(Vertex(vertex.wrapped())); }

Dict<std::string, double> ring_parameters(const RingTable& ring_data, const std::string& size) {
    const auto found = ring_data.get(detail::py_strip(size));
    if (!found) throw std::invalid_argument(size + " invalid, must be one of " + detail::py_list_repr(ring_data.keys()));
    return *found;
}

Part external_ring(const Dict<std::string, double>& ring_dict, const std::string& ring_size) {
    const double d3 = ring_dict.at("d3"), b = ring_dict.at("b"), a = ring_dict.at("a"), d5 = ring_dict.at("d5"),
                 s = ring_dict.at("s");
    Sketch ring_plan;
    // Sizes 3-9 need a compact overlapping-slot construction because their lug proportions do not
    // leave enough room for the general tangent-arc profile.
    if (detail::py_int_str(ring_size) <= 9) {
        ring_plan = Pos(0, b / 4) * Circle((d3 + 3 * b / 2) / 2);
        ring_plan += Pos(0, -d3 / 2 - a / 2) * SlotOverall(3 * d5, a);
        ring_plan = Sketch(fillet(ring_plan.vertices().group_by(Axis::Y)[1], 2 * d5));
        ring_plan -= Circle(d3 / 2);
        ring_plan -= Rectangle(d5, 2 * d3, Align2(Align::CENTER, Align::MAX));
        ring_plan -= Pos(0, -d3 / 2 - a / 2) * SlotOverall(2 * d5, d5);
    } else {
        // The helper circles locate a smooth tangent transition from the ring body to one plier
        // lug. Only half is retained and mirrored for exact symmetry.
        const Vector c1_cntr =
            point(intersections(CenterArc(Vector(0, 0), d3 / 2 + b, 0, 90), Axis(Vector(d3 / 5, 0), Vector(0, 1)))[0]);
        const Vector c2_cntr = point(intersections(CenterArc(Vector(0, 0), d3 / 2 + a, 270, 90),
                                                   Axis(Vector(a / 1.5 + 2.5 * d5, 0), Vector(0, 1)))[0]);
        const ThreePointArc cntr_path(c2_cntr, Vector((d3 + a + b) / 2, 0), c1_cntr);

        const CenterArc c1(c1_cntr, b / 5, 0, 360);
        const CenterArc c2(c2_cntr, a / 1.5, 0, 360);
        const Edge a1 = ConstrainedArcs(TangencyTarget(c1, Tangency::OUTSIDE), TangencyTarget(c2),
                                        ConstrainedArcOptions{.radius = cntr_path.radius() - a / 2, .center_on = std::nullopt, .center = std::nullopt, .sagitta = Sagitta::SHORT})
                            .edges()
                            .sort_by(Axis::X)[-1];
        const Vertex a1_top = a1.vertices().sort_by(Axis::Y)[-1];
        const Vertex a1_bot = a1.vertices().sort_by(Axis::Y)[0];
        const Polyline p({Vector(a1_top), Vector(a1_top + Vector(0, b)), Vector(a1_top + Vector(d3, b)),
                          Vector(a1_bot + Vector(d3, -a)), Vector(a1_bot + Vector(0, -a)), Vector(a1_bot)});
        ring_plan = Pos(0, (b - a) / 2) * Circle((d3 + a + b) / 2);
        ring_plan -= std::vector<Face>{Face(Wire(c1)), Face(Wire(c2))};
        std::vector<Edge> outline = p.edges();
        outline.push_back(a1);
        ring_plan -= Face(Wire(outline));
        ring_plan = Sketch(fillet(ring_plan.vertices().sort_by(Axis::Y)[0], a / 2));
        ring_plan -= Circle(d3 / 2);
        ring_plan -= Rectangle(d5, d3, Align2(Align::CENTER, Align::MAX));
        ring_plan -= Pos(point(intersections(CenterArc(Vector(0, 0), (d3 + a) / 2, 270, 90),
                                             Axis(Vector(1.5 * d5, 0), Vector(0, 1)))[0])) *
                     Circle(d5 / 2);
        ring_plan = split(ring_plan, Plane::YZ);
        ring_plan += mirror(ring_plan, Plane::YZ);
    }
    // Create the Solid
    return extrude(ring_plan, s);
}

Part internal_ring(const Dict<std::string, double>& ring_dict, const std::string& ring_size) {
    const double d3 = ring_dict.at("d3"), b = ring_dict.at("b"), a = ring_dict.at("a"), d5 = ring_dict.at("d5"),
                 s = ring_dict.at("s");
    const CenterArc c1(Vector(0, 0), (d3 - a) / 2, 270, 90);

    // Sizes 8-11 need the smaller offset to keep their compact lug connected; larger rings have
    // enough material for a two-hole-diameter offset.
    const Axis a1(Vector(detail::py_int_str(ring_size) < 12 ? 1.5 * d5 : 2 * d5, 0), Vector(0, 1));
    const Vector i = point(intersections(c1, a1)[0]);

    const Circle base(d3 / 2);
    Sketch ring_plan = base;
    const Sketch hole = Pos(0, -b / 4) * Circle((d3 - 3 * b / 2) / 2);
    ring_plan -= hole;
    ring_plan = split(ring_plan, -Plane::from_x_y(Vector(0, 0), i, Vector(0, 0, 1)));
    const Sketch end_loop = Pos(i) * Circle(a / 2);
    ring_plan += end_loop;

    // Fillet the internal sharp corner
    const Vertex interior_corner = (end_loop & hole).vertices().sort_by(Axis::Y)[-1];
    ring_plan = Sketch(fillet(ring_plan.vertices().sort_by_distance(interior_corner)[0], a / 3));

    // Build the squarish corner
    const ShapeList<Edge> l1 = ConstrainedLines(end_loop.edge(), TangencyTarget(Vector(0, 0))).edges().sort_by(Axis::X);
    const Vector i2 = point(intersections(base.edge(), Axis(l1[0])).sort_by(Axis::Y)[0]);
    const Line l2(i2, Vector(l1[0].vertices().sort_by(Axis::Y)[0]));
    const Vector i3 = point(intersections(base.edge(), Axis(l1[1])).sort_by(Axis::X)[-1]);
    const RadiusArc a2(i2, i3, -d3 / 2);
    const Line l3(i, i3);
    const Line l4(l1[0].position_at(0), i);
    ring_plan += -Face(Wire(std::vector<Edge>{l2, a2, l3, l4}));
    ring_plan -= Pos(i) * Circle(d5 / 2);

    // Create the other side
    ring_plan += mirror(ring_plan, Plane::YZ);

    // Create the Solid
    return extrude(ring_plan, s);
}

}  // namespace

RingTable read_retaining_ring_parameters_from_csv(const std::string& filename) {
    RingTable parameters;
    const detail::DictReader reader = detail::dict_reader(detail::data_file("data/" + filename));
    if (!reader.fieldnames || reader.fieldnames->empty()) throw std::invalid_argument("No header found in " + filename);
    const std::string& size_field = reader.fieldnames->at(0);
    for (const auto& row : reader.rows) {
        const std::optional<std::string>& size = row.at(size_field);
        if (!size || size->empty()) continue;
        Dict<std::string, double> values;
        for (const auto& [name, value] : row.fields) {
            if (name == size_field || !value || value->empty()) continue;
            values[name] = detail::py_float(*value);
        }
        values[size_field] = detail::py_float(*size);
        parameters[*size] = values;
    }
    return parameters;
}

// ---------------------------------------------------------------------------
// RetainingRing
// ---------------------------------------------------------------------------

Dict<std::string, std::vector<std::string>> RetainingRing::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> out;
    for (const auto& [name, found] : ExternalSnapRing::select_by_size(size)) out[name] = found;
    for (const auto& [name, found] : InternalSnapRing::select_by_size(size)) out[name] = found;
    return out;
}

std::string RetainingRing::info() const { return class_name_ + "(" + standard + "): " + ring_size; }

void RetainingRing::init(const std::string& class_name, const std::string& class_standard,
                         const Dict<std::string, double>& dims, const std::string& size, const Rotation& rotation,
                         std::optional<Align3> align) {
    class_name_ = class_name;
    standard = class_standard;
    ring_size = detail::py_strip(size);
    ring_dict = dims;

    assign(make_ring(), rotation, align);
    label = class_name + "-" + ring_size;
    color = Color(0xC0C0C0);
    // Find the two largest planar faces at "top" and "bottom"
    const auto top_bottom = faces().group_by([](const Face& f) { return f.area(); })[-1];
    const std::ptrdiff_t center_index = class_name == "ExternalSnapRing" ? -2 : -1;
    std::vector<Location> center_locs;
    for (std::size_t i = 0; i < top_bottom.size(); ++i) {
        const Face& face = top_bottom[static_cast<std::ptrdiff_t>(i)];
        const Vector arc_center = face.edges()
                                      .filter_by(GeomType::CIRCLE)
                                      .sort_by([](const Edge& e) { return e.radius(); })[center_index]
                                      .arc_center();
        const Vector normal = face.normal_at(arc_center);
        center_locs.emplace_back(Plane(arc_center, std::nullopt, normal * (i == 0 ? -1.0 : 1.0)));
    }
    if (center_locs.size() < 2) throw std::out_of_range("list index out of range");
    RigidJoint("a", *this, center_locs[0]);
    RigidJoint("b", *this, center_locs[1]);
}

// ---- ExternalSnapRing

const std::string ExternalSnapRing::standard = "DIN471";

const RingTable& ExternalSnapRing::ring_data() {
    static const RingTable table = read_retaining_ring_parameters_from_csv("din471.csv");
    return table;
}

std::vector<std::string> ExternalSnapRing::sizes() { return ring_data().keys(); }

Dict<std::string, double> ExternalSnapRing::parameters(const std::string& size) {
    return ring_parameters(ring_data(), size);
}

Dict<std::string, std::vector<std::string>> ExternalSnapRing::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> out;
    if (ring_data().contains(detail::py_strip(size))) out["ExternalSnapRing"] = {standard};
    return out;
}

ExternalSnapRing::ExternalSnapRing(const std::string& size, const Rotation& rotation, std::optional<Align3> align) {
    init("ExternalSnapRing", standard, parameters(detail::py_strip(size)), size, rotation, align);
}

Part ExternalSnapRing::make_ring() const { return external_ring(ring_dict, ring_size); }

// ---- InternalSnapRing

const std::string InternalSnapRing::standard = "DIN472";

const RingTable& InternalSnapRing::ring_data() {
    static const RingTable table = read_retaining_ring_parameters_from_csv("din472.csv");
    return table;
}

std::vector<std::string> InternalSnapRing::sizes() { return ring_data().keys(); }

Dict<std::string, double> InternalSnapRing::parameters(const std::string& size) {
    return ring_parameters(ring_data(), size);
}

Dict<std::string, std::vector<std::string>> InternalSnapRing::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> out;
    if (ring_data().contains(detail::py_strip(size))) out["InternalSnapRing"] = {standard};
    return out;
}

InternalSnapRing::InternalSnapRing(const std::string& size, const Rotation& rotation, std::optional<Align3> align) {
    init("InternalSnapRing", standard, parameters(detail::py_strip(size)), size, rotation, align);
}

Part InternalSnapRing::make_ring() const { return internal_ring(ring_dict, ring_size); }

// ---------------------------------------------------------------------------
// RetainingRingGroove
// ---------------------------------------------------------------------------

RetainingRingGroove::RetainingRingGroove(const RetainingRing& ring_, const Rotation& rotation,
                                         std::optional<Align3> align) {
    const auto* external = dynamic_cast<const ExternalSnapRing*>(&ring_);
    const auto* internal = dynamic_cast<const InternalSnapRing*>(&ring_);
    if (external == nullptr && internal == nullptr) {
        throw std::invalid_argument("RetainingRingGroove only accepts ExternalSnapRing or InternalSnapRing");
    }
    if (external != nullptr) {
        ring = std::make_shared<const ExternalSnapRing>(*external);
    } else {
        ring = std::make_shared<const InternalSnapRing>(*internal);
    }
    groove_diameter = ring_.ring_dict.at("d2");
    groove_width = ring_.ring_dict.at("m");
    clearance_diameter = ring_.ring_dict.at("d4");

    const double inner_diameter = external != nullptr ? groove_diameter : clearance_diameter;
    const double outer_diameter = external != nullptr ? clearance_diameter : groove_diameter;

    Part groove = Solid::make_cylinder(outer_diameter / 2, groove_width) -
                  Solid::make_cylinder(inner_diameter / 2, groove_width);

    assign(groove, rotation, align);
    label = "RetainingRingGroove-" + ring_.standard + "-" + ring_.ring_size;
}

}  // namespace bd_warehouse
