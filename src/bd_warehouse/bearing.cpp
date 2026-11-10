// Port of bd_warehouse's bearing.py.
#include "bd_warehouse/bearing.hpp"

#include <algorithm>
#include <cmath>

#include "bd_warehouse/fastener.hpp"
#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;
using detail::placed_sketch;

namespace {

double num(const ParamDict& data, const std::string& key) { return data.at(key).number(); }

/// The Plane bd_warehouse draws the tapered bearing's races on: Plane.XZ moved to z.
Plane race_plane(double z) { return Plane(Vector(0, 0, z), Vector(1, 0, 0), Vector(0, -1, 0)); }

}  // namespace

// ---------------------------------------------------------------------------
// Bearing
// ---------------------------------------------------------------------------

const StringTable& Bearing::clearance_hole_drill_sizes() { return Fastener::clearance_hole_drill_sizes(); }

const ParamTable& Bearing::clearance_hole_data() { return Fastener::clearance_hole_data(); }

Dict<std::string, std::vector<std::string>> Bearing::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> type_dict;
    const auto add = [&](const std::string& name, const std::set<std::string>& types, auto sizes_of_type) {
        for (const auto& bearing_type : types) {
            const std::vector<std::string> sizes = sizes_of_type(bearing_type);
            if (std::find(sizes.begin(), sizes.end(), size) != sizes.end()) type_dict[name].push_back(bearing_type);
        }
    };
    add("SingleRowDeepGrooveBallBearing", SingleRowDeepGrooveBallBearing::types(),
        SingleRowDeepGrooveBallBearing::sizes);
    add("SingleRowCappedDeepGrooveBallBearing", SingleRowCappedDeepGrooveBallBearing::types(),
        SingleRowCappedDeepGrooveBallBearing::sizes);
    add("SingleRowAngularContactBallBearing", SingleRowAngularContactBallBearing::types(),
        SingleRowAngularContactBallBearing::sizes);
    add("SingleRowCylindricalRollerBearing", SingleRowCylindricalRollerBearing::types(),
        SingleRowCylindricalRollerBearing::sizes);
    add("SingleRowTaperedRollerBearing", SingleRowTaperedRollerBearing::types(),
        SingleRowTaperedRollerBearing::sizes);
    return type_dict;
}

bool Bearing::method_exists(const std::string& method) const {
    const std::set<std::string>* methods = detail::class_methods(class_name_);
    return methods != nullptr && methods->count(method) > 0;
}

ParamDict Bearing::clearance_hole_diameters() const {
    const auto found = clearance_hole_data().get(detail::py_split(bearing_size, "-")[0]);
    if (!found) {
        // Python prints the bore as the table gives it (an int for "8", a float for "8.5")
        const std::string raw = isolate_fastener_type(bearing_type, class_bearing_data()).at(bearing_size).at("d");
        const detail::PyNumber bore = detail::py_eval_number(raw);
        const std::string text =
            bore.is_int ? std::to_string(static_cast<long long>(bore.value)) : detail::py_str(bore.value);
        throw std::invalid_argument("No clearance hole data for size " + text);
    }
    return *found;
}

double Bearing::bore_diameter() const { return num(bearing_dict, "d"); }

double Bearing::outer_diameter() const { return num(bearing_dict, "D"); }

double Bearing::thickness() const {
    return bearing_class() == "SingleRowTaperedRollerBearing" ? num(bearing_dict, "T") : num(bearing_dict, "B");
}

int Bearing::roller_count() const {
    return static_cast<int>(1.8 * M_PI * race_center_radius() / roller_diameter());
}

double Bearing::default_race_center_radius() const {
    return (num(bearing_dict, "D1") + num(bearing_dict, "d1")) / 4;
}

double Bearing::default_roller_diameter() const {
    return 0.625 * (num(bearing_dict, "D1") - num(bearing_dict, "d1"));
}

std::string Bearing::info() const { return bearing_class() + "(" + bearing_type + "): " + bearing_size; }

void Bearing::init(const std::string& class_name, const std::string& size, const std::string& bearing_type_) {
    class_name_ = class_name;
    bearing_size = detail::py_strip(size);
    const std::set<std::string> types = detail::types_of(class_bearing_data());
    if (types.count(bearing_type_) == 0) {
        throw std::invalid_argument(bearing_type_ + " invalid, must be one of " + detail::py_set_repr(types));
    }
    bearing_type = bearing_type_;
    capped = method_exists("cap");
    if (bearing_size.empty()) throw std::out_of_range("string index out of range");
    is_metric = bearing_size[0] == 'M';
    hole_locations.clear();

    const StringTable isolated = isolate_fastener_type(bearing_type, class_bearing_data());
    const auto row = isolated.get(bearing_size);
    if (!row) {
        throw std::invalid_argument(size + " invalid, must be one of " +
                                    detail::py_list_repr(detail::sizes_of(bearing_type, class_bearing_data())));
    }
    bearing_dict = evaluate_parameter_dict(*row, is_metric);

    assign(make_bearing(), Rotation(), std::nullopt);
    // Change position to match PressFitHole expectations
    set_position(position() + Vector(0, 0, num(bearing_dict, "B") / 2));
    const BoundBox bbox = bounding_box();
    RigidJoint("a", *this, Pos(0, 0, bbox.min.Z));
    RigidJoint("b", *this, Pos(0, 0, bbox.max.Z));
    label = class_name + "-" + bearing_size;
}

Compound Bearing::make_bearing() const {
    Part outer_race = build123d::revolve(outer_race_section(), Axis::Z);
    outer_race.material = metals::alloy_steel(metals::AlloySteel::G4140_QUENCHED_TEMPERED);
    outer_race.label = "OuterRace";
    Part inner_race = build123d::revolve(inner_race_section(), Axis::Z);
    inner_race.material = metals::alloy_steel(metals::AlloySteel::G4140_QUENCHED_TEMPERED);
    inner_race.label = "InnerRace";

    std::vector<Shape> bearing_pieces{outer_race, inner_race};
    if (capped) {
        Solid cap_solid = *cap();
        cap_solid.label = "Cap";
        cap_solid.material = plastics::rubber(PolymerOptions{.color = ColorSpec("black")});
        bearing_pieces.push_back(cap_solid);
        bearing_pieces.push_back(Solid(cap_solid).mirror(Plane::XY));  // copy.copy(cap).mirror(Plane.XY)
    } else {
        Solid roller_solid = roller();
        roller_solid.label = "Roller";
        roller_solid.material = metals::alloy_steel(metals::AlloySteel::G52100_HARDENED_LOW_TEMPERED);
        const double radius = race_center_radius();
        const int count = roller_count();
        for (const Location& location : PolarLocations(radius, count)) bearing_pieces.push_back(location * roller_solid);

        if (method_exists("cage")) {
            Solid cage_solid = *cage();
            cage_solid.label = "Cage";
            cage_solid.material = metals::mild_steel(MetalOptions{.finish = finishes::black_oxide()});
            bearing_pieces.push_back(cage_solid);
        }
    }
    Compound bearing;
    bearing.children = bearing_pieces;
    return bearing;
}

Face Bearing::default_inner_race_section() const {
    const double d1 = num(bearing_dict, "d1"), d = num(bearing_dict, "d"), B = num(bearing_dict, "B"),
                 r12 = num(bearing_dict, "r12");
    Builder section(2);
    section.add(detail::place(RectangleRounded((d1 - d) / 2, B, r12), {Location(Vector((d1 + d) / 4, 0))}));
    return placed_sketch(section, Plane::XZ).face();
}

Face Bearing::default_outer_race_section() const {
    const double D1 = num(bearing_dict, "D1"), D = num(bearing_dict, "D"), B = num(bearing_dict, "B"),
                 r12 = num(bearing_dict, "r12");
    Builder section(2);
    section.add(detail::place(RectangleRounded((D - D1) / 2, B, r12), {Location(Vector((D1 + D) / 4, 0))}));
    return placed_sketch(section, Plane::XZ).face();
}

Face Bearing::default_countersink_profile(double interference) const {
    const double D = num(bearing_dict, "D"), B = num(bearing_dict, "B");
    Builder profile(2);
    profile.add(Rectangle(D / 2 - interference, B, Align::MIN));
    return placed_sketch(profile, Plane::XZ).face();
}

Solid Bearing::default_roller() const { return Solid::make_sphere(roller_diameter() / 2); }

Solid Bearing::default_cap() const {
    const double D1 = num(bearing_dict, "D1"), d1 = num(bearing_dict, "d1"), B = num(bearing_dict, "B");
    Builder cap_builder(3);
    {
        Builder sketch(2);
        sketch.add(Circle(D1 / 2));
        sketch.add(Circle(d1 / 2), Mode::SUBTRACT);
        cap_builder.publish(sketch.product(), Mode::ADD, {Location()}, {Location(Plane::XY.offset(B * 0.42))});
    }
    detail::extrude(cap_builder, B * 0.05);
    return cap_builder.part().solid();
}

// ---------------------------------------------------------------------------
// The bearing classes
// ---------------------------------------------------------------------------

#define BD_WAREHOUSE_BEARING_FUNCTIONS(Class, file)                                                       \
    const StringTable& Class::bearing_data() {                                                            \
        static const StringTable table = read_fastener_parameters_from_csv(file);                         \
        return table;                                                                                     \
    }                                                                                                     \
    std::set<std::string> Class::types() { return detail::types_of(bearing_data()); }                     \
    std::vector<std::string> Class::sizes(const std::string& bearing_type) {                              \
        return detail::sizes_of(bearing_type, bearing_data());                                            \
    }                                                                                                     \
    Class::Class(const std::string& size, const std::string& bearing_type) { init(#Class, size, bearing_type); }

BD_WAREHOUSE_BEARING_FUNCTIONS(SingleRowDeepGrooveBallBearing, "single_row_deep_groove_ball_bearing_parameters.csv")
BD_WAREHOUSE_BEARING_FUNCTIONS(SingleRowCappedDeepGrooveBallBearing,
                               "single_row_capped_deep_groove_ball_bearing_parameters.csv")
BD_WAREHOUSE_BEARING_FUNCTIONS(SingleRowAngularContactBallBearing,
                               "single_row_angular_contact_ball_bearing_parameters.csv")
BD_WAREHOUSE_BEARING_FUNCTIONS(SingleRowCylindricalRollerBearing,
                               "single_row_cylindrical_roller_bearing_parameters.csv")
BD_WAREHOUSE_BEARING_FUNCTIONS(SingleRowTaperedRollerBearing, "single_row_tapered_roller_bearing_parameters.csv")

#undef BD_WAREHOUSE_BEARING_FUNCTIONS

// ---- SingleRowAngularContactBallBearing

double SingleRowAngularContactBallBearing::roller_diameter() const {
    const double d = num(bearing_dict, "d"), d2 = num(bearing_dict, "d2"), D = num(bearing_dict, "D");
    const double D2 = D - (d2 - d);
    return (D2 - d2) / 2;
}

double SingleRowAngularContactBallBearing::contact_angle() const {
    const double a = num(bearing_dict, "a"), D = num(bearing_dict, "D");
    return detail::degrees(std::atan(a / (D / 2)));
}

Face SingleRowAngularContactBallBearing::inner_race_section() const {
    const double d = num(bearing_dict, "d"), d1 = num(bearing_dict, "d1"), d2 = num(bearing_dict, "d2"),
                 r12 = num(bearing_dict, "r12"), B = num(bearing_dict, "B"), D = num(bearing_dict, "D");
    const double roller_r = roller_diameter() / 2;
    Builder section(2);
    section.add(detail::place(Rectangle((d1 - d) / 2, B), {Location(Vector((d1 + d) / 4, 0))}));
    section.add(detail::place(Rectangle((d1 - d2) / 2, B / 2, Align2(Align::MAX, Align::MIN)),
                              {Location(Vector(d1 / 2, 0))}),
                Mode::SUBTRACT);
    section.add(detail::place(Circle(roller_r), {Location(Vector((D + d) / 4, 0))}), Mode::SUBTRACT);
    const auto left = section.vertices().group_by(Axis::X)[0];
    detail::fillet(section, std::vector<Shape>(left.begin(), left.end()), r12);
    return placed_sketch(section, Plane::XZ).face();
}

Face SingleRowAngularContactBallBearing::outer_race_section() const {
    const double d = num(bearing_dict, "d"), D = num(bearing_dict, "D"), D1 = num(bearing_dict, "D1"),
                 d2 = num(bearing_dict, "d2"), r12 = num(bearing_dict, "r12"), r34 = num(bearing_dict, "r34"),
                 B = num(bearing_dict, "B");
    const double D2 = D - (d2 - d);
    const double roller_r = roller_diameter() / 2;
    Builder section(2);
    section.add(detail::place(Rectangle((D1 - D) / 2, B), {Location(Vector((D1 + D) / 4, 0))}));
    section.add(detail::place(Rectangle((D1 - D2) / 2, B / 2, Align2(Align::MIN, Align::MAX)),
                              {Location(Vector(D1 / 2, 0))}),
                Mode::SUBTRACT);
    section.add(detail::place(Circle(roller_r), {Location(Vector((D + d) / 4, 0))}), Mode::SUBTRACT);
    detail::fillet(section, {section.vertices().group_by(Axis::X)[-1].sort_by(Axis::Y)[-1]}, r12);
    detail::fillet(section, {section.vertices().group_by(Axis::X)[-1].sort_by(Axis::Y)[0]}, r34);
    return placed_sketch(section, Plane::XZ).face();
}

std::optional<Solid> SingleRowAngularContactBallBearing::cage() const {
    const double d = num(bearing_dict, "d"), d1 = num(bearing_dict, "d1"), D = num(bearing_dict, "D"),
                 D1 = num(bearing_dict, "D1"), d2 = num(bearing_dict, "d2"), B = num(bearing_dict, "B");
    const Sphere hole(0.525 * roller_diameter());
    const double cage_t = 0.2 * (D1 - d2);

    const Edge cage_line = Plane::XZ * Spline({Vector(d2 / 2 + cage_t, 0.4 * B), Vector(d1 / 2 + cage_t, -0.4 * B)},
                                              {Vector(0, -1), Vector(0, -1)});
    const Face cage_face = Face::revolve(cage_line, 360, Axis::Z);
    const Sketch holed = cage_face - (PolarLocations((D - d) / 2, roller_count()) * hole);
    return Solid::thicken(holed, -cage_t);
}

// ---- SingleRowCylindricalRollerBearing

Solid SingleRowCylindricalRollerBearing::roller() const {
    const double roller_length = 0.7 * num(bearing_dict, "B");
    return Solid::make_cylinder(roller_diameter() / 2, roller_length, Plane::XY.offset(-roller_length / 2));
}

// ---- SingleRowTaperedRollerBearing

double SingleRowTaperedRollerBearing::roller_diameter() const {
    if (!roller_cache_) roller();
    return *std::max_element(roller_diameters_.begin(), roller_diameters_.end()) * 1.25;
}

double SingleRowTaperedRollerBearing::contact_angle() const {
    return detail::degrees(std::atan(num(bearing_dict, "e")));
}

double SingleRowTaperedRollerBearing::race_center_radius() const {
    if (!roller_cache_) roller();
    return race_center_radius_;
}

int SingleRowTaperedRollerBearing::roller_count() const {
    if (!roller_cache_) roller();
    return static_cast<int>(std::floor(race_center_radius_ * 2 * M_PI / roller_diameter()));
}

Face SingleRowTaperedRollerBearing::outer_race_section() const {
    if (!outer_race_section_cache_) outer_race_section_cache_ = _outer_race_section();
    return *outer_race_section_cache_;
}

Face SingleRowTaperedRollerBearing::_outer_race_section() const {
    const double B = num(bearing_dict, "B"), C = num(bearing_dict, "C"), D = num(bearing_dict, "D"),
                 r34 = num(bearing_dict, "r34");
    Builder section(2);
    {
        Builder line(1);
        const Polyline l1({{D / 2, 0}, {D / 2, C}, {D / 2 - r34 * 1.5, C}});
        line.add(l1);
        const PolarLine l2(l1.position_at(1), C, Vector(0, -1).rotate(Axis::Z, -contact_angle()), LengthMode::VERTICAL);
        line.add(l2);
        line.add(Line(l2.position_at(1), l1.position_at(0)));
        section.publish(line.product());
    }
    detail::make_face(section);
    const auto right = section.vertices().group_by(Axis::X)[-1];
    detail::fillet(section, std::vector<Shape>(right.begin(), right.end()), r34);
    return placed_sketch(section, race_plane(-B / 2)).face();
}

Face SingleRowTaperedRollerBearing::inner_race_section() const {
    if (!inner_race_section_cache_) inner_race_section_cache_ = _inner_race_section();
    return *inner_race_section_cache_;
}

Face SingleRowTaperedRollerBearing::_inner_race_section() const {
    const double d = num(bearing_dict, "d"), da = num(bearing_dict, "da"), B = num(bearing_dict, "B"),
                 T = num(bearing_dict, "T"), r12 = num(bearing_dict, "r12");
    const double inner_raceway_angle = contact_angle() / 1.5;
    Builder section(2);
    {
        Builder line(1);
        const Polyline l1({{da / 2 - r12, -B}, {d / 2, -B}, {d / 2, 0}});
        line.add(l1);
        const PolarLine l2(l1.position_at(0), B, 90 - inner_raceway_angle, LengthMode::VERTICAL);
        line.add(l2);
        line.add(Line(l1.position_at(1), l2.position_at(1)));
        section.publish(line.product());
    }
    detail::make_face(section);
    const auto left = section.vertices().group_by(Axis::X)[0];
    detail::fillet(section, std::vector<Shape>(left.begin(), left.end()), r12);
    // Make a slot around the inner race to capture the rollers
    const Edge outside_edge = section.edges().sort_by([](const Edge& e) { return e.length(); })[-1];
    taper_length = outside_edge.length();
    const Vector middle = outside_edge.position_at(0.5);
    const Edge path = Edge::make_line(middle, middle + outside_edge.tangent_at(0.5).rotate(Axis::Z, 90) * r12 / 2);
    // sweep() in a BuildSketch adds the swept face to the sketch; insert() then subtracts it
    const Sketch swept = detail::sweep(section, {outside_edge.trim(0.075, 0.925)}, path);
    detail::insert(section, {swept}, {Location()}, Mode::SUBTRACT);
    return placed_sketch(section, race_plane(T - B / 2)).face();
}

Solid SingleRowTaperedRollerBearing::roller() const {
    if (!roller_cache_) roller_cache_ = _roller();
    return *roller_cache_;
}

Solid SingleRowTaperedRollerBearing::_roller() const {
    const double GAP = 0.05;
    const Face inner_section = inner_race_section();
    const Face outer_section = outer_race_section();
    const Edge inner_edge =
        inner_section.edges().filter_by(Axis::Z, true).sort_by([](const Edge& e) { return e.length(); })[-1];
    const Edge outer_edge = outer_section.edges().sort_by(Axis::X)[0];
    const Edge roller_inner_edge = inner_edge.trim(GAP, 1 - GAP);
    const Axis c_inner_a(inner_edge);
    const Axis c_outer_a(outer_edge);
    const auto apex = c_inner_a.intersect(c_outer_a);
    if (!apex || !std::holds_alternative<Vector>(*apex)) {
        throw std::runtime_error("race section edges do not intersect");
    }
    const Axis r_axis(std::get<Vector>(*apex), (c_inner_a.direction + c_outer_a.direction * -1) / 2);
    const Face roller_non_planar_face = Face::revolve(roller_inner_edge, 360, r_axis);
    const auto roller_circles = roller_non_planar_face.edges().filter_by(GeomType::CIRCLE);
    std::vector<Face> roller_faces;
    for (const auto& e : roller_circles) roller_faces.emplace_back(Wire(e));
    roller_faces.push_back(roller_non_planar_face);
    Solid roller_solid{Shell(roller_faces)};
    roller_diameters_.clear();
    for (const auto& e : roller_circles) roller_diameters_.push_back(2 * e.radius());
    if (roller_diameters_.empty()) throw std::invalid_argument("min() arg is an empty sequence");
    const double smallest = *std::min_element(roller_diameters_.begin(), roller_diameters_.end());
    const auto cage_edges = build123d::section(roller_solid, Plane::XZ)
                                .intersect(Axis(r_axis.position + Vector(0.3 * smallest, 0, 0), r_axis.direction));
    if (cage_edges.empty()) throw std::runtime_error("roller section does not cross the cage line");
    cage_edge = Edge(cage_edges[0].wrapped());
    race_center_radius_ = roller_solid.faces().sort_by(Axis::Z)[-1].center().X;
    roller_solid.set_position(roller_solid.position() - Vector(race_center_radius_, 0, 0));
    return roller_solid;
}

std::optional<Solid> SingleRowTaperedRollerBearing::cage() const {
    // To make the cage first create the appropriate conical surface then punch holes for the
    // rollers which can then be thickened to a solid object. This is more efficient then making
    // holes in a solid cage.
    const Solid roller_solid = roller();
    Part roller_hole_cutter = build123d::offset(roller_solid, 0.25 * MM, OffsetOptions{.openings = {}, .kind = Kind::INTERSECTION, .side = Side::BOTH, .closed = true, .min_edge_length = std::nullopt});
    roller_hole_cutter.move(Pos(race_center_radius(), 0, 0));
    const double roller_max_r = roller_solid.faces()
                                    .filter_by(GeomType::PLANE)
                                    .sort_by([](const Face& f) { return f.area(); })[0]
                                    .edge()
                                    .radius();
    const Edge cage_side =
        Pos(roller_max_r / 4, 0, 0) * Edge::make_line(cage_edge.position_at(-0.1), cage_edge.position_at(1.1));
    const int bottom = cage_edge.position_at(0).Y < cage_edge.position_at(1).Y ? 1 : 0;
    const double hook_sign = bottom == 0 ? -1 : 1;
    const JernArc hook(cage_side.position_at(bottom), cage_side.tangent_at(bottom) * hook_sign, 3 * MM, 80);
    const Wire cage_profile(std::vector<Edge>{cage_side, hook});
    const Shell cage_surface = Shell::revolve(cage_profile, 360, Axis::Z);
    const Sketch holed = cage_surface - (PolarLocations(0, roller_count()) * roller_hole_cutter);
    return Solid::thicken(holed, 0.5 * MM);
}

// ---------------------------------------------------------------------------
// PressFitHole
// ---------------------------------------------------------------------------

PressFitHole::PressFitHole(const Bearing& bearing, double interference, const std::string& fit,
                           std::optional<double> depth) {
    // (bd_warehouse's default in builder mode, twice the size of the builder's part, needs one)
    hole_depth = depth ? *depth : bearing.thickness();

    const ParamDict diameters = bearing.clearance_hole_diameters();
    const auto found = diameters.get(fit);
    if (!found) throw std::invalid_argument(fit + " invalid, must be one of " + detail::fits_repr(diameters));
    const double axle_hole_radius = found->number() / 2;

    const Vector bore_direction(0, 0, -1);
    const Solid axle_hole =
        Solid::make_cylinder(axle_hole_radius, hole_depth, Plane(Vector(0, 0, 0), std::nullopt, bore_direction));

    const Face seat_profile = bearing.countersink_profile(interference);
    const double seat_depth = seat_profile.vertices().sort_by(Axis::Z)[-1].Z();
    const Part bearing_seat = build123d::revolve(seat_profile).moved(Pos(0, 0, -seat_depth));
    const Shape hole_part = bearing_seat.fuse(axle_hole);

    assign(hole_part, Rotation(), std::nullopt);
}

}  // namespace bd_warehouse
