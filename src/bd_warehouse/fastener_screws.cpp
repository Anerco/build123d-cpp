// Port of bd_warehouse's fastener.py: screws, washers and holes (nuts and helpers are in
// fastener.cpp).
#include <algorithm>
#include <cmath>
#include <limits>

#include "../py_compat.hpp"
#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;
using detail::placed_sketch;

namespace {

double num(const ParamDict& data, const std::string& key) { return data.at(key).number(); }

Face face_of(const Shape& shape) { return shape.face(); }

double fit_diameter(const ParamDict& diameters, const std::string& fit) {
    const auto found = diameters.get(fit);
    if (!found) throw std::invalid_argument(fit + " invalid, must be one of " + detail::fits_repr(diameters));
    return found->number();
}

/// _first_piece: the first solid of a boolean result, raising if the operation produced nothing.
Shape first_piece(const ShapeList<Shape>& pieces) {
    if (pieces.empty()) throw std::runtime_error("boolean operation produced no result");
    return pieces[0];
}

std::string py_list_repr(const std::vector<double>& items) {
    std::string out = "[";
    for (std::size_t i = 0; i < items.size(); ++i) out += (i ? ", " : "") + detail::py_str(items[i]);
    return out + "]";
}

/// Python's f"{x:0.2f}".
std::string fixed2(double x) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.2f", x);
    return buffer;
}

template <class C>
void add_types_of_size(Dict<std::string, std::vector<std::string>>& type_dict, const std::string& name,
                       const std::string& size) {
    for (const auto& fastener_type : C::types()) {
        const auto sizes = C::sizes(fastener_type);
        if (std::find(sizes.begin(), sizes.end(), size) != sizes.end()) type_dict[name].push_back(fastener_type);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Screw
// ---------------------------------------------------------------------------

const Dict<std::string, std::vector<double>>& Screw::nominal_length_range() {
    static const Dict<std::string, std::vector<double>> table = lookup_nominal_screw_lengths();
    return table;
}

Dict<std::string, std::vector<std::string>> Screw::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> type_dict;
    add_types_of_size<ButtonHeadScrew>(type_dict, "ButtonHeadScrew", size);
    add_types_of_size<ButtonHeadWithCollarScrew>(type_dict, "ButtonHeadWithCollarScrew", size);
    add_types_of_size<CheeseHeadScrew>(type_dict, "CheeseHeadScrew", size);
    add_types_of_size<CounterSunkScrew>(type_dict, "CounterSunkScrew", size);
    add_types_of_size<HexHeadScrew>(type_dict, "HexHeadScrew", size);
    add_types_of_size<HexHeadWithFlangeScrew>(type_dict, "HexHeadWithFlangeScrew", size);
    add_types_of_size<PanHeadScrew>(type_dict, "PanHeadScrew", size);
    add_types_of_size<PanHeadWithCollarScrew>(type_dict, "PanHeadWithCollarScrew", size);
    add_types_of_size<RaisedCheeseHeadScrew>(type_dict, "RaisedCheeseHeadScrew", size);
    add_types_of_size<RaisedCounterSunkOvalHeadScrew>(type_dict, "RaisedCounterSunkOvalHeadScrew", size);
    add_types_of_size<SetScrew>(type_dict, "SetScrew", size);
    add_types_of_size<ShoulderScrew>(type_dict, "ShoulderScrew", size);
    add_types_of_size<SocketHeadCapScrew>(type_dict, "SocketHeadCapScrew", size);
    add_types_of_size<LowProfileScrew>(type_dict, "LowProfileScrew", size);
    return type_dict;
}

double Screw::iso888_thread_length() const {
    double standard_thread_length;
    if (length <= 125 * MM) {
        standard_thread_length = 2 * thread_diameter + 6 * MM;
    } else if (length <= 200 * MM) {
        standard_thread_length = 2 * thread_diameter + 12 * MM;
    } else {
        standard_thread_length = 2 * thread_diameter + 25 * MM;
    }
    return std::min(max_thread_length, standard_thread_length);
}

double Screw::iso7380_thread_length() const {
    const double standard_thread_length =
        thread_diameter == 16 * MM ? 3 * thread_diameter : 2 * thread_diameter + 12 * MM;
    return std::min(max_thread_length, standard_thread_length);
}

double Screw::threaded_to_head_thread_length() const {
    const double incomplete_thread_length = 2 * thread_pitch;
    return max_thread_length <= incomplete_thread_length ? max_thread_length
                                                         : max_thread_length - incomplete_thread_length;
}

double Screw::machine_screw_thread_length() const {
    const auto in = [&](std::initializer_list<const char*> types) {
        return std::any_of(types.begin(), types.end(), [&](const char* t) { return fastener_type == t; });
    };
    double cutoff;
    if (in({"iso1207", "iso14580", "iso1580", "din967"})) {
        cutoff = thread_diameter <= 3 * MM ? 30 * MM : 40 * MM;
    } else if (in({"iso7045", "iso7048", "iso14583"})) {
        if (thread_diameter <= 2.5 * MM) {
            cutoff = 30 * MM;
        } else if (thread_diameter <= 3 * MM) {
            cutoff = 35 * MM;
        } else {
            cutoff = 40 * MM;
        }
    } else if (in({"iso7046", "iso7047", "iso14581", "iso14584"})) {
        if (thread_diameter <= 2.5 * MM) {
            cutoff = 30 * MM;
        } else if (thread_diameter <= 3 * MM) {
            cutoff = 35 * MM;
        } else if (thread_diameter <= 4 * MM) {
            cutoff = 40 * MM;
        } else {
            cutoff = 45 * MM;
        }
    } else if (in({"iso2009", "iso2010"})) {
        if (thread_diameter <= 3 * MM) {
            cutoff = 30 * MM;
        } else if (thread_diameter <= 3.5 * MM) {
            cutoff = 40 * MM;
        } else {
            cutoff = 45 * MM;
        }
    } else {
        return default_thread_length();
    }
    return length <= cutoff ? threaded_to_head_thread_length() : iso888_thread_length();
}

double Screw::asme_b18_6_3_thread_length() const {
    const bool number_5_or_smaller = thread_diameter <= 0.125 * IN;
    if (length <= 3 * thread_diameter) return max_thread_length - thread_pitch;
    if (length <= (number_5_or_smaller ? 1.125 * IN : 2 * IN)) return max_thread_length - 2 * thread_pitch;
    return (number_5_or_smaller ? 1 : 1.5) * IN;
}

double Screw::asme_b18_3_thread_length() const {
    static const Dict<std::string, double> minimum_thread_lengths = {
        {"#3", 0.62},  {"#4", 0.75},  {"#5", 0.75},  {"#6", 0.75},  {"#8", 0.88},
        {"#10", 0.88}, {"1/4", 1.00}, {"5/16", 1.12}, {"3/8", 1.25}, {"7/16", 1.38},
        {"1/2", 1.50}, {"5/8", 1.75}, {"3/4", 2.00}, {"7/8", 2.25}, {"1", 2.50}};
    const std::string size = detail::py_split(thread_size, "-")[0];
    const double minimum_thread_length = minimum_thread_lengths.at(size) * IN;
    const double first_partial_length = num(screw_data, "short");
    if (length < first_partial_length) {
        if (thread_diameter <= 0.625 * IN) {
            const double incomplete_thread_length = 2 * thread_pitch;
            return max_thread_length <= incomplete_thread_length ? max_thread_length
                                                                 : max_thread_length - incomplete_thread_length;
        }
        return max_thread_length;
    }
    return std::min(max_thread_length, minimum_thread_length);
}

std::optional<Face> Screw::default_shank_profile() const {
    if (grip_length <= 0) return std::nullopt;
    Builder profile(2);
    profile.add(detail::place(Rectangle(thread_diameter / 2, grip_length, Align2(Align::MIN, Align::MAX)),
                              {Location(Vector(0, -length_offset()))}));
    return face_of(placed_sketch(profile, Plane::XZ));
}

double Screw::min_hole_depth(bool counter_sunk) const {
    const auto profile = countersink_profile("Loose");
    if (!profile) return 0;  // SetScrew
    const double head_offset = profile->vertices().sort_by(Axis::Z)[-1].Z();
    return counter_sunk ? length + head_offset - length_offset() : length - length_offset();
}

std::optional<std::vector<double>> Screw::nominal_lengths() const {
    const auto range_min = screw_data.get("short");
    const auto range_max = screw_data.get("long");
    if (!range_min || !range_max || !nominal_length_range().contains(fastener_type)) return std::nullopt;
    std::vector<double> result;
    for (const double size : nominal_length_range().at(fastener_type)) {
        if (range_min->number() <= size && size <= range_max->number()) result.push_back(size);
    }
    return result;
}

std::string Screw::info() const {
    const std::string hand_text = hand == "left" ? " left hand thread" : "";
    return screw_class() + "(" + fastener_type + "): " + thread_size + "x" + detail::py_str(length) + hand_text;
}

Solid Screw::make_body_hole(double hole_radius, double depth, double) const {
    return Solid::make_cylinder(hole_radius, depth, Plane(Vector(0, 0, 0), std::nullopt, Vector(0, 0, -1)));
}

std::optional<Shape> Screw::make_head() const {
    const auto profile_face = head_profile();
    if (!profile_face) throw std::invalid_argument(class_name_ + " must define head_profile");
    const Face profile = *profile_face;
    const BoundBox profile_bbox = profile.bounding_box();
    const double max_head_height = profile_bbox.size.Z;
    const double max_head_radius = profile_bbox.max.X;
    const double min_head_height = profile_bbox.min.Z;

    // Create the basic head shape
    Shape head = build123d::revolve(profile);
    const auto plan = head_plan();
    // Without a plan, one outside of the maximum profile dimension so that the recess can be
    // safely contained within it
    const Face plan_face =
        plan ? *plan : Face::make_rect(3 * max_head_radius, 3 * max_head_radius, Plane::XY.offset(min_head_height));

    // Potentially modify the head to conform to the shape of head_plan (e.g. hex) and/or to
    // add an engagement recess
    if (const auto recess_data = head_recess()) {
        const auto& [recess_plan, recess_depth, recess_taper] = *recess_data;
        const Solid recess =
            Solid::extrude_taper(recess_plan, Vector(0, 0, -recess_depth), recess_taper).translate(Vector(0, 0, max_head_height));
        const Part head_blank = build123d::extrude(plan_face, max_head_height, ExtrudeOptions{.dir = Vector(0, 0, 1)}) - recess;
        head = first_piece(head.intersect(head_blank));
    } else if (plan) {
        const Part head_blank = build123d::extrude(plan_face, max_head_height);
        head = first_piece(head.intersect(head_blank));
    }

    // Add a flange as it exists outside of the head plan
    if (const auto flange = flange_profile()) head = head.fuse(build123d::revolve(*flange));
    return head;
}

HeadRecess Screw::default_head_recess() const {
    std::optional<Face> recess_plan;
    double recess_depth = 0.0, recess_taper = 0.0;
    // Slot Recess
    if (screw_data.contains("dk") && screw_data.contains("n") && screw_data.contains("t")) {
        recess_plan = slot_recess(num(screw_data, "dk"), num(screw_data, "n"));
        recess_depth = num(screw_data, "t");
        recess_taper = 0;
    }
    // Hex Recess
    if (screw_data.contains("s") && screw_data.contains("t")) {
        recess_plan = hex_recess(num(screw_data, "s"));
        recess_depth = num(screw_data, "t");
        recess_taper = 0;
    }
    // Philips, Torx or Robertson Recess
    if (const auto recess_value = screw_data.get("recess")) {
        std::string recess = recess_value->is_number() ? detail::py_str(recess_value->number()) : recess_value->text();
        for (auto& c : recess) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (recess.rfind("PH", 0) == 0) {
            std::tie(recess_plan, recess_depth) = cross_recess(recess);
            recess_taper = 20;
        } else if (recess.rfind("T", 0) == 0) {
            std::tie(recess_plan, recess_depth) = hexalobular_recess(recess);
            recess_taper = 5;
        } else if (recess.rfind("R", 0) == 0) {
            std::tie(recess_plan, recess_depth) = square_recess(recess);
            recess_taper = 0;
        }
    }
    if (!recess_plan) throw std::invalid_argument("Recess data missing from screw_data of " + class_name_);
    return {*recess_plan, recess_depth, recess_taper};
}

Face Screw::default_countersink_profile(const std::string& fit) const {
    const double clearance_hole_diameter = fit_diameter(clearance_hole_diameters(), fit);
    const double width = clearance_hole_diameter - thread_diameter + num(screw_data, "dk");
    Builder profile(2);
    profile.add(Rectangle(width / 2, num(screw_data, "k"), Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

void Screw::init(const std::string& class_name, const std::string& size, double length_,
                 const std::string& fastener_type_, const std::string& hand_, bool simple_, double socket_clearance_,
                 const Rotation& rotation, std::optional<Align3> align) {
    class_name_ = class_name;
    hole_locations.clear();
    screw_size = size;

    const auto size_parts = detail::py_split(detail::py_strip(size), "-");
    if (size_parts.size() != 2) {
        throw std::invalid_argument(detail::py_list_repr(size_parts) +
                                    " invalid, must be formatted as size-pitch or size-TPI");
    }
    thread_size = size;
    is_metric = !thread_size.empty() && thread_size[0] == 'M';
    if (is_metric) {
        thread_diameter = detail::py_float(size_parts[0].substr(1));
        thread_pitch = detail::py_float(size_parts[1]);
    } else {
        std::tie(thread_diameter, thread_pitch) = decode_imperial_size(thread_size);
    }

    length = length_;
    const auto types = detail::types_of(class_fastener_data());
    if (types.count(fastener_type_) == 0) {
        throw std::invalid_argument(fastener_type_ + " invalid, must be one of " + detail::py_set_repr(types));
    }
    fastener_type = fastener_type_;
    if (hand_ != "left" && hand_ != "right") throw std::invalid_argument(hand_ + " invalid, must be one of 'left' or 'right'");
    hand = hand_;
    simple = simple_;
    const StringTable isolated = isolate_fastener_type(fastener_type, class_fastener_data());
    const auto row = isolated.get(thread_size);
    if (!row) {
        throw std::invalid_argument(size + " invalid, must be one of " +
                                    detail::py_list_repr(detail::sizes_of(fastener_type, class_fastener_data())));
    }
    screw_data = evaluate_parameter_dict(*row, is_metric);
    socket_clearance = socket_clearance_;  // Only used for hex head screws

    const double offset = length_offset();
    if (offset >= length) {
        throw std::invalid_argument("Screw length " + detail::py_str(length) + " is <= countersunk screw head " +
                                    detail::py_str(offset));
    }
    max_thread_length = length - offset;
    thread_length = calculate_thread_length();
    if (!(0 < thread_length && thread_length <= max_thread_length)) {
        throw std::invalid_argument("Invalid thread length " + detail::py_str(thread_length) +
                                    " for screw body length " + detail::py_str(max_thread_length));
    }
    grip_length = calculate_shank_length();
    if (grip_length < 0) throw std::invalid_argument("Invalid shank length " + detail::py_str(grip_length) + " for screw body");
    thread_offset = calculate_thread_offset();
    if (thread_offset < 0) throw std::invalid_argument("Invalid thread offset " + detail::py_str(thread_offset));
    std::optional<Shape> head = make_head();

    EndFinishes ends;
    if (!head) {  // A fully custom screw
        head_height = 0;
        head_diameter = 0;
        ends = {"fade", "fade"};
    } else {
        const BoundBox head_bb = head->bounding_box();
        head_height = head_bb.max.Z;
        head_diameter = 2 * std::max(head_bb.max.X, head_bb.max.Y);
        ends = {"fade", thread_offset > 0 ? "fade" : "raw"};
        head = head->translate(Vector(0, 0, -length_offset()));
    }

    IsoThread thread(thread_diameter, thread_pitch, thread_length,
                     IsoThreadOptions{.external = true, .hand = hand, .end_finishes = ends, .simple = simple});
    const double thread_base = -offset - grip_length - thread_offset;
    const double thread_tip = thread_base - thread_length;
    if (!simple) thread.locate(Pos(0, 0, thread_tip));
    thread.label = "thread";

    const Solid thread_core = Solid::make_cylinder(thread.min_radius(), thread_length, Plane::XY.offset(thread_tip));
    const auto shank = shank_profile();
    std::optional<Part> unthreaded_shank;
    if (shank) unthreaded_shank = build123d::revolve(*shank, Axis::Z);

    Shape screw;
    if (const auto custom = custom_make()) {
        screw = *custom;
    } else if (head) {
        std::vector<Shape> tools{thread_core};
        if (unthreaded_shank) tools.push_back(*unthreaded_shank);
        screw = head->fuse(tools);
    } else {
        screw = thread_core;
    }

    // Unwrap the Compound as it's unnecessary
    if (!screw.is_null() && screw.wrapped().ShapeType() == TopAbs_COMPOUND) screw = Compound(screw.wrapped()).unwrap(true);
    screw.label = "body";

    if (!simple) {
        Part assembly;
        assembly.children = std::vector<Shape>{screw, thread};
        screw = assembly;
    }
    assign(screw, rotation, align);
    label = class_name + "(" + size + ", " + fixed2(length) + ", " + fastener_type + ")";
    material = metals::mild_steel(MetalOptions{.finish = finishes::zinc_plate()});
    RigidJoint("a", *this, Location());
}

// ---- the screw classes

#define BD_WAREHOUSE_SCREW_CONSTRUCTORS(Class, default_type)                                                      \
    Class::Class(const std::string& size, double length_, const std::string& fastener_type_, const std::string& hand_, \
                 bool simple_, const Rotation& rotation, std::optional<Align3> align) {                           \
        init(#Class, size, length_, fastener_type_, hand_, simple_, 6 * MM, rotation, align);                     \
    }                                                                                                             \
    Class::Class(const std::string& size, double length_, const ScrewOptions& o) {                                \
        init(#Class, size, length_, o.fastener_type ? *o.fastener_type : std::string(default_type), o.hand,       \
             o.simple, 6 * MM, o.rotation, o.align);                                                              \
    }

namespace {

/// A head profile drawn with lines in a BuildLine, made into a face on Plane.XZ.
template <class F>
Face line_profile(F draw) {
    Builder profile(2);
    {
        Builder line(1);
        draw(line);
        profile.publish(line.product());
    }
    detail::make_face(profile);
    return face_of(placed_sketch(profile, Plane::XZ));
}

}  // namespace

BD_WAREHOUSE_TABLE_FUNCTIONS(ButtonHeadScrew, "button_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(ButtonHeadScrew, "iso7380_1")

std::optional<Face> ButtonHeadScrew::head_profile() const {
    const double dk = num(screw_data, "dk"), dl = num(screw_data, "dl"), k = num(screw_data, "k"),
                 rf = num(screw_data, "rf");
    return line_profile([&](Builder& line) {
        const Polyline l1({{0, 0}, {0, k}, {dl / 2, k}});
        line.add(l1);
        const RadiusArc l2(l1.position_at(1), Vector(dk / 2, 0), rf);
        line.add(l2);
        line.add(Line(l2.position_at(1), l1.position_at(0)));
    });
}

BD_WAREHOUSE_TABLE_FUNCTIONS(ButtonHeadWithCollarScrew, "button_head_with_collar_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(ButtonHeadWithCollarScrew, "iso7380_2")

std::optional<Face> ButtonHeadWithCollarScrew::head_profile() const {
    const double dk = num(screw_data, "dk"), dl = num(screw_data, "dl"), dc = num(screw_data, "dc"),
                 k = num(screw_data, "k"), rf = num(screw_data, "rf"), c = num(screw_data, "c");
    return line_profile([&](Builder& line) {
        line.add(Polyline({{0, 0}, {0, k}, {dl / 2, k}}));
        line.add(RadiusArc(Vector(dl / 2, k), Vector(dk / 2, c), rf));
        line.add(Line(Vector(dk / 2, c), Vector(dc / 2 - c / 2, c)));
        line.add(JernArc(Vector(dc / 2 - c / 2, c), Vector(1, 0), c / 2, -180.0));
        line.add(Line(Vector(dc / 2 - c / 2, 0), Vector(0, 0)));
    });
}

std::optional<Face> ButtonHeadWithCollarScrew::countersink_profile(const std::string& fit) const {
    const double clearance_hole_diameter = fit_diameter(clearance_hole_diameters(), fit);
    const double width = clearance_hole_diameter - thread_diameter + num(screw_data, "dc");
    Builder profile(2);
    profile.add(Rectangle(width / 2, num(screw_data, "k"), Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(CheeseHeadScrew, "cheese_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(CheeseHeadScrew, "iso7048")

std::optional<Face> CheeseHeadScrew::head_profile() const {
    const double k = num(screw_data, "k"), dk = num(screw_data, "dk");
    Builder profile(2);
    profile.add(Trapezoid(dk, k, 90 - 5, Align2(Align::CENTER, Align::MIN)));
    const auto top = profile.vertices().group_by(Axis::Z)[-1];
    detail::fillet(profile, std::vector<Shape>(top.begin(), top.end()), k * 0.25);
    detail::split(profile, std::nullopt, Plane::YZ);
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(CounterSunkScrew, "countersunk_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(CounterSunkScrew, "iso10642")

double CounterSunkScrew::calculate_thread_length() const {
    if (fastener_type == "iso10642" || fastener_type == "iso14582") {
        return std::min(max_thread_length, 2 * thread_diameter + 12 * MM);
    }
    if (fastener_type == "iso2009" || fastener_type == "iso7046" || fastener_type == "iso14581") {
        return machine_screw_thread_length();
    }
    return default_thread_length();
}

double CounterSunkScrew::length_offset() const { return num(screw_data, "k"); }

std::optional<Face> CounterSunkScrew::head_profile() const {
    const double a = num(screw_data, "a"), k = num(screw_data, "k"), dk = num(screw_data, "dk");
    Builder profile(2);
    profile.add(Trapezoid(dk, k, 90 - a / 2, std::nullopt, 180, Align2(Align::CENTER, Align::MAX)));
    const auto top = profile.vertices().group_by(Axis::Y)[-1];
    detail::fillet(profile, std::vector<Shape>(top.begin(), top.end()), k * 0.075);
    detail::split(profile, std::nullopt, Plane::YZ);
    return face_of(placed_sketch(profile, Plane::XZ));
}

std::optional<Face> CounterSunkScrew::countersink_profile(const std::string&) const {
    const double a = num(screw_data, "a"), dk = num(screw_data, "dk"), k = num(screw_data, "k");
    Builder profile(2);
    profile.add(Trapezoid(dk, k, 90 - a / 2, std::nullopt, 180, Align2(Align::CENTER, Align::MIN)));
    detail::split(profile, std::nullopt, Plane::YZ);
    return face_of(placed_sketch(profile, Plane::XZ));
}

namespace {

Face hex_head_profile(double k, double s) {
    const double e = polygon_diagonal(s, 6);
    // Chamfer angle must be between 15 and 30 degrees
    const double cs = (e - s) * std::tan(detail::radians(15)) / 2;
    Builder profile(2);
    profile.add(Polygon({{0, 0}, {e / 2, 0}, {e / 2, k - cs}, {s / 2, k}, {0, k}, {0, 0}}, 0.0, Align::NONE));
    return face_of(placed_sketch(profile, Plane::XZ));
}

Face hex_head_plan(double s) {
    Builder plan(2);
    plan.add(RegularPolygon(s / 2, 6, false));
    return face_of(placed_sketch(plan, Plane::XY));
}

}  // namespace

BD_WAREHOUSE_TABLE_FUNCTIONS(HexHeadScrew, "hex_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(HexHeadScrew, "iso4014")

double HexHeadScrew::calculate_thread_length() const {
    return fastener_type == "din931" || fastener_type == "iso4014" ? iso888_thread_length() : default_thread_length();
}

std::optional<Face> HexHeadScrew::head_profile() const { return hex_head_profile(num(screw_data, "k"), num(screw_data, "s")); }

std::optional<Face> HexHeadScrew::head_plan() const { return hex_head_plan(num(screw_data, "s")); }

std::optional<Face> HexHeadScrew::countersink_profile(const std::string&) const {
    const double k = num(screw_data, "k"), s = num(screw_data, "s");
    const double e = polygon_diagonal(s, 6);
    const double width = e + socket_clearance + e;
    Builder profile(2);
    profile.add(Rectangle(width / 2, k, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(HexHeadWithFlangeScrew, "hex_head_with_flange_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(HexHeadWithFlangeScrew, "din1662")

std::optional<Face> HexHeadWithFlangeScrew::head_profile() const {
    return hex_head_profile(num(screw_data, "k"), num(screw_data, "s"));
}

std::optional<Face> HexHeadWithFlangeScrew::head_plan() const { return hex_head_plan(num(screw_data, "s")); }

std::optional<Face> HexHeadWithFlangeScrew::flange_profile() const {
    const double dc = num(screw_data, "dc"), c = num(screw_data, "c");
    const double flange_angle = 25;
    const Vector tangent_point = Vector((c / 2) * std::cos(detail::radians(90 - flange_angle)),
                                        (c / 2) * std::sin(detail::radians(90 - flange_angle))) +
                                 Vector((dc - c) / 2, c / 2);
    return line_profile([&](Builder& line) {
        const Line l1(Vector(0, 0), Vector(dc / 2 - c / 2, 0));
        line.add(l1);
        line.add(RadiusArc(l1.position_at(1), tangent_point, -c / 2));
        const PolarLine l3(tangent_point, dc / 2 - c / 2, 180 - flange_angle);
        line.add(l3);
        line.add(Polyline({l3.position_at(1), {0, l3.position_at(1).Y}, {0, 0}}));
    });
}

std::optional<Face> HexHeadWithFlangeScrew::countersink_profile(const std::string& fit) const {
    const double clearance_hole_diameter = fit_diameter(clearance_hole_diameters(), fit);
    const double dc = num(screw_data, "dc"), s = num(screw_data, "s"), k = num(screw_data, "k");
    const double shaft_clearance = clearance_hole_diameter - thread_diameter;
    const double width = std::max(dc + shaft_clearance, polygon_diagonal(s, 6) + socket_clearance);
    Builder profile(2);
    profile.add(Rectangle(width / 2, k, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(PanHeadScrew, "pan_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(PanHeadScrew, "iso14583")

double PanHeadScrew::calculate_thread_length() const {
    if (fastener_type == "iso1580" || fastener_type == "iso14583") return machine_screw_thread_length();
    return asme_b18_6_3_thread_length();
}

std::optional<Face> PanHeadScrew::head_profile() const {
    const double k = num(screw_data, "k"), dk = num(screw_data, "dk");
    return line_profile([&](Builder& line) {
        const Line l1(Vector(0, 0), Vector(dk / 2, 0));
        line.add(l1);
        const Spline l2({l1.position_at(1), Vector(dk * 0.25, k)},
                        {Vector(-std::sin(detail::radians(5)), std::cos(detail::radians(5))), Vector(-1, 0)});
        line.add(l2);
        line.add(Polyline({l2.position_at(1), {0, l2.position_at(1).Y}, {0, 0}}));
    });
}

BD_WAREHOUSE_TABLE_FUNCTIONS(PanHeadWithCollarScrew, "pan_head_with_collar_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(PanHeadWithCollarScrew, "din967")

std::optional<Face> PanHeadWithCollarScrew::head_profile() const {
    const double rf = num(screw_data, "rf"), k = num(screw_data, "k"), dk = num(screw_data, "dk"),
                 c = num(screw_data, "c");
    const double flat = std::sqrt(k - c) * std::sqrt(2 * rf - (k - c));
    return line_profile([&](Builder& line) {
        line.add(Polyline({{0, 0}, {dk / 2, 0}, {dk / 2, c}, {flat, c}}));
        line.add(RadiusArc(Vector(flat, c), Vector(0, k), -rf));
        line.add(Line(Vector(0, k), Vector(0, 0)));
    });
}

BD_WAREHOUSE_TABLE_FUNCTIONS(RaisedCheeseHeadScrew, "raised_cheese_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(RaisedCheeseHeadScrew, "iso7045")

std::optional<Face> RaisedCheeseHeadScrew::head_profile() const {
    const double dk = num(screw_data, "dk"), k = num(screw_data, "k"), rf = num(screw_data, "rf");
    const double oval_height = rf - std::sqrt(4 * std::pow(rf, 2) - std::pow(dk, 2)) / 2;
    return line_profile([&](Builder& line) {
        line.add(Line(Vector(0, 0), Vector(0, k)));
        line.add(RadiusArc(Vector(0, k), Vector(dk / 2, k - oval_height), rf));
        line.add(Polyline({{dk / 2, k - oval_height}, {dk / 2, 0}, {0, 0}}));
    });
}

BD_WAREHOUSE_TABLE_FUNCTIONS(RaisedCounterSunkOvalHeadScrew, "raised_countersunk_oval_head_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(RaisedCounterSunkOvalHeadScrew, "iso14584")

double RaisedCounterSunkOvalHeadScrew::calculate_thread_length() const {
    if (fastener_type == "iso2010" || fastener_type == "iso7047" || fastener_type == "iso14584") {
        return machine_screw_thread_length();
    }
    return default_thread_length();
}

double RaisedCounterSunkOvalHeadScrew::length_offset() const { return num(screw_data, "k"); }

std::optional<Face> RaisedCounterSunkOvalHeadScrew::head_profile() const {
    const double a = num(screw_data, "a"), k = num(screw_data, "k"), rf = num(screw_data, "rf"),
                 dk = num(screw_data, "dk");
    const double side_length = k / std::cos(detail::radians(a / 2));
    const double oval_height = rf - std::sqrt(4 * std::pow(rf, 2) - std::pow(dk, 2)) / 2;
    Builder profile(2);
    {
        Builder line(1);
        const Line l1(Vector(0, 0), Vector(0, k + oval_height));
        line.add(l1);
        const RadiusArc l2(l1.position_at(1), Vector(dk / 2, k), rf);
        line.add(l2);
        const PolarLine l3(l2.position_at(1), side_length, -90 - a / 2);
        line.add(l3);
        line.add(Line(l3.position_at(1), Vector(0, 0)));
        profile.publish(line.product());
    }
    detail::make_face(profile);
    const Vertex corner = profile.vertices().group_by(Axis::Z)[-1].sort_by(Axis::X)[-1];
    detail::fillet(profile, {corner}, k * 0.075);
    return face_of(placed_sketch(profile, Plane::XZ));
}

std::optional<Face> RaisedCounterSunkOvalHeadScrew::countersink_profile(const std::string&) const {
    const double a = num(screw_data, "a"), k = num(screw_data, "k"), dk = num(screw_data, "dk");
    const double side_length = k / std::cos(detail::radians(a / 2));
    return line_profile([&](Builder& line) {
        const Polyline l1({{0, 0}, {0, k}, {dk / 2, k}});
        line.add(l1);
        const PolarLine l2(l1.position_at(1), side_length, -90 - a / 2);
        line.add(l2);
        line.add(Line(l2.position_at(1), Vector(0, 0)));
    });
}

BD_WAREHOUSE_TABLE_FUNCTIONS(SetScrew, "setscrew_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(SetScrew, "iso4026")

Solid SetScrew::make_setscrew() const {
    const double s = num(screw_data, "s"), t = num(screw_data, "t");
    // bd_warehouse builds the thread here for its min_radius only (its shape is not used): a
    // simple thread gives the same radius
    const IsoThread thread(thread_diameter * 1, thread_pitch, length,
                           IsoThreadOptions{.external = true, .hand = hand, .end_finishes = {"fade", "fade"}, .simple = true});
    Builder screw(3);
    // Core
    screw.add(Cylinder(thread.min_radius(), length, Align3(Align::CENTER, Align::CENTER, Align::MAX)));
    // Recess
    {
        Builder sketch(2);
        sketch.add(RegularPolygon(s / 2, 6, false));
        screw.publish(sketch.product());
    }
    detail::extrude(screw, -t, {}, Mode::SUBTRACT);
    return screw.part().solid();
}

std::optional<Face> SetScrew::countersink_profile(const std::string&) const { return std::nullopt; }

BD_WAREHOUSE_TABLE_FUNCTIONS(ShoulderScrew, "shoulder_screw.csv")

std::string ShoulderScrew::validated(const std::string& size, double length_, const std::string& fastener_type_) {
    const auto types = detail::types_of(fastener_data());
    if (types.count(fastener_type_) == 0) {
        throw std::invalid_argument(fastener_type_ + " invalid, must be one of " + detail::py_set_repr(types));
    }
    const auto& nominal_lengths = Screw::nominal_length_range().at(fastener_type_);
    const StringTable isolated = isolate_fastener_type(fastener_type_, fastener_data());
    const ParamDict size_data = evaluate_parameter_dict(isolated.get(size).value_or(Dict<std::string, std::string>()));
    if (size_data.empty()) {
        throw std::invalid_argument(size + " invalid, must be one of " +
                                    detail::py_list_repr(detail::sizes_of(fastener_type_, fastener_data())));
    }
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> valid_lengths;
    for (const double nominal_length : nominal_lengths) {
        const auto short_value = size_data.get("short");
        const auto long_value = size_data.get("long");
        if ((short_value ? short_value->number() : inf) <= nominal_length &&
            nominal_length <= (long_value ? long_value->number() : -inf)) {
            valid_lengths.push_back(nominal_length);
        }
    }
    if (std::find(valid_lengths.begin(), valid_lengths.end(), length_) == valid_lengths.end()) {
        throw std::invalid_argument(detail::py_str(length_) + " invalid shoulder length for " + size + "; must be one of " +
                                    py_list_repr(valid_lengths));
    }
    return fastener_type_;
}

ShoulderScrew::ShoulderScrew(const std::string& size, double length_, const std::string& fastener_type_,
                             const std::string& hand_, bool simple_, const Rotation& rotation,
                             std::optional<Align3> align) {
    length = length_;
    init("ShoulderScrew", size, length_, validated(size, length_, fastener_type_), hand_, simple_, 6 * MM, rotation,
         align);
}

ShoulderScrew::ShoulderScrew(const std::string& size, double length_, const ScrewOptions& o)
    : ShoulderScrew(size, length_, o.fastener_type ? *o.fastener_type : std::string("iso7379"), o.hand, o.simple,
                    o.rotation, o.align) {}

double ShoulderScrew::calculate_thread_length() const { return num(screw_data, "b") - num(screw_data, "g2"); }

double ShoulderScrew::calculate_shank_length() const { return length; }

double ShoulderScrew::calculate_thread_offset() const { return num(screw_data, "g2"); }

double ShoulderScrew::min_hole_depth(bool counter_sunk) const {
    const double head_offset = counter_sunk ? num(screw_data, "k") : 0;
    return head_offset + length + num(screw_data, "b");
}

std::optional<Face> ShoulderScrew::head_profile() const {
    const double dk = num(screw_data, "dk"), k = num(screw_data, "k");
    Builder profile(2);
    profile.add(Rectangle(dk / 2, k, Align::MIN));
    const Vertex corner = profile.vertices().group_by(Axis::Y)[-1].sort_by(Axis::X)[-1];
    detail::fillet(profile, {corner}, k * 0.075);
    return face_of(placed_sketch(profile, Plane::XZ));
}

std::optional<Face> ShoulderScrew::shank_profile() const {
    const double ds = num(screw_data, "ds"), dk = num(screw_data, "dk"), k = num(screw_data, "k"),
                 s = num(screw_data, "s"), g1 = num(screw_data, "g1"), g2 = num(screw_data, "g2"),
                 r1 = num(screw_data, "r1"), r2 = num(screw_data, "r2"), dg1 = num(screw_data, "dg1"),
                 dg2 = num(screw_data, "dg2");
    const double rs = s <= 10 ? 0.15 : 0.2;
    Builder profile(2);
    {
        Builder perimeter(1);
        perimeter.add(Polyline({{0, k},
                                {dk / 2, k},
                                {dk / 2, 0},
                                {dg1 / 2, 0},
                                {dg1 / 2, -g1},
                                {ds / 2, -g1 - (ds - dg1) / 2},
                                {ds / 2, -length},
                                {dg2 / 2, -length},
                                {dg2 / 2, -length - g2},
                                {0, -length - g2}},
                               true));
        detail::fillet(perimeter, {perimeter.vertices().group_by(Axis::Y)[-2].sort_by(Axis::X)[0]}, r1);
        detail::fillet(perimeter, {perimeter.vertices().group_by(Axis::Y)[1].sort_by(Axis::X)[-1]}, rs);
        detail::fillet(perimeter, {perimeter.vertices().group_by(Axis::Y)[1].sort_by(Axis::X)[0]}, r2);
        profile.publish(perimeter.product());
    }
    detail::make_face(profile);
    detail::split(profile, std::nullopt, Plane::XZ);
    return face_of(placed_sketch(profile, Plane::XZ));
}

std::optional<Face> ShoulderScrew::countersink_profile(const std::string& fit) const {
    const double clearance_hole_diameter = fit_diameter(clearance_hole_diameters(), fit);
    const double k = num(screw_data, "k");
    const double width = clearance_hole_diameter - thread_diameter + num(screw_data, "dk");
    Builder profile(2);
    profile.add(Rectangle(width / 2, k, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

Solid ShoulderScrew::make_body_hole(double hole_radius, double depth, double head_offset) const {
    const Solid thread_hole = Screw::make_body_hole(hole_radius, depth, head_offset);
    const Solid shoulder_hole = Solid::make_cylinder(num(screw_data, "ds") / 2, std::min(head_offset + length, depth),
                                                     Plane(Vector(0, 0, 0), std::nullopt, Vector(0, 0, -1)));
    return Solid(shoulder_hole.fuse(thread_hole).wrapped());
}

BD_WAREHOUSE_TABLE_FUNCTIONS(SocketHeadCapScrew, "socket_head_cap_parameters.csv")
BD_WAREHOUSE_SCREW_CONSTRUCTORS(SocketHeadCapScrew, "iso4762")

double SocketHeadCapScrew::calculate_thread_length() const {
    if (fastener_type == "iso4762") return std::min(max_thread_length, 2 * thread_diameter + 12 * MM);
    return asme_b18_3_thread_length();
}

std::optional<Face> SocketHeadCapScrew::head_profile() const {
    const double dk = num(screw_data, "dk"), k = num(screw_data, "k");
    Builder profile(2);
    profile.add(Rectangle(dk / 2, k, Align::MIN));
    const Vertex corner = profile.vertices().group_by(Axis::Y)[-1].sort_by(Axis::X)[-1];
    detail::fillet(profile, {corner}, k * 0.075);
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(LowProfileScrew, "low_profile_parameters.csv")

LowProfileScrew::LowProfileScrew(const std::string& size, double length_, const std::string& fastener_type_,
                                 const std::string& hand_, bool simple_, const Rotation& rotation,
                                 std::optional<Align3> align) {
    init("LowProfileScrew", size, length_, fastener_type_, hand_, simple_, 6 * MM, rotation, align);
    material = metals::mild_steel(MetalOptions{.finish = FinishSpec{finishes::zinc_plate(), finishes::black_oxide()}});
}

LowProfileScrew::LowProfileScrew(const std::string& size, double length_, const ScrewOptions& o)
    : LowProfileScrew(size, length_, o.fastener_type ? *o.fastener_type : std::string("OpenBuilds"), o.hand, o.simple,
                      o.rotation, o.align) {}

std::optional<Face> LowProfileScrew::head_profile() const {
    const double dk = num(screw_data, "dk"), k = num(screw_data, "k");
    Builder profile(2);
    profile.add(SlotOverall(dk, 2));
    profile.add(Rectangle(dk, k), Mode::INTERSECT);
    detail::split(profile, std::nullopt, Plane::YZ);
    Face face = face_of(placed_sketch(profile, Plane::XZ));
    face.move(Pos(0, 0, k / 2));
    return face;
}

#undef BD_WAREHOUSE_SCREW_CONSTRUCTORS

// ---------------------------------------------------------------------------
// Washers
// ---------------------------------------------------------------------------

Dict<std::string, std::vector<std::string>> Washer::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> type_dict;
    add_types_of_size<PlainWasher>(type_dict, "PlainWasher", size);
    add_types_of_size<ChamferedWasher>(type_dict, "ChamferedWasher", size);
    add_types_of_size<CheeseHeadWasher>(type_dict, "CheeseHeadWasher", size);
    add_types_of_size<InternalToothLockWasher>(type_dict, "InternalToothLockWasher", size);
    return type_dict;
}

Dict<std::string, std::string> Washer::tap_drill_sizes() const {
    throw std::invalid_argument("'" + class_name_ + "' object has no attribute 'tap_drill_sizes'");
}

Dict<std::string, std::string> Washer::clearance_drill_sizes() const {
    throw std::invalid_argument("'" + class_name_ + "' object has no attribute 'clearance_drill_sizes'");
}

ParamDict Washer::tap_hole_diameters() const {
    throw std::invalid_argument("'" + class_name_ + "' object has no attribute 'tap_hole_diameters'");
}

std::string Washer::info() const { return washer_class() + "(" + fastener_type + "): " + thread_size; }

double Washer::washer_thickness() const { return bounding_box().size.Z; }

double Washer::washer_diameter() const {
    double largest = 0.0;
    bool first = true;
    for (const auto& v : vertices()) {
        const double radius = (Vector(0, 0, v.Z()) - Vector(v)).length();
        if (first || radius > largest) largest = radius;
        first = false;
    }
    if (first) throw std::invalid_argument("max() arg is an empty sequence");
    return 2 * largest;
}

void Washer::init(const std::string& class_name, const std::string& size, const std::string& fastener_type_,
                  const Rotation& rotation, std::optional<Align3> align) {
    class_name_ = class_name;
    hole_locations.clear();
    washer_size = size;
    thread_size = size;
    is_metric = !thread_size.empty() && thread_size[0] == 'M';
    // Used only for clearance gap calculations
    // (a size that isn't a number, e.g. "#10", leaves it NaN: bd_warehouse keeps the str)
    const Param diameter = is_metric ? Param(detail::py_float(size.substr(1))) : imperial_str_to_float(size);
    thread_diameter = diameter.is_number() ? diameter.number() : std::numeric_limits<double>::quiet_NaN();

    const auto types = detail::types_of(class_fastener_data());
    if (types.count(fastener_type_) == 0) {
        throw std::invalid_argument(fastener_type_ + " invalid, must be one of " + detail::py_set_repr(types));
    }
    fastener_type = fastener_type_;
    const StringTable isolated = isolate_fastener_type(fastener_type, class_fastener_data());
    const auto row = isolated.get(thread_size);
    if (!row) {
        throw std::invalid_argument(size + " invalid, must be one of " +
                                    detail::py_list_repr(detail::sizes_of(fastener_type, class_fastener_data())));
    }
    washer_data = evaluate_parameter_dict(*row, is_metric);
    const Shape bd_object = make_washer();
    assign(bd_object, rotation, align);
    label = class_name + "(" + size + ", " + fastener_type + ")";
    material = metals::mild_steel(MetalOptions{.finish = finishes::zinc_plate()});
    RigidJoint("a", *this, Location());
    RigidJoint("b", *this, Pos(0, 0, washer_thickness()));
}

Shape Washer::make_washer() const { return build123d::revolve(washer_profile()).solid(); }

Face Washer::default_washer_profile() const {
    const double d1 = num(washer_data, "d1"), d2 = num(washer_data, "d2"), h = num(washer_data, "h");
    Builder profile(2);
    profile.add(detail::place(Rectangle((d2 - d1) / 2, h, Align::MIN), {Location(Vector(d1 / 2, 0))}));
    return face_of(placed_sketch(profile, Plane::XZ));
}

Face Washer::default_countersink_profile(const std::string& fit) const {
    const double clearance_hole_diameter = fit_diameter(clearance_hole_diameters(), fit);
    if (std::isnan(thread_diameter)) throw std::invalid_argument("unsupported operand type(s) for -: 'float' and 'str'");
    const double gap = clearance_hole_diameter - thread_diameter;
    const double d2 = num(washer_data, "d2"), h = num(washer_data, "h");
    Builder profile(2);
    profile.add(Rectangle(d2 / 2 + gap, h, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(PlainWasher, "plain_washer_parameters.csv")

PlainWasher::PlainWasher(const std::string& size, const std::string& fastener_type_, const Rotation& rotation,
                         std::optional<Align3> align) {
    init("PlainWasher", size, fastener_type_, rotation, align);
}

BD_WAREHOUSE_TABLE_FUNCTIONS(ChamferedWasher, "chamfered_washer_parameters.csv")

ChamferedWasher::ChamferedWasher(const std::string& size, const std::string& fastener_type_, const Rotation& rotation,
                                 std::optional<Align3> align) {
    init("ChamferedWasher", size, fastener_type_, rotation, align);
}

Face ChamferedWasher::washer_profile() const {
    const double d1 = num(washer_data, "d1"), d2 = num(washer_data, "d2"), h = num(washer_data, "h");
    Builder profile(2);
    profile.add(detail::place(Rectangle((d2 - d1) / 2, h, Align::MIN), {Location(Vector(d1 / 2, 0))}));
    const Vertex corner = profile.vertices().group_by(Axis::Y)[-1].sort_by(Axis::X)[-1];
    detail::chamfer(profile, {corner}, 0.25 * h);
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(CheeseHeadWasher, "cheese_head_washer_parameters.csv")

CheeseHeadWasher::CheeseHeadWasher(const std::string& size, const std::string& fastener_type_,
                                   const Rotation& rotation, std::optional<Align3> align) {
    init("CheeseHeadWasher", size, fastener_type_, rotation, align);
}

Face CheeseHeadWasher::washer_profile() const {
    const double d1 = num(washer_data, "d1"), d2 = num(washer_data, "d2"), h = num(washer_data, "h");
    Builder profile(2);
    profile.add(detail::place(Rectangle((d2 - d1) / 2, h, Align::MIN), {Location(Vector(d1 / 2, 0))}));
    const auto inner = profile.vertices().group_by(Axis::X)[0];
    detail::chamfer(profile, std::vector<Shape>(inner.begin(), inner.end()), 0.25 * h);
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(InternalToothLockWasher, "internal_tooth_lock_washer_parameters.csv")

InternalToothLockWasher::InternalToothLockWasher(const std::string& size, const std::string& fastener_type_,
                                                 const Rotation& rotation, std::optional<Align3> align) {
    init("InternalToothLockWasher", size, fastener_type_, rotation, align);
}

Shape InternalToothLockWasher::make_washer() const {
    const double d1 = num(washer_data, "d1"), d2 = num(washer_data, "d2"), h = num(washer_data, "h");
    const double n_value = num(washer_data, "n") / (!is_metric ? IN : MM);
    const int n = static_cast<int>(std::nearbyint(n_value));  // Python's round(): ties to even

    // tooth outer diameter
    const double dt = d1 + (d2 - d1) / 2;

    // tooth arc angle
    static const Dict<int, double> arc_angles = {{6, 30}, {8, 27}, {9, 22.5}, {10, 20}, {12, 20}, {14, 16}, {16, 12}};
    const auto at_value = arc_angles.get(n);
    if (!at_value) throw std::invalid_argument("unsupported operand type(s) for *: 'float' and 'NoneType'");
    const double at = *at_value;

    // tooth inner and outer widths
    const double w1 = d1 * M_PI * at / 360, w2 = dt * M_PI * at / 360;

    // solve for angle to rotate inner face of tooth -> overall height == 2*h per DIN 6797
    const double a = w1 / 2, b = h / 2;
    const double radicand = std::pow(w1 / 2, 2) + std::pow(h / 2, 2) - std::pow(h, 2);
    if (radicand < 0) throw std::invalid_argument("math domain error");
    const double c = h, d = std::sqrt(radicand);
    const double angle = (-std::atan(d / c) + std::atan(a / b)) * 180 / M_PI;

    // tooth inner and outer faces, projected onto their cylinders
    const Face f1 = (Rotation(0, angle, 0) * (Plane::XZ * Rectangle(w1, h))).faces()[0];
    const Face f2 = (Plane::XZ * Rectangle(w2, h)).faces()[0];
    const auto inner_face = f1.project_to_shape(Cylinder(d1 / 2, 10 * h), Vector(0, -1, 0)).faces();
    const auto outer_face = f2.project_to_shape(Cylinder(dt / 2, 10 * h), Vector(0, -1, 0)).faces();

    const Part teeth = build123d::loft(std::vector<Shape>{inner_face.at(0), outer_face.at(0)});
    const Part body = Cylinder(d2 / 2, h) - Cylinder(dt / 2, h);
    return Pos(0, 0, h / 2) * (body + (PolarLocations(0, n) * teeth));
}

}  // namespace bd_warehouse
