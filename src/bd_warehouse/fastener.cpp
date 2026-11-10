// Port of bd_warehouse's fastener.py.
#include "bd_warehouse/fastener.hpp"

#include <algorithm>
#include <cmath>

#include <BRepOffsetAPI_MakeFilling.hxx>
#include <GeomAbs_Shape.hxx>
#include <TopoDS.hxx>

#include "../py_compat.hpp"
#include "builder.hpp"
#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;

namespace detail {

Sketch placed_sketch(const Builder& sketch, const Plane& plane) {
    return Sketch(place(sketch.product(), {Location()}, {Location(plane)}).wrapped());
}

Face make_surface_from_edges(const std::vector<Edge>& edges) {
    BRepOffsetAPI_MakeFilling surface(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
    for (const auto& edge : edges) surface.Add(TopoDS::Edge(edge.wrapped()), GeomAbs_C0);
    Face surface_face;
    try {
        surface.Build();
        surface_face = Face(surface.Shape());
    } catch (const Standard_Failure&) {
        throw std::runtime_error("Error building non-planar face with provided exterior");
    }
    surface_face = surface_face.fix();
    if (!surface_face.is_valid()) throw std::runtime_error("non planar face is invalid");
    return surface_face;
}

std::set<std::string> types_of(const StringTable& fastener_data) {
    if (fastener_data.empty()) throw std::out_of_range("list index out of range");
    std::set<std::string> out;
    for (const auto& [key, value] : fastener_data.begin()->second) out.insert(py_split(key, ":")[0]);
    return out;
}

std::vector<std::string> sizes_of(const std::string& fastener_type, const StringTable& fastener_data) {
    return isolate_fastener_type(fastener_type, fastener_data).keys();
}

std::string py_set_repr(const std::set<std::string>& items) {
    if (items.empty()) return "set()";
    std::string out = "{";
    bool first = true;
    for (const auto& item : items) {
        out += (first ? "" : ", ") + py_repr(item);
        first = false;
    }
    return out + "}";
}

std::string fits_repr(const ParamDict& diameters) { return py_list_repr(diameters.keys()); }

}  // namespace detail

using detail::placed_sketch;

namespace {

/// The Face of a Sketch (Shape.face(): the only one).
Face face_of(const Shape& shape) { return shape.face(); }

double num(const ParamDict& data, const std::string& key) { return data.at(key).number(); }

FinishedMaterial zinc_plated_mild_steel() {
    return metals::mild_steel(MetalOptions{.finish = finishes::zinc_plate()});
}

/// The fit's diameter of a hole table (std::invalid_argument for another fit).
double fit_diameter(const ParamDict& diameters, const std::string& fit) {
    const auto found = diameters.get(fit);
    if (!found) throw std::invalid_argument(fit + " invalid, must be one of " + detail::fits_repr(diameters));
    return found->number();
}

}  // namespace

// ---------------------------------------------------------------------------
// Tables and helpers
// ---------------------------------------------------------------------------

double polygon_diagonal(double width, int num_sides) { return width / std::cos(M_PI / num_sides); }

bool method_exists(const std::string& cls, const std::string& method) {
    const std::set<std::string>* methods = detail::class_methods(cls);
    if (methods == nullptr) throw std::invalid_argument(cls + " is not a bd_warehouse fastener or bearing class");
    return methods->count(method) > 0;
}

StringTable read_fastener_parameters_from_csv(const std::string& filename) {
    StringTable parameters;
    const detail::DictReader reader = detail::dict_reader(detail::data_file("data/" + filename));
    if (!reader.fieldnames) throw std::invalid_argument("No header found in " + filename);
    const std::string& first = reader.fieldnames->at(0);
    for (const auto& row : reader.rows) {
        if (row.rest) throw std::invalid_argument("the row of " + filename + " has more values than columns");
        Dict<std::string, std::string> values;
        for (const auto& [name, value] : row.fields) {
            if (!value) throw std::invalid_argument("the row of " + filename + " has fewer values than columns");
            values[name] = *value;
        }
        const std::string key = values.at(first);
        values.pop(first);
        parameters[key] = values;
    }
    return parameters;
}

std::pair<double, double> decode_imperial_size(const std::string& size) {
    static const Dict<std::string, double> imperial_numbered_sizes = {
        {"#0000", 0.0210 * IN}, {"#000", 0.0340 * IN}, {"#00", 0.0470 * IN}, {"#0", 0.0600 * IN},
        {"#1", 0.0730 * IN},    {"#2", 0.0860 * IN},   {"#3", 0.0990 * IN},  {"#4", 0.1120 * IN},
        {"#5", 0.1250 * IN},    {"#6", 0.1380 * IN},   {"#8", 0.1640 * IN},  {"#10", 0.1900 * IN},
        {"#12", 0.2160 * IN}};
    const auto sizes = detail::py_split(size, "-");
    if (size.empty()) throw std::out_of_range("string index out of range");
    double major_diameter;
    if (size[0] == '#') {
        major_diameter = imperial_numbered_sizes.at(sizes[0]);
    } else {
        major_diameter = imperial_str_to_float(sizes[0]).number();
    }
    if (sizes.size() < 2) throw std::out_of_range("list index out of range");
    const double tpi = imperial_str_to_float(sizes[1]).number();
    if (tpi / IN == 0) throw std::domain_error("float division by zero");
    const double pitch = IN / (tpi / IN);
    return {major_diameter, pitch};
}

Param metric_str_to_float(const std::string& measure) {
    if (is_safe(measure)) return detail::py_eval_number(measure).value;
    return Param(measure);
}

ParamDict evaluate_parameter_dict(const Dict<std::string, std::string>& parameters, bool is_metric) {
    ParamDict measurements;
    for (const auto& [params, value] : parameters) {
        measurements[params] = is_metric ? metric_str_to_float(value) : imperial_str_to_float(value);
    }
    return measurements;
}

ParamTable evaluate_parameter_dict_of_dict(const StringTable& parameters, bool is_metric) {
    ParamTable measurements;
    for (const auto& [key, value] : parameters) measurements[key] = evaluate_parameter_dict(value, is_metric);
    return measurements;
}

StringTable isolate_fastener_type(const std::string& target_fastener, const StringTable& fastener_data) {
    StringTable result;
    for (const auto& [size, parameters] : fastener_data) {
        Dict<std::string, std::string> dimension_dict;
        for (const auto& [type_dimension, value] : parameters) {
            const auto parts = detail::py_split(detail::py_strip(type_dimension), ":");
            if (parts.size() > 2) throw std::invalid_argument("too many values to unpack (expected 2)");
            if (parts.size() < 2) {
                throw std::invalid_argument("not enough values to unpack (expected 2, got " + std::to_string(parts.size()) +
                                            ")");
            }
            if (target_fastener == parts[0] && value != "") dimension_dict[parts[1]] = value;
        }
        if (!dimension_dict.empty()) result[size] = dimension_dict;
    }
    return result;
}

Dict<std::string, double> read_drill_sizes() {
    Dict<std::string, double> drill_sizes;
    const detail::DictReader reader = detail::dict_reader(detail::data_file("data/drill_sizes.csv"));
    if (!reader.fieldnames) throw std::invalid_argument("No header found in drill_sizes.csv");
    const auto& names = *reader.fieldnames;
    for (const auto& row : reader.rows) drill_sizes[row.str(names.at(0))] = detail::py_float(row.str(names.at(1))) * IN;
    return drill_sizes;
}

ParamTable lookup_drill_diameters(const StringTable& drill_hole_sizes) {
    const Dict<std::string, double> drill_sizes = read_drill_sizes();
    ParamTable drill_hole_diameters;
    for (const auto& [size, drill_data] : drill_hole_sizes) {
        ParamDict hole_data;
        for (const auto& [fit, drill] : drill_data) {
            if (const auto found = drill_sizes.get(drill)) {
                hole_data[fit] = *found;
            } else if (!size.empty() && size[0] == 'M') {
                hole_data[fit] = detail::py_float(drill);
            } else {
                hole_data[fit] = imperial_str_to_float(drill);
            }
        }
        drill_hole_diameters[size] = hole_data;
    }
    return drill_hole_diameters;
}

Dict<std::string, std::vector<double>> lookup_nominal_screw_lengths() {
    Dict<std::string, std::vector<double>> nominal_screw_lengths;
    for (const auto& row : detail::dict_reader(detail::data_file("data/nominal_screw_lengths.csv")).rows) {
        const double unit_factor = row.str("Unit") == "mm" ? MM : IN;
        std::vector<double> sizes;
        for (const auto& size : detail::py_split(row.str("Nominal_Sizes"), ",")) {
            sizes.push_back(unit_factor * detail::py_float(size));
        }
        nominal_screw_lengths[row.str("Screw_Type")] = sizes;
    }
    return nominal_screw_lengths;
}

std::pair<Face, double> cross_recess(const std::string& size) {
    static const Dict<std::string, double> widths = {
        {"PH0", 1.9}, {"PH1", 3.1}, {"PH2", 5.3}, {"PH3", 6.8}, {"PH4", 10.0}};
    static const Dict<std::string, double> depths = {
        {"PH0", 1.1}, {"PH1", 2.0}, {"PH2", 3.27}, {"PH3", 3.53}, {"PH4", 5.88}};
    const auto width = widths.get(size);
    if (!width) {
        throw std::invalid_argument(size +
                                    " is an invalid cross size {'PH0': 1.9, 'PH1': 3.1, 'PH2': 5.3, 'PH3': 6.8, "
                                    "'PH4': 10.0}");
    }
    const double m = *width;
    Builder recess(2);
    recess.add(Rectangle(m, m / 6));
    recess.add(Rectangle(m / 6, m));
    const auto inner = recess.vertices().group_by(SortBy::DISTANCE)[0];
    detail::fillet(recess, std::vector<Shape>(inner.begin(), inner.end()), m / 3);
    return {recess.face(), depths.at(size)};
}

Face hex_recess(double size) {
    Builder plan(2);
    plan.add(RegularPolygon(size / 2, 6, false));
    return plan.face();
}

std::pair<Face, double> hexalobular_recess(const std::string& size) {
    const ParamTable table = evaluate_parameter_dict_of_dict(read_fastener_parameters_from_csv("iso10664def.csv"));
    const auto found = table.get(size);
    if (!found) throw std::invalid_argument(size + " is an invalid hexalobular size");
    const ParamDict& screw_data = *found;
    const double A = num(screw_data, "A"), B = num(screw_data, "B"), Re = num(screw_data, "Re");

    // Given the outer (A) and inner (B) diameters and the external radius (Re), calculate the
    // internal radius
    const double sqrt_3 = std::sqrt(3.0);
    const double Ri = (std::pow(A, 2) - sqrt_3 * A * B - 4 * A * Re + std::pow(B, 2) + 2 * sqrt_3 * B * Re) /
                      (2 * (sqrt_3 * A - 2 * B - 2 * sqrt_3 * Re + 4 * Re));

    const Vector center_external_arc[] = {Vector(0, A / 2 - Re), Vector(sqrt_3 * (A / 2 - Re) / 2, A / 4 - Re / 2)};
    const Vector center_internal_arc(B / 4 + Ri / 2, sqrt_3 * (B / 2 + Ri) / 2);

    // Determine where the two arcs are tangent (i.e. touching)
    const Vector tangent_points[] = {
        center_external_arc[0] + (center_internal_arc - center_external_arc[0]).normalized() * Re,
        center_external_arc[1] + (center_internal_arc - center_external_arc[1]).normalized() * Re};

    // Create one sixth of the wire and repeat it
    Builder plan(2);
    Builder arc(1);  // a private BuildLine
    arc.add(RadiusArc(Vector(0, A / 2), tangent_points[0], Re));
    arc.add(RadiusArc(tangent_points[0], tangent_points[1], -Ri));
    arc.add(RadiusArc(tangent_points[1], Vector(sqrt_3 * A / 4, A / 4), Re));
    detail::insert(plan, {arc.wire()}, PolarLocations(0, 6));
    detail::make_face(plan);
    return {plan.face(), 0.6 * A};
}

Face slot_recess(double width, double length) { return Face::make_rect(width, length); }

std::pair<Face, double> square_recess(const std::string& size) {
    static const Dict<std::string, double> widths = {
        {"R00", 1.80}, {"R0", 2.31}, {"R1", 2.86}, {"R2", 3.38}, {"R3", 4.85}};
    static const Dict<std::string, double> depths = {
        {"R00", 1.85}, {"R0", 2.87}, {"R1", 3.56}, {"R2", 4.19}, {"R3", 5.11}};
    const auto width = widths.get(size);
    if (!width) {
        throw std::invalid_argument(size +
                                    " is an invalid square size {'R00': 1.8, 'R0': 2.31, 'R1': 2.86, 'R2': 3.38, "
                                    "'R3': 4.85}");
    }
    return {Face::make_rect(*width, *width), depths.at(size)};
}

// ---------------------------------------------------------------------------
// Fastener
// ---------------------------------------------------------------------------

const StringTable& Fastener::clearance_hole_drill_sizes() {
    static const StringTable table = read_fastener_parameters_from_csv("clearance_hole_sizes.csv");
    return table;
}

const ParamTable& Fastener::clearance_hole_data() {
    static const ParamTable table = lookup_drill_diameters(clearance_hole_drill_sizes());
    return table;
}

const StringTable& Fastener::tap_hole_drill_sizes() {
    static const StringTable table = read_fastener_parameters_from_csv("tap_hole_sizes.csv");
    return table;
}

const ParamTable& Fastener::tap_hole_data() {
    static const ParamTable table = lookup_drill_diameters(tap_hole_drill_sizes());
    return table;
}

Dict<std::string, std::string> Fastener::tap_drill_sizes() const {
    const auto found = tap_hole_drill_sizes().get(thread_size);
    if (!found) throw std::invalid_argument("No tap hole data for size " + thread_size);
    return *found;
}

ParamDict Fastener::tap_hole_diameters() const {
    const auto found = tap_hole_data().get(thread_size);
    if (!found) throw std::invalid_argument("No tap hole data for size " + thread_size);
    return *found;
}

Dict<std::string, std::string> Fastener::clearance_drill_sizes() const {
    const auto found = clearance_hole_drill_sizes().get(detail::py_split(thread_size, "-")[0]);
    if (!found) throw std::invalid_argument("No clearance hole data for size " + thread_size);
    return *found;
}

ParamDict Fastener::clearance_hole_diameters() const {
    const auto found = clearance_hole_data().get(detail::py_split(thread_size, "-")[0]);
    if (!found) throw std::invalid_argument("No clearance hole data for size " + thread_size);
    return *found;
}

// ---------------------------------------------------------------------------
// Nut
// ---------------------------------------------------------------------------

namespace {

template <class C>
void add_types_of_size(Dict<std::string, std::vector<std::string>>& type_dict, const std::string& name,
                       const std::string& size) {
    for (const auto& fastener_type : C::types()) {
        const auto sizes = C::sizes(fastener_type);
        if (std::find(sizes.begin(), sizes.end(), size) != sizes.end()) type_dict[name].push_back(fastener_type);
    }
}

}  // namespace

Dict<std::string, std::vector<std::string>> Nut::select_by_size(const std::string& size) {
    Dict<std::string, std::vector<std::string>> type_dict;
    add_types_of_size<DomedCapNut>(type_dict, "DomedCapNut", size);
    add_types_of_size<HeatSetNut>(type_dict, "HeatSetNut", size);
    add_types_of_size<HexNut>(type_dict, "HexNut", size);
    add_types_of_size<HexNutWithFlange>(type_dict, "HexNutWithFlange", size);
    add_types_of_size<UnchamferedHexagonNut>(type_dict, "UnchamferedHexagonNut", size);
    add_types_of_size<SquareNut>(type_dict, "SquareNut", size);
    return type_dict;
}

void Nut::init(const std::string& class_name, const std::string& size, const std::string& fastener_type_,
               const std::string& hand_, bool simple_, const Rotation& rotation, std::optional<Align3> align) {
    class_name_ = class_name;
    hole_locations.clear();

    nut_size = detail::py_strip(size);
    const auto size_parts = detail::py_split(nut_size, "-");
    if (size_parts.size() < 2) {
        throw std::invalid_argument(detail::py_list_repr(size_parts) +
                                    " invalid, must be formatted as size-pitch(-length) or size-TPI(-length) where "
                                    "length is optional");
    }
    thread_size = size_parts[0] + "-" + size_parts[1];
    if (size_parts.size() >= 3) length_size = size_parts[2];
    is_metric = thread_size[0] == 'M';
    if (is_metric) {
        thread_diameter = detail::py_float(size_parts[0].substr(1));
        thread_pitch = detail::py_float(size_parts[1]);
    } else {
        std::tie(thread_diameter, thread_pitch) = decode_imperial_size(thread_size);
    }

    const auto types = detail::types_of(class_fastener_data());
    if (types.count(fastener_type_) == 0) {
        throw std::invalid_argument(fastener_type_ + " invalid, must be one of " + detail::py_set_repr(types));
    }
    fastener_type = fastener_type_;
    if (hand_ != "left" && hand_ != "right") throw std::invalid_argument(hand_ + " invalid, must be one of 'left' or 'right'");
    hand = hand_;
    simple = simple_;
    socket_clearance = 6 * MM;  // Used as extra clearance when countersinking
    const StringTable isolated = isolate_fastener_type(fastener_type, class_fastener_data());
    const auto row = isolated.get(nut_size);
    if (!row) {
        throw std::invalid_argument(size + " invalid, must be one of " +
                                    detail::py_list_repr(detail::sizes_of(fastener_type, class_fastener_data())));
    }
    nut_data = evaluate_parameter_dict(*row, is_metric);

    const Shape bd_object = make_nut();
    // Unwrap the Compound if possible
    if (!bd_object.is_null() && bd_object.wrapped().ShapeType() == TopAbs_COMPOUND && bd_object.solids().size() == 1) {
        assign(bd_object.solid(), rotation, align);
    } else {
        assign(bd_object, rotation, align);
    }
    label = class_name + "(" + size + ", " + fastener_type + ")";
    material = zinc_plated_mild_steel();
    RigidJoint("a", *this, Location());
    RigidJoint("b", *this, Pos(0, 0, nut_thickness()));
}

Shape Nut::make_nut() const {
    const Face profile = nut_profile();
    const double max_nut_height = profile.bounding_box().max.Z;
    const double nut_thread_height = num(nut_data, "m");

    // Create the basic nut shape
    const Part revolved = build123d::revolve(profile, Axis::Z);

    // Modify the head to conform to the shape of nut_plan (e.g. hex); some nuts (e.g. domed
    // nuts) extend beyond the threaded section
    const Part nut_blank = build123d::extrude(nut_plan(), max_nut_height, ExtrudeOptions{.dir = Vector(0, 0, 1)}) -
                           Solid::make_cylinder(thread_diameter / 2, nut_thread_height);
    const auto nut_pieces = revolved.intersect(nut_blank);
    if (nut_pieces.empty()) throw std::runtime_error("nut profile and plan do not intersect");
    Shape nut = nut_pieces[0];

    // Add a flange as it exists outside of the head plan
    if (const auto flange_face = flange_profile()) {
        const Part flange = build123d::revolve(build123d::split(*flange_face, Plane::YZ.offset(thread_diameter / 2 + 0.1)), Axis::Z);
        nut = nut.fuse(flange);
    }
    nut.label = "body";

    // Add the thread to the nut body
    if (!simple) {
        IsoThread thread(thread_diameter, thread_pitch, num(nut_data, "m"),
                         IsoThreadOptions{.external = false, .hand = hand, .end_finishes = {"fade", "fade"}});
        thread.label = "thread";
        Part assembly;
        assembly.children = std::vector<Shape>{nut, thread};
        return assembly;
    }
    return nut;
}

std::string Nut::info() const { return nut_class() + "(" + fastener_type + "): " + thread_size; }

double Nut::nut_thickness() const { return bounding_box().max.Z; }

double Nut::nut_diameter() const {
    const auto bottom_vertices = nut_plan().vertices();
    if (bottom_vertices.size() < 3) throw std::invalid_argument("Invalid nut: " + class_name_);
    const Edge bottom_arc = Edge::make_three_point_arc(Vector(bottom_vertices[0]), Vector(bottom_vertices[1]),
                                                       Vector(bottom_vertices[2]));
    return 2 * bottom_arc.radius();
}

Face Nut::default_nut_profile() const {
    const double m = num(nut_data, "m"), s = num(nut_data, "s");
    const double e = polygon_diagonal(s, 6);
    // Chamfer angle must be between 15 and 30 degrees
    const double cs = (e - s) * std::tan(detail::radians(15)) / 2;
    // When intersecting a revolved shape with an extruded polygon the OCCT core may fail unless
    // the polygon is slightly larger than the circle, so the profile is reduced a little
    Builder profile(2);
    profile.add(Polygon({{0, 0}, {s / 2, 0}, {e / 2 - 0.001, cs}, {e / 2 - 0.001, m - cs}, {s / 2, m}, {0, m}, {0, 0}},
                        0.0, Align::NONE));
    return face_of(placed_sketch(profile, Plane::XZ));
}

Face Nut::default_nut_plan() const {
    Builder plan(2);
    plan.add(RegularPolygon(num(nut_data, "s") / 2, 6, false));
    return plan.face();
}

Face Nut::default_countersink_profile(const std::string&) const {
    const double m = num(nut_data, "m"), s = num(nut_data, "s");
    const double width = polygon_diagonal(s, 6) + socket_clearance;
    Builder profile(2);
    profile.add(Rectangle(width / 2, m, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

// ---- DomedCapNut

#define BD_WAREHOUSE_NUT_CONSTRUCTORS(Class, default_type)                                                         \
    Class::Class(const std::string& size, const std::string& fastener_type_, const std::string& hand_, bool simple_, \
                 const Rotation& rotation, std::optional<Align3> align) {                                          \
        init(#Class, size, fastener_type_, hand_, simple_, rotation, align);                                       \
    }                                                                                                              \
    Class::Class(const std::string& size, const NutOptions& o) {                                                   \
        init(#Class, size, o.fastener_type ? *o.fastener_type : std::string(default_type), o.hand, o.simple,       \
             o.rotation, o.align);                                                                                 \
    }

BD_WAREHOUSE_TABLE_FUNCTIONS(DomedCapNut, "domed_cap_nut_parameters.csv")
BD_WAREHOUSE_NUT_CONSTRUCTORS(DomedCapNut, "din1587")

Face DomedCapNut::nut_profile() const {
    const double dk = num(nut_data, "dk"), m = num(nut_data, "m"), s = num(nut_data, "s");
    const double e = polygon_diagonal(s, 6);
    // Chamfer angle must be between 15 and 30 degrees
    const double cs = (e - s) * std::tan(detail::radians(15)) / 2;
    Builder profile(2);
    {
        Builder line(1);
        line.add(Polyline({{1 * MM, 0}, {s / 2, 0}, {e / 2, cs}, {e / 2, m - cs}, {s / 2, m}, {dk / 2, m}}));
        line.add(RadiusArc(Vector(dk / 2, m), Vector(0, m + dk / 2), -dk / 2));
        line.add(Line(Vector(0, m + dk / 2), Vector(0, m + dk / 2 - 1 * MM)));
        line.add(RadiusArc(Vector(0, m + dk / 2 - 1 * MM), Vector(dk / 2 - 1 * MM, m), (dk / 2 - 1 * MM)));
        line.add(Polyline({{dk / 2 - 1 * MM, m}, {1 * MM, m}, {1 * MM, 0}}));
        profile.publish(line.product());
    }
    detail::make_face(profile);
    return face_of(placed_sketch(profile, Plane::XZ));
}

std::optional<Face> DomedCapNut::countersink_profile(const std::string&) const {
    const double dk = num(nut_data, "dk"), m = num(nut_data, "m"), s = num(nut_data, "s");
    const double width = polygon_diagonal(s, 6) + socket_clearance;
    Builder profile(2);
    profile.add(Rectangle(width / 2, m + dk / 2, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

// ---- HeatSetNut

BD_WAREHOUSE_TABLE_FUNCTIONS(HeatSetNut, "heatset_nut_parameters.csv")

HeatSetNut::HeatSetNut(const std::string& size, const std::string& fastener_type_, const std::string& hand_,
                       bool simple_, const Rotation& rotation, std::optional<Align3> align) {
    init("HeatSetNut", size, fastener_type_, hand_, simple_, rotation, align);
}

HeatSetNut::HeatSetNut(const std::string& size, const NutOptions& o) {
    init("HeatSetNut", size, o.fastener_type ? *o.fastener_type : std::string("McMaster-Carr"), o.hand, o.simple,
         o.rotation, o.align);
}

double HeatSetNut::nut_diameter() const {
    return detail::py_float(fastener_data().at(nut_size).at(fastener_type + ":s"));
}

std::vector<Face> HeatSetNut::knurled_cylinder_faces(double diameter, double bottom_hole_radius,
                                                     double top_hole_radius, double height, double knurl_depth,
                                                     double pitch, int tip_count, const std::string& hand) {
    // Start by creating helical edges for the inside and outside of the knurling
    const bool lefthand = hand == "left";
    std::vector<Edge> inside_edges, outside_edges;
    for (int i = 0; i < tip_count; ++i) {
        inside_edges.push_back(
            Edge::make_helix(pitch, height, diameter / 2 - knurl_depth, Vector(0, 0, 0), Vector(0, 0, 1), 0.0, lefthand)
                .rotate(Axis::Z, i * 360.0 / tip_count));
    }
    for (int i = 0; i < tip_count; ++i) {
        outside_edges.push_back(Edge::make_helix(pitch, height, diameter / 2, Vector(0, 0, 0), Vector(0, 0, 1), 0.0, lefthand)
                                    .rotate(Axis::Z, (i + 0.5) * 360 / tip_count));
    }
    // Connect the ends of the helical edges into star shaped bottom and top faces: two zigzag
    // lines per tip, inside -> outside -> next inside
    std::vector<Edge> bottom_edges, top_edges;
    for (int i = 0; i < tip_count; ++i) {
        const Edge& next_inside_edge = inside_edges[static_cast<std::size_t>((i + 1) % tip_count)];
        for (int end = 0; end < 2; ++end) {
            std::vector<Edge>& edges = end == 0 ? bottom_edges : top_edges;
            edges.push_back(Edge::make_line(inside_edges[i].position_at(end), outside_edges[i].position_at(end)));
            edges.push_back(Edge::make_line(outside_edges[i].position_at(end), next_inside_edge.position_at(end)));
        }
    }

    // Build the faces from the edges
    std::vector<Face> outside_faces;
    for (int i = 0; i < tip_count; ++i) {
        outside_faces.push_back(detail::make_surface_from_edges(
            {inside_edges[i], outside_edges[i], bottom_edges[2 * i], top_edges[2 * i]}));
        outside_faces.push_back(detail::make_surface_from_edges(
            {outside_edges[i], inside_edges[static_cast<std::size_t>((i + 1) % tip_count)], bottom_edges[2 * i + 1],
             top_edges[2 * i + 1]}));
    }

    // Create the top and bottom faces with holes in them
    const Face bottom_face(Wire(bottom_edges), {Wire(Edge::make_circle(bottom_hole_radius))});
    const Face top_face(Wire(top_edges), {Wire(Edge::make_circle(top_hole_radius, Plane::XY.offset(height)))});

    std::vector<Face> out{bottom_face, top_face};
    out.insert(out.end(), outside_faces.begin(), outside_faces.end());
    return out;
}

double HeatSetNut::fill_factor() const {
    const auto drill_sizes = read_drill_sizes();
    const double hole_radius = drill_sizes.at(detail::py_strip(nut_data.at("drill").text())) / 2;
    const double m = num(nut_data, "m");
    const double heatset_volume = volume() + m * M_PI * std::pow(thread_diameter / 2, 2);
    const double hole_volume = m * M_PI * std::pow(hole_radius, 2);
    return heatset_volume / hole_volume;
}

Shape HeatSetNut::make_nut() const {
    const double dc = num(nut_data, "dc"), m = num(nut_data, "m"), s = num(nut_data, "s");
    const Part nut_base = Cylinder(dc / 2, 0.11 * m, Align3(Align::CENTER, Align::CENTER, Align::MIN)) +
                          Cylinder(0.425 * dc, 0.24 * m, Align3(Align::CENTER, Align::CENTER, Align::MIN));
    const auto sorted_faces = nut_base.faces().sort_by(Axis::Z);
    const Face base_bottom_face = sorted_faces[0].make_holes({Wire(Edge::make_circle(thread_diameter / 2))});
    const auto base_outside_faces = sorted_faces.slice(1, -1);

    // Create the lower knurled section Faces
    std::vector<Face> lower_knurl_faces;
    for (const auto& f : knurled_cylinder_faces(s, 0.425 * dc, 0.425 * dc, 0.33 * m, 0.1 * s, 3 * m,
                                                static_cast<int>(num(nut_data, "knurls")), "right")) {
        lower_knurl_faces.push_back(f.translate(Vector(0, 0, 0.24 * m)));
    }
    // Create the Face in the gap between the knurled sections
    const Face nut_middle_face =
        Face::extrude(Edge::make_circle(0.425 * dc, Plane::XY.offset(0.57 * m)), Vector(0, 0, 0.1 * m));

    // Create the Faces in the upper knurled section
    std::vector<Face> upper_knurl_faces;
    for (const auto& f : knurled_cylinder_faces(s, 0.425 * dc, thread_diameter / 2, 0.33 * m, 0.1 * s, 3 * m, 20, "left")) {
        upper_knurl_faces.push_back(f.translate(Vector(0, 0, 0.67 * m)));
    }

    // Create the Face for the inside of the nut
    const Face thread_hole_face = Face::extrude(Edge::make_circle(thread_diameter / 2), Vector(0, 0, m));

    // Build a Shell of the nut from all of the Faces
    std::vector<Face> faces{base_bottom_face, nut_middle_face, thread_hole_face};
    faces.insert(faces.end(), base_outside_faces.begin(), base_outside_faces.end());
    faces.insert(faces.end(), lower_knurl_faces.begin(), lower_knurl_faces.end());
    faces.insert(faces.end(), upper_knurl_faces.begin(), upper_knurl_faces.end());
    const Shell nut_shell(faces);

    // Finally create the Solid from the Shell
    Solid nut(nut_shell);
    nut.label = "body";

    // Add the thread to the nut body
    if (!simple) {
        const IsoThread thread(thread_diameter, thread_pitch, m,
                               IsoThreadOptions{.external = false, .hand = hand, .end_finishes = {"fade", "fade"}});
        Part assembly;
        assembly.children = std::vector<Shape>{nut, thread};
        return assembly;
    }
    return nut;
}

Face HeatSetNut::nut_profile() const { throw std::logic_error("HeatSetNut has no nut_profile"); }

Face HeatSetNut::nut_plan() const { throw std::logic_error("HeatSetNut has no nut_plan"); }

Sketch HeatSetNut::countersink_profile(double manufacturing_compensation) const {
    const auto drill_sizes = read_drill_sizes();
    const double hole_radius =
        drill_sizes.at(detail::py_strip(nut_data.at("drill").text())) / 2 + manufacturing_compensation;
    Builder profile(2);
    profile.add(Rectangle(hole_radius, num(nut_data, "m"), Align::MIN));
    return placed_sketch(profile, Plane::XZ);
}

std::optional<Face> HeatSetNut::countersink_profile(const std::string& fit) const {
    throw std::invalid_argument("unsupported operand type(s) for +: 'float' and 'str' (HeatSetNut's countersink_profile "
                                "takes a manufacturing_compensation, not the fit '" + fit + "')");
}

// ---- HexNut, HexNutWithFlange, UnchamferedHexagonNut, SquareNut

BD_WAREHOUSE_TABLE_FUNCTIONS(HexNut, "hex_nut_parameters.csv")
BD_WAREHOUSE_NUT_CONSTRUCTORS(HexNut, "iso4032")

BD_WAREHOUSE_TABLE_FUNCTIONS(HexNutWithFlange, "hex_nut_with_flange_parameters.csv")
BD_WAREHOUSE_NUT_CONSTRUCTORS(HexNutWithFlange, "din1665")

namespace {

/// The flange profile of hexagon nuts and bolts (dc, c from the table).
Face hex_flange_profile(double dc, double c) {
    const double flange_angle = 25;
    const Vector tangent_point = Vector((c / 2) * std::cos(detail::radians(90 - flange_angle)),
                                        (c / 2) * std::sin(detail::radians(90 - flange_angle))) +
                                 Vector((dc - c) / 2, c / 2);
    Builder profile(2);
    {
        Builder line(1);
        const Line l1(Vector(0, 0), Vector(dc / 2 - c / 2, 0));
        line.add(l1);
        line.add(RadiusArc(l1.position_at(1), tangent_point, -c / 2));
        const PolarLine l3(tangent_point, dc / 2 - c / 2, 180 - flange_angle);
        line.add(l3);
        line.add(Polyline({l3.position_at(1), {0, l3.position_at(1).Y}, {0, 0}}));
        profile.publish(line.product());
    }
    detail::make_face(profile);
    return face_of(placed_sketch(profile, Plane::XZ));
}

}  // namespace

std::optional<Face> HexNutWithFlange::flange_profile() const {
    return hex_flange_profile(num(nut_data, "dc"), num(nut_data, "c"));
}

std::optional<Face> HexNutWithFlange::countersink_profile(const std::string& fit) const {
    const double clearance_hole_diameter = fit_diameter(clearance_hole_diameters(), fit);
    const double dc = num(nut_data, "dc"), s = num(nut_data, "s"), m = num(nut_data, "m");
    const double clearance = clearance_hole_diameter - thread_diameter;
    const double width = std::max(dc + clearance, polygon_diagonal(s, 6) + socket_clearance);
    Builder profile(2);
    profile.add(Rectangle(width / 2, m, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(UnchamferedHexagonNut, "unchamfered_hex_nut_parameters.csv")
BD_WAREHOUSE_NUT_CONSTRUCTORS(UnchamferedHexagonNut, "iso4036")

Face UnchamferedHexagonNut::nut_profile() const {
    const double m = num(nut_data, "m"), s = num(nut_data, "s");
    Builder profile(2);
    profile.add(Rectangle(polygon_diagonal(s, 6) / 2 - 0.001, m, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

BD_WAREHOUSE_TABLE_FUNCTIONS(SquareNut, "square_nut_parameters.csv")
BD_WAREHOUSE_NUT_CONSTRUCTORS(SquareNut, "din557")

Face SquareNut::nut_profile() const {
    const double m = num(nut_data, "m"), s = num(nut_data, "s");
    const double e = polygon_diagonal(s, 4);
    // Chamfer angle must be between 15 and 30 degrees
    const double cs = (e - s) * std::tan(detail::radians(15)) / 2;
    Builder profile(2);
    profile.add(Polygon({{0, 0}, {e / 2 - 0.001, 0}, {e / 2 - 0.001, m - cs}, {s / 2, m}, {0, m}, {0, 0}}, 0.0,
                        Align::NONE));
    return face_of(placed_sketch(profile, Plane::XZ));
}

Face SquareNut::nut_plan() const { return Face::make_rect(num(nut_data, "s"), num(nut_data, "s")); }

std::optional<Face> SquareNut::countersink_profile(const std::string&) const {
    const double m = num(nut_data, "m"), s = num(nut_data, "s");
    const double width = polygon_diagonal(s, 4) + socket_clearance;
    Builder profile(2);
    profile.add(Rectangle(width / 2, m, Align::MIN));
    return face_of(placed_sketch(profile, Plane::XZ));
}

#undef BD_WAREHOUSE_NUT_CONSTRUCTORS

}  // namespace bd_warehouse

// The screws, washers and holes are in fastener_screws.cpp.
