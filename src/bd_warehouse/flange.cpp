// Port of bd_warehouse's flange.py.
#include "bd_warehouse/flange.hpp"

#include <algorithm>
#include <cmath>

#include "bd_warehouse/pipe.hpp"
#include "bd_warehouse/thread.hpp"
#include "builder.hpp"
#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;
using detail::placed_sketch;

#include "flange_tables.inc"

const std::vector<std::string>& flange_nps() {
    static const std::vector<std::string> values = {"1/2", "3/4", "1",  "1 1/4", "1 1/2", "2",  "2 1/2",
                                                    "3",   "4",   "5",  "6",     "8",     "10", "12",
                                                    "14",  "16",  "18", "20",    "22",    "24"};
    return values;
}

const std::vector<std::string>& flange_face_types() {
    static const std::vector<std::string> values = {"Flat", "Raised", "Ring", "Tongue", "Groove", "Male", "Female"};
    return values;
}

const std::vector<int>& flange_classes() {
    static const std::vector<int> values = {150, 300, 400, 600, 900, 1500, 2500};
    return values;
}

namespace {

/// A number of a flange table (Python calculates with the value; text raises TypeError).
double num(const Param& value) { return value.number(); }

/// The bolt hole diameter from the table's bolt size (imperial_str_to_float of the text).
double bolt_diameter(const Param& d_imp) {
    if (d_imp.is_number()) throw std::invalid_argument("object of type 'int' has no len()");
    return imperial_str_to_float(d_imp.text()).number();
}

int bolt_count(const Param& n) {
    const double value = n.number();
    if (std::trunc(value) != value) throw std::invalid_argument("'float' object cannot be interpreted as an integer");
    return static_cast<int>(value);
}

const Param& item(const std::vector<Param>& row, std::size_t index) {
    if (index >= row.size()) throw std::out_of_range("list index out of range");
    return row[index];
}

/// The vertices of a group of a sketch's vertices (as Shapes, for the builder operations).
template <class List>
std::vector<Shape> shapes(const List& list) {
    return std::vector<Shape>(list.begin(), list.end());
}

}  // namespace

// ---------------------------------------------------------------------------
// Flange
// ---------------------------------------------------------------------------

Flange::Flange(const Shape& flange_section, double bcd, int bolt_hole_count, double bolt_hole_diameter,
               const Rotation& rotation, std::optional<Align3> align) {
    init_flange(flange_section, bcd, bolt_hole_count, bolt_hole_diameter, rotation, align);
}

void Flange::init_flange(const Shape& flange_section, double bcd, int bolt_hole_count, double bolt_hole_diameter,
                         const Rotation& rotation, std::optional<Align3> align) {
    Builder flange_builder(3);
    {
        Builder sketch(2);
        detail::insert(sketch, {flange_section});
        detail::split(sketch, std::nullopt, Plane::YZ);
        flange_builder.publish(sketch.product(), Mode::ADD, {Location()}, {Location(Plane::XZ)});
    }
    detail::revolve(flange_builder);
    detail::hole(flange_builder, bolt_hole_diameter / 2, std::nullopt, PolarLocations(bcd / 2, bolt_hole_count));
    assign(flange_builder.part(), rotation, align);
}

bool Flange::inputs_are_valid(const std::string& nps, int flange_class, const std::optional<std::string>& face_type) {
    const auto& valid_nps = flange_nps();
    if (std::find(valid_nps.begin(), valid_nps.end(), nps) == valid_nps.end()) {
        throw std::invalid_argument("Invalid nps value - the valid values are: " + detail::py_tuple_repr(valid_nps));
    }
    const auto& classes = flange_classes();
    if (std::find(classes.begin(), classes.end(), flange_class) == classes.end()) {
        std::string text = "(";
        for (std::size_t i = 0; i < classes.size(); ++i) text += (i ? ", " : "") + std::to_string(classes[i]);
        throw std::invalid_argument("Invalid flange_class value - the valid values are: " + text + ")");
    }
    const auto& faces = flange_face_types();
    if (face_type && std::find(faces.begin(), faces.end(), *face_type) == faces.end()) {
        throw std::invalid_argument("Invalid face_type value - the valid values are: " + detail::py_tuple_repr(faces));
    }
    return true;
}

bool Flange::dimension_is_numeric(const Param& value) { return value.is_number(); }

void Flange::require_numeric_dimensions(const std::vector<Param>& flange_data, const Dict<std::string, int>& columns,
                                        const std::string& flange_name) {
    std::vector<std::string> unavailable;
    for (const auto& [column_name, column_index] : columns) {
        if (column_index < 0 || static_cast<std::size_t>(column_index) >= flange_data.size()) {
            unavailable.push_back(column_name);
            continue;
        }
        const Param& value = flange_data[static_cast<std::size_t>(column_index)];
        if (!dimension_is_numeric(value)) unavailable.push_back(column_name + "=" + detail::py_repr(value.text()));
    }
    if (!unavailable.empty()) {
        std::string text;
        for (std::size_t i = 0; i < unavailable.size(); ++i) text += (i ? ", " : "") + unavailable[i];
        throw std::invalid_argument(flange_name + " requires unavailable ASME table dimensions: " + text);
    }
}

std::pair<std::vector<Param>, std::vector<Param>> Flange::get_flange_data(const std::string& nps, int flange_class,
                                                                          const std::optional<std::string>&) {
    // face_type is accepted for API symmetry; the face data comes from get_face_section_data
    if (flange_class == 150) return {flange_data_class_150().at(nps), drilling_class_150().at(nps)};
    if (flange_class == 300) return {flange_data_class_300().at(nps), drilling_class_300().at(nps)};
    throw std::invalid_argument("Unsupported flange class");
}

std::pair<std::optional<Sketch>, double> Flange::get_face_section_data(const std::string& nps, int flange_class,
                                                                       const std::optional<std::string>& face_type,
                                                                       std::optional<double> face_thickness) {
    const FlangeTable* ring_table = nullptr;
    if (flange_class == 150) {
        ring_table = &ring_joint_facings_class_150();
    } else if (flange_class == 300) {
        ring_table = &ring_joint_facings_class_300();
    } else {
        throw std::invalid_argument("Unsupported flange class");
    }
    const std::vector<Param>& ring_data = ring_table->at(nps);
    if (ring_data.size() < 6) {
        throw std::invalid_argument("not enough values to unpack (expected 5, got " +
                                    std::to_string(ring_data.size() > 1 ? ring_data.size() - 1 : 0) + ")");
    }
    const Param &P = ring_data[1], &F = ring_data[3], &R = ring_data[4], &K = ring_data[5];
    std::optional<Param> E = ring_data[2];

    if (face_thickness) {
        E = Param(*face_thickness);
    } else if (!face_type) {
        E = std::nullopt;
    } else if (*face_type == "Flat") {
        E = Param(0);
    } else if (*face_type == "Raised") {
        E = Param(flange_class <= 300 ? 2 * MM : 7 * MM);
    } else if (*face_type == "Ring") {
        E = ring_data[2];
    } else {
        throw std::invalid_argument("face_type " + *face_type + " not supported");
    }

    std::optional<Sketch> face_section;
    if (face_type && *face_type != "Flat") {
        const auto e = [&] {
            if (!E) throw std::invalid_argument("unsupported operand type(s) for -: 'float' and 'NoneType'");
            return num(*E);
        };
        Builder face_builder(2);
        if (*face_type == "Raised" || *face_type == "Ring" || *face_type == "Groove") {
            face_builder.add(Rectangle(num(K), e(), Align2(Align::CENTER, Align::MIN)));
        }
        if (*face_type == "Ring") {
            Builder groove(2);
            groove.add(detail::place(Trapezoid(num(F), e(), 90 - 23), {Location(Vector(num(P) / 2, 0))}));
            detail::fillet(groove, shapes(groove.vertices().group_by(Axis::Y)[-1]), num(R));
            face_builder.publish(groove.product(), Mode::SUBTRACT, {Location()}, {Location(Plane::XZ)});
        }
        if (face_builder.obj) face_section = placed_sketch(face_builder, Plane::XY);
    }
    const double height = face_section ? face_section->bounding_box().max.Y : 0.0;
    return {face_section, height};
}

// ---------------------------------------------------------------------------
// BlindFlange
// ---------------------------------------------------------------------------

BlindFlange::BlindFlange(const std::string& nps_, int flange_class_, const std::string& face_type_,
                         const Rotation& rotation, std::optional<Align3> align) {
    nps = nps_;
    flange_class = flange_class_;
    face_type = face_type_;

    // Validate inputs
    inputs_are_valid(nps, flange_class, face_type);

    // Get the flange parameters
    const auto [flange_data, bolt_data] = get_flange_data(nps, flange_class, face_type);
    const Param &O = item(flange_data, 0), &tf = item(flange_data, 1), &B = item(flange_data, 9);
    const Param &W = item(bolt_data, 1), &d_imp = item(bolt_data, 2), &n = item(bolt_data, 3);
    const double d = bolt_diameter(d_imp);

    // Get the face profile if any
    const auto [face_profile, face_thickness] = Flange::get_face_section_data(nps, flange_class, face_type);

    od = num(O);
    thickness = num(tf) + face_thickness;

    // Create the profile
    Builder flange_profile(2);
    flange_profile.add(
        detail::place(Rectangle(num(O), num(tf), Align2(Align::CENTER, Align::MIN)), {Location(Vector(0, face_thickness))}));
    detail::fillet(flange_profile, shapes(flange_profile.vertices().group_by(Axis::Y)[-1]), num(tf) / 4);
    if (face_profile) detail::insert(flange_profile, {*face_profile});
    flange_profile.add(Rectangle(num(B), face_thickness, Align2(Align::CENTER, Align::MIN)), Mode::SUBTRACT);

    init_flange(placed_sketch(flange_profile, Plane::XZ), num(W), bolt_count(n), d, rotation, align);

    // Add the joints
    RigidJoint("face", *this, Location(Plane::YX));
}

// ---------------------------------------------------------------------------
// LappedFlange, LappedFlangeStub
// ---------------------------------------------------------------------------

LappedFlange::LappedFlange(const std::string& nps_, int flange_class_, const Rotation& rotation,
                           std::optional<Align3> align) {
    nps = nps_;
    flange_class = flange_class_;

    // Validate inputs
    inputs_are_valid(nps, flange_class, std::nullopt);

    // Get the flange parameters
    const auto [flange_data, bolt_data] = get_flange_data(nps, flange_class, std::nullopt);
    const Param &O = item(flange_data, 0), &tf = item(flange_data, 1), &X = item(flange_data, 3),
                &Y = item(flange_data, 6), &B = item(flange_data, 10), &r = item(flange_data, 12);
    const Param &W = item(bolt_data, 1), &d_imp = item(bolt_data, 2), &n = item(bolt_data, 3);
    const double d = bolt_diameter(d_imp);

    // Get the face profile if any
    const double face_thickness = Flange::get_face_section_data(nps, flange_class, std::nullopt).second;

    od = num(O);
    id = num(B);
    thickness = num(Y) + face_thickness;

    // Create the profile
    Builder flange_profile(2);
    const std::vector<Location> at_face{Location(Vector(0, face_thickness))};
    flange_profile.add(detail::place(Rectangle(num(X), num(Y), Align2(Align::CENTER, Align::MIN)), at_face));
    flange_profile.add(detail::place(Rectangle(num(O), num(tf), Align2(Align::CENTER, Align::MIN)), at_face));
    std::vector<Shape> vertices = shapes(flange_profile.vertices().group_by(Axis::Y)[-1]);
    const auto second = flange_profile.vertices().group_by(Axis::Y)[-2];
    vertices.insert(vertices.end(), second.begin(), second.end());
    detail::fillet(flange_profile, vertices, (num(Y) - num(tf)) / 4);
    flange_profile.add(Rectangle(num(B), num(Y) + face_thickness, Align2(Align::CENTER, Align::MIN)), Mode::SUBTRACT);
    detail::fillet(flange_profile, shapes(flange_profile.vertices().group_by(Axis::Y)[0].sort_by(Axis::X).slice(1, 3)),
                   num(r));

    init_flange(placed_sketch(flange_profile, Plane::XZ), num(W), bolt_count(n), d, rotation, align);

    // Add the joint
    RigidJoint("lap", *this, Location(Plane::XY));
}

LappedFlangeStub::LappedFlangeStub(const std::string& nps_, const std::string& material,
                                   const std::string& pipe_identifier, double stub_length, const std::string& face_type,
                                   const Rotation& rotation, std::optional<Align3> align) {
    nps = nps_;

    const Pipe stub_pipe(nps, material, pipe_identifier, Edge::make_line(Vector(0, 0, 0), Vector(0, 0, stub_length)));
    // Re B16.5 6.4.3.2
    const double t = face_type == "Male" ? 7 * MM : stub_pipe.thickness;

    // Get the face profile - note that flange class of 150 isn't used
    const auto [face_profile, face_thickness] = Flange::get_face_section_data(nps, 150, face_type, t);

    Builder stub(3);
    {
        Builder sketch(2);
        if (face_profile) detail::insert(sketch, {*face_profile});
        sketch.add(Rectangle(stub_pipe.id, face_thickness, Align2(Align::CENTER, Align::MIN)), Mode::SUBTRACT);
        detail::split(sketch, std::nullopt, Plane::YZ);
        stub.publish(sketch.product(), Mode::ADD, {Location()}, {Location(Plane::XZ)});
    }
    detail::revolve(stub);
    detail::insert(stub, {stub_pipe});
    stub_edges = stub.edges(Select::NEW);
    detail::fillet(stub, shapes(stub.edges(Select::NEW)), weld_radius);
    detail::chamfer(stub, {stub.edges().group_by(Axis::Z)[-1].sort_by(SortBy::RADIUS)[-1]}, stub_pipe.thickness * 0.75);

    assign(stub.part(), rotation, align);

    // Add the joints
    RigidJoint("pipe", *this, Location(Plane::YX.offset(-stub_length)));
    RigidJoint("face", *this, Location(Plane::XY));
    RigidJoint("lap", *this, Location(Plane::XY.offset(face_thickness)));
}

// ---------------------------------------------------------------------------
// SlipOnFlange
// ---------------------------------------------------------------------------

SlipOnFlange::SlipOnFlange(const std::string& nps_, int flange_class_, const std::string& face_type_,
                           const Rotation& rotation, std::optional<Align3> align) {
    nps = nps_;
    flange_class = flange_class_;
    face_type = face_type_;

    // Validate inputs
    inputs_are_valid(nps, flange_class, face_type);

    // Get the flange parameters
    const auto [flange_data, bolt_data] = get_flange_data(nps, flange_class, face_type);
    const Param &O = item(flange_data, 0), &tf = item(flange_data, 1), &X = item(flange_data, 3),
                &Y = item(flange_data, 5), &B = item(flange_data, 9);
    const Param &W = item(bolt_data, 1), &d_imp = item(bolt_data, 2), &n = item(bolt_data, 3);
    const double d = bolt_diameter(d_imp);

    // Get the face profile if any
    const auto [face_profile, face_thickness] = Flange::get_face_section_data(nps, flange_class, face_type);

    od = num(O);
    id = num(B);
    thickness = num(Y) + face_thickness;

    // Create the profile
    Builder flange_profile(2);
    const std::vector<Location> at_face{Location(Vector(0, face_thickness))};
    flange_profile.add(detail::place(Rectangle(num(X), num(Y), Align2(Align::CENTER, Align::MIN)), at_face));
    flange_profile.add(detail::place(Rectangle(num(O), num(tf), Align2(Align::CENTER, Align::MIN)), at_face));
    std::vector<Shape> vertices = shapes(flange_profile.vertices().group_by(Axis::Y)[-1]);
    const auto second = flange_profile.vertices().group_by(Axis::Y)[-2];
    vertices.insert(vertices.end(), second.begin(), second.end());
    detail::fillet(flange_profile, vertices, (num(Y) - num(tf)) / 4);
    if (face_profile) detail::insert(flange_profile, {*face_profile});
    flange_profile.add(Rectangle(num(B), num(Y) + face_thickness, Align2(Align::CENTER, Align::MIN)), Mode::SUBTRACT);

    init_flange(placed_sketch(flange_profile, Plane::XZ), num(W), bolt_count(n), d, rotation, align);

    // Add the joints
    RigidJoint("pipe", *this, Location(Plane::YX.offset(-(1.0 / 16) * IN)));
    RigidJoint("face", *this, Location(Plane::XY));
}

// ---------------------------------------------------------------------------
// SocketWeldFlange
// ---------------------------------------------------------------------------

std::pair<std::optional<Sketch>, double> SocketWeldFlange::get_face_section_data(
    const std::string& nps, int flange_class, const std::optional<std::string>& face_type,
    std::optional<double> face_thickness) {
    const auto flange_data = Flange::get_flange_data(nps, flange_class, face_type).first;
    const int socket_depth_index = flange_class == 150 ? 13 : 14;
    Flange::require_numeric_dimensions(flange_data, {{"socket bore", 11}, {"socket depth", socket_depth_index}},
                                       "SocketWeldFlange");
    return Flange::get_face_section_data(nps, flange_class, face_type, face_thickness);
}

SocketWeldFlange::SocketWeldFlange(const std::string& nps_, int flange_class_, const std::string& face_type_,
                                   const Rotation& rotation, std::optional<Align3> align) {
    nps = nps_;
    flange_class = flange_class_;
    face_type = face_type_;

    // Validate inputs
    inputs_are_valid(nps, flange_class, face_type);

    // Get the flange parameters
    const auto [flange_data, bolt_data] = get_flange_data(nps, flange_class, face_type);
    const int socket_depth_index = flange_class == 150 ? 13 : 14;
    require_numeric_dimensions(flange_data,
                               {{"outside diameter", 0},
                                {"flange thickness", 1},
                                {"hub diameter", 3},
                                {"length through hub", 5},
                                {"bore", 9},
                                {"socket bore", 11},
                                {"corner radius", 12},
                                {"socket depth", socket_depth_index}},
                               "SocketWeldFlange");
    const Param &O = item(flange_data, 0), &tf = item(flange_data, 1), &X = item(flange_data, 3),
                &Y = item(flange_data, 5), &B11 = item(flange_data, 9), &B13 = item(flange_data, 11);
    const Param& D = item(flange_data, static_cast<std::size_t>(socket_depth_index));
    const Param &W = item(bolt_data, 1), &d_imp = item(bolt_data, 2), &n = item(bolt_data, 3);
    const double d = bolt_diameter(d_imp);

    // Get the face profile if any
    const auto [face_profile, face_thickness] = Flange::get_face_section_data(nps, flange_class, face_type);

    od = num(O);
    id = num(B13);
    thickness = num(Y) + face_thickness;

    // Create the profile
    Builder flange_profile(2);
    const std::vector<Location> at_face{Location(Vector(0, face_thickness))};
    flange_profile.add(detail::place(Rectangle(num(X), num(Y), Align2(Align::CENTER, Align::MIN)), at_face));
    flange_profile.add(detail::place(Rectangle(num(O), num(tf), Align2(Align::CENTER, Align::MIN)), at_face));
    std::vector<Shape> vertices = shapes(flange_profile.vertices().group_by(Axis::Y)[-1]);
    const auto second = flange_profile.vertices().group_by(Axis::Y)[-2];
    vertices.insert(vertices.end(), second.begin(), second.end());
    detail::fillet(flange_profile, vertices, (num(Y) - num(tf)) / 4);
    if (face_profile) detail::insert(flange_profile, {*face_profile});
    flange_profile.add(Rectangle(num(B13), num(Y) + face_thickness, Align2(Align::CENTER, Align::MIN)),
                       Mode::SUBTRACT);
    flange_profile.add(detail::place(Rectangle(num(B11), num(D), Align2(Align::CENTER, Align::MAX)),
                                     {Location(Vector(0, num(Y) + face_thickness))}),
                       Mode::SUBTRACT);

    init_flange(placed_sketch(flange_profile, Plane::XZ), num(W), bolt_count(n), d, rotation, align);

    // Add the joints
    RigidJoint("pipe", *this, Location(Plane::YX.offset(-(num(Y) + face_thickness - num(D)))));
    RigidJoint("face", *this, Location(Plane::XY));
}

// ---------------------------------------------------------------------------
// WeldNeckFlange
// ---------------------------------------------------------------------------

std::pair<std::optional<Sketch>, double> WeldNeckFlange::get_face_section_data(
    const std::string& nps, int flange_class, const std::optional<std::string>& face_type,
    std::optional<double> face_thickness) {
    const auto flange_data = Flange::get_flange_data(nps, flange_class, face_type).first;
    Flange::require_numeric_dimensions(flange_data, {{"welding neck bore", 11}}, "WeldNeckFlange");
    return Flange::get_face_section_data(nps, flange_class, face_type, face_thickness);
}

WeldNeckFlange::WeldNeckFlange(const std::string& nps_, int flange_class_, const std::string& face_type_,
                               const Rotation& rotation, std::optional<Align3> align) {
    nps = nps_;
    flange_class = flange_class_;
    face_type = face_type_;

    // Validate inputs
    inputs_are_valid(nps, flange_class, face_type);

    // Get the flange parameters
    const auto [flange_data, bolt_data] = get_flange_data(nps, flange_class, face_type);
    require_numeric_dimensions(flange_data,
                               {{"outside diameter", 0},
                                {"flange thickness", 1},
                                {"hub diameter", 3},
                                {"hub end diameter", 4},
                                {"length through hub", 7},
                                {"welding neck bore", 11},
                                {"corner radius", 12}},
                               "WeldNeckFlange");
    const Param &O = item(flange_data, 0), &tf = item(flange_data, 1), &X = item(flange_data, 3),
                &Ah = item(flange_data, 4), &Y = item(flange_data, 7), &B = item(flange_data, 11);
    const Param &W = item(bolt_data, 1), &d_imp = item(bolt_data, 2), &n = item(bolt_data, 3);
    const double d = bolt_diameter(d_imp);

    // Get the face profile if any
    const auto [face_profile, face_thickness] = Flange::get_face_section_data(nps, flange_class, face_type);

    od = num(O);
    id = num(B);
    thickness = num(Y) + face_thickness;

    const double trap_angle = detail::degrees(std::atan2(num(Y) - num(tf), (num(X) - num(Ah)) / 2));
    // Create the basic shape
    Builder flange_profile(2);
    flange_profile.add(detail::place(Rectangle(num(O), num(tf), Align2(Align::CENTER, Align::MIN)),
                                     {Location(Vector(0, face_thickness))}));
    flange_profile.add(detail::place(Trapezoid(num(X), num(Y) - num(tf), trap_angle, Align2(Align::CENTER, Align::MIN)),
                                     {Location(Vector(0, face_thickness + num(tf)))}));
    detail::fillet(flange_profile, shapes(flange_profile.vertices().group_by(Axis::Y)[-2]), num(tf) / 6);
    const double c = (num(Ah) - num(B)) / 2.5;
    detail::chamfer(flange_profile, shapes(flange_profile.vertices().group_by(Axis::Y)[-1]), c);
    if (face_profile) detail::insert(flange_profile, {*face_profile});
    flange_profile.add(Rectangle(num(B), num(Y) + face_thickness, Align2(Align::CENTER, Align::MIN)), Mode::SUBTRACT);

    init_flange(placed_sketch(flange_profile, Plane::XZ), num(W), bolt_count(n), d, rotation, align);

    // Add the joints
    RigidJoint("pipe", *this, Location(Plane::YX.offset(-num(Y) - face_thickness)));
    RigidJoint("face", *this, Location(Plane::XY));
}

}  // namespace bd_warehouse
