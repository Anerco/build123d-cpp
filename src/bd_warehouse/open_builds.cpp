// Port of bd_warehouse's open_builds.py.
#include "bd_warehouse/open_builds.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

#include "bd_warehouse/bearing.hpp"
#include "bd_warehouse/thread.hpp"
#include "builder.hpp"
#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;

const SocketHeadCapScrew& open_builds::m5() {
    static const SocketHeadCapScrew screw("M5-0.8", 20 * MM);
    return screw;
}

namespace {

using Locs = std::vector<Location>;
using open_builds::CAVITY_RADIUS;
using open_builds::FILLET_RADIUS;

Location at(double x, double y, double z = 0.0) { return Location(Vector(x, y, z)); }

/// GridLocations(x_spacing, y_spacing, x_count, y_count) as a list.
Locs grid(double x_spacing, double y_spacing, int x_count, int y_count) {
    const GridLocations locations(x_spacing, y_spacing, x_count, y_count);
    return Locs(locations.begin(), locations.end());
}

template <class List>
std::vector<Shape> shapes(const List& list) {
    return std::vector<Shape>(list.begin(), list.end());
}

/// The end of `with BuildSketch(workplane): ...` in a BuildPart: the sketch published on the
/// workplane (its faces become pending faces).
void publish_sketch(Builder& part, const Builder& sketch, const Plane& workplane = Plane::XY) {
    part.publish(sketch.product(), Mode::ADD, {Location()}, {Location(workplane)});
}

/// metals.aluminum(finish=[finishes.brushed(), finishes.anodize("black")])
FinishedMaterial black_anodized_aluminum() {
    MetalOptions options;
    options.finish = FinishSpec({finishes::brushed(), finishes::anodize(ColorSpec("black"))});
    return metals::aluminum(options);
}

/// The value of `key` in a dict of valid values, or Python's ValueError naming the keys.
template <class T>
T valid_value(const Dict<std::string, T>& values, const std::string& key, const std::string& message) {
    const std::optional<T> found = values.get(key);
    if (!found) throw std::invalid_argument(key + message + detail::py_tuple_repr(values.keys()));
    return *found;
}

/// A label for each location, as Python's zip(labels, locations).
template <class F>
void for_each_labeled(const std::vector<std::string>& labels, const Locs& locations, F make_joint) {
    for (std::size_t i = 0; i < std::min(labels.size(), locations.size()); ++i) make_joint(labels[i], locations[i]);
}

/// _VSlotGroove: Sketch Object: OpenBuilds V Slot Linear Rail Groove (rotated by `rotation`).
Sketch vslot_groove(double rotation = 0) {
    Builder groove(2);
    {
        Builder line(1);
        line.add(Polyline({Vector(3.69, 0), Vector(3.9, 0.21), Vector(3.9, 2.8393398282202034),
                           Vector(6.560660171779773, 5.5), Vector(8.2, 5.5), Vector(8.2, 3.125), Vector(8.545, 3.125),
                           Vector(10, 4.58), Vector(10, 0)}));
        detail::mirror(line, std::nullopt, Plane::XZ);
        groove.publish(line.product());
    }
    detail::make_face(groove);
    return BaseSketchObject(groove.sketch(), rotation, std::nullopt);
}

/// _VSlotInternalCavity: Sketch Object: OpenBuilds V Slot Linear Rail Internal Cavity.
Sketch vslot_internal_cavity(double rotation = 0) {
    Builder cavity(2);
    {
        Builder line(1);
        line.add(Polyline({Vector(-8.2, 0), Vector(-8.2, -2.7), Vector(-6.23933983, -2.7), Vector(-2.83933983, -6.1),
                           Vector(0, -6.1)}));
        detail::mirror(line, std::nullopt, Plane::XZ);
        detail::mirror(line, std::nullopt, Plane::YZ);
        cavity.publish(line.product());
    }
    detail::make_face(cavity);
    return BaseSketchObject(cavity.sketch(), rotation, std::nullopt);
}

/// The grooves of each V-Slot rail size: their positions and angles.
struct GrooveSet {
    std::vector<std::pair<double, double>> positions;
    std::vector<double> angles;
};

const Dict<std::string, GrooveSet>& vslot_grooves() {
    static const Dict<std::string, GrooveSet> grooves = {
        {"20x20", GrooveSet{{{0, 0}, {0, 0}, {0, 0}, {0, 0}}, {0, 90, 180, 270}}},
        {"20x40", GrooveSet{{{0, 10}, {0, 10}, {0, 10}, {0, -10}, {0, -10}, {0, -10}}, {0, 90, 180, 180, 270, 0}}},
        {"20x60",
         GrooveSet{{{0, 0}, {0, 20}, {0, 20}, {0, 20}, {0, 0}, {0, -20}, {0, -20}, {0, -20}},
                   {0, 0, 90, 180, 180, 180, 270, 0}}},
        {"20x80",
         GrooveSet{{{0, 10}, {0, 30}, {0, 30}, {0, 30}, {0, 10}, {0, -10}, {0, -30}, {0, -30}, {0, -30}, {0, -10}},
                   {0, 0, 90, 180, 180, 180, 180, 270, 0, 0}}},
        {"40x40",
         GrooveSet{{{10, 10}, {10, 10}, {-10, 10}, {-10, 10}, {-10, -10}, {-10, -10}, {10, -10}, {10, -10}},
                   {0, 90, 90, 180, 180, 270, 270, 0}}},
    };
    return grooves;
}

void check_rail_size(const std::string& rail_size) {
    if (!vslot_grooves().contains(rail_size)) {
        throw std::invalid_argument("The rail_size of " + rail_size +
                                    " isn't valid - must be one of 20x20, 20x40, 20x60, 20x80, or 40x40");
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// AcmeAntiBacklashNutBlock8mm, AcmeAntiBacklashNutBlock8mmAssembly
// ---------------------------------------------------------------------------

AcmeAntiBacklashNutBlock8mm::AcmeAntiBacklashNutBlock8mm(const Rotation& rotation, std::optional<Align3> align) {
    const Locs nut_locations{at(10, -6.5), at(-10, -6.5)};
    Builder block(3);
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(34, 33, 3));
        const Locs slot{at(-9, 6)};
        sketch.add(detail::place(Circle(2.5), slot), Mode::SUBTRACT);
        sketch.add(detail::place(Rectangle(26, 5, Align2(Align::MIN, Align::CENTER)), slot), Mode::SUBTRACT);
        sketch.add(detail::place(Circle(2.55), nut_locations), Mode::SUBTRACT);
        publish_sketch(block, sketch, Plane::XY.offset(-6));
    }
    detail::extrude(block, 12);
    const Locs mounts = nut_locations;
    {
        Builder sketch(2);
        sketch.add(detail::place(Circle(4.5), mounts));
        publish_sketch(block, sketch, Plane::XY.offset(6));
    }
    detail::extrude(block, -2, {}, Mode::SUBTRACT);
    {
        Builder sketch(2);
        sketch.add(detail::place(RegularPolygon(4.618802153517, 6, true, 30), nut_locations));
        publish_sketch(block, sketch, Plane::XY.offset(-6));
    }
    detail::extrude(block, 5, {}, Mode::SUBTRACT);
    detail::hole(block, 4, std::nullopt, {Location(Plane::XZ)});
    const Locs screw_hole = detail::compose({Location(-Plane::XZ.offset(-17))}, {at(10, 0)});
    detail::threaded_hole(block, open_builds::m5(), "Soft", "Normal", 10, false, true, screw_hole);

    assign(block.part(), rotation, align);
    material = black_anodized_aluminum();
    label = "AcmeAntiBacklashNutBlock8mm";
    const ShapeList<Face> mount_faces = faces().group_by(Axis::Z)[-3].sort_by(Axis::X, true);
    for (std::size_t i = 0; i < std::min<std::size_t>(2, mount_faces.size()); ++i) {
        RigidJoint(i == 0 ? "a" : "b", *this, Pos(mount_faces[static_cast<std::ptrdiff_t>(i)].center()));
    }
    for_each_labeled({"nut_a", "nut_b"}, mounts, [&](const std::string& name, const Location& loc) {
        RigidJoint(name, *this, loc * Location(Vector(0, 0, -6), Vector(0, 0, 1), 30));
    });
    RigidJoint("screw", *this, screw_hole.at(0));
    RigidJoint("bottom", *this, Pos(0, 0, -6));
}

AcmeAntiBacklashNutBlock8mmAssembly::AcmeAntiBacklashNutBlock8mmAssembly() {
    SocketHeadCapScrew screw("M5-0.8", 13 * MM);
    HexNut nut0("M5-0.8");
    HexNut nut1 = nut0;
    Part acme_nut = Rot(0, 0, -90) * AcmeAntiBacklashNutBlock8mm();
    acme_nut.joints["nut_a"].connect_to(nut0.joints["a"]);
    acme_nut.joints["nut_b"].connect_to(nut1.joints["a"]);
    acme_nut.joints["screw"].connect_to(screw.joints["a"]);
    AluminumSpacer s0("6mm");
    AluminumSpacer s1 = s0;
    acme_nut.joints["a"].connect_to(s0.joints["a"]);
    acme_nut.joints["b"].connect_to(s1.joints["a"]);

    children = std::vector<Shape>{acme_nut, s0, s1, nut0, nut1, screw};
    label = "AcmeAntiBacklashNutBlock8mmAssembly";
    RigidJoint("a", *this, s0.joints["b"].location() * Rot(0, 0, 90));
    RigidJoint("b", *this, s1.joints["b"].location() * Rot(0, 0, 90));
}

// ---------------------------------------------------------------------------
// AluminumSpacer
// ---------------------------------------------------------------------------

AluminumSpacer::AluminumSpacer(const std::string& length, const Rotation& rotation, std::optional<Align3> align) {
    static const Dict<std::string, double> valid_lengths = {
        {"3mm", 3 * MM},     {"1/8in", IN / 8},   {"6mm", 6 * MM},   {"1/4in", IN / 4},
        {"9mm", 9 * MM},     {"10mm", 10 * MM},   {"13.2mm", 13.2 * MM}, {"20mm", 20 * MM},
        {"35mm", 35 * MM},   {"1-1/2in", 1.5 * IN}, {"40mm", 40 * MM},
    };
    const double spacer_length = valid_value(valid_lengths, length, " is an invalid length, must be one of ");

    Builder spacer(3);
    {
        Builder sketch(2);
        sketch.add(Circle(5));
        sketch.add(Circle(2.6), Mode::SUBTRACT);
        spacer.publish(sketch.product());
    }
    detail::extrude(spacer, spacer_length);

    assign(spacer.part(), rotation, align);
    material = metals::aluminum();
    label = "AluminumSpacer-" + length;
    // "a" and "b" are the bottom and top mounting faces, respectively.
    RigidJoint("a", *this, Location());
    RigidJoint("b", *this, Pos(0, 0, spacer_length));
    // "center" is the top-face mounting/rotation axis used by wheel assemblies; it is not the
    // geometric midpoint of the spacer.
    RigidJoint("center", *this, Pos(0, 0, spacer_length));
}

// ---------------------------------------------------------------------------
// The C-Beam assemblies
// ---------------------------------------------------------------------------

CBeamAssembly::CBeamAssembly(double rail_length, std::optional<double> gantry_position,
                             std::optional<double> screw_length, bool end_plate, bool simple)
    : CBeamAssembly(rail_length, CBeamAssemblyOptions{gantry_position, screw_length, end_plate, simple}) {}

CBeamAssembly::CBeamAssembly(double rail_length, const CBeamAssemblyOptions& o) {
    Part rail = Rot(-90, 0, 0) * CBeamLinearRail(rail_length);
    CBeamEndAssembly end_assembly;
    std::vector<Shape> components{rail, end_assembly};
    std::optional<double> screw_length = o.screw_length;
    if (o.end_plate) {
        Part end_plate = Pos(0, rail_length) * (Rot(-90, 0, -90) * CBeamEndMount(Rotation(), Align3(Align::NONE)));
        SingleRowCappedDeepGrooveBallBearing bearing("M8-16-5");
        end_plate.joints["bearing"].connect_to(bearing.joints["b"]);
        const LowProfileScrew lm5("M5-0.8", 25 * MM);
        std::vector<LowProfileScrew> lm5s(4, lm5);
        for (int i = 0; i < 4; ++i) end_plate.joints["screw-" + std::to_string(i)].connect_to(lm5s[i].joints["a"]);
        if (!screw_length) {
            screw_length = end_plate.joints["bearing"].location().position().Y -
                           end_assembly.joints["screw"].location().position().Y;
        }
        // (end_assembly hasn't moved: the copy in components is the same)
        components.push_back(end_plate);
        components.push_back(bearing);
        for (const auto& s : lm5s) components.push_back(s);
    } else if (!screw_length) {
        screw_length = rail_length;
    }

    MetricLeadScrew screw(*screw_length, o.simple);
    end_assembly.joints["screw"].connect_to(screw.joints["axis"]);
    components.push_back(screw);

    std::optional<XLargeCBeamGantry> gantry;
    if (o.gantry_position) {
        gantry.emplace(6);
        rail.joints["screw_axis"].connect_to(gantry->joints["nut"], {.position = o.gantry_position});
        components.push_back(*gantry);
    }

    children = components;
    if (gantry) {
        const Vector gantry_top_center = gantry->faces().sort_by(Axis::X)[-1].without_holes().center();
        RigidJoint("gantry_center", *this, Location(Plane(gantry_top_center, Vector(0, 1, 0), Vector(1, 0, 0))));
    }
    const BoundBox rail_bbox = rail.bounding_box();
    RigidJoint("rail_back", *this,
               Location(Plane(Vector(rail_bbox.min.X, rail_length / 2, 0), Vector(0, 1, 0), Vector(1, 0, 0))));
    RigidJoint("rail_front", *this,
               Location(Plane(Vector(rail_bbox.max.X, rail_length / 2, 0), Vector(0, 1, 0), Vector(-1, 0, 0))));
    RigidJoint("rail_cross_front", *this,
               Location(Plane(Vector(rail_bbox.max.X, rail_length / 2, 0), Vector(0, 0, -1), Vector(1, 0, 0))));
    LinearJoint("screw_axis", *this, Axis(rail.joints["screw_axis"].location().position(), Vector(0, 1, 0)),
                {0, rail_length});
    label = "CBeam Assembly";
}

CBeamCapped::CBeamCapped(double length) {
    const LowProfileScrew lm5("M5-0.8", 25 * MM);
    std::vector<LowProfileScrew> lm5s(8, lm5);
    Part rail = Rot(-90, 0, 0) * CBeamLinearRail(length);
    const SingleRowCappedDeepGrooveBallBearing b("M8-16-5");
    std::vector<SingleRowCappedDeepGrooveBallBearing> bearings(2, b);
    std::vector<Part> end_plates;
    end_plates.reserve(2);
    for (const auto& [a, l] : std::vector<std::pair<double, double>>{{-90, length}, {90, 0}}) {
        end_plates.push_back(Pos(0, l) * (Rot(a, 0, -90) * CBeamEndMount(Rotation(), Align3(Align::NONE))));
    }
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            end_plates[i].joints["screw-" + std::to_string(j)].connect_to(lm5s[i * 4 + j].joints["a"]);
        }
        end_plates[i].joints["bearing"].connect_to(bearings[i].joints["b"]);
    }

    std::vector<Shape> components{rail};
    for (const auto& p : end_plates) components.push_back(p);
    for (const auto& p : bearings) components.push_back(p);
    for (const auto& s : lm5s) components.push_back(s);
    children = components;
    label = "CBeamCapped";
    RigidJoint("bottom_center", *this, Location(Plane(Vector(-20, length / 2, 0), Vector(0, 1, 0), Vector(1, 0, 0))));
    RigidJoint("top_center", *this, Location(Plane(Vector(20, length / 2, 0), Vector(0, 0, 1), Vector(1, 0, 0))));
}

CBeamEndAssembly::CBeamEndAssembly() {
    const LowProfileScrew s("M5-0.8", 25 * MM);
    std::vector<LowProfileScrew> lm5s(4, s);
    Part end_plate = Rot(90, 0, -90) * CBeamEndMount(Rotation(), Align3(Align::NONE));
    for (int i = 0; i < 4; ++i) end_plate.joints["screw-" + std::to_string(i)].connect_to(lm5s[i].joints["a"]);
    SingleRowCappedDeepGrooveBallBearing bearing("M8-16-5");
    end_plate.joints["bearing"].connect_to(bearing.joints["b"]);
    const AluminumSpacer stand_off("40mm");
    std::vector<AluminumSpacer> stand_offs(2, stand_off);
    for (int i = 0; i < 2; ++i) end_plate.joints["motor_mount-" + std::to_string(i)].connect_to(stand_offs[i].joints["a"]);

    StepperMotor motor("Nema23");
    stand_offs[0].joints["b"].connect_to(motor.joints["c"]);
    stand_offs[1].joints["b"].connect_to(motor.joints["d"]);
    FlexibleCoupler coupler("8mm");
    motor.joints["shaft"].connect_to(coupler.joints["a"]);

    std::vector<Shape> components{end_plate, motor, coupler, bearing};
    for (const auto& so : stand_offs) components.push_back(so);
    for (const auto& screw : lm5s) components.push_back(screw);
    children = components;
    label = "CBeam End Assembly";
    RigidJoint("screw", *this, -coupler.joints["b"].location());
}

CBeamEndMount::CBeamEndMount(const Rotation& rotation, std::optional<Align3> align) {
    const SocketHeadCapScrew& m5 = open_builds::m5();
    Builder plate(3);
    plate.add(Box(80, 50, 12));
    detail::counter_bore_hole(plate, 5.588, 8.05, 4.5, std::nullopt,
                              {Location(Vector(0, 7.4, -6), Vector(0, 1, 0), 180)});
    const Locs motor_mounts = detail::compose({at(0, 8.9 - 25, 6)}, grid(47.14, 0, 2, 1));
    detail::threaded_hole(plate, m5, "Soft", "Normal", std::nullopt, false, true, motor_mounts);
    const Locs h1 = detail::compose({at(0, 10 - 25, 6)}, grid(20, 0, 2, 1));
    detail::counter_bore_hole(plate, 2.6, 4.6, 1.55, std::nullopt, h1);
    const Locs h2 = detail::compose({at(0, 5, 6)}, grid(60, 0, 2, 1));
    detail::counter_bore_hole(plate, 2.6, 4.6, 1.55, std::nullopt, h2);
    const Plane back(plate.faces().sort_by(Axis::Y)[-1], Vector(1, 0, 0));
    detail::threaded_hole(plate, m5, "Soft", "Normal", 14 * MM, false, true,
                          detail::compose({Location(back)}, grid(20, 0, 2, 1)));

    // (bd_warehouse adds these joints to the builder's part, which carries them on)
    Part plate_part = plate.part();
    for (std::size_t i = 0; i < motor_mounts.size(); ++i) {
        RigidJoint("motor_mount-" + std::to_string(i), plate_part, motor_mounts[i]);
    }
    Locs screw_holes = h1;
    screw_holes.insert(screw_holes.end(), h2.begin(), h2.end());
    for (std::size_t i = 0; i < screw_holes.size(); ++i) {
        RigidJoint("screw-" + std::to_string(i), plate_part, screw_holes[i] * Pos(0, 0, -1.55));
    }

    assign(plate_part.moved(Pos(0, 5, 6)), rotation, align);
    label = "CBeamEndMount";
    material = black_anodized_aluminum();
    const Edge bearing_circle = faces()
                                    .sort_by(Axis::Z)[2]
                                    .edges()
                                    .filter_by(GeomType::CIRCLE)
                                    .sort_by([](const Edge& e) { return e.radius(); })[-1];
    RigidJoint("bearing", *this, Pos(bearing_circle.arc_center()));
}

CBeamLinearRailProfile::CBeamLinearRailProfile(double rotation, std::optional<Align2> align) {
    Sketch cavity;
    {
        Builder sketch(2);
        {
            Builder line(1);
            line.add(Polyline({Vector(-2.7, 38.2), Vector(-2.7, 36.239339828220196), Vector(-6.1, 32.839339828220226),
                               Vector(-6.1, 27.160660171779885), Vector(-0.7393398282202048, 21.8), Vector(2.7, 21.8),
                               Vector(2.7, 23.76066017177987), Vector(6.1, 27.160660171779845),
                               Vector(6.1, 32.83933982822018), Vector(2.7, 36.239339828220174), Vector(2.7, 38.2)},
                              true));
            sketch.publish(line.product());
        }
        cavity = detail::make_face(sketch);
    }

    Builder cbeam(2);
    cbeam.add(detail::place(Rectangle(40, 20), {at(0, -30), at(0, 30)}));
    cbeam.add(detail::place(Rectangle(20, 80), {at(-10, 0)}));
    const ShapeList<Vertex> v = cbeam.vertices().filter_by([](const Vertex& vertex) { return vertex.X() == 0; }, true);
    detail::fillet(cbeam, shapes(v), FILLET_RADIUS);
    cbeam.add(detail::place(Circle(CAVITY_RADIUS),
                            {at(10, 30), at(-10, 30), at(-10, 10), at(-10, -10), at(-10, -30), at(10, -30)}),
              Mode::SUBTRACT);
    const Locs right{at(10, -30), at(10, 30)};
    cbeam.add(detail::place(vslot_groove(), right), Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_groove(90), right), Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_groove(-90), right), Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_groove(-90), {at(-10, -30)}), Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_groove(90), {at(-10, 30)}), Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_groove(180), detail::compose({at(-10, 0)}, grid(0, 20, 1, 4))), Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_groove(), detail::compose({at(-10, 0)}, grid(0, 20, 1, 2))), Mode::SUBTRACT);
    const Plane diagonal_up(Vector(20, 0, 0), std::nullopt, Vector(1, 1, 0));
    const Plane diagonal_down(Vector(20, 0, 0), std::nullopt, Vector(1, -1, 0));
    detail::insert(cbeam, {cavity}, {Location()}, Mode::SUBTRACT);
    detail::insert(cbeam, {cavity.mirror(Plane::XZ)}, {Location()}, Mode::SUBTRACT);
    detail::insert(cbeam, {cavity.mirror(diagonal_up)}, {Location()}, Mode::SUBTRACT);
    detail::insert(cbeam, {cavity.mirror(diagonal_down)}, {Location()}, Mode::SUBTRACT);
    detail::insert(cbeam, {cavity.mirror(Plane::XZ).mirror(diagonal_down)}, {Location()}, Mode::SUBTRACT);
    cbeam.add(detail::place(vslot_internal_cavity(), {at(-10, 0)}), Mode::SUBTRACT);

    assign(cbeam.sketch(), rotation, align);
}

CBeamLinearRail::CBeamLinearRail(double length, const Rotation& rotation, std::optional<Align3> align) {
    ExtrudeOptions along_z;
    along_z.dir = Vector(0, 0, 1);
    const Part rail = build123d::extrude(CBeamLinearRailProfile(), length, along_z);
    assign(rail, rotation, align);
    material = metals::aluminum();
    label = "CBeamLinearRailx" + detail::py_str(length);
    LinearJoint("screw_axis", *this, Axis(Vector(10, 0, 0), Vector(0, 0, 1)), {0, length});
    RigidJoint("bottom_center", *this,
               Location(Plane(Vector(-20, 0, length / 2), Vector(0, 0, 1), Vector(1, 0, 0))));
    RigidJoint("top_center", *this, Location(Plane(Vector(20, 0, length / 2), Vector(0, -1, 0), Vector(1, 0, 0))));
}

// ---------------------------------------------------------------------------
// The C-Beam plates
// ---------------------------------------------------------------------------

CBeamGantryPlate::CBeamGantryPlate(const Rotation& rotation, std::optional<Align3> align) {
    const SocketHeadCapScrew& m5 = open_builds::m5();
    Builder plate(3);
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(38.575 * 2, 38.575 * 2, 4.635));
        sketch.add(detail::place(SlotCenterToCenter(10, 2.55 * 2), {at(-5, 0)}), Mode::SUBTRACT);
        plate.publish(sketch.product());
    }
    detail::extrude(plate, 6);
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(29.9, 29.9, 4.92));
        publish_sketch(plate, sketch, Plane::XY.offset(6));
    }
    detail::extrude(plate, -1.5, {}, Mode::SUBTRACT);

    detail::counter_bore_hole(plate, 3.6, 6, 1.5, std::nullopt, detail::compose({at(0, 14.25, 6)}, grid(60, 0, 2, 1)));
    detail::counter_bore_hole(plate, 2.55, 4.5, 1.5, std::nullopt,
                              detail::compose({at(0, -14.25, 6)}, grid(60, 0, 2, 1)));
    detail::threaded_hole(plate, m5, "Soft", "Normal", 6 * MM, false, true,
                          detail::compose({at(0, 0, 6)}, grid(40, 40, 2, 2)));
    detail::threaded_hole(plate, m5, "Soft", "Normal", 6 * MM, false, true,
                          detail::compose({at(0, 0, 6)}, grid(20, 60, 2, 2)));
    detail::hole(plate, 2.55, std::nullopt, grid(60, 30, 2, 3));
    detail::hole(plate, 2.55, std::nullopt, grid(20, 20, 3, 2));
    detail::hole(plate, 2.55, std::nullopt, grid(0, 40, 1, 2));
    detail::hole(plate, 2.55, std::nullopt, {at(-10, -10), at(-10, 10), at(10, 0)});

    assign(plate.part(), rotation, align);
    material = black_anodized_aluminum();
    label = "CBeamGantryPlate";
}

CBeamGantryPlateXLarge::CBeamGantryPlateXLarge(const Rotation& rotation, std::optional<Align3> align) {
    Builder plate(3);
    const Locs nut_holes = grid(20, 20, 2, 2);
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(125, 125, 10));
        sketch.add(detail::place(Circle(2.6), grid(30, 30, 3, 3)), Mode::SUBTRACT);
        sketch.add(detail::place(Circle(2.6), grid(20, 80.6, 2, 2)), Mode::SUBTRACT);
        sketch.add(detail::place(Circle(2.6), grid(65, 80.6, 2, 2)), Mode::SUBTRACT);
        sketch.add(detail::place(Circle(2.6), grid(60, 20, 2, 2)), Mode::SUBTRACT);
        sketch.add(detail::place(Circle(2.6), grid(60, 100.6, 2, 2)), Mode::SUBTRACT);
        sketch.add(detail::place(Circle(2.6), nut_holes), Mode::SUBTRACT);
        publish_sketch(plate, sketch, Plane::XY);
    }
    detail::extrude(plate, 6);

    // Recess
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(35, 35, 2.8));
        publish_sketch(plate, sketch, Plane::XY.offset(6));
    }
    detail::extrude(plate, -1.6, {}, Mode::SUBTRACT);

    // Slots
    {
        Builder sketch(2);
        sketch.add(detail::place(SlotOverall(11.135, 4.568 * 2), grid(102.6, 20, 2, 2)));
        publish_sketch(plate, sketch, Plane::XY.offset(6));
    }
    detail::extrude(plate, -1.6, {}, Mode::SUBTRACT);
    {
        Builder sketch(2);
        sketch.add(detail::place(SlotOverall(7.1, 2.55 * 2), grid(102.6, 20, 2, 2)));
        publish_sketch(plate, sketch, Plane::XY);
    }
    detail::extrude(plate, 6, {}, Mode::SUBTRACT);

    // CounterSink - tapped holes (thread not modelled)
    detail::counter_sink_hole(plate, 2.1, 2.7, std::nullopt, 90, detail::compose({at(0, 0, 6)}, grid(100.6, 60, 2, 2)));

    // CounterBore
    const Locs eccentric_mounts = detail::compose({at(0, -50.3, 6)}, grid(50.3, 0, 3, 1));
    detail::counter_bore_hole(plate, 3.6, 6.125, 1.6, 6, eccentric_mounts);
    const Locs fixed_mounts = detail::compose({at(0, 50.3, 6)}, grid(50.3, 0, 3, 1));
    detail::counter_bore_hole(plate, 2.55, 4.5675, 1.6, 6, fixed_mounts);

    assign(plate.part(), rotation, align);
    material = black_anodized_aluminum();
    label = "CBeamGantryPlateXLarge";
    thickness = 6 * MM;
    for_each_labeled({"a", "b", "c"}, eccentric_mounts, [&](const std::string& name, const Location& loc) {
        RigidJoint(name, *this, Pos(loc.position() - Vector(0, 0, 6)));
    });
    for_each_labeled({"d", "e", "f"}, fixed_mounts, [&](const std::string& name, const Location& loc) {
        RigidJoint(name, *this, Pos(loc.position() - Vector(0, 0, 6)));
    });
    for_each_labeled({"a", "b", "c", "d"}, nut_holes, [&](const std::string& name, const Location& loc) {
        RigidJoint("nut_" + name, *this, loc);
    });
    RigidJoint("top_center", *this, Location(Vector(0, 0, 6 * MM)));
}

CBeamRiserPlate::CBeamRiserPlate(const Rotation& rotation, std::optional<Align3> align) {
    Builder plate(3);
    {
        Builder sk(2);
        sk.add(Rectangle(38.75 * 2, 7.437 * 2));
        detail::fillet(sk, shapes(sk.vertices().group_by(Axis::Y)[0]), 1.2);
        detail::fillet(sk, shapes(sk.vertices().group_by(Axis::Y)[-1]), 3.5);
        sk.add(detail::place(Circle(2.55), detail::compose({at(0, -1.323)}, grid(20, 0, 2, 1))), Mode::SUBTRACT);
        plate.publish(sk.product());
    }
    detail::extrude(plate, 8);
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(32.24, 2 * 11.86712, 0.75));
        publish_sketch(plate, sketch, Plane(Vector(0, -7.427, 8)));
    }
    detail::extrude(plate, -2.1, {}, Mode::SUBTRACT);
    detail::threaded_hole(plate, open_builds::m5(), "Soft", "Normal", 8 * MM, false, true,
                          detail::compose({at(0, -1.323, 8)}, grid(60, 0, 2, 1)));

    assign(plate.part(), rotation, align);
    material = black_anodized_aluminum();
    label = "CBeamRiserPlate";
}

// ---------------------------------------------------------------------------
// EccentricSpacer, FlexibleCoupler, LBracket, LockCollar, MetricLeadScrew
// ---------------------------------------------------------------------------

EccentricSpacer::EccentricSpacer(const std::string& cam_height, const Rotation& rotation, std::optional<Align3> align) {
    static const Dict<std::string, double> valid_heights = {{"6mm", 6 * MM}, {"1/4in", IN / 4}};
    const double cam_length = valid_value(valid_heights, cam_height, " is an invalid cam height, must be one of ");

    // 10mm across the flats
    const double hex_radius = (2.0 / 3) * 10 * MM * std::sin(detail::radians(60));
    Builder spacer(3);
    {
        Builder sketch(2);
        sketch.add(RegularPolygon(hex_radius, 6));
        spacer.publish(sketch.product());
    }
    detail::extrude(spacer, 6);
    {
        Builder chamf(2);
        chamf.add(Rectangle(hex_radius, cam_length, Align2(Align::MIN)));
        detail::chamfer(chamf, shapes(chamf.vertices().group_by(Axis::Y)[0]), 0.6);
        publish_sketch(spacer, chamf, Plane::YZ);
    }
    detail::revolve(spacer, Axis::Z, 360.0, true, Mode::INTERSECT);
    spacer.add(detail::place(Cylinder(hex_radius + 0.25 * MM, 1 * MM, Align3(Align::CENTER, Align::CENTER, Align::MAX)),
                             {at(0, 0, cam_length)}));
    {
        Builder sketch(2);
        sketch.add(Circle(3.555));
        spacer.publish(sketch.product());
    }
    detail::extrude(spacer, cam_length + 2.5 * MM);
    detail::hole(spacer, 2.5, std::nullopt, {at(0, 0.79 * MM)});  // eccentric hole
    {
        const Face text_face = spacer.faces().filter_by(Axis::Y).sort_by(Axis::Y)[-1];
        Builder sketch(2);
        TextOptions bold;
        bold.font_style = FontStyle::BOLD;
        sketch.add(Text(cam_height, 1.5 * MM, bold));
        publish_sketch(spacer, sketch, Plane(text_face));
    }
    detail::extrude(spacer, -0.1 * MM, {}, Mode::SUBTRACT);

    Part spacer_part = spacer.part();
    spacer_part.locate(Pos(0, -0.79 * MM, 0));
    assign(spacer_part, rotation, align);
    material = metals::aluminum();
    label = "EccentricSpacer-" + cam_height;
    // "a" and "b" are the bottom and top mounting faces, respectively.
    RigidJoint("a", *this, Location());
    RigidJoint("b", *this, Pos(0, 0, 2.5 * MM + cam_length - 2.5 * MM));
    // "center" identifies the eccentric mounting/rotation axis at the top face.
    RigidJoint("center", *this, Pos(0, -0.79 * MM, cam_length));
}

FlexibleCoupler::FlexibleCoupler(const std::string& shaft_diameter, const Rotation& rotation,
                                 std::optional<Align3> align) {
    static const Dict<std::string, double> valid_diameters = {{"8mm", 8 * MM}, {"1/4in", IN / 4}};
    const double d2 =
        valid_value(valid_diameters, shaft_diameter, " is an invalid shaft diameter, must be one of ");
    const double d1 = 5 * MM;  // NEMA 17

    Builder coupler(3);
    coupler.add(Cylinder(10, 25, Align3(Align::CENTER, Align::CENTER, Align::MAX)));
    {
        const Locs top{Location(Plane(coupler.faces().sort_by(Axis::Z)[-1]))};
        detail::hole(coupler, d1 / 2, 25 * MM / 2, top);
        coupler.add(detail::place(Box(10, 0.9, 8.3, Align3(Align::MIN, Align::CENTER, Align::MAX)), top), Mode::SUBTRACT);
    }
    {
        const Locs bottom{Location(Plane(coupler.faces().sort_by(Axis::Z)[0]))};
        detail::hole(coupler, d2 / 2, 25 * MM / 2, bottom);
        coupler.add(detail::place(Box(11, 0.9, 8.3, Align3(Align::MIN, Align::CENTER, Align::MAX)), bottom),
                    Mode::SUBTRACT);
    }
    detail::chamfer(coupler, shapes(coupler.edges().filter_by(GeomType::CIRCLE)), 0.3 * MM);
    {
        const Locs set_screws{Location(Vector(5, 9.2, -4), Vector(1, 0, 0), -90),
                              Location(Vector(5, 9.2, -21), Vector(1, 0, 0), -90)};
        coupler.add(detail::place(Cylinder(6.0 / 2, 6), set_screws), Mode::SUBTRACT);
        detail::hole(coupler, 1.7, std::nullopt, set_screws);
    }
    const Helix spiral(2, 2, 11, Vector(0, 0, -18.2));
    {
        Builder line(1);
        line.add(spiral);
        coupler.publish(line.product());
    }
    {
        Builder sketch(2);
        sketch.add(Rectangle(10, 0.7, Align2(Align::MIN, Align::MIN)));
        publish_sketch(coupler, sketch, Plane(spiral.location_at(0)));
    }
    const Part spiral_cut =
        detail::sweep(coupler, true, Transition::TRANSFORMED, true, Mode::PRIVATE).rotate(Axis::Z, 90);
    for (int i = 0; i < 10; i += 2) {
        detail::insert(coupler, {spiral_cut.moved(Pos(0, 0, i))}, {Location()}, Mode::SUBTRACT);
    }

    assign(coupler.part().rotate(Axis::X, 180), rotation, align);
    MetalOptions bead_blasted;
    bead_blasted.finish = FinishSpec({finishes::bead_blast()});
    material = metals::aluminum(bead_blasted);
    label = "FlexibleCoupler-" + shaft_diameter;
    RigidJoint("a", *this, Pos(0, 0, 12.5 * MM));
    RigidJoint("b", *this, Location(Vector(0, 0, 12.5 * MM), Vector(1, 0, 0), 180));
}

LBracket::LBracket(const Rotation& rotation, std::optional<Align3> align) {
    Builder bracket(3);
    for (const auto& [c, pln] : std::vector<std::pair<double, Plane>>{{15, Plane::XY}, {10, -Plane::ZY}}) {
        Builder sketch(2);
        sketch.add(Rectangle(20, 14.5, Align2(Align::MIN, Align::CENTER)));
        sketch.add(detail::place(SlotCenterToCenter(1.75, 5.5, 90), {at(c, 0)}), Mode::SUBTRACT);
        publish_sketch(bracket, sketch, pln);
        detail::extrude(bracket, 3);
    }

    assign(bracket.part(), rotation, align);
    material = black_anodized_aluminum();
    RigidJoint("corner", *this, -Location());
    RigidJoint("screw_a", *this, Pos(15, 0, 3));
    RigidJoint("screw_b", *this, Location(Plane(Vector(3, 0, 10), std::nullopt, Vector(1, 0, 0))));
    RigidJoint("nut_a", *this, Pos(15, 0, 0) * Rot(0, 0, 90));
    RigidJoint("nut_b", *this, Location(Plane(Vector(0, 0, 10), Vector(0, 1, 0), Vector(1, 0, 0))));
    label = "LBracket";
}

LockCollar::LockCollar(const std::string& inside_diameter) {
    static const Dict<std::string, std::array<double, 3>> valid_sizes = {
        {"5mm", {5 * MM, IN / 2, 7 * MM}},
        {"8mm", {8 * MM, 14 * MM, 7 * MM}},
        {"1/4in", {IN / 4, IN / 2, 7 * MM}},
    };
    const auto [bore, od, h] =
        valid_value(valid_sizes, inside_diameter, " is an invalid inside diameter, must be one of ");

    SetScrew setscrew("M5-0.8", 4 * MM);
    Builder collar(3);
    {
        Builder sketch(2);
        sketch.add(Circle(od / 2));
        sketch.add(Circle(bore / 2), Mode::SUBTRACT);
        collar.publish(sketch.product());
    }
    detail::extrude(collar, h);
    const Locs recorded = detail::threaded_hole(collar, setscrew, "Soft", "Normal", od / 2, true, true,
                                                {Location(Vector(0, od / 2, h / 2), Vector(1, 0, 0), -90)});
    setscrew.hole_locations.insert(setscrew.hole_locations.end(), recorded.begin(), recorded.end());

    Part collar_part = collar.part();
    collar_part.color = Color(0x020202);
    collar_part.label = "Collar";
    setscrew.locate(setscrew.hole_locations.at(0));
    children = std::vector<Shape>{collar_part, setscrew};
    label = "LockCollar" + inside_diameter;
    RigidJoint("a", *this, Location());
    RigidJoint("b", *this, -Location(Vector(0, 0, h)));
}

MetricLeadScrew::MetricLeadScrew(double length, bool simple) {
    double root_radius = 7.8 * MM / 2;
    std::optional<TrapezoidalThread> thread;
    if (!simple) {
        thread.emplace(7.8 * MM, 2 * MM, 30, length,
                       TrapezoidalThreadOptions{.starts = 4, .end_finishes = {"chamfer", "chamfer"}});
        root_radius = thread->root_radius;
    }
    std::vector<Shape> components{Cylinder(root_radius, length, Align3(Align::CENTER, Align::CENTER, Align::MIN))};
    if (!simple) components.push_back(*thread);
    children = components;

    material = metals::stainless();
    RigidJoint("axis", *this, Location());
    label = "8mm lead screw x " + detail::py_str(length);
}

// ---------------------------------------------------------------------------
// RouterSpindleMount
// ---------------------------------------------------------------------------

RouterSpindleMount::RouterSpindleMount(const std::string& parts, const Rotation& rotation, std::optional<Align3> align) {
    const SocketHeadCapScrew m5("M5-0.8", 20 * MM);
    const double width = 90.8 * MM;
    const double height = 30.6 * MM + 54.5 * MM;
    const double thickness = 20 * MM;
    Locs top_mount_centers;
    std::optional<Part> base_part, faceplate_part;
    if (parts == "base" || parts == "both") {
        Builder base(3);
        // Create the basic shape
        {
            Builder c(2);
            c.add(detail::place(Rectangle(width, height), {at(0, height / 2 - 54.5)}));
            c.add(Circle(35.5), Mode::SUBTRACT);
            c.add(Rectangle(28.15 * 2, 35.5, Align2(Align::CENTER, Align::MIN)), Mode::SUBTRACT);
            detail::fillet(c, shapes(c.vertices().group_by(Axis::Y)[0]), 5);
            detail::fillet(c, shapes(c.vertices().group_by(Axis::Y)[-2]), 7.5);
            publish_sketch(base, c, Plane::XY.offset(-thickness / 2));
        }
        detail::extrude(base, thickness);

        // Create the holes
        top_mount_centers = {at(30.0, -44.2, thickness / 2), at(20.0, -44.2, thickness / 2),
                             at(-20.0, -44.2, thickness / 2), at(-30.0, -44.2, thickness / 2)};
        detail::threaded_hole(base, m5, "Soft", "Normal", std::nullopt, false, true, top_mount_centers);
        detail::threaded_hole(base, m5, "Soft", "Normal", 7 * MM, false, true,
                              detail::compose({Location(Plane(base.faces().sort_by(Axis::Y)[0]))}, grid(20, 0, 4, 1)));
        detail::threaded_hole(base, m5, "Soft", "Normal", 9 * MM, false, true,
                              detail::compose({Location(-Plane::YZ.offset(-width / 2)), Location(Plane::YZ.offset(width / 2))},
                                              {at(10.6, 0), at(-29.4, 0)}));
        detail::threaded_hole(
            base, m5, "Soft", "Normal", 13 * MM, false, true,
            detail::compose({Location(-Plane::XZ.offset(-base.part().bounding_box().max.Y))}, grid(36.8 * 2, 0, 2, 1)));
        base_part = base.part();
    }

    if (parts == "faceplate" || parts == "both") {
        Builder faceplate(3);
        {
            Builder s(2);
            // Create basic half shape
            s.add(Rectangle(23.65, 48.2, Align2(Align::MIN)));
            s.add(detail::place(Rectangle(45.4, 48.2 - 36.2, Align2(Align::MIN, Align::MAX)), {at(0, 48.2)}));
            s.add(Circle(35.5), Mode::SUBTRACT);

            // Fillet corners
            detail::fillet(s, {s.vertices().group_by(Axis::Y)[-1].sort_by(Axis::X)[-1]}, 1.4);
            detail::fillet(s, {s.vertices().group_by(Axis::Y)[-3].sort_by(Axis::X)[0]}, 3);
            detail::fillet(s, {s.vertices().sort_by(Axis::Y)[0]}, 1.3);

            // Mirror
            detail::mirror(s, std::nullopt, Plane::YZ);
            publish_sketch(faceplate, s, Plane::XY.offset(-thickness / 2));
        }
        detail::extrude(faceplate, thickness);

        // Holes
        {
            Locs face_locations;
            for (const Face& face : faceplate.faces().group_by(Axis::Y)[-1]) face_locations.emplace_back(Plane(face));
            detail::counter_bore_hole(faceplate, 3, 5.1, 1.5, std::nullopt,
                                      detail::compose(face_locations, grid(36.8 * 2, 0, 2, 1)));
        }

        // Sunken embossed text
        {
            const Face front = faceplate.faces().sort_by(Axis::Y)[-1];
            Builder sketch(2);
            sketch.add(RectangleRounded(27.45 * 2, 13, 2.5));
            publish_sketch(faceplate, sketch, Plane(front));
        }
        detail::extrude(faceplate, -1 * MM, {}, Mode::SUBTRACT);
        {
            const Face recess = faceplate.faces().filter_by(Axis::Y).sort_by(Axis::Y)[-2];
            Builder sketch(2);
            TextOptions bold;
            bold.font_style = FontStyle::BOLD;
            sketch.add(Text("OPENBUILDS", 7.5 * MM, bold));
            publish_sketch(faceplate, sketch, Plane(recess));
        }
        detail::extrude(faceplate, 0.1 * MM);
        faceplate_part = faceplate.part();
    }

    // bd_warehouse 0.4.0 reads both builders here, so "base" or "faceplate" alone fail (Python's
    // UnboundLocalError)
    if (!base_part) {
        throw std::runtime_error("cannot access local variable 'base' where it is not associated with a value");
    }
    if (!faceplate_part) {
        throw std::runtime_error("cannot access local variable 'faceplate' where it is not associated with a value");
    }
    Part mount;
    if (parts == "both") {
        mount = *base_part + *faceplate_part;
    } else if (parts == "base") {
        mount = *base_part;
    } else if (parts == "faceplate") {
        mount = *faceplate_part;
    } else {
        throw std::invalid_argument("the parts parameter must one of 'base', 'faceplate' or 'both' not " + parts);
    }
    mount.label = "RouterSpindleMount";
    assign(mount, rotation, align);
    material = black_anodized_aluminum();
    for_each_labeled({"a", "b", "c", "d"}, top_mount_centers, [&](const std::string& name, const Location& pos) {
        RigidJoint("top_" + name, *this, -pos);
    });
    for_each_labeled({"a", "b", "c", "d"}, top_mount_centers, [&](const std::string& name, const Location& pos) {
        RigidJoint("bottom_" + name, *this, pos * Pos(0, 0, -thickness));
    });
    RigidJoint("back_mount", *this, faces().sort_by(Axis::Y)[0].location_at(0.5, 0.5, Vector(-1, 0, 0)));
}

// ---------------------------------------------------------------------------
// ShimWasher, SpacerBlock, TNut
// ---------------------------------------------------------------------------

ShimWasher::ShimWasher(const std::string& shim_type, const Rotation& rotation, std::optional<Align3> align) {
    // OD, ID, Thickness
    static const Dict<std::string, std::array<double, 3>> dimensions = {
        {"MiniVWheel", {8, 5, 1}},
        {"10x5x1", {10, 5, 1}},
        {"12x8x1", {12, 8, 1}},
        {"SlotWasher", {15, 5, 2}},
        {"FlatWasher", {0.75 * IN, 0.25 * IN, 0.08 * IN}},
    };
    const auto found = dimensions.get(shim_type);
    if (!found) {
        throw std::invalid_argument("Invalid shim_type " + shim_type + " must be one of " +
                                    detail::py_tuple_repr(dimensions.keys()));
    }
    const auto [od, bore, thickness] = *found;

    Builder shim(3);
    {
        Builder sketch(2);
        sketch.add(Circle(od / 2));
        sketch.add(Circle(bore / 2), Mode::SUBTRACT);
        shim.publish(sketch.product());
    }
    detail::extrude(shim, thickness);
    detail::fillet(shim, shapes(shim.edges().group_by(SortBy::LENGTH)[-1]), thickness / 5);

    assign(shim.part(), rotation, align);
    material = metals::aluminum();
    label = "ShimWasher-" + shim_type;
    RigidJoint("a", *this, Location());
    RigidJoint("b", *this, Pos(0, 0, thickness));
}

SpacerBlock::SpacerBlock(const Rotation& rotation, std::optional<Align3> align) {
    Builder plate(3);
    {
        Builder sketch(2);
        sketch.add(RectangleRounded(86.4, 20, 3.36));
        sketch.add(detail::place(Circle(3.6), grid(30, 0, 3, 1)), Mode::SUBTRACT);
        plate.publish(sketch.product());
    }
    detail::extrude(plate, 12);
    detail::threaded_hole(plate, open_builds::m5(), "Soft", "Normal", 12, false, true,
                          detail::compose({at(0, 0, 12)}, grid(40, 0, 2, 1)));

    assign(plate.part(), rotation, align);
    material = black_anodized_aluminum();
    label = "SpacerBlock";
}

TNut::TNut(const Rotation& rotation, std::optional<Align3> align) {
    const double w = 5.9, h = 4, l = 10;
    Builder nut_builder(3);
    {
        Builder profile(2);
        profile.add(Rectangle(w, h, Align2(Align::CENTER, Align::MAX)));
        profile.add(detail::place(Rectangle(l, 3.2, Align2(Align::CENTER, Align::MAX)), {at(0, -0.8)}));
        detail::chamfer(profile, shapes(profile.vertices().group_by(Axis::Y)[0]), 1.6);
        publish_sketch(nut_builder, profile, Plane::XZ);
    }
    ExtrudeOptions both_ways;
    both_ways.both = true;
    detail::extrude(nut_builder, 5.9 / 2, both_ways);
    {
        Builder plan(2);
        plan.add(Rectangle(l, w));
        const std::vector<Shape> corners{plan.vertices().group_by(Axis::Y)[0].sort_by(Axis::X)[0],
                                         plan.vertices().group_by(Axis::Y)[-1].sort_by(Axis::X)[-1]};
        detail::fillet(plan, corners, w / 2);
        nut_builder.publish(plan.product());
    }
    detail::extrude(nut_builder, -h, {}, Mode::INTERSECT);
    detail::tap_hole(nut_builder, open_builds::m5(), "Soft", "Normal", std::nullopt, false);

    assign(nut_builder.part(), rotation, align);
    material = metals::mild_steel();
    RigidJoint("a", *this, Pos(0, 0, 1));
    RigidJoint("b", *this, Pos(0, 0, 0));
    label = "TNut";
}

// ---------------------------------------------------------------------------
// V-Slot rails
// ---------------------------------------------------------------------------

VSlotLinearRailProfile::VSlotLinearRailProfile(const std::string& rail_size, double rotation,
                                               std::optional<Align2> align) {
    Builder cavity_40x40(2);
    {
        Builder line(1);
        const Polyline l({Vector(18.2, 2.7), Vector(16.239339828219503, 2.7), Vector(12.839339828219602, 6.1),
                          Vector(7.1606601717797975, 6.1), Vector(6.1, 7.160660171780418),
                          Vector(6.1, 12.839339828220236), Vector(2.7, 16.23933982822021), Vector(2.7, 18.2),
                          Vector(-2.7, 18.2)});
        line.add(l);
        detail::insert(line, {l.rotate(Axis::Z, 90)});
        detail::insert(line, {l.rotate(Axis::Z, 180)});
        detail::insert(line, {l.rotate(Axis::Z, 270)});
        cavity_40x40.publish(line.product());
    }
    detail::make_face(cavity_40x40);

    check_rail_size(rail_size);
    const std::vector<std::string> size = detail::py_split(rail_size, "x");
    const int width = std::stoi(size.at(0));
    const int height = std::stoi(size.at(1));

    Builder vslot(2);
    vslot.add(RectangleRounded(width, height, FILLET_RADIUS));
    vslot.add(detail::place(Circle(CAVITY_RADIUS), grid(20, 20, width / 20, height / 20)), Mode::SUBTRACT);
    const GrooveSet& grooves = vslot_grooves().at(rail_size);
    for (std::size_t i = 0; i < std::min(grooves.positions.size(), grooves.angles.size()); ++i) {
        const auto& [x, y] = grooves.positions[i];
        vslot.add(detail::place(vslot_groove(grooves.angles[i]), {at(x, y)}), Mode::SUBTRACT);
    }
    if (rail_size == "20x40" || rail_size == "20x60" || rail_size == "20x80") {
        vslot.add(detail::place(vslot_internal_cavity(), grid(0, 20, 1, height / 20 - 1)), Mode::SUBTRACT);
    } else if (rail_size == "40x40") {
        detail::insert(vslot, {cavity_40x40.sketch()}, {Location()}, Mode::SUBTRACT);
    }

    assign(vslot.sketch(), rotation, align);
}

VSlotLinearRail::VSlotLinearRail(const std::string& rail_size, double length, const Rotation& rotation,
                                 std::optional<Align3> align) {
    check_rail_size(rail_size);
    ExtrudeOptions along_z;
    along_z.dir = Vector(0, 0, 1);
    const Part rail = build123d::extrude(VSlotLinearRailProfile(rail_size), length, along_z);
    assign(rail, rotation, align);
    material = metals::aluminum();
    label = "VSlot Rail " + rail_size + "x" + detail::py_str(length);
}

// ---------------------------------------------------------------------------
// Gantry and wheels
// ---------------------------------------------------------------------------

XLargeCBeamGantry::XLargeCBeamGantry(int wheel_count, double eccentric_angle) {
    if (wheel_count != 4 && wheel_count != 6) {
        throw std::invalid_argument("wheel_count of " + std::to_string(wheel_count) + " must be either 4 or 6");
    }
    CBeamGantryPlateXLarge plate;
    XtremeSolidVWheelAssembly w0(true);
    XtremeSolidVWheelAssembly w2 = w0;
    XtremeSolidVWheelAssembly w3(false);
    XtremeSolidVWheelAssembly w5 = w3;
    AcmeAntiBacklashNutBlock8mmAssembly acme_nut_assembly;

    plate.joints["a"].connect_to(w0.joints["mount"], {.angle = eccentric_angle});
    plate.joints["c"].connect_to(w2.joints["mount"], {.angle = eccentric_angle});
    plate.joints["d"].connect_to(w3.joints["mount"]);
    plate.joints["f"].connect_to(w5.joints["mount"]);
    std::vector<Shape> wheels{w0, w2, w3, w5};
    if (wheel_count == 6) {
        XtremeSolidVWheelAssembly w1 = w0;
        XtremeSolidVWheelAssembly w4 = w3;
        plate.joints["b"].connect_to(w1.joints["mount"], {.angle = eccentric_angle});
        plate.joints["e"].connect_to(w4.joints["mount"]);
        wheels.push_back(w1);
        wheels.push_back(w4);
    }
    plate.joints["nut_a"].connect_to(acme_nut_assembly.joints["a"]);

    label = "XLargeCBeamGantry";
    std::vector<Shape> components{plate, acme_nut_assembly};
    components.insert(components.end(), wheels.begin(), wheels.end());
    children = components;
    RigidJoint("nut", *this, Location(Vector(0, 0, -12.5 * MM), Vector(0, 1, 0), -90));
    RigidJoint("top_center", *this, plate.joints["top_center"].location());
}

XtremeSolidVWheel::XtremeSolidVWheel(const Rotation& rotation, std::optional<Align3> align) {
    Builder wheel(3);
    {
        Builder x_section(2);
        const Locs center{at(8, 0)};
        x_section.add(detail::place(Rectangle(3.95, 10.2, Align2(Align::MIN, Align::CENTER)), center));
        x_section.add(detail::place(Rectangle(2, 1), center));
        detail::chamfer(x_section, shapes(x_section.vertices().group_by(Axis::X)[-1]), 2.15);
        publish_sketch(wheel, x_section, Plane::YZ);
    }
    detail::revolve(wheel);

    assign(wheel.part(), rotation, align);
    TransparentOptions polycarbonate;
    polycarbonate.color = ColorSpec(Color("white"));
    polycarbonate.thickness_mm = 2;
    polycarbonate.opacity = 0.1;
    polycarbonate.roughness = 0.45;
    material = plastics::pc(polycarbonate);
    label = "XtremeSolidVWheel";
    RigidJoint("a", *this, Pos(0, 0, -11.0 / 2));
    RigidJoint("b", *this, Pos(0, 0, +11.0 / 2));
}

XtremeSolidVWheelAssembly::XtremeSolidVWheelAssembly(bool eccentric) {
    HexNut nut("M5-0.8");
    SingleRowCappedDeepGrooveBallBearing b0("M5-16-5");
    SingleRowCappedDeepGrooveBallBearing b1 = b0;
    ShimWasher shim0("10x5x1");
    ShimWasher shim1 = shim0;
    Part spacer = eccentric ? Part(EccentricSpacer("6mm")) : Part(AluminumSpacer("6mm"));
    XtremeSolidVWheel tire;
    tire.joints["a"].connect_to(b0.joints["a"]);
    b0.joints["b"].connect_to(shim0.joints["a"]);
    b0.joints["a"].connect_to(nut.joints["b"]);
    tire.joints["b"].connect_to(b1.joints["b"]);
    b1.joints["b"].connect_to(shim1.joints["a"]);
    shim1.joints["b"].connect_to(spacer.joints["a"]);
    LowProfileScrew screw("M5-0.8", 30, ScrewOptions{.simple = false});
    screw.set_position(screw.position() + Vector(0, 0, spacer.joints["b"].location().position().Z + 6 - 1.55));

    label = "XtremeSolidVWheelAssembly";
    children = std::vector<Shape>{tire, b0, shim0, shim1, b1, spacer, nut, screw};
    // The spacer's "center" joint is the wheel assembly's mounting axis.
    RevoluteJoint("mount", *this, Axis(spacer.joints["center"].location().position(), Vector(0, 0, 1)));
}

// ---------------------------------------------------------------------------
// StepperMotor
// ---------------------------------------------------------------------------

StepperMotor::StepperMotor(const std::string& motor_type, std::optional<double> motor_length) {
    // length, shaft_d, flat, shaft_l1, shaft_l2
    static const Dict<std::string, std::array<double, 5>> motor_data = {
        {"Nema17", {48, 5, 2, 9, 24}},
        {"Nema23", {56, IN / 4, 2.675, 5.6, 20.6}},
        {"Nema23HighTorque", {86, IN / 4, 2.675, 5.6, 20.6}},
    };
    const auto found = motor_data.get(motor_type);
    if (!found) {
        throw std::invalid_argument(motor_type + " is invalid, must be one of dict_keys(" +
                                    detail::py_list_repr(motor_data.keys()) + ")");
    }
    auto [length, shaft_d, flat, shaft_l1, shaft_l2] = *found;
    if (motor_length) {
        if (*motor_length <= 0) {
            throw std::invalid_argument("Expected motor_length greater than 0, got " + detail::py_str(*motor_length));
        }
        length = *motor_length;
    }

    const SocketHeadCapScrew m3("M3-0.5", 20);
    Builder stepper(3);
    Locs mount_holes;
    if (motor_type == "Nema23" || motor_type == "Nema23HighTorque") {
        const double skt1_length = std::min(length / 2, 11.0);
        const double skt2_length = std::min(length, 4.8);
        Builder skt1(2);
        {
            Builder line(1);
            const Line l1(Vector(28.2, 0), Vector(28.2, 16.07));
            line.add(l1);
            const JernArc l2(l1.position_at(1), l1.tangent_at(1), 2.5, 90);
            line.add(l2);
            const Line l3(l2.position_at(1), Vector(23.57, 18.57));
            line.add(l3);
            const JernArc l4(l3.position_at(1), l3.tangent_at(1), 5, -45);
            line.add(l4);
            detail::mirror(line, std::nullopt, Plane(l4.location_at(1)));
            detail::mirror(line, std::nullopt, Plane::YZ);
            detail::mirror(line, std::nullopt, Plane::XZ);
            skt1.publish(line.product());
        }
        detail::make_face(skt1);
        stepper.publish(skt1.product());
        detail::extrude(stepper, -skt1_length);
        detail::insert(stepper, {Face(skt1.wire().offset_2d(-0.1))});
        detail::extrude(stepper, -length + skt1_length);
        detail::insert(stepper, {skt1.face().located(Pos(0, 0, -length + skt1_length))});
        detail::extrude(stepper, -skt1_length);
        {
            Builder sketch(2);
            sketch.add(RectangleRounded(28.2 * 2, 28.2 * 2, 4.6));
            mount_holes = grid(23.57 * 2, 23.57 * 2, 2, 2);
            sketch.add(detail::place(Circle(2.55), mount_holes), Mode::SUBTRACT);
            stepper.publish(sketch.product());
        }
        detail::extrude(stepper, -skt2_length);
        {
            Builder sketch(2);
            sketch.add(Circle(19.05));
            stepper.publish(sketch.product());
        }
        detail::extrude(stepper, 1.6);
    } else {
        const double skt1_length = std::min(length / 2, 8.75);
        Builder skt1(2);
        skt1.add(Rectangle(21.209 * 2, 21.209 * 2));
        skt1.add(Circle(26.5), Mode::INTERSECT);
        stepper.publish(skt1.product());
        detail::extrude(stepper, -skt1_length);
        {
            Builder sketch(2);
            sketch.add(Rectangle(21.109 * 2, 21.109 * 2));
            sketch.add(Circle(25), Mode::INTERSECT);
            stepper.publish(sketch.product());
        }
        detail::extrude(stepper, -length);
        detail::insert(stepper, {skt1.face().located(Pos(0, 0, -length + skt1_length))});
        detail::extrude(stepper, -skt1_length);
        mount_holes = grid(15.5 * 2, 15.5 * 2, 2, 2);
        detail::hole(stepper, 1.5, 6, mount_holes);
        detail::threaded_hole(stepper, m3, "Soft", "Normal", 7.5, false, true, mount_holes);
        {
            Builder sketch(2);
            sketch.add(Circle(11));
            stepper.publish(sketch.product());
        }
        detail::extrude(stepper, 2);
    }

    Builder shaft(3);
    {
        Builder sketch(2);
        sketch.add(Circle(shaft_d / 2));
        shaft.publish(sketch.product());
    }
    detail::extrude(shaft, shaft_l1);
    {
        Builder sketch(2);
        sketch.add(Circle(shaft_d / 2));
        sketch.add(detail::place(Rectangle(shaft_d, shaft_d, Align2(Align::CENTER, Align::MIN)), {at(0, flat)}),
                   Mode::SUBTRACT);
        shaft.publish(sketch.product());
    }
    detail::extrude(shaft, shaft_l2);

    Part stepper_part = stepper.part();
    Part shaft_part = shaft.part();
    stepper_part.color = Color(0x020202);
    shaft_part.color = Color(0xC0C0C0);

    label = "StepperMotor-" + motor_type;
    children = std::vector<Shape>{stepper_part, shaft_part};
    for_each_labeled({"a", "b", "c", "d"}, mount_holes, [&](const std::string& name, const Location& loc) {
        RigidJoint(name, *this, -loc);
    });
    RigidJoint("shaft", *this, faces().sort_by(Axis::Z)[-1].location_at(0.5, 0.5));
}

}  // namespace bd_warehouse
