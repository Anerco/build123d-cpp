// bd_warehouse's open_builds.py, compared with cases_bdw_open_builds_py.py.
#include <functional>
#include <regex>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

const std::vector<std::string> kSpacerLengths = {"3mm",  "1/8in",  "6mm",  "1/4in",   "9mm", "10mm",
                                                 "13.2mm", "20mm", "35mm", "1-1/2in", "40mm"};
const std::vector<std::string> kShimTypes = {"MiniVWheel", "10x5x1", "12x8x1", "SlotWasher", "FlatWasher"};
const std::vector<std::string> kRailSizes = {"20x20", "20x40", "20x60", "20x80", "40x40"};
const std::vector<std::string> kMotorTypes = {"Nema17", "Nema23", "Nema23HighTorque"};

std::string key(const std::string& text) {
    std::string out = std::regex_replace(text, std::regex("[^A-Za-z0-9]+"), "_");
    while (!out.empty() && out.front() == '_') out.erase(out.begin());
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out;
}

std::string joint_kind(const Joint& joint) {
    if (joint.is<RigidJoint>()) return "RigidJoint";
    if (joint.is<LinearJoint>()) return "LinearJoint";
    if (joint.is<RevoluteJoint>()) return "RevoluteJoint";
    if (joint.is<CylindricalJoint>()) return "CylindricalJoint";
    return "BallJoint";
}

/// A location as its position and x and z directions (Euler angles of 180 degrees flip between
/// -180 and 180 with the sign of rounding noise).
V frame(const Location& loc) { return V::list({loc.position(), loc.x_axis().direction, loc.z_axis().direction}); }

V joint_values(const Shape& shape) {
    std::vector<V> out;
    for (const auto& [name, joint] : shape.joints.items()) {
        out.push_back(V::list({name, joint_kind(joint), frame(joint.location())}));
    }
    return V::list(out);
}

/// The material's fields (Python's repr prints a thickness given as an int without ".0").
V material_values(const Shape& shape) {
    const auto& m = shape.material.get();
    if (!m) return V();
    return V::list({m->material.name, m->finish ? V(m->finish->repr()) : V(), m->color ? V(m->color->repr()) : V(),
                    opt(m->thickness_mm), opt(m->opacity), opt(m->roughness), V::list({m->scale.first, m->scale.second}),
                    m->rotation});
}

V color_values(const Shape& shape) {
    const auto& color = shape.color.get();
    if (!color) return V();
    const auto rgba = color->to_tuple();
    return V::list({rgba[0], rgba[1], rgba[2], rgba[3]});
}

std::vector<V> shape_fields(const Shape& shape) {
    return {shape.label, material_values(shape), color_values(shape), frame(shape.location()), joint_values(shape)};
}

V shape_values(const Shape& shape) { return V::list(shape_fields(shape)); }

V assembly_values(const Shape& shape) {
    std::vector<V> out = shape_fields(shape);
    std::vector<V> children;
    for (const auto& c : shape.children) {
        children.push_back(V::list({c.label, c.volume(), frame(c.location()), material_values(c), color_values(c),
                                    c.children.size()}));
    }
    out.push_back(V::list(children));
    return V::list(out);
}

using Values = std::function<V(const Shape&)>;

template <class T>
void add_case(const std::string& name, std::function<T()> make, const Values& values = shape_values) {
    compare::add("bdw_open_builds_" + name, [make] { return Shape(make()); });
    compare::add_value("bdw_open_builds_" + name + "_values", [make, values] { return values(make()); });
}

}  // namespace

COMPARE_CASES {
    using compare::add_value;
    const Values assembly = assembly_values;

    // ---- parts
    add_case<AcmeAntiBacklashNutBlock8mm>("acme_nut_block", [] { return AcmeAntiBacklashNutBlock8mm(); });
    add_case<AcmeAntiBacklashNutBlock8mm>("acme_nut_block_rotated", [] {
        return AcmeAntiBacklashNutBlock8mm(Rotation(0, 0, 45), Align3(Align::CENTER, Align::CENTER, Align::MIN));
    });
    for (const auto& length : kSpacerLengths) {
        add_case<AluminumSpacer>("aluminum_spacer_" + key(length), [length] { return AluminumSpacer(length); });
    }
    add_case<AluminumSpacer>("aluminum_spacer_aligned",
                             [] { return AluminumSpacer("20mm", Rotation(90, 0, 0), Align3(Align::MIN)); });
    add_case<CBeamEndMount>("cbeam_end_mount", [] { return CBeamEndMount(); });
    add_case<CBeamEndMount>("cbeam_end_mount_align_none", [] { return CBeamEndMount(Rotation(), Align3(Align::NONE)); });
    add_case<CBeamEndMount>("cbeam_end_mount_rotated", [] { return CBeamEndMount(Rotation(90, 0, -90), std::nullopt); });
    add_case<CBeamLinearRailProfile>("cbeam_linear_rail_profile", [] { return CBeamLinearRailProfile(); });
    add_case<CBeamLinearRailProfile>("cbeam_linear_rail_profile_rotated",
                                     [] { return CBeamLinearRailProfile(30, Align2(Align::MIN, Align::MAX)); });
    add_case<CBeamLinearRail>("cbeam_linear_rail", [] { return CBeamLinearRail(100.0); });
    add_case<CBeamLinearRail>("cbeam_linear_rail_rotated", [] {
        return CBeamLinearRail(250.5, Rotation(-90, 0, 0), Align3(Align::MIN, Align::CENTER, Align::CENTER));
    });
    add_case<CBeamGantryPlate>("cbeam_gantry_plate", [] { return CBeamGantryPlate(); });
    compare::add("bdw_open_builds_cbeam_gantry_plate_xlarge", [] { return CBeamGantryPlateXLarge(); });
    add_value("bdw_open_builds_cbeam_gantry_plate_xlarge_values", [] {
        const CBeamGantryPlateXLarge plate;
        std::vector<V> out = shape_fields(plate);
        out.emplace_back(plate.thickness);
        return V::list(out);
    });
    add_case<CBeamGantryPlateXLarge>("cbeam_gantry_plate_xlarge_aligned",
                                     [] { return CBeamGantryPlateXLarge(Rotation(0, 0, 30), Align3(Align::MAX)); });
    add_case<CBeamRiserPlate>("cbeam_riser_plate", [] { return CBeamRiserPlate(); });
    add_case<EccentricSpacer>("eccentric_spacer_6mm", [] { return EccentricSpacer("6mm"); });
    add_case<EccentricSpacer>("eccentric_spacer_1_4in", [] { return EccentricSpacer("1/4in"); });
    add_case<EccentricSpacer>("eccentric_spacer_default", [] { return EccentricSpacer(); });
    add_case<FlexibleCoupler>("flexible_coupler_8mm", [] { return FlexibleCoupler("8mm"); });
    add_case<FlexibleCoupler>("flexible_coupler_1_4in",
                              [] { return FlexibleCoupler("1/4in", Rotation(0, 90, 0), Align3(Align::CENTER)); });
    add_case<LBracket>("lbracket", [] { return LBracket(); });
    add_case<LBracket>("lbracket_aligned", [] { return LBracket(Rotation(0, 0, 90), Align3(Align::CENTER)); });
    add_case<RouterSpindleMount>("router_spindle_mount", [] { return RouterSpindleMount(); });
    add_case<RouterSpindleMount>("router_spindle_mount_aligned",
                                 [] { return RouterSpindleMount("both", Rotation(0, 0, 180), Align3(Align::MIN)); });
    for (const auto& shim_type : kShimTypes) {
        add_case<ShimWasher>("shim_washer_" + key(shim_type), [shim_type] { return ShimWasher(shim_type); });
    }
    add_case<SpacerBlock>("spacer_block", [] { return SpacerBlock(); });
    add_case<TNut>("tnut", [] { return TNut(); });
    add_case<TNut>("tnut_aligned", [] { return TNut(Rotation(180, 0, 0), Align3(Align::MIN)); });
    for (const auto& size : kRailSizes) {
        add_case<VSlotLinearRailProfile>("vslot_profile_" + size, [size] { return VSlotLinearRailProfile(size); });
        add_case<VSlotLinearRail>("vslot_rail_" + size, [size] { return VSlotLinearRail(size, 50.0); });
    }
    add_case<VSlotLinearRailProfile>("vslot_profile_default", [] { return VSlotLinearRailProfile(); });
    add_case<VSlotLinearRailProfile>("vslot_profile_rotated",
                                     [] { return VSlotLinearRailProfile("40x40", 45, Align2(Align::MIN, Align::MIN)); });
    add_case<VSlotLinearRail>("vslot_rail_rotated",
                              [] { return VSlotLinearRail("20x40", 120.5, Rotation(0, 90, 0), std::nullopt); });
    add_case<XtremeSolidVWheel>("xtreme_solid_vwheel", [] { return XtremeSolidVWheel(); });
    add_case<XtremeSolidVWheel>("xtreme_solid_vwheel_align_none",
                                [] { return XtremeSolidVWheel(Rotation(), std::nullopt); });

    // ---- assemblies
    add_case<AcmeAntiBacklashNutBlock8mmAssembly>("acme_nut_block_assembly",
                                                  [] { return AcmeAntiBacklashNutBlock8mmAssembly(); }, assembly);
    add_case<CBeamAssembly>("cbeam_assembly", [] { return CBeamAssembly(300.0); }, assembly);
    add_case<CBeamAssembly>("cbeam_assembly_gantry", [] { return CBeamAssembly(300.0, 100.0); }, assembly);
    add_case<CBeamAssembly>("cbeam_assembly_no_end_plate", [] { return CBeamAssembly(300.0, {.end_plate = false}); },
                            assembly);
    add_case<CBeamAssembly>(
        "cbeam_assembly_screw_length", [] { return CBeamAssembly(250.0, {.screw_length = 200.0, .end_plate = false}); },
        assembly);
    add_case<CBeamAssembly>(
        "cbeam_assembly_thread", [] { return CBeamAssembly(120.0, {.gantry_position = 40.0, .simple = false}); },
        assembly);
    add_case<CBeamCapped>("cbeam_capped", [] { return CBeamCapped(200.0); }, assembly);
    add_case<CBeamEndAssembly>("cbeam_end_assembly", [] { return CBeamEndAssembly(); }, assembly);
    for (const std::string size : {"5mm", "8mm", "1/4in"}) {
        add_case<LockCollar>("lock_collar_" + key(size), [size] { return LockCollar(size); }, assembly);
    }
    add_case<MetricLeadScrew>("metric_lead_screw", [] { return MetricLeadScrew(100.0); }, assembly);
    add_case<MetricLeadScrew>("metric_lead_screw_thread", [] { return MetricLeadScrew(30.0, false); }, assembly);
    add_case<XLargeCBeamGantry>("xlarge_cbeam_gantry_4", [] { return XLargeCBeamGantry(4); }, assembly);
    add_case<XLargeCBeamGantry>("xlarge_cbeam_gantry_6", [] { return XLargeCBeamGantry(6); }, assembly);
    add_case<XLargeCBeamGantry>("xlarge_cbeam_gantry_4_60", [] { return XLargeCBeamGantry(4, 60.0); }, assembly);
    add_case<XLargeCBeamGantry>("xlarge_cbeam_gantry_default", [] { return XLargeCBeamGantry(); }, assembly);
    add_case<XtremeSolidVWheelAssembly>("xtreme_solid_vwheel_assembly_eccentric",
                                        [] { return XtremeSolidVWheelAssembly(true); }, assembly);
    add_case<XtremeSolidVWheelAssembly>("xtreme_solid_vwheel_assembly", [] { return XtremeSolidVWheelAssembly(false); },
                                        assembly);
    for (const auto& motor_type : kMotorTypes) {
        add_case<StepperMotor>("stepper_motor_" + motor_type, [motor_type] { return StepperMotor(motor_type); },
                               assembly);
    }
    add_case<StepperMotor>("stepper_motor_Nema17_30", [] { return StepperMotor("Nema17", 30.0); }, assembly);
    add_case<StepperMotor>("stepper_motor_Nema23_120", [] { return StepperMotor("Nema23", 120.5); }, assembly);
    add_case<StepperMotor>("stepper_motor_Nema23HighTorque_15", [] { return StepperMotor("Nema23HighTorque", 15.0); },
                           assembly);

    add_value("bdw_open_builds_module_values", [] {
        const SocketHeadCapScrew& m5 = open_builds::m5();
        return V::list({open_builds::CAVITY_RADIUS, open_builds::FILLET_RADIUS, m5.label, m5.volume(), m5.length});
    });
    add_value("bdw_open_builds_errors", [] {
        const std::vector<std::function<void()>> calls = {
            [] { AluminumSpacer("7mm"); },
            [] { EccentricSpacer("5mm"); },
            [] { FlexibleCoupler("5mm"); },
            [] { LockCollar("6mm"); },
            [] { ShimWasher("20x5x1"); },
            [] { VSlotLinearRailProfile("30x30"); },
            [] { VSlotLinearRail("20x30", 10.0); },
            [] { XLargeCBeamGantry(5); },
            [] { StepperMotor("Nema34"); },
            [] { StepperMotor("Nema17", 0.0); },
            [] { StepperMotor("Nema23", -5.5); },
            [] { RouterSpindleMount("base"); },
            [] { RouterSpindleMount("faceplate"); },
            [] { RouterSpindleMount("top"); },
        };
        std::vector<V> out;
        for (const auto& call : calls) out.push_back(error_of(call));
        return V::list(out);
    });
}
