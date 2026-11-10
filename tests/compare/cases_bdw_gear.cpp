// bd_warehouse's gear.py, compared with cases_bdw_gear_py.py.
#include <cmath>
#include <functional>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

V finite(double x) { return std::isinf(x) ? V() : V(x); }
V opt_num(const std::optional<double>& x) { return x ? V(*x) : V(); }
V material_name(const Shape& s) {
    const auto& m = s.material.get();
    return m ? V(m->material.name) : V();
}

std::vector<std::pair<std::string, V>> tooth_attrs(const InvoluteToothProfile& t) {
    return {{"module", t.module},
            {"tooth_count", t.tooth_count},
            {"pitch_radius", t.pitch_radius},
            {"base_radius", t.base_radius},
            {"addendum", t.addendum},
            {"addendum_radius", t.addendum_radius},
            {"dedendum", t.dedendum},
            {"root_radius", t.root_radius}};
}

template <class R>
std::vector<std::pair<std::string, V>> rack_attrs(const R& r) {
    return {{"normal_module", r.normal_module},
            {"transverse_module", r.transverse_module},
            {"normal_pressure_angle", r.normal_pressure_angle},
            {"transverse_pressure_angle", r.transverse_pressure_angle},
            {"module", r.module},
            {"module_system", r.module_system},
            {"tooth_count", r.tooth_count},
            {"pressure_angle", r.pressure_angle},
            {"helix_angle", r.helix_angle},
            {"addendum", r.addendum},
            {"dedendum", r.dedendum},
            {"root_fillet", opt_num(r.root_fillet)},
            {"base_height", r.base_height},
            {"pitch", r.pitch},
            {"pitch_length", r.pitch_length},
            {"tooth_height", r.tooth_height}};
}

template <class S>
std::vector<std::pair<std::string, V>> spur_attrs(const S& s) {
    return {{"pitch_radius", s.pitch_radius},
            {"base_radius", s.base_radius},
            {"addendum_radius", s.addendum_radius},
            {"root_radius", s.root_radius}};
}

std::vector<std::pair<std::string, V>> helical_attrs(const HelicalGear& h) {
    return {{"normal_module", h.normal_module},
            {"transverse_module", h.transverse_module},
            {"normal_pressure_angle", h.normal_pressure_angle},
            {"transverse_pressure_angle", h.transverse_pressure_angle},
            {"module", h.module},
            {"module_system", h.module_system},
            {"tooth_count", h.tooth_count},
            {"pressure_angle", h.pressure_angle},
            {"helix_angle", h.helix_angle},
            {"thickness", h.thickness},
            {"pitch_radius", h.pitch_radius},
            {"base_radius", h.base_radius},
            {"addendum_radius", h.addendum_radius},
            {"root_radius", h.root_radius},
            {"twist_angle", h.twist_angle},
            {"lead", finite(h.lead)}};
}

std::vector<std::pair<std::string, V>> worm_attrs(const WormDimensions& w) {
    return {{"module", w.module},
            {"starts", w.starts},
            {"diametral_quotient", w.diametral_quotient},
            {"flank_form", w.flank_form},
            {"normal_pressure_angle", w.normal_pressure_angle},
            {"axial_pressure_angle", w.axial_pressure_angle},
            {"transverse_pressure_angle", opt_num(w.transverse_pressure_angle)},
            {"addendum", w.addendum},
            {"dedendum", w.dedendum},
            {"pitch_radius", w.pitch_radius},
            {"addendum_radius", w.addendum_radius},
            {"root_radius", w.root_radius},
            {"base_radius", opt_num(w.base_radius)},
            {"axial_pitch", w.axial_pitch},
            {"lead", w.lead},
            {"lead_angle", w.lead_angle},
            {"base_lead_angle", opt_num(w.base_lead_angle)}};
}

V pairs(std::vector<std::pair<std::string, V>> fields, std::vector<V> extra = {}) {
    std::vector<V> out;
    for (auto& [name, value] : fields) out.push_back(V::list({name, value}));
    for (auto& e : extra) out.push_back(e);
    return V::list(out);
}

template <class T>
using Specs = std::vector<std::pair<std::string, std::function<T()>>>;

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    const Specs<InvoluteToothProfile> tooth_specs = {
        {"m2_z12_14p5_fillet", [] { return InvoluteToothProfile(2, 12, 14.5, 0.5); }},
        {"m2_z12_20", [] { return InvoluteToothProfile(2, 12, 20); }},
        {"m1_z20_20", [] { return InvoluteToothProfile(1, 20, 20, {.root_fillet = 0.2}); }},
        {"m1_z40_20_fillet_fails", [] { return InvoluteToothProfile(1, 40, 20, {.root_fillet = 0.2}); }},
        {"m3_z8_25_add", [] { return InvoluteToothProfile(3, 8, 25, {.addendum = 2.5, .dedendum = 3.5}); }},
        {"m0p5_z100_20", [] { return InvoluteToothProfile(0.5, 100, 20); }},
    };
    for (const auto& [name, make] : tooth_specs) {
        add("bdw_gear_tooth_" + name, make);
        add_value("bdw_gear_tooth_" + name + "_values", [make = make] { return pairs(tooth_attrs(make())); });
    }

    const Specs<RackGearPlan> rack_plan_specs = {
        {"m2_z10_20", [] { return RackGearPlan(2, 10, 20); }},
        {"m2_z10_20_fillet", [] { return RackGearPlan(2, 10, 20, {.root_fillet = 0.3}); }},
        {"m1p5_z6_14p5_helix", [] { return RackGearPlan(1.5, 6, 14.5, {.helix_angle = 20, .base_height = 5}); }},
        {"m2_z5_20_transverse",
         [] { return RackGearPlan(2, 5, 20, {.helix_angle = -15, .module_system = "transverse", .addendum = 1.5}); }},
        {"m2_z4_20_rotated", [] { return RackGearPlan(2, 4, 20, {.rotation = 30, .align = Align2(Align::CENTER)}); }},
    };
    for (const auto& [name, make] : rack_plan_specs) {
        add("bdw_gear_rack_plan_" + name, make);
        add_value("bdw_gear_rack_plan_" + name + "_values", [make = make] { return pairs(rack_attrs(make())); });
    }

    const Specs<RackGear> rack_specs = {
        {"m2_z10_20_t8", [] { return RackGear(2, 10, 20, 8); }},
        {"m2_z10_20_t8_helix", [] { return RackGear(2, 10, 20, 8, {.helix_angle = 15}); }},
        {"m2_z10_20_t8_helix_neg",
         [] { return RackGear(2, 10, 20, 8, {.helix_angle = -25, .module_system = "transverse"}); }},
        {"m1_z20_20_t5_fillet", [] { return RackGear(1, 20, 20, 5, {.root_fillet = 0.25, .dedendum = 1.4}); }},
        {"m2_z6_20_t6_rotated",
         [] { return RackGear(2, 6, 20, 6, {.rotation = Rotation(0, 0, 90), .align = Align3(Align::MIN)}); }},
    };
    for (const auto& [name, make] : rack_specs) {
        add("bdw_gear_rack_" + name, make);
        add_value("bdw_gear_rack_" + name + "_values", [make = make] {
            const RackGear r = make();
            auto fields = rack_attrs(r);
            fields.push_back({"thickness", r.thickness});
            fields.push_back({"lateral_offset", r.lateral_offset});
            return pairs(fields);
        });
    }

    const Specs<SpurGear> spur_specs = {
        {"m2_z12_14p5_t5_fillet", [] { return SpurGear(2, 12, 14.5, 5, 0.5); }},
        {"m1p5_z31_20_t4", [] { return SpurGear(1.5, 31, 20, 4); }},
        {"m1_z20_20_t3_addendum", [] { return SpurGear(1, 20, 20, 3, {.addendum = 0.8, .dedendum = 1.3}); }},
        {"m2_z15_20_t6_rotated",
         [] {
             return SpurGear(2, 15, 20, 6,
                             {.rotation = Rotation(90, 0, 0), .align = Align3(Align::MIN, Align::CENTER, Align::MAX)});
         }},
    };
    for (const auto& [name, make] : spur_specs) {
        add("bdw_gear_spur_" + name, make);
        add_value("bdw_gear_spur_" + name + "_values", [make = make] {
            const SpurGear s = make();
            return pairs(spur_attrs(s), {material_name(s)});
        });
    }

    const Specs<HelicalGear> helical_specs = {
        {"m2_z13_20_h45_t10", [] { return HelicalGear(2, 13, 20, 45, 10); }},
        {"m2_z20_20_hm20_t8_transverse", [] { return HelicalGear(2, 20, 20, -20, 8, {.module_system = "transverse"}); }},
        {"m1_z30_20_h0_t5", [] { return HelicalGear(1, 30, 20, 0, 5); }},
        {"m1p5_z18_20_h15_t6_fillet", [] { return HelicalGear(1.5, 18, 20, 15, 6, {.root_fillet = 0.3, .addendum = 1.2}); }},
        {"m2_z12_20_h30_t8_aligned", [] { return HelicalGear(2, 12, 20, 30, 8, {.align = Align3(Align::MIN)}); }},
    };
    for (const auto& [name, make] : helical_specs) {
        add("bdw_gear_helical_" + name, make);
        add_value("bdw_gear_helical_" + name + "_values", [make = make] {
            const HelicalGear h = make();
            return pairs(helical_attrs(h), {material_name(h)});
        });
    }

    const Specs<WormFlank> flank_specs = {
        {"zi_lower", [] { return WormFlank(2, 2, 10, 30); }},
        {"zi_upper_left", [] { return WormFlank(2, 1, 10, 30, {.hand = "left", .side = "upper"}); }},
        {"za_lower", [] { return WormFlank(2, 1, 10, 30, {.flank_form = "ZA"}); }},
        {"za_upper", [] { return WormFlank(1.5, 3, 12, 20, {.pressure_angle = 15, .flank_form = "ZA", .side = "upper"}); }},
        {"zi_low_q", [] { return WormFlank(2, 1, 6, 25, {.pressure_angle = 25}); }},
        {"zi_addendum", [] { return WormFlank(3, 2, 9, 40, {.addendum = 2.5, .dedendum = 3.0}); }},
    };
    for (const auto& [name, make] : flank_specs) {
        add("bdw_gear_flank_" + name, make);
        add_value("bdw_gear_flank_" + name + "_values", [make = make] {
            const WormFlank f = make();
            auto fields = worm_attrs(f);
            fields.push_back({"axial_length", f.axial_length});
            fields.push_back({"hand", f.hand});
            fields.push_back({"side", f.side});
            fields.push_back({"inner_radius", f.inner_radius});
            return pairs(fields);
        });
    }

    const Specs<Worm> worm_specs = {
        {"zi_m2_s2_q10_l30", [] { return Worm(2, 2, 10, 30); }},
        {"za_m2_s1_q10_l30", [] { return Worm(2, 1, 10, 30, {.flank_form = "ZA"}); }},
        {"zi_m2_s3_q12_l40_left", [] { return Worm(2, 3, 12, 40, {.hand = "left"}); }},
        {"zi_m1_s1_q8_l15", [] { return Worm(1, 1, 8, 15); }},
        {"zi_m1p5_s4_q10_l25_pa25", [] { return Worm(1.5, 4, 10, 25, {.pressure_angle = 25}); }},
        {"za_m2_s2_q11_l12_left_rotated",
         [] {
             return Worm(2, 2, 11, 12,
                         {.flank_form = "ZA", .hand = "left", .rotation = Rotation(90, 0, 0), .align = Align3(Align::MAX)});
         }},
    };
    for (const auto& [name, make] : worm_specs) {
        add("bdw_gear_worm_" + name, make);
        add_value("bdw_gear_worm_" + name + "_values", [make = make] {
            const Worm w = make();
            auto fields = worm_attrs(w);
            fields.push_back({"length", w.length});
            fields.push_back({"hand", w.hand});
            return pairs(fields, {material_name(w)});
        });
    }

    const Specs<WormWheel> wheel_specs = {
        {"m2_z30_t12_s2", [] { return WormWheel(2, 30, 12, 2); }},
        {"m2_z40_t10", [] { return WormWheel(2, 40, 10); }},
        {"m1p5_z25_t8_left",
         [] { return WormWheel(1.5, 25, 8, {.starts = 3, .diametral_quotient = 12, .hand = "left", .root_fillet = 0.2}); }},
    };
    for (const auto& [name, make] : wheel_specs) {
        add("bdw_gear_wheel_" + name, make);
        add_value("bdw_gear_wheel_" + name + "_values", [make = make] {
            const WormWheel w = make();
            auto fields = helical_attrs(w);
            fields.push_back({"starts", w.starts});
            fields.push_back({"diametral_quotient", w.diametral_quotient});
            fields.push_back({"hand", w.hand});
            fields.push_back({"lead_angle", w.lead_angle});
            fields.push_back({"axial_pressure_angle", w.axial_pressure_angle});
            fields.push_back({"center_distance", w.center_distance});
            return pairs(fields, {material_name(w)});
        });
    }

    const Specs<WormGear> gear_specs = {
        {"m2_s2_q10_z30", [] { return WormGear(2, 2, 10, 30, 30, 12); }},
        {"m1p5_s1_q8_z20_za", [] { return WormGear(1.5, 1, 8, 20, 20, 8, {.flank_form = "ZA"}); }},
        {"m2_s3_q12_z36_left", [] { return WormGear(2, 3, 12, 36, 40, 10, {.hand = "left", .root_fillet = 0.1}); }},
    };
    for (const auto& [name, make] : gear_specs) {
        add("bdw_gear_worm_gear_" + name, make);
        add_value("bdw_gear_worm_gear_" + name + "_values", [make = make] {
            const WormGear g = make();
            std::vector<V> children, joints;
            for (const auto& c : g.children) {
                children.push_back(V::list({c.label, c.volume(), material_name(c), loc_values(c.location())}));
            }
            for (const auto& [n, j] : g.joints.items()) {
                joints.push_back(V::list({n, loc_values(j.as<RigidJoint>().relative_location())}));
            }
            return V::list({g.center_distance, g.ratio, V::list(children), V::list(joints)});
        });
    }

    add("bdw_gear_spur_plan", [] { return SpurGearPlan(2, 12, 14.5, 0.5); });
    add("bdw_gear_spur_plan_rotated", [] { return SpurGearPlan(1, 25, 20, {.rotation = 10, .align = Align2(Align::MIN)}); });
    add_value("bdw_gear_spur_plan_values", [] { return pairs(spur_attrs(SpurGearPlan(1, 25, 20))); });

    add_value("bdw_gear_errors", [] {
        const std::vector<std::function<void()>> calls = {
            [] { InvoluteToothProfile(2, 12, 20, {.closed = true}); },
            [] { RackGearPlan(0, 10, 20); },
            [] { RackGearPlan(2, 0, 20); },
            [] { RackGearPlan(2, 10, 90); },
            [] { RackGearPlan(2, 10, 20, {.helix_angle = 90}); },
            [] { RackGearPlan(2, 10, 20, {.module_system = "axial"}); },
            [] { RackGearPlan(2, 10, 20, {.root_fillet = -1}); },
            [] { RackGearPlan(2, 10, 20, {.addendum = 0}); },
            [] { RackGearPlan(2, 10, 20, {.dedendum = -1}); },
            [] { RackGearPlan(2, 10, 20, {.base_height = 0}); },
            [] { RackGearPlan(2, 10, 45, {.addendum = 3}); },
            [] { RackGearPlan(2, 10, 45, {.dedendum = 3}); },
            [] { RackGearPlan(2, 10, 20, {.root_fillet = 3}); },
            [] { RackGear(2, 10, 20, 0); },
            [] { HelicalGear(0, 13, 20, 45, 10); },
            [] { HelicalGear(2, 0, 20, 45, 10); },
            [] { HelicalGear(2, 13, 20, 45, 0); },
            [] { HelicalGear(2, 13, -1, 45, 10); },
            [] { HelicalGear(2, 13, 20, -90, 10); },
            [] { HelicalGear(2, 13, 20, 45, 10, {.module_system = "axial"}); },
            [] { WormFlank(2, 1, 10, 0); },
            [] { WormFlank(2, 1, 10, 30, {.hand = "up"}); },
            [] { WormFlank(2, 1, 10, 30, {.side = "middle"}); },
            [] { WormFlank(0, 1, 10, 30); },
            [] { WormFlank(2, 0, 10, 30); },
            [] { WormFlank(2, 1, 0, 30); },
            [] { WormFlank(2, 1, 10, 30, {.pressure_angle = 0}); },
            [] { WormFlank(2, 1, 10, 30, {.flank_form = "ZK"}); },
            [] { WormFlank(2, 1, 10, 30, {.addendum = 0}); },
            [] { WormFlank(2, 1, 10, 30, {.dedendum = -2}); },
            [] { WormFlank(2, 1, 2, 30); },
            [] { WormFlank(2, 1, 10, 30, {.pressure_angle = 60, .flank_form = "ZA"}); },
            [] { Worm(2, 1, 10, -5); },
            [] { Worm(2, 1, 10, 30, {.hand = "up"}); },
            [] { WormWheel(2, 30, 12, {.hand = "up"}); },
            [] { WormGear(2, 1, 10, 30, 30, 12, {.flank_form = "ZK"}); },
        };
        // (the message of a failed fillet is build123d's: only its kind is compared)
        std::vector<V> out{error_kind([] { InvoluteToothProfile(2, 12, 20, {.root_fillet = 5}); })};
        for (const auto& call : calls) out.push_back(error_of(call));
        return V::list(out);
    });
}
