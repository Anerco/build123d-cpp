// bd_warehouse's bushing.py, compared with cases_bdw_bushing_py.py.
#include <array>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

using Spec = std::array<double, 6>;

const std::vector<std::pair<std::string, Spec>> kSpecs = {
    {"5_8_6_2p5_13_0p75", {5, 8, 6, 2.5, 13, 0.75}},
    {"5_8_6_2p5_13_0", {5, 8, 6, 2.5, 13, 0}},
    {"8_16_10_4_19_2p5", {8, 16, 10, 4, 19, 2.5}},
    {"3_6_3_1p5_6_1", {3, 6, 3, 1.5, 6, 1}},
    {"12p7_25p4_20_6_32_4p5", {12.7, 25.4, 20, 6, 32, 4.5}},
};

HexFlangedEccentricBushing make(const Spec& s) { return HexFlangedEccentricBushing(s[0], s[1], s[2], s[3], s[4], s[5]); }

V values(const HexFlangedEccentricBushing& b) {
    return V::list({b.label, b.bore_diameter, b.body_diameter, b.body_length, b.flange_height, b.hex_size,
                    b.eccentricity, b.overall_length, b.material.get()->material.name, loc_values(b.location()),
                    loc_values(b.joints.get<RigidJoint>("outer_center").relative_location()),
                    loc_values(b.joints.get<RigidJoint>("ecc_top_center").relative_location())});
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    for (const auto& [name, spec] : kSpecs) {
        const Spec s = spec;
        add("bdw_bushing_" + name, [s] { return make(s); });
        add_value("bdw_bushing_" + name + "_values", [s] { return values(make(s)); });
    }
    add("bdw_bushing_rotated", [] { return HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, Rotation(90, 0, 45)); });
    add("bdw_bushing_aligned", [] {
        return HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, Rotation(), Align3(Align::MIN));
    });
    add_value("bdw_bushing_aligned_values", [] {
        return values(HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, Rotation(0, 0, 30),
                                                 Align3(Align::CENTER, Align::MAX, Align::NONE)));
    });
    add("bdw_bushing_align_none", [] {
        return HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, Rotation(), std::nullopt);
    });
    add_value("bdw_bushing_errors", [] {
        return V::list({error_of([] { HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, -0.5); }),
                        error_of([] { HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 1.5); }),
                        error_of([] { HexFlangedEccentricBushing(8, 8, 6, 2.5, 13, 0); }),
                        error_of([] { HexFlangedEccentricBushing(5, 8, 6, 2.5, 7.9, 0.5); })});
    });
}
