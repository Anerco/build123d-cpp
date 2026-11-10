// bd_warehouse's sprocket.py, compared with cases_bdw_sprocket_py.py.
#include <functional>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

V values(const Sprocket& s) {
    const Face plan = s.plan();
    return V::list({s.num_teeth, s.chain_pitch, s.roller_diameter, s.clearance, s.thickness, s.bolt_circle_diameter,
                    s.num_mount_bolts, s.mount_bolt_diameter, s.bore_diameter, s.pitch_radius(), s.outer_radius(),
                    s.pitch_circumference(), plan.area(), plan.edges().size()});
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    const std::vector<std::pair<std::string, std::function<Sprocket()>>> specs = {
        {"z3", [] { return Sprocket(3); }},
        {"z5", [] { return Sprocket(5); }},
        {"z9", [] { return Sprocket(9); }},
        {"z16", [] { return Sprocket(16); }},
        {"z32", [] { return Sprocket(32); }},
        {"z60", [] { return Sprocket(60); }},
        {"z20_clearance", [] { return Sprocket(20, {.clearance = 0.2}); }},
        {"z15_quarter_inch",
         [] { return Sprocket(15, {.chain_pitch = 0.25 * IN, .roller_diameter = 0.13 * IN, .thickness = 0.11 * IN}); }},
        {"z25_small_rollers", [] { return Sprocket(25, {.roller_diameter = 0.2 * IN}); }},
        {"z20_bolts_bore",
         [] {
             return Sprocket(20, {.bolt_circle_diameter = 40, .num_mount_bolts = 4, .mount_bolt_diameter = 5,
                                  .bore_diameter = 20});
         }},
        {"z24_bore", [] { return Sprocket(24, {.bore_diameter = 30}); }},
        {"z12_rotated", [] { return Sprocket(12, {.rotation = Rotation(90, 0, 0), .align = Align3(Align::MIN)}); }},
        {"z12_align_none", [] { return Sprocket(12, {.align = std::nullopt}); }},
    };
    for (const auto& [name, make] : specs) {
        add("bdw_sprocket_" + name, make);
        add_value("bdw_sprocket_" + name + "_values", [make = make] { return values(make()); });
    }
    for (const auto& [name, make] : specs) {
        if (name == "z5" || name == "z32" || name == "z20_clearance") {
            add("bdw_sprocket_" + name + "_plan", [make = make] { return make().plan(); });
        }
    }

    add_value("bdw_sprocket_statics", [] {
        std::vector<V> out;
        for (const int n : {3, 8, 32, 100}) {
            for (const double p : {12.7, 6.35, 25.4}) {
                out.push_back(V::list({Sprocket::sprocket_pitch_radius(n, p), Sprocket::sprocket_circumference(n, p)}));
            }
        }
        return V::list(out);
    });
    add_value("bdw_sprocket_errors", [] {
        return V::list({error_of([] { Sprocket(20, {.chain_pitch = 10, .roller_diameter = 10}); }),
                        error_of([] { Sprocket(20, {.chain_pitch = 10.0, .roller_diameter = 12.5}); }),
                        error_of([] { Sprocket(2); }), error_of([] { Sprocket(-4); })});
    });
}
