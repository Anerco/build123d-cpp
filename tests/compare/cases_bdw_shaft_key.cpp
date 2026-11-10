// bd_warehouse's shaft_key.py, compared with cases_bdw_shaft_key_py.py.
#include <limits>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

double key_length(int d) { return 2.5 * d; }

V param_items(const ParamDict& d) {
    std::vector<V> out;
    for (const auto& [k, v] : d) out.push_back(V::list({k, num(v)}));
    return V::list(out);
}

V key_values(const ShaftKey& k) {
    const auto rgba = k.color.get()->to_tuple();
    return V::list({k.label, k.info(), k.key_form, k.shaft_diameter, param_items(k.key_dict), k.length, k.width,
                    k.key_height, k.plan_area(), V::list({rgba[0], rgba[1], rgba[2], rgba[3]}),
                    loc_values(k.joints.get<RigidJoint>("a").relative_location())});
}

V keyway_values(const Keyway& w) {
    return V::list({w.label, w.keyway_type, w.fit, w.nominal_width, w.length, w.keyway_depth,
                    w.keyway_depth_tolerance, w.width_tolerance, w.min_width, w.max_width, w.width, w.key.label});
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    for (const int d : ShaftKey::sizes()) {
        for (const std::string form : {"A", "B"}) {
            const std::string name = "bdw_shaft_key_" + std::to_string(d) + "_" + form;
            add(name, [d, form] { return ShaftKey(d, key_length(d), form); });
            add_value(name + "_values", [d, form] { return key_values(ShaftKey(d, key_length(d), form)); });
        }
    }
    for (const int d : {6, 10, 12, 22, 30, 50}) {
        for (const std::string keyway_type : {"shaft", "hub"}) {
            for (const std::string fit : {"Tight", "Loose", "Sliding"}) {
                for (const std::string form : {"A", "B"}) {
                    const std::string name =
                        "bdw_shaft_key_keyway_" + std::to_string(d) + "_" + keyway_type + "_" + fit + "_" + form;
                    add(name, [=] { return Keyway(ShaftKey(d, key_length(d), form), keyway_type, fit); });
                    add_value(name + "_values", [=] {
                        return keyway_values(Keyway(ShaftKey(d, key_length(d), form), keyway_type, fit));
                    });
                }
            }
        }
        add("bdw_shaft_key_broach_" + std::to_string(d),
            [d] { return Keyway(ShaftKey(d, key_length(d)), "hub").broach_profile(); });
    }

    add_value("bdw_shaft_key_sizes", [] { return V::list_of(ShaftKey::sizes()); });
    add_value("bdw_shaft_key_key_data", [] {
        std::vector<V> out;
        for (const auto& [k, row] : ShaftKey::key_data()) out.push_back(V::list({k, param_items(row)}));
        return V::list(out);
    });
    add_value("bdw_shaft_key_parameters", [] { return param_items(ShaftKey::parameters(12.0)); });
    add_value("bdw_shaft_key_select_by_size", [] {
        std::vector<V> out;
        for (const double d : {12.0, 19.0, 50.0, 51.0}) {
            std::vector<V> entries;
            for (const auto& [cls, standards] : ShaftKey::select_by_size(d)) {
                entries.push_back(V::list({cls, V::list_of(standards)}));
            }
            out.push_back(V::list(entries));
        }
        return V::list(out);
    });
    add_value("bdw_shaft_key_keyway_fits", [] {
        std::vector<V> out;
        for (const auto& [k, row] : Keyway::keyway_fits()) {
            std::vector<V> items;
            for (const auto& [n, v] : row) items.push_back(V::list({n, v}));
            out.push_back(V::list({k, V::list(items)}));
        }
        return V::list(out);
    });
    add_value("bdw_shaft_key_fractional_length", [] { return key_values(ShaftKey(20, 25.5, "B")); });
    add("bdw_shaft_key_rotated", [] { return ShaftKey(20, 30, "A", Rotation(0, 0, 90), Align3(Align::MIN)); });
    add("bdw_shaft_key_keyway_rotated", [] {
        return Keyway(ShaftKey(20, 30), "hub", "Tight", Rotation(90, 0, 0),
                      Align3(Align::CENTER, Align::MIN, Align::MAX));
    });
    add_value("bdw_shaft_key_errors", [] {
        const double inf = std::numeric_limits<double>::infinity();
        return V::list({error_of([] { ShaftKey(12, 20, "C"); }), error_of([] { ShaftKey(12.5, 20); }),
                        error_of([inf] { ShaftKey(inf, 20); }), error_of([] { ShaftKey(0, 20); }),
                        error_of([] { ShaftKey(-6, 20); }), error_of([] { ShaftKey(19, 20); }),
                        error_of([] { ShaftKey(12, 0); }), error_of([] { ShaftKey(12, 3.5); }),
                        error_of([] { ShaftKey::parameters(51); }), error_of([] { ShaftKey::select_by_size(1.5); }),
                        error_of([] { Keyway(ShaftKey(12, 20), "bore"); }),
                        error_of([] { Keyway(ShaftKey(12, 20), "hub", "Press"); }),
                        error_of([] { Keyway(ShaftKey(12, 20), "shaft").broach_profile(); })});
    });
}
