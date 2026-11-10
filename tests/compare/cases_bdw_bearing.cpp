// bd_warehouse's bearing.py, compared with cases_bdw_bearing_py.py.
#include <functional>
#include <memory>
#include <regex>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

std::string case_name(const std::vector<std::string>& parts) {
    std::string out = "bdw_bearing";
    for (const auto& p : parts) out += "_" + std::regex_replace(p, std::regex("[^A-Za-z0-9]+"), "_");
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out;
}

const std::vector<std::string> kMethods = {"cap", "cage", "roller", "_roller", "inner_race_section",
                                           "contact_angle", "bore_diameter", "nothing"};

struct BearingClass {
    std::string name;
    std::function<std::set<std::string>()> types;
    std::function<std::vector<std::string>(const std::string&)> sizes;
    std::function<std::unique_ptr<Bearing>(const std::string&, const std::string&)> make;
};

template <class C>
BearingClass bearing_class(const std::string& name) {
    return {name, [] { return C::types(); }, [](const std::string& t) { return C::sizes(t); },
            [](const std::string& s, const std::string& t) { return std::unique_ptr<Bearing>(new C(s, t)); }};
}

const std::vector<BearingClass>& classes() {
    static const std::vector<BearingClass> list = {
        bearing_class<SingleRowDeepGrooveBallBearing>("SingleRowDeepGrooveBallBearing"),
        bearing_class<SingleRowCappedDeepGrooveBallBearing>("SingleRowCappedDeepGrooveBallBearing"),
        bearing_class<SingleRowAngularContactBallBearing>("SingleRowAngularContactBallBearing"),
        bearing_class<SingleRowCylindricalRollerBearing>("SingleRowCylindricalRollerBearing"),
        bearing_class<SingleRowTaperedRollerBearing>("SingleRowTaperedRollerBearing"),
    };
    return list;
}

const BearingClass& class_named(const std::string& name) {
    for (const auto& c : classes()) {
        if (c.name == name) return c;
    }
    throw std::invalid_argument("unknown bearing class " + name);
}

V param_items(const ParamDict& d) {
    std::vector<V> out;
    for (const auto& [k, v] : d) out.push_back(V::list({k, num(v)}));
    return V::list(out);
}

V bearing_values(const Bearing& b) {
    std::vector<V> children;
    for (const auto& c : b.children) {
        children.push_back(V::list({c.label, c.volume(), c.material.get()->material.name, loc_values(c.location())}));
    }
    std::vector<V> methods;
    for (const auto& m : kMethods) methods.emplace_back(b.method_exists(m));
    std::vector<V> out{b.label,
                       b.info(),
                       b.bearing_class(),
                       b.bearing_size,
                       b.bearing_type,
                       b.capped,
                       b.is_metric,
                       b.bore_diameter(),
                       b.outer_diameter(),
                       b.thickness(),
                       b.roller_count(),
                       b.roller_diameter(),
                       b.race_center_radius(),
                       b.length_offset(),
                       V::list({}),
                       param_items(b.bearing_dict),
                       param_items(b.clearance_hole_diameters()),
                       loc_values(b.location()),
                       loc_values(b.joints.get<RigidJoint>("a").relative_location()),
                       loc_values(b.joints.get<RigidJoint>("b").relative_location()),
                       V::list(children),
                       V::list(methods)};
    if (const auto* a = dynamic_cast<const SingleRowAngularContactBallBearing*>(&b)) out.emplace_back(a->contact_angle());
    if (const auto* t = dynamic_cast<const SingleRowTaperedRollerBearing*>(&b)) {
        out.emplace_back(t->contact_angle());
        out.emplace_back(t->taper_length);
        out.emplace_back(t->cage_edge.position_at(0));
        out.emplace_back(t->cage_edge.position_at(1));
    }
    return V::list(out);
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    // ---- every size of every type of every bearing
    for (const auto& c : classes()) {
        for (const auto& bearing_type : c.types()) {
            for (const auto& size : c.sizes(bearing_type)) {
                const auto make = c.make;
                add(case_name({c.name, bearing_type, size}), [make, size, bearing_type] {
                    return Shape(*make(size, bearing_type));
                });
                add_value(case_name({c.name, bearing_type, size, "values"}),
                          [make, size, bearing_type] { return bearing_values(*make(size, bearing_type)); });
            }
        }
    }

    // ---- the parts of a few bearings
    const std::vector<std::pair<std::string, std::string>> part_specs = {
        {"SingleRowDeepGrooveBallBearing", "M8-22-7"},     {"SingleRowCappedDeepGrooveBallBearing", "M8-22-7"},
        {"SingleRowAngularContactBallBearing", "M10-30-9"}, {"SingleRowAngularContactBallBearing", "M25-62-17"},
        {"SingleRowCylindricalRollerBearing", "M15-35-11"}, {"SingleRowTaperedRollerBearing", "M25-52-19.25"},
        {"SingleRowTaperedRollerBearing", "M17-47-20.25"},
    };
    for (const auto& [cls_name, size] : part_specs) {
        const auto make = class_named(cls_name).make;
        const std::string s = size;
        add(case_name({cls_name, size, "inner_race_section"}), [make, s] { return make(s, "SKT")->inner_race_section(); });
        add(case_name({cls_name, size, "outer_race_section"}), [make, s] { return make(s, "SKT")->outer_race_section(); });
        add(case_name({cls_name, size, "roller"}), [make, s] { return make(s, "SKT")->roller(); });
        add(case_name({cls_name, size, "countersink_profile"}), [make, s] { return make(s, "SKT")->countersink_profile(); });
        add(case_name({cls_name, size, "countersink_profile_interference"}),
            [make, s] { return make(s, "SKT")->countersink_profile(0.05); });
        add(case_name({cls_name, size, "make_bearing"}), [make, s] { return make(s, "SKT")->make_bearing(); });
        add_value(case_name({cls_name, size, "default_parts"}), [make, s] {
            const auto b = make(s, "SKT");
            return V::list({b->default_race_center_radius(), b->default_roller_diameter(),
                            b->default_inner_race_section().area(), b->default_outer_race_section().area(),
                            b->default_countersink_profile(0.1).area(), b->default_roller().volume(),
                            b->default_cap().volume()});
        });
        if (cls_name == "SingleRowAngularContactBallBearing" || cls_name == "SingleRowTaperedRollerBearing") {
            add(case_name({cls_name, size, "cage"}), [make, s] { return *make(s, "SKT")->cage(); });
        }
        if (cls_name == "SingleRowCappedDeepGrooveBallBearing") {
            add(case_name({cls_name, size, "cap"}), [make, s] { return *make(s, "SKT")->cap(); });
        }
    }

    // ---- PressFitHole
    add("bdw_bearing_press_fit_hole", [] { return PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7")); });
    add("bdw_bearing_press_fit_hole_interference", [] {
        return PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7"), 0.05, "Close", 20);
    });
    add("bdw_bearing_press_fit_hole_loose", [] {
        return PressFitHole(SingleRowAngularContactBallBearing("M12-32-10"), 0, "Loose", 5);
    });
    add("bdw_bearing_press_fit_hole_tapered", [] { return PressFitHole(SingleRowTaperedRollerBearing("M25-52-19.25")); });
    add("bdw_bearing_press_fit_hole_capped", [] {
        return PressFitHole(SingleRowCappedDeepGrooveBallBearing("M5-10-4", "OpenBuilds"), 0.02);
    });
    add_value("bdw_bearing_press_fit_hole_depth", [] {
        return V::list({PressFitHole(SingleRowTaperedRollerBearing("M25-52-19.25")).hole_depth,
                        PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7"), 0, "Normal", 12.5).hole_depth});
    });
    add_value("bdw_bearing_press_fit_hole_bad_fit", [] {
        return error_of([] { PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7"), 0, "Snug"); });
    });

    // ---- class functions
    add_value("bdw_bearing_types", [] {
        std::vector<V> out;
        for (const auto& c : classes()) out.push_back(V::list({c.name, V::list_of(c.types())}));
        return V::list(out);
    });
    add_value("bdw_bearing_sizes", [] {
        std::vector<V> out;
        for (const auto& c : classes()) {
            for (const auto& t : c.types()) out.push_back(V::list({c.name, t, V::list_of(c.sizes(t))}));
        }
        return V::list(out);
    });
    add_value("bdw_bearing_select_by_size", [] {
        std::vector<V> out;
        for (const char* size : {"M8-22-7", "M5-10-4", "M17-47-14", "M25-52-15", "M25-52-19.25", "M99-1-1"}) {
            std::vector<V> entries;
            for (const auto& [cls, types] : Bearing::select_by_size(size)) entries.push_back(V::list({cls, V::list_of(types)}));
            out.push_back(V::list(entries));
        }
        return V::list(out);
    });
    add_value("bdw_bearing_clearance_tables", [] {
        std::vector<V> drill;
        for (const auto& [k, v] : Bearing::clearance_hole_drill_sizes().at("M8")) drill.push_back(V::list({k, v}));
        return V::list({Bearing::clearance_hole_drill_sizes().size(), V::list(drill),
                        param_items(Bearing::clearance_hole_data().at("M8"))});
    });
    add_value("bdw_bearing_method_exists_fastener", [] {
        return V::list({method_exists("HexNut", "nut_profile"), method_exists("HexNut", "head_profile"),
                        method_exists("SetScrew", "head_profile"), method_exists("SetScrew", "custom_make"),
                        method_exists("SocketHeadCapScrew", "head_recess"),
                        method_exists("SocketHeadCapScrew", "flange_profile"),
                        method_exists("CounterSunkScrew", "countersink_profile"),
                        method_exists("HeatSetNut", "knurled_cylinder_faces"),
                        method_exists("PlainWasher", "washer_profile"), method_exists("PlainWasher", "nut_profile"),
                        method_exists("SingleRowCappedDeepGrooveBallBearing", "cap")});
    });

    // ---- errors
    add_value("bdw_bearing_bad_type", [] { return error_of([] { SingleRowDeepGrooveBallBearing("M8-22-7", "ABC"); }); });
    add_value("bdw_bearing_bad_size", [] { return error_of([] { SingleRowAngularContactBallBearing("M8-22-7"); }); });
    add_value("bdw_bearing_empty_size", [] { return error_of([] { SingleRowDeepGrooveBallBearing("  "); }); });
    add_value("bdw_bearing_size_stripped", [] { return V(SingleRowDeepGrooveBallBearing(" M8-22-7 ").label); });
    add_value("bdw_bearing_tapered_fillet_fails", [] {
        return error_kind([] { SingleRowTaperedRollerBearing("M32-53-14.5"); });
    });
}
