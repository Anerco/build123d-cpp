// bd_warehouse's o_rings.py, compared with cases_bdw_o_rings_py.py.
#include <cmath>
#include <limits>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

const std::vector<std::string> kShapeSizes = {"001", "025", "116", "214", "325", "381", "425", "475"};
const std::vector<double> kWidths = {1.02, 1.27, 1.52, 1.78, 2.62, 3.53, 5.33, 6.99};

V opt_number(const std::optional<double>& x) { return x ? V(*x) : V(); }

V ring_values(const ORing& r) {
    const auto rgba = r.color.get()->to_tuple();
    return V::list({r.label,
                    r.info(),
                    r.o_ring_size,
                    r.o_ring_type,
                    r.dash_number,
                    r.inner_diameter,
                    r.inner_diameter_tolerance,
                    r.width,
                    r.width_tolerance,
                    r.major_radius,
                    r.minor_radius,
                    r.length,
                    opt_number(r.gland_width),
                    r.gland_width_tol,
                    opt_number(r.gland_depth),
                    opt_number(r.gland_depth_tol),
                    opt_number(r.gland_radius),
                    opt_number(r.gland_radius_tol),
                    !r.gland_profile.has_value(),
                    V::list({rgba[0], rgba[1], rgba[2], rgba[3]}),
                    loc_values(r.joints.get<RigidJoint>("a").relative_location())});
}

V gland_values(const ORing& r) {
    return V::list({opt_number(r.gland_width), r.gland_width_tol, opt_number(r.gland_depth),
                    opt_number(r.gland_depth_tol), opt_number(r.gland_radius), opt_number(r.gland_radius_tol)});
}

V neighbors(const ORingNeighbors& result) {
    std::vector<V> out;
    for (const auto& [width, codes] : result) {
        std::vector<V> items;
        for (const auto& code : codes) items.push_back(code ? V(*code) : V());
        out.push_back(V::list({width, V::list(items)}));
    }
    return V::list(out);
}

V number_table(const NumberTable& table) {
    std::vector<V> rows;
    for (const auto& [key, row] : table) {
        std::vector<V> fields;
        for (const auto& [name, value] : row) fields.push_back(V::list({name, opt_number(value)}));
        rows.push_back(V::list({key, V::list(fields)}));
    }
    return V::list(rows);
}

V param_items(const ParamDict& d) {
    std::vector<V> out;
    for (const auto& [k, v] : d) out.push_back(V::list({k, num(v)}));
    return V::list(out);
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    add_value("bdw_o_rings_all_sizes", [] {
        std::vector<V> out;
        for (const auto& s : ORing::sizes()) out.push_back(ring_values(ORing(s)));
        return V::list(out);
    });
    add_value("bdw_o_rings_types", [] { return V::list_of(ORing::types()); });
    add_value("bdw_o_rings_sizes", [] { return V::list_of(ORing::sizes()); });
    add_value("bdw_o_rings_parameters", [] { return param_items(ORing::parameters("-25")); });
    add_value("bdw_o_rings_o_ring_parameters", [] {
        std::vector<V> out;
        for (const auto& [t, table] : ORing::o_ring_parameters()) {
            std::vector<V> rows;
            for (const auto& [s, p] : table) rows.push_back(V::list({s, param_items(p)}));
            out.push_back(V::list({t, V::list(rows)}));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_width_families", [] {
        std::vector<V> out;
        for (const auto& [t, families] : ORing::o_ring_width_families()) {
            std::vector<V> fam;
            for (const auto& [w, entries] : families) {
                std::vector<V> items;
                for (const auto& [id, code] : entries) items.push_back(V::list({id, code}));
                fam.push_back(V::list({w, V::list(items)}));
            }
            out.push_back(V::list({t, V::list(fam)}));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_o_ring_data", [] {
        std::vector<V> rows;
        for (const auto& [k, row] : ORing::o_ring_data()) {
            std::vector<V> fields;
            for (const auto& [k2, v2] : row) fields.push_back(V::list({k2, v2}));
            rows.push_back(V::list({k, V::list(fields)}));
        }
        return V::list(rows);
    });
    add_value("bdw_o_rings_static_gland_data", [] { return number_table(ORing::o_ring_static_gland_data()); });
    add_value("bdw_o_rings_dynamic_gland_data", [] { return number_table(ORing::o_ring_dynamic_gland_data()); });
    add_value("bdw_o_rings_read_parameters_from_csv", [] {
        return number_table(read_parameters_from_csv("o-ring_dynamic_gland_parameters.csv"));
    });
    add_value("bdw_o_rings_normalize_size", [] {
        std::vector<V> out;
        for (const char* s : {"1", "-25", " 025 ", "-7", "475", "000"}) out.emplace_back(ORing::_normalize_size(s));
        return V::list(out);
    });
    add_value("bdw_o_rings_normalize_size_errors", [] {
        std::vector<V> out;
        for (const char* s : {"", "1234", "--5", "2a", "-", "+5", "1.5"}) {
            out.push_back(error_of([s] { ORing::_normalize_size(s); }));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_select_by_size", [] {
        std::vector<V> out;
        for (const char* s : {"25", "-001", "475", "999"}) {
            std::vector<V> entries;
            for (const auto& [cls, types] : ORing::select_by_size(s)) entries.push_back(V::list({cls, V::list_of(types)}));
            out.push_back(V::list(entries));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_select_by_inner_diameter", [] {
        std::vector<V> out;
        for (const double d : {650.0, 20.0, 1.07, 24.99, 100.0, 2.9, 500.0}) {
            out.push_back(neighbors(ORing::select_by_inner_diameter(d)));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_select_by_inner_diameter_tolerance", [] {
        std::vector<V> out;
        for (const auto& [d, t] : std::vector<std::pair<double, double>>{{20, 0.0}, {20, 0.2}, {37.47, 0.0}, {8.1, 0.01}}) {
            out.push_back(neighbors(ORing::select_by_inner_diameter(d, t)));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_select_by_length", [] {
        std::vector<V> out;
        for (const double l : {101.42, 10.0, 300.0, 2000.0, 6.5}) out.push_back(neighbors(ORing::select_by_length(l)));
        return V::list(out);
    });
    add_value("bdw_o_rings_select_by_length_tolerance", [] {
        std::vector<V> out;
        for (const auto& [l, t] : std::vector<std::pair<double, double>>{{101.42, 0.0}, {101.42, 0.5}, {55.0, 0.01}}) {
            out.push_back(neighbors(ORing::select_by_length(l, t)));
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_select_errors", [] {
        return V::list({error_of([] { ORing::select_by_inner_diameter(0); }),
                        error_of([] { ORing::select_by_inner_diameter(-3.0); }),
                        error_of([] { ORing::select_by_length(10, -0.1); }),
                        error_of([] { ORing::select_by_length(10, 0.05, "as568"); }),
                        error_of([] { ORing::sizes("as568"); }), error_of([] { ORing::nominal_widths("as568"); }),
                        error_of([] { ORing::parameters("26", "as568"); }), error_of([] { ORing::parameters("999"); }),
                        error_of([] { ORing::parameters("abc"); })});
    });
    add_value("bdw_o_rings_nominal_widths", [] { return V::list_of(ORing::nominal_widths()); });
    add_value("bdw_o_rings_gland_width_for", [] {
        std::vector<V> out;
        for (const double w : kWidths) {
            for (const char* a : {"static", "dynamic"}) {
                for (const int n : {0, 1, 2}) {
                    if (w < 1.75 && n > 0) continue;
                    out.emplace_back(ORing::gland_width_for(w, a, n));
                }
            }
        }
        return V::list(out);
    });
    add_value("bdw_o_rings_gland_width_for_errors", [] {
        const double inf = std::numeric_limits<double>::infinity();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return V::list({error_of([] { ORing::gland_width_for(1.78, "rotary"); }),
                        error_of([] { ORing::gland_width_for(1.78, "static", 3); }),
                        error_of([] { ORing::gland_width_for(0.0); }), error_of([] { ORing::gland_width_for(-1.5); }),
                        error_of([inf] { ORing::gland_width_for(inf); }), error_of([nan] { ORing::gland_width_for(nan); }),
                        error_of([] { ORing::gland_width_for(1.8); }),
                        error_of([] { ORing::gland_width_for(1.02, "static", 1); }),
                        error_of([] { ORing::gland_width_for(1.27, "dynamic", 2); })});
    });

    // ---- O-rings
    add("bdw_o_rings_rotated", [] {
        return ORing("214", {.rotation = Rotation(90, 0, 0), .align = Align3(Align::MIN, Align::CENTER, Align::MAX)});
    });
    add_value("bdw_o_rings_rotated_values", [] {
        return V::list({loc_values(ORing("214", {.rotation = Rotation(0, 45, 0)}).location())});
    });
    add_value("bdw_o_rings_bad_size", [] { return error_of([] { ORing("1000"); }); });
    add_value("bdw_o_rings_bad_code", [] { return error_of([] { ORing("999"); }); });
    add_value("bdw_o_rings_bad_type", [] { return error_of([] { ORing("025", "as568"); }); });
    add_value("bdw_o_rings_gland_errors", [] {
        return V::list({error_of([] { ORing("001").static_gland_profile(true, 1); }),
                        error_of([] { ORing("001").dynamic_gland_profile(2); }),
                        error_of([] { ORing("025").static_gland_profile(false, 5); }),
                        error_of([] { ORing("025").dynamic_gland_profile(-1); })});
    });

    for (const auto& size : kShapeSizes) {
        add("bdw_o_rings_ring_" + size, [size] { return ORing(size); });
    }
    for (const std::string size : {"010", "025", "116", "214", "325", "381", "425", "475"}) {
        for (const bool axial : {true, false}) {
            for (const int rings : {0, 1, 2}) {
                const std::string name = "bdw_o_rings_static_gland_" + size + "_" + (axial ? "axial" : "radial") + "_" +
                                         std::to_string(rings);
                add(name, [size, axial, rings] { return ORing(size).static_gland_profile(axial, rings); });
                add_value(name + "_values", [size, axial, rings] {
                    ORing r(size);
                    r.static_gland_profile(axial, rings);
                    return gland_values(r);
                });
            }
        }
        for (const int rings : {0, 1, 2}) {
            const std::string name = "bdw_o_rings_dynamic_gland_" + size + "_" + std::to_string(rings);
            add(name, [size, rings] { return ORing(size).dynamic_gland_profile(rings); });
            add_value(name + "_values", [size, rings] {
                ORing r(size);
                r.dynamic_gland_profile(rings);
                return gland_values(r);
            });
        }
    }
}
