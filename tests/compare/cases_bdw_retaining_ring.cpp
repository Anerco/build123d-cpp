// bd_warehouse's retaining_ring.py, compared with cases_bdw_retaining_ring_py.py.
#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

V number_items(const Dict<std::string, double>& d) {
    std::vector<V> out;
    for (const auto& [k, v] : d) out.push_back(V::list({k, v}));
    return V::list(out);
}

V ring_values(const RetainingRing& r) {
    const auto rgba = r.color.get()->to_tuple();
    return V::list({r.label, r.info(), r.ring_size, r.standard, number_items(r.ring_dict),
                    V::list({rgba[0], rgba[1], rgba[2], rgba[3]}), loc_values(r.location()),
                    loc_values(r.joints.get<RigidJoint>("a").relative_location()),
                    loc_values(r.joints.get<RigidJoint>("b").relative_location())});
}

V groove_values(const RetainingRingGroove& g) {
    return V::list({g.label, g.groove_diameter, g.groove_width, g.clearance_diameter, g.ring->label});
}

std::vector<V> select_values(const Dict<std::string, std::vector<std::string>>& result) {
    std::vector<V> out;
    for (const auto& [cls, standards] : result) out.push_back(V::list({cls, V::list_of(standards)}));
    return out;
}

V table_items(const RingTable& table) {
    std::vector<V> rows;
    for (const auto& [k, row] : table) rows.push_back(V::list({k, number_items(row)}));
    return V::list(rows);
}

template <class C>
void add_ring_cases(const std::string& name, const std::vector<std::string>& groove_sizes) {
    using compare::add;
    using compare::add_value;
    for (const auto& size : C::sizes()) {
        add("bdw_retaining_ring_" + name + "_" + size, [size] { return C(size); });
        add_value("bdw_retaining_ring_" + name + "_" + size + "_values", [size] { return ring_values(C(size)); });
    }
    for (const auto& size : groove_sizes) {
        add("bdw_retaining_ring_" + name + "_" + size + "_groove", [size] { return RetainingRingGroove(C(size)); });
        add_value("bdw_retaining_ring_" + name + "_" + size + "_groove_values",
                  [size] { return groove_values(RetainingRingGroove(C(size))); });
    }
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    add_ring_cases<ExternalSnapRing>("ExternalSnapRing", {"3", "9", "10", "40", "100", "300"});
    add_ring_cases<InternalSnapRing>("InternalSnapRing", {"8", "11", "12", "30", "100", "300"});

    add_value("bdw_retaining_ring_read_din471", [] { return table_items(read_retaining_ring_parameters_from_csv("din471.csv")); });
    add_value("bdw_retaining_ring_read_din472", [] { return table_items(read_retaining_ring_parameters_from_csv("din472.csv")); });
    add_value("bdw_retaining_ring_sizes", [] {
        return V::list({V::list_of(ExternalSnapRing::sizes()), V::list_of(InternalSnapRing::sizes())});
    });
    add_value("bdw_retaining_ring_parameters", [] {
        return V::list({number_items(ExternalSnapRing::parameters(" 12 ")), number_items(InternalSnapRing::parameters("47"))});
    });
    add_value("bdw_retaining_ring_select_by_size", [] {
        std::vector<V> out;
        for (const char* s : {"40", " 8 ", "3", "31", "500"}) out.push_back(V::list(select_values(RetainingRing::select_by_size(s))));
        for (const char* s : {"8", "31"}) out.push_back(V::list(select_values(ExternalSnapRing::select_by_size(s))));
        for (const char* s : {"3", "31"}) out.push_back(V::list(select_values(InternalSnapRing::select_by_size(s))));
        return V::list(out);
    });
    add_value("bdw_retaining_ring_size_stripped", [] { return V(ExternalSnapRing(" 40 ").label); });
    add("bdw_retaining_ring_rotated", [] { return ExternalSnapRing("40", Rotation(90, 0, 0), Align3(Align::MIN)); });
    add_value("bdw_retaining_ring_rotated_values", [] {
        return ring_values(InternalSnapRing("30", Rotation(0, 30, 0), Align3(Align::MAX)));
    });
    add("bdw_retaining_ring_groove_rotated", [] {
        return RetainingRingGroove(InternalSnapRing("30"), Rotation(0, 90, 0), Align3(Align::CENTER, Align::CENTER, Align::MIN));
    });
    add_value("bdw_retaining_ring_bad_external", [] { return error_of([] { ExternalSnapRing("2"); }); });
    add_value("bdw_retaining_ring_bad_internal", [] { return error_of([] { InternalSnapRing("3"); }); });
    add_value("bdw_retaining_ring_bad_parameters", [] { return error_of([] { InternalSnapRing::parameters("12.5"); }); });
}
