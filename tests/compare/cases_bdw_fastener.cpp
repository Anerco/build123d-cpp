// bd_warehouse's fastener.py, compared with cases_bdw_fastener_py.py.
#include <memory>
#include <regex>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

std::string case_name(const std::vector<std::string>& parts) {
    std::string out = "bdw_fastener";
    for (const auto& p : parts) out += "_" + std::regex_replace(p, std::regex("[^A-Za-z0-9]+"), "_");
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out;
}

std::string num_str(double x) {
    // Python's str() of the spec numbers (ints print without ".0")
    if (x == std::floor(x)) return std::to_string(static_cast<long long>(x));
    return compare::V(x).dump();
}

struct NutSpec {
    std::string cls, type, size;
};
struct ScrewSpec {
    std::string cls, type, size;
    double length;
    std::string length_text;  // as Python prints the spec's length
};
struct WasherSpec {
    std::string cls, type, size;
};

const std::vector<NutSpec> kNutSpecs = {
    {"DomedCapNut", "din1587", "M6-1"},
    {"DomedCapNut", "din1587", "M24-3"},
    {"HeatSetNut", "McMaster-Carr", "M3-0.5-Standard"},
    {"HeatSetNut", "McMaster-Carr", "M4-0.7-Short"},
    {"HeatSetNut", "Hilitchi", "M4-0.7-8"},
    {"HeatSetNut", "CNCKitchen", "M3-0.5-5.7"},
    {"HeatSetNut", "ruthex", "M2-0.4-4"},
    {"HeatSetNut", "AE-SamZhihui", "M1.4-0.3-H2-D2.3"},
    {"HexNut", "iso4032", "M1.6-0.35"},
    {"HexNut", "iso4032", "M6-1"},
    {"HexNut", "iso4032", "M64-6"},
    {"HexNut", "iso4033", "M10-1.5"},
    {"HexNut", "iso4035", "M8-1.25"},
    {"HexNutWithFlange", "din1665", "M5-0.8"},
    {"HexNutWithFlange", "din1665", "M20-1.5"},
    {"UnchamferedHexagonNut", "iso4036", "M2-0.4"},
    {"UnchamferedHexagonNut", "iso4036", "M10-1.5"},
    {"SquareNut", "din557", "M5-0.8"},
    {"SquareNut", "din557", "M16-2"},
};

const std::vector<ScrewSpec> kScrewSpecs = {
    {"ButtonHeadScrew", "iso7380_1", "M3-0.5", 8, "8.0"},
    {"ButtonHeadScrew", "iso7380_1", "M16-2", 40, "40.0"},
    {"ButtonHeadWithCollarScrew", "iso7380_2", "M5-0.8", 12, "12.0"},
    {"CheeseHeadScrew", "iso1207", "M3-0.5", 10, "10.0"},
    {"CheeseHeadScrew", "iso14580", "M4-0.7", 30, "30.0"},
    {"CheeseHeadScrew", "iso7048", "M5-0.8", 45, "45.0"},
    {"CounterSunkScrew", "iso10642", "M6-1", 20, "20.0"},
    {"CounterSunkScrew", "iso14581", "M3-0.5", 12, "12.0"},
    {"CounterSunkScrew", "iso14582", "M8-1.25", 30, "30.0"},
    {"CounterSunkScrew", "iso2009", "M5-0.8", 25, "25.0"},
    {"CounterSunkScrew", "iso7046", "M4-0.7", 50, "50.0"},
    {"HexHeadScrew", "din931", "M8-1.25", 40, "40.0"},
    {"HexHeadScrew", "iso4014", "M6-1", 30, "30.0"},
    {"HexHeadScrew", "iso4014", "M20-2.5", 220, "220.0"},
    {"HexHeadScrew", "iso4017", "M12-1.75", 25, "25.0"},
    {"HexHeadWithFlangeScrew", "din1662", "M6-1", 20, "20.0"},
    {"HexHeadWithFlangeScrew", "din1665", "M10-1.5", 30, "30.0"},
    {"PanHeadScrew", "iso14583", "M4-0.7", 10, "10.0"},
    {"PanHeadScrew", "iso1580", "M3-0.5", 45, "45.0"},
    {"PanHeadScrew", "asme_b_18.6.3", "#4-40", 0.5 * IN, "12.7"},
    {"PanHeadScrew", "asme_b_18.6.3", "1/4-20", 1.5 * IN, "38.099999999999994"},
    {"PanHeadWithCollarScrew", "din967", "M4-0.7", 12, "12.0"},
    {"RaisedCheeseHeadScrew", "iso7045", "M3-0.5", 8, "8.0"},
    {"RaisedCounterSunkOvalHeadScrew", "iso14584", "M5-0.8", 20, "20.0"},
    {"RaisedCounterSunkOvalHeadScrew", "iso2010", "M3-0.5", 12, "12.0"},
    {"RaisedCounterSunkOvalHeadScrew", "iso7047", "M6-1", 40, "40.0"},
    {"SetScrew", "iso4026", "M6-1", 10, "10.0"},
    {"SetScrew", "iso4026", "M3-0.5", 4, "4.0"},
    {"ShoulderScrew", "iso7379", "M6-1", 16, "16.0"},
    {"ShoulderScrew", "iso7379", "M10-1.5", 40, "40.0"},
    {"SocketHeadCapScrew", "iso4762", "M6-1", 20, "20.0"},
    {"SocketHeadCapScrew", "iso4762", "M3-0.5", 8, "8.0"},
    {"SocketHeadCapScrew", "iso4762", "M20-2.5", 100, "100.0"},
    {"SocketHeadCapScrew", "asme_b18.3", "#10-32", 0.75 * IN, "19.049999999999997"},
    {"SocketHeadCapScrew", "asme_b18.3", "1/4-20", 1 * IN, "25.4"},
    {"LowProfileScrew", "OpenBuilds", "M5-0.8", 8, "8.0"},
};

const std::vector<WasherSpec> kWasherSpecs = {
    {"PlainWasher", "iso7089", "M6"},
    {"PlainWasher", "iso7091", "M10"},
    {"PlainWasher", "iso7093", "M3"},
    {"PlainWasher", "iso7094", "M8"},
    {"ChamferedWasher", "iso7090", "M6"},
    {"ChamferedWasher", "iso7090", "M20"},
    {"CheeseHeadWasher", "iso7092", "M4"},
    {"InternalToothLockWasher", "din6797", "M5"},
    {"InternalToothLockWasher", "asme_b18.21.1", "#10"},
    {"InternalToothLockWasher", "asme_b18.21.1", "1/4"},
};

std::unique_ptr<Nut> make_nut(const std::string& cls, const std::string& size, const std::string& type) {
    if (cls == "DomedCapNut") return std::make_unique<DomedCapNut>(size, type);
    if (cls == "HeatSetNut") return std::make_unique<HeatSetNut>(size, type);
    if (cls == "HexNut") return std::make_unique<HexNut>(size, type);
    if (cls == "HexNutWithFlange") return std::make_unique<HexNutWithFlange>(size, type);
    if (cls == "UnchamferedHexagonNut") return std::make_unique<UnchamferedHexagonNut>(size, type);
    if (cls == "SquareNut") return std::make_unique<SquareNut>(size, type);
    throw std::invalid_argument("unknown nut class " + cls);
}

std::unique_ptr<Screw> make_screw(const std::string& cls, const std::string& size, double length, const std::string& type) {
    if (cls == "ButtonHeadScrew") return std::make_unique<ButtonHeadScrew>(size, length, type);
    if (cls == "ButtonHeadWithCollarScrew") return std::make_unique<ButtonHeadWithCollarScrew>(size, length, type);
    if (cls == "CheeseHeadScrew") return std::make_unique<CheeseHeadScrew>(size, length, type);
    if (cls == "CounterSunkScrew") return std::make_unique<CounterSunkScrew>(size, length, type);
    if (cls == "HexHeadScrew") return std::make_unique<HexHeadScrew>(size, length, type);
    if (cls == "HexHeadWithFlangeScrew") return std::make_unique<HexHeadWithFlangeScrew>(size, length, type);
    if (cls == "PanHeadScrew") return std::make_unique<PanHeadScrew>(size, length, type);
    if (cls == "PanHeadWithCollarScrew") return std::make_unique<PanHeadWithCollarScrew>(size, length, type);
    if (cls == "RaisedCheeseHeadScrew") return std::make_unique<RaisedCheeseHeadScrew>(size, length, type);
    if (cls == "RaisedCounterSunkOvalHeadScrew") return std::make_unique<RaisedCounterSunkOvalHeadScrew>(size, length, type);
    if (cls == "SetScrew") return std::make_unique<SetScrew>(size, length, type);
    if (cls == "ShoulderScrew") return std::make_unique<ShoulderScrew>(size, length, type);
    if (cls == "SocketHeadCapScrew") return std::make_unique<SocketHeadCapScrew>(size, length, type);
    if (cls == "LowProfileScrew") return std::make_unique<LowProfileScrew>(size, length, type);
    throw std::invalid_argument("unknown screw class " + cls);
}

std::unique_ptr<Washer> make_washer(const std::string& cls, const std::string& size, const std::string& type) {
    if (cls == "PlainWasher") return std::make_unique<PlainWasher>(size, type);
    if (cls == "ChamferedWasher") return std::make_unique<ChamferedWasher>(size, type);
    if (cls == "CheeseHeadWasher") return std::make_unique<CheeseHeadWasher>(size, type);
    if (cls == "InternalToothLockWasher") return std::make_unique<InternalToothLockWasher>(size, type);
    throw std::invalid_argument("unknown washer class " + cls);
}

V param_items(const ParamDict& d) {
    std::vector<V> out;
    for (const auto& [k, v] : d) out.push_back(V::list({k, num(v)}));
    return V::list(out);
}

V string_items(const Dict<std::string, std::string>& d) {
    std::vector<V> out;
    for (const auto& [k, v] : d) out.push_back(V::list({k, v}));
    return V::list(out);
}

std::string material_name(const Shape& s) { return s.material.get()->material.name; }

V nut_values(const Nut& n) {
    return V::list({n.label, n.info(), n.nut_class(), n.thread_size, n.thread_diameter, n.thread_pitch, n.is_metric,
                    n.fastener_type, n.hand, n.simple, n.nut_size, n.nut_thickness(), n.nut_diameter(),
                    param_items(n.nut_data), loc_values(n.joints.get<RigidJoint>("a").relative_location()),
                    loc_values(n.joints.get<RigidJoint>("b").relative_location()), material_name(n)});
}

V screw_values(const Screw& s) {
    const auto nominal = s.nominal_lengths();
    return V::list({s.label, s.info(), s.screw_class(), s.thread_size, s.thread_diameter, s.thread_pitch, s.is_metric,
                    s.fastener_type, s.length, s.head_height, s.head_diameter, s.max_thread_length, s.thread_length,
                    s.grip_length, s.thread_offset, nominal ? V::list_of(*nominal) : V(), s.min_hole_depth(),
                    s.min_hole_depth(false), s.length_offset(), param_items(s.screw_data),
                    loc_values(s.joints.get<RigidJoint>("a").relative_location()), material_name(s)});
}

V washer_values(const Washer& w) {
    // (Python keeps a size that isn't a number, e.g. "#10", as the thread diameter; NaN here)
    const V diameter = std::isnan(w.thread_diameter) ? V(w.thread_size) : V(w.thread_diameter);
    return V::list({w.label, w.info(), w.washer_class(), w.thread_size, diameter, w.washer_thickness(),
                    w.washer_diameter(), param_items(w.washer_data),
                    loc_values(w.joints.get<RigidJoint>("b").relative_location())});
}

V hole_tables(const Fastener& x, bool washer = false) {
    std::vector<V> out{string_items(x.clearance_drill_sizes()), param_items(x.clearance_hole_diameters())};
    if (!washer) {
        out.push_back(string_items(x.tap_drill_sizes()));
        out.push_back(param_items(x.tap_hole_diameters()));
    }
    return V::list(out);
}

V select_values(const Dict<std::string, std::vector<std::string>>& result) {
    std::vector<V> out;
    for (const auto& [cls, types] : result) out.push_back(V::list({cls, V::list_of(types)}));
    return V::list(out);
}

V recess_values(const std::pair<Face, double>& result) {
    const auto& [face, depth] = result;
    const BoundBox bb = face.bounding_box();
    return V::list({face.area(), face.edges().size(), depth, V::list({bb.min, bb.max})});
}

template <class C>
V types_of_class(const std::string& name) {
    return V::list({name, V::list_of(C::types())});
}

template <class C>
void sizes_of_class(std::vector<V>& out, const std::string& name) {
    for (const auto& t : C::types()) out.push_back(V::list({name, t, V::list_of(C::sizes(t))}));
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    for (const auto& spec : kNutSpecs) {
        add(case_name({spec.cls, spec.type, spec.size}), [spec] { return Shape(*make_nut(spec.cls, spec.size, spec.type)); });
        add_value(case_name({spec.cls, spec.type, spec.size, "values"}),
                  [spec] { return nut_values(*make_nut(spec.cls, spec.size, spec.type)); });
    }
    for (const auto& spec : kScrewSpecs) {
        add(case_name({spec.cls, spec.type, spec.size, spec.length_text}),
            [spec] { return Shape(*make_screw(spec.cls, spec.size, spec.length, spec.type)); });
        add_value(case_name({spec.cls, spec.type, spec.size, spec.length_text, "values"}),
                  [spec] { return screw_values(*make_screw(spec.cls, spec.size, spec.length, spec.type)); });
    }
    for (const auto& spec : kWasherSpecs) {
        add(case_name({spec.cls, spec.type, spec.size}), [spec] { return Shape(*make_washer(spec.cls, spec.size, spec.type)); });
        add_value(case_name({spec.cls, spec.type, spec.size, "values"}),
                  [spec] { return washer_values(*make_washer(spec.cls, spec.size, spec.type)); });
    }

    // ---- threads, hands, rotations and alignments
    add("bdw_fastener_hexnut_threaded", [] { return HexNut("M6-1", {.simple = false}); });
    add("bdw_fastener_hexnut_threaded_left", [] { return HexNut("M8-1.25", "iso4035", "left", false); });
    add("bdw_fastener_flange_nut_threaded", [] { return HexNutWithFlange("M8-1.25", {.simple = false}); });
    add("bdw_fastener_heatset_threaded", [] { return HeatSetNut("M3-0.5-Standard", {.simple = false}); });
    add("bdw_fastener_shcs_threaded", [] { return SocketHeadCapScrew("M4-0.7", 10, {.simple = false}); });
    add("bdw_fastener_shcs_threaded_left", [] { return SocketHeadCapScrew("M5-0.8", 30, {.hand = "left", .simple = false}); });
    add("bdw_fastener_countersunk_threaded", [] { return CounterSunkScrew("M5-0.8", 12, {.simple = false}); });
    add("bdw_fastener_setscrew_threaded", [] { return SetScrew("M5-0.8", 8, {.simple = false}); });
    add("bdw_fastener_shoulder_threaded", [] { return ShoulderScrew("M5-0.8", 10, {.simple = false}); });
    add("bdw_fastener_hexnut_rotated", [] {
        return HexNut("M6-1", {.rotation = Rotation(90, 0, 30), .align = Align3(Align::MIN)});
    });
    add("bdw_fastener_hexnut_threaded_rotated", [] {
        return HexNut("M5-0.8", {.simple = false, .rotation = Rotation(90, 0, 0)});
    });
    add("bdw_fastener_shcs_aligned", [] {
        return SocketHeadCapScrew("M6-1", 16, {.align = Align3(Align::CENTER, Align::CENTER, Align::MIN)});
    });
    add("bdw_fastener_washer_rotated", [] { return PlainWasher("M6", "iso7089", Rotation(0, 45, 0), Align3(Align::MAX)); });
    add_value("bdw_fastener_hexnut_threaded_children", [] { return children_values(HexNut("M5-0.8", {.simple = false})); });

    // ---- profiles
    add("bdw_fastener_hexnut_profile", [] { return HexNut("M6-1").nut_profile(); });
    add("bdw_fastener_hexnut_plan", [] { return HexNut("M6-1").nut_plan(); });
    add("bdw_fastener_hexnut_countersink", [] { return *HexNut("M6-1").countersink_profile("Normal"); });
    add("bdw_fastener_flange_nut_flange_profile", [] { return *HexNutWithFlange("M10-1.5").flange_profile(); });
    add("bdw_fastener_flange_nut_countersink", [] { return *HexNutWithFlange("M10-1.5").countersink_profile("Close"); });
    add("bdw_fastener_domed_nut_countersink", [] { return *DomedCapNut("M8-1.25").countersink_profile("Loose"); });
    add("bdw_fastener_square_nut_plan", [] { return SquareNut("M8-1.25").nut_plan(); });
    add("bdw_fastener_square_nut_countersink", [] { return *SquareNut("M8-1.25").countersink_profile("Normal"); });
    add("bdw_fastener_heatset_countersink", [] { return HeatSetNut("M4-0.7-Standard").countersink_profile(0.2); });
    add_value("bdw_fastener_heatset_knurled_faces", [] {
        std::vector<V> out;
        for (const auto& x : HeatSetNut::knurled_cylinder_faces(6, 2, 1.5, 3, 0.5, 9, 8, "left")) {
            out.push_back(V::list({x.area(), x.edges().size()}));
        }
        return V::list(out);
    });
    add_value("bdw_fastener_heatset_fill_factor", [] { return V(HeatSetNut("M3-0.5-Short").fill_factor()); });
    add_value("bdw_fastener_heatset_diameter", [] { return V(HeatSetNut("M4-0.7-8", "Hilitchi").nut_diameter()); });
    add("bdw_fastener_shcs_head_profile", [] { return *SocketHeadCapScrew("M6-1", 20).head_profile(); });
    add("bdw_fastener_shcs_countersink", [] { return *SocketHeadCapScrew("M6-1", 20).countersink_profile("Close"); });
    add("bdw_fastener_shcs_head", [] { return *SocketHeadCapScrew("M6-1", 20).make_head(); });
    add("bdw_fastener_shcs_shank", [] { return *SocketHeadCapScrew("M8-1.25", 50).shank_profile(); });
    add("bdw_fastener_shcs_recess", [] { return std::get<0>(*SocketHeadCapScrew("M6-1", 20).head_recess()); });
    add("bdw_fastener_shcs_body_hole", [] { return SocketHeadCapScrew("M6-1", 20).make_body_hole(3.2, 12); });
    add("bdw_fastener_hexhead_plan", [] { return *HexHeadScrew("M10-1.5", 30).head_plan(); });
    add("bdw_fastener_hexhead_countersink", [] { return *HexHeadScrew("M10-1.5", 30).countersink_profile("Normal"); });
    add("bdw_fastener_flange_screw_flange", [] { return *HexHeadWithFlangeScrew("M8-1.25", 30).flange_profile(); });
    add("bdw_fastener_flange_screw_countersink", [] {
        return *HexHeadWithFlangeScrew("M8-1.25", 30).countersink_profile("Loose");
    });
    add("bdw_fastener_countersunk_countersink", [] { return *CounterSunkScrew("M6-1", 20).countersink_profile("Normal"); });
    add("bdw_fastener_oval_countersink", [] {
        return *RaisedCounterSunkOvalHeadScrew("M5-0.8", 20).countersink_profile("Normal");
    });
    add("bdw_fastener_collar_countersink", [] { return *ButtonHeadWithCollarScrew("M6-1", 16).countersink_profile("Normal"); });
    add("bdw_fastener_shoulder_shank", [] { return *ShoulderScrew("M8-1.25", 20).shank_profile(); });
    add("bdw_fastener_shoulder_countersink", [] { return *ShoulderScrew("M8-1.25", 20).countersink_profile("Normal"); });
    add("bdw_fastener_shoulder_body_hole", [] { return ShoulderScrew("M8-1.25", 20).make_body_hole(3.4, 40, 8); });
    add_value("bdw_fastener_setscrew_countersink", [] {
        return V(!SetScrew("M6-1", 10).countersink_profile("Normal").has_value());
    });
    add("bdw_fastener_lowprofile_head", [] { return *LowProfileScrew("M4-0.7", 10).head_profile(); });
    add("bdw_fastener_washer_profile", [] { return ChamferedWasher("M8").washer_profile(); });
    add("bdw_fastener_washer_countersink", [] { return *PlainWasher("M8", "iso7093").countersink_profile("Loose"); });
    add("bdw_fastener_cheese_washer_profile", [] { return CheeseHeadWasher("M5").washer_profile(); });

    // ---- tables, types, sizes
    add_value("bdw_fastener_hole_tables_nut", [] { return hole_tables(HexNut("M6-1")); });
    add_value("bdw_fastener_hole_tables_screw", [] {
        return hole_tables(SocketHeadCapScrew("#10-32", 10, "asme_b18.3"));
    });
    add_value("bdw_fastener_hole_tables_washer", [] { return hole_tables(PlainWasher("M6", "iso7089"), true); });
    add_value("bdw_fastener_washer_clearance_drill_sizes", [] {
        return error_of([] { PlainWasher("M6", "iso7089").clearance_drill_sizes(); });
    });
    add_value("bdw_fastener_types", [] {
        return V::list({types_of_class<DomedCapNut>("DomedCapNut"), types_of_class<HeatSetNut>("HeatSetNut"),
                        types_of_class<HexNut>("HexNut"), types_of_class<HexNutWithFlange>("HexNutWithFlange"),
                        types_of_class<UnchamferedHexagonNut>("UnchamferedHexagonNut"),
                        types_of_class<SquareNut>("SquareNut"), types_of_class<ButtonHeadScrew>("ButtonHeadScrew"),
                        types_of_class<ButtonHeadWithCollarScrew>("ButtonHeadWithCollarScrew"),
                        types_of_class<CheeseHeadScrew>("CheeseHeadScrew"),
                        types_of_class<CounterSunkScrew>("CounterSunkScrew"),
                        types_of_class<HexHeadScrew>("HexHeadScrew"),
                        types_of_class<HexHeadWithFlangeScrew>("HexHeadWithFlangeScrew"),
                        types_of_class<PanHeadScrew>("PanHeadScrew"),
                        types_of_class<PanHeadWithCollarScrew>("PanHeadWithCollarScrew"),
                        types_of_class<RaisedCheeseHeadScrew>("RaisedCheeseHeadScrew"),
                        types_of_class<RaisedCounterSunkOvalHeadScrew>("RaisedCounterSunkOvalHeadScrew"),
                        types_of_class<SetScrew>("SetScrew"), types_of_class<ShoulderScrew>("ShoulderScrew"),
                        types_of_class<SocketHeadCapScrew>("SocketHeadCapScrew"),
                        types_of_class<LowProfileScrew>("LowProfileScrew"), types_of_class<PlainWasher>("PlainWasher"),
                        types_of_class<ChamferedWasher>("ChamferedWasher"),
                        types_of_class<CheeseHeadWasher>("CheeseHeadWasher"),
                        types_of_class<InternalToothLockWasher>("InternalToothLockWasher")});
    });
    add_value("bdw_fastener_sizes", [] {
        std::vector<V> out;
        sizes_of_class<DomedCapNut>(out, "DomedCapNut");
        sizes_of_class<HeatSetNut>(out, "HeatSetNut");
        sizes_of_class<HexNut>(out, "HexNut");
        sizes_of_class<HexNutWithFlange>(out, "HexNutWithFlange");
        sizes_of_class<UnchamferedHexagonNut>(out, "UnchamferedHexagonNut");
        sizes_of_class<SquareNut>(out, "SquareNut");
        sizes_of_class<ButtonHeadScrew>(out, "ButtonHeadScrew");
        sizes_of_class<ButtonHeadWithCollarScrew>(out, "ButtonHeadWithCollarScrew");
        sizes_of_class<CheeseHeadScrew>(out, "CheeseHeadScrew");
        sizes_of_class<CounterSunkScrew>(out, "CounterSunkScrew");
        sizes_of_class<HexHeadScrew>(out, "HexHeadScrew");
        sizes_of_class<HexHeadWithFlangeScrew>(out, "HexHeadWithFlangeScrew");
        sizes_of_class<PanHeadScrew>(out, "PanHeadScrew");
        sizes_of_class<PanHeadWithCollarScrew>(out, "PanHeadWithCollarScrew");
        sizes_of_class<RaisedCheeseHeadScrew>(out, "RaisedCheeseHeadScrew");
        sizes_of_class<RaisedCounterSunkOvalHeadScrew>(out, "RaisedCounterSunkOvalHeadScrew");
        sizes_of_class<SetScrew>(out, "SetScrew");
        sizes_of_class<ShoulderScrew>(out, "ShoulderScrew");
        sizes_of_class<SocketHeadCapScrew>(out, "SocketHeadCapScrew");
        sizes_of_class<LowProfileScrew>(out, "LowProfileScrew");
        sizes_of_class<PlainWasher>(out, "PlainWasher");
        sizes_of_class<ChamferedWasher>(out, "ChamferedWasher");
        sizes_of_class<CheeseHeadWasher>(out, "CheeseHeadWasher");
        sizes_of_class<InternalToothLockWasher>(out, "InternalToothLockWasher");
        return V::list(out);
    });
    add_value("bdw_fastener_select_nut", [] { return select_values(Nut::select_by_size("M6-1")); });
    add_value("bdw_fastener_select_screw", [] { return select_values(Screw::select_by_size("M6-1")); });
    add_value("bdw_fastener_select_screw_imperial", [] { return select_values(Screw::select_by_size("1/4-20")); });
    add_value("bdw_fastener_select_washer", [] { return select_values(Washer::select_by_size("M8")); });
    add_value("bdw_fastener_nominal_lengths", [] {
        std::vector<V> out;
        for (const auto& [k, v] : Screw::nominal_length_range()) out.push_back(V::list({k, V::list_of(v)}));
        return V::list(out);
    });

    // ---- functions
    add_value("bdw_fastener_polygon_diagonal", [] {
        return V::list({polygon_diagonal(10), polygon_diagonal(8, 4), polygon_diagonal(5, 8)});
    });
    add_value("bdw_fastener_read_csv", [] { return table_values(read_fastener_parameters_from_csv("square_nut_parameters.csv")); });
    add_value("bdw_fastener_read_csv_missing", [] { return table_values(read_fastener_parameters_from_csv("missing.csv")); });
    add_value("bdw_fastener_decode_imperial", [] {
        std::vector<V> out;
        for (const char* s : {"#10-32", "1/4-20", "#0-80", "1 1/2-6", "3/8-24"}) {
            const auto [d, p] = decode_imperial_size(s);
            out.push_back(V::list({d, p}));
        }
        return V::list(out);
    });
    add_value("bdw_fastener_decode_imperial_bad", [] { return error_of([] { decode_imperial_size("#7-32"); }); });
    add_value("bdw_fastener_metric_str_to_float", [] {
        std::vector<V> out;
        for (const char* s : {"12", "2.5", "1/2", "PH2", "T30", " 4", "1//2"}) out.push_back(num(metric_str_to_float(s)));
        return V::list(out);
    });
    add_value("bdw_fastener_evaluate", [] {
        return param_items(evaluate_parameter_dict({{"a", "1/4"}, {"b", "#7"}, {"c", "1 1/2"}}, false));
    });
    add_value("bdw_fastener_evaluate_dict_of_dict", [] {
        return table_values(evaluate_parameter_dict_of_dict({{"M3", {{"x", "3"}, {"y", "R1"}}}, {"M4", {{"x", "4.5"}}}}));
    });
    add_value("bdw_fastener_isolate", [] {
        return table_values(isolate_fastener_type("iso4033", read_fastener_parameters_from_csv("hex_nut_parameters.csv")));
    });
    add_value("bdw_fastener_isolate_bad", [] {
        return error_of([] { isolate_fastener_type("x", {{"M3", {{"a:b:c", "1"}}}}); });
    });
    add_value("bdw_fastener_drill_sizes", [] {
        std::vector<V> out;
        for (const auto& [k, v] : read_drill_sizes()) out.push_back(V::list({k, v}));
        return V::list(out);
    });
    add_value("bdw_fastener_lookup_drill", [] {
        return table_values(lookup_drill_diameters(
            {{"M3", {{"Close", "3.2"}, {"Loose", "#30"}}}, {"#4", {{"Close", "1/8"}, {"Normal", "C"}}}}));
    });
    add_value("bdw_fastener_cross_recess", [] { return recess_values(cross_recess("PH2")); });
    add_value("bdw_fastener_cross_recess_bad", [] { return error_of([] { cross_recess("PH5"); }); });
    add("bdw_fastener_hex_recess", [] { return hex_recess(5); });
    add_value("bdw_fastener_hexalobular_recess", [] { return recess_values(hexalobular_recess("T20")); });
    add_value("bdw_fastener_hexalobular_recess_t100", [] { return recess_values(hexalobular_recess("T100")); });
    add_value("bdw_fastener_hexalobular_recess_bad", [] { return error_of([] { hexalobular_recess("T7"); }); });
    add("bdw_fastener_slot_recess", [] { return slot_recess(5, 1); });
    add_value("bdw_fastener_square_recess", [] { return recess_values(square_recess("R2")); });
    add_value("bdw_fastener_square_recess_bad", [] { return error_of([] { square_recess("R4"); }); });

    // ---- holes
    add("bdw_fastener_clearance_hole", [] { return ClearanceHole(SocketHeadCapScrew("M6-1", 20), "Normal", 15); });
    add("bdw_fastener_clearance_hole_close", [] { return ClearanceHole(SocketHeadCapScrew("M4-0.7", 16), "Close", 10); });
    add("bdw_fastener_clearance_hole_loose_flat", [] {
        return ClearanceHole(HexHeadScrew("M8-1.25", 30), "Loose", 12, false);
    });
    add("bdw_fastener_clearance_hole_countersunk", [] { return ClearanceHole(CounterSunkScrew("M5-0.8", 16), "Normal", 8); });
    add("bdw_fastener_clearance_hole_oval", [] {
        return ClearanceHole(RaisedCounterSunkOvalHeadScrew("M4-0.7", 16), "Normal", 8);
    });
    add("bdw_fastener_clearance_hole_nut", [] { return ClearanceHole(HexNut("M6-1"), "Normal", 10); });
    add("bdw_fastener_clearance_hole_captive_hex", [] { return ClearanceHole(HexNut("M6-1"), "Normal", 10, true, true); });
    add("bdw_fastener_clearance_hole_captive_domed", [] {
        return ClearanceHole(DomedCapNut("M8-1.25"), "Close", 10, true, true);
    });
    add("bdw_fastener_clearance_hole_captive_square", [] { return ClearanceHole(SquareNut("M6-1"), "Normal", 10, true, true); });
    add("bdw_fastener_clearance_hole_shoulder", [] { return ClearanceHole(ShoulderScrew("M6-1", 16), "Normal", 30); });
    add("bdw_fastener_clearance_hole_rotated", [] {
        return ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), "Normal", 5, true, false, Rotation(0, 90, 0));
    });
    add("bdw_fastener_clearance_hole_washer", [] { return ClearanceHole(PlainWasher("M6", "iso7089"), "Normal", 5); });
    add_value("bdw_fastener_clearance_hole_attrs", [] {
        return V(ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), "Normal", 5).hole_depth);
    });
    add_value("bdw_fastener_clearance_hole_no_depth", [] { return error_of([] { ClearanceHole(SocketHeadCapScrew("M3-0.5", 8)); }); });
    add_value("bdw_fastener_clearance_hole_heatset", [] {
        return error_of([] { ClearanceHole(HeatSetNut("M3-0.5-Short"), "Normal", 5); });
    });
    add_value("bdw_fastener_clearance_hole_captive_bad", [] {
        return error_of([] { ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), "Normal", 5, true, true); });
    });
    add_value("bdw_fastener_clearance_hole_bad_fit", [] {
        return error_of([] { ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), "Snug", 5); });
    });
    add("bdw_fastener_tap_hole", [] { return TapHole(SocketHeadCapScrew("M6-1", 20), "Soft", "Normal", 15); });
    add("bdw_fastener_tap_hole_hard", [] { return TapHole(HexHeadScrew("M10-1.5", 40), "Hard", "Close", 20, false); });
    add("bdw_fastener_tap_hole_countersunk", [] { return TapHole(CounterSunkScrew("M4-0.7", 12), "Soft", "Normal", 10); });
    add("bdw_fastener_tap_hole_nut", [] { return TapHole(HexNut("M8-1.25"), "Soft", "Normal", 10); });
    add_value("bdw_fastener_tap_hole_bad_material", [] {
        return error_of([] { TapHole(HexNut("M8-1.25"), "Wood", "Normal", 10); });
    });
    add_value("bdw_fastener_tap_hole_washer", [] {
        return error_of([] { TapHole(PlainWasher("M8", "iso7089"), "Soft", "Normal", 10); });
    });
    add("bdw_fastener_threaded_hole", [] { return ThreadedHole(SocketHeadCapScrew("M6-1", 20), "Soft", "Normal", 15); });
    add("bdw_fastener_threaded_hole_threaded", [] {
        return ThreadedHole(SocketHeadCapScrew("M4-0.7", 10), "Soft", "Normal", 8, true, false);
    });
    add("bdw_fastener_threaded_hole_flat", [] {
        return ThreadedHole(HexHeadScrew("M8-1.25", 30), "Hard", "Normal", 12, false);
    });
    add("bdw_fastener_threaded_hole_shoulder", [] { return ThreadedHole(ShoulderScrew("M6-1", 16), "Soft", "Normal", 30); });
    add_value("bdw_fastener_threaded_hole_attrs", [] {
        const ThreadedHole t(SocketHeadCapScrew("M4-0.7", 10), "Soft", "Normal", 8, true, false);
        return V::list({t.hole_depth, t.thread->length, V::list_of(std::vector<double>{}), children_values(t),
                        !ThreadedHole(SocketHeadCapScrew("M4-0.7", 10), "Soft", "Normal", 8).thread.has_value()});
    });
    add_value("bdw_fastener_threaded_hole_nut", [] {
        return error_of([] { ThreadedHole(HexNut("M8-1.25"), "Soft", "Normal", 10); });
    });
    add_value("bdw_fastener_threaded_hole_shallow", [] {
        return error_of([] { ThreadedHole(SocketHeadCapScrew("M6-1", 20), "Soft", "Normal", 4); });
    });
    add_value("bdw_fastener_threaded_hole_zero", [] {
        return error_of([] { ThreadedHole(SocketHeadCapScrew("M6-1", 20), "Soft", "Normal", 0); });
    });
    add("bdw_fastener_insert_hole", [] { return InsertHole(HeatSetNut("M3-0.5-Standard"), "Normal", 10); });
    add("bdw_fastener_insert_hole_compensated", [] {
        return InsertHole(HeatSetNut("M4-0.7-8", "Hilitchi"), "Close", 12, 0.2);
    });

    // ---- errors
    add_value("bdw_fastener_nut_bad_size_format", [] { return error_of([] { HexNut("M6"); }); });
    add_value("bdw_fastener_nut_bad_type", [] { return error_of([] { DomedCapNut("M6-1", "iso4032"); }); });
    add_value("bdw_fastener_nut_bad_hand", [] { return error_of([] { HexNut("M6-1", {.hand = "up"}); }); });
    add_value("bdw_fastener_nut_bad_size", [] { return error_of([] { SquareNut("M7-1"); }); });
    add_value("bdw_fastener_screw_bad_size_format", [] { return error_of([] { SocketHeadCapScrew("M6", 20); }); });
    add_value("bdw_fastener_screw_bad_size", [] { return error_of([] { ButtonHeadScrew("M7-1", 20); }); });
    add_value("bdw_fastener_screw_bad_type", [] { return error_of([] { ButtonHeadScrew("M6-1", 20, "iso4762"); }); });
    add_value("bdw_fastener_screw_too_short", [] { return error_of([] { CounterSunkScrew("M6-1", 3); }); });
    add_value("bdw_fastener_shoulder_bad_length", [] { return error_of([] { ShoulderScrew("M6-1", 15); }); });
    add_value("bdw_fastener_shoulder_bad_size", [] { return error_of([] { ShoulderScrew("M7-1", 15); }); });
    add_value("bdw_fastener_washer_bad_size", [] { return error_of([] { PlainWasher("M7", "iso7089"); }); });
    add_value("bdw_fastener_washer_bad_type", [] { return error_of([] { ChamferedWasher("M6", "iso7089"); }); });
    add_value("bdw_fastener_no_tap_data", [] { return error_of([] { HexNut("M1.6-0.35").tap_hole_diameters(); }); });
}
