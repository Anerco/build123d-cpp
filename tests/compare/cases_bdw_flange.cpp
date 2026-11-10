// bd_warehouse's flange.py, compared with cases_bdw_flange_py.py.
#include <functional>
#include <memory>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

const std::vector<std::string> kShapeFaces = {"Flat", "Raised", "Ring"};
const std::vector<std::pair<std::string, std::string>> kStubSpecs = {
    {"copper", "K"}, {"stainless", "40S"}, {"abs", "40"}};
const std::vector<std::string> kStubNps = {"1", "2", "4", "8"};

std::string key(const std::string& nps) {
    std::string out = nps;
    for (auto& c : out) {
        if (c == ' ' || c == '/') c = '_';
    }
    return out;
}

V opt_text(const std::optional<std::string>& s) { return s ? V(*s) : V(); }

/// The attributes the flange classes share (nullopt where a class doesn't have one).
struct FlangeInfo {
    std::string nps;
    std::optional<double> od, thickness, id;
    std::optional<int> flange_class;
    std::optional<std::string> face_type;
};

template <class F>
FlangeInfo info_of(const F& f) {
    FlangeInfo info{f.nps, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
    if constexpr (!std::is_same_v<F, LappedFlangeStub>) {
        info.od = f.od;
        info.thickness = f.thickness;
        info.flange_class = f.flange_class;
    }
    if constexpr (std::is_same_v<F, LappedFlange> || std::is_same_v<F, SlipOnFlange> ||
                  std::is_same_v<F, SocketWeldFlange> || std::is_same_v<F, WeldNeckFlange>) {
        info.id = f.id;
    }
    if constexpr (std::is_same_v<F, BlindFlange> || std::is_same_v<F, SlipOnFlange> ||
                  std::is_same_v<F, SocketWeldFlange> || std::is_same_v<F, WeldNeckFlange>) {
        info.face_type = f.face_type;
    }
    return info;
}

template <class F>
V flange_values(const F& f) {
    const FlangeInfo info = info_of(f);
    std::vector<V> joints;
    for (const auto& [name, joint] : f.joints.items()) {
        joints.push_back(V::list({name, loc_values(joint.template as<RigidJoint>().relative_location())}));
    }
    return V::list({info.nps, opt(info.od), opt(info.thickness), opt(info.id),
                    info.flange_class ? V(*info.flange_class) : V(), opt_text(info.face_type), V::list(joints)});
}

/// The flange's values, or the error it throws.
template <class F, class Build>
V attempt(Build build) {
    std::unique_ptr<F> made;
    const V error = error_of([&] { made = std::make_unique<F>(build()); });
    if (!made) return error;
    return flange_values(*made);
}

V table(const FlangeTable& t) {
    std::vector<V> rows;
    for (const auto& [k, row] : t) {
        std::vector<V> cells;
        for (const auto& v : row) cells.push_back(num(v));
        rows.push_back(V::list({k, V::list(cells)}));
    }
    return V::list(rows);
}

V section_values(const std::pair<std::optional<Sketch>, double>& result) {
    const auto& [section, height] = result;
    if (!section) return V::list({V(), height});
    const BoundBox bb = section->bounding_box();
    return V::list({section->area(), section->faces().size(), V::list({bb.min.X, bb.min.Y, bb.max.X, bb.max.Y}), height});
}

/// error_of(...) or the values (Python's `error_of(...).data or values`).
template <class F>
V error_or_section(F get) {
    std::optional<std::pair<std::optional<Sketch>, double>> result;
    const V error = error_of([&] { result = get(); });
    if (!result) return error;
    return section_values(*result);
}

template <class C>
void add_class_cases(const std::string& cls_name) {
    using compare::add;
    using compare::add_value;
    add_value("bdw_flange_" + cls_name + "_values", [] {
        std::vector<V> out;
        std::vector<std::string> faces = flange_face_types();
        faces.push_back("Bogus");
        for (const auto& nps : flange_nps()) {
            for (const int k : {150, 300}) {
                for (const auto& f : faces) out.push_back(attempt<C>([&] { return C(nps, k, f); }));
            }
        }
        return V::list(out);
    });
    for (const auto& nps : flange_nps()) {
        for (const int flange_class : {150, 300}) {
            for (const auto& face : kShapeFaces) {
                add("bdw_flange_" + cls_name + "_" + key(nps) + "_" + std::to_string(flange_class) + "_" + face,
                    [nps, flange_class, face] { return C(nps, flange_class, face); });
            }
        }
    }
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    add_value("bdw_flange_literals", [] {
        return V::list({V::list_of(flange_nps()), V::list_of(flange_face_types()), V::list_of(flange_classes())});
    });
    add_value("bdw_flange_nps_to_dn", [] {
        std::vector<V> out;
        for (const auto& [k, v] : nps_to_dn()) out.push_back(V::list({k, v}));
        return V::list(out);
    });
    add_value("bdw_flange_drilling_class_150", [] { return table(drilling_class_150()); });
    add_value("bdw_flange_drilling_class_300", [] { return table(drilling_class_300()); });
    add_value("bdw_flange_flange_data_class_150", [] { return table(flange_data_class_150()); });
    add_value("bdw_flange_flange_data_class_300", [] { return table(flange_data_class_300()); });
    add_value("bdw_flange_ring_joint_facings_class_150", [] { return table(ring_joint_facings_class_150()); });
    add_value("bdw_flange_ring_joint_facings_class_300", [] { return table(ring_joint_facings_class_300()); });
    add_value("bdw_flange_inputs_are_valid", [] {
        return V::list({Flange::inputs_are_valid("2", 300, "Raised"), Flange::inputs_are_valid("24", 2500, std::nullopt),
                        error_of([] { Flange::inputs_are_valid("3 1/2", 150, "Raised"); }),
                        error_of([] { Flange::inputs_are_valid("2", 125, "Raised"); }),
                        error_of([] { Flange::inputs_are_valid("2", 150, "Smooth"); })});
    });
    add_value("bdw_flange_dimension_is_numeric", [] {
        std::vector<Param> values = flange_data_class_150().at("14");
        const auto& more = drilling_class_150().at("1/2");
        values.insert(values.end(), more.begin(), more.end());
        std::vector<V> out;
        for (const auto& v : values) out.emplace_back(Flange::dimension_is_numeric(v));
        return V::list(out);
    });
    add_value("bdw_flange_require_numeric_dimensions", [] {
        return V::list({error_of([] {
                            Flange::require_numeric_dimensions(flange_data_class_150().at("2"), {{"a", 0}, {"b", 14}}, "X");
                        }),
                        error_of([] {
                            Flange::require_numeric_dimensions(flange_data_class_150().at("20"),
                                                               {{"a", 11}, {"b", 13}, {"c", 1}}, "Y");
                        }),
                        error_of([] {
                            Flange::require_numeric_dimensions(flange_data_class_300().at("1"), {{"a", 0}, {"b", 14}}, "Z");
                        })});
    });
    add_value("bdw_flange_get_flange_data", [] {
        std::vector<V> out;
        for (const auto& nps : flange_nps()) {
            for (const int c : {150, 300}) {
                const auto [flange_data, bolt_data] = Flange::get_flange_data(nps, c, std::nullopt);
                std::vector<V> a, b;
                for (const auto& v : flange_data) a.push_back(num(v));
                for (const auto& v : bolt_data) b.push_back(num(v));
                out.push_back(V::list({V::list(a), V::list(b)}));
            }
        }
        out.push_back(error_of([] { Flange::get_flange_data("2", 600, "Raised"); }));
        out.push_back(error_of([] { Flange::get_flange_data("7", 150, std::nullopt); }));
        return V::list(out);
    });
    add_value("bdw_flange_get_face_section_data", [] {
        std::vector<V> out;
        const std::vector<std::optional<std::string>> faces = {std::nullopt, "Flat", "Raised", "Ring", "Groove"};
        for (const auto& nps : flange_nps()) {
            for (const int c : {150, 300}) {
                for (const auto& f : faces) {
                    out.push_back(error_or_section([&] { return Flange::get_face_section_data(nps, c, f); }));
                }
            }
        }
        return V::list(out);
    });
    add_value("bdw_flange_get_face_section_data_thickness", [] {
        std::vector<V> out;
        const std::vector<std::optional<std::string>> faces = {std::nullopt, "Flat",   "Raised", "Ring",
                                                               "Groove",     "Tongue", "Male",   "Female"};
        for (const auto& f : faces) {
            for (const double t : {3.0, 7.0}) {
                out.push_back(error_or_section([&] { return Flange::get_face_section_data("4", 150, f, t); }));
            }
        }
        out.push_back(error_of([] { Flange::get_face_section_data("4", 400, "Raised"); }));
        return V::list(out);
    });
    add("bdw_flange_ring_face_section", [] { return *Flange::get_face_section_data("4", 300, "Ring").first; });
    add_value("bdw_flange_socket_weld_face_section_data", [] {
        std::vector<V> out;
        for (const auto& nps : flange_nps()) {
            for (const int c : {150, 300}) {
                out.push_back(error_or_section([&] { return SocketWeldFlange::get_face_section_data(nps, c, "Raised"); }));
            }
        }
        return V::list(out);
    });
    add_value("bdw_flange_weld_neck_face_section_data", [] {
        std::vector<V> out;
        for (const auto& nps : flange_nps()) {
            for (const int c : {150, 300}) {
                out.push_back(error_or_section([&] { return WeldNeckFlange::get_face_section_data(nps, c, "Ring"); }));
            }
        }
        return V::list(out);
    });
    add("bdw_flange_base", [] {
        return Flange(Plane::XZ * Rectangle(200, 20, Align2(Align::MIN, Align::MIN)), 150, 6, 12);
    });
    add("bdw_flange_base_rotated", [] {
        return Flange(Plane::XZ * Rectangle(100, 15, Align2(Align::MIN, Align::MIN)), 80, 4, 8, Rotation(90, 0, 0),
                      Align3(Align::MAX));
    });
    add("bdw_flange_blind_rotated", [] { return BlindFlange("4", 150, "Raised", Rotation(0, 90, 0), std::nullopt); });
    add("bdw_flange_weld_neck_aligned", [] {
        return WeldNeckFlange("6", 300, "Ring", Rotation(), Align3(Align::MIN, Align::MAX, Align::CENTER));
    });
    add_value("bdw_flange_lapped_values", [] {
        std::vector<V> out;
        for (const auto& nps : flange_nps()) {
            for (const int c : {150, 300}) out.push_back(attempt<LappedFlange>([&] { return LappedFlange(nps, c); }));
        }
        out.push_back(error_of([] { LappedFlange("2", 600); }));
        out.push_back(error_of([] { LappedFlange("2 1/4", 150); }));
        return V::list(out);
    });
    add_value("bdw_flange_stub_values", [] {
        std::vector<V> out;
        for (const auto& nps : kStubNps) {
            for (const auto& [m, i] : kStubSpecs) {
                for (const char* f : {"Flat", "Raised", "Ring", "Groove", "Male"}) {
                    out.push_back(attempt<LappedFlangeStub>([&] { return LappedFlangeStub(nps, m, i, 50, f); }));
                }
            }
        }
        out.push_back(error_of([] { LappedFlangeStub("2", "steel", "40", 50); }));
        return V::list(out);
    });
    add_value("bdw_flange_stub_edges", [] {
        std::vector<V> out;
        for (const auto& e : LappedFlangeStub("2", "copper", "K", 60).stub_edges) {
            out.push_back(V::list({e.length(), build123d::to_string(e.geom_type())}));
        }
        return V::list(out);
    });
    add_value("bdw_flange_stub_weld_radius", [] { return V(LappedFlangeStub::weld_radius); });
    add("bdw_flange_stub_rotated", [] {
        return LappedFlangeStub("4", "stainless", "40S", 80, "Ring", Rotation(0, 0, 45), std::nullopt);
    });

    add_class_cases<BlindFlange>("BlindFlange");
    add_class_cases<SlipOnFlange>("SlipOnFlange");
    add_class_cases<SocketWeldFlange>("SocketWeldFlange");
    add_class_cases<WeldNeckFlange>("WeldNeckFlange");

    for (const auto& nps : flange_nps()) {
        for (const int flange_class : {150, 300}) {
            add("bdw_flange_LappedFlange_" + key(nps) + "_" + std::to_string(flange_class),
                [nps, flange_class] { return LappedFlange(nps, flange_class); });
        }
    }
    for (const auto& nps : kStubNps) {
        for (const auto& [material, identifier] : kStubSpecs) {
            for (const std::string face : {"Raised", "Ring", "Groove"}) {
                const std::string m = material, i = identifier;
                add("bdw_flange_LappedFlangeStub_" + nps + "_" + m + "_" + i + "_" + face,
                    [nps, m, i, face] { return LappedFlangeStub(nps, m, i, 50, face); });
            }
        }
    }
}
