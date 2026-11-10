// bd_warehouse's thread.py, compared with cases_bdw_thread_py.py.
#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

V finishes(const EndFinishes& f) { return pair_values(f); }

V thread_attrs(const Thread& t) {
    return attrs({{"external", t.external},
                  {"apex_radius", t.apex_radius},
                  {"apex_width", t.apex_width},
                  {"root_radius", t.root_radius},
                  {"root_width", t.root_width},
                  {"pitch", t.pitch},
                  {"length", t.length},
                  {"apex_offset", t.apex_offset},
                  {"interference", t.interference},
                  {"right_hand", t.right_hand},
                  {"end_finishes", finishes(t.end_finishes)},
                  {"tooth_height", t.tooth_height},
                  {"taper", t.taper},
                  {"simple", t.simple},
                  {"manufacturing_compensation", t.manufacturing_compensation},
                  {"thread_loops", opt(t.thread_loops)}});
}

V thread_profile_values(const Face& p) {
    const BoundBox bb = p.bounding_box();
    return V::list({p.area(), p.edges().size(), V::list({bb.min.X, bb.min.Y, bb.max.X, bb.max.Y})});
}

V iso_attrs(const IsoThread& t) {
    return attrs({{"major_diameter", t.major_diameter},
                  {"pitch", t.pitch},
                  {"length", t.length},
                  {"external", t.external},
                  {"thread_angle", t.thread_angle},
                  {"hand", t.hand},
                  {"end_finishes", finishes(t.end_finishes)},
                  {"interference", t.interference},
                  {"simple", t.simple},
                  {"manufacturing_compensation", t.manufacturing_compensation},
                  {"apex_radius", t.apex_radius},
                  {"root_radius", t.root_radius},
                  {"h_parameter", t.h_parameter()},
                  {"min_radius", t.min_radius()}});
}

std::vector<std::pair<std::string, V>> whitworth_fields(const WhitworthThread& t) {
    return {{"major_diameter", t.major_diameter},
            {"pitch", t.pitch},
            {"length", t.length},
            {"external", t.external},
            {"hand", t.hand},
            {"end_finishes", finishes(t.end_finishes)},
            {"interference", t.interference},
            {"simple", t.simple},
            {"manufacturing_compensation", t.manufacturing_compensation},
            {"crest_truncation", t.crest_truncation},
            {"fundamental_triangle_height", t.fundamental_triangle_height},
            {"thread_height", t.thread_height},
            {"rounding_radius", t.rounding_radius},
            {"minor_diameter", t.minor_diameter},
            {"apex_radius", t.apex_radius},
            {"root_radius", t.root_radius},
            {"crest_diameter", t.crest_diameter},
            {"tooth_height", t.tooth_height},
            {"thread_loops", opt(t.thread_loops)},
            {"thread_angle", WhitworthThread::thread_angle}};
}

V whitworth_attrs(const WhitworthThread& t) { return attrs(whitworth_fields(t)); }

V bspp_attrs(const BSPPThread& t) {
    auto fields = whitworth_fields(t);
    const std::vector<std::pair<std::string, V>> more{{"thread_size", t.thread_size},
                                                       {"tpi", t.tpi},
                                                       {"tolerance_class", opt(t.tolerance_class)},
                                                       {"standard", t.standard},
                                                       {"basic_major_diameter", t.basic_major_diameter},
                                                       {"basic_pitch_diameter", t.basic_pitch_diameter},
                                                       {"basic_minor_diameter", t.basic_minor_diameter},
                                                       {"designation", t.designation},
                                                       {"pitch_diameter_limits", pair_values(t.pitch_diameter_limits)},
                                                       {"major_diameter_limits", opt(t.major_diameter_limits)},
                                                       {"minor_diameter_limits", opt(t.minor_diameter_limits)}};
    fields.insert(fields.end(), more.begin(), more.end());
    return attrs(fields);
}

V trapezoidal_attrs(const TrapezoidalThread& t) {
    return attrs({{"thread_size", t.thread_size},
                  {"external", t.external},
                  {"length", t.length},
                  {"diameter", t.diameter},
                  {"thread_angle", t.thread_angle},
                  {"pitch", t.pitch},
                  {"lead", t.lead},
                  {"apex_radius", t.apex_radius},
                  {"root_radius", t.root_radius},
                  {"hand", t.hand},
                  {"end_finishes", finishes(t.end_finishes)},
                  {"interference", t.interference},
                  {"manufacturing_compensation", t.manufacturing_compensation}});
}

V bottle_attrs(const PlasticBottleThread& t) {
    return attrs({{"thread_size", t.thread_size},
                  {"external", t.external},
                  {"bottle_type", t.bottle_type},
                  {"hand", t.hand},
                  {"interference", t.interference},
                  {"manufacturing_compensation", t.manufacturing_compensation},
                  {"style", opt(t.style)},
                  {"diameter", t.diameter},
                  {"finish", opt(t.finish)},
                  {"major_diameter", t.major_diameter},
                  {"minor_diameter", t.minor_diameter},
                  {"tpi", t.tpi},
                  {"root_width", t.root_width},
                  {"apex_width", t.apex_width},
                  {"apex_offset", t.apex_offset},
                  {"apex_radius", t.apex_radius},
                  {"root_radius", t.root_radius},
                  {"pitch", t.pitch},
                  {"starts", t.starts},
                  {"lead", t.lead},
                  {"length", t.length},
                  {"bottle_major_diameter", opt(t.bottle_major_diameter)},
                  {"bottle_minor_diameter", opt(t.bottle_minor_diameter)},
                  {"closure_minor_diameter", opt(t.closure_minor_diameter)}});
}

V match_values(const std::vector<PlasticBottleThreadMatch>& matches) {
    std::vector<V> out;
    for (const auto& m : matches) {
        out.push_back(V::list({m.size, m.bottle_type, pair_values(m.major_diameter), pair_values(m.neck_diameter),
                               m.pitch, opt(m.min_turns), m.error}));
    }
    return V::list(out);
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    // ---- helpers
    add_value("bdw_thread_is_safe", [] {
        std::vector<V> out;
        for (const char* s : {"1/4", "1 1/2", "12.5", "", "M6", "12345678901", "1234567890", "3/8-16", ".5"}) {
            out.emplace_back(is_safe(s));
        }
        return V::list(out);
    });
    add_value("bdw_thread_imperial_str_to_float", [] {
        std::vector<V> out;
        for (const char* s : {"1/4", "1 1/2", "3", ".5", "M6", "#10", "1  1/2", "1//2", " 3/4 "}) {
            out.push_back(num(imperial_str_to_float(s)));
        }
        return V::list(out);
    });
    add_value("bdw_thread_imperial_zero_division", [] { return error_of([] { imperial_str_to_float("1/0"); }); });
    add_value("bdw_thread_imperial_syntax", [] { return num(imperial_str_to_float("1..2")); });

    // ---- Thread
    add("bdw_thread_thread_default", [] { return Thread(5, 0.5, 4, 1.5, 2, 4); });
    add_value("bdw_thread_thread_default_attrs", [] { return thread_attrs(Thread(5, 0.5, 4, 1.5, 2, 4)); });
    add_value("bdw_thread_thread_default_children", [] { return children_values(Thread(5, 0.5, 4, 1.5, 2, 4)); });
    add_value("bdw_thread_thread_profile", [] { return thread_profile_values(Thread(5, 0.5, 4, 1.5, 2, 4).thread_profile); });
    add_value("bdw_thread_thread_profile_left", [] {
        return thread_profile_values(
            Thread(5, 0.5, 4, 1.5, 2, 4, {.apex_offset = 0.3, .hand = "left", .simple = true}).thread_profile);
    });
    add("bdw_thread_thread_no_interference", [] { return Thread(5, 0.5, 4, 1.5, 2, 4, {.interference = 0}); });
    add("bdw_thread_thread_left", [] {
        return Thread(5, 0.5, 4, 1.5, 2, 5, {.hand = "left", .end_finishes = {"fade", "fade"}});
    });
    add("bdw_thread_thread_internal", [] { return Thread(4, 0.5, 5, 1.5, 2, 4, {.end_finishes = {"square", "chamfer"}}); });
    add("bdw_thread_thread_offset", [] {
        return Thread(5, 0.4, 4, 1.6, 2, 3, {.apex_offset = 0.3, .end_finishes = {"chamfer", "square"}});
    });
    add("bdw_thread_thread_rotated", [] {
        return Thread(5, 0.5, 4, 1.5, 2, 4, {.rotation = Rotation(90, 0, 0), .align = Align3(Align::MAX)});
    });
    add("bdw_thread_thread_compensation", [] {
        return Thread(5, 0.5, 4, 1.5, 2, 4, {.manufacturing_compensation = 0.2});
    });
    add("bdw_thread_thread_short_fade", [] { return Thread(5, 0.5, 4, 1.5, 2, 1, {.end_finishes = {"fade", "fade"}}); });
    add_value("bdw_thread_thread_simple", [] { return thread_attrs(Thread(5, 0.5, 4, 1.5, 2, 4, {.simple = true})); });
    add_value("bdw_thread_thread_bad_finish", [] {
        return error_of([] { Thread(5, 0.5, 4, 1.5, 2, 4, {.end_finishes = {"raw", "x"}}); });
    });
    add_value("bdw_thread_thread_taper", [] { return error_of([] { Thread(5, 0.5, 4, 1.5, 2, 4, {.taper_angle = 5}); }); });

    // ---- IsoThread
    add("bdw_thread_iso_m6", [] { return IsoThread(6, 1, 10); });
    add_value("bdw_thread_iso_m6_attrs", [] { return iso_attrs(IsoThread(6, 1, 10)); });
    add("bdw_thread_iso_m10_internal", [] {
        return IsoThread(10, 1.5, 8, {.external = false, .end_finishes = {"chamfer", "fade"}});
    });
    add("bdw_thread_iso_m8_left", [] { return IsoThread(8, 1.25, 6, true, "left"); });
    add("bdw_thread_iso_raw", [] { return IsoThread(5, 0.8, 4, {.end_finishes = {"raw", "raw"}}); });
    add("bdw_thread_iso_square", [] { return IsoThread(5, 0.8, 4, {.end_finishes = {"square", "square"}}); });
    add("bdw_thread_iso_chamfer", [] { return IsoThread(12, 1.75, 7, {.end_finishes = {"chamfer", "chamfer"}}); });
    add("bdw_thread_iso_fade", [] { return IsoThread(4, 0.7, 5, {.end_finishes = {"fade", "fade"}}); });
    add("bdw_thread_iso_internal_square", [] {
        return IsoThread(6, 1, 5, {.external = false, .end_finishes = {"square", "square"}});
    });
    add("bdw_thread_iso_rotated", [] {
        return IsoThread(6, 1, 5, {.rotation = Rotation(90, 0, 0), .align = Align3(Align::MIN, Align::CENTER, Align::MAX)});
    });
    add("bdw_thread_iso_no_interference", [] { return IsoThread(6, 1, 5, {.interference = 0}); });
    add("bdw_thread_iso_compensation", [] {
        return IsoThread(6, 1, 5, {.external = false, .manufacturing_compensation = 0.15});
    });
    add_value("bdw_thread_iso_simple", [] { return iso_attrs(IsoThread(20, 2.5, 30, {.simple = true})); });
    add("bdw_thread_iso_imperial", [] { return IsoThread(0.25 * IN, IN / 20, 0.5 * IN); });
    add_value("bdw_thread_iso_bad_hand", [] { return error_of([] { IsoThread(6, 1, 5, true, "up"); }); });
    add_value("bdw_thread_iso_bad_finish", [] {
        return error_of([] { IsoThread(6, 1, 5, {.end_finishes = {"fade", "taper"}}); });
    });

    // ---- WhitworthThread, BSPPThread
    add("bdw_thread_whitworth", [] { return WhitworthThread(20, 2, 6); });
    add_value("bdw_thread_whitworth_attrs", [] { return whitworth_attrs(WhitworthThread(20, 2, 6)); });
    add("bdw_thread_whitworth_internal", [] {
        return WhitworthThread(20, 2, 6, {.external = false, .end_finishes = {"fade", "fade"}});
    });
    add("bdw_thread_whitworth_truncated", [] {
        return WhitworthThread(20, 2, 6, {.end_finishes = {"square", "chamfer"}, .crest_truncation = 0.3});
    });
    add("bdw_thread_whitworth_truncated_internal", [] {
        return WhitworthThread(20, 2, 6,
                               {.external = false, .hand = "left", .end_finishes = {"chamfer", "square"}, .crest_truncation = 0.25});
    });
    add_value("bdw_thread_whitworth_profile", [] {
        return thread_profile_values(WhitworthThread(20, 2, 6, {.simple = true}).thread_profile);
    });
    add_value("bdw_thread_whitworth_profile_truncated", [] {
        return thread_profile_values(
            WhitworthThread(20, 2, 6, {.external = false, .interference = 0, .simple = true, .crest_truncation = 0.4})
                .thread_profile);
    });
    add_value("bdw_thread_whitworth_bad_truncation", [] {
        return error_of([] { WhitworthThread(20, 2, 6, {.crest_truncation = 1.5}); });
    });
    add_value("bdw_thread_whitworth_bad_hand", [] { return error_of([] { WhitworthThread(20, 2, 6, true, "LEFT"); }); });
    add_value("bdw_thread_bspp_sizes", [] { return V::list_of(BSPPThread::sizes()); });
    add_value("bdw_thread_bspp_normalize", [] {
        std::vector<V> out;
        for (const char* s : {"G1/2", "g 1/2", " 1 1/4\" ", "G  2   1/2", "1\xE2\x80\xB3"}) out.emplace_back(BSPPThread::_normalize_size(s));
        return V::list(out);
    });
    add("bdw_thread_bspp_half", [] { return BSPPThread("G1/2", 10); });
    add_value("bdw_thread_bspp_half_attrs", [] { return bspp_attrs(BSPPThread("G1/2", 10, {.simple = true})); });
    add("bdw_thread_bspp_internal", [] {
        return BSPPThread("1/4", 8, {.external = false, .end_finishes = {"chamfer", "fade"}});
    });
    add_value("bdw_thread_bspp_internal_attrs", [] {
        return bspp_attrs(BSPPThread("1/4", 8, {.external = false, .simple = true}));
    });
    add_value("bdw_thread_bspp_class_b", [] {
        return bspp_attrs(BSPPThread("G 1", 12, {.tolerance_class = "b", .hand = "left", .simple = true}));
    });
    add("bdw_thread_bspp_crest", [] { return BSPPThread("G1/8", 5, {.crest_diameter = 9.6}); });
    add_value("bdw_thread_bspp_crest_internal", [] {
        return bspp_attrs(BSPPThread("G3/8", 8, {.external = false, .simple = true, .crest_diameter = 15.0}));
    });
    add_value("bdw_thread_bspp_bad_size", [] { return error_of([] { BSPPThread("G7", 10); }); });
    add_value("bdw_thread_bspp_bad_class", [] { return error_of([] { BSPPThread("G1/2", 10, {.tolerance_class = "C"}); }); });
    add_value("bdw_thread_bspp_internal_class", [] {
        return error_of([] { BSPPThread("G1/2", 10, {.external = false, .tolerance_class = "A"}); });
    });
    add_value("bdw_thread_bspp_bad_crest", [] { return error_of([] { BSPPThread("G1/2", 10, {.crest_diameter = 30}); }); });

    // ---- TrapezoidalThread, AcmeThread, MetricTrapezoidalThread
    add("bdw_thread_trapezoidal", [] { return TrapezoidalThread(12, 2, 30, 8); });
    add_value("bdw_thread_trapezoidal_attrs", [] { return trapezoidal_attrs(TrapezoidalThread(12, 2, 30, 8)); });
    add("bdw_thread_trapezoidal_starts", [] {
        return TrapezoidalThread(12, 2, 30, 8, {.starts = 2, .end_finishes = {"square", "square"}});
    });
    add_value("bdw_thread_trapezoidal_starts_children", [] {
        return children_values(TrapezoidalThread(10, 2, 30, 6, {.starts = 3, .end_finishes = {"raw", "raw"}}));
    });
    add("bdw_thread_trapezoidal_internal", [] {
        return TrapezoidalThread(16, 4, 30, 10, {.external = false, .hand = "left", .rotation = Rotation(0, 90, 0)});
    });
    add_value("bdw_thread_trapezoidal_bad_hand", [] {
        return error_of([] { TrapezoidalThread(12, 2, 30, 8, {.hand = "x"}); });
    });
    add_value("bdw_thread_acme_sizes", [] { return V::list_of(AcmeThread::sizes()); });
    add_value("bdw_thread_acme_pitch", [] {
        std::vector<V> out;
        for (const auto& [k, v] : AcmeThread::acme_pitch) out.push_back(V::list({k, v}));
        return V::list(out);
    });
    add("bdw_thread_acme", [] { return AcmeThread("1/2", 20); });
    add_value("bdw_thread_acme_attrs", [] { return trapezoidal_attrs(AcmeThread("1 1/4", 10, false)); });
    add("bdw_thread_acme_internal", [] {
        return AcmeThread("3/8", 10, {.external = false, .end_finishes = {"chamfer", "square"}});
    });
    add_value("bdw_thread_acme_bad_size", [] { return error_of([] { AcmeThread("5/32", 10); }); });
    add_value("bdw_thread_acme_bad_text", [] { return error_of([] { AcmeThread("M8", 10); }); });
    add_value("bdw_thread_metric_sizes", [] { return V::list_of(MetricTrapezoidalThread::sizes()); });
    add("bdw_thread_metric", [] { return MetricTrapezoidalThread("8x1.5", 15); });
    add_value("bdw_thread_metric_attrs", [] {
        return trapezoidal_attrs(MetricTrapezoidalThread("20x4", 15, {.hand = "left"}));
    });
    add("bdw_thread_metric_internal", [] {
        return MetricTrapezoidalThread("14x3", 9, {.external = false, .end_finishes = {"raw", "fade"}});
    });
    add_value("bdw_thread_metric_bad_size", [] { return error_of([] { MetricTrapezoidalThread("8x2", 10); }); });

    // ---- PlasticBottleThread
    add_value("bdw_thread_bottle_types", [] { return V::list_of(PlasticBottleThread::bottle_types()); });
    add_value("bdw_thread_bottle_sizes", [] { return V::list_of(PlasticBottleThread::sizes()); });
    add_value("bdw_thread_bottle_sizes_pco", [] { return V::list_of(PlasticBottleThread::sizes("pco1881")); });
    add_value("bdw_thread_bottle_sizes_bad", [] { return error_of([] { PlasticBottleThread::sizes("x"); }); });
    add_value("bdw_thread_bottle_identify", [] {
        return match_values(PlasticBottleThread::identify(30, std::nullopt, 4, std::nullopt, 1.5));
    });
    add_value("bdw_thread_bottle_identify_major", [] {
        return match_values(PlasticBottleThread::identify(std::nullopt, 38.2, std::nullopt, 6, std::nullopt, "L", 0.3));
    });
    add_value("bdw_thread_bottle_identify_pco", [] {
        return match_values(PlasticBottleThread::identify(std::nullopt, 27.4, 2.7));
    });
    add_value("bdw_thread_bottle_identify_turns", [] {
        return match_values(PlasticBottleThread::identify(36, std::nullopt, std::nullopt, std::nullopt, 1.0));
    });
    add_value("bdw_thread_bottle_identify_errors", [] {
        return V::list({error_of([] { PlasticBottleThread::identify(); }),
                        error_of([] { PlasticBottleThread::identify(30, std::nullopt, 4, 6); }),
                        error_of([] { PlasticBottleThread::identify(30, std::nullopt, std::nullopt, std::nullopt,
                                                                    std::nullopt, "X"); })});
    });
    add("bdw_thread_bottle_m38", [] { return PlasticBottleThread("M38SP444"); });
    add_value("bdw_thread_bottle_m38_attrs", [] {
        return bottle_attrs(PlasticBottleThread("M38SP444", {.external = false, .manufacturing_compensation = 0.2}));
    });
    add("bdw_thread_bottle_m38_internal", [] {
        return PlasticBottleThread("M38SP444", {.external = false, .manufacturing_compensation = 0.2});
    });
    add("bdw_thread_bottle_l28", [] { return PlasticBottleThread("L28SP400", {.hand = "left", .interference = 0.1}); });
    add("bdw_thread_bottle_pco", [] { return PlasticBottleThread("28", {.bottle_type = "pco1881"}); });
    add_value("bdw_thread_bottle_pco_attrs", [] {
        return bottle_attrs(PlasticBottleThread("28mm", {.external = false, .bottle_type = "pco1881"}));
    });
    add("bdw_thread_bottle_pco_internal", [] {
        return PlasticBottleThread("28MM", {.external = false, .bottle_type = "pco1881"});
    });
    add_value("bdw_thread_bottle_bad_size", [] { return error_of([] { PlasticBottleThread("X38SP444"); }); });
    add_value("bdw_thread_bottle_bad_finish", [] { return error_of([] { PlasticBottleThread("M38SP999"); }); });
    add_value("bdw_thread_bottle_bad_diameter", [] { return error_of([] { PlasticBottleThread("M37SP400"); }); });
    add_value("bdw_thread_bottle_bad_type", [] {
        return error_of([] { PlasticBottleThread("28", {.bottle_type = "din"}); });
    });
    add_value("bdw_thread_bottle_bad_pco", [] {
        return error_of([] { PlasticBottleThread("38", {.bottle_type = "pco1881"}); });
    });
}
