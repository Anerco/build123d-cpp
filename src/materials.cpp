// bd_materials, ported: core (Range, RangeMaterial), finishes, processes, FinishedMaterial, the
// catalog (generated into material_catalog.inc by scripts/gen_materials_data.py), the registry
// and applicability. The looks (bd_materials.pbr) are in material_looks.cpp.
//
// The library itself only reads the public tables through the functions below (function-local
// statics), so they can be used during static initialization.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <unordered_map>

#include "build123d/material_catalog.hpp"
#include "build123d/materials.hpp"
#include "build123d/warnings.hpp"
#include "pbr_internal.hpp"
#include "pyformat.hpp"

namespace build123d {

namespace {

using detail::pbr::repr_floats;
using detail::pbr::repr_str;

using detail::pbr::py_lower;
using detail::pbr::py_strip;

/// registry._normalize: strip, lower, spaces and hyphens to underscores.
std::string normalize_name(const std::string& name) {
    std::string key = py_lower(py_strip(name));
    for (char& c : key) {
        if (c == ' ' || c == '-') c = '_';
    }
    return key;
}

std::string join(const std::vector<std::string>& parts, const std::string& sep) {
    std::string out;
    for (std::size_t i = 0; i < parts.size(); ++i) out += (i ? sep : "") + parts[i];
    return out;
}

std::string repr_pair(const std::pair<double, double>& p) { return repr_floats({p.first, p.second}, true); }

// ---------------------------------------------------------------------------
// core.py tables
// ---------------------------------------------------------------------------

using UnitTable = std::vector<std::pair<std::string, std::string>>;

const UnitTable& mechanical_units() {
    static const UnitTable t = {
        {"areal_density", "g/m²"},
        {"compressive_strength_parallel", "MPa"},
        {"density", "kg/m³"},
        {"elongation_at_break", "%"},
        {"hardness", "per hardness_scale"},
        {"janka_hardness", "N"},
        {"modulus_of_elasticity", "GPa"},
        {"modulus_of_rupture", "MPa"},
        {"poisson_ratio", ""},
        {"shear_modulus", "GPa"},
        {"shear_strength", "MPa"},
        {"tensile_strength", "MPa"},
        {"thickness", "mm"},
        {"yield_strength", "MPa"},
    };
    return t;
}

const UnitTable& thermal_units() {
    static const UnitTable t = {
        {"glass_transition_temperature", "°C"},
        {"heat_deflection_temperature", "°C"},
        {"max_service_temp", "°C"},
        {"melting_temperature", "°C"},
        {"specific_heat_capacity", "J/(kg·K)"},
        {"thermal_conductivity", "W/(m·K)"},
        {"thermal_expansion", "1/K"},
    };
    return t;
}

const std::set<std::string>& ferrous() {
    static const std::set<std::string> s = {"mild_steel", "alloy_steel", "tool_steel", "spring_steel"};
    return s;
}
const std::set<std::string>& metal_set() {
    static const std::set<std::string> s = {"metal"};
    return s;
}
const std::set<std::string>& paintable() {
    static const std::set<std::string> s = {"metal", "plastic", "resin", "wood"};
    return s;
}
const std::set<std::string>& coatable_poly() {
    static const std::set<std::string> s = {"metal", "plastic", "resin"};
    return s;
}
const std::set<std::string>& everything() {
    static const std::set<std::string> s = {"metal", "plastic", "resin", "wood", "glass", "paper", "textile"};
    return s;
}

// ---------------------------------------------------------------------------
// The material classes
// ---------------------------------------------------------------------------

struct ClassRow {
    const char* kind;
    const char* category;
    const char* fields;
};

const ClassRow kClasses[] = {
#define B3D_MATERIAL_CLASSES
#include "material_catalog.inc"
#undef B3D_MATERIAL_CLASSES
};

const ClassRow& class_row(const std::string& kind) {
    for (const ClassRow& row : kClasses) {
        if (kind == row.kind) return row;
    }
    throw std::invalid_argument("unknown material class " + repr_str(kind));
}

std::vector<std::string> split(const char* text) {
    std::vector<std::string> out;
    std::string cur;
    for (const char* p = text; *p; ++p) {
        if (*p == ' ') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur += *p;
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

using RangeField = std::optional<Range> RangeMaterial::*;

const std::vector<std::pair<std::string, RangeField>>& range_fields() {
    static const std::vector<std::pair<std::string, RangeField>> t = {
        {"tensile_strength", &RangeMaterial::tensile_strength},
        {"modulus_of_elasticity", &RangeMaterial::modulus_of_elasticity},
        {"shear_modulus", &RangeMaterial::shear_modulus},
        {"poisson_ratio", &RangeMaterial::poisson_ratio},
        {"specific_heat_capacity", &RangeMaterial::specific_heat_capacity},
        {"max_service_temp", &RangeMaterial::max_service_temp},
        {"thermal_expansion", &RangeMaterial::thermal_expansion},
        {"thermal_conductivity", &RangeMaterial::thermal_conductivity},
        {"yield_strength", &RangeMaterial::yield_strength},
        {"shear_strength", &RangeMaterial::shear_strength},
        {"elongation_at_break", &RangeMaterial::elongation_at_break},
        {"glass_transition_temperature", &RangeMaterial::glass_transition_temperature},
        {"heat_deflection_temperature", &RangeMaterial::heat_deflection_temperature},
        {"hardness", &RangeMaterial::hardness},
        {"melting_temperature", &RangeMaterial::melting_temperature},
        {"modulus_of_rupture", &RangeMaterial::modulus_of_rupture},
        {"compressive_strength_parallel", &RangeMaterial::compressive_strength_parallel},
        {"janka_hardness", &RangeMaterial::janka_hardness},
        {"areal_density", &RangeMaterial::areal_density},
        {"thickness", &RangeMaterial::thickness},
    };
    return t;
}

RangeField range_field(const std::string& name) {
    for (const auto& [field, member] : range_fields()) {
        if (field == name) return member;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// finishes.py tables
// ---------------------------------------------------------------------------

using finishes::Chemical;
using finishes::Coating;
using finishes::Finish;
using finishes::Marking;
using finishes::Mechanical;
using finishes::MetalPlating;

const std::map<Mechanical, Finish>& mechanical_finishes() {
    static const std::map<Mechanical, Finish> t = {
        {Mechanical::BEAD_BLAST, {"Bead Blast"}},
        {Mechanical::BRUSHED, {"Brushed"}},
        {Mechanical::FINE_SANDING, {"Fine Sanding"}},
        {Mechanical::SMOOTH_MACHINING, {"Smooth machining (Ra1.6µm)", "as-machined roughness spec"}},
        {Mechanical::ELECTROPOLISHED, {"Electropolished"}},
    };
    return t;
}

const std::map<Chemical, Finish>& chemical_finishes() {
    static const std::map<Chemical, Finish> t = {
        {Chemical::ANODIZED,
         {"Anodized",
          "typical colors: natural, black, gray, gold, rose gold, red, blue, champagne, brown, purple, green, orange"}},
        {Chemical::CHEM_FILM, {"Chemical conversion coat (Chem film)", "typical colors: clear, gold"}},
        {Chemical::CONDUCTIVE_OXIDATION, {"Electrically conductive oxidation", "color: clear"}},
        {Chemical::BLACK_OXIDE, {"Black oxide", "color: black"}},
        {Chemical::PASSIVATION, {"Passivation"}},
        {Chemical::PICKLING, {"Pickling"}},
        {Chemical::DYEING, {"Dyeing", "anodized aluminum, or dyeable polymers (esp. nylon)"}},
    };
    return t;
}

const std::map<MetalPlating, Finish>& metal_plating_finishes() {
    static const std::map<MetalPlating, Finish> t = {
        {MetalPlating::CHROME_PLATING, {"Chrome plating", "color: chrome"}},
        {MetalPlating::GOLD_PLATING, {"Gold plating", "color: gold"}},
        {MetalPlating::NICKEL_PLATING, {"Nickel plating", "color: nickel"}},
        {MetalPlating::SILVER_PLATING, {"Silver plating", "color: silver"}},
        {MetalPlating::TIN_PLATING, {"Tin plating", "color: silver"}},
        {MetalPlating::PVD, {"PVD (Physical Vapor Deposition)", "typical colors: gold, rose gold, black, gunmetal, blue"}},
        {MetalPlating::ZINC_PLATING,
         {"Zinc plating", "corrosion protection for ferrous steels; typical colors: clear, blue, yellow, black"}},
        {MetalPlating::VACUUM_PLATING, {"Vacuum plating"}},
    };
    return t;
}

const std::map<Coating, Finish>& coating_finishes() {
    static const std::map<Coating, Finish> t = {
        {Coating::POWDER_COAT, {"Powder coat"}},
        {Coating::ELECTROPHORESIS, {"Electrophoresis"}},
        {Coating::SPRAY_PAINT, {"Spray painting"}},
    };
    return t;
}

const std::map<Marking, Finish>& marking_finishes() {
    static const std::map<Marking, Finish> t = {
        {Marking::LASER_ENGRAVING, {"Laser engraving"}},
        {Marking::ETCHING, {"Etching"}},
        {Marking::SILKSCREEN, {"Silkscreen"}},
    };
    return t;
}

std::vector<Finish> all_finishes() {
    std::vector<Finish> out;
    for (const auto& [k, f] : mechanical_finishes()) out.push_back(f);
    for (const auto& [k, f] : chemical_finishes()) out.push_back(f);
    for (const auto& [k, f] : metal_plating_finishes()) out.push_back(f);
    for (const auto& [k, f] : coating_finishes()) out.push_back(f);
    for (const auto& [k, f] : marking_finishes()) out.push_back(f);
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// core.py
// ---------------------------------------------------------------------------

Range::Range(double min_value, double max_value) : min(min_value), max(max_value) {
    if (min > max) throw std::invalid_argument("Range min " + pyfmt::repr(min) + " > max " + pyfmt::repr(max));
}

const Range NOT_SUITABLE = Range(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
const std::vector<std::pair<std::string, std::string>> MECHANICAL_PROPERTY_UNITS = mechanical_units();
const std::vector<std::pair<std::string, std::string>> THERMAL_PROPERTY_UNITS = thermal_units();
const std::vector<std::pair<std::string, std::string>> PROPERTY_UNITS = [] {
    UnitTable t = mechanical_units();
    t.insert(t.end(), thermal_units().begin(), thermal_units().end());
    return t;
}();
const std::set<std::string> ALLOWED_CATEGORIES = {"generic", "metal", "plastic", "resin", "wood", "glass", "paper", "textile"};
const std::set<std::string> FERROUS = ferrous();
const std::set<std::string> METAL = metal_set();
const std::set<std::string> PAINTABLE = paintable();
const std::set<std::string> COATABLE_POLY = coatable_poly();
const std::set<std::string> EVERYTHING = everything();

std::vector<std::string> RangeMaterial::fields() const { return split(class_row(kind).fields); }

std::optional<Range> RangeMaterial::property(const std::string& field) const {
    const RangeField member = range_field(field);
    if (member == nullptr) return std::nullopt;
    const auto have = fields();
    if (std::find(have.begin(), have.end(), field) == have.end()) return std::nullopt;
    return this->*member;
}

std::string RangeMaterial::str() const {
    const auto have = fields();
    auto has = [&have](const std::string& field) { return std::find(have.begin(), have.end(), field) != have.end(); };
    const std::string scale = has("hardness_scale") ? hardness_scale : "";
    std::size_t width = 0;
    for (const UnitTable* units : {&mechanical_units(), &thermal_units()}) {
        for (const auto& [field, unit] : *units) {
            if (has(field)) width = std::max(width, field.size());
        }
    }
    std::string out = name;
    const std::pair<const char*, const UnitTable*> groups[] = {{"mechanical", &mechanical_units()},
                                                               {"thermal", &thermal_units()}};
    for (const auto& [group, units] : groups) {
        std::vector<std::pair<std::string, std::string>> names;
        for (const auto& entry : *units) {
            if (has(entry.first)) names.push_back(entry);
        }
        if (names.empty()) continue;
        out += "\n  # " + std::string(group) + " properties";
        for (const auto& [field, field_unit] : names) {
            std::string unit = field == "hardness" ? scale : field_unit;
            std::string body;
            if (field == "density") {
                body = pyfmt::format_g(density, 6);
            } else {
                const RangeField member = range_field(field);
                const std::optional<Range>& value = this->*member;
                if (!value) {
                    body = "missing";
                    unit.clear();
                } else if (std::isnan(value->min)) {
                    body = "n/a";
                    unit.clear();
                } else {
                    body = pyfmt::format_g(value->min, 6) + " to " + pyfmt::format_g(value->max, 6);
                }
            }
            std::string line = "  " + field + std::string(width - field.size(), ' ') + "  " + body + " " + unit;
            while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) line.pop_back();
            out += "\n" + line;
        }
    }
    return out;
}

RangeMaterial with_density(const RangeMaterial& material, std::optional<double> density) {
    RangeMaterial out = material;
    if (density) out.density = *density;
    return out;
}

// ---------------------------------------------------------------------------
// finishes.py
// ---------------------------------------------------------------------------

namespace finishes {

const std::map<Mechanical, Finish> MECHANICAL_FINISHES = mechanical_finishes();
const std::map<Chemical, Finish> CHEMICAL_FINISHES = chemical_finishes();
const std::map<MetalPlating, Finish> METAL_PLATING_FINISHES = metal_plating_finishes();
const std::map<Coating, Finish> COATING_FINISHES = coating_finishes();
const std::map<Marking, Finish> MARKING_FINISHES = marking_finishes();
const std::vector<Finish> ALL_FINISHES = all_finishes();

std::string Finish::repr() const {
    return "Finish(name=" + repr_str(name) + ", notes=" + (notes ? repr_str(*notes) : "None") + ")";
}

std::string AppliedFinish::repr() const {
    std::string sheen_repr = "None";
    if (sheen) sheen_repr = *sheen == Sheen::GLOSS ? "<Sheen.GLOSS: 1>" : "<Sheen.MATTE: 2>";
    return "AppliedFinish(finish=" + finish.repr() + ", color=" + (color ? color->repr() : "None") +
           ", sheen=" + sheen_repr + ", scale=" + repr_pair(scale) + ", rotation=" + pyfmt::repr(rotation) + ")";
}

bool AppliedFinish::operator==(const AppliedFinish& o) const {
    return finish == o.finish && color == o.color && sheen == o.sheen && scale == o.scale && rotation == o.rotation;
}

AppliedFinish bead_blast(std::pair<double, double> scale) {
    return {mechanical_finishes().at(Mechanical::BEAD_BLAST), std::nullopt, std::nullopt, scale};
}
AppliedFinish brushed(std::pair<double, double> scale, double rotation) {
    return {mechanical_finishes().at(Mechanical::BRUSHED), std::nullopt, std::nullopt, scale, rotation};
}
AppliedFinish fine_sanding(std::pair<double, double> scale) {
    return {mechanical_finishes().at(Mechanical::FINE_SANDING), std::nullopt, std::nullopt, scale};
}
AppliedFinish smooth_machining() { return {mechanical_finishes().at(Mechanical::SMOOTH_MACHINING)}; }
AppliedFinish electropolish() { return {mechanical_finishes().at(Mechanical::ELECTROPOLISHED)}; }
AppliedFinish anodize(std::optional<ColorSpec> color) { return {chemical_finishes().at(Chemical::ANODIZED), std::move(color)}; }
AppliedFinish chem_film() { return {chemical_finishes().at(Chemical::CHEM_FILM)}; }
AppliedFinish conductive_oxidation() { return {chemical_finishes().at(Chemical::CONDUCTIVE_OXIDATION)}; }
AppliedFinish black_oxide() { return {chemical_finishes().at(Chemical::BLACK_OXIDE)}; }
AppliedFinish passivate() { return {chemical_finishes().at(Chemical::PASSIVATION)}; }
AppliedFinish pickle() { return {chemical_finishes().at(Chemical::PICKLING)}; }
AppliedFinish dye(std::optional<ColorSpec> color) { return {chemical_finishes().at(Chemical::DYEING), std::move(color)}; }
AppliedFinish chrome() { return {metal_plating_finishes().at(MetalPlating::CHROME_PLATING)}; }
AppliedFinish gold_plate() { return {metal_plating_finishes().at(MetalPlating::GOLD_PLATING)}; }
AppliedFinish nickel_plate() { return {metal_plating_finishes().at(MetalPlating::NICKEL_PLATING)}; }
AppliedFinish silver_plate() { return {metal_plating_finishes().at(MetalPlating::SILVER_PLATING)}; }
AppliedFinish tin_plate() { return {metal_plating_finishes().at(MetalPlating::TIN_PLATING)}; }
AppliedFinish pvd(std::optional<ColorSpec> color) { return {metal_plating_finishes().at(MetalPlating::PVD), std::move(color)}; }
AppliedFinish zinc_plate(std::optional<ColorSpec> color) {
    return {metal_plating_finishes().at(MetalPlating::ZINC_PLATING), std::move(color)};
}
AppliedFinish vacuum_plating(std::optional<ColorSpec> color, std::optional<Sheen> sheen) {
    return {metal_plating_finishes().at(MetalPlating::VACUUM_PLATING), std::move(color), sheen};
}
AppliedFinish powder_coat(std::optional<ColorSpec> color, std::optional<Sheen> sheen) {
    return {coating_finishes().at(Coating::POWDER_COAT), std::move(color), sheen};
}
AppliedFinish spray_paint(std::optional<ColorSpec> color, std::optional<Sheen> sheen) {
    return {coating_finishes().at(Coating::SPRAY_PAINT), std::move(color), sheen};
}
AppliedFinish electrophoresis(std::optional<ColorSpec> color) {
    return {coating_finishes().at(Coating::ELECTROPHORESIS), std::move(color)};
}
AppliedFinish laser_engrave() { return {marking_finishes().at(Marking::LASER_ENGRAVING)}; }
AppliedFinish etch() { return {marking_finishes().at(Marking::ETCHING)}; }
AppliedFinish silkscreen(std::optional<ColorSpec> color) { return {marking_finishes().at(Marking::SILKSCREEN), std::move(color)}; }

}  // namespace finishes

std::string FinishSpec::repr() const {
    if (!list_) return items_.at(0).repr();
    std::vector<std::string> parts;
    for (const auto& f : items_) parts.push_back(f.repr());
    return "[" + join(parts, ", ") + "]";
}

// ---------------------------------------------------------------------------
// processes.py
// ---------------------------------------------------------------------------

namespace processes {

namespace {

const char* const kProcessNames[] = {"FDM", "SLS", "MJF", "SLM", "VAT", "MOLDED", "MACHINED", "CAST", "WROUGHT"};
const char* const kProcessValues[] = {"fdm", "sls", "mjf", "slm", "vat", "molded", "machined", "cast", "wrought"};

}  // namespace

std::string to_string(Process process) { return kProcessValues[static_cast<int>(process)]; }

std::string AppliedProcess::repr() const {
    const int i = static_cast<int>(process);
    return std::string("AppliedProcess(process=<Process.") + kProcessNames[i] + ": '" + kProcessValues[i] +
           "'>, scale=" + repr_pair(scale) + ", rotation=" + pyfmt::repr(rotation) +
           ", pitch_mm=" + (pitch_mm ? pyfmt::repr(*pitch_mm) : "None") + ", skin=" + (skin ? "True" : "False") +
           ", mm_per_uv=" + pyfmt::repr(mm_per_uv) + ")";
}

bool AppliedProcess::operator==(const AppliedProcess& o) const {
    return process == o.process && scale == o.scale && rotation == o.rotation && pitch_mm == o.pitch_mm &&
           skin == o.skin && mm_per_uv == o.mm_per_uv;
}

AppliedProcess fdm(double layer_height_mm, double rotation, double mm_per_uv) {
    return {Process::FDM, {1.0, 1.0}, rotation, layer_height_mm, false, mm_per_uv};
}
AppliedProcess fdm_skin(double line_width_mm, double rotation) {
    return {Process::FDM, {1.0, 1.0}, rotation, line_width_mm, true};
}
AppliedProcess sls(std::pair<double, double> scale) { return {Process::SLS, scale}; }
AppliedProcess mjf(std::pair<double, double> scale) { return {Process::MJF, scale}; }
AppliedProcess slm(std::pair<double, double> scale) { return {Process::SLM, scale}; }
AppliedProcess vat() { return {Process::VAT}; }
AppliedProcess molded() { return {Process::MOLDED}; }
AppliedProcess machined() { return {Process::MACHINED}; }
AppliedProcess cast() { return {Process::CAST}; }
AppliedProcess wrought() { return {Process::WROUGHT}; }

}  // namespace processes

// ---------------------------------------------------------------------------
// finished.py
// ---------------------------------------------------------------------------

FinishedMaterial::FinishedMaterial(RangeMaterial material_in, std::optional<FinishSpec> finish_in,
                                   const FinishedMaterialOptions& options) {
    if (options.pbr && (finish_in || options.process || options.color || options.thickness_mm || options.opacity ||
                        options.roughness || options.scale != std::pair<double, double>{1.0, 1.0} ||
                        options.rotation != 0.0)) {
        throw std::invalid_argument(
            "pbr=... is a full override; do not combine it with finish, process, color, thickness_mm, opacity, "
            "roughness, scale, or rotation");
    }
    if (finish_in && options.process) {
        throw std::invalid_argument(
            "finish and process are mutually exclusive: a finish defines the surface, so a process (the raw as-made "
            "surface) is ignored -- pass one or the other");
    }
    material = std::move(material_in);
    finish = std::move(finish_in);
    color = options.color;
    thickness_mm = options.thickness_mm;
    opacity = options.opacity;
    roughness = options.roughness;
    if (options.process && std::holds_alternative<processes::Process>(*options.process)) {
        const auto p = std::get<processes::Process>(*options.process);
        static const char* const names[] = {"FDM", "SLS", "MJF", "SLM", "VAT", "MOLDED", "MACHINED", "CAST", "WROUGHT"};
        warn(std::string("process=Process.") + names[static_cast<int>(p)] +
             " is deprecated; call the process function instead -- process=" + processes::to_string(p) +
             "() (from bd_materials.processes), which is also the only way to pass the per-part geometry, e.g. "
             "fdm(rotation=90)");
    }
    scale = options.scale;
    rotation = options.rotation;
    if (options.process) {
        if (const auto* applied = std::get_if<processes::AppliedProcess>(&*options.process)) {
            process = *applied;
        } else {
            process = processes::AppliedProcess{std::get<processes::Process>(*options.process)};
        }
    }
    pbr_ = options.pbr;
}

FinishedMaterial::FinishedMaterial(const std::string& name) : FinishedMaterial(registry::resolve(name)) {}

std::shared_ptr<const PbrProperties> FinishedMaterial::pbr_ptr() const {
    if (pbr_) return pbr_;
    return std::make_shared<const PbrProperties>(pbr());
}

PbrProperties FinishedMaterial::pbr() const {
    if (pbr_) return *pbr_;
    return get_pbr_properties(material, finish, process,
                              {.color = color,
                               .thickness_mm = thickness_mm,
                               .opacity = opacity,
                               .roughness = roughness,
                               .scale = scale,
                               .rotation = rotation});
}

std::string FinishedMaterial::repr() const {
    std::vector<std::string> parts = {material.name};
    if (color) parts.push_back("color=" + color->repr());
    if (thickness_mm) parts.push_back("thickness_mm=" + pyfmt::repr(*thickness_mm));
    if (opacity) parts.push_back("opacity=" + pyfmt::repr(*opacity));
    if (roughness) parts.push_back("roughness=" + pyfmt::repr(*roughness));
    if (scale != std::pair<double, double>{1.0, 1.0}) parts.push_back("scale=" + repr_pair(scale));
    if (rotation != 0.0) parts.push_back("rotation=" + pyfmt::repr(rotation));
    if (finish) parts.push_back("finish=" + finish->repr());
    if (process) parts.push_back("process=" + process->repr());
    if (pbr_) parts.emplace_back("pbr=<override>");
    return "FinishedMaterial(" + join(parts, ", ") + ")";
}

// ---------------------------------------------------------------------------
// The catalog (materials/*.py)
// ---------------------------------------------------------------------------

namespace detail {

namespace {

struct GradeRow {
    const char* module;
    const char* enum_name;
    const char* member;
    int value;
    const char* kind;
    const char* name;
    double density;
    const char* family;
    bool transparent;
    const char* hardness_scale;
    const char* properties;
};

const GradeRow kGrades[] = {
#define B3D_MATERIAL_GRADES
#include "material_catalog.inc"
#undef B3D_MATERIAL_GRADES
};

RangeMaterial build_grade(const GradeRow& row) {
    RangeMaterial m;
    m.kind = row.kind;
    m.category = class_row(row.kind).category;
    m.name = row.name;
    m.density = row.density;
    if (row.family != nullptr) m.family = std::string(row.family);
    m.transparent = row.transparent;
    if (row.hardness_scale != nullptr) m.hardness_scale = row.hardness_scale;
    for (const std::string& item : split(row.properties)) {
        const auto eq = item.find('=');
        const RangeField member = range_field(item.substr(0, eq));
        const std::string value = item.substr(eq + 1);
        if (value == "None") continue;
        const auto comma = value.find(',');
        const double lo = std::strtod(value.substr(0, comma).c_str(), nullptr);
        const double hi = std::strtod(value.substr(comma + 1).c_str(), nullptr);
        m.*member = Range(lo, hi);
    }
    return m;
}

/// The grades' materials, built once.
const std::vector<RangeMaterial>& grade_materials() {
    static const std::vector<RangeMaterial> materials = [] {
        std::vector<RangeMaterial> out;
        for (const GradeRow& row : kGrades) out.push_back(build_grade(row));
        return out;
    }();
    return materials;
}

const RangeMaterial& catalog_grade(const char* module, const char* enum_name, int value) {
    const auto& materials = grade_materials();
    for (std::size_t i = 0; i < std::size(kGrades); ++i) {
        const GradeRow& row = kGrades[i];
        if (row.value == value && std::strcmp(row.enum_name, enum_name) == 0 && std::strcmp(row.module, module) == 0) {
            return materials[i];
        }
    }
    throw std::invalid_argument(std::string("no grade ") + std::to_string(value) + " of " + enum_name);
}

struct AllRow {
    const char* module;
    const char* enum_name;
    int value;
};

const AllRow kAll[] = {
#define B3D_MATERIAL_ALL
#include "material_catalog.inc"
#undef B3D_MATERIAL_ALL
};

/// materials.ALL_MATERIALS, built once.
const std::vector<RangeMaterial>& all_materials() {
    static const std::vector<RangeMaterial> materials = [] {
        std::vector<RangeMaterial> out;
        for (const AllRow& row : kAll) out.push_back(catalog_grade(row.module, row.enum_name, row.value));
        return out;
    }();
    return materials;
}

FinishedMaterial family_material(const RangeMaterial& m, const MetalOptions& o) {
    return FinishedMaterial(with_density(m, o.density), o.finish, {.process = o.process});
}
FinishedMaterial family_material(const RangeMaterial& m, const PolymerOptions& o) {
    return FinishedMaterial(with_density(m, o.density), o.finish, {.color = o.color, .process = o.process});
}
FinishedMaterial family_material(const RangeMaterial& m, const TransparentOptions& o) {
    return FinishedMaterial(with_density(m, o.density), o.finish,
                            {.color = o.color,
                             .thickness_mm = o.thickness_mm,
                             .opacity = o.opacity,
                             .roughness = o.roughness,
                             .process = o.process});
}
FinishedMaterial family_material(const RangeMaterial& m, const PaperOptions& o) {
    return FinishedMaterial(with_density(m, o.density), o.finish,
                            {.color = o.color, .scale = o.scale, .rotation = o.rotation, .process = o.process});
}
FinishedMaterial family_material(const RangeMaterial& m, const TexturedOptions& o) {
    return FinishedMaterial(with_density(m, o.density), o.finish,
                            {.scale = o.scale, .rotation = o.rotation, .process = o.process});
}
FinishedMaterial family_material(const RangeMaterial& m, const TextileOptions& o) {
    return FinishedMaterial(with_density(m, o.density), o.finish,
                            {.color = o.color, .scale = o.scale, .rotation = o.rotation, .process = o.process});
}

/// Python's TypeError for a keyword the family function doesn't take.
void reject(const char* function, const registry::MaterialArgs& a, const std::set<std::string>& taken) {
    const std::pair<const char*, bool> given[] = {
        {"finish", a.finish.has_value()},     {"color", a.color.has_value()},
        {"thickness_mm", a.thickness_mm.has_value()}, {"opacity", a.opacity.has_value()},
        {"roughness", a.roughness.has_value()}, {"scale", a.scale.has_value()},
        {"rotation", a.rotation.has_value()},   {"process", a.process.has_value()},
        {"density", a.density.has_value()},
    };
    for (const auto& [key, set] : given) {
        if (set && taken.count(key) == 0) {
            throw std::invalid_argument(std::string(function) + "() got an unexpected keyword argument '" + key + "'");
        }
    }
}

template <class T>
T options_from(const registry::MaterialArgs& a, const char* function);

template <>
MetalOptions options_from<MetalOptions>(const registry::MaterialArgs& a, const char* function) {
    reject(function, a, {"finish", "process", "density"});
    return {a.finish, a.process, a.density};
}

template <>
PolymerOptions options_from<PolymerOptions>(const registry::MaterialArgs& a, const char* function) {
    reject(function, a, {"color", "finish", "process", "density"});
    return {a.color, a.finish, a.process, a.density};
}

template <>
TransparentOptions options_from<TransparentOptions>(const registry::MaterialArgs& a, const char* function) {
    reject(function, a, {"color", "thickness_mm", "opacity", "roughness", "finish", "process", "density"});
    return {a.color, a.thickness_mm, a.opacity, a.roughness, a.finish, a.process, a.density};
}

template <>
PaperOptions options_from<PaperOptions>(const registry::MaterialArgs& a, const char* function) {
    reject(function, a, {"color", "finish", "process", "density", "scale", "rotation"});
    PaperOptions o;
    if (a.color) o.color = a.color;
    o.finish = a.finish;
    o.process = a.process;
    o.density = a.density;
    if (a.scale) o.scale = *a.scale;
    if (a.rotation) o.rotation = *a.rotation;
    return o;
}

template <>
TexturedOptions options_from<TexturedOptions>(const registry::MaterialArgs& a, const char* function) {
    reject(function, a, {"finish", "process", "density", "scale", "rotation"});
    TexturedOptions o;
    o.finish = a.finish;
    o.process = a.process;
    o.density = a.density;
    if (a.scale) o.scale = *a.scale;
    if (a.rotation) o.rotation = *a.rotation;
    return o;
}

template <>
TextileOptions options_from<TextileOptions>(const registry::MaterialArgs& a, const char* function) {
    reject(function, a, {"color", "finish", "process", "density", "scale", "rotation"});
    TextileOptions o;
    o.color = a.color;
    o.finish = a.finish;
    o.process = a.process;
    o.density = a.density;
    if (a.scale) o.scale = *a.scale;
    if (a.rotation) o.rotation = *a.rotation;
    return o;
}

}  // namespace

}  // namespace detail

#define B3D_MATERIAL_DEFINITIONS
#include "material_catalog.inc"
#undef B3D_MATERIAL_DEFINITIONS

// ---------------------------------------------------------------------------
// registry.py
// ---------------------------------------------------------------------------

namespace {

struct InvokerRow {
    const char* function;
    registry::Factory::Function call;
};

const InvokerRow kInvokers[] = {
#define B3D_MATERIAL_INVOKERS
#include "material_catalog.inc"
#undef B3D_MATERIAL_INVOKERS
};

struct RegistryRow {
    const char* key;
    const char* function;
    int grade;
};

const RegistryRow kRegistry[] = {
#define B3D_MATERIAL_REGISTRY
#include "material_catalog.inc"
#undef B3D_MATERIAL_REGISTRY
};

const RegistryRow* find_registry(const std::string& key) {
    static const std::unordered_map<std::string, const RegistryRow*> index = [] {
        std::unordered_map<std::string, const RegistryRow*> out;
        for (const RegistryRow& row : kRegistry) out.emplace(row.key, &row);
        return out;
    }();
    const auto it = index.find(key);
    return it == index.end() ? nullptr : it->second;
}

registry::Factory::Function invoker(const char* function) {
    for (const InvokerRow& row : kInvokers) {
        if (std::strcmp(row.function, function) == 0) return row.call;
    }
    throw std::logic_error(std::string("no family function ") + function);
}

// difflib.SequenceMatcher (no junk, autojunk) and get_close_matches, on bytes.
class SequenceMatcher {
public:
    explicit SequenceMatcher(const std::string& b) : b_(b) {
        for (std::size_t i = 0; i < b_.size(); ++i) b2j_[static_cast<unsigned char>(b_[i])].push_back(static_cast<int>(i));
        const std::size_t n = b_.size();
        if (n >= 200) {
            const std::size_t ntest = n / 100 + 1;
            for (auto it = b2j_.begin(); it != b2j_.end();) {
                it = it->second.size() > ntest ? b2j_.erase(it) : std::next(it);
            }
        }
        for (const char c : b_) ++fullbcount_[static_cast<unsigned char>(c)];
    }

    void set_seq1(const std::string& a) { a_ = a; }

    double real_quick_ratio() const {
        const std::size_t la = a_.size(), lb = b_.size();
        return ratio_of(std::min(la, lb), la + lb);
    }

    double quick_ratio() const {
        std::map<unsigned char, long> avail;
        std::size_t matches = 0;
        for (const char ch : a_) {
            const auto c = static_cast<unsigned char>(ch);
            long numb;
            if (avail.count(c)) {
                numb = avail[c];
            } else {
                const auto it = fullbcount_.find(c);
                numb = it == fullbcount_.end() ? 0 : it->second;
            }
            avail[c] = numb - 1;
            if (numb > 0) ++matches;
        }
        return ratio_of(matches, a_.size() + b_.size());
    }

    double ratio() const {
        std::size_t matches = 0;
        for (const auto& block : matching_blocks()) matches += std::get<2>(block);
        return ratio_of(matches, a_.size() + b_.size());
    }

private:
    static double ratio_of(std::size_t matches, std::size_t length) {
        return length ? 2.0 * static_cast<double>(matches) / static_cast<double>(length) : 1.0;
    }

    std::tuple<int, int, int> find_longest_match(int alo, int ahi, int blo, int bhi) const {
        int besti = alo, bestj = blo, bestsize = 0;
        std::unordered_map<int, int> j2len;
        static const std::vector<int> nothing;
        for (int i = alo; i < ahi; ++i) {
            std::unordered_map<int, int> newj2len;
            const auto found = b2j_.find(static_cast<unsigned char>(a_[i]));
            const std::vector<int>& js = found == b2j_.end() ? nothing : found->second;
            for (const int j : js) {
                if (j < blo) continue;
                if (j >= bhi) break;
                const auto prev = j2len.find(j - 1);
                const int k = newj2len[j] = (prev == j2len.end() ? 0 : prev->second) + 1;
                if (k > bestsize) {
                    besti = i - k + 1;
                    bestj = j - k + 1;
                    bestsize = k;
                }
            }
            j2len = std::move(newj2len);
        }
        // no junk: only the extensions by non-junk (popular) elements apply
        while (besti > alo && bestj > blo && a_[besti - 1] == b_[bestj - 1]) {
            --besti;
            --bestj;
            ++bestsize;
        }
        while (besti + bestsize < ahi && bestj + bestsize < bhi && a_[besti + bestsize] == b_[bestj + bestsize]) {
            ++bestsize;
        }
        return {besti, bestj, bestsize};
    }

    std::vector<std::tuple<int, int, int>> matching_blocks() const {
        const int la = static_cast<int>(a_.size()), lb = static_cast<int>(b_.size());
        std::vector<std::array<int, 4>> queue = {{0, la, 0, lb}};
        std::vector<std::tuple<int, int, int>> blocks;
        while (!queue.empty()) {
            const auto [alo, ahi, blo, bhi] = queue.back();
            queue.pop_back();
            const auto x = find_longest_match(alo, ahi, blo, bhi);
            const auto [i, j, k] = x;
            if (k) {
                blocks.push_back(x);
                if (alo < i && blo < j) queue.push_back({alo, i, blo, j});
                if (i + k < ahi && j + k < bhi) queue.push_back({i + k, ahi, j + k, bhi});
            }
        }
        std::sort(blocks.begin(), blocks.end());
        int i1 = 0, j1 = 0, k1 = 0;
        std::vector<std::tuple<int, int, int>> out;
        for (const auto& [i2, j2, k2] : blocks) {
            if (i1 + k1 == i2 && j1 + k1 == j2) {
                k1 += k2;
            } else {
                if (k1) out.emplace_back(i1, j1, k1);
                i1 = i2;
                j1 = j2;
                k1 = k2;
            }
        }
        if (k1) out.emplace_back(i1, j1, k1);
        out.emplace_back(la, lb, 0);
        return out;
    }

    std::string a_;
    std::string b_;
    std::map<unsigned char, std::vector<int>> b2j_;
    std::map<unsigned char, long> fullbcount_;
};

std::vector<std::string> get_close_matches(const std::string& word, const std::vector<std::string>& possibilities,
                                           std::size_t n = 3, double cutoff = 0.6) {
    std::vector<std::pair<double, std::string>> result;
    SequenceMatcher s(word);
    for (const std::string& x : possibilities) {
        s.set_seq1(x);
        if (s.real_quick_ratio() >= cutoff && s.quick_ratio() >= cutoff && s.ratio() >= cutoff) {
            result.emplace_back(s.ratio(), x);
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& l, const auto& r) { return l > r; });
    if (result.size() > n) result.resize(n);
    std::vector<std::string> out;
    for (const auto& [score, x] : result) out.push_back(x);
    return out;
}

}  // namespace

namespace registry {

Factory factory(const std::string& name) {
    const std::string key = normalize_name(name);
    if (const RegistryRow* row = find_registry(key)) return Factory(invoker(row->function), row->grade);
    std::vector<std::string> keys;
    for (const RegistryRow& row : kRegistry) keys.emplace_back(row.key);
    const auto suggestions = get_close_matches(key, keys);
    const std::string hint = suggestions.empty() ? "" : " -- did you mean " + join(suggestions, " / ") + "?";
    throw std::invalid_argument("unknown material " + repr_str(name) + hint +
                                " (bd_materials.material_names() lists all " + std::to_string(std::size(kRegistry)) +
                                ")");
}

FinishedMaterial resolve(const std::string& name) { return factory(name)(); }
FinishedMaterial resolve(const char* name) { return factory(name)(); }
FinishedMaterial resolve(const FinishedMaterial& material) { return material; }
FinishedMaterial resolve(const RangeMaterial& material) { return FinishedMaterial(material); }

std::vector<std::string> material_names() {
    std::vector<std::string> names;
    for (const RegistryRow& row : kRegistry) names.emplace_back(row.key);
    std::sort(names.begin(), names.end());
    return names;
}

std::string canonical_name(const RangeMaterial& material) {
    if (find_registry(normalize_name(material.name)) == nullptr) {
        throw std::invalid_argument(repr_str(material.name) +
                                    " is not a catalog material (user-defined materials have no registry entry), so "
                                    "it has no name resolve() would accept");
    }
    return material.name;
}

std::string canonical_name(const FinishedMaterial& material) { return canonical_name(material.material); }

}  // namespace registry

FinishedMaterial resolve_material(const std::string& name) { return registry::resolve(name); }

std::vector<std::string> material_names() { return registry::material_names(); }

// ---------------------------------------------------------------------------
// applicability.py
// ---------------------------------------------------------------------------

namespace applicability {

namespace {

using Table = std::vector<std::pair<FinishKey, std::set<std::string>>>;

const Table& substrates() {
    static const Table t = [] {
        auto set = [](std::initializer_list<const char*> items) {
            std::set<std::string> s;
            for (const char* i : items) s.insert(i);
            return s;
        };
        auto with = [](std::set<std::string> a, const std::set<std::string>& b) {
            a.insert(b.begin(), b.end());
            return a;
        };
        return Table{
            {Mechanical::BEAD_BLAST, set({"metal", "plastic"})},
            {Mechanical::BRUSHED, metal_set()},
            {Mechanical::FINE_SANDING, set({"metal", "plastic", "wood"})},
            {Mechanical::SMOOTH_MACHINING, set({"metal", "plastic", "wood"})},
            {Mechanical::ELECTROPOLISHED, set({"stainless", "titanium"})},
            {Chemical::ANODIZED, set({"aluminum", "titanium"})},
            {Chemical::CHEM_FILM, set({"aluminum"})},
            {Chemical::CONDUCTIVE_OXIDATION, set({"aluminum"})},
            {Chemical::BLACK_OXIDE, with(ferrous(), set({"stainless", "copper", "brass"}))},
            {Chemical::PASSIVATION, set({"stainless", "titanium"})},
            {Chemical::PICKLING, with(ferrous(), set({"stainless"}))},
            {Chemical::DYEING, set({"plastic", "resin", "aluminum"})},
            {MetalPlating::CHROME_PLATING, metal_set()},
            {MetalPlating::GOLD_PLATING, metal_set()},
            {MetalPlating::NICKEL_PLATING, metal_set()},
            {MetalPlating::SILVER_PLATING, metal_set()},
            {MetalPlating::TIN_PLATING, metal_set()},
            {MetalPlating::PVD, metal_set()},
            {MetalPlating::ZINC_PLATING, ferrous()},
            {MetalPlating::VACUUM_PLATING, coatable_poly()},
            {Coating::POWDER_COAT, metal_set()},
            {Coating::ELECTROPHORESIS, metal_set()},
            {Coating::SPRAY_PAINT, paintable()},
            {Marking::LASER_ENGRAVING, everything()},
            {Marking::ETCHING, set({"metal", "glass"})},
            {Marking::SILKSCREEN, set({"metal", "plastic", "resin", "glass"})},
        };
    }();
    return t;
}

const Finish& finish_of(const FinishKey& key) {
    return std::visit(
        [](auto k) -> const Finish& {
            using K = decltype(k);
            if constexpr (std::is_same_v<K, Mechanical>) {
                return mechanical_finishes().at(k);
            } else if constexpr (std::is_same_v<K, Chemical>) {
                return chemical_finishes().at(k);
            } else if constexpr (std::is_same_v<K, MetalPlating>) {
                return metal_plating_finishes().at(k);
            } else if constexpr (std::is_same_v<K, Coating>) {
                return coating_finishes().at(k);
            } else {
                return marking_finishes().at(k);
            }
        },
        key);
}

bool matches(const RangeMaterial& material, const std::set<std::string>& subs) {
    return subs.count(material.category) > 0 || (material.family && subs.count(*material.family) > 0);
}

}  // namespace

const std::vector<std::pair<FinishKey, std::set<std::string>>> TYPICAL_SUBSTRATES = substrates();

std::vector<Finish> typical_finishes(const RangeMaterial& material) {
    std::vector<Finish> out;
    for (const auto& [key, subs] : substrates()) {
        if (matches(material, subs)) out.push_back(finish_of(key));
    }
    return out;
}

std::vector<RangeMaterial> typical_materials(const FinishKey& finish) {
    const std::set<std::string>* subs = nullptr;
    for (const auto& [key, s] : substrates()) {
        if (key == finish) subs = &s;
    }
    if (subs == nullptr) throw std::out_of_range("not a finish of TYPICAL_SUBSTRATES");
    std::vector<RangeMaterial> out;
    for (const RangeMaterial& m : detail::all_materials()) {
        if (matches(m, *subs)) out.push_back(m);
    }
    return out;
}

std::vector<RangeMaterial> typical_materials(const Finish& finish) {
    for (const auto& [key, subs] : substrates()) {
        if (finish_of(key) == finish) return typical_materials(key);
    }
    throw std::out_of_range(finish.repr());
}

std::vector<RangeMaterial> typical_materials(const finishes::AppliedFinish& finish) {
    return typical_materials(finish.finish);
}

}  // namespace applicability

}  // namespace build123d
