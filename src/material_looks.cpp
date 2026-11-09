// bd_materials.pbr, ported step by step: the look of a material with its finish(es), process
// and choices (get_pbr_properties), from threejs_materials' presets.
#include <cstdio>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "build123d/materials.hpp"
#include "build123d/pbr.hpp"
#include "pbr_internal.hpp"

namespace build123d {

namespace {

using detail::pbr::py_lower;
using detail::pbr::py_strip;
using finishes::AppliedFinish;
using finishes::Finish;
using finishes::Sheen;
using processes::AppliedProcess;
using processes::Process;
using Texture = std::optional<std::string>;
using Uv = std::pair<std::pair<double, double>, double>;

const std::pair<const char*, const char*> kCss3[] = {
#include "css3_hex.inc"
};

/// _COLOR_HEX: the finish color names webcolors doesn't know ("natural" and "clear": no tint).
const std::map<std::string, std::optional<std::string>>& color_hex() {
    static const std::map<std::string, std::optional<std::string>> t = {
        {"natural", std::nullopt},     {"clear", std::nullopt},         {"gunmetal", "#2a3439"},
        {"nickel", "#b8b8b0"},         {"chrome", "#c8ccce"},           {"rose gold", "#b76e79"},
        {"champagne", "#e6c99a"},
    };
    return t;
}

const char* const kBlackOxideGrey = "#2a2a2a";
const char* const kEcoatCharcoal = "#2e2e2e";

/// _normalize_color: a color as threejs_materials takes it (a hex or color string, or three
/// floats); std::nullopt for no tint.
std::optional<ColorSpec> normalize_color(const std::optional<ColorSpec>& color) {
    if (!color) return std::nullopt;
    switch (color->kind()) {
        case ColorSpec::Kind::NAME: {
            const std::string key = py_lower(py_strip(color->name()));
            const auto hex = color_hex().find(key);
            if (hex != color_hex().end()) {
                return hex->second ? std::optional<ColorSpec>(ColorSpec(*hex->second)) : std::nullopt;
            }
            if (!key.empty() && key[0] == '#') return color;
            for (const auto& [name, value] : kCss3) {
                if (key == name) return ColorSpec(value);
            }
            return color;
        }
        case ColorSpec::Kind::INT: {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "#%06llx", static_cast<unsigned long long>(color->packed() & 0xFFFFFF));
            return ColorSpec(buf);
        }
        case ColorSpec::Kind::NAME_ALPHA: return normalize_color(ColorSpec(color->name()));
        case ColorSpec::Kind::FLOATS:
        case ColorSpec::Kind::COLOR: break;
    }
    const auto& v = color->values();
    if (v.size() < 3) throw std::out_of_range("list index out of range");
    return ColorSpec{v[0], v[1], v[2]};
}

PbrProperties preset(const char* category, const std::string& name, const PbrPresetArgs& args = {}) {
    return pbr_preset(category, name, args);
}

bool has_preset(const char* category, const std::string& name) {
    for (const std::string& n : pbr_preset_names(category)) {
        if (n == name) return true;
    }
    return false;
}

bool is_fdm(const Texture& texture) { return texture && (*texture == "fdm" || *texture == "fdm_skin"); }

/// _FDM_AUTHORED_PITCH
double authored_pitch(const std::string& texture) {
    return texture == "fdm" ? processes::AUTHORED_LAYER_HEIGHT_MM : processes::AUTHORED_LINE_WIDTH_MM;
}

/// _metal_name: the threejs metal of a material.
std::string metal_name(const RangeMaterial& material) {
    static const std::set<std::string> bare = {"aluminum", "brass", "bronze", "copper", "stainless",
                                               "titanium", "tin",   "gold",   "silver"};
    static const std::set<std::string> ferrous = {"mild_steel", "alloy_steel", "tool_steel", "spring_steel"};
    if (material.family && bare.count(*material.family)) return *material.family;
    if (material.family && ferrous.count(*material.family)) return "steel";
    return "stainless";
}

bool is_carbon(const RangeMaterial& material) { return material.family && *material.family == "CFRP"; }

/// _metal_tex_base: the bare metal, brushed or matte (FDM textures: matte).
PbrProperties metal_tex_base(const RangeMaterial& material, Texture texture) {
    const std::string name = metal_name(material);
    if (is_fdm(texture)) texture = "matte";
    if (texture) return preset("metal", name + "_" + *texture);
    return preset("metal", name);
}

/// _apply_uv
PbrProperties apply_uv(const PbrProperties& base, const std::pair<double, double>& scale, double rotation) {
    return base.scale(scale.first, scale.second, base.normalize_uvs, rotation);
}

/// _fdm_scale: the texture's tile size for a layer height / line width.
std::pair<double, double> fdm_scale(const PbrProperties& base, std::optional<double> pitch_mm, const std::string& texture,
                                    double mm_per_uv = 1.0) {
    if (!base.texture_repeat) return {1.0, 1.0};
    if (mm_per_uv == 0) throw std::domain_error("float division by zero");
    const double tile_u = 1.0 / (*base.texture_repeat)[0] / mm_per_uv;
    const double tile_v = 1.0 / (*base.texture_repeat)[1] / mm_per_uv;
    if (!pitch_mm) return {tile_u, tile_v};
    const double ratio = *pitch_mm / authored_pitch(texture);
    return {tile_u * ratio, tile_v * ratio};
}

/// _apply_transmissive: opacity as transmission, and the roughness.
PbrProperties apply_transmissive(const PbrProperties& base, std::optional<double> opacity,
                                 std::optional<double> roughness) {
    PbrOverrides overrides;
    if (opacity) overrides.transmission = 1.0 - *opacity;
    if (roughness) overrides.roughness = *roughness;
    if (!overrides.transmission && !overrides.roughness) return base;
    return base.override(overrides);
}

/// _plain_base: the look of the bare material.
PbrProperties plain_base(const RangeMaterial& material, const Texture& texture, const std::optional<ColorSpec>& rgb,
                         std::optional<double> thickness, std::optional<double> opacity = std::nullopt,
                         std::optional<double> roughness = std::nullopt) {
    const std::string& cat = material.category;
    const std::string fam = material.family ? *material.family : "";
    if (cat == "metal") return metal_tex_base(material, texture);
    if (cat == "wood") return preset("wood", has_preset("wood", fam) ? fam : "oak", {.color = rgb});
    if (cat == "glass") {
        return apply_transmissive(preset("glass", "glass", {.color = rgb, .thickness = thickness}), opacity, roughness);
    }
    if (cat == "paper") return preset("paper", has_preset("paper", fam) ? fam : "paper", {.color = rgb});
    if (cat == "textile") return preset("textile", has_preset("textile", fam) ? fam : "fabric_weave", {.color = rgb});
    if (material.transparent) {
        return apply_transmissive(preset("plastic", "acrylic", {.color = rgb, .thickness = thickness}), opacity,
                                  roughness);
    }
    if (is_carbon(material)) return preset("plastic", "carbon_fiber", {.color = rgb});
    if (texture == "fdm_skin") return preset("plastic", "plastic_fdm_skin", {.color = rgb});
    if (texture == "fdm") return preset("plastic", "plastic_fdm", {.color = rgb});
    if (texture) return preset("plastic", "plastic_rough", {.color = rgb});
    return preset("plastic", "plastic", {.color = rgb});
}

// The surface finishes (_SURFACE)

enum class Surface { ANODIZED, BLACK_OXIDE, DYEING, PVD, CHROME, GOLD, NICKEL, SILVER, TIN, ZINC, COAT, ECOAT };

std::optional<Surface> surface_of(const Finish& f) {
    static const std::vector<std::pair<Finish, Surface>> table = {
        {finishes::anodize(std::nullopt).finish, Surface::ANODIZED},
        {finishes::black_oxide().finish, Surface::BLACK_OXIDE},
        {finishes::dye(std::nullopt).finish, Surface::DYEING},
        {finishes::pvd().finish, Surface::PVD},
        {finishes::chrome().finish, Surface::CHROME},
        {finishes::gold_plate().finish, Surface::GOLD},
        {finishes::nickel_plate().finish, Surface::NICKEL},
        {finishes::silver_plate().finish, Surface::SILVER},
        {finishes::tin_plate().finish, Surface::TIN},
        {finishes::zinc_plate().finish, Surface::ZINC},
        {finishes::vacuum_plating(std::nullopt).finish, Surface::COAT},
        {finishes::powder_coat(std::nullopt).finish, Surface::COAT},
        {finishes::spray_paint(std::nullopt).finish, Surface::COAT},
        {finishes::electrophoresis().finish, Surface::ECOAT},
    };
    for (const auto& [finish, surface] : table) {
        if (finish == f) return surface;
    }
    return std::nullopt;
}

/// _TEXTURE: the finishes that are a texture.
Texture texture_of(const Finish& f) {
    static const Finish brushed = finishes::brushed().finish;
    static const Finish bead_blast = finishes::bead_blast().finish;
    static const Finish fine_sanding = finishes::fine_sanding().finish;
    if (f == brushed) return "brushed";
    if (f == bead_blast || f == fine_sanding) return "matte";
    return std::nullopt;
}

/// _PROCESS_TEXTURE
Texture process_texture(Process process) {
    switch (process) {
        case Process::FDM: return "fdm";
        case Process::SLS:
        case Process::MJF:
        case Process::SLM: return "matte";
        default: return std::nullopt;
    }
}

PbrProperties anodized(const RangeMaterial& m, const std::optional<ColorSpec>& rgb, const Texture& texture) {
    if (m.family == "aluminum" && !texture) return preset("metal", "aluminum_anodized", {.color = rgb});
    const PbrProperties base = metal_tex_base(m, texture);
    if (rgb) return base.override({.color = *rgb, .roughness = 0.3});
    return base;
}

PbrProperties surface_look(Surface surface, const RangeMaterial& m, std::optional<ColorSpec> rgb, const Texture& texture,
                           std::optional<Sheen> sheen) {
    switch (surface) {
        case Surface::ANODIZED: return anodized(m, rgb, texture);
        case Surface::PVD:
            if (!rgb) return metal_tex_base(m, texture);
            if (texture) return metal_tex_base(m, texture).override({.color = *rgb, .roughness = 0.15});
            return preset("coats", "metallic_coat_gloss", {.color = rgb});
        case Surface::BLACK_OXIDE:
            if (texture) return metal_tex_base(m, texture).override({.color = kBlackOxideGrey, .roughness = 0.5});
            return preset("coats", "metallic_coat_matte", {.color = kBlackOxideGrey});
        case Surface::DYEING:
            if (m.family == "aluminum") return anodized(m, rgb, texture);
            return plain_base(m, texture, rgb, std::nullopt);
        case Surface::CHROME: return preset("coats", "chrome");
        case Surface::GOLD: return preset("metal", "gold");
        case Surface::SILVER: return preset("metal", "silver");
        case Surface::NICKEL: return preset("metal", "nickel");
        case Surface::TIN: return preset("metal", "tin_matte");
        case Surface::ZINC: {
            const PbrProperties base = preset("metal", "zinc_matte");
            if (rgb) return base.override({.color = *rgb});
            return base;
        }
        case Surface::COAT:
            return preset("coats", sheen == Sheen::MATTE ? "coat_matte" : "coat_gloss", {.color = rgb});
        case Surface::ECOAT:
            if (!rgb || *rgb == ColorSpec("#000000")) rgb = ColorSpec(kEcoatCharcoal);  // _normalize_color("black")
            if (m.family == "aluminum") return metal_tex_base(m, texture).override({.color = *rgb, .roughness = 0.3});
            return preset("coats", "coat_gloss", {.color = rgb}).override({.roughness = 0.4});
    }
    throw std::logic_error("unknown surface");
}

}  // namespace

PbrProperties get_pbr_properties(const RangeMaterial& material, const std::optional<FinishSpec>& finish,
                                 const std::optional<AppliedProcess>& process, const GetPbrPropertiesOptions& options) {
    if (options.pbr) return *options.pbr;

    Texture texture;
    std::optional<Uv> finish_uv;
    std::optional<Surface> surface;
    std::optional<ColorSpec> surface_color;
    std::optional<Sheen> surface_sheen;
    if (finish) {
        for (const AppliedFinish& af : finish->items()) {
            if (const Texture t = texture_of(af.finish)) {
                texture = t;
                finish_uv = Uv{af.scale, af.rotation};
            } else if (const auto s = surface_of(af.finish)) {
                surface = s;
                surface_color = af.color;
                surface_sheen = af.sheen;
            }
        }
    }
    if (!texture && process) {
        texture = process_texture(process->process);
        if (texture == "fdm" && process->skin) texture = "fdm_skin";
    }

    PbrProperties result;
    if (!surface) {
        result = plain_base(material, texture, normalize_color(options.color), options.thickness_mm, options.opacity,
                            options.roughness);
    } else {
        result = surface_look(*surface, material, normalize_color(surface_color), texture, surface_sheen);
    }

    std::pair<double, double> scale = options.scale;
    double rotation = options.rotation;
    if (finish_uv && *finish_uv != Uv{{1.0, 1.0}, 0.0}) {
        scale = finish_uv->first;
        rotation = finish_uv->second;
    } else if (process) {
        Uv process_uv, as_authored;
        if (is_fdm(texture)) {
            const double baked_rotation = result.texture_rotation.value_or(0.0);
            process_uv = {fdm_scale(result, process->pitch_mm, *texture, process->mm_per_uv), process->rotation};
            as_authored = {fdm_scale(result, std::nullopt, *texture), baked_rotation};
        } else {
            process_uv = {process->scale, process->rotation};
            as_authored = {{1.0, 1.0}, 0.0};
        }
        if (process_uv != as_authored) {
            scale = process_uv.first;
            rotation = process_uv.second;
        }
    }
    if (scale != std::pair<double, double>{1.0, 1.0} || rotation != 0.0) result = apply_uv(result, scale, rotation);
    return result;
}

}  // namespace build123d
