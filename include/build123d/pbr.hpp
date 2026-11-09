// build123d-cpp: physically based (PBR) material looks, the part of the threejs_materials
// package that build123d and bd_materials use: the PbrProperties model, the preset library
// (pbr_properties/*.py), override(), scale(), interpolate_color(), and inject_materials(), which
// export_gltf uses to put the looks into glTF files.
//
//   PbrProperties look = pbr_preset("wood", "oak", {.scale = {{2, 2}}});
//   PbrProperties red = pbr_preset("plastic", "plastic").override({.color = "red", .roughness = 0.3});
//   auto [r, g, b, a] = look.interpolate_color();
//
// Textures (CC0) aren't part of the sources: they're looked up at run time in
// $BUILD123D_TEXTURE_DIR, <prefix>/share/build123d/textures or data/textures (see
// scripts/fetch-textures.sh). Using a texture that isn't there raises std::runtime_error.
#pragma once

#include <array>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "build123d/geometry.hpp"
#include "build123d/json.hpp"

namespace build123d {

/// A color as threejs_materials and bd_materials take it (their `Color` types): a color name or
/// hex string ("red", "#ff0000", "#ff000080", "rgb(255, 0, 0)", ...), a packed 0xRRGGBB
/// integer, a (name, alpha) pair, 3 or 4 floats in [0, 1], or a build123d Color.
class ColorSpec {
public:
    enum class Kind { NAME, INT, NAME_ALPHA, FLOATS, COLOR };

    ColorSpec(const char* name) : kind_(Kind::NAME), name_(name) {}  // NOLINT
    ColorSpec(std::string name) : kind_(Kind::NAME), name_(std::move(name)) {}  // NOLINT
    ColorSpec(int packed) : kind_(Kind::INT), packed_(packed) {}  // NOLINT
    ColorSpec(long long packed) : kind_(Kind::INT), packed_(packed) {}  // NOLINT
    ColorSpec(std::string name, double alpha) : kind_(Kind::NAME_ALPHA), name_(std::move(name)), alpha_(alpha) {}
    ColorSpec(std::initializer_list<double> values) : kind_(Kind::FLOATS), values_(values) {}
    ColorSpec(std::vector<double> values) : kind_(Kind::FLOATS), values_(std::move(values)) {}  // NOLINT
    ColorSpec(const Color& color);  // NOLINT
    ColorSpec(bool) = delete;

    Kind kind() const { return kind_; }
    const std::string& name() const { return name_; }
    long long packed() const { return packed_; }
    double alpha() const { return alpha_; }
    const std::vector<double>& values() const { return values_; }
    /// Python's repr() of the value ('red', 16711680, ('red', 0.5), (1.0, 0.0, 0.0), Color(...)).
    std::string repr() const;
    bool operator==(const ColorSpec& other) const;
    bool operator!=(const ColorSpec& other) const { return !(*this == other); }

private:
    Kind kind_;
    std::string name_;
    long long packed_ = 0;
    double alpha_ = 1.0;
    std::vector<double> values_;
};

/// threejs_materials.PbrValues: the scalar and color values of a look (colors: sRGB ratios for
/// `color`, linear for the others), unset ones std::nullopt.
struct PbrValues {
    std::optional<std::vector<double>> color{};
    std::optional<double> metalness{};
    std::optional<double> roughness{};
    std::optional<double> ior{};
    std::optional<std::vector<double>> normal_scale{};
    std::optional<std::vector<double>> emissive{};
    std::optional<double> emissive_intensity{};
    std::optional<double> transmission{};
    std::optional<double> opacity{};
    std::optional<bool> transparent{};
    std::optional<double> alpha_test{};
    std::optional<double> clearcoat{};
    std::optional<double> clearcoat_roughness{};
    std::optional<double> sheen{};
    std::optional<std::vector<double>> sheen_color{};
    std::optional<double> sheen_roughness{};
    std::optional<double> anisotropy{};
    std::optional<double> anisotropy_rotation{};
    std::optional<double> specular_intensity{};
    std::optional<std::vector<double>> specular_color{};
    std::optional<std::vector<double>> attenuation_color{};
    std::optional<double> attenuation_distance{};
    std::optional<double> thickness{};
    std::optional<double> iridescence{};
    std::optional<double> iridescence_ior{};
    std::optional<std::vector<double>> iridescence_thickness_range{};
    std::optional<double> dispersion{};
    std::optional<double> displacement_scale{};
    std::optional<int> side{};

    /// to_dict(): the set values under their three.js (camelCase) names.
    Json to_dict() const;
    static PbrValues from_dict(const Json& data);
    /// repr(): "PbrValues(color=[...], metalness=1.0)"
    std::string repr() const;
    bool operator==(const PbrValues& other) const;
};

/// threejs_materials.PbrMaps: texture file names (relative to the maps directory) or data URIs.
struct PbrMaps {
    std::optional<std::string> color{};
    std::optional<std::string> metalness{};
    std::optional<std::string> roughness{};
    std::optional<std::string> normal{};
    std::optional<std::string> emissive{};
    std::optional<std::string> ao{};
    std::optional<std::string> opacity{};
    std::optional<std::string> clearcoat{};
    std::optional<std::string> clearcoat_roughness{};
    std::optional<std::string> clearcoat_normal{};
    std::optional<std::string> transmission{};
    std::optional<std::string> sheen_color{};
    std::optional<std::string> sheen_roughness{};
    std::optional<std::string> anisotropy{};
    std::optional<std::string> iridescence{};
    std::optional<std::string> specular_intensity{};
    std::optional<std::string> specular_color{};
    std::optional<std::string> thickness{};
    std::optional<std::string> displacement{};
    std::optional<std::string> metallic_roughness{};

    Json to_dict() const;
    static PbrMaps from_dict(const Json& data);
    std::string repr() const;
    bool operator==(const PbrMaps& other) const;
};

/// The keyword arguments of PbrProperties::override(): the values to replace.
struct PbrOverrides {
    std::optional<ColorSpec> color{};
    std::optional<double> roughness{};
    std::optional<double> metalness{};
    std::optional<double> ior{};
    std::optional<double> transmission{};
    std::optional<double> opacity{};
    std::optional<bool> transparent{};
    std::optional<double> alpha_test{};
    std::optional<double> clearcoat{};
    std::optional<double> clearcoat_roughness{};
    std::optional<double> sheen{};
    std::optional<ColorSpec> sheen_color{};
    std::optional<double> sheen_roughness{};
    std::optional<double> anisotropy{};
    std::optional<double> anisotropy_rotation{};
    std::optional<double> specular_intensity{};
    std::optional<ColorSpec> specular_color{};
    std::optional<ColorSpec> emissive{};
    std::optional<double> emissive_intensity{};
    std::optional<ColorSpec> attenuation_color{};
    std::optional<double> attenuation_distance{};
    std::optional<double> thickness{};
    std::optional<double> iridescence{};
    std::optional<double> iridescence_ior{};
    std::optional<std::array<double, 2>> iridescence_thickness_range{};
    std::optional<double> dispersion{};
    std::optional<std::array<double, 2>> normal_scale{};
    std::optional<double> displacement_scale{};
    std::optional<int> side{};
};

/// threejs_materials.PbrProperties: a material look (values, texture maps and their
/// directory, texture repeat/rotation). Numbers are Python floats (an id derived by override()
/// or scale() is computed from them as Python computes it from floats).
class PbrProperties {
public:
    std::string id;
    std::string name;
    std::string source;
    std::string url;
    std::string license;
    PbrValues values;
    PbrMaps maps;
    std::optional<std::array<double, 2>> texture_repeat{};
    /// Degrees, counterclockwise.
    std::optional<double> texture_rotation{};
    bool normalize_uvs = true;
    /// The directory of the file maps.
    std::optional<std::string> maps_dir{};
    /// Whether texture_repeat is a Python tuple (set by scale()) rather than a list; only repr
    /// shows it.
    bool texture_repeat_tuple = false;

    /// PbrProperties.from_dict(data): "id", "name", "source", "url", "license", "values"
    /// (camelCase), "textures", "texture_repeat", "texture_rotation", "normalize_uvs", "maps_dir".
    static PbrProperties from_dict(const Json& data);
    /// A copy with the given values replaced and a new id ("<name>_<hash>").
    PbrProperties override(const PbrOverrides& overrides) const;
    /// A copy with the textures repeated 1/u by 1/v times and rotated by `rotation` degrees;
    /// `fixed` keeps their size the same on every part (normalize_uvs).
    PbrProperties scale(double u = 1, double v = 1, bool fixed = true, double rotation = 0.0) const;
    /// A copy without texture maps.
    PbrProperties strip_maps() const;
    /// The look's color as sRGB + alpha, each rounded to 4 digits: the color, tinted color
    /// texture average or tint, with an alpha from opacity or transmission (as Python).
    std::array<double, 4> interpolate_color(const std::optional<ColorSpec>& override_color = std::nullopt) const;
    /// interpolate_color() as a build123d Color, as build123d's material setter applies it.
    Color interpolated_color() const;
    /// repr() / dump(): the multi-line summary.
    std::string dump() const;
};

/// The keyword arguments of the preset factories (threejs_materials.pbr_properties); each
/// factory takes only some of them (std::invalid_argument for the others, as Python's
/// TypeError).
struct PbrPresetArgs {
    std::optional<ColorSpec> color{};
    std::optional<double> roughness{};
    std::optional<double> thickness{};
    std::optional<std::pair<double, double>> scale{};
    std::optional<double> rotation{};
};

/// A preset look: threejs_materials.pbr_properties.<category>.<name>(**args). Categories:
/// "coats", "glass", "metal", "paper", "plastic", "textile", "wood".
PbrProperties pbr_preset(const std::string& category, const std::string& name, const PbrPresetArgs& args = {});
/// The presets of a category (its __all__).
std::vector<std::string> pbr_preset_names(const std::string& category);

/// threejs_materials.inject_materials: put full PBR materials into a .gltf/.glb file written by
/// export_gltf, for the nodes given by their index (the file is rewritten as pygltflib writes
/// it). Nodes given the same look object (the same shared_ptr) share one glTF material.
void inject_materials(const std::string& target_path,
                      const std::vector<std::pair<int, std::shared_ptr<const PbrProperties>>>& node_materials);

}  // namespace build123d
