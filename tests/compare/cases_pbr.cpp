// Comparison cases for the PBR materials (bd_materials, threejs_materials and the glTF
// injection); cases_pbr_py.py computes the same with Python. Textures are read from
// $BUILD123D_TEXTURE_DIR (or wherever the library finds them).
#include "case_registry.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "../../src/pil_image.hpp"
#include "../../src/textures.hpp"
#include "build123d/build123d.hpp"

using namespace build123d;
using compare::V;
namespace pil = build123d::detail::pil;
namespace fs = std::filesystem;

namespace {

std::string texture_root() {
    const std::string root = detail::find_texture_dir();
    if (root.empty()) throw std::runtime_error("PBR textures not found (set BUILD123D_TEXTURE_DIR)");
    return root;
}

/// Exact fingerprints of pixel data (cases_pbr_py.py pixel_fingerprint).
V fingerprint(const pil::Image& img) {
    std::int64_t s1 = 0, s2 = 0, s3 = 0;
    for (std::size_t i = 0; i < img.pixels.size(); ++i) {
        const std::int64_t p = img.pixels[i];
        s1 += p;
        s2 += p * static_cast<std::int64_t>(i % 65521 + 1);
        s3 += p * static_cast<std::int64_t>((i * 7919) % 104729);
    }
    return V::list({img.mode, static_cast<double>(img.width), static_cast<double>(img.height),
                    std::to_string(s1) + "/" + std::to_string(s2) + "/" + std::to_string(s3)});
}

/// _open_texture_image: palette images become L (threejs_materials checks len(getbands()) == 1).
pil::Image open_texture(const std::string& path) {
    pil::Image img = pil::open_png_file(path);
    if (img.mode == "P") img = pil::convert(img, "L");
    return img;
}

std::vector<std::string> texture_files() {
    std::vector<std::string> out;
    for (const auto& dir : fs::directory_iterator(texture_root())) {
        if (!dir.is_directory()) continue;
        for (const auto& file : fs::directory_iterator(dir.path())) {
            if (file.path().extension() == ".png") {
                out.push_back(dir.path().filename().string() + "/" + file.path().filename().string());
            }
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// A Json value as a comparison value.
V jv(const Json& j) {
    switch (j.type()) {
        case Json::Type::NUL: return V(nullptr);
        case Json::Type::BOOL: return V(j.as_bool());
        case Json::Type::INT:
        case Json::Type::FLOAT: return V(j.as_double());
        case Json::Type::STRING: return V(j.as_string());
        case Json::Type::ARRAY: {
            std::vector<V> items;
            for (const Json& x : j.as_array()) items.push_back(jv(x));
            return V::list(items);
        }
        case Json::Type::OBJECT: {
            std::vector<std::pair<std::string, V>> fields;
            for (const auto& [k, x] : j.as_object()) fields.emplace_back(k, jv(x));
            return V::obj(fields);
        }
    }
    return V(nullptr);
}

/// What a look is (cases_pbr_py.py pbr_summary); the maps directory by its last component.
V pbr_summary(const PbrProperties& p, bool with_color = true) {
    std::string repr = p.dump();
    std::optional<std::string> dir;
    if (p.maps_dir) {
        dir = fs::path(*p.maps_dir).filename().string();
        const auto at = repr.find(*p.maps_dir);
        if (at != std::string::npos) repr.replace(at, p.maps_dir->size(), "<maps_dir>/" + *dir);
    }
    V repeat = p.texture_repeat ? V::list({(*p.texture_repeat)[0], (*p.texture_repeat)[1]}) : V(nullptr);
    std::vector<std::pair<std::string, V>> fields = {
        {"id", p.id}, {"name", p.name}, {"source", p.source}, {"url", p.url}, {"license", p.license},
        {"values", jv(p.values.to_dict())}, {"maps", jv(p.maps.to_dict())}, {"texture_repeat", repeat},
        {"texture_rotation", p.texture_rotation ? V(*p.texture_rotation) : V(nullptr)},
        {"normalize_uvs", p.normalize_uvs}, {"maps_dir", dir ? V(*dir) : V(nullptr)}, {"repr", repr}};
    if (with_color) {
        const auto c = p.interpolate_color();
        fields.emplace_back("color", V::list({c[0], c[1], c[2], c[3]}));
    }
    return V::obj(fields);
}

/// The value of f(), or the kind of error it raises (Python's ValueError/TypeError are
/// std::invalid_argument).
template <class F>
V outcome(F f) {
    try {
        return f();
    } catch (const std::invalid_argument&) {
        return V("error");
    } catch (const std::out_of_range&) {
        return V("error");
    } catch (const std::domain_error&) {
        return V("error");
    }
}

}  // namespace

COMPARE_CASES {
    compare::add_value("pbr_pil_textures", [] {
        std::vector<V> out;
        for (const std::string& name : texture_files()) {
            const pil::Image raw = pil::open_png_file(texture_root() + "/" + name);
            const pil::Image img = open_texture(texture_root() + "/" + name);
            const pil::Image average = pil::resize_lanczos(pil::convert(img, "RGB"), 1, 1);
            out.push_back(V::list({name, raw.mode, fingerprint(img),
                                   V::list({static_cast<double>(average.pixels[0]), static_cast<double>(average.pixels[1]),
                                            static_cast<double>(average.pixels[2])})}));
        }
        return V::list(out);
    });
    compare::add_value("pbr_pil_resize", [] {
        std::vector<V> out;
        const pil::Image l = open_texture(texture_root() + "/birch/roughness.png");
        const pil::Image rgb = open_texture(texture_root() + "/walnut/color.png");
        const pil::Image p = open_texture(texture_root() + "/carbon_fiber/color.png");
        for (const auto& [w, h] : std::vector<std::pair<int, int>>{{1024, 1024}, {333, 777}, {2048, 100}, {7, 3000}, {1, 1}}) {
            out.push_back(fingerprint(pil::resize_lanczos(l, w, h)));
            out.push_back(fingerprint(pil::resize_lanczos(rgb, w, h)));
            out.push_back(fingerprint(pil::resize_lanczos(p, w, h)));
        }
        // RGB -> L, L -> RGB, putalpha, a channel
        out.push_back(fingerprint(pil::convert(rgb, "L")));
        out.push_back(fingerprint(pil::put_alpha(rgb, pil::resize_lanczos(l, rgb.width, rgb.height))));
        out.push_back(fingerprint(pil::channel(pil::convert(l, "RGB"), 1)));
        // PNG written and read back
        out.push_back(fingerprint(pil::open_png(pil::encode_png(pil::convert(rgb, "RGBA")))));
        return V::list(out);
    });
}

COMPARE_CASES {
    compare::add_value("pbr_presets", [] {
        std::vector<V> out;
        for (const char* category : {"coats", "glass", "metal", "paper", "plastic", "textile", "wood"}) {
            for (const std::string& name : pbr_preset_names(category)) {
                out.push_back(V::list({std::string(category) + "." + name, pbr_summary(pbr_preset(category, name))}));
            }
        }
        return V::list(out);
    });
    compare::add_value("pbr_preset_variants", [] {
        std::vector<V> out;
        auto add = [&out](const char* category, const char* name, const PbrPresetArgs& args) {
            out.push_back(outcome([&] { return pbr_summary(pbr_preset(category, name, args)); }));
        };
        add("plastic", "plastic", {.color = "red"});
        add("plastic", "plastic", {.color = "#336699", .roughness = 0.5});
        add("plastic", "acrylic", {.color = ColorSpec{0.2, 0.4, 0.6}, .roughness = 0.1, .thickness = 3});
        add("glass", "glass", {.color = "#ffeedd80", .thickness = 2});
        add("wood", "oak", {.color = "orange", .scale = {{2, 3}}, .rotation = 30});
        add("wood", "walnut", {.roughness = 0.4});
        add("metal", "aluminum_brushed", {.roughness = 0.2, .scale = {{0.5, 0.5}}});
        add("metal", "steel_matte", {.rotation = 90});
        add("plastic", "plastic_fdm", {.color = "blue", .scale = {{6.4, 6.4}}, .rotation = 0});
        add("plastic", "plastic_fdm", {.scale = {{3.2, 3.2}}});
        add("plastic", "plastic_fdm_skin", {.rotation = 45});
        add("plastic", "plastic_fdm_skin", {.rotation = 0});
        add("textile", "leather", {.color = "saddlebrown"});
        add("paper", "paper", {.color = ColorSpec{1.0, 0.9, 0.8, 0.5}});
        add("coats", "chrome", {.roughness = 0.25});
        add("coats", "coat_gloss", {.color = "rgb(10%, 20%, 30%)"});
        add("metal", "aluminum", {.color = "red"});        // no color parameter
        add("metal", "titanium", {.thickness = 1});        // no thickness parameter
        add("metal", "unobtainium", {});                   // no such preset
        add("plastic", "plastic", {.color = 0xff0000});    // an int is no color here
        add("plastic", "plastic", {.color = "nocolor"});   // unknown color name
        add("plastic", "plastic", {.color = ColorSpec{1.0, 0.0}});
        return V::list(out);
    });
    compare::add_value("pbr_overrides", [] {
        std::vector<V> out;
        const PbrProperties base = pbr_preset("metal", "aluminum");
        auto add = [&out, &base](const PbrOverrides& o) {
            out.push_back(outcome([&] { return pbr_summary(base.override(o), false); }));
        };
        add({.color = "#ff000080"});
        add({.color = "#ff000080", .opacity = 0.9});
        add({.color = "hsl(120, 50%, 50%)", .roughness = 0.3, .metalness = 0.5});
        add({.color = "HSV(200, 40%, 80%)"});
        add({.color = "#abc"});
        add({.color = "#abcd"});
        add({.color = "rgba(1, 2, 3, 4)"});
        add({.ior = 1.45, .transmission = 0.5, .transparent = true, .alpha_test = 0.25});
        add({.clearcoat = 1, .clearcoat_roughness = 0.1, .sheen = 0.5, .sheen_color = "red", .sheen_roughness = 0.3});
        add({.anisotropy = 0.5, .anisotropy_rotation = 1.2, .specular_intensity = 0.7, .specular_color = ColorSpec{0.1, 0.2, 0.3}});
        add({.emissive = "#808080", .emissive_intensity = 2, .attenuation_color = "teal", .attenuation_distance = 5});
        add({.thickness = 1.5, .iridescence = 0.8, .iridescence_ior = 1.3, .iridescence_thickness_range = {{100, 400}},
             .dispersion = 0.2, .normal_scale = {{0.5, 0.5}}, .displacement_scale = 0.1, .side = 2});
        add({});
        add({.color = 16711680});
        add({.color = ColorSpec("red", 0.5)});
        add({.color = "#ff0000zz"});
        // chained: the id comes from the previous one
        out.push_back(pbr_summary(base.override({.roughness = 0.3}).override({.color = "gold"}).scale(2, 2), false));
        out.push_back(pbr_summary(pbr_preset("wood", "oak").scale(0.5, 0.25, false, 15), false));
        out.push_back(pbr_summary(pbr_preset("wood", "oak").scale().strip_maps(), false));
        return V::list(out);
    });
    compare::add_value("pbr_interpolate_color", [] {
        std::vector<V> out;
        for (const auto& [category, name] : std::vector<std::pair<const char*, const char*>>{
                 {"wood", "oak"}, {"plastic", "carbon_fiber"}, {"textile", "leather"}, {"metal", "brass"}, {"glass", "glass"}}) {
            const PbrProperties p = pbr_preset(category, name);
            for (const auto& color : std::vector<std::optional<ColorSpec>>{
                     std::nullopt, ColorSpec("red"), ColorSpec("#80808080"), ColorSpec{0.9, 0.5, 0.1}, ColorSpec{2.0, 2.0, 2.0}}) {
                const auto c = p.interpolate_color(color);
                out.push_back(V::list({c[0], c[1], c[2], c[3]}));
            }
        }
        return V::list(out);
    });
}
