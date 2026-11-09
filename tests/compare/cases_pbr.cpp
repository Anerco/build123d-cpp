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

// ---- bd_materials -----------------------------------------------------------------------

namespace {

namespace fin = build123d::finishes;
namespace proc = build123d::processes;

V opt_str(const std::optional<std::string>& s) { return s ? V(*s) : V(nullptr); }

/// cases_pbr_py.py material_summary
V material_summary(const RangeMaterial& m) {
    const auto fields = m.fields();
    std::vector<V> props, names;
    bool has_scale = false;
    for (const std::string& f : fields) {
        names.emplace_back(f);
        if (f == "hardness_scale") has_scale = true;
        if (f == "name" || f == "density" || f == "family" || f == "transparent" || f == "hardness_scale") continue;
        const auto r = m.property(f);
        props.push_back(r ? V::list({f, r->min, r->max}) : V::list({f, nullptr}));
    }
    return V::obj({{"name", m.name},
                   {"kind", m.kind},
                   {"category", m.category},
                   {"family", opt_str(m.family)},
                   {"transparent", m.transparent},
                   {"density", m.density},
                   {"fields", V::list(names)},
                   {"hardness_scale", has_scale ? V(m.hardness_scale) : V(nullptr)},
                   {"props", V::list(props)},
                   {"str", m.str()},
                   {"repr", m.repr()}});
}

V fm_summary(const FinishedMaterial& fm, bool with_color = true) {
    return V::obj({{"repr", fm.repr()}, {"str", fm.str()}, {"name", fm.name()}, {"pbr", pbr_summary(fm.pbr(), with_color)}});
}

/// cases_pbr_py.py message
template <class F>
V message(F f) {
    try {
        f();
        return V("no error");
    } catch (const std::invalid_argument& e) {
        return V(std::string(e.what()));
    } catch (const std::out_of_range&) {
        return V("IndexError/KeyError");
    } catch (const std::domain_error&) {
        return V("ZeroDivisionError");
    }
}

const char* name_of(fin::Mechanical k) {
    static const char* const n[] = {"", "BEAD_BLAST", "BRUSHED", "FINE_SANDING", "SMOOTH_MACHINING", "ELECTROPOLISHED"};
    return n[static_cast<int>(k)];
}
const char* name_of(fin::Chemical k) {
    static const char* const n[] = {"",           "ANODIZED", "CHEM_FILM", "CONDUCTIVE_OXIDATION", "BLACK_OXIDE",
                                    "PASSIVATION", "PICKLING", "DYEING"};
    return n[static_cast<int>(k)];
}
const char* name_of(fin::MetalPlating k) {
    static const char* const n[] = {"",           "CHROME_PLATING", "GOLD_PLATING", "NICKEL_PLATING", "SILVER_PLATING",
                                    "TIN_PLATING", "PVD",            "ZINC_PLATING", "VACUUM_PLATING"};
    return n[static_cast<int>(k)];
}
const char* name_of(fin::Coating k) {
    static const char* const n[] = {"", "POWDER_COAT", "ELECTROPHORESIS", "SPRAY_PAINT"};
    return n[static_cast<int>(k)];
}
const char* name_of(fin::Marking k) {
    static const char* const n[] = {"", "LASER_ENGRAVING", "ETCHING", "SILKSCREEN"};
    return n[static_cast<int>(k)];
}

template <class Map>
V finish_table(const char* name, const Map& table) {
    std::vector<V> rows;
    for (const auto& [k, f] : table) {
        rows.push_back(V::list({name_of(k), static_cast<int>(k), f.name, opt_str(f.notes), f.repr()}));
    }
    return V::list({name, V::list(rows)});
}

template <class Map>
V grade_table(const Map& table) {
    std::vector<V> rows;
    for (const auto& [k, m] : table) rows.push_back(V::list({static_cast<int>(k), m.name}));
    return V::list(rows);
}

V names_of(const std::vector<RangeMaterial>& materials) {
    std::vector<V> out;
    for (const auto& m : materials) out.emplace_back(m.name);
    return V::list(out);
}

V names_of(const std::vector<fin::Finish>& finishes) {
    std::vector<V> out;
    for (const auto& f : finishes) out.emplace_back(f.name);
    return V::list(out);
}

V strings(const std::vector<std::string>& items) {
    std::vector<V> out(items.begin(), items.end());
    return V::list(out);
}

V string_set(const std::set<std::string>& items) { return strings({items.begin(), items.end()}); }

V unit_table(const std::vector<std::pair<std::string, std::string>>& t) {
    std::vector<V> out;
    for (const auto& [k, v] : t) out.push_back(V::list({k, v}));
    return V::list(out);
}

std::shared_ptr<const PbrProperties> shared_preset(const char* category, const char* name) {
    return std::make_shared<const PbrProperties>(pbr_preset(category, name));
}

}  // namespace

COMPARE_CASES {
    compare::add_value("pbr_material_catalog", [] {
        std::vector<V> out;
        for (const std::string& name : registry::material_names()) {
            const FinishedMaterial fm = registry::resolve(name);
            out.push_back(V::list({name, registry::canonical_name(fm), material_summary(fm.material), fm_summary(fm)}));
        }
        return V::list(out);
    });

    compare::add_value("pbr_material_variants", [] {
        std::vector<V> out;
        auto add = [&out](auto f) { out.push_back(outcome([&] { return fm_summary(f()); })); };

        const std::vector<fin::AppliedFinish> every_finish = {
            fin::bead_blast(), fin::brushed(), fin::fine_sanding(), fin::smooth_machining(), fin::electropolish(),
            fin::anodize("black"), fin::chem_film(), fin::conductive_oxidation(), fin::black_oxide(), fin::passivate(),
            fin::pickle(), fin::dye("red"), fin::chrome(), fin::gold_plate(), fin::nickel_plate(), fin::silver_plate(),
            fin::tin_plate(), fin::pvd(), fin::zinc_plate(), fin::vacuum_plating("blue"), fin::powder_coat("red"),
            fin::spray_paint("green"), fin::electrophoresis(), fin::laser_engrave(), fin::etch(), fin::silkscreen()};
        for (const auto& f : every_finish) add([&] { return metals::aluminum({.finish = f}); });
        const std::vector<std::function<FinishedMaterial(const MetalOptions&)>> metal_families = {
            [](const MetalOptions& o) { return metals::stainless(o); },
            [](const MetalOptions& o) { return metals::mild_steel(o); },
            [](const MetalOptions& o) { return metals::titanium(o); },
            [](const MetalOptions& o) { return metals::brass(o); },
            [](const MetalOptions& o) { return metals::copper(o); },
            [](const MetalOptions& o) { return metals::magnesium(o); },
            [](const MetalOptions& o) { return metals::gold(o); },
            [](const MetalOptions& o) { return metals::bronze(o); },
            [](const MetalOptions& o) { return metals::tool_steel(o); },
        };
        for (const auto& mat : metal_families) {
            for (const auto& f : {fin::brushed(), fin::bead_blast(), fin::anodize("red"), fin::pvd("gunmetal"),
                                  fin::black_oxide(), fin::electrophoresis("navy")}) {
                add([&] { return mat({.finish = f}); });
            }
        }
        // finish geometry and lists
        add([] { return metals::aluminum({.finish = fin::brushed({2.0, 2.0}, 30.0)}); });
        add([] { return metals::aluminum({.finish = fin::bead_blast({0.5, 0.5})}); });
        add([] { return metals::aluminum({.finish = fin::fine_sanding({1.0, 3.0})}); });
        add([] { return metals::aluminum({.finish = fin::brushed({1.0, 1.0}, 0.0)}); });
        add([] { return metals::aluminum({.finish = FinishSpec{fin::brushed(), fin::anodize("red")}}); });
        add([] { return metals::aluminum({.finish = FinishSpec{fin::anodize("blue"), fin::bead_blast({2.0, 2.0})}}); });
        add([] {
            return metals::aluminum(
                {.finish = FinishSpec{fin::powder_coat("red"), fin::spray_paint("blue", fin::Sheen::MATTE)}});
        });
        add([] { return metals::aluminum({.finish = FinishSpec(std::vector<fin::AppliedFinish>{})}); });
        add([] { return metals::aluminum({.finish = FinishSpec{fin::laser_engrave()}}); });
        // finish colors
        for (const auto& c : std::vector<std::optional<ColorSpec>>{
                 ColorSpec("rose gold"), ColorSpec("natural"), ColorSpec(" Champagne "), ColorSpec("#336699"),
                 ColorSpec("#33669980"), ColorSpec(0x336699), ColorSpec(-1), ColorSpec("red", 0.5),
                 ColorSpec{0.1, 0.2, 0.3}, ColorSpec{0.1, 0.2, 0.3, 0.4}, ColorSpec(Color(0.2, 0.4, 0.6, 0.5)),
                 ColorSpec("gunmetal"), ColorSpec("nocolor"), std::nullopt}) {
            add([&] { return metals::aluminum({.finish = fin::anodize(c)}); });
        }
        // the other surfaces
        add([] { return metals::aluminum({.finish = fin::pvd("rose gold")}); });
        add([] { return metals::stainless({.finish = FinishSpec{fin::brushed(), fin::pvd("gold")}}); });
        add([] { return metals::stainless({.finish = FinishSpec{fin::bead_blast(), fin::black_oxide()}}); });
        add([] { return plastics::nylon({.finish = fin::dye("blue")}); });
        add([] { return wood::oak({.finish = fin::dye("red")}); });
        add([] { return metals::aluminum({.finish = fin::dye(std::nullopt)}); });
        add([] { return metals::aluminum({.finish = FinishSpec{fin::brushed(), fin::dye("red")}}); });
        add([] { return metals::mild_steel({.finish = fin::zinc_plate("yellow")}); });
        add([] { return metals::mild_steel({.finish = fin::zinc_plate()}); });
        add([] { return plastics::abs_({.finish = fin::vacuum_plating("silver", fin::Sheen::MATTE)}); });
        add([] { return metals::aluminum({.finish = fin::powder_coat("red", std::nullopt)}); });
        add([] { return metals::aluminum({.finish = fin::electrophoresis(0)}); });
        add([] { return metals::aluminum({.finish = fin::electrophoresis("#000")}); });
        add([] { return metals::aluminum({.finish = fin::electrophoresis("black")}); });
        add([] { return metals::stainless({.finish = fin::electrophoresis("red")}); });
        add([] { return metals::aluminum({.finish = FinishSpec{fin::brushed(), fin::electrophoresis()}}); });
        add([] { return metals::copper({.finish = fin::silkscreen()}); });
        // processes
        for (const auto& p : {proc::fdm(), proc::fdm(0.1), proc::fdm(0.2, 90.0), proc::fdm(0.3, 0.0, 2.0),
                              proc::fdm(0.2, 0.0, 0.5), proc::fdm_skin(), proc::fdm_skin(0.6, 0.0),
                              proc::fdm_skin(0.4, 45.0), proc::sls(), proc::sls({2.0, 2.0}), proc::mjf(), proc::slm(),
                              proc::vat(), proc::molded(), proc::machined(), proc::cast(), proc::wrought()}) {
            add([&] { return plastics::pla({.color = "orange", .process = p}); });
        }
        for (const auto& p : {proc::fdm(), proc::slm({3.0, 3.0}), proc::cast(), proc::fdm_skin()}) {
            add([&] { return metals::aluminum({.process = p}); });
            add([&] { return wood::oak({.process = p}); });
        }
        add([] { return plastics::cfrp({.process = proc::fdm()}); });
        add([] { return plastics::pc({.opacity = 0.5, .process = proc::fdm()}); });
        add([] { return plastics::pla({.process = proc::fdm(0.0)}); });
        add([] { return plastics::pla({.process = proc::fdm(0.2, 0.0, 0.0)}); });
        // colors
        for (const auto& c : std::vector<ColorSpec>{"orange", "ORANGE", "#FF8800", "#ff880080", ColorSpec(0xFF8800),
                                                    ColorSpec("red", 0.5), ColorSpec{0.1, 0.2, 0.3},
                                                    ColorSpec{0.1, 0.2, 0.3, 0.4}, ColorSpec(Color(1.0, 0.5, 0.0)),
                                                    "natural", "clear", "chrome", "rgb(10%, 20%, 30%)",
                                                    "hsl(120, 50%, 50%)", ColorSpec{1.0}, "nocolor"}) {
            add([&] { return plastics::pla({.color = c}); });
        }
        add([] { return plastics::cfrp({.color = "red"}); });
        add([] { return plastics::petg(plastics::PETG::CF, {.color = "#11223380"}); });
        // see-through
        add([] { return plastics::pc(); });
        add([] { return plastics::pc({.opacity = 0.5}); });
        add([] { return plastics::pc({.opacity = 1.0}); });
        add([] { return plastics::pmma({.color = "blue", .thickness_mm = 3.0, .roughness = 0.1}); });
        add([] { return glass::soda_lime(); });
        add([] { return glass::borosilicate({.thickness_mm = 5.0, .opacity = 0.2, .roughness = 0.05}); });
        add([] { return glass::soda_lime({.color = "#ccffcc", .finish = fin::etch()}); });
        add([] { return resins::transparent({.color = "gold", .opacity = 0.8}); });
        // wood, paper, textile, resins
        add([] { return wood::oak(); });
        add([] { return wood::walnut({.scale = {2.0, 2.0}}); });
        add([] { return wood::maple({.rotation = 90.0}); });
        add([] { return wood::mdf(); });
        add([] { return wood::osb({.scale = {0.5, 0.5}, .rotation = 45.0}); });
        add([] { return wood::hardwood(wood::Hardwood::GENERIC); });
        add([] { return wood::softwood(wood::Softwood::PINE); });
        add([] { return wood::engineered_wood(wood::EngineeredWood::OSB, {.density = 600.0}); });
        add([] { return wood::ash({.finish = fin::fine_sanding({2.0, 2.0})}); });
        add([] { return wood::birch({.finish = fin::spray_paint("white", fin::Sheen::MATTE)}); });
        add([] { return wood::beech({.finish = fin::brushed(), .scale = {3.0, 3.0}}); });
        add([] { return wood::american_cherry({.finish = fin::brushed({2.0, 2.0}), .scale = {3.0, 3.0}}); });
        add([] { return paper::paper(); });
        add([] { return paper::paper({.color = std::nullopt}); });
        add([] { return paper::paper({.color = "yellow", .scale = {2.0, 2.0}}); });
        add([] { return paper::cardboard(); });
        add([] { return paper::cardboard({.rotation = 90.0}); });
        add([] { return paper::foamboard({.color = "black"}); });
        add([] { return textile::leather({.color = "saddlebrown"}); });
        add([] { return textile::felt(); });
        add([] { return textile::woven({.scale = {3.0, 3.0}, .rotation = 15.0}); });
        add([] { return resins::standard({.color = "grey"}); });
        add([] { return resins::tough(); });
        add([] { return resins::ceramic({.process = proc::vat()}); });
        // materials of your own
        add([] {
            return metals::custom_metal("MyAlloy", 7000.0,
                                        {.family = "aluminum", .tensile_strength = 300.0, .hardness = Range(10.0, 20.0)});
        });
        add([] { return metals::custom_metal("Mg", 1800.0, {.family = "magnesium", .finish = fin::anodize("red")}); });
        add([] { return metals::custom_metal("Gilded", 8000.0, {.pbr = shared_preset("metal", "gold")}); });
        add([] { return wood::custom_wood("Teak", 650.0, {.family = "teak"}); });
        add([] { return wood::custom_wood("Walnutish", 650.0, {.family = "walnut", .scale = {2.0, 2.0}}); });
        add([] { return paper::custom_paper("Kraft", 700.0, {.family = "corrugated_cardboard", .color = "tan"}); });
        add([] { return textile::custom_textile("Hide", 900.0, {.family = "leather", .color = "brown"}); });
        add([] { return textile::custom_textile("Cloth", 300.0); });
        add([] { return plastics::custom_plastic("CF-Nylon", 1200.0, {.family = "CFRP"}); });
        add([] {
            return plastics::custom_plastic("Clear", 1200.0, {.transparent = true, .thickness_mm = 2.0, .opacity = 0.5});
        });
        add([] { return plastics::custom_plastic("Fdm", 1200.0, {.process = proc::fdm(0.1)}); });
        add([] { return resins::custom_resin("R", 1100.0, {.color = "teal"}); });
        add([] { return glass::custom_glass("G", 2400.0, {.opacity = 0.3}); });
        add([] { return glass::custom_glass("G2", 2400.0, {.transparent = false}); });
        // FinishedMaterial itself
        const RangeMaterial base = metals::aluminum().material;
        add([&] { return FinishedMaterial(base); });
        add([&] { return FinishedMaterial(base, fin::brushed(), {.color = "red", .scale = {2.0, 2.0}, .rotation = 10.0}); });
        add([&] { return FinishedMaterial(base, {.color = "red", .thickness_mm = 1.0, .opacity = 0.5, .roughness = 0.2}); });
        add([] {
            return FinishedMaterial(plastics::pla().material, {.color = "red",
                                                               .thickness_mm = 1.0,
                                                               .opacity = 0.5,
                                                               .roughness = 0.2,
                                                               .scale = {2.0, 3.0},
                                                               .rotation = 45.0,
                                                               .process = proc::fdm()});
        });
        add([&] { return FinishedMaterial(base, {.pbr = shared_preset("metal", "copper")}); });
        add([] { return FinishedMaterial(wood::oak().material, {.scale = {0.5, 0.5}}); });
        add([&] { return FinishedMaterial(with_density(base, 1000.0)); });
        add([] { return FinishedMaterial(glass::soda_lime().material, {.color = "blue", .opacity = 0.3}); });
        add([] {
            FinishedMaterial fm = metals::aluminum();
            fm.set_pbr(pbr_preset("wood", "oak"));
            return fm;
        });
        add([&] {
            FinishedMaterial fm(base, {.pbr = shared_preset("metal", "copper")});
            fm.set_pbr(nullptr);
            return fm;
        });
        out.push_back(outcome([&] {
            return pbr_summary(
                get_pbr_properties(base, FinishSpec{fin::brushed()}, proc::fdm(), {.color = "red", .scale = {2.0, 2.0}}));
        }));
        out.push_back(outcome([&] {
            return pbr_summary(get_pbr_properties(base, fin::brushed({1.0, 1.0}, 0.0), proc::slm({2.0, 2.0})));
        }));
        out.push_back(outcome([&] {
            return pbr_summary(get_pbr_properties(base, std::nullopt, std::nullopt, {.pbr = shared_preset("wood", "oak")}));
        }));
        {
            std::vector<std::string> warnings;
            const WarningHandler previous =
                set_warning_handler([&warnings](const std::string& m) { warnings.push_back(m); });
            const FinishedMaterial fm = plastics::pla({.process = proc::Process::FDM});
            set_warning_handler(previous);
            out.push_back(V::obj({{"fm", fm_summary(fm)}, {"warnings", strings(warnings)}}));
        }
        return V::list(out);
    });

    compare::add_value("pbr_material_messages", [] {
        std::vector<V> out;
        const RangeMaterial base = metals::aluminum().material;
        const auto copper = shared_preset("metal", "copper");
        out.push_back(message([&] { (void)FinishedMaterial(base, fin::brushed(), {.pbr = copper}); }));
        out.push_back(message([&] { (void)FinishedMaterial(base, {.color = "red", .pbr = copper}); }));
        out.push_back(message([&] { (void)FinishedMaterial(base, {.scale = {2.0, 1.0}, .pbr = copper}); }));
        out.push_back(message([&] { (void)FinishedMaterial(base, {.rotation = 1.0, .pbr = copper}); }));
        out.push_back(message([&] { (void)FinishedMaterial(base, {.process = proc::fdm(), .pbr = copper}); }));
        out.push_back(message([&] { (void)FinishedMaterial(base, fin::brushed(), {.process = proc::fdm()}); }));
        out.push_back(message([&] {
            (void)FinishedMaterial(base, FinishSpec(std::vector<fin::AppliedFinish>{}), {.process = proc::fdm()});
        }));
        out.push_back(message([] { (void)metals::aluminum({.finish = fin::brushed(), .process = proc::cast()}); }));
        out.push_back(message([] { (void)registry::factory("aluminum")({.color = "red"}); }));
        out.push_back(message([] { (void)registry::factory("Alu_G7075_T6")({.thickness_mm = 1.0}); }));
        out.push_back(message([] { (void)registry::factory("paper")({.opacity = 0.5}); }));
        out.push_back(message([] { (void)registry::factory("oak")({.color = "red"}); }));
        out.push_back(message([] { (void)registry::factory("abs")({.scale = std::pair{1.0, 1.0}}); }));
        std::string aluminum30;
        for (int i = 0; i < 30; ++i) aluminum30 += "aluminum";
        for (const std::string& name : std::vector<std::string>{
                 "alumnium", "Aluminium", "stainles steel", "pla generic", "xyz", "Alu-G6061", "", "  oak  x", "titanum",
                 "petg-cf2", std::string(250, 'a'), aluminum30, "ToolSteel_D2", "HARDWOOD_OKA", "brass c360",
                 "Nylon PA 12", "steel"}) {
            out.push_back(message([&] { (void)registry::factory(name); }));
        }
        out.push_back(message([] { (void)registry::canonical_name(metals::custom_metal("Mine", 7000.0)); }));
        out.push_back(message([] { (void)registry::canonical_name(metals::custom_metal("Alu G6061 T6", 7000.0)); }));
        out.push_back(message([] { (void)registry::canonical_name(metals::aluminum().material); }));
        out.push_back(message([] { (void)Range(2.0, 1.0); }));
        out.push_back(message([] { (void)plastics::pla({.color = ColorSpec{1.0}}).pbr(); }));
        out.push_back(message([] { (void)applicability::typical_materials(fin::Finish{"Custom"}); }));
        out.push_back(message([] { (void)plastics::pla({.process = proc::fdm(0.0)}).pbr(); }));
        return V::list(out);
    });

    compare::add_value("pbr_material_tables", [] {
        const std::vector<fin::AppliedFinish> applied = {
            fin::bead_blast(), fin::brushed({2.0, 3.0}, 45.0), fin::fine_sanding({0.5, 0.5}), fin::smooth_machining(),
            fin::electropolish(), fin::anodize("red"), fin::chem_film(), fin::conductive_oxidation(),
            fin::black_oxide(), fin::passivate(), fin::pickle(), fin::dye("blue"), fin::chrome(), fin::gold_plate(),
            fin::nickel_plate(), fin::silver_plate(), fin::tin_plate(), fin::pvd(), fin::pvd("gold"),
            fin::zinc_plate(), fin::vacuum_plating("red", fin::Sheen::MATTE), fin::powder_coat(std::nullopt, std::nullopt),
            fin::spray_paint("white"), fin::electrophoresis(), fin::laser_engrave(), fin::etch(), fin::silkscreen(),
            fin::anodize(0x123456), fin::anodize(ColorSpec("red", 0.5)), fin::anodize(ColorSpec{0.1, 0.2, 0.3}),
            fin::anodize(ColorSpec(Color(1.0, 0.0, 0.0))), fin::anodize("it's")};
        const std::vector<proc::AppliedProcess> processes = {
            proc::fdm(),   proc::fdm(0.1, 90.0, 2.0), proc::fdm_skin(), proc::fdm_skin(0.6, 0.0), proc::sls(),
            proc::sls({2.0, 2.0}), proc::mjf(), proc::slm(), proc::vat(), proc::molded(), proc::machined(),
            proc::cast(), proc::wrought()};
        std::vector<V> reprs_applied, reprs_processes, all_finishes, values, typical_f, typical_m, typical_by_finish;
        for (const auto& f : applied) reprs_applied.emplace_back(f.repr());
        for (const auto& p : processes) reprs_processes.emplace_back(p.repr());
        for (const auto& f : fin::ALL_FINISHES) all_finishes.emplace_back(f.repr());
        for (int i = 0; i < 9; ++i) values.emplace_back(proc::to_string(static_cast<proc::Process>(i)));
        for (const auto& m : ALL_MATERIALS) typical_f.push_back(V::list({m.name, names_of(typical_finishes(m))}));
        for (const auto& [key, subs] : applicability::TYPICAL_SUBSTRATES) {
            const std::string name = std::visit([](auto k) { return std::string(name_of(k)); }, key);
            typical_m.push_back(V::list({name, string_set(subs), names_of(typical_materials(key))}));
        }
        for (const auto& f : fin::ALL_FINISHES) typical_by_finish.push_back(V::list({f.name, names_of(typical_materials(f))}));
        const std::pair<const char*, const std::vector<RangeMaterial>*> modules[] = {
            {"ALL_GLASSES", &glass::ALL_GLASSES}, {"ALL_METALS", &metals::ALL_METALS},
            {"ALL_PAPERS", &paper::ALL_PAPERS},   {"ALL_PLASTICS", &plastics::ALL_PLASTICS},
            {"ALL_RESINS", &resins::ALL_RESINS},  {"ALL_TEXTILES", &textile::ALL_TEXTILES},
            {"ALL_WOODS", &wood::ALL_WOODS}};
        std::vector<V> module_lists;
        for (const auto& [name, list] : modules) module_lists.push_back(V::list({name, names_of(*list)}));
        return V::list({
            V::list({finish_table("MECHANICAL_FINISHES", fin::MECHANICAL_FINISHES),
                     finish_table("CHEMICAL_FINISHES", fin::CHEMICAL_FINISHES),
                     finish_table("METAL_PLATING_FINISHES", fin::METAL_PLATING_FINISHES),
                     finish_table("COATING_FINISHES", fin::COATING_FINISHES),
                     finish_table("MARKING_FINISHES", fin::MARKING_FINISHES)}),
            V::list(all_finishes),
            V::list(reprs_applied),
            V::list({applied[0] == fin::bead_blast(), applied[1] == fin::brushed(),
                     fin::Finish{"Brushed"} == applied[1].finish}),
            V::list(reprs_processes),
            V::list({processes[0] == proc::fdm(), processes[0] == proc::fdm_skin()}),
            V::list(values),
            V::list({proc::AUTHORED_LAYER_HEIGHT_MM, proc::AUTHORED_LINE_WIDTH_MM, proc::AUTHORED_SKIN_ROTATION}),
            V::list(typical_f),
            V::list(typical_m),
            V::list(typical_by_finish),
            names_of(typical_materials(fin::brushed({2.0, 2.0}))),
            names_of(typical_finishes(wood::custom_wood("x", 1.0).material)),
            strings(registry::material_names()),
            names_of(ALL_MATERIALS),
            V::list(module_lists),
            grade_table(metals::ALU_MATERIALS),
            grade_table(wood::HARDWOOD_MATERIALS),
            grade_table(plastics::NYLON_MATERIALS),
            unit_table(PROPERTY_UNITS),
            unit_table(MECHANICAL_PROPERTY_UNITS),
            unit_table(THERMAL_PROPERTY_UNITS),
            V::list({string_set(ALLOWED_CATEGORIES), string_set(FERROUS), string_set(METAL), string_set(PAINTABLE),
                     string_set(COATABLE_POLY), string_set(EVERYTHING)}),
            V::list({NOT_SUITABLE.min, NOT_SUITABLE.max, Range(1.0, 3.0).value_at(0.25), Range(2.0, 2.0).value_at(0.5)}),
            material_summary(with_density(metals::aluminum().material, 2000.0)),
            material_summary(with_density(metals::aluminum().material, std::nullopt)),
            material_summary(
                metals::custom_metal("C", 1.0, {.tensile_strength = NOT_SUITABLE, .yield_strength = 5.0}).material),
            V(wood::custom_wood("W", 600.0).material.str()),
            V(paper::custom_paper("P", 600.0, {.thickness = 0.1}).material.str()),
        });
    });
}

// ---- glTF / GLB with materials ----------------------------------------------------------

namespace {

std::string pbr_dir(const std::string& side) {
    const fs::path path = fs::temp_directory_path() / "build123d_compare_pbr" / side;
    fs::create_directories(path);
    return path.string();
}

std::uint32_t le32(const pil::Bytes& data, std::size_t at) {
    return static_cast<std::uint32_t>(data.at(at)) | (static_cast<std::uint32_t>(data.at(at + 1)) << 8) |
           (static_cast<std::uint32_t>(data.at(at + 2)) << 16) | (static_cast<std::uint32_t>(data.at(at + 3)) << 24);
}

std::vector<double> accessor_values(const Json& doc, const pil::Bytes& blob, std::size_t index) {
    const Json& acc = doc["accessors"].at(index);
    const Json& bv = doc["bufferViews"].at(static_cast<std::size_t>(acc["bufferView"].as_int()));
    const int type = static_cast<int>(acc["componentType"].as_int());
    static const std::map<std::string, std::size_t> counts = {{"SCALAR", 1}, {"VEC2", 2}, {"VEC3", 3}, {"VEC4", 4}, {"MAT4", 16}};
    const std::size_t n = counts.at(acc["type"].as_string());
    const std::size_t size = type == 5120 || type == 5121 ? 1 : type == 5122 || type == 5123 ? 2 : 4;
    const std::size_t stride = bv.contains("byteStride") ? static_cast<std::size_t>(bv["byteStride"].as_int()) : size * n;
    const std::size_t offset = static_cast<std::size_t>((bv.contains("byteOffset") ? bv["byteOffset"].as_int() : 0) +
                                                        (acc.contains("byteOffset") ? acc["byteOffset"].as_int() : 0));
    std::vector<double> out;
    for (std::int64_t i = 0; i < acc["count"].as_int(); ++i) {
        for (std::size_t c = 0; c < n; ++c) {
            const std::uint8_t* p = blob.data() + offset + static_cast<std::size_t>(i) * stride + c * size;
            switch (type) {
                case 5120: out.push_back(static_cast<std::int8_t>(p[0])); break;
                case 5121: out.push_back(p[0]); break;
                case 5122: {
                    std::int16_t x;
                    std::memcpy(&x, p, 2);
                    out.push_back(x);
                    break;
                }
                case 5123: {
                    std::uint16_t x;
                    std::memcpy(&x, p, 2);
                    out.push_back(x);
                    break;
                }
                case 5125: {
                    std::uint32_t x;
                    std::memcpy(&x, p, 4);
                    out.push_back(x);
                    break;
                }
                default: {
                    float x;
                    std::memcpy(&x, p, 4);
                    out.push_back(x);
                }
            }
        }
    }
    return out;
}

/// cases_pbr_py.py key_orders
void key_orders(const Json& value, std::vector<V>& out) {
    if (value.is_object()) {
        std::vector<V> keys;
        for (const auto& [k, v] : value.as_object()) keys.emplace_back(k);
        out.push_back(V::list(keys));
        for (const auto& [k, v] : value.as_object()) key_orders(v, out);
    } else if (value.is_array()) {
        for (const Json& v : value.as_array()) key_orders(v, out);
    }
}

std::vector<double> triangle_set(const std::vector<double>& values) {
    std::vector<std::array<double, 3>> tris;
    for (std::size_t i = 0; i + 2 < values.size(); i += 3) {
        std::array<double, 3> t = {values[i], values[i + 1], values[i + 2]};
        const auto first = static_cast<std::size_t>(std::min_element(t.begin(), t.end()) - t.begin());
        std::rotate(t.begin(), t.begin() + static_cast<std::ptrdiff_t>(first), t.end());
        tris.push_back(t);
    }
    std::sort(tris.begin(), tris.end());
    std::vector<double> out;
    for (const auto& t : tris) out.insert(out.end(), t.begin(), t.end());
    return out;
}

void erase_key(Json& object, const std::string& key) {
    auto& o = object.as_object();
    o.erase(std::remove_if(o.begin(), o.end(), [&key](const auto& e) { return e.first == key; }), o.end());
}

/// cases_pbr_py.py gltf_file_summary
V gltf_file_summary(const std::string& path) {
    const pil::Bytes data = pil::read_file(path);
    const fs::path base = fs::path(path).parent_path();
    const bool glb = path.size() > 4 && path.compare(path.size() - 4, 4, ".glb") == 0;
    Json doc;
    pil::Bytes blob;
    std::vector<V> header;
    if (glb) {
        const std::size_t json_len = le32(data, 12);
        doc = Json::parse(std::string(data.begin() + 20, data.begin() + 20 + static_cast<std::ptrdiff_t>(json_len)));
        const std::size_t start = 20 + json_len;
        const std::size_t bin_len = le32(data, start);
        blob.assign(data.begin() + static_cast<std::ptrdiff_t>(start + 8),
                    data.begin() + static_cast<std::ptrdiff_t>(start + 8 + bin_len));
        const char last = static_cast<char>(data.at(20 + json_len - 1));
        header = {std::string(data.begin(), data.begin() + 4), static_cast<double>(le32(data, 4)),
                  le32(data, 8) == data.size(), static_cast<double>(json_len % 4),
                  std::memcmp(data.data() + start + 4, "BIN\0", 4) == 0, static_cast<double>(bin_len % 4),
                  last == ' ' || last == '}'};
    } else {
        doc = Json::parse(std::string(data.begin(), data.end()));
        blob = pil::read_file((base / doc["buffers"].at(0)["uri"].as_string()).string());
        header = {doc["buffers"].at(0)["byteLength"].as_int() == static_cast<std::int64_t>(blob.size()),
                  std::string(data.begin(), data.begin() + 3), std::string(1, static_cast<char>(data.back()))};
    }
    std::vector<V> images;
    std::set<std::int64_t> image_views;
    if (doc.contains("images")) {
        for (const Json& img : doc["images"].as_array()) {
            pil::Bytes raw;
            if (img.contains("bufferView")) {
                const Json& bv = doc["bufferViews"].at(static_cast<std::size_t>(img["bufferView"].as_int()));
                const auto off = static_cast<std::ptrdiff_t>(bv.contains("byteOffset") ? bv["byteOffset"].as_int() : 0);
                raw.assign(blob.begin() + off, blob.begin() + off + static_cast<std::ptrdiff_t>(bv["byteLength"].as_int()));
                image_views.insert(img["bufferView"].as_int());
            } else {
                raw = pil::read_file((base / img["uri"].as_string()).string());
            }
            images.push_back(fingerprint(pil::open_png(raw)));
        }
    }
    bool packed = true;
    if (glb) {
        std::int64_t offset = 0;
        for (const Json& bv : doc["bufferViews"].as_array()) {
            packed = packed && (bv.contains("byteOffset") ? bv["byteOffset"].as_int() : 0) == offset;
            const std::int64_t len = bv["byteLength"].as_int();
            offset += len + (4 - len % 4) % 4;
        }
        packed = packed && offset == static_cast<std::int64_t>(blob.size());
    }
    std::vector<V> orders;
    key_orders(doc, orders);
    // triangles as a set (OCCT builds order the triangles of some meshes differently)
    std::set<std::int64_t> index_accessors;
    if (doc.contains("meshes")) {
        for (const Json& m : doc["meshes"].as_array()) {
            for (const Json& p : m["primitives"].as_array()) {
                if (p.contains("indices")) index_accessors.insert(p["indices"].as_int());
            }
        }
    }
    std::vector<V> accessors;
    if (doc.contains("accessors")) {
        for (std::size_t i = 0; i < doc["accessors"].size(); ++i) {
            std::vector<double> values = accessor_values(doc, blob, i);
            if (index_accessors.count(static_cast<std::int64_t>(i))) values = triangle_set(values);
            accessors.push_back(V::list_of(values));
        }
    }
    if (doc.contains("bufferViews")) {
        auto& views = doc["bufferViews"].as_array();
        for (std::size_t i = 0; i < views.size(); ++i) {
            if (glb) erase_key(views[i], "byteOffset");
            if (image_views.count(static_cast<std::int64_t>(i))) erase_key(views[i], "byteLength");
        }
    }
    if (glb && doc.contains("buffers")) {
        for (Json& b : doc["buffers"].as_array()) erase_key(b, "byteLength");
    }
    return V::obj({{"json", jv(doc)},
                   {"images", V::list(images)},
                   {"accessors", V::list(accessors)},
                   {"header", V::list(header)},
                   {"packed", packed},
                   {"key_orders", V::list(orders)}});
}

V export_summaries(const Shape& shape, const std::string& name, double linear = 1e-3, double angular = 0.1) {
    std::vector<V> out;
    for (const auto& [ext, binary] : {std::pair<const char*, bool>{".gltf", false}, {".glb", true}}) {
        const std::string path = (fs::path(pbr_dir("cpp")) / (name + ext)).string();
        export_gltf(shape, path, Unit::MM, binary, linear, angular);
        out.push_back(gltf_file_summary(path));
    }
    return V::list(out);
}

/// Python's Shape.material setter: the material, and the color of its look.
void assign(Shape& shape, const FinishedMaterial& material) {
    shape.material.set_raw(material);
    shape.color = material.pbr().interpolated_color();
}

}  // namespace

COMPARE_CASES {
    compare::add_value("pbr_gltf_metal", [] {
        Part box = Box(10, 20, 30);
        assign(box, metals::aluminum());
        return export_summaries(box, "metal");
    });
    compare::add_value("pbr_gltf_plastic", [] {
        Part part = Cylinder(5, 10) - Box(2, 2, 10);
        part.label = "plastic part";
        assign(part, plastics::pla({.color = "orange"}));
        return export_summaries(part, "plastic", 0.1, 0.5);
    });
    compare::add_value("pbr_gltf_glass", [] {
        Part part = Sphere(5);
        assign(part, glass::soda_lime({.color = "#ccffcc", .thickness_mm = 2.0, .opacity = 0.3}));
        return export_summaries(part, "glass", 0.1, 0.5);
    });
    compare::add_value("pbr_gltf_wood", [] {
        Part box = Box(30, 20, 10);
        assign(box, wood::oak({.scale = {2.0, 2.0}, .rotation = 30.0}));
        return export_summaries(box, "wood");
    });
    compare::add_value("pbr_gltf_textured", [] {
        std::vector<V> out;
        const std::vector<std::pair<const char*, FinishedMaterial>> materials = {
            {"carbon", plastics::cfrp()},
            {"fdm", plastics::pla({.color = "red", .process = proc::fdm()})},
            {"brushed", metals::stainless({.finish = fin::brushed({2.0, 2.0}, 45.0)})},
            {"rough", plastics::abs_({.color = "blue", .process = proc::sls()})}};
        for (const auto& [name, material] : materials) {
            Part box = Box(20, 10, 5);
            assign(box, material);
            out.push_back(export_summaries(box, name));
        }
        return V::list(out);
    });
    compare::add_value("pbr_gltf_assembly", [] {
        const auto look = shared_preset("wood", "oak");
        const FinishedMaterial shared(metals::aluminum().material, {.pbr = look});
        Part a = Box(10, 10, 10);
        a.label = "a";
        assign(a, metals::aluminum({.finish = fin::brushed()}));
        Part b = Pos(20, 0, 0) * Box(5, 5, 10);
        b.label = "b";
        assign(b, plastics::pla({.color = "red"}));
        Part c = Pos(40, 0, 0) * Box(8, 8, 8);
        c.label = "c";
        assign(c, shared);
        Part d = Pos(60, 0, 0) * Box(6, 6, 6);
        d.label = "d";
        assign(d, shared);
        Part e = Pos(80, 0, 0) * Box(5, 5, 5);
        e.label = "e";
        assign(e, FinishedMaterial(plastics::pla().material, {.pbr = look}));
        Part f = Pos(0, 20, 0) * Box(4, 4, 4);
        f.label = "f";
        assign(f, *a.material);
        Part g = Pos(0, 40, 0) * Box(3, 3, 3);
        g.label = "g";
        return export_summaries(Compound::make_assembly({a, b, c, d, e, f, g}, "asm"), "assembly");
    });
    compare::add_value("pbr_gltf_inherited", [] {
        Part a = Box(10, 10, 10);
        a.label = "a";
        Part b = Pos(20, 0, 0) * Box(5, 5, 5);
        b.label = "b";
        const Compound inner = Compound::make_assembly({Pos(0, 20, 0) * Box(3, 3, 3)}, "inner");
        Compound assembly = Compound::make_assembly({a, b, inner}, "asm");
        assign(assembly, plastics::pla({.color = "blue"}));
        return export_summaries(assembly, "inherited");
    });
    compare::add_value("pbr_gltf_extensions", [] {
        const std::string tex = *pbr_preset("wood", "oak").maps_dir;
        const std::string rough = *pbr_preset("metal", "steel_matte").maps_dir;
        Json data = Json::parse(R"({"id": "x", "name": "everything", "source": "test",
            "values": {"color": [0.8, 0.2, 0.1], "metalness": 0.3, "roughness": 0.6, "ior": 1.6, "transmission": 0.4,
                       "thickness": 2.0, "attenuationColor": [0.9, 0.8, 0.7], "attenuationDistance": 5.0,
                       "clearcoat": 0.7, "clearcoatRoughness": 0.2, "sheen": 0.5, "sheenColor": [0.1, 0.2, 0.3],
                       "sheenRoughness": 0.4, "iridescence": 0.6, "iridescenceIOR": 1.3,
                       "iridescenceThicknessRange": [100.0, 400.0], "anisotropy": 0.5, "anisotropyRotation": 0.7,
                       "specularIntensity": 0.8, "specularColor": [1.0, 0.9, 0.8], "emissive": [0.1, 0.0, 0.0],
                       "emissiveIntensity": 2.0, "dispersion": 0.3, "normalScale": [0.5, 0.5], "side": 2,
                       "alphaTest": 0.25},
            "textures": {"color": "color.png", "normal": "normal.png", "roughness": "roughness.png",
                         "ao": "roughness.png", "emissive": "color.png", "transmission": "roughness.png",
                         "clearcoat": "roughness.png", "clearcoatNormal": "normal.png", "sheenColor": "color.png",
                         "specularColor": "color.png", "thickness": "roughness.png"},
            "texture_repeat": [1.0, 1.0]})");
        data["maps_dir"] = Json(tex);
        Json blend = Json::parse(R"({"id": "b", "name": "", "source": "test",
            "values": {"color": [0.5, 0.5, 0.5], "transparent": true, "opacity": 0.5, "dispersion": 0.1, "metalness": 0.0},
            "textures": {"metalness": "roughness.png", "opacity": "roughness.png", "roughness": "roughness.png"}})");
        blend["maps_dir"] = Json(rough);
        Json mask = Json::parse(R"({"id": "m", "name": "masked", "source": "test", "values": {"opacity": 0.8},
            "textures": {"opacity": "roughness.png", "color": "color.png"}, "texture_rotation": 90.0,
            "normalize_uvs": false})");
        mask["maps_dir"] = Json(tex);
        std::vector<Shape> parts;
        int i = 0;
        for (const Json* d : {&data, &blend, &mask}) {
            Part box = Pos(i * 20.0, 0, 0) * Box(10, 10, 10);
            box.label = "p" + std::to_string(i);
            assign(box, metals::custom_metal("Look", 1000.0,
                                             {.pbr = std::make_shared<const PbrProperties>(PbrProperties::from_dict(*d))}));
            parts.push_back(box);
            ++i;
        }
        return export_summaries(Compound::make_assembly(parts, "ext"), "extensions");
    });
    compare::add_value("pbr_gltf_inject_direct", [] {
        std::vector<V> out;
        for (const char* name : {"direct.gltf", "direct.glb"}) {
            const std::string path = (fs::path(pbr_dir("cpp")) / name).string();
            export_gltf(Box(10, 10, 10), path, Unit::MM, std::string(name).find(".glb") != std::string::npos);
            inject_materials(path, {{1, std::make_shared<const PbrProperties>(pbr_preset("plastic", "plastic", {.color = "red"}))},
                                    {0, shared_preset("metal", "copper")},
                                    {5, shared_preset("metal", "gold")}});
            out.push_back(gltf_file_summary(path));
        }
        return V::list(out);
    });
}
