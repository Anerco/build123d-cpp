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
