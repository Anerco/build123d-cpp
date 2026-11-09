// The parts of Pillow (PIL 13) that threejs_materials uses on textures: PNG reading and
// writing, the mode conversions (P, L, LA, RGB, RGBA) and resizing with the LANCZOS filter
// (libImaging/Resample.c, 8 bits per channel), ported exactly. PNG data is read and written
// with lodepng.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace build123d::detail::pil {

using Bytes = std::vector<std::uint8_t>;

/// A PIL image: 8-bit bands interleaved; mode "P" holds palette indices.
struct Image {
    std::string mode;  // "L", "LA", "RGB", "RGBA" or "P"
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> pixels;
    std::vector<std::array<std::uint8_t, 4>> palette;  // mode "P": RGBA entries

    int bands() const;
    bool operator==(const Image& other) const;
};

/// Image.open of PNG data / a PNG file (std::runtime_error if it can't be read).
Image open_png(const Bytes& data);
Image open_png_file(const std::string& path);
/// The bytes of a file (std::runtime_error if it can't be read).
Bytes read_file(const std::string& path);
/// img.save(buf, format="PNG"): a PNG of an L, LA, RGB or RGBA image.
Bytes encode_png(const Image& image);
/// img.convert(mode) for the conversions threejs_materials uses.
Image convert(const Image& image, const std::string& mode);
/// img.resize(size, Image.Resampling.LANCZOS) (palette images aren't resampled, as in PIL).
Image resize_lanczos(const Image& image, int width, int height);
/// img.getchannel(index) as an L image.
Image channel(const Image& image, int band);
/// PIL.Image.new(mode, size, color)
Image new_image(const std::string& mode, int width, int height, const std::vector<std::uint8_t>& color);
/// img.putalpha(alpha) for an RGB or RGBA image and an L alpha of the same size.
Image put_alpha(const Image& image, const Image& alpha);

/// base64.b64encode / b64decode
std::string base64_encode(const Bytes& data);
Bytes base64_decode(const std::string& text);

}  // namespace build123d::detail::pil
