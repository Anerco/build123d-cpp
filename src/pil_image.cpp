// Pillow's image operations as threejs_materials uses them (see pil_image.hpp).
#include "pil_image.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include "lodepng.h"

namespace build123d::detail::pil {

namespace {

constexpr int kPrecisionBits = 32 - 8 - 2;
constexpr double kPi = 3.14159265358979323846;  // M_PI

int bands_of(const std::string& mode) {
    if (mode == "L" || mode == "P") return 1;
    if (mode == "LA") return 2;
    if (mode == "RGB") return 3;
    if (mode == "RGBA") return 4;
    throw std::invalid_argument("unsupported image mode " + mode);
}

/// L24(rgb) >> 16: ITU-R 601-2 luma as PIL computes it.
std::uint8_t luma(std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    return static_cast<std::uint8_t>((r * 19595 + g * 38470 + b * 7471 + 0x8000) >> 16);
}

double sinc_filter(double x) {
    if (x == 0.0) return 1.0;
    x = x * kPi;
    return std::sin(x) / x;
}

double lanczos_filter(double x) {
    if (-3.0 <= x && x < 3.0) return sinc_filter(x) * sinc_filter(x / 3);
    return 0.0;
}

struct Coefficients {
    int ksize = 0;
    std::vector<int> bounds;
    std::vector<int> kk;  // normalized to fixed point (normalize_coeffs_8bpc)
};

/// precompute_coeffs + normalize_coeffs_8bpc
Coefficients precompute_coeffs(int in_size, float in0, float in1, int out_size) {
    double filterscale = static_cast<double>(in1 - in0) / out_size;
    const double scale = filterscale;
    if (filterscale < 1.0) filterscale = 1.0;
    const double support = 3.0 * filterscale;
    Coefficients c;
    c.ksize = static_cast<int>(std::ceil(support)) * 2 + 1;
    std::vector<double> kk(static_cast<std::size_t>(out_size) * static_cast<std::size_t>(c.ksize), 0.0);
    c.bounds.assign(static_cast<std::size_t>(out_size) * 2, 0);
    const double inv_filterscale = 1.0 / filterscale;
    for (int xx = 0; xx < out_size; ++xx) {
        const double center = in0 + (xx + 0.5) * scale;
        double ww = 0.0;
        int xmin = static_cast<int>(center - support + 0.5);
        if (xmin < 0) xmin = 0;
        int xmax = static_cast<int>(center + support + 0.5);
        if (xmax > in_size) xmax = in_size;
        xmax -= xmin;
        double* k = &kk[static_cast<std::size_t>(xx) * static_cast<std::size_t>(c.ksize)];
        int x = 0;
        for (; x < xmax; ++x) {
            const double w = lanczos_filter((x + xmin - center + 0.5) * inv_filterscale);
            k[x] = w;
            ww += w;
        }
        if (ww != 0.0) {
            for (x = 0; x < xmax; ++x) k[x] /= ww;
        }
        for (; x < c.ksize; ++x) k[x] = 0;
        c.bounds[static_cast<std::size_t>(xx) * 2] = xmin;
        c.bounds[static_cast<std::size_t>(xx) * 2 + 1] = xmax;
    }
    c.kk.resize(kk.size());
    for (std::size_t i = 0; i < kk.size(); ++i) {
        c.kk[i] = kk[i] < 0 ? static_cast<int>(-0.5 + kk[i] * (1 << kPrecisionBits))
                            : static_cast<int>(0.5 + kk[i] * (1 << kPrecisionBits));
    }
    return c;
}

std::uint8_t clip8(int in) {
    const int v = in >> kPrecisionBits;
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<std::uint8_t>(v);
}

/// _ImagingResampleHorizontal_8bpc over rows [offset, offset + rows)
Image resample_horizontal(const Image& in, int out_width, int rows, int offset, const Coefficients& c) {
    const int nb = in.bands();
    Image out{in.mode, out_width, rows, {}, {}};
    out.pixels.resize(static_cast<std::size_t>(out_width) * static_cast<std::size_t>(rows) * nb);
    for (int yy = 0; yy < rows; ++yy) {
        const std::uint8_t* line_in = &in.pixels[static_cast<std::size_t>(yy + offset) * in.width * nb];
        std::uint8_t* line_out = &out.pixels[static_cast<std::size_t>(yy) * out_width * nb];
        for (int xx = 0; xx < out_width; ++xx) {
            const int xmin = c.bounds[static_cast<std::size_t>(xx) * 2];
            const int xmax = c.bounds[static_cast<std::size_t>(xx) * 2 + 1];
            const int* k = &c.kk[static_cast<std::size_t>(xx) * static_cast<std::size_t>(c.ksize)];
            for (int b = 0; b < nb; ++b) {
                int ss = 1 << (kPrecisionBits - 1);
                for (int x = 0; x < xmax; ++x) ss += line_in[(x + xmin) * nb + b] * k[x];
                line_out[xx * nb + b] = clip8(ss);
            }
        }
    }
    return out;
}

/// _ImagingResampleVertical_8bpc
Image resample_vertical(const Image& in, int out_height, const Coefficients& c) {
    const int nb = in.bands();
    Image out{in.mode, in.width, out_height, {}, {}};
    out.pixels.resize(static_cast<std::size_t>(in.width) * static_cast<std::size_t>(out_height) * nb);
    for (int yy = 0; yy < out_height; ++yy) {
        const int* k = &c.kk[static_cast<std::size_t>(yy) * static_cast<std::size_t>(c.ksize)];
        const int ymin = c.bounds[static_cast<std::size_t>(yy) * 2];
        const int ymax = c.bounds[static_cast<std::size_t>(yy) * 2 + 1];
        std::uint8_t* line_out = &out.pixels[static_cast<std::size_t>(yy) * in.width * nb];
        for (int xx = 0; xx < in.width; ++xx) {
            for (int b = 0; b < nb; ++b) {
                int ss = 1 << (kPrecisionBits - 1);
                for (int y = 0; y < ymax; ++y) {
                    ss += in.pixels[(static_cast<std::size_t>(y + ymin) * in.width + xx) * nb + b] * k[y];
                }
                line_out[xx * nb + b] = clip8(ss);
            }
        }
    }
    return out;
}

const char kBase64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

}  // namespace

int Image::bands() const { return bands_of(mode); }

bool Image::operator==(const Image& other) const {
    return mode == other.mode && width == other.width && height == other.height && pixels == other.pixels &&
           palette == other.palette;
}

Bytes read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("No such file or directory: '" + path + "'");
    return Bytes(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

Image open_png(const Bytes& data) {
    lodepng::State state;
    state.decoder.color_convert = 0;
    std::vector<unsigned char> raw;
    unsigned w = 0, h = 0;
    unsigned error = lodepng::decode(raw, w, h, state, data);
    if (error) throw std::runtime_error(std::string("cannot identify image file: ") + lodepng_error_text(error));
    const LodePNGColorMode& color = state.info_png.color;
    Image img;
    img.width = static_cast<int>(w);
    img.height = static_cast<int>(h);
    LodePNGColorType type = color.colortype;
    if (color.bitdepth != 8) {
        // Other bit depths are read as 8 bits per channel (16-bit samples keep their high byte).
        state = lodepng::State();
        state.info_raw.colortype = type == LCT_PALETTE ? LCT_RGBA : type;
        state.info_raw.bitdepth = 8;
        raw.clear();
        error = lodepng::decode(raw, w, h, state, data);
        if (error) throw std::runtime_error(std::string("cannot identify image file: ") + lodepng_error_text(error));
        if (type == LCT_PALETTE) type = LCT_RGBA;
    }
    switch (type) {
        case LCT_GREY: img.mode = "L"; break;
        case LCT_GREY_ALPHA: img.mode = "LA"; break;
        case LCT_RGB: img.mode = "RGB"; break;
        case LCT_RGBA: img.mode = "RGBA"; break;
        case LCT_PALETTE: {
            img.mode = "P";
            for (std::size_t i = 0; i < color.palettesize; ++i) {
                img.palette.push_back({color.palette[i * 4], color.palette[i * 4 + 1], color.palette[i * 4 + 2],
                                       color.palette[i * 4 + 3]});
            }
            // PIL's palette has 256 entries; missing ones are black
            while (img.palette.size() < 256) img.palette.push_back({0, 0, 0, 255});
            break;
        }
        default: throw std::runtime_error("unsupported PNG color type");
    }
    img.pixels.assign(raw.begin(), raw.end());
    return img;
}

Image open_png_file(const std::string& path) { return open_png(read_file(path)); }

Bytes encode_png(const Image& image) {
    lodepng::State state;
    LodePNGColorType type = LCT_RGB;
    if (image.mode == "L") {
        type = LCT_GREY;
    } else if (image.mode == "LA") {
        type = LCT_GREY_ALPHA;
    } else if (image.mode == "RGB") {
        type = LCT_RGB;
    } else if (image.mode == "RGBA") {
        type = LCT_RGBA;
    } else {
        throw std::invalid_argument("can't write mode " + image.mode + " as PNG");
    }
    state.info_raw.colortype = type;
    state.info_raw.bitdepth = 8;
    state.info_png.color.colortype = type;  // keep the mode, as PIL does
    state.info_png.color.bitdepth = 8;
    state.encoder.auto_convert = 0;
    std::vector<unsigned char> out;
    const unsigned error = lodepng::encode(out, image.pixels, static_cast<unsigned>(image.width),
                                           static_cast<unsigned>(image.height), state);
    if (error) throw std::runtime_error(std::string("PNG encoding failed: ") + lodepng_error_text(error));
    return Bytes(out.begin(), out.end());
}

Image convert(const Image& image, const std::string& mode) {
    if (image.mode == mode) return image;
    const std::size_t n = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    Image out{mode, image.width, image.height, {}, {}};
    out.pixels.resize(n * static_cast<std::size_t>(bands_of(mode)));
    const auto& in = image.pixels;
    for (std::size_t i = 0; i < n; ++i) {
        std::uint8_t r, g, b, a = 255, l;
        if (image.mode == "P") {
            const auto& p = image.palette[in[i]];
            r = p[0];
            g = p[1];
            b = p[2];
            l = luma(r, g, b);
        } else if (image.mode == "L") {
            r = g = b = l = in[i];
        } else if (image.mode == "LA") {
            r = g = b = l = in[i * 2];
            a = in[i * 2 + 1];
        } else if (image.mode == "RGB") {
            r = in[i * 3];
            g = in[i * 3 + 1];
            b = in[i * 3 + 2];
            l = luma(r, g, b);
        } else if (image.mode == "RGBA") {
            r = in[i * 4];
            g = in[i * 4 + 1];
            b = in[i * 4 + 2];
            a = in[i * 4 + 3];
            l = luma(r, g, b);
        } else {
            throw std::invalid_argument("unsupported image mode " + image.mode);
        }
        if (mode == "L") {
            out.pixels[i] = image.mode == "LA" ? r : l;
        } else if (mode == "LA") {
            out.pixels[i * 2] = l;
            out.pixels[i * 2 + 1] = a;
        } else if (mode == "RGB") {
            out.pixels[i * 3] = r;
            out.pixels[i * 3 + 1] = g;
            out.pixels[i * 3 + 2] = b;
        } else if (mode == "RGBA") {
            out.pixels[i * 4] = r;
            out.pixels[i * 4 + 1] = g;
            out.pixels[i * 4 + 2] = b;
            out.pixels[i * 4 + 3] = image.mode == "P" ? image.palette[in[i]][3] : a;
        } else {
            throw std::invalid_argument("unsupported conversion to " + mode);
        }
    }
    return out;
}

Image resize_lanczos(const Image& image, int width, int height) {
    if (image.mode == "P") throw std::invalid_argument("palette images aren't resampled with LANCZOS");
    if (image.mode == "LA" || image.mode == "RGBA") {
        throw std::invalid_argument("resampling images with alpha isn't supported");
    }
    if (width == image.width && height == image.height) return image;
    const int in_w = image.width;
    const int in_h = image.height;
    const bool need_horizontal = width != in_w;
    const bool need_vertical = height != in_h;
    const bool horizontal_first = !((in_h - height) > 0 && (in_h - height) > (in_w - width) * 2);
    Coefficients vertical;
    Coefficients horizontal;
    int ybox_first = 0;
    int ybox_last = in_h;
    if ((need_horizontal && horizontal_first) || need_vertical) {
        vertical = precompute_coeffs(in_h, 0.0f, static_cast<float>(in_h), height);
    }
    if (need_horizontal) {
        if (horizontal_first) {
            ybox_first = vertical.bounds[0];
            ybox_last = vertical.bounds[static_cast<std::size_t>(height) * 2 - 2] +
                        vertical.bounds[static_cast<std::size_t>(height) * 2 - 1];
            if (ybox_first != 0) {
                for (int i = 0; i < height; ++i) vertical.bounds[static_cast<std::size_t>(i) * 2] -= ybox_first;
            }
        }
        horizontal = precompute_coeffs(in_w, 0.0f, static_cast<float>(in_w), width);
    }
    Image current = image;
    if (horizontal_first) {
        if (need_horizontal) current = resample_horizontal(current, width, ybox_last - ybox_first, ybox_first, horizontal);
        if (need_vertical) current = resample_vertical(current, height, vertical);
    } else {
        if (need_vertical) current = resample_vertical(current, height, vertical);
        if (need_horizontal) current = resample_horizontal(current, width, height, 0, horizontal);
    }
    return current;
}

Image channel(const Image& image, int band) {
    const int nb = image.bands();
    if (band < 0 || band >= nb) throw std::invalid_argument("band index out of range");
    Image out{"L", image.width, image.height, {}, {}};
    const std::size_t n = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    out.pixels.resize(n);
    for (std::size_t i = 0; i < n; ++i) out.pixels[i] = image.pixels[i * nb + static_cast<std::size_t>(band)];
    return out;
}

Image new_image(const std::string& mode, int width, int height, const std::vector<std::uint8_t>& color) {
    const int nb = bands_of(mode);
    Image out{mode, width, height, {}, {}};
    out.pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * nb);
    for (std::size_t i = 0; i < out.pixels.size(); ++i) out.pixels[i] = color[i % static_cast<std::size_t>(nb)];
    return out;
}

Image put_alpha(const Image& image, const Image& alpha) {
    const Image rgb = convert(image, "RGB");
    Image out{"RGBA", rgb.width, rgb.height, {}, {}};
    const std::size_t n = static_cast<std::size_t>(rgb.width) * static_cast<std::size_t>(rgb.height);
    out.pixels.resize(n * 4);
    for (std::size_t i = 0; i < n; ++i) {
        out.pixels[i * 4] = rgb.pixels[i * 3];
        out.pixels[i * 4 + 1] = rgb.pixels[i * 3 + 1];
        out.pixels[i * 4 + 2] = rgb.pixels[i * 3 + 2];
        out.pixels[i * 4 + 3] = alpha.pixels[i];
    }
    return out;
}

std::string base64_encode(const Bytes& data) {
    std::string out;
    out.reserve((data.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < data.size(); i += 3) {
        const unsigned v = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
        out += kBase64[(v >> 18) & 63];
        out += kBase64[(v >> 12) & 63];
        out += kBase64[(v >> 6) & 63];
        out += kBase64[v & 63];
    }
    if (i + 1 == data.size()) {
        const unsigned v = data[i] << 16;
        out += kBase64[(v >> 18) & 63];
        out += kBase64[(v >> 12) & 63];
        out += "==";
    } else if (i + 2 == data.size()) {
        const unsigned v = (data[i] << 16) | (data[i + 1] << 8);
        out += kBase64[(v >> 18) & 63];
        out += kBase64[(v >> 12) & 63];
        out += kBase64[(v >> 6) & 63];
        out += '=';
    }
    return out;
}

Bytes base64_decode(const std::string& text) {
    int table[256];
    for (int& t : table) t = -1;
    for (int i = 0; i < 64; ++i) table[static_cast<unsigned char>(kBase64[i])] = i;
    Bytes out;
    unsigned buffer = 0;
    int bits = 0;
    for (const char ch : text) {
        if (ch == '=') break;
        const int v = table[static_cast<unsigned char>(ch)];
        if (v < 0) continue;
        buffer = (buffer << 6) | static_cast<unsigned>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<std::uint8_t>((buffer >> bits) & 0xFF));
        }
    }
    return out;
}

}  // namespace build123d::detail::pil
