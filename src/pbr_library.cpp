// threejs_materials' PbrProperties model, its preset library and its color helpers
// (library.py, models.py, utils.py, pbr_properties/*.py), ported step by step.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <regex>
#include <stdexcept>

#include "build123d/pbr.hpp"
#include "build123d/warnings.hpp"
#include "io_internal.hpp"
#include "pbr_internal.hpp"
#include "pyformat.hpp"
#include "textures.hpp"

namespace build123d {

namespace {

namespace pil = detail::pil;

// ---------------------------------------------------------------------------
// Python helpers
// ---------------------------------------------------------------------------

double libm_pow(double x, double y) {
    static double (*volatile fn)(double, double) = static_cast<double (*)(double, double)>(&std::pow);
    return fn(x, y);
}

/// Python's float ** float for the positive values used here.
double py_pow(double x, double y) {
    if (y == 0) return 1.0;
    if (x == 0.0) return 0.0;
    if (x == 1.0) return 1.0;
    return libm_pow(x, y);
}

double py_mod(double x, double y) {
    double mod = std::fmod(x, y);
    if (mod != 0.0) {
        if ((y < 0) != (mod < 0)) mod += y;
    } else {
        mod = std::copysign(0.0, y);
    }
    return mod;
}

std::string strip(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

/// int(text, 16)
int parse_hex(const std::string& text) {
    if (text.empty()) throw std::invalid_argument("invalid literal for int() with base 16: ''");
    int v = 0;
    for (const char c : text) {
        const int d = std::isdigit(static_cast<unsigned char>(c)) ? c - '0'
                      : (c >= 'a' && c <= 'f')                     ? c - 'a' + 10
                      : (c >= 'A' && c <= 'F')                     ? c - 'A' + 10
                                                                   : -1;
        if (d < 0) throw std::invalid_argument("invalid literal for int() with base 16: " + detail::pbr::repr_str(text));
        v = v * 16 + d;
    }
    return v;
}

// ---------------------------------------------------------------------------
// BLAKE2b (RFC 7693), for library._hash_override (hashlib.blake2b(digest_size=4))
// ---------------------------------------------------------------------------

std::uint64_t rotr64(std::uint64_t x, int n) { return (x >> n) | (x << (64 - n)); }

std::string blake2b_hex(const std::string& data, std::size_t outlen) {
    static const std::uint64_t iv[8] = {0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
                                        0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
                                        0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL};
    static const std::uint8_t sigma[12][16] = {
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, {14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3},
        {11, 8, 12, 0, 5, 2, 15, 13, 10, 14, 3, 6, 7, 1, 9, 4}, {7, 9, 3, 1, 13, 12, 11, 14, 2, 6, 5, 10, 4, 0, 15, 8},
        {9, 0, 5, 7, 2, 4, 10, 15, 14, 1, 11, 12, 6, 8, 3, 13}, {2, 12, 6, 10, 0, 11, 8, 3, 4, 13, 7, 5, 15, 14, 1, 9},
        {12, 5, 1, 15, 14, 13, 4, 10, 0, 7, 6, 3, 9, 2, 8, 11}, {13, 11, 7, 14, 12, 1, 3, 9, 5, 0, 15, 4, 8, 6, 2, 10},
        {6, 15, 14, 9, 11, 3, 0, 8, 12, 2, 13, 7, 1, 4, 10, 5}, {10, 2, 8, 4, 7, 6, 1, 5, 15, 11, 9, 14, 3, 12, 13, 0},
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, {14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3}};
    std::uint64_t h[8];
    for (int i = 0; i < 8; ++i) h[i] = iv[i];
    h[0] ^= 0x01010000ULL ^ static_cast<std::uint64_t>(outlen);
    std::uint64_t t0 = 0, t1 = 0;
    auto compress = [&](const std::uint8_t* block, bool last) {
        std::uint64_t m[16], v[16];
        for (int i = 0; i < 16; ++i) {
            m[i] = 0;
            for (int b = 7; b >= 0; --b) m[i] = (m[i] << 8) | block[i * 8 + b];
        }
        for (int i = 0; i < 8; ++i) {
            v[i] = h[i];
            v[i + 8] = iv[i];
        }
        v[12] ^= t0;
        v[13] ^= t1;
        if (last) v[14] = ~v[14];
        auto g = [&](int a, int b, int c, int d, std::uint64_t x, std::uint64_t y) {
            v[a] = v[a] + v[b] + x;
            v[d] = rotr64(v[d] ^ v[a], 32);
            v[c] = v[c] + v[d];
            v[b] = rotr64(v[b] ^ v[c], 24);
            v[a] = v[a] + v[b] + y;
            v[d] = rotr64(v[d] ^ v[a], 16);
            v[c] = v[c] + v[d];
            v[b] = rotr64(v[b] ^ v[c], 63);
        };
        for (int r = 0; r < 12; ++r) {
            const std::uint8_t* s = sigma[r];
            g(0, 4, 8, 12, m[s[0]], m[s[1]]);
            g(1, 5, 9, 13, m[s[2]], m[s[3]]);
            g(2, 6, 10, 14, m[s[4]], m[s[5]]);
            g(3, 7, 11, 15, m[s[6]], m[s[7]]);
            g(0, 5, 10, 15, m[s[8]], m[s[9]]);
            g(1, 6, 11, 12, m[s[10]], m[s[11]]);
            g(2, 7, 8, 13, m[s[12]], m[s[13]]);
            g(3, 4, 9, 14, m[s[14]], m[s[15]]);
        }
        for (int i = 0; i < 8; ++i) h[i] ^= v[i] ^ v[i + 8];
    };
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(data.data());
    std::size_t remaining = data.size();
    std::uint8_t block[128];
    while (remaining > 128) {
        t0 += 128;
        if (t0 < 128) ++t1;
        compress(bytes, false);
        bytes += 128;
        remaining -= 128;
    }
    std::memset(block, 0, sizeof(block));
    std::memcpy(block, bytes, remaining);
    t0 += remaining;
    if (t0 < remaining) ++t1;
    compress(block, true);
    std::string hex;
    char buf[3];
    for (std::size_t i = 0; i < outlen; ++i) {
        std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>((h[i / 8] >> (8 * (i % 8))) & 0xFF));
        hex += buf;
    }
    return hex;
}

/// library._hash_override: blake2b(json.dumps([parent_id, sorted(overrides.items())])).
std::string hash_override(const std::string& parent_id, std::vector<std::pair<std::string, Json>> overrides) {
    std::stable_sort(overrides.begin(), overrides.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    Json items = Json::array();
    for (auto& [key, value] : overrides) items.push_back(Json::array({Json(key), value}));
    const Json payload = Json::array({Json(parent_id), items});
    return blake2b_hex(payload.dump(), 4);
}

// ---------------------------------------------------------------------------
// PIL.ImageColor and colorsys
// ---------------------------------------------------------------------------

const std::pair<const char*, std::array<int, 3>> kColormap[] = {
#include "pil_colormap.inc"
};

double colorsys_v(double m1, double m2, double hue) {
    hue = py_mod(hue, 1.0);
    if (hue < 1.0 / 6.0) return m1 + (m2 - m1) * hue * 6.0;
    if (hue < 0.5) return m2;
    if (hue < 2.0 / 3.0) return m1 + (m2 - m1) * (2.0 / 3.0 - hue) * 6.0;
    return m1;
}

std::array<double, 3> hls_to_rgb(double h, double l, double s) {
    if (s == 0.0) return {l, l, l};
    const double m2 = l <= 0.5 ? l * (1.0 + s) : l + s - (l * s);
    const double m1 = 2.0 * l - m2;
    return {colorsys_v(m1, m2, h + 1.0 / 3.0), colorsys_v(m1, m2, h), colorsys_v(m1, m2, h - 1.0 / 3.0)};
}

std::array<double, 3> hsv_to_rgb(double h, double s, double v) {
    if (s == 0.0) return {v, v, v};
    int i = static_cast<int>(h * 6.0);
    const double f = (h * 6.0) - i;
    const double p = v * (1.0 - s);
    const double q = v * (1.0 - s * f);
    const double t = v * (1.0 - s * (1.0 - f));
    i = ((i % 6) + 6) % 6;
    switch (i) {
        case 0: return {v, t, p};
        case 1: return {q, v, p};
        case 2: return {p, v, t};
        case 3: return {p, q, v};
        case 4: return {t, p, v};
        default: return {v, p, q};
    }
}

/// re.match(pattern + "$", text): `$` also matches before a final newline.
bool full_match(const std::string& text, const std::regex& re, std::smatch& m) {
    if (std::regex_match(text, m, re)) return true;
    return false;
}

}  // namespace

namespace detail::pbr {

std::vector<int> getrgb(const std::string& color_in) {
    if (color_in.size() > 100) throw std::invalid_argument("color specifier is too long");
    std::string color = lower(color_in);
    for (const auto& [name, rgb] : kColormap) {
        if (color == name) return {rgb[0], rgb[1], rgb[2]};
    }
    std::string text = color;
    if (!text.empty() && text.back() == '\n') text.pop_back();
    static const std::regex hex3("#[a-f0-9]{3}");
    static const std::regex hex4("#[a-f0-9]{4}");
    static const std::regex hex6("#[a-f0-9]{6}");
    static const std::regex hex8("#[a-f0-9]{8}");
    static const std::regex rgb_int(R"(rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\))");
    static const std::regex rgb_pct(R"(rgb\(\s*(\d+)%\s*,\s*(\d+)%\s*,\s*(\d+)%\s*\))");
    static const std::regex hsl(R"(hsl\(\s*(\d+\.?\d*)\s*,\s*(\d+\.?\d*)%\s*,\s*(\d+\.?\d*)%\s*\))");
    static const std::regex hsv(R"(hs[bv]\(\s*(\d+\.?\d*)\s*,\s*(\d+\.?\d*)%\s*,\s*(\d+\.?\d*)%\s*\))");
    static const std::regex rgba(R"(rgba\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\))");
    std::smatch m;
    auto twice = [&text](std::size_t i) { return parse_hex(std::string(2, text[i])); };
    if (full_match(text, hex3, m)) return {twice(1), twice(2), twice(3)};
    if (full_match(text, hex4, m)) return {twice(1), twice(2), twice(3), twice(4)};
    if (full_match(text, hex6, m)) {
        return {parse_hex(text.substr(1, 2)), parse_hex(text.substr(3, 2)), parse_hex(text.substr(5, 2))};
    }
    if (full_match(text, hex8, m)) {
        return {parse_hex(text.substr(1, 2)), parse_hex(text.substr(3, 2)), parse_hex(text.substr(5, 2)),
                parse_hex(text.substr(7, 2))};
    }
    auto num = [&m](int i) { return std::stoll(m[i].str()); };
    auto real = [&m](int i) { return std::stod(m[i].str()); };
    if (full_match(text, rgb_int, m)) {
        return {static_cast<int>(num(1)), static_cast<int>(num(2)), static_cast<int>(num(3))};
    }
    if (full_match(text, rgb_pct, m)) {
        return {static_cast<int>((num(1) * 255) / 100.0 + 0.5), static_cast<int>((num(2) * 255) / 100.0 + 0.5),
                static_cast<int>((num(3) * 255) / 100.0 + 0.5)};
    }
    if (full_match(text, hsl, m)) {
        const auto rgb = hls_to_rgb(real(1) / 360.0, real(3) / 100.0, real(2) / 100.0);
        return {static_cast<int>(rgb[0] * 255 + 0.5), static_cast<int>(rgb[1] * 255 + 0.5),
                static_cast<int>(rgb[2] * 255 + 0.5)};
    }
    if (full_match(text, hsv, m)) {
        const auto rgb = hsv_to_rgb(real(1) / 360.0, real(2) / 100.0, real(3) / 100.0);
        return {static_cast<int>(rgb[0] * 255 + 0.5), static_cast<int>(rgb[1] * 255 + 0.5),
                static_cast<int>(rgb[2] * 255 + 0.5)};
    }
    if (full_match(text, rgba, m)) {
        return {static_cast<int>(num(1)), static_cast<int>(num(2)), static_cast<int>(num(3)), static_cast<int>(num(4))};
    }
    throw std::invalid_argument("unknown color specifier: " + repr_str(color));
}

double srgb_to_linear(double c) {
    if (c <= 0.04045) return c / 12.92;
    return py_pow((c + 0.055) / 1.055, 2.4);
}

double linear_to_srgb(double c) {
    c = std::max(0.0, std::min(1.0, c));
    if (c <= 0.0031308) return c * 12.92;
    return 1.055 * py_pow(c, 1.0 / 2.4) - 0.055;
}

namespace {

std::pair<std::array<double, 3>, std::optional<double>> normalize(const ColorSpec& c, bool linear) {
    auto convert = [linear](int v) { return linear ? srgb_to_linear(v / 255.0) : v / 255.0; };
    switch (c.kind()) {
        case ColorSpec::Kind::NAME: {
            const std::string s = strip(c.name());
            if (!s.empty() && s[0] == '#' && s.size() == 9) {
                const auto rgb = getrgb(s.substr(0, 7));
                const double a = parse_hex(s.substr(7, 2)) / 255.0;
                return {{convert(rgb[0]), convert(rgb[1]), convert(rgb[2])}, a};
            }
            const auto rgb = getrgb(s);
            return {{convert(rgb[0]), convert(rgb[1]), convert(rgb[2])}, std::nullopt};
        }
        case ColorSpec::Kind::FLOATS: {
            const auto& v = c.values();
            if (v.size() == 3) return {{v[0], v[1], v[2]}, std::nullopt};
            if (v.size() == 4) return {{v[0], v[1], v[2]}, v[3]};
            throw std::invalid_argument("Color tuple/list must have 3 or 4 elements, got " + std::to_string(v.size()));
        }
        case ColorSpec::Kind::NAME_ALPHA:
            throw std::invalid_argument("Color tuple/list must have 3 or 4 elements, got 2");
        case ColorSpec::Kind::INT: throw std::invalid_argument("Unsupported color type: int");
        case ColorSpec::Kind::COLOR: break;
    }
    throw std::invalid_argument("Unsupported color type: Color");
}

}  // namespace

std::pair<std::array<double, 3>, std::optional<double>> normalize_srgb_color(const ColorSpec& color) {
    return normalize(color, false);
}

std::pair<std::array<double, 3>, std::optional<double>> normalize_color(const ColorSpec& color) {
    return normalize(color, true);
}

bool is_data_uri(const std::string& s) { return s.rfind("data:", 0) == 0; }

pil::Bytes texture_bytes(const std::string& ref, const std::optional<std::string>& texture_dir) {
    if (is_data_uri(ref)) {
        const auto comma = ref.find(',');
        if (comma == std::string::npos) throw std::invalid_argument("not enough values to unpack (expected 2, got 1)");
        return pil::base64_decode(ref.substr(comma + 1));
    }
    const std::string path = texture_dir ? *texture_dir + "/" + ref : ref;
    try {
        return pil::read_file(path);
    } catch (const std::runtime_error&) {
        if (find_texture_dir().empty()) throw_missing_textures(path);
        throw;
    }
}

pil::Image open_texture_image(const std::string& ref, const std::optional<std::string>& texture_dir) {
    pil::Image img = pil::open_png(texture_bytes(ref, texture_dir));
    if (img.mode == "P") img = pil::convert(img, "L");
    return img;
}

std::string resolve_to_data_uri(const std::string& ref, const std::string& texture_dir) {
    if (is_data_uri(ref)) return ref;
    const pil::Bytes data = texture_bytes(ref, texture_dir);
    pil::Image img;
    bool png = true;
    try {
        img = pil::open_png(data);
    } catch (const std::runtime_error&) {
        png = false;
    }
    if (png && img.mode == "P") {
        return "data:image/png;base64," + pil::base64_encode(pil::encode_png(pil::convert(img, "L")));
    }
    const std::string ext = lower(ref.substr(ref.find_last_of('.') == std::string::npos ? ref.size() : ref.find_last_of('.')));
    std::string mime = "application/octet-stream";
    if (ext == ".png") mime = "image/png";
    if (ext == ".jpg" || ext == ".jpeg") mime = "image/jpeg";
    return "data:" + mime + ";base64," + pil::base64_encode(data);
}

std::string repr_str(const std::string& s) {
    const char quote = (s.find('\'') != std::string::npos && s.find('"') == std::string::npos) ? '"' : '\'';
    std::string out(1, quote);
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        if (ch == quote || ch == '\\') {
            out += '\\';
            out += ch;
        } else if (ch == '\t') {
            out += "\\t";
        } else if (ch == '\n') {
            out += "\\n";
        } else if (ch == '\r') {
            out += "\\r";
        } else if (c < 0x20 || c == 0x7f) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\x%02x", c);
            out += buf;
        } else {
            out += ch;
        }
    }
    return out + quote;
}

std::string repr_floats(const std::vector<double>& values, bool tuple) {
    std::string out = tuple ? "(" : "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out += ", ";
        out += pyfmt::repr(values[i]);
    }
    if (tuple && values.size() == 1) out += ",";
    return out + (tuple ? ")" : "]");
}

}  // namespace detail::pbr

using namespace detail::pbr;

// ---------------------------------------------------------------------------
// ColorSpec
// ---------------------------------------------------------------------------

ColorSpec::ColorSpec(const Color& color) : kind_(Kind::COLOR) {
    const auto t = detail::python_color_tuple(color);
    values_ = {t[0], t[1], t[2], t[3]};
}

std::string ColorSpec::repr() const {
    switch (kind_) {
        case Kind::NAME: return repr_str(name_);
        case Kind::INT: return std::to_string(packed_);
        case Kind::NAME_ALPHA: return "(" + repr_str(name_) + ", " + pyfmt::repr(alpha_) + ")";
        case Kind::FLOATS: return repr_floats(values_, true);
        case Kind::COLOR: break;
    }
    return "Color(" + pyfmt::repr(values_[0]) + ", " + pyfmt::repr(values_[1]) + ", " + pyfmt::repr(values_[2]) +
           ", " + pyfmt::repr(values_[3]) + ")";
}

bool ColorSpec::operator==(const ColorSpec& other) const {
    return kind_ == other.kind_ && name_ == other.name_ && packed_ == other.packed_ && alpha_ == other.alpha_ &&
           values_ == other.values_;
}

// ---------------------------------------------------------------------------
// PbrValues / PbrMaps
// ---------------------------------------------------------------------------

namespace {

// (field, three.js name, kind): N number, L list, B bool, I int
#define B3D_PBR_VALUE_FIELDS(X)                                            \
    X(color, "color", L)                                                   \
    X(metalness, "metalness", N)                                           \
    X(roughness, "roughness", N)                                           \
    X(ior, "ior", N)                                                       \
    X(normal_scale, "normalScale", L)                                      \
    X(emissive, "emissive", L)                                             \
    X(emissive_intensity, "emissiveIntensity", N)                          \
    X(transmission, "transmission", N)                                     \
    X(opacity, "opacity", N)                                               \
    X(transparent, "transparent", B)                                       \
    X(alpha_test, "alphaTest", N)                                          \
    X(clearcoat, "clearcoat", N)                                           \
    X(clearcoat_roughness, "clearcoatRoughness", N)                        \
    X(sheen, "sheen", N)                                                   \
    X(sheen_color, "sheenColor", L)                                        \
    X(sheen_roughness, "sheenRoughness", N)                                \
    X(anisotropy, "anisotropy", N)                                         \
    X(anisotropy_rotation, "anisotropyRotation", N)                        \
    X(specular_intensity, "specularIntensity", N)                          \
    X(specular_color, "specularColor", L)                                  \
    X(attenuation_color, "attenuationColor", L)                            \
    X(attenuation_distance, "attenuationDistance", N)                      \
    X(thickness, "thickness", N)                                           \
    X(iridescence, "iridescence", N)                                       \
    X(iridescence_ior, "iridescenceIOR", N)                                \
    X(iridescence_thickness_range, "iridescenceThicknessRange", L)         \
    X(dispersion, "dispersion", N)                                         \
    X(displacement_scale, "displacementScale", N)                          \
    X(side, "side", I)

#define B3D_PBR_MAP_FIELDS(X)                         \
    X(color, "color")                                 \
    X(metalness, "metalness")                         \
    X(roughness, "roughness")                         \
    X(normal, "normal")                               \
    X(emissive, "emissive")                           \
    X(ao, "ao")                                       \
    X(opacity, "opacity")                             \
    X(clearcoat, "clearcoat")                         \
    X(clearcoat_roughness, "clearcoatRoughness")      \
    X(clearcoat_normal, "clearcoatNormal")            \
    X(transmission, "transmission")                   \
    X(sheen_color, "sheenColor")                      \
    X(sheen_roughness, "sheenRoughness")              \
    X(anisotropy, "anisotropy")                       \
    X(iridescence, "iridescence")                     \
    X(specular_intensity, "specularIntensity")        \
    X(specular_color, "specularColor")                \
    X(thickness, "thickness")                         \
    X(displacement, "displacement")                   \
    X(metallic_roughness, "metallicRoughness")

Json to_json(double v) { return Json(v); }
Json to_json(bool v) { return Json(v); }
Json to_json(int v) { return Json(v); }
Json to_json(const std::vector<double>& v) {
    Json out = Json::array();
    for (const double x : v) out.push_back(Json(x));
    return out;
}

void from_json(const Json& j, std::optional<double>& out) { out = j.as_double(); }
void from_json(const Json& j, std::optional<bool>& out) { out = j.as_bool(); }
void from_json(const Json& j, std::optional<int>& out) { out = static_cast<int>(j.as_int()); }
void from_json(const Json& j, std::optional<std::vector<double>>& out) {
    std::vector<double> v;
    for (const Json& x : j.as_array()) v.push_back(x.as_double());
    out = v;
}

std::string repr_value(double v) { return pyfmt::repr(v); }
std::string repr_value(bool v) { return v ? "True" : "False"; }
std::string repr_value(int v) { return std::to_string(v); }
std::string repr_value(const std::vector<double>& v) { return repr_floats(v, false); }

std::string repr_map(const std::string& v) {
    return is_data_uri(v) ? std::string("'data:...;base64,...'") : repr_str(v);
}

/// models._to_snake
const std::map<std::string, std::string>& camel_to_snake() {
    static const std::map<std::string, std::string> m = {
        {"normalScale", "normal_scale"},
        {"emissiveIntensity", "emissive_intensity"},
        {"alphaTest", "alpha_test"},
        {"clearcoatRoughness", "clearcoat_roughness"},
        {"sheenColor", "sheen_color"},
        {"sheenRoughness", "sheen_roughness"},
        {"anisotropyRotation", "anisotropy_rotation"},
        {"specularIntensity", "specular_intensity"},
        {"specularColor", "specular_color"},
        {"attenuationColor", "attenuation_color"},
        {"attenuationDistance", "attenuation_distance"},
        {"iridescenceIOR", "iridescence_ior"},
        {"iridescenceThicknessRange", "iridescence_thickness_range"},
        {"displacementScale", "displacement_scale"},
        {"clearcoatNormal", "clearcoat_normal"},
        {"metallicRoughness", "metallic_roughness"},
    };
    return m;
}

std::string to_snake(const std::string& name) {
    const auto it = camel_to_snake().find(name);
    return it == camel_to_snake().end() ? name : it->second;
}

}  // namespace

Json PbrValues::to_dict() const {
    Json out = Json::object();
#define X(field, camel, kind) \
    if (field) out[camel] = to_json(*field);
    B3D_PBR_VALUE_FIELDS(X)
#undef X
    return out;
}

PbrValues PbrValues::from_dict(const Json& data) {
    PbrValues v;
    for (const auto& [key, value] : data.as_object()) {
        const std::string snake = to_snake(key);
#define X(field, camel, kind) \
    if (snake == #field) {    \
        from_json(value, v.field); \
        continue;             \
    }
        B3D_PBR_VALUE_FIELDS(X)
#undef X
    }
    return v;
}

std::string PbrValues::repr() const {
    std::vector<std::string> parts;
#define X(field, camel, kind) \
    if (field) parts.push_back(std::string(#field) + "=" + repr_value(*field));
    B3D_PBR_VALUE_FIELDS(X)
#undef X
    std::string out = "PbrValues(";
    for (std::size_t i = 0; i < parts.size(); ++i) out += (i ? ", " : "") + parts[i];
    return out + ")";
}

bool PbrValues::operator==(const PbrValues& o) const {
#define X(field, camel, kind) \
    if (field != o.field) return false;
    B3D_PBR_VALUE_FIELDS(X)
#undef X
    return true;
}

Json PbrMaps::to_dict() const {
    Json out = Json::object();
#define X(field, camel) \
    if (field) out[camel] = Json(*field);
    B3D_PBR_MAP_FIELDS(X)
#undef X
    return out;
}

PbrMaps PbrMaps::from_dict(const Json& data) {
    PbrMaps m;
    for (const auto& [key, value] : data.as_object()) {
        const std::string snake = to_snake(key);
        if (value.is_null()) continue;
#define X(field, camel)        \
    if (snake == #field) {     \
        m.field = value.as_string(); \
        continue;              \
    }
        B3D_PBR_MAP_FIELDS(X)
#undef X
    }
    return m;
}

std::string PbrMaps::repr() const {
    std::vector<std::string> parts;
#define X(field, camel) \
    if (field) parts.push_back(std::string(#field) + "=" + repr_map(*field));
    B3D_PBR_MAP_FIELDS(X)
#undef X
    std::string out = "PbrMaps(";
    for (std::size_t i = 0; i < parts.size(); ++i) out += (i ? ", " : "") + parts[i];
    return out + ")";
}

bool PbrMaps::operator==(const PbrMaps& o) const {
#define X(field, camel) \
    if (field != o.field) return false;
    B3D_PBR_MAP_FIELDS(X)
#undef X
    return true;
}

// ---------------------------------------------------------------------------
// PbrProperties
// ---------------------------------------------------------------------------

PbrProperties PbrProperties::from_dict(const Json& data) {
    PbrProperties p;
    p.id = data["id"].as_string();
    p.name = data["name"].as_string();
    p.source = data["source"].as_string();
    p.url = data.contains("url") ? data["url"].as_string() : "";
    p.license = data.contains("license") ? data["license"].as_string() : "";
    if (data.contains("values")) p.values = PbrValues::from_dict(data["values"]);
    if (data.contains("textures")) p.maps = PbrMaps::from_dict(data["textures"]);
    if (data.contains("texture_repeat") && !data["texture_repeat"].is_null()) {
        const auto& r = data["texture_repeat"].as_array();
        p.texture_repeat = std::array<double, 2>{r.at(0).as_double(), r.at(1).as_double()};
    }
    if (data.contains("texture_rotation") && !data["texture_rotation"].is_null()) {
        p.texture_rotation = data["texture_rotation"].as_double();
    }
    if (data.contains("normalize_uvs")) p.normalize_uvs = data["normalize_uvs"].as_bool();
    if (data.contains("maps_dir") && !data["maps_dir"].is_null()) p.maps_dir = data["maps_dir"].as_string();
    return p;
}

PbrProperties PbrProperties::override(const PbrOverrides& o) const {
    // the keyword arguments that are set, in override()'s order, colors normalized
    // (_normalize_color_overrides)
    std::vector<std::pair<std::string, Json>> overrides;
    PbrValues v = values;
    std::optional<double> color_alpha;
    if (o.color) {
        const auto [rgb, alpha] = normalize_srgb_color(*o.color);
        v.color = std::vector<double>{rgb[0], rgb[1], rgb[2]};
        overrides.emplace_back("color", to_json(*v.color));
        color_alpha = alpha;
    }
    auto number = [&](const char* key, const std::optional<double>& value, std::optional<double>& field) {
        if (!value) return;
        field = *value;
        overrides.emplace_back(key, Json(*value));
    };
    auto linear = [&](const char* key, const std::optional<ColorSpec>& value,
                      std::optional<std::vector<double>>& field) {
        if (!value) return;
        const auto rgb = normalize_color(*value).first;
        field = std::vector<double>{rgb[0], rgb[1], rgb[2]};
        overrides.emplace_back(key, to_json(*field));
    };
    auto pair = [&](const char* key, const std::optional<std::array<double, 2>>& value,
                    std::optional<std::vector<double>>& field) {
        if (!value) return;
        field = std::vector<double>{(*value)[0], (*value)[1]};
        overrides.emplace_back(key, to_json(*field));
    };
    number("roughness", o.roughness, v.roughness);
    number("metalness", o.metalness, v.metalness);
    number("ior", o.ior, v.ior);
    number("transmission", o.transmission, v.transmission);
    number("opacity", o.opacity, v.opacity);
    if (o.transparent) {
        v.transparent = *o.transparent;
        overrides.emplace_back("transparent", Json(*o.transparent));
    }
    number("alpha_test", o.alpha_test, v.alpha_test);
    number("clearcoat", o.clearcoat, v.clearcoat);
    number("clearcoat_roughness", o.clearcoat_roughness, v.clearcoat_roughness);
    number("sheen", o.sheen, v.sheen);
    linear("sheen_color", o.sheen_color, v.sheen_color);
    number("sheen_roughness", o.sheen_roughness, v.sheen_roughness);
    number("anisotropy", o.anisotropy, v.anisotropy);
    number("anisotropy_rotation", o.anisotropy_rotation, v.anisotropy_rotation);
    number("specular_intensity", o.specular_intensity, v.specular_intensity);
    linear("specular_color", o.specular_color, v.specular_color);
    linear("emissive", o.emissive, v.emissive);
    number("emissive_intensity", o.emissive_intensity, v.emissive_intensity);
    linear("attenuation_color", o.attenuation_color, v.attenuation_color);
    number("attenuation_distance", o.attenuation_distance, v.attenuation_distance);
    number("thickness", o.thickness, v.thickness);
    number("iridescence", o.iridescence, v.iridescence);
    number("iridescence_ior", o.iridescence_ior, v.iridescence_ior);
    pair("iridescence_thickness_range", o.iridescence_thickness_range, v.iridescence_thickness_range);
    number("dispersion", o.dispersion, v.dispersion);
    pair("normal_scale", o.normal_scale, v.normal_scale);
    number("displacement_scale", o.displacement_scale, v.displacement_scale);
    if (o.side) {
        v.side = *o.side;
        overrides.emplace_back("side", Json(*o.side));
    }
    if (color_alpha && !o.opacity) {
        v.opacity = *color_alpha;
        overrides.emplace_back("opacity", Json(*color_alpha));
    }
    PbrProperties out = *this;
    out.values = v;
    if (!overrides.empty()) out.id = name + "_" + hash_override(id, overrides);
    return out;
}

PbrProperties PbrProperties::scale(double u, double v, bool fixed, double rotation) const {
    return detail::pbr::scale(*this, Json(u), Json(v), fixed, Json(rotation));
}

PbrProperties detail::pbr::scale(const PbrProperties& self, const Json& u, const Json& v, bool fixed,
                                 const Json& rotation) {
    const double du = u.as_double(), dv = v.as_double(), drot = rotation.as_double();
    // 1 / u, then 1 / v: ZeroDivisionError, whose message depends on the operand's type
    for (const Json* d : {&u, &v}) {
        if (d->as_double() == 0) {
            throw std::domain_error(d->type() == Json::Type::INT ? "division by zero" : "float division by zero");
        }
    }
    std::vector<std::pair<std::string, Json>> kwargs = {
        {"u", u}, {"v", v}, {"fixed", Json(fixed)}, {"rotation", rotation}};
    PbrProperties out = self;
    out.id = self.name + "_" + hash_override(self.id, kwargs);
    out.texture_repeat = std::array<double, 2>{1.0 / du, 1.0 / dv};
    out.texture_repeat_tuple = true;
    out.texture_rotation = drot != 0.0 ? std::optional<double>(drot) : std::nullopt;
    out.normalize_uvs = fixed;
    return out;
}

PbrProperties PbrProperties::strip_maps() const {
    PbrProperties out = *this;
    out.maps = PbrMaps();
    out.maps_dir.reset();
    return out;
}

std::array<double, 4> PbrProperties::interpolate_color(const std::optional<ColorSpec>& override_color) const {
    std::optional<std::array<double, 3>> tint;
    if (override_color) {
        tint = normalize_srgb_color(*override_color).first;
    } else if (values.color) {
        const auto& c = *values.color;
        tint = std::array<double, 3>{c.at(0), c.at(1), c.at(2)};
    }
    double sr, sg, sb;
    if (maps.color) {
        pil::Image img;
        if (maps_dir) {
            img = open_texture_image(*maps.color, maps_dir);
        } else {
            if (!is_data_uri(*maps.color)) throw std::invalid_argument("not enough values to unpack (expected 2, got 1)");
            img = pil::open_png(texture_bytes(*maps.color, std::nullopt));
        }
        const pil::Image px = pil::resize_lanczos(pil::convert(img, "RGB"), 1, 1);
        const double lr = srgb_to_linear(px.pixels[0] / 255.0);
        const double lg = srgb_to_linear(px.pixels[1] / 255.0);
        const double lb = srgb_to_linear(px.pixels[2] / 255.0);
        if (tint) {
            const double mr = lr * srgb_to_linear((*tint)[0]);
            const double mg = lg * srgb_to_linear((*tint)[1]);
            const double mb = lb * srgb_to_linear((*tint)[2]);
            sr = linear_to_srgb(std::min(mr, 1.0));
            sg = linear_to_srgb(std::min(mg, 1.0));
            sb = linear_to_srgb(std::min(mb, 1.0));
        } else {
            sr = linear_to_srgb(lr);
            sg = linear_to_srgb(lg);
            sb = linear_to_srgb(lb);
        }
    } else if (tint) {
        sr = (*tint)[0];
        sg = (*tint)[1];
        sb = (*tint)[2];
    } else {
        sr = sg = sb = 0.5;
    }
    double alpha = 1.0;
    if (values.opacity && *values.opacity < 1.0) {
        alpha = *values.opacity;
    } else if (values.transmission && *values.transmission > 0) {
        alpha = std::max(0.15, 1.0 - *values.transmission * 0.7);
    }
    return {pyfmt::round(sr, 4), pyfmt::round(sg, 4), pyfmt::round(sb, 4), pyfmt::round(alpha, 4)};
}

Color PbrProperties::interpolated_color() const {
    const auto c = interpolate_color();
    return Color(c[0], c[1], c[2], c[3]);
}

std::string PbrProperties::dump() const {
    std::string out = "PbrProperties(name=" + repr_str(name) + ", source=" + repr_str(source) +
                      ", license=" + repr_str(license) + ")";
    out += "\n  values:  " + values.repr();
    out += "\n  maps:    " + maps.repr();
    if (texture_repeat) {
        out += "\n  texture_repeat: " + repr_floats({(*texture_repeat)[0], (*texture_repeat)[1]}, texture_repeat_tuple);
    }
    if (!normalize_uvs) out += "\n  normalize_uvs: False";
    if (maps_dir && maps.to_dict().size() > 0) out += "\n  maps_dir: " + *maps_dir;
    return out;
}

// ---------------------------------------------------------------------------
// The preset library (pbr_properties/*.py)
// ---------------------------------------------------------------------------

namespace {

struct PresetSpec {
    const char* category;
    const char* name;
    const char* data;
    const char* maps_dir;
    bool color, roughness, thickness, scale;
    double scale_u, scale_v;
    bool scale_int;
    double rotation;
    bool rotation_compared;
    bool fixed;
};

const PresetSpec kPresets[] = {
#include "pbr_presets.inc"
};

}  // namespace

std::vector<std::string> pbr_preset_names(const std::string& category) {
    std::vector<std::string> out;
    for (const PresetSpec& p : kPresets) {
        if (category == p.category) out.emplace_back(p.name);
    }
    if (out.empty()) throw std::invalid_argument("module 'threejs_materials' has no attribute " + repr_str(category));
    return out;
}

PbrProperties pbr_preset(const std::string& category, const std::string& name, const PbrPresetArgs& args) {
    if (category == "plastic" && name == "plastic_clean") {
        warn("plastic_clean() is deprecated, use plastic() instead.");
        return pbr_preset("plastic", "plastic", args);
    }
    const PresetSpec* spec = nullptr;
    for (const PresetSpec& p : kPresets) {
        if (category == p.category && name == p.name) spec = &p;
    }
    if (spec == nullptr) {
        pbr_preset_names(category);  // unknown category
        throw std::invalid_argument("module 'threejs_materials.pbr_properties." + category +
                                    "' has no attribute " + repr_str(name));
    }
    auto unexpected = [&name](const char* key) {
        throw std::invalid_argument(name + "() got an unexpected keyword argument '" + key + "'");
    };
    if (args.color && !spec->color) unexpected("color");
    if (args.roughness && !spec->roughness) unexpected("roughness");
    if (args.thickness && !spec->thickness) unexpected("thickness");
    if (args.scale && !spec->scale) unexpected("scale");
    if (args.rotation && !spec->scale) unexpected("rotation");

    Json data = Json::parse(spec->data);
    if (spec->maps_dir != nullptr) {
        const std::string root = detail::find_texture_dir();
        data["maps_dir"] = Json((root.empty() ? std::string("$BUILD123D_TEXTURE_DIR") : root) + "/" + spec->maps_dir);
    }
    PbrProperties m = PbrProperties::from_dict(data);
    if (args.color) m = m.override({.color = *args.color});
    if (args.roughness) m = m.override({.roughness = *args.roughness});
    if (args.thickness) m = m.override({.thickness = *args.thickness});
    if (spec->scale) {
        const auto [u, v] = args.scale.value_or(std::pair<double, double>{spec->scale_u, spec->scale_v});
        const double rotation = args.rotation.value_or(spec->rotation);
        const bool rotated = spec->rotation_compared ? rotation != spec->rotation : rotation != 0.0;
        if (u != spec->scale_u || v != spec->scale_v || rotated) {
            // the default scale is Python ints in most factories (scale=(1, 1))
            const bool ints = !args.scale && spec->scale_int;
            const Json ju = ints ? Json(static_cast<long long>(u)) : Json(u);
            const Json jv = ints ? Json(static_cast<long long>(v)) : Json(v);
            m = detail::pbr::scale(m, ju, jv, spec->fixed, Json(rotation));
        }
    }
    return m;
}

}  // namespace build123d
