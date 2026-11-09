// Python build123d's tuple(Color) and str(Color) (geometry.py Color.__iter__, __str__).
#include <cctype>
#include <cmath>
#include <cstdint>

#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>

#include "io_internal.hpp"
#include "pyformat.hpp"

namespace build123d::detail {

namespace {

const std::pair<std::uint32_t, const char*> kCss3Names[] = {
#include "css3_names.inc"
};

}  // namespace

std::array<double, 4> python_color_tuple(const Color& color) {
    const Quantity_ColorRGBA q = color.wrapped();
    double r, g, b;
    q.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
    return {pyfmt::round(r, 7), pyfmt::round(g, 7), pyfmt::round(b, 7), pyfmt::round(q.Alpha(), 7)};
}

std::string python_color_str(const Color& color) {
    const Quantity_ColorRGBA q = color.wrapped();
    double rgb[3];
    q.GetRGB().Values(rgb[0], rgb[1], rgb[2], Quantity_TOC_sRGB);
    // webcolors.rgb_to_name([round(c * 255) for c in rgb]): clamped to 0..255
    std::uint32_t key = 0;
    for (double c : rgb) {
        double v = std::nearbyint(c * 255);
        v = v < 0 ? 0 : (v > 255 ? 255 : v);
        key = (key << 8) | static_cast<std::uint32_t>(v);
    }
    std::string name;
    std::string qualifier = "is";
    for (const auto& [value, css_name] : kCss3Names) {
        if (value == key) name = css_name;
    }
    if (name.empty()) {
        // This still uses OCCT X11 colors instead of css3
        name = Quantity_Color::StringName(q.GetRGB().Name());
        qualifier = "near";
    }
    for (auto& c : name) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    const auto t = python_color_tuple(color);
    return "Color: (" + pyfmt::repr(t[0]) + ", " + pyfmt::repr(t[1]) + ", " + pyfmt::repr(t[2]) + ", " +
           pyfmt::repr(t[3]) + ") " + qualifier + " '" + name + "'";
}

}  // namespace build123d::detail
