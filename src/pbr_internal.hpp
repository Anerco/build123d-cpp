// Helpers of the threejs_materials port shared by the material library and the glTF
// injection (utils.py).
#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "build123d/pbr.hpp"
#include "pil_image.hpp"

namespace build123d::detail::pbr {

/// PIL.ImageColor.getrgb: 3 or 4 integer channels (std::invalid_argument for an unknown
/// color, as Python's ValueError).
std::vector<int> getrgb(const std::string& color);
/// utils._normalize_srgb_color: sRGB ratios and the alpha, if any.
std::pair<std::array<double, 3>, std::optional<double>> normalize_srgb_color(const ColorSpec& color);
/// utils._normalize_color: linear ratios (for strings) and the alpha, if any.
std::pair<std::array<double, 3>, std::optional<double>> normalize_color(const ColorSpec& color);
double srgb_to_linear(double c);
double linear_to_srgb(double c);

bool is_data_uri(const std::string& s);
/// The bytes of a texture: a data URI, or a file in the maps directory (missing preset
/// textures raise the "fetch the textures" error).
pil::Bytes texture_bytes(const std::string& ref, const std::optional<std::string>& texture_dir);
/// utils._open_texture_image: palette images become L.
pil::Image open_texture_image(const std::string& ref, const std::optional<std::string>& texture_dir);
/// utils._resolve_to_data_uri
std::string resolve_to_data_uri(const std::string& ref, const std::string& texture_dir);

/// PbrProperties.scale(u, v, fixed, rotation) with u, v and rotation as the Python values the
/// caller passes (ints or floats: they're hashed into the id as json.dumps writes them).
PbrProperties scale(const PbrProperties& self, const Json& u, const Json& v, bool fixed, const Json& rotation);

/// Python's repr of a str, a float and a list of floats.
std::string repr_str(const std::string& s);
std::string repr_floats(const std::vector<double>& values, bool tuple);

}  // namespace build123d::detail::pbr
