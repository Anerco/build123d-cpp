// Where the textures of threejs_materials' material presets are found at run time (they're
// CC0 data, ~87 MB, not part of the sources: scripts/fetch-textures.sh downloads them).
#pragma once

#include <string>

namespace build123d::detail {

/// The texture directory (threejs_materials' pbr_properties/_assets): $BUILD123D_TEXTURE_DIR,
/// ../share/build123d/textures next to the library, the install prefix's share/build123d/textures,
/// then data/textures in the source tree. Empty if none has the textures.
std::string find_texture_dir();

/// The error raised when a texture is needed and the textures aren't installed.
[[noreturn]] void throw_missing_textures(const std::string& file);

}  // namespace build123d::detail
