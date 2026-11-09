// build123d-cpp: version information (build123d's _version module).
#pragma once

#include <array>

namespace build123d {

/// The build123d release this library matches: build123d.__version__ and version_tuple.
inline constexpr const char* version = "0.13.0";
inline constexpr std::array<int, 3> version_tuple = {0, 13, 0};

/// The version of this library itself (the CMake project version).
inline constexpr const char* cpp_version = "0.1.0";

}  // namespace build123d
