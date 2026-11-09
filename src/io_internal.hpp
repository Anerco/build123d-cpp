// Private helpers of the import/export code.
#pragma once

#include <array>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "build123d/topology.hpp"

namespace build123d::detail {

/// Silence OCCT's console messages below failures, as build123d does before writing files.
void quiet_messages();

/// Python's tuple(color): OCCT's float color as sRGB, each value rounded to 7 digits.
std::array<double, 4> python_color_tuple(const Color& color);
/// Python's str(color): "Color: (1.0, 0.0, 0.0, 1.0) is 'RED'" (a CSS3 name) or
/// "... near 'DARKSLATEGRAY'" (OCCT's nearest named color).
std::string python_color_str(const Color& color);

}  // namespace build123d::detail
