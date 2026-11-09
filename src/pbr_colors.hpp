// The look build123d gives a shape when a catalog material is assigned to it by name: Python's
// Shape.material setter colors the shape with material.pbr.interpolate_color() (bd_materials +
// threejs_materials). For the material system (materials.hpp), which merges with this.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace build123d::detail {

/// The RGBA color of resolve(name).pbr.interpolate_color() for a material grade
/// (RangeMaterial.name, e.g. "Alu_G6061_T6") without a finish or color of its own; nothing for
/// an unknown grade.
std::optional<std::array<double, 4>> catalog_pbr_color(const std::string& grade_name);

/// The grades catalog_pbr_color knows, sorted.
std::vector<std::string> catalog_pbr_grades();

}  // namespace build123d::detail
