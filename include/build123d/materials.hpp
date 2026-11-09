// build123d-cpp: the material of a shape, the part of the bd_materials package that
// build123d uses (Shape.material, mass(), compute_mass()).
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "build123d/geometry.hpp"

namespace build123d {

/// bd_materials' RangeMaterial: a material's physical data as build123d reads it.
struct RangeMaterial {
    /// Catalog (grade) name, e.g. "Alu_G6061_T6".
    std::string name;
    /// Density in kg/m³.
    double density = 0.0;
    /// Material class ("metal", "plastic", ...).
    std::string category = "generic";
    /// PBR / identity key, e.g. "aluminum".
    std::optional<std::string> family;
    /// See-through material (needs a part thickness to render).
    bool transparent = false;

    /// Part mass in grams from a volume in mm³ (m = density * V).
    double mass(double volume_mm3) const { return density * volume_mm3 * 1e-6; }
};

/// bd_materials' FinishedMaterial: a RangeMaterial plus the per-part choices.
/// Shape::material holds one; mass() reads `material.density`.
struct FinishedMaterial {
    RangeMaterial material;
    /// A selected base color, if any.
    std::optional<Color> color;

    FinishedMaterial() = default;
    explicit FinishedMaterial(RangeMaterial m, std::optional<Color> c = std::nullopt)
        : material(std::move(m)), color(std::move(c)) {}
    /// The material's catalog name.
    const std::string& name() const { return material.name; }
};

/// bd_materials.resolve: the FinishedMaterial for a catalog name ("aluminum",
/// "Alu_G7075_T6", ...), matched case-insensitively with spaces and hyphens read as
/// underscores. std::invalid_argument if the name is unknown.
FinishedMaterial resolve_material(const std::string& name);

/// bd_materials.material_names: every name resolve_material() accepts, sorted.
std::vector<std::string> material_names();

}  // namespace build123d
