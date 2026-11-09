// Mass properties: Shape.compute_volume, compute_mass, matrix_of_inertia,
// principal_properties, static_moments, radius_of_gyration, combined_center
// (shape_core.py) and the per-class `mass` (zero_d, one_d, two_d, three_d, composite).
#include <cmath>

#include <BRepGProp.hxx>
#include <GProp_PrincipalProps.hxx>
#include <gp_Mat.hxx>

#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

GProp_GProps volume_properties(const Shape& s, const char* what) {
    if (s.is_null()) throw std::invalid_argument(std::string("Can't calculate ") + what + " for empty shape");
    GProp_GProps properties;
    BRepGProp::VolumeProperties(s.wrapped(), properties);
    return properties;
}

/// Shape.compute_mass with the material the shape has (or inherits).
double compute_mass_with(const Shape& s, const std::optional<FinishedMaterial>& material, Unit mass_unit,
                         Unit length_unit) {
    if (!s) return 0.0;
    // volume(), not compute_volume(): 0 for vertices/edges/wires/faces and the enclosed
    // volume of a manifold shell
    const double volume = s.volume();
    if (volume == 0) return 0.0;
    if (!material) throw std::invalid_argument("Shape's density is missing");
    const double density_kg_m3 = material->material.density;
    if (density_kg_m3 == 0) {
        warn("Shape's density is 0");
        return 0.0;
    }
    const double density =
        density_kg_m3 * units_per_kilogram(mass_unit) / std::pow(units_per_meter(length_unit), 3);
    return volume * density;
}

/// The per-class mass; `inherited` stands for build123d's lookup of the material up the
/// assembly tree.
double mass_of(const Shape& s, const std::optional<FinishedMaterial>& inherited, Unit mass_unit, Unit length_unit) {
    if (s.is_null()) return 0.0;
    const std::optional<FinishedMaterial>& material = s.material ? s.material : inherited;
    switch (s.wrapped().ShapeType()) {
        case TopAbs_SHELL:
        case TopAbs_SOLID: return compute_mass_with(s, material, mass_unit, length_unit);
        case TopAbs_COMPOUND:
        case TopAbs_COMPSOLID: {
            double total = 0.0;
            if (!s.children.empty()) {
                for (const auto& child : s.children) total += mass_of(child, material, mass_unit, length_unit);
                return total;
            }
            // the sub-shapes are not build123d objects: they take this compound's material
            const Compound compound(s.wrapped());
            for (const auto& solid : compound.get_type<Solid>()) total += compute_mass_with(solid, material, mass_unit, length_unit);
            for (const auto& shell : compound.get_type<Shell>()) total += compute_mass_with(shell, material, mass_unit, length_unit);
            return total;
        }
        default: return 0.0;  // a vertex, edge, wire or face has no mass
    }
}

}  // namespace

double Shape::compute_volume() const {
    if (!*this || obj_.ShapeType() == TopAbs_VERTEX) return 0.0;
    GProp_GProps properties;
    switch (obj_.ShapeType()) {
        case TopAbs_EDGE:
        case TopAbs_WIRE: BRepGProp::LinearProperties(obj_, properties); break;
        case TopAbs_FACE:
        case TopAbs_SHELL: BRepGProp::SurfaceProperties(obj_, properties); break;
        default: BRepGProp::VolumeProperties(obj_, properties); break;
    }
    return properties.Mass();
}

double Shape::compute_mass(Unit mass_unit, Unit length_unit) const {
    return compute_mass_with(*this, material, mass_unit, length_unit);
}

double Shape::mass(Unit mass_unit, Unit length_unit) const { return mass_of(*this, std::nullopt, mass_unit, length_unit); }

std::array<std::array<double, 3>, 3> Shape::matrix_of_inertia() const {
    if (is_null()) throw std::invalid_argument("Can't calculate matrix for empty shape");
    const gp_Mat m = volume_properties(*this, "matrix").MatrixOfInertia();
    std::array<std::array<double, 3>, 3> out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) out[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = m.Value(i + 1, j + 1);
    }
    return out;
}

std::vector<std::pair<Vector, double>> Shape::principal_properties() const {
    if (is_null()) throw std::invalid_argument("Can't calculate properties for empty shape");
    const GProp_PrincipalProps props = volume_properties(*this, "properties").PrincipalProperties();
    double i1, i2, i3;
    props.Moments(i1, i2, i3);
    return {{Vector(props.FirstAxisOfInertia()), i1},
            {Vector(props.SecondAxisOfInertia()), i2},
            {Vector(props.ThirdAxisOfInertia()), i3}};
}

std::array<double, 3> Shape::static_moments() const {
    if (is_null()) throw std::invalid_argument("Can't calculate moments for empty shape");
    double mx, my, mz;
    volume_properties(*this, "moments").StaticMoments(mx, my, mz);
    return {mx, my, mz};
}

double Shape::radius_of_gyration(const Axis& axis) const {
    if (is_null()) throw std::invalid_argument("Can't calculate radius of gyration for empty shape");
    return volume_properties(*this, "radius of gyration").RadiusOfGyration(axis.wrapped());
}

Vector Shape::combined_center(const std::vector<Shape>& objects, CenterOf center_of) {
    if (center_of == CenterOf::GEOMETRY) throw std::invalid_argument("CenterOf.GEOMETRY not implemented");
    if (objects.empty()) throw std::invalid_argument("No objects to find the center of");
    if (center_of == CenterOf::MASS) {
        double total_mass = 0.0;
        for (const auto& o : objects) total_mass += o.compute_volume();
        Vector sum_wc = objects[0].center(CenterOf::MASS).multiply(objects[0].compute_volume());
        for (std::size_t i = 1; i < objects.size(); ++i) {
            sum_wc = sum_wc.add(objects[i].center(CenterOf::MASS).multiply(objects[i].compute_volume()));
        }
        return sum_wc.multiply(1.0 / total_mass);
    }
    const double total = static_cast<double>(objects.size());
    Vector sum_wc = objects[0].bounding_box().center();
    for (std::size_t i = 1; i < objects.size(); ++i) sum_wc = sum_wc.add(objects[i].bounding_box().center());
    return sum_wc.multiply(1.0 / total);
}

}  // namespace build123d
