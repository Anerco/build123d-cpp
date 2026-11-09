// build123d-cpp: import and export. Mirrors exporters3d.py and importers.py.
#pragma once

#include <string>

#include "build123d/topology.hpp"

namespace build123d {

/// STEP with labels and colors (assemblies from Compound::make_assembly keep their tree).
bool export_step(const Shape& to_export, const std::string& file_path, Unit unit = Unit::MM,
                 bool write_pcurves = true, PrecisionMode precision_mode = PrecisionMode::AVERAGE);
bool export_stl(const Shape& to_export, const std::string& file_path, double tolerance = 1e-3,
                double angular_tolerance = 0.1, bool ascii_format = false);
bool export_brep(const Shape& to_export, const std::string& file_path);
/// glTF (.gltf, or .glb when binary) with +Y up, as build123d.
bool export_gltf(const Shape& to_export, const std::string& file_path, Unit unit = Unit::MM,
                 bool binary = false, double linear_deflection = 1e-3,
                 double angular_deflection = 0.1);

Shape import_brep(const std::string& file_path);
/// STEP file as a Compound; named/colored components become children, each placed relative
/// to its parent (as in build123d).
Compound import_step(const std::string& file_path);
/// STL mesh as a single Face.
Face import_stl(const std::string& file_path);

}  // namespace build123d
