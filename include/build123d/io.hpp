// build123d-cpp: import and export. Mirrors exporters3d.py and importers.py.
#pragma once

#include <ctime>
#include <iosfwd>
#include <optional>
#include <string>

#include "build123d/topology.hpp"

namespace build123d {

/// STEP with labels and colors (assemblies from Compound::make_assembly keep their tree).
/// `timestamp` goes into the header's FILE_NAME (Python's str or datetime; see isoformat()).
/// Raises std::runtime_error if the file can't be written.
bool export_step(const Shape& to_export, const std::string& file_path, Unit unit = Unit::MM,
                 bool write_pcurves = true, PrecisionMode precision_mode = PrecisionMode::AVERAGE,
                 const std::optional<std::string>& timestamp = std::nullopt);
/// STEP written to a stream (Python's BytesIO).
bool export_step(const Shape& to_export, std::ostream& stream, Unit unit = Unit::MM,
                 bool write_pcurves = true, PrecisionMode precision_mode = PrecisionMode::AVERAGE,
                 const std::optional<std::string>& timestamp = std::nullopt);
/// Python's datetime.isoformat() of a calendar time, for export_step's timestamp:
/// "2024-05-01T12:30:00".
std::string isoformat(const std::tm& time);

/// Raises std::runtime_error if the directory doesn't exist (Python's FileNotFoundError).
bool export_stl(const Shape& to_export, const std::string& file_path, double tolerance = 1e-3,
                double angular_tolerance = 0.1, bool ascii_format = false);
/// BREP file; false if it can't be written, as in Python.
bool export_brep(const Shape& to_export, const std::string& file_path);
/// BREP written to a stream (Python's BytesIO).
bool export_brep(const Shape& to_export, std::ostream& stream);
/// glTF (.gltf, or .glb when binary) with +Y up and TEXCOORD_0 UVs, as build123d. False if
/// the file can't be written.
bool export_gltf(const Shape& to_export, const std::string& file_path, Unit unit = Unit::MM,
                 bool binary = false, double linear_deflection = 1e-3,
                 double angular_deflection = 0.1);
/// Wavefront OBJ. With `include_uvs`, per-vertex texture coordinates (packed into one atlas
/// with `atlas_packing`) and normals are written too. Raises std::runtime_error if the file
/// can't be written.
bool export_obj(const Shape& to_export, const std::string& file_path, double linear_deflection = 0.001,
                double angular_deflection = 0.1, bool include_uvs = true, bool atlas_packing = true,
                double atlas_gutter = 0.0);
/// OBJ written to a stream.
bool export_obj(const Shape& to_export, std::ostream& stream, double linear_deflection = 0.001,
                double angular_deflection = 0.1, bool include_uvs = true, bool atlas_packing = true,
                double atlas_gutter = 0.0);

/// The shape in a BREP file, of its own type: get it with cast_shape<Solid>(...) etc.
/// (cast.hpp). Raises std::invalid_argument if the file can't be read (Python's ValueError).
Shape import_brep(const std::string& file_path);
/// The contents of a STEP file, as in Python: a Compound whose children are the free shapes,
/// or the single free shape itself. Assemblies become Compounds whose children are their
/// components, each placed relative to its parent, with labels (cleaned like Python's: control
/// characters dropped and " .()" replaced by "_") and colors. Use cast_shape<T>() for the
/// specific type. Raises std::runtime_error if the file doesn't exist.
Shape import_step(const std::string& file_path);
/// STL mesh as a single Face; `model_unit` is the unit the file was written in (e.g. Unit::M
/// for Blender), scaled to millimeters.
Face import_stl(const std::string& file_path, Unit model_unit = Unit::MM);

}  // namespace build123d
