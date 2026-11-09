// build123d-cpp: 3MF and STL meshes through the 3MF Consortium's lib3mf. Mirrors mesher.py.
//
//   Mesher exporter;
//   exporter.add_shape(part, 0.001, 0.1, MeshType::MODEL, "P-100");
//   exporter.add_meta_data("build123d", "note", "hello", "string", false);
//   exporter.write("part.3mf");                       // or "part.stl"
//
//   Mesher importer;
//   ShapeList<Shape> shapes = importer.read("part.3mf");  // Solids, or Shells if not closed
#pragma once

#include <iosfwd>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "build123d/enums.hpp"
#include "build123d/topology.hpp"

namespace build123d {

/// One metadata entry of a 3MF model (Mesher::get_meta_data).
struct MeshMetaData {
    std::string name_space;
    std::string name;
    std::string type;
    std::string value;
};

/// The type and value of a metadata entry (Mesher::get_meta_data_by_key).
struct MeshMetaDataValue {
    std::string type;
    std::string value;
};

/// The properties of a mesh (Mesher::get_mesh_properties).
struct MeshProperties {
    std::string name;
    std::string part_number;
    MeshType type = MeshType::MODEL;
    std::string uuid;
};

/// Exports and imports meshed objects stored in 3MF or STL files (mesher.py Mesher).
/// Errors reported by lib3mf are raised as std::runtime_error.
class Mesher {
public:
    /// The model's unit (MC, MM, CM, M, IN or FT; std::invalid_argument otherwise).
    explicit Mesher(Unit unit = Unit::MM);
    ~Mesher();
    Mesher(Mesher&&) noexcept;
    Mesher& operator=(Mesher&&) noexcept;
    Mesher(const Mesher&) = delete;
    Mesher& operator=(const Mesher&) = delete;

    /// Unit used in the model (the file's unit after read()).
    Unit model_unit() const;
    /// Number of triangles in each of the model's meshes.
    std::vector<int> triangle_counts() const;
    /// Number of vertices in each of the model's meshes.
    std::vector<int> vertex_counts() const;
    /// Number of meshes in the model (including the empty ones of skipped degenerate shapes).
    int mesh_count() const;
    /// lib3mf's version, "major.minor.micro".
    std::string library_version() const;

    /// Add metadata to the model.
    void add_meta_data(const std::string& name_space, const std::string& name, const std::string& value,
                       const std::string& metadata_type, bool must_preserve);
    /// Add the source file of the calling code to the metadata, with the name space
    /// "build123d", the file's base name as name and the type "cpp" (Python: "python").
    /// `source_file` defaults to the caller's file.
    void add_code_to_metadata(const std::string& source_file = __builtin_FILE());
    /// All the metadata. Raises std::runtime_error if the model has none.
    std::vector<MeshMetaData> get_meta_data() const;
    /// The type and value of a metadata entry; std::runtime_error if there is no such entry.
    MeshMetaDataValue get_meta_data_by_key(const std::string& name_space, const std::string& name) const;
    /// The properties of all the meshes.
    std::vector<MeshProperties> get_mesh_properties() const;

    /// Add a shape: a Compound with children adds each child (with the compound's color when
    /// a child has none), a Compound with a label or color itself, any other Compound each of
    /// its direct sub-shapes, other shapes themselves. `uuid_value` is a UUID string. Raises
    /// std::runtime_error if a mesh is invalid (e.g. not closed); warns about degenerate shapes
    /// (skipped) and meshes that aren't manifold.
    void add_shape(const Shape& shape, double linear_deflection = 0.001, double angular_deflection = 0.1,
                   MeshType mesh_type = MeshType::MODEL, const std::optional<std::string>& part_number = std::nullopt,
                   const std::optional<std::string>& uuid_value = std::nullopt);
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    void add_shape(const std::vector<S>& shapes, double linear_deflection = 0.001, double angular_deflection = 0.1,
                   MeshType mesh_type = MeshType::MODEL, const std::optional<std::string>& part_number = std::nullopt,
                   const std::optional<std::string>& uuid_value = std::nullopt) {
        std::vector<Shape> list(shapes.begin(), shapes.end());
        add_shapes(list, linear_deflection, angular_deflection, mesh_type, part_number, uuid_value);
    }

    /// The shapes in a .3mf or .stl file, with their labels and colors (std::invalid_argument
    /// for another extension).
    ShapeList<Shape> read(const std::string& file_name);
    /// Write a .3mf or .stl file (std::invalid_argument for another extension).
    void write(const std::string& file_name) const;
    /// Write "3mf" or "stl" data to a stream (std::invalid_argument for another type).
    void write_stream(std::ostream& stream, const std::string& file_type) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void add_shapes(const std::vector<Shape>& shapes, double linear_deflection, double angular_deflection,
                    MeshType mesh_type, const std::optional<std::string>& part_number,
                    const std::optional<std::string>& uuid_value);
};

}  // namespace build123d
