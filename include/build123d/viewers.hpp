// Viewers (build123d/jupyter_tools.py and vtk_tools.py) without a VTK dependency: the mesh
// build123d sends to viewers, its VTK XML PolyData text, and the HTML that renders a shape
// with vtk.js (build123d's Jupyter display). Programs that use VTK can include
// build123d/vtk.hpp for to_vtk_poly_data, which returns a vtkPolyData.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

/// The triangle mesh of a shape as build123d's viewers and 3MF export take it
/// (Mesher._mesh_shape): each face's vertices in turn (not shared between faces) and
/// triangles indexing them, oriented like their faces.
struct ShapeMesh {
    std::vector<Vector> vertices;
    std::vector<std::array<std::int64_t, 3>> triangles;
};

/// Mesher._mesh_shape: mesh the shape (BRepMesh_IncrementalMesh with a deflection relative to
/// the edge sizes) and collect the triangulations of its faces (std::runtime_error if a face
/// can't be meshed).
ShapeMesh mesh_shape(const Shape& shape, double linear_deflection, double angular_deflection);

/// The arrays of build123d's to_vtk_poly_data: points as float, triangles, and with `normals`
/// the point and cell normals that vtkPolyDataNormals computes (feature angle 360).
struct VtkMesh {
    std::vector<std::array<float, 3>> points;
    std::vector<std::array<std::int64_t, 3>> polys;
    /// Empty unless normals were requested.
    std::vector<std::array<float, 3>> point_normals;
    std::vector<std::array<float, 3>> cell_normals;
};

/// to_vtk_poly_data without VTK. `tolerance` and `angular_tolerance` default to 1e-3 and 0.1.
VtkMesh vtk_mesh(const Shape& obj, std::optional<double> tolerance = std::nullopt,
                 std::optional<double> angular_tolerance = std::nullopt, bool normals = false);

/// to_vtkpoly_string: the mesh with normals as a VTK XML PolyData document (a .vtp file),
/// readable by VTK and vtk.js.
std::string to_vtkpoly_string(const Shape& shape, double tolerance = 1e-3, double angular_tolerance = 0.1);

/// shape_to_html: an HTML snippet (a div and a script) that renders the shape with vtk.js,
/// as build123d shows shapes in Jupyter.
std::string shape_to_html(const Shape& shape);

}  // namespace build123d
