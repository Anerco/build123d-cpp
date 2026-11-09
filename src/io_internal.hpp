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

/// Shape.tessellate_with_uvs (topology/shape_core.py): vertices, triangles, per-vertex
/// normals and UVs, optionally packed into one atlas. (A private copy for the exporters;
/// the public method belongs to the topology classes.)
struct UvMesh {
    std::vector<Vector> vertices;
    std::vector<std::array<int, 3>> triangles;
    std::vector<Vector> normals;
    std::vector<std::pair<double, double>> uvs;
};
UvMesh tessellate_with_uvs(const Shape& shape, double tolerance, double angular_tolerance = 0.1,
                           bool atlas_packing = true, double atlas_gutter = 0.0);

/// pack_utils._pack2d: positions of rectangles (width, length) packed into a square-ish area,
/// in input order.
std::vector<std::pair<double, double>> pack2d(const std::vector<std::pair<double, double>>& sizes);

/// Shape.to_splines: every curve and surface approximated by B-splines.
TopoDS_Shape to_splines(const TopoDS_Shape& shape, int degree = 3, double tolerance = 1e-3, bool nurbs = false);

}  // namespace build123d::detail
