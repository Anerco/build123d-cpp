// build123d-cpp: analytic primitives from a triangle mesh. Mirrors brep_from_stl.py.
//
//   Mesher importer;
//   const Shape mesh = importer.read("part.stl")[0];
//   auto [primitives, leftovers, code] = detect_primitives(mesh);
#pragma once

#include <string>
#include <tuple>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

/// Detect planes, cylinders and spheres in a triangle mesh (a Shape whose faces are the
/// triangles, e.g. read by Mesher) and return (brep_from_stl.py detect_primitives):
///   - the primitive faces: a Face for each detected plane, cylinder or sphere;
///   - the leftovers: the mesh faces no primitive explains;
///   - Python build123d code that rebuilds each primitive face ("r0 = Plane.XY * Rectangle(...)"),
///     in the same order as the primitive faces.
/// Detection runs as in Python: planes from the cleaned mesh first, then spheres, cylinders and
/// the remaining planar regions, using the same clustering (scikit-learn's DBSCAN) and fits.
/// Raises std::invalid_argument for an empty shape, std::out_of_range for faces with closed
/// (one-vertex) edges, which a triangle mesh doesn't have.
std::tuple<ShapeList<Face>, ShapeList<Face>, std::vector<std::string>> detect_primitives(const Shape& mesh);

}  // namespace build123d
