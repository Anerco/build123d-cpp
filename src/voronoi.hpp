// The vertices of a 2D Voronoi diagram, as scipy.spatial.Voronoi(points).vertices computes
// them with Qhull ("Qbb Qc Qz"), for full_round.
#pragma once

#include <array>
#include <vector>

namespace build123d::detail {

using Point2 = std::array<double, 2>;

/// The Voronoi vertices of the points: the circumcenters of the Delaunay triangulation, one
/// per set of cocircular points (Qhull merges such facets). Duplicate points are ignored, as
/// Qhull's "Qc" keeps them as coplanar points. Throws std::runtime_error when the points don't
/// span the plane, like Qhull's "initial simplex is flat".
std::vector<Point2> voronoi_vertices(const std::vector<Point2>& points);

/// Exact orientation (sign of (b - a) x (c - a)) and in-circle (d inside the circle through
/// counter-clockwise a, b, c is positive) predicates.
int orient2d_sign(const Point2& a, const Point2& b, const Point2& c);
int incircle_sign(const Point2& a, const Point2& b, const Point2& c, const Point2& d);

}  // namespace build123d::detail
