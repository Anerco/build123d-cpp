// build123d-cpp: free topology helpers of build123d (topo_explore_connected_edges,
// topo_explore_common_vertex, topo_distance_to, new_edges, delta, polar,
// edges_to_wires). Mirrors build123d/topology/{one_d,zero_d,shape_core,utils}.py.
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

/// Edges of `parent` (default: edge.topo_parent()) that share a vertex with `edge` and
/// meet it with at least the given continuity. A moved copy of a parent edge works too:
/// the result is then in the moved edge's frame. std::invalid_argument without a parent.
ShapeList<Edge> topo_explore_connected_edges(const Edge& edge, const std::optional<Shape>& parent = std::nullopt,
                                             ContinuityLevel continuity = ContinuityLevel::C0);

/// The vertex two edges share, if any.
std::optional<Vertex> topo_explore_common_vertex(const Edge& edge1, const Edge& edge2);

/// A key function for ShapeList::sort_by / group_by: the topological distance of a shape
/// to `other` (0 for the reference shapes, 1 for their neighbours, ...; infinity when
/// unreachable), measured on the shared topo_parent. Adjacency is through shared edges
/// (faces), vertices (edges, wires), faces (shells, solids) or edges (vertices).
std::function<double(const Shape&)> topo_distance_to(const Shape& other);
std::function<double(const Shape&)> topo_distance_to(const std::vector<Shape>& others);
template <class T, std::enable_if_t<std::is_base_of_v<Shape, T> && !std::is_same_v<T, Shape>, int> = 0>
std::function<double(const Shape&)> topo_distance_to(const std::vector<T>& others) {
    return topo_distance_to(std::vector<Shape>(others.begin(), others.end()));
}

/// The edges of `combined` that are not edges of any of `objects` (build123d's
/// new_edges(*objects, combined=...)); each has `combined` as its topo_parent.
ShapeList<Edge> new_edges(const std::vector<Shape>& objects, const Shape& combined);
template <class T, std::enable_if_t<std::is_base_of_v<Shape, T> && !std::is_same_v<T, Shape>, int> = 0>
ShapeList<Edge> new_edges(const std::vector<T>& objects, const Shape& combined) {
    return new_edges(std::vector<Shape>(objects.begin(), objects.end()), combined);
}

/// The shapes of the first list whose OCCT shape (same TShape, location and orientation)
/// is not in the second list, in their order (build123d's delta).
template <class T>
ShapeList<T> delta(const std::vector<T>& shapes_one, const std::vector<T>& shapes_two) {
    ShapeList<T> out;
    for (const auto& s : shapes_one) {
        bool found = false;
        for (const auto& o : shapes_two) found = found || s.is_equal(o);
        if (!found) out.push_back(s);
    }
    return out;
}

/// Polar to cartesian coordinates: (length * cos(angle), length * sin(angle)), angle in
/// degrees (build123d returns the 2-tuple; here Z is 0).
Vector polar(double length, double angle);

/// Connect edges into as few wires as possible (ShapeAnalysis_FreeBounds).
ShapeList<Wire> edges_to_wires(const std::vector<Edge>& edges, double tol = 1e-6);

}  // namespace build123d
