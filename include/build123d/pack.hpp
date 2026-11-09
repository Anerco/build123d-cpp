// pack: arrange shapes in a compact, non-overlapping 2D layout (pack.py, pack_utils.py).
#pragma once

#include <type_traits>
#include <utility>
#include <vector>

#include "build123d/geometry.hpp"
#include "build123d/topology.hpp"

namespace build123d {

namespace detail {

/// _pack2d (pack_utils.py): positions (x, y) of rectangles given as (width, length), packed into
/// a square-ish area by the growing binary-tree packer, in the input order.
std::vector<std::pair<double, double>> pack2d(const std::vector<std::pair<double, double>>& sizes);

/// The locations pack() moves the objects by.
std::vector<Location> pack_locations(const std::vector<Shape>& objects, double padding, bool align_z);

/// pack()'s final check that no two packed bounding boxes overlap (std::runtime_error).
void check_pack_overlap(const std::vector<Shape>& packed);

}  // namespace detail

/// Pack objects in a squarish area in Plane.XY: each object is moved so that the bounding boxes,
/// grown by `padding`, don't overlap. `align_z` also aligns the shape bottoms to Plane.XY.
/// Returns the moved objects in the input order.
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
ShapeList<typename T::topo_type> pack(const std::vector<T>& objects, double padding, bool align_z = false) {
    const std::vector<Location> locations =
        detail::pack_locations(std::vector<Shape>(objects.begin(), objects.end()), padding, align_z);
    ShapeList<typename T::topo_type> translated;
    translated.reserve(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) translated.push_back(objects[i].moved(locations[i]));
    detail::check_pack_overlap(std::vector<Shape>(translated.begin(), translated.end()));
    return translated;
}

}  // namespace build123d
