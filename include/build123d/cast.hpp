// build123d-cpp: the specific type of an untyped Shape (Python's Shape.cast), for results such
// as import_brep(), import_step(), deserialize_shape() and Mesher::read().
//
//   Shape s = import_brep("part.brep");
//   Solid solid = cast_shape<Solid>(s);             // std::invalid_argument if it isn't one
//   Part part = cast_shape<Part>(s);                // a Solid or a Compound of solids
//   std::visit([](const auto& typed) { ... }, cast_shape(s));
#pragma once

#include <type_traits>
#include <variant>

#include "build123d/topology.hpp"

namespace build123d {

/// Every specific topological type, as Python's Shape.cast returns them.
using AnyShape = std::variant<Vertex, Edge, Wire, Face, Shell, Solid, Compound>;

/// The shape as its specific type, keeping its label, color and children. A null shape gives
/// an empty Compound; a CompSolid (build123d has no class for it) a Compound.
AnyShape cast_shape(const Shape& shape);

namespace detail {

enum class CastTarget { VERTEX, EDGE, WIRE, FACE, SHELL, SOLID, COMPOUND, MIXIN1D, MIXIN2D, MIXIN3D, CURVE, SKETCH, PART };

/// Whether `shape` can be viewed as the target type; throws std::invalid_argument if not.
void check_cast(const Shape& shape, CastTarget target);

template <class T>
constexpr CastTarget cast_target() {
    if constexpr (std::is_same_v<T, Vertex>) return CastTarget::VERTEX;
    else if constexpr (std::is_same_v<T, Edge>) return CastTarget::EDGE;
    else if constexpr (std::is_same_v<T, Wire>) return CastTarget::WIRE;
    else if constexpr (std::is_same_v<T, Face>) return CastTarget::FACE;
    else if constexpr (std::is_same_v<T, Shell>) return CastTarget::SHELL;
    else if constexpr (std::is_same_v<T, Solid>) return CastTarget::SOLID;
    else if constexpr (std::is_same_v<T, Mixin1D>) return CastTarget::MIXIN1D;
    else if constexpr (std::is_same_v<T, Mixin2D>) return CastTarget::MIXIN2D;
    else if constexpr (std::is_same_v<T, Mixin3D>) return CastTarget::MIXIN3D;
    else if constexpr (std::is_base_of_v<Curve, T>) return CastTarget::CURVE;
    else if constexpr (std::is_base_of_v<Sketch, T>) return CastTarget::SKETCH;
    else if constexpr (std::is_base_of_v<Part, T>) return CastTarget::PART;
    else if constexpr (std::is_base_of_v<Compound, T>) return CastTarget::COMPOUND;
    else if constexpr (std::is_base_of_v<Solid, T>) return CastTarget::SOLID;
    else if constexpr (std::is_base_of_v<Face, T>) return CastTarget::FACE;
    else if constexpr (std::is_base_of_v<Wire, T>) return CastTarget::WIRE;
    else if constexpr (std::is_base_of_v<Edge, T>) return CastTarget::EDGE;
    else if constexpr (std::is_base_of_v<Shell, T>) return CastTarget::SHELL;
    else return CastTarget::VERTEX;
}

}  // namespace detail

/// The shape as type T, keeping its label, color and children:
/// - Vertex, Edge, Wire, Face, Shell, Solid, Compound: the shape must be of that type;
/// - Mixin1D (an Edge or Wire), Mixin2D (a Face or Shell), Mixin3D (a Solid or Compound);
/// - Curve, Sketch, Part: a Compound of objects of that dimension, or a single such object
///   (an Edge or Wire, a Face or Shell, a Solid), which is wrapped in a Compound;
/// - Shape: unchanged.
/// Raises std::invalid_argument otherwise.
template <class T>
T cast_shape(const Shape& shape) {
    static_assert(std::is_base_of_v<Shape, T>, "cast_shape needs a shape type");
    if constexpr (std::is_same_v<T, Shape>) {
        return shape;
    } else {
        constexpr detail::CastTarget target = detail::cast_target<T>();
        detail::check_cast(shape, target);
        T result(shape.wrapped());
        if constexpr (target == detail::CastTarget::CURVE || target == detail::CastTarget::SKETCH ||
                      target == detail::CastTarget::PART) {
            // A single object has been wrapped into a compound: keep that geometry
            result.label = shape.label;
            result.color = shape.color;
            result.children = shape.children;
            result.set_topo_parent(shape.topo_parent());
        } else {
            static_cast<Shape&>(result) = shape;
        }
        return result;
    }
}

}  // namespace build123d
