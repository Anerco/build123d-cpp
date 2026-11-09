// build123d-cpp: shapes and locations to and from bytes. Mirrors persistence.py, which lets
// Python pickle build123d objects.
//
//   Bytes data = to_bytes(part);                    // pickle.dumps(part)
//   Part copy = from_bytes<Part>(data);              // pickle.loads(data)
//   Bytes brep = serialize_shape(part.wrapped());    // the BinTools payload Python writes
#pragma once

#include <cstdint>
#include <vector>

#include <TopLoc_Location.hxx>
#include <TopoDS_Shape.hxx>

#include "build123d/cast.hpp"
#include "build123d/geometry.hpp"
#include "build123d/topology.hpp"

namespace build123d {

/// A byte buffer (Python's bytes).
using Bytes = std::vector<std::uint8_t>;

/// persistence.serialize_shape: OCCT's BinTools binary format with triangulations and without
/// normals, byte for byte what Python writes. A null shape gives empty bytes (Python's None).
Bytes serialize_shape(const TopoDS_Shape& shape);
/// persistence.deserialize_shape; empty bytes give a null shape. Raises std::runtime_error if
/// the data can't be read.
TopoDS_Shape deserialize_shape(const Bytes& buffer);

/// persistence.serialize_location: the translation and the rotation quaternion (X, Y, Z, W) as
/// seven native 32-bit floats (28 bytes, Python's struct.pack("f")). Like Python this keeps
/// single precision only, and drops any scale.
Bytes serialize_location(const TopLoc_Location& location);
/// persistence.deserialize_location; empty bytes give the identity. Raises
/// std::invalid_argument if the buffer is shorter than 28 bytes.
TopLoc_Location deserialize_location(const Bytes& buffer);

/// The equivalent of pickling a build123d shape: its geometry (as serialize_shape), label,
/// color, topo_parent and assembly children (recursively).
Bytes to_bytes(const Shape& shape);
/// The equivalent of pickling a Location (as serialize_location: single precision).
Bytes to_bytes(const Location& location);
/// The shape to_bytes() stored; see cast_shape() and from_bytes<T>() for its specific type.
/// Raises std::invalid_argument if the data isn't a serialized shape.
Shape shape_from_bytes(const Bytes& data);
/// The location to_bytes() stored. Raises std::invalid_argument if the data isn't one.
Location location_from_bytes(const Bytes& data);

/// from_bytes<Location>(data), or from_bytes<Solid>(data), from_bytes<Part>(data), ... for
/// shapes (checked as cast_shape<T>).
template <class T>
T from_bytes(const Bytes& data) {
    if constexpr (std::is_same_v<T, Location>) {
        return location_from_bytes(data);
    } else {
        return cast_shape<T>(shape_from_bytes(data));
    }
}

}  // namespace build123d
