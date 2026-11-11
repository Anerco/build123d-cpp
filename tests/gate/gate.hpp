// The operation cache's gate (scripts/opcache_gate.py), linked into the unit tests, the
// comparison cases and the examples: inert unless the gate's variables are set.
#pragma once

#include <string>

#include <build123d/build123d.hpp>

namespace gate {

/// The cache on, keeping every operation, when BUILD123D_OPCACHE_DIR names its folder;
/// done before main() runs, and again by tests that turn the cache off.
void enable_from_environment();

/// A hash of the shape's bytes (serialize_shape), for runs compared byte for byte, when
/// BUILD123D_SHOW_BREP is set; empty otherwise.
std::string brep_hash(const build123d::Shape& shape);

}  // namespace gate
