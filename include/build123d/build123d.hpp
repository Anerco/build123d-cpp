// build123d-cpp: C++ port of the build123d algebra API (https://github.com/gumyr/build123d)
// on top of OpenCASCADE.
//
//   #include <build123d/build123d.hpp>
//   using namespace build123d;
//
//   Part part = Box(10, 10, 5) - Cylinder(2, 5);
//   part = fillet(part.edges().filter_by(Axis::Z), 1);
//   export_step(part, "part.step");
#pragma once

#include "build123d/enums.hpp"
#include "build123d/geometry.hpp"
#include "build123d/history.hpp"
#include "build123d/io.hpp"
#include "build123d/joints.hpp"
#include "build123d/locations.hpp"
#include "build123d/materials.hpp"
#include "build123d/objects.hpp"
#include "build123d/operations.hpp"
#include "build123d/topology.hpp"
#include "build123d/topology_utils.hpp"
#include "build123d/warnings.hpp"

#define BUILD123D_CPP_VERSION "0.1.0"
