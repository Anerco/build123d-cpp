// Helpers shared by the fastener sources (fastener.cpp, fastener_screws.cpp) and the modules
// built on them.
#pragma once

#include <set>
#include <string>
#include <vector>

#include "bd_warehouse/fastener.hpp"
#include "builder.hpp"

namespace bd_warehouse::detail {

/// `with BuildSketch(plane) as profile: ...` then `profile.sketch`: the builder's sketch at `plane`.
build123d::Sketch placed_sketch(const Builder& sketch, const build123d::Plane& plane);

/// Face.make_surface of a list of edges, added in the given order (the C++ library takes a Wire).
build123d::Face make_surface_from_edges(const std::vector<build123d::Edge>& edges);

/// A fastener class's types(): the type part of the first row's column names.
std::set<std::string> types_of(const StringTable& fastener_data);
/// A fastener class's sizes(fastener_type).
std::vector<std::string> sizes_of(const std::string& fastener_type, const StringTable& fastener_data);
/// Python's repr of a set of str (sorted here; Python's order is arbitrary).
std::string py_set_repr(const std::set<std::string>& items);
/// Python's list(diameters.keys()) repr.
std::string fits_repr(const ParamDict& diameters);

}  // namespace bd_warehouse::detail

/// fastener_data(), types() and sizes() of a fastener class reading `file`.
#define BD_WAREHOUSE_TABLE_FUNCTIONS(Class, file)                                    \
    const StringTable& Class::fastener_data() {                                      \
        static const StringTable table = read_fastener_parameters_from_csv(file);   \
        return table;                                                                \
    }                                                                                \
    std::set<std::string> Class::types() { return detail::types_of(fastener_data()); } \
    std::vector<std::string> Class::sizes(const std::string& fastener_type) {        \
        return detail::sizes_of(fastener_type, fastener_data());                     \
    }
