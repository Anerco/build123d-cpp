// What the example programs pass to show(), kept instead of printed.
#pragma once

#include <build123d/build123d.hpp>

#include <string>
#include <utility>
#include <vector>

namespace example {

inline std::vector<std::pair<std::string, build123d::Shape>>& shown() {
    static std::vector<std::pair<std::string, build123d::Shape>> shapes;
    return shapes;
}

inline void show(const build123d::Shape& shape, const std::string& name) { shown().emplace_back(name, shape); }

}  // namespace example
