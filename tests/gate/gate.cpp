#include "gate.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace gate {

void enable_from_environment() {
    if (const char* dir = std::getenv("BUILD123D_OPCACHE_DIR")) {
        build123d::enable_operation_cache({.folder = dir, .min_ms = 0});
    }
}

std::string brep_hash(const build123d::Shape& shape) {
    if (!std::getenv("BUILD123D_SHOW_BREP")) return "";
    std::uint64_t h = 1469598103934665603ull;
    for (const std::uint8_t b : build123d::serialize_shape(shape.wrapped())) h = (h ^ b) * 1099511628211ull;
    char buf[17];
    std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

namespace {
const bool enabled = (enable_from_environment(), true);
}

}  // namespace gate
