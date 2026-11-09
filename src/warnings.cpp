#include "build123d/warnings.hpp"

#include <iostream>
#include <mutex>
#include <utility>

namespace build123d {

namespace {

std::mutex& handler_mutex() {
    static std::mutex mutex;
    return mutex;
}

WarningHandler& current_handler() {
    static WarningHandler handler = [](const std::string& message) {
        std::cerr << "build123d warning: " << message << std::endl;
    };
    return handler;
}

}  // namespace

WarningHandler set_warning_handler(WarningHandler handler) {
    const std::lock_guard<std::mutex> lock(handler_mutex());
    std::swap(current_handler(), handler);
    return handler;
}

void warn(const std::string& message) {
    WarningHandler handler;
    {
        const std::lock_guard<std::mutex> lock(handler_mutex());
        handler = current_handler();
    }
    if (handler) handler(message);
}

}  // namespace build123d
