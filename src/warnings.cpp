#include "build123d/warnings.hpp"

#include <iostream>
#include <mutex>
#include <utility>
#include <vector>

#include "opcache.hpp"

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

/// The warning captures open on this thread, innermost last (detail::opcache::WarningCapture).
thread_local std::vector<std::vector<std::string>*> captures;

}  // namespace

WarningHandler set_warning_handler(WarningHandler handler) {
    const std::lock_guard<std::mutex> lock(handler_mutex());
    std::swap(current_handler(), handler);
    return handler;
}

void warn(const std::string& message) {
    for (auto* capture : captures) capture->push_back(message);
    WarningHandler handler;
    {
        const std::lock_guard<std::mutex> lock(handler_mutex());
        handler = current_handler();
    }
    if (handler) handler(message);
}

namespace detail::opcache {

WarningCapture::WarningCapture() { captures.push_back(&messages); }

WarningCapture::~WarningCapture() {
    for (auto it = captures.end(); it != captures.begin();) {
        --it;
        if (*it == &messages) {
            captures.erase(it);
            break;
        }
    }
}

}  // namespace detail::opcache

}  // namespace build123d
