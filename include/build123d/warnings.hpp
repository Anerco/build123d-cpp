// Warnings: the C++ counterpart of Python's warnings.warn() calls in build123d.
#pragma once

#include <functional>
#include <string>

namespace build123d {

/// Receives each warning message. The default handler prints "build123d warning: ..." to stderr.
using WarningHandler = std::function<void(const std::string& message)>;

/// Replace the warning handler and return the previous one. Pass an empty function to
/// silence warnings.
WarningHandler set_warning_handler(WarningHandler handler);

/// Issue a warning through the current handler.
void warn(const std::string& message);

}  // namespace build123d
