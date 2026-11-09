// Upload a part to PCBWay's 3D printing service (build123d/exporters3d.py export_to_pcbway).
#pragma once

#include <functional>
#include <string>

#include "build123d/enums.hpp"
#include "build123d/topology.hpp"

namespace build123d {

/// Where export_to_pcbway uploads to and how it opens the result. The defaults are PCBWay's
/// service and the system browser; change them only for testing.
struct PcbwayUpload {
    /// PCBWay's upload endpoint for build123d.
    std::string upload_url = "https://www.pcbway.com/common/Build123dUpFile";
    /// Opens the pricing page and returns false if it couldn't. Empty: open_in_browser().
    std::function<bool(const std::string& url)> open_browser = {};
    /// Seconds to wait for the connection, and for data from the server (requests' (10, 120)).
    long connect_timeout = 10;
    long read_timeout = 120;
};

/// export_to_pcbway: upload the shape as a STEP file to PCBWay, open the pricing page PCBWay
/// returns in the browser (warning if that fails) and return the page's URL. Throws
/// std::runtime_error if the upload fails or PCBWay's answer is not a successful one.
/// Needs build123d built with libcurl.
std::string export_to_pcbway(const Shape& to_export, Unit unit = Unit::MM, bool write_pcurves = true,
                             PrecisionMode precision_mode = PrecisionMode::AVERAGE);
std::string export_to_pcbway(const Shape& to_export, const PcbwayUpload& upload, Unit unit = Unit::MM,
                             bool write_pcurves = true, PrecisionMode precision_mode = PrecisionMode::AVERAGE);

/// Open an http(s) URL in the system's browser (Python's webbrowser.open(url, new=2)):
/// xdg-open on Linux, open on macOS, the shell on Windows. False if no browser was started.
bool open_in_browser(const std::string& url);

}  // namespace build123d
