// Handle: sections placed along a spline and swept through it.
// Port of build123d's examples/handle_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const int segment_count = 6;

    // Create a path for the sweep along the handle
    const Spline handle_center_line({{-10, 0, 0}, {0, 0, 5}, {10, 0, 0}}, {{0, 0, 1}, {0, 0, -1}}, {1.5, 1.5});

    // Create the cross sections
    Sketch sections;
    for (int i = 0; i <= segment_count; ++i) {
        const Location location = handle_center_line ^ (static_cast<double>(i) / segment_count);
        Sketch circle;
        if (i % segment_count == 0) {
            circle = location * Circle(1);
        } else {
            circle = location * Rectangle(1.25, 3);
            circle = fillet(circle.vertices(), 0.2);
        }
        sections += circle;
    }

    // Create the handle by sweeping along the path
    const Part handle = sweep(sections, handle_center_line, {.multisection = true});

    example::show(handle_center_line, "handle_path");
    example::show(handle, "handle");
    export_step(handle, "handle.step");
    return 0;
}
