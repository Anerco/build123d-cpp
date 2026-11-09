// Clock face with minute and hour marks and numbers.
// Port of build123d's examples/clock_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include <string>

#include "show.hpp"

using namespace build123d;

int main() {
    const double clock_radius = 10;

    const CenterArc l1({0, 0}, clock_radius * 0.975, 0.75, 4.5);
    const CenterArc l2({0, 0}, clock_radius * 0.925, 0.75, 4.5);
    const Line l3(l1.position_at(0), l2.position_at(0));
    const Line l4(l1.position_at(1), l2.position_at(1));
    Sketch minute_indicator = make_face(std::vector<Edge>{l1, l3, l2, l4});
    minute_indicator = fillet(minute_indicator.vertices(), clock_radius * 0.01);

    Sketch clock_face = Circle(clock_radius);
    clock_face -= PolarLocations(0, 60) * minute_indicator;
    clock_face -= PolarLocations(clock_radius * 0.875, 12) * SlotOverall(clock_radius * 0.05, clock_radius * 0.025);

    std::vector<Sketch> numbers;
    int hour = 0;
    for (const auto& loc : PolarLocations(clock_radius * 0.75, 12, 60, -360, false)) {
        numbers.push_back(loc * Text(std::to_string(++hour), clock_radius * 0.175,
                                     {.font_style = FontStyle::BOLD, .align = Align::CENTER}));
    }
    clock_face -= numbers;

    example::show(clock_face, "clock_face");
    export_step(clock_face, "clock_face.step");
    return 0;
}
