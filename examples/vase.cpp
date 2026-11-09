// Vase: a profile of arcs and splines, revolved, hollowed and filleted.
// Port of build123d's examples/vase_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const Line l1({0, 0}, {12, 0});
    const RadiusArc l2(l1.position_at(1), {15, 20}, 50);
    const Spline l3({l2.position_at(1), {22, 40}, {20, 50}}, {l2 % 1, {-0.75, 1}});
    const RadiusArc l4(l3.position_at(1), l3.position_at(1) + Vector(0, 5), 5);
    const Spline l5({l4.position_at(1), l4.position_at(1) + Vector(2.5, 2.5), l4.position_at(1) + Vector(0, 5)},
                    {l4 % 1, {-1, 0}});
    Curve outline = l1 + l2 + l3 + l4 + l5;
    outline += Polyline({l5.position_at(1), l5.position_at(1) + Vector(0, 1), {0, l5.position_at(1).Y + 1},
                         l1.position_at(0)});
    const Sketch profile = make_face(outline.edges());
    Part vase = revolve(profile, Axis::Y);
    vase = offset(vase, -1, {.openings = {vase.faces().sort_by(Axis::Y).last()}});
    const auto top_edges = vase.edges().filter_by(GeomType::CIRCLE).filter_by_position(Axis::Y, 60, 62);
    vase = fillet(top_edges, 0.25);
    vase = fillet(vase.edges().sort_by(Axis::Y).first(), 0.5);

    example::show(vase, "vase");
    export_step(vase, "vase.step");
    return 0;
}
