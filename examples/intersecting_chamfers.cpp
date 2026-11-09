// Chamfers whose edges meet at shared vertices.
// Port of build123d's examples/intersecting_chamfers_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const Align3 align(Align::CENTER, Align::MIN, Align::MIN);
    Part blocks = Pos(-1, -1, 0) * Box(1, 2, 1, align);
    blocks += Box(1, 1, 2, align);
    blocks += Pos(1, -1, 0) * Box(1, 2, 1, align);

    const auto bottom_edges = blocks.edges().filter_by_position(Axis::Z, 0, 1, {true, false});
    Part blocks2 = chamfer(bottom_edges, 0.1);

    const auto top_edges = blocks2.edges().filter_by_position(Axis::Z, 1, 2, {false, true});
    blocks2 = chamfer(top_edges, 0.1);

    example::show(blocks2, "intersecting_chamfers");
    export_step(blocks2, "intersecting_chamfers.step");
    return 0;
}
