// Hole, CounterBoreHole and CounterSinkHole cut into cylinders.
// Port of build123d's examples/holes_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    // Circle with a hole
    Part thru_hole = Cylinder(3, 2);
    thru_hole -= Hole(1, 2);

    // Recessed counter bore
    Part recessed_counter_bore = Cylinder(3, 2);
    recessed_counter_bore -= CounterBoreHole(1, 1.5, 0.5, 2);

    // Recessed counter sink
    Part recessed_counter_sink = Cylinder(3, 2);
    recessed_counter_sink -= CounterSinkHole(1, 1.5, 2);

    // Flush counter sink
    Part flush_counter_sink = Cylinder(3, 2);
    const Plane plane(flush_counter_sink.faces().sort_by().last());
    flush_counter_sink -= plane * CounterSinkHole(1, 1.5, 2);

    example::show(thru_hole, "thru_hole");
    example::show(recessed_counter_bore, "recessed_counter_bore");
    example::show(recessed_counter_sink, "recessed_counter_sink");
    example::show(flush_counter_sink, "flush_counter_sink");
    export_step(Compound::make_assembly({Pos(-7.5, 7.5) * thru_hole, Pos(7.5, 7.5) * recessed_counter_bore,
                                         Pos(-7.5, -7.5) * recessed_counter_sink,
                                         Pos(7.5, -7.5) * flush_counter_sink}),
                "holes.step");
    return 0;
}
