// Circuit board: a rectangle with rows of pin holes and mounting holes, extruded.
// Port of build123d's examples/circuit_board_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const double pcb_length = 70 * MM;
    const double pcb_width = 30 * MM;
    const double pcb_height = 3 * MM;

    Sketch pcb = Rectangle(pcb_length, pcb_width);

    // Pin holes along the long edges, then the short edges
    std::vector<Sketch> holes;
    for (int i = 0; i < 65 / 5; ++i) {
        for (const double y : {-15, -10, 10, 15}) holes.push_back(Pos(i * 5 - 30, y) * Circle(1));
    }
    pcb -= holes;
    holes.clear();
    for (const double x : {30, 35}) {
        for (int i = 0; i < 30 / 5 - 1; ++i) holes.push_back(Pos(x, i * 5 - 10) * Circle(1));
    }
    pcb -= holes;

    // Mounting holes
    pcb -= GridLocations(60, 20, 2, 2) * Circle(2);

    const Part board = extrude(pcb, pcb_height);

    example::show(board, "circuit_board");
    export_step(board, "circuit_board.step");
    return 0;
}
