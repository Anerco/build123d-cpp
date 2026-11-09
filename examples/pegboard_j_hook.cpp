// Pegboard J hook: a round profile swept along a path of lines and arcs, flattened top and bottom.
// Port of build123d's examples/pegboard_j_hook_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const double pegd = 6.35 + 0.1;  // mm ~0.25inch
    const double c2c = 25.4;         // mm 1.0inch
    const double arcd = 7.2;
    const double both = 10;
    const double topx = 6;
    const double midx = 8;
    const double maind = 0.82 * pegd;
    const double midd = 1.0 * pegd;
    const double hookd = 23;
    const double hookx = 10;
    const double splitz = maind / 2 - 0.1;
    const double topangs = 70;

    // Line on the XY plane
    const Line l1({-both, 0}, {c2c - arcd / 2 - 0.5, 0});
    const JernArc l2(l1.position_at(1), l1 % 1, arcd / 2, topangs);
    const PolarLine l3(l2.position_at(1), topx, l2 % 1);
    const JernArc l4(l3.position_at(1), l3 % 1, arcd / 2, -topangs);
    const PolarLine l5(l4.position_at(1), topx, l4 % 1);
    const JernArc l6(l1.position_at(0), (l1 % 0).reverse(), hookd / 2, 170);
    const PolarLine l7(l6.position_at(1), hookx, l6 % 1);

    // Sweep a circle along the path
    const Curve sprof = Curve() + std::vector<Shape>{l1, l2, l3, l4, l5, l6, l7};
    const Wire wire(sprof.edges());
    Part mainp = sweep(Plane::YZ * Circle(maind / 2), wire);

    // Add the stub that goes into the pegboard
    const Line stub({0, 0}, {0, midx + maind / 2});
    mainp += sweep(Plane::XZ * Circle(midd / 2), stub);

    // Cut the top and bottom flat
    mainp = split(mainp, Plane(Vector(0, 0, -splitz)));
    mainp = split(mainp, Plane(Vector(0, 0, splitz)), Keep::BOTTOM);

    example::show(mainp, "pegboard_j_hook");
    export_step(mainp, "pegboard_j_hook.step");
    return 0;
}
