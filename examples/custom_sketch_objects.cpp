// A playing card box whose lid is engraved with custom sketch objects (card suits).
// Port of build123d's examples/custom_sketch_objects_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

namespace {

// The outline of each suit, scaled to `height`. These are free functions because inside a
// class derived from Sketch, its mirror() and scale() members hide the free operations.
Sketch club(double height) {
    const Line l0({0, -188}, {76, -188});
    const Bezier b0({l0.position_at(1), {61, -185}, {33, -173}, {17, -81}});
    const Bezier b1({b0.position_at(1), {49, -128}, {146, -145}, {167, -67}});
    const Bezier b2({b1.position_at(1), {187, 9}, {94, 52}, {32, 18}});
    const Bezier b3({b2.position_at(1), {92, 57}, {113, 188}, {0, 188}});
    Curve outline = l0 + b0 + b1 + b2 + b3;
    outline += mirror(outline, Plane::YZ);
    const Sketch face = make_face(outline);
    return scale(face, height / face.bounding_box().size.Y);
}

Sketch spade(double height) {
    const Bezier b0({{0, 198}, {6, 190}, {41, 127}, {112, 61}});
    const Bezier b1({b0.position_at(1), {242, -72}, {114, -168}, {11, -105}});
    const Bezier b2({b1.position_at(1), {31, -174}, {42, -179}, {53, -198}});
    const Line l0(b2.position_at(1), {0, -198});
    Curve outline = b0 + b1 + b2 + l0;
    outline += mirror(outline, Plane::YZ);
    const Sketch face = make_face(outline);
    return scale(face, height / face.bounding_box().size.Y);
}

Sketch heart(double height) {
    const Bezier b1({{0, 146}, {20, 169}, {67, 198}, {97, 198}});
    const Bezier b2({b1.position_at(1), {125, 198}, {151, 186}, {168, 167}});
    const Bezier b3({b2.position_at(1), {197, 133}, {194, 88}, {158, 31}});
    const Bezier b4({b3.position_at(1), {126, -13}, {94, -48}, {62, -95}});
    const Bezier b5({b4.position_at(1), {40, -128}, {0, -198}});
    Curve outline = b1 + b2 + b3 + b4 + b5;
    outline += mirror(outline, Plane::YZ);
    const Sketch face = make_face(outline);
    return scale(face, height / face.bounding_box().size.Y);
}

Sketch diamond(double height) {
    Curve outline = Bezier({{135, 0}, {94, 69}, {47, 134}, {0, 198}});
    outline += mirror(outline, Plane::XZ);
    outline += mirror(outline, Plane::YZ);
    const Sketch face = make_face(outline);
    return scale(face, height / face.bounding_box().size.Y);
}

}  // namespace

// Custom sketch objects are classes derived from Sketch.
class Club : public Sketch {
public:
    explicit Club(double height) : Sketch(club(height)) {}
};

class Spade : public Sketch {
public:
    explicit Spade(double height) : Sketch(spade(height)) {}
};

class Heart : public Sketch {
public:
    explicit Heart(double height) : Sketch(heart(height)) {}
};

class Diamond : public Sketch {
public:
    explicit Diamond(double height) : Sketch(diamond(height)) {}
};

int main() {
    // The size of the card storage
    const double pocket_w = 2.5 * IN + 2 * MM;
    const double pocket_l = 3.5 * IN + 2 * MM;
    const double pocket_t = 0.5 * IN + 2 * MM;
    const double wall_t = 3 * MM;                   // Wall thickness
    const double bottom_t = wall_t / 2;             // Top and bottom thickness
    const double lid_gap = 0.5 * MM;                // Spacing between base and lid
    const double lip_t = wall_t / 2 - lid_gap / 2;  // Lip thickness

    const Sketch box_plan = RectangleRounded(pocket_w + 2 * wall_t, pocket_l + 2 * wall_t, pocket_w / 15);
    Part box = extrude(box_plan, bottom_t + pocket_t / 2);
    const Face base_top = box.faces().sort_by(Axis::Z).last();
    const Sketch walls = Plane(base_top) * offset(box_plan, -lip_t);
    box += extrude(walls, pocket_t / 2);
    const Sketch top = Plane::XY.offset(wall_t / 2) * offset(box_plan, -wall_t);
    box -= extrude(top, pocket_t);

    Part pocket = extrude(box_plan, pocket_t / 2 + bottom_t);
    const Sketch lid_bottom = offset(box_plan, -(wall_t - lip_t));
    pocket -= extrude(lid_bottom, pocket_t / 2);
    pocket = Pos(0, 0, (wall_t + pocket_t) / 2) * pocket;
    const Plane plane(pocket.faces().sort_by().last());

    Sketch suites = Pos(-0.3 * pocket_w, 0.3 * pocket_l) * Heart(pocket_l / 5);
    suites += Pos(-0.3 * pocket_w, -0.3 * pocket_l) * Diamond(pocket_l / 5);
    suites += Pos(0.3 * pocket_w, 0.3 * pocket_l) * Spade(pocket_l / 5);
    suites += Pos(0.3 * pocket_w, -0.3 * pocket_l) * Club(pocket_l / 5);
    suites = plane * suites;

    const Part lid = pocket - extrude(suites, -wall_t, {.dir = Vector(0, 0, 1)});

    example::show(box, "card_box");
    example::show(lid, "card_box_lid");
    export_step(Compound::make_assembly({box, Pos(pocket_w + 20, 0, 0) * lid}), "card_box.step");
    return 0;
}
