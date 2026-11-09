// The build123d logo with a "customizable" line under it.
// Port of build123d's examples/build123d_customizable_logo_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const Text logo_text("123d", 10, {.align = Align::MIN});
    const double font_height = logo_text.vertices().sort_by(Axis::Y)[-1].Y();

    const Text build_text("build", 5, {.align = Align::CENTER});
    const BoundBox build_bb = build_text.bounding_box();
    const double build_width = build_bb.max.X - build_bb.min.X;

    const Text cust_text("customizable", 2.9, {.font_style = FontStyle::BOLD, .align = Align::CENTER});
    const BoundBox cust_bb = cust_text.bounding_box();

    Line l1({font_height * 0.3, 0}, {font_height * 0.3, font_height});
    const Curve one = l1 + TangentArc(l1.position_at(1), {0, font_height * 0.7}, (l1 % 1) * -1);

    const Sketch two = Pos(font_height * 0.35, 0) * Text("2", 10, {.align = Align::MIN});

    const Sketch three_d_text = Text("3d", 10, {.align = Align::MIN});
    const Part three_d = Pos(font_height * 1.1, 0) * extrude(three_d_text, font_height * 0.3);
    const double logo_width = three_d.vertices().sort_by(Axis::X)[-1].X();

    const TangentArc t1({0, 0}, {1, 0.75}, {1, 0});
    const Curve arrow_left = t1 + mirror(t1, Plane::XZ);

    const double ext_line_length = font_height * 0.5;
    const double dim_line_length = (logo_width - build_width - 2 * font_height * 0.05) / 2;

    l1 = Line({0, -font_height * 0.1}, {0, -ext_line_length - font_height * 0.1});
    const Line l2({logo_width, -font_height * 0.1}, {logo_width, -ext_line_length - font_height * 0.1});
    Curve extension_lines = Curve() + (l1 + l2);
    extension_lines += Pos(l1.position_at(0.5)) * arrow_left;
    extension_lines += (Pos(l2.position_at(0.5)) * Rot(0, 0, 180)) * arrow_left;
    extension_lines += Line(l1.position_at(0.5), l1.position_at(0.5) + Vector(dim_line_length, 0));
    extension_lines += Line(l2.position_at(0.5), l2.position_at(0.5) - Vector(dim_line_length, 0));

    // Precisely center the build Faces
    const Pos p1((l1.position_at(0.5) + l2.position_at(0.5)) / 2 -
                 Vector((build_bb.max.X + build_bb.min.X) / 2, 0));
    Sketch build = p1 * build_text;

    // Place the customizable text under the dimension line
    const Pos p2((l1.position_at(1) + l2.position_at(1)) / 2 - Vector(cust_bb.max.X + cust_bb.min.X, 1.4));
    build += p2 * cust_text;

    const Compound cmpd(std::vector<Shape>{three_d, two, one, build, extension_lines});

    example::show(cmpd, "build123d_customizable_logo");
    export_step(cmpd, "build123d_customizable_logo.step");
    return 0;
}
