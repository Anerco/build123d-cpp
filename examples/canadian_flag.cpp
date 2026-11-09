// The Canadian flag waving in the wind: planar faces projected onto a spline surface,
// exported as a colored assembly.
// Port of build123d's examples/canadian_flag_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include <cmath>

#include "show.hpp"

using namespace build123d;

int main() {
    const double height = 50;
    const double width = 2 * height;
    const double wave_amplitude = 3;

    // Calculate the surface displacement of the flag at a given position
    auto surface = [](double amplitude, double u, double v) {
        return v * amplitude / 20 * std::cos(3.5 * PI * u) + amplitude / 10 * v * std::sin(1.1 * PI * v);
    };

    // Note that the surface to project on must be a little larger than the faces
    // being projected onto it to create valid projected faces
    std::vector<std::vector<Vector>> points;
    for (int v = 0; v <= 40; ++v) {
        auto& row = points.emplace_back();
        for (int u = 0; u <= 40; ++u) {
            row.emplace_back(width * (v * 1.1 / 40 - 0.05), height * (u * 1.2 / 40 - 0.1),
                             height * surface(wave_amplitude, u / 40.0, v / 40.0) / 2);
        }
    }
    const Face the_wind = Face::make_surface_from_array_of_points(points);

    const Sketch field_planar = Plane::XY.offset(10) * Rectangle(width / 4, height, Align::MIN);
    const Face west_field_planar = field_planar.faces()[0];
    const Sketch east_field_planar = mirror(west_field_planar, Plane::YZ.offset(width / 2));

    const Polyline l1({{0.0000, 0.0771}, {0.0187, 0.0771}, {0.0094, 0.2569}});
    const Polyline l2({{0.0325, 0.2773}, {0.2115, 0.2458}, {0.1873, 0.3125}});
    const RadiusArc r1(l1.position_at(1), l2.position_at(0), 0.0271);
    const Polyline l3({{0.1915, 0.3277}, {0.3875, 0.4865}, {0.3433, 0.5071}});
    const TangentArc r2(l2.position_at(1), l3.position_at(0), l2 % 1);
    const Polyline l4({{0.3362, 0.5235}, {0.375, 0.6427}, {0.2621, 0.6188}});
    const SagittaArc r3(l3.position_at(1), l4.position_at(0), 0.003);
    const Polyline l5({{0.2469, 0.6267}, {0.225, 0.6781}, {0.1369, 0.5835}});
    const ThreePointArc r4(l4.position_at(1),
                           (l4.position_at(1) + l5.position_at(0)) * 0.5 + Vector(-0.002, -0.002),
                           l5.position_at(0));
    const Polyline l6({{0.1138, 0.5954}, {0.1562, 0.8146}, {0.0881, 0.7752}});
    const Spline s({l5.position_at(1), l6.position_at(0)}, {l5 % 1, l6 % 0}, {2, 2});
    const Line l7({0.0692, 0.7808}, {0.0000, 0.9167});
    const TangentArc r5(l6.position_at(1), l7.position_at(0), l6 % 1);

    Curve outline = l1 + std::vector<Shape>{l2, r1, l3, r2, l4, r3, l5, r4, l6, s, l7, r5};
    outline += mirror(outline, Plane::YZ);

    const Sketch maple_leaf_unit = make_face(outline);
    const Sketch center_field_unit = Rectangle(1, 1, {Align::CENTER, Align::MIN}) - maple_leaf_unit;

    auto scale_move = [&](const Sketch& obj) { return Plane(Vector(width / 2, 0, 10)) * scale(obj, height); };
    auto project = [&](const Shape& obj) {
        return obj.faces()[0].project_to_shape(the_wind, Vector(0, 0, -1))[0];
    };

    const Sketch maple_leaf_planar = scale_move(maple_leaf_unit);
    const Sketch center_field_planar = scale_move(center_field_unit);

    Shape west_field = project(west_field_planar);
    west_field.color = Color("red");
    Shape east_field = project(east_field_planar);
    east_field.color = Color("red");
    Shape center_field = project(center_field_planar);
    center_field.color = Color("white");
    Shape maple_leaf = project(maple_leaf_planar);
    maple_leaf.color = Color("red");

    const Compound canadian_flag = Compound::make_assembly({west_field, east_field, center_field, maple_leaf});

    example::show(the_wind, "the_wind");
    example::show(canadian_flag, "canadian_flag");
    export_step(Rot(90, 0, 0) * canadian_flag, "canadian_flag.step");
    return 0;
}
