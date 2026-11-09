// Projecting faces and text onto a sphere: straight along a direction, and wrapped along
// a path on the surface.
// Port of build123d's examples/projection_algebra.py (Apache-2.0).
#include <build123d/build123d.hpp>

#include "show.hpp"

using namespace build123d;

int main() {
    const Part sphere = Sphere(50);

    // Example 1 - Mapping A Face on Sphere
    Vector projection_direction(0, 1, 0);
    const Sketch square = Plane::ZX.offset(-80) * Rectangle(20, 20);
    const auto square_projected = square.faces()[0].project_to_shape(sphere, projection_direction);
    std::vector<Solid> thickened;
    for (const auto& f : square_projected) thickened.push_back(Solid::thicken(f, 2));
    const Part square_solids = Part() + thickened;
    const Face face = square.faces()[0];
    const Part projection_beams = loft(std::vector<Shape>{face, Pos(0, 160, 0) * face});

    // Example 2 - Flat Projection of Text on Sphere
    projection_direction = Vector(0, -1, 0);
    const Sketch flat_planar_text = Rot(90, 0, 0) * Text("Flat", 30);
    std::vector<Shape> projected_faces;
    std::vector<Part> beams;
    for (const auto& f : flat_planar_text.faces()) {
        projected_faces.push_back(f.project_to_shape(sphere, projection_direction)[0]);
        beams.push_back(extrude(f, 80, {.dir = projection_direction}));
    }
    const Sketch flat_projected_text_faces = Sketch() + projected_faces;
    const Part flat_projection_beams = Part() + beams;

    // Example 3 - Project a text string along a path onto a shape
    const Part cyl = Plane::YZ * Cylinder(80, 100, {Align::CENTER, Align::CENTER, Align::MIN});
    const Part obj = sphere - Pos(-50, 0, -70) * cyl;
    const Edge arch_path = obj.edges().sort_by().first();

    const Text text("'the quick brown fox jumped over the lazy dog'", 15,
                    {.align = Align2(Align::MIN, Align::CENTER)});
    const Sketch projected_text(sphere.project_faces(text.faces(), arch_path));

    example::show(square_solids, "projection_square_solids");
    example::show(projection_beams, "projection_beams");
    example::show(flat_projected_text_faces, "projection_flat_text");
    example::show(flat_projection_beams, "projection_flat_beams");
    example::show(projected_text, "projection_text_on_path");
    export_step(Compound::make_assembly({sphere, square_solids, Pos(-100, -100) * flat_projected_text_faces,
                                         Pos(100, 100) * projected_text}),
                "projection.step");
    return 0;
}
