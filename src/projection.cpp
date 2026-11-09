// Projection of curves (Edge/Wire.project_to_shape, Mixin1D.project in topology/one_d.py) and
// the project() operation (operations_generic.py).
#include <algorithm>
#include <functional>
#include <limits>
#include <memory>

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepProj_Projection.hxx>
#include <TopoDS.hxx>

#include "build123d/operations.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

// ---------------------------------------------------------------------------
// Curves
// ---------------------------------------------------------------------------

ShapeList<Wire> Wire::project_to_shape(const Shape& target_object, std::optional<Vector> direction,
                                       std::optional<Vector> center) const {
    if (obj_.IsNull() || !target_object) throw std::invalid_argument("Can't project empty Wires or to empty Shapes");

    std::optional<Vector> direction_vector;
    Vector center_point;
    if (direction && !center) {
        direction_vector = direction->normalized();
    } else if (center && !direction) {
        center_point = *center;
    } else {
        throw std::invalid_argument("Provide exactly one of direction or center");
    }

    // Project the wire on the target object
    ShapeList<Wire> output_wires;
    occt("project_to_shape", [&] {
        std::unique_ptr<BRepProj_Projection> projection_object;
        if (direction_vector) {
            projection_object =
                std::make_unique<BRepProj_Projection>(obj_, target_object.wrapped(), direction_vector->to_dir());
        } else {
            projection_object =
                std::make_unique<BRepProj_Projection>(obj_, target_object.wrapped(), center_point.to_pnt());
        }
        // Generate a list of the projected wires with aligned orientation
        const TopAbs_Orientation target_orientation = obj_.Orientation();
        while (projection_object->More()) {
            const TopoDS_Wire projected_wire = projection_object->Current();
            if (target_orientation == projected_wire.Orientation()) {
                output_wires.push_back(Wire(projected_wire));
            } else {
                output_wires.push_back(Wire(projected_wire.Reversed()));
            }
            projection_object->Next();
        }
        return 0;
    });

    // BRepProj_Projection is inconsistent in the order that it returns projected wires, so
    // sort by distance from the original planar wire
    if (output_wires.size() > 1) {
        std::vector<std::pair<Wire, double>> output_wires_distances;
        const Vector planar_wire_center = Shape::center();
        for (const auto& output_wire : output_wires) {
            const Vector output_wire_center = output_wire.center();
            if (direction_vector) {
                const Vector output_wire_direction = (output_wire_center - planar_wire_center).normalized();
                if (output_wire_direction.dot(*direction_vector) >= 0) {
                    output_wires_distances.emplace_back(output_wire, (output_wire_center - planar_wire_center).length());
                }
            } else {
                output_wires_distances.emplace_back(output_wire, (output_wire_center - center_point).length());
            }
        }
        std::stable_sort(output_wires_distances.begin(), output_wires_distances.end(),
                         [](const auto& a, const auto& b) { return a.second < b.second; });
        output_wires.clear();
        for (const auto& w : output_wires_distances) output_wires.push_back(w.first);
    }

    // Clean the wires remove cases where projection artificially split edges
    ShapeList<Wire> cleaned_wires;
    for (const auto& w : output_wires) cleaned_wires.push_back(w.clean());
    return cleaned_wires;
}

ShapeList<Edge> Edge::project_to_shape(const Shape& target_object, std::optional<Vector> direction,
                                       std::optional<Vector> center) const {
    return Wire(*this).project_to_shape(target_object, direction, center).edges();
}

ShapeList<Edge> Mixin1D::project_to_shape(const Shape& target, const Vector& direction) const {
    require_not_null("Can't project an empty edge/wire");
    if (obj_.ShapeType() == TopAbs_EDGE) return Edge(obj_).project_to_shape(target, direction);
    return Wire(obj_).project_to_shape(target, direction).edges();
}

namespace {

std::vector<TopoDS_Shape> curve_projections(const TopoDS_Shape& curve, const Face& face, const Vector& direction) {
    if (curve.IsNull() || !face) throw std::invalid_argument("Can't project an empty Edge or Wire onto empty Face");
    return occt("project", [&] {
        BRepProj_Projection bldr(curve, face.wrapped(), direction.to_dir());
        return top_level(bldr.Shape());
    });
}

}  // namespace

Mixin1D Mixin1D::project(const Face& face, const Vector& direction) const {
    const auto shapes = curve_projections(obj_, face, direction);
    // select the closest projection
    std::optional<Mixin1D> return_value;
    occt("project", [&] {
        BRepExtrema_DistShapeShape dist_calc;
        dist_calc.LoadS1(obj_);
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& shape : shapes) {
            dist_calc.LoadS2(shape);
            dist_calc.Perform();
            const double dist = dist_calc.Value();
            if (dist < min_dist) {
                min_dist = dist;
                return_value = Mixin1D(shape);
            }
        }
        return 0;
    });
    // Python's return value is unbound when nothing was projected
    if (!return_value) throw std::runtime_error("cannot access local variable 'return_value': no projection");
    return *return_value;
}

ShapeList<Mixin1D> Mixin1D::project(const Face& face, const Vector& direction, bool closest) const {
    if (closest) return ShapeList<Mixin1D>{project(face, direction)};
    ShapeList<Mixin1D> out;
    for (const auto& shape : curve_projections(obj_, face, direction)) out.push_back(Mixin1D(shape));
    return out;
}

// ---------------------------------------------------------------------------
// project()
// ---------------------------------------------------------------------------

namespace detail {

ProjectOutput project_impl(const std::vector<const Shape*>& objects, const std::vector<Vector>& vectors,
                           const Plane& workplane) {
    if (objects.empty() && vectors.empty()) throw std::invalid_argument("No object to project");

    // The size of the object determines the size of the target projection screen as the
    // screen is normal to the direction of parallel projection
    std::vector<TopoDS_Shape> shape_list;
    std::vector<Vector> point_list;
    std::vector<Face> face_list;
    std::vector<Shape> line_list;
    for (const Shape* o : objects) {
        if (o->is_null()) continue;
        shape_list.push_back(o->wrapped());
        switch (o->wrapped().ShapeType()) {
            case TopAbs_VERTEX: point_list.emplace_back(Vertex(o->wrapped())); break;
            case TopAbs_FACE: face_list.emplace_back(o->wrapped()); break;
            case TopAbs_EDGE:
            case TopAbs_WIRE: line_list.push_back(*o); break;
            default: break;  // other shapes are not projected
        }
    }
    for (const auto& v : vectors) {
        shape_list.push_back(Vertex(v).wrapped());
        point_list.push_back(v);
    }
    const double object_size = Shape(make_compound(shape_list)).bounding_box(std::nullopt, false).diagonal();

    const Plane& working_plane = workplane;
    const double projection_flip = 1;
    const Face target = Face::make_rect(3 * object_size, 3 * object_size, working_plane);

    ProjectOutput out;
    auto project_one = [&](const Shape& obj, const std::function<ShapeList<Shape>(const Vector&)>& project_to_target) {
        const Vector obj_to_screen = (target.center() - obj.center()).normalized();
        Vector projection_direction;
        if (working_plane.from_local_coords(obj_to_screen).Z < 0) {
            projection_direction = -working_plane.z_dir * projection_flip;
        } else {
            projection_direction = working_plane.z_dir * projection_flip;
        }
        const ShapeList<Shape> projection = project_to_target(projection_direction);
        for (const auto& f : projection.faces()) out.shapes.push_back(f.wrapped());
    };
    for (const auto& face : face_list) {
        project_one(face, [&](const Vector& d) { return face.project_to_shape(target, d); });
    }
    for (const auto& line : line_list) {
        project_one(line, [&](const Vector& d) {
            ShapeList<Shape> projected;
            if (line.wrapped().ShapeType() == TopAbs_EDGE) {
                for (const auto& e : Edge(line.wrapped()).project_to_shape(target, d)) projected.push_back(e);
            } else {
                for (const auto& w : Wire(line.wrapped()).project_to_shape(target, d)) projected.push_back(w);
            }
            return projected;
        });
    }

    for (const auto& pnt : point_list) {
        const Vector pnt_to_target = (working_plane.origin - pnt).normalized();
        Axis projection_axis(pnt, working_plane.z_dir * projection_flip);
        if (working_plane.from_local_coords(pnt_to_target).Z < 0) projection_axis = -projection_axis;
        const auto intersection = working_plane.intersect(projection_axis);
        if (intersection && std::holds_alternative<Axis>(*intersection)) {
            throw std::runtime_error("working_plane and projection_axis are parallel");
        }
        if (intersection) out.points.push_back(working_plane.to_local_coords(std::get<Vector>(*intersection)));
    }
    return out;
}

}  // namespace detail

std::vector<Vector> project(const std::vector<Vector>& points, const Plane& workplane, const std::optional<Shape>&) {
    return detail::project_impl({}, points, workplane).points;
}

std::vector<Vector> project(const Vector& point, const Plane& workplane, const std::optional<Shape>& target) {
    return project(std::vector<Vector>{point}, workplane, target);
}

std::vector<Vector> project(const std::vector<Vertex>& points, const Plane& workplane, const std::optional<Shape>&) {
    return detail::project_impl(detail::shape_ptrs(points), {}, workplane).points;
}

std::vector<Vector> project(const Vertex& point, const Plane& workplane, const std::optional<Shape>& target) {
    return project(std::vector<Vertex>{point}, workplane, target);
}

std::variant<Compound, std::vector<Vector>> project(const std::vector<Shape>& objects, const Plane& workplane,
                                                    const std::optional<Shape>&) {
    const auto output = detail::project_impl(detail::shape_ptrs(objects), {}, workplane);
    if (!output.points.empty()) return output.points;
    return Compound(detail::make_compound(output.shapes));
}

std::variant<Compound, std::vector<Vector>> project(const Shape& object, const Plane& workplane,
                                                    const std::optional<Shape>& target) {
    return project(std::vector<Shape>{object}, workplane, target);
}

}  // namespace build123d
