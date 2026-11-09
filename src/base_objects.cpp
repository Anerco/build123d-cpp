// The base classes of the objects (Python's BaseCurveObject, BaseLineObject, BaseEdgeObject,
// BaseSketchObject and BasePartObject in objects_curve.py, objects_sketch.py, objects_part.py).
#include "build123d/objects.hpp"
#include "internal.hpp"

namespace build123d {

BaseCurveObject::BaseCurveObject(const Curve& curve) : Curve(curve.wrapped()) {}

BaseLineObject::BaseLineObject(const Wire& curve) : Wire(curve.wrapped()) {}

BaseEdgeObject::BaseEdgeObject(const Edge& curve) : Edge(curve.wrapped()) {}

BaseSketchObject::BaseSketchObject(const Shape& obj, double rotation, std::optional<Align2> align) {
    assign(obj, rotation, align);
}

void BaseSketchObject::assign(const Shape& obj_in, double rotation_in, std::optional<Align2> align) {
    Shape obj = obj_in;
    if (align) obj.move(Location(obj.bounding_box().to_align_offset(*align)));
    rotation = rotation_in;
    const ShapeList<Face> new_faces =
        rotation_in != 0.0 ? obj.moved(Rotation(0, 0, rotation_in)).faces() : obj.faces();
    std::vector<TopoDS_Shape> topods;
    for (const auto& f : new_faces) topods.push_back(f.wrapped());
    obj_ = detail::make_compound(topods);
}

BasePartObject::BasePartObject(const Shape& part, const Rotation& rotation, std::optional<Align3> align) {
    assign(part, rotation, align);
}

void BasePartObject::assign(const Shape& part_in, const Rotation& rotation_in, std::optional<Align3> align) {
    Shape part = part_in;
    if (align) part.move(Location(part.bounding_box().to_align_offset(*align)));
    rotation = rotation_in;
    const Shape transformed = rotation_in != Rotation() ? part.moved(rotation_in) : part;
    if (!transformed.is_null() && transformed.wrapped().ShapeType() == TopAbs_COMPOUND) {
        obj_ = transformed.wrapped();  // don't add extra layers
    } else {
        obj_ = detail::make_compound({transformed.wrapped()});
    }
    label = part_in.label;
    children = part_in.children;
}

}  // namespace build123d
