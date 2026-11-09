// cast_shape: the specific type of an untyped Shape (Python's Shape.cast).
#include "build123d/cast.hpp"

#include <TopoDS_Iterator.hxx>

#include "internal.hpp"

namespace build123d {

namespace {

const char* target_name(detail::CastTarget t) {
    switch (t) {
        case detail::CastTarget::VERTEX: return "Vertex";
        case detail::CastTarget::EDGE: return "Edge";
        case detail::CastTarget::WIRE: return "Wire";
        case detail::CastTarget::FACE: return "Face";
        case detail::CastTarget::SHELL: return "Shell";
        case detail::CastTarget::SOLID: return "Solid";
        case detail::CastTarget::COMPOUND: return "Compound";
        case detail::CastTarget::MIXIN1D: return "Mixin1D";
        case detail::CastTarget::MIXIN2D: return "Mixin2D";
        case detail::CastTarget::MIXIN3D: return "Mixin3D";
        case detail::CastTarget::CURVE: return "Curve";
        case detail::CastTarget::SKETCH: return "Sketch";
        case detail::CastTarget::PART: return "Part";
    }
    return "?";
}

const char* type_name(const Shape& shape) {
    if (shape.is_null()) return "empty shape";
    switch (shape.wrapped().ShapeType()) {
        case TopAbs_VERTEX: return "Vertex";
        case TopAbs_EDGE: return "Edge";
        case TopAbs_WIRE: return "Wire";
        case TopAbs_FACE: return "Face";
        case TopAbs_SHELL: return "Shell";
        case TopAbs_SOLID: return "Solid";
        case TopAbs_COMPSOLID: return "CompSolid";
        case TopAbs_COMPOUND: return "Compound";
        default: return "Shape";
    }
}

/// Whether every non-compound object in the compound has the given dimension.
bool compound_of_dim(const TopoDS_Shape& compound, int dim) {
    for (const auto& s : detail::top_level(compound)) {
        if (detail::topods_dim(s) != dim) return false;
    }
    return true;
}

}  // namespace

namespace detail {

void check_cast(const Shape& shape, CastTarget target) {
    bool ok = false;
    const bool null = shape.is_null();
    const TopAbs_ShapeEnum type = null ? TopAbs_SHAPE : shape.wrapped().ShapeType();
    switch (target) {
        case CastTarget::VERTEX: ok = type == TopAbs_VERTEX; break;
        case CastTarget::EDGE: ok = type == TopAbs_EDGE; break;
        case CastTarget::WIRE: ok = type == TopAbs_WIRE; break;
        case CastTarget::FACE: ok = type == TopAbs_FACE; break;
        case CastTarget::SHELL: ok = type == TopAbs_SHELL; break;
        case CastTarget::SOLID: ok = type == TopAbs_SOLID; break;
        case CastTarget::COMPOUND: ok = null || type == TopAbs_COMPOUND; break;
        case CastTarget::MIXIN1D: ok = type == TopAbs_EDGE || type == TopAbs_WIRE; break;
        case CastTarget::MIXIN2D: ok = type == TopAbs_FACE || type == TopAbs_SHELL; break;
        case CastTarget::MIXIN3D: ok = null || type == TopAbs_SOLID || type == TopAbs_COMPOUND; break;
        case CastTarget::CURVE:
            ok = null || type == TopAbs_EDGE || type == TopAbs_WIRE ||
                 (type == TopAbs_COMPOUND && compound_of_dim(shape.wrapped(), 1));
            break;
        case CastTarget::SKETCH:
            ok = null || type == TopAbs_FACE || type == TopAbs_SHELL ||
                 (type == TopAbs_COMPOUND && compound_of_dim(shape.wrapped(), 2));
            break;
        case CastTarget::PART:
            ok = null || type == TopAbs_SOLID || type == TopAbs_COMPSOLID ||
                 (type == TopAbs_COMPOUND && compound_of_dim(shape.wrapped(), 3));
            break;
    }
    if (!ok) {
        throw std::invalid_argument(std::string("Can't cast a ") + type_name(shape) + " to a " + target_name(target));
    }
}

}  // namespace detail

AnyShape cast_shape(const Shape& shape) {
    if (shape.is_null()) return cast_shape<Compound>(shape);
    switch (shape.wrapped().ShapeType()) {
        case TopAbs_VERTEX: return cast_shape<Vertex>(shape);
        case TopAbs_EDGE: return cast_shape<Edge>(shape);
        case TopAbs_WIRE: return cast_shape<Wire>(shape);
        case TopAbs_FACE: return cast_shape<Face>(shape);
        case TopAbs_SHELL: return cast_shape<Shell>(shape);
        case TopAbs_SOLID: return cast_shape<Solid>(shape);
        case TopAbs_COMPOUND: return cast_shape<Compound>(shape);
        default: {
            // A CompSolid: wrap it, as there is no build123d class for it
            Compound result(shape.wrapped());
            result.label = shape.label;
            result.color = shape.color;
            result.children = shape.children;
            return result;
        }
    }
}

}  // namespace build123d
