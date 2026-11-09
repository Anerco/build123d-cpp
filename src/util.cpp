#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include <BOPAlgo_GlueEnum.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepLib_FindSurface.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Quaternion.hxx>

#include "internal.hpp"

namespace build123d {

namespace {
thread_local bool g_clean = true;
}

SkipClean::SkipClean() : previous_(g_clean) { g_clean = false; }
SkipClean::~SkipClean() { g_clean = previous_; }
bool SkipClean::clean_enabled() { return g_clean; }

namespace detail {

bool clean_enabled() { return g_clean; }

std::string occt_message(const Standard_Failure& e) {
#if OCC_VERSION_HEX >= 0x080000
    std::string msg = e.what() ? e.what() : "";
    std::string type = e.ExceptionType();
#else
    std::string msg = e.GetMessageString() ? e.GetMessageString() : "";
    std::string type = e.DynamicType()->Name();
#endif
    return msg.empty() ? type : type + ": " + msg;
}

double py_round(double value, int digits) {
    if (!std::isfinite(value) || std::abs(value) >= 1e15) return value;
    // Python's round() rounds the decimal representation; printing with the requested
    // number of digits and parsing back gives the same result for all practical values.
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", digits, value);
    double r = std::strtod(buf, nullptr);
    return r == 0.0 ? 0.0 : r;
}

bool rounded_equal(double a, double b, int digits) { return py_round(a, digits) == py_round(b, digits); }

TopoDS_Compound make_compound(const std::vector<TopoDS_Shape>& shapes) {
    TopoDS_Compound comp;
    BRep_Builder builder;
    builder.MakeCompound(comp);
    for (const auto& s : shapes) {
        if (!s.IsNull()) builder.Add(comp, s);
    }
    return comp;
}

std::vector<TopoDS_Shape> top_level(const TopoDS_Shape& shape) {
    std::vector<TopoDS_Shape> out;
    if (shape.IsNull()) return out;
    std::vector<TopoDS_Shape> stack{shape};
    while (!stack.empty()) {
        TopoDS_Shape current = stack.back();
        stack.pop_back();
        if (current.ShapeType() == TopAbs_COMPOUND) {
            for (TopoDS_Iterator it(current); it.More(); it.Next()) {
                const TopoDS_Shape& child = it.Value();
                if (child.ShapeType() == TopAbs_COMPOUND) {
                    stack.push_back(child);
                } else {
                    out.push_back(child);
                }
            }
        } else {
            out.push_back(current);
        }
    }
    return out;
}

std::vector<TopoDS_Shape> children(const TopoDS_Shape& shape) {
    std::vector<TopoDS_Shape> out;
    if (shape.IsNull()) return out;
    if (shape.ShapeType() != TopAbs_COMPOUND) return {shape};
    for (TopoDS_Iterator it(shape); it.More(); it.Next()) out.push_back(it.Value());
    return out;
}

int topods_dim(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return -1;
    switch (shape.ShapeType()) {
        case TopAbs_VERTEX: return 0;
        case TopAbs_EDGE:
        case TopAbs_WIRE: return 1;
        case TopAbs_FACE:
        case TopAbs_SHELL: return 2;
        case TopAbs_SOLID: return 3;
        case TopAbs_COMPSOLID: return 3;
        case TopAbs_COMPOUND: {
            int dim = -2;
            for (const auto& s : top_level(shape)) {
                const int d = topods_dim(s);
                if (dim == -2) {
                    dim = d;
                } else if (d != dim) {
                    return -1;
                }
            }
            return dim == -2 ? -1 : dim;
        }
        default: return -1;
    }
}

TopoDS_Shape unwrap(const TopoDS_Shape& shape, bool fully) {
    if (shape.IsNull() || shape.ShapeType() != TopAbs_COMPOUND) return shape;
    if (shape.NbChildren() == 1) {
        TopoDS_Iterator it(shape);
        const TopoDS_Shape single = it.Value();
        if (single.ShapeType() == TopAbs_COMPOUND) return unwrap(single, fully);
        return fully ? single : shape;
    }
    return shape;
}

std::vector<TopoDS_Shape> entities(const TopoDS_Shape& shape, TopAbs_ShapeEnum type) {
    std::vector<TopoDS_Shape> out;
    if (shape.IsNull()) return out;
    OccShapeMap seen;
    for (TopExp_Explorer exp(shape, type); exp.More(); exp.Next()) {
        const TopoDS_Shape& item = exp.Current();
        const int index = seen.FindIndex(item);
        if (index == 0) {
            seen.Add(item);
            out.push_back(item);
        } else {
            out[static_cast<std::size_t>(index - 1)] = item;
        }
    }
    return out;
}

OccShapeList to_list(const std::vector<TopoDS_Shape>& shapes) {
    OccShapeList list;
    for (const auto& s : shapes) {
        if (!s.IsNull()) list.Append(s);
    }
    return list;
}

TopoDS_Shape clean_shape(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return shape;
    try {
        ShapeUpgrade_UnifySameDomain upgrader(shape, true, true, true);
        upgrader.AllowInternalEdges(false);
        upgrader.Build();
        return upgrader.Shape();
    } catch (const Standard_Failure&) {
        return shape;  // best effort, like build123d
    }
}

TopoDS_Shape fix_shape(const TopoDS_Shape& shape) {
    ShapeFix_Shape fixer(shape);
    fixer.Perform();
    return fixer.Shape();
}

TopoDS_Shape bool_op(BoolOp op, const std::vector<TopoDS_Shape>& args,
                     const std::vector<TopoDS_Shape>& tools, double fuzzy, bool clean, bool glue) {
    return occt("Boolean operation", [&] {
        std::unique_ptr<BRepAlgoAPI_BooleanOperation> operation;
        switch (op) {
            case BoolOp::FUSE: operation = std::make_unique<BRepAlgoAPI_Fuse>(); break;
            case BoolOp::CUT: operation = std::make_unique<BRepAlgoAPI_Cut>(); break;
            case BoolOp::COMMON: operation = std::make_unique<BRepAlgoAPI_Common>(); break;
            case BoolOp::SECTION: operation = std::make_unique<BRepAlgoAPI_Section>(); break;
        }
        operation->SetArguments(to_list(args));
        operation->SetTools(to_list(tools));
        operation->SetRunParallel(true);
        if (fuzzy > 0) operation->SetFuzzyValue(fuzzy);
        if (glue) operation->SetGlue(BOPAlgo_GlueShift);
        operation->Build();
        if (!operation->IsDone() || operation->HasErrors()) {
            throw std::runtime_error("Boolean operation failed");
        }
        TopoDS_Shape result = operation->Shape();
        if (clean && clean_enabled()) result = clean_shape(result);
        return unwrap(result, true);
    });
}

TopoDS_Face make_face_from_wires(const TopoDS_Wire& outer, const std::vector<TopoDS_Wire>& inner) {
    if (!inner.empty() && !BRep_Tool::IsClosed(outer)) {
        throw std::invalid_argument("Cannot build face(s): outer wire is not closed");
    }
    std::vector<TopoDS_Shape> all{outer};
    all.insert(all.end(), inner.begin(), inner.end());
    BRepLib_FindSurface finder(make_compound(all), -1, true);
    if (!finder.Found()) throw std::invalid_argument("Cannot build face(s): wires not planar");

    return occt("Face", [&] {
        ShapeFix_Shape fix_outer(outer);
        fix_outer.Perform();
        const TopoDS_Wire wire = TopoDS::Wire(fix_outer.Shape());
        BRepBuilderAPI_MakeFace builder(wire, true);
        for (const auto& w : inner) {
            if (!BRep_Tool::IsClosed(w)) {
                throw std::invalid_argument("Cannot build face(s): inner wire is not closed");
            }
            ShapeFix_Shape fix_inner(w);
            fix_inner.Perform();
            builder.Add(TopoDS::Wire(fix_inner.Shape()));
        }
        builder.Build();
        if (!builder.IsDone()) {
            throw std::invalid_argument("Cannot build face(s): error " +
                                        std::to_string(static_cast<int>(builder.Error())));
        }
        ShapeFix_Face fix_face(builder.Face());
        fix_face.FixOrientation();
        fix_face.Perform();
        return TopoDS::Face(fix_face.Result());
    });
}

TopoDS_Shape extrude_shape(const TopoDS_Shape& shape, const Vector& direction) {
    if (shape.IsNull()) throw std::invalid_argument("Can't extrude an empty shape");
    const auto type = shape.ShapeType();
    if (type != TopAbs_VERTEX && type != TopAbs_EDGE && type != TopAbs_WIRE &&
        type != TopAbs_FACE && type != TopAbs_SHELL) {
        throw std::invalid_argument("extrude not supported for this shape type");
    }
    return occt("extrude", [&] {
        BRepPrimAPI_MakePrism prism(shape, direction.to_vec());
        TopoDS_Shape result = prism.Shape();
        if (result.ShapeType() == TopAbs_COMPSOLID) {
            std::vector<TopoDS_Shape> solids;
            for (TopExp_Explorer exp(result, TopAbs_SOLID); exp.More(); exp.Next()) {
                solids.push_back(exp.Current());
            }
            result = make_compound(solids);
        }
        return result;
    });
}

TopoDS_Shape make_loft(const std::vector<TopoDS_Shape>& sections, bool filled, bool ruled) {
    if (sections.size() < 2) throw std::invalid_argument("More than one wire is required");
    std::size_t vertex_count = 0;
    for (const auto& s : sections) {
        if (s.ShapeType() == TopAbs_VERTEX) ++vertex_count;
    }
    const bool first_v = sections.front().ShapeType() == TopAbs_VERTEX;
    const bool last_v = sections.back().ShapeType() == TopAbs_VERTEX;
    if (vertex_count > 2) throw std::invalid_argument("Only two vertices are allowed");
    if (vertex_count == 1 && !first_v && !last_v) {
        throw std::invalid_argument("The vertex must be either at the beginning or end of the list");
    }
    if (vertex_count == 2) {
        if (sections.size() == 2) {
            throw std::invalid_argument("You can't have only 2 vertices to loft; try adding some wires");
        }
        if (!(first_v && last_v)) {
            throw std::invalid_argument("The vertices must be at the beginning and end of the list");
        }
    }
    return occt("loft", [&] {
        BRepOffsetAPI_ThruSections builder(filled, ruled);
        for (const auto& s : sections) {
            if (s.ShapeType() == TopAbs_VERTEX) {
                builder.AddVertex(TopoDS::Vertex(s));
            } else if (s.ShapeType() == TopAbs_WIRE) {
                builder.AddWire(TopoDS::Wire(s));
            }
        }
        builder.Build();
        return builder.Shape();
    });
}

TopoDS_Shape sew_faces(const std::vector<TopoDS_Shape>& faces) {
    BRepBuilderAPI_Sewing sewing;
    for (const auto& f : faces) sewing.Add(f);
    sewing.Perform();
    return sewing.SewedShape();
}

TopoDS_Shape transform(const TopoDS_Shape& shape, const gp_Trsf& trsf, bool copy) {
    return occt("transform", [&] {
        BRepBuilderAPI_Transform t(shape, trsf, copy);
        return t.Shape();
    });
}

GProp_GProps properties(const TopoDS_Shape& shape, int dim) {
    GProp_GProps props;
    switch (dim) {
        case 1: BRepGProp::LinearProperties(shape, props); break;
        case 2: BRepGProp::SurfaceProperties(shape, props); break;
        default: BRepGProp::VolumeProperties(shape, props); break;
    }
    return props;
}

double linear_length(const TopoDS_Shape& shape) { return properties(shape, 1).Mass(); }

namespace {
gp_Vec face_normal_vec(const TopoDS_Face& face, const gp_Pnt& point) {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    GeomAPI_ProjectPointOnSurf projector(point, surface);
    double u = 0, v = 0;
    projector.LowerDistanceParameters(u, v);
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(face).Normal(u, v, p, n);
    return n;
}
}  // namespace

Vector face_normal_at_origin_projection(const TopoDS_Face& face) {
    return Vector(face_normal_vec(face, gp_Pnt(0, 0, 0)));
}

Vector face_normal_at(const TopoDS_Face& face, const Vector& point) {
    return Vector(face_normal_vec(face, point.to_pnt())).normalized();
}

gp_EulerSequence to_occ(Intrinsic o) {
    switch (o) {
        case Intrinsic::XYZ: return gp_Intrinsic_XYZ;
        case Intrinsic::XZY: return gp_Intrinsic_XZY;
        case Intrinsic::YZX: return gp_Intrinsic_YZX;
        case Intrinsic::YXZ: return gp_Intrinsic_YXZ;
        case Intrinsic::ZXY: return gp_Intrinsic_ZXY;
        case Intrinsic::ZYX: return gp_Intrinsic_ZYX;
        case Intrinsic::XYX: return gp_Intrinsic_XYX;
        case Intrinsic::XZX: return gp_Intrinsic_XZX;
        case Intrinsic::YZY: return gp_Intrinsic_YZY;
        case Intrinsic::YXY: return gp_Intrinsic_YXY;
        case Intrinsic::ZXZ: return gp_Intrinsic_ZXZ;
        case Intrinsic::ZYZ: return gp_Intrinsic_ZYZ;
    }
    return gp_Intrinsic_XYZ;
}

gp_EulerSequence to_occ(Extrinsic o) {
    switch (o) {
        case Extrinsic::XYZ: return gp_Extrinsic_XYZ;
        case Extrinsic::XZY: return gp_Extrinsic_XZY;
        case Extrinsic::YZX: return gp_Extrinsic_YZX;
        case Extrinsic::YXZ: return gp_Extrinsic_YXZ;
        case Extrinsic::ZXY: return gp_Extrinsic_ZXY;
        case Extrinsic::ZYX: return gp_Extrinsic_ZYX;
        case Extrinsic::XYX: return gp_Extrinsic_XYX;
        case Extrinsic::XZX: return gp_Extrinsic_XZX;
        case Extrinsic::YZY: return gp_Extrinsic_YZY;
        case Extrinsic::YXY: return gp_Extrinsic_YXY;
        case Extrinsic::ZXZ: return gp_Extrinsic_ZXZ;
        case Extrinsic::ZYZ: return gp_Extrinsic_ZYZ;
    }
    return gp_Extrinsic_XYZ;
}

GeomAbs_JoinType to_occ(Kind kind) {
    switch (kind) {
        case Kind::ARC: return GeomAbs_Arc;
        case Kind::INTERSECTION: return GeomAbs_Intersection;
        case Kind::TANGENT: return GeomAbs_Tangent;
    }
    return GeomAbs_Arc;
}

BRepBuilderAPI_TransitionMode to_occ(Transition transition) {
    switch (transition) {
        case Transition::TRANSFORMED: return BRepBuilderAPI_Transformed;
        case Transition::ROUND: return BRepBuilderAPI_RoundCorner;
        case Transition::RIGHT: return BRepBuilderAPI_RightCorner;
    }
    return BRepBuilderAPI_Transformed;
}

gp_Trsf euler_trsf(const Vector& angles, gp_EulerSequence sequence) {
    gp_Quaternion q;
    q.SetEulerAngles(sequence, angles.X * DEG2RAD, angles.Y * DEG2RAD, angles.Z * DEG2RAD);
    gp_Trsf t;
    t.SetRotation(q);
    return t;
}

[[noreturn]] void throw_count(const char* type, std::size_t count) {
    throw std::invalid_argument(std::string("Expected exactly one ") + type + ", found " +
                                std::to_string(count));
}

TopoDS_Shape make_compound_topods(const std::vector<TopoDS_Shape>& shapes) {
    return make_compound(shapes);
}

int topods_type(const TopoDS_Shape& shape) {
    return shape.IsNull() ? -1 : static_cast<int>(shape.ShapeType());
}

}  // namespace detail
}  // namespace build123d
