// Private helpers shared by the build123d-cpp sources.
#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <BRepBuilderAPI_TransitionMode.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_JoinType.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopLoc_Location.hxx>
#include <gp_EulerSequence.hxx>

#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#if OCC_VERSION_HEX >= 0x080000
#include <NCollection_HArray1.hxx>
#include <NCollection_HSequence.hxx>
#else
#include <TColStd_HArray1OfBoolean.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#endif

#include <initializer_list>
#include <memory>

#include "build123d/history.hpp"
#include "build123d/topology.hpp"

namespace build123d::detail {

/// The topo_path of shapes extracted from `source`: source's own path plus source.
std::shared_ptr<const std::vector<Shape>> extraction_path(const Shape& source);

/// build123d's find_same_topods: the candidate that is `query` (IsSame), else the first
/// that shares its TShape at another location (IsPartner).
std::optional<TopoDS_Shape> find_same_topods(const TopoDS_Shape& query, const std::vector<TopoDS_Shape>& candidates);
/// The location that carries `source`'s frame onto `target`'s.
TopLoc_Location relocation_between(const TopoDS_Shape& source, const TopoDS_Shape& target);
/// The faces of `parent` (default: edge.topo_parent()) that meet at `edge`, in the edge's frame.
std::vector<TopoDS_Face> topo_explore_connected_faces(const Shape& edge,
                                                      const std::optional<Shape>& parent = std::nullopt);

/// build123d's Shape._select: narrow `owner`'s sub-shapes by what its last operation did.
/// std::invalid_argument when a LAST/NEW selection is asked of a shape with no record.
const ShapeHistory* history_for_select(const Shape& owner, Select select);
template <class T>
ShapeList<T> select_shapes(const Shape& owner, ShapeList<T> shapes, Select select) {
    if (select == Select::ALL) return shapes;
    const ShapeHistory* record = history_for_select(owner, select);
    ShapeList<T> out;
    for (auto& s : shapes) {
        const bool keep = select == Select::LAST ? record->is_last(s.wrapped()) : record->is_new(s.wrapped());
        if (keep) out.push_back(std::move(s));
    }
    return out;
}

using OccShapeList = NCollection_List<TopoDS_Shape>;
using OccShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using OccShapeListMap =
    NCollection_IndexedDataMap<TopoDS_Shape, OccShapeList, TopTools_ShapeMapHasher>;
#if OCC_VERSION_HEX >= 0x080000
using OccHSeqShape = NCollection_HSequence<TopoDS_Shape>;
using OccHArrayPnt = NCollection_HArray1<gp_Pnt>;
using OccHArrayReal = NCollection_HArray1<double>;
using OccHArrayBool = NCollection_HArray1<bool>;
#else
using OccHSeqShape = TopTools_HSequenceOfShape;
using OccHArrayPnt = TColgp_HArray1OfPnt;
using OccHArrayReal = TColStd_HArray1OfReal;
using OccHArrayBool = TColStd_HArray1OfBoolean;
#endif
using OccArrayPnt = NCollection_Array1<gp_Pnt>;
using OccArrayVec = NCollection_Array1<gp_Vec>;
using OccArrayReal = NCollection_Array1<double>;
using OccArrayInt = NCollection_Array1<int>;
using OccArray2Pnt = NCollection_Array2<gp_Pnt>;
using OccArray2Real = NCollection_Array2<double>;

/// ShapeAnalysis_FreeBounds::ConnectEdgesToWires.
std::vector<TopoDS_Wire> connect_edges_to_wires(const std::vector<TopoDS_Shape>& edges, double tol);

std::string occt_message(const Standard_Failure& e);

/// Run an OCCT operation, converting Standard_Failure into std::runtime_error.
template <class F>
auto occt(const char* context, F&& f) -> decltype(f()) {
    try {
        return f();
    } catch (const Standard_Failure& e) {
        throw std::runtime_error(std::string(context) + ": " + occt_message(e));
    }
}

bool clean_enabled();

// ---- topology helpers
TopoDS_Compound make_compound(const std::vector<TopoDS_Shape>& shapes);
/// Non-compound children, flattening nested compounds.
std::vector<TopoDS_Shape> top_level(const TopoDS_Shape& shape);
/// Direct children of a compound (or the shape itself).
std::vector<TopoDS_Shape> children(const TopoDS_Shape& shape);
int topods_dim(const TopoDS_Shape& shape);
/// Strip compounds with a single element.
TopoDS_Shape unwrap(const TopoDS_Shape& shape, bool fully = true);
/// Unique sub-shapes of a type, in explorer order (the last orientation seen is kept).
std::vector<TopoDS_Shape> entities(const TopoDS_Shape& shape, TopAbs_ShapeEnum type);
OccShapeList to_list(const std::vector<TopoDS_Shape>& shapes);

// ---- algorithms
TopoDS_Shape clean_shape(const TopoDS_Shape& shape);
TopoDS_Shape fix_shape(const TopoDS_Shape& shape);
enum class BoolOp { FUSE, CUT, COMMON, SECTION };
/// Boolean with optional fuzzy value; cleans (if enabled) and unwraps single compounds.
TopoDS_Shape bool_op(BoolOp op, const std::vector<TopoDS_Shape>& args,
                     const std::vector<TopoDS_Shape>& tools, double fuzzy = 0.0,
                     bool clean = true, bool glue = false);
TopoDS_Face make_face_from_wires(const TopoDS_Wire& outer,
                                 const std::vector<TopoDS_Wire>& inner = {});
TopoDS_Shape extrude_shape(const TopoDS_Shape& shape, const Vector& direction);
TopoDS_Shape make_loft(const std::vector<TopoDS_Shape>& sections, bool filled, bool ruled);
TopoDS_Shape sew_faces(const std::vector<TopoDS_Shape>& faces);
TopoDS_Shape transform(const TopoDS_Shape& shape, const gp_Trsf& trsf, bool copy);

// ---- properties
/// GProp properties of the given dimension (1 linear, 2 surface, 3 volume).
GProp_GProps properties(const TopoDS_Shape& shape, int dim);
double linear_length(const TopoDS_Shape& shape);
/// Unnormalized normal where the origin projects onto the face (Plane.get_topods_face_normal).
Vector face_normal_at_origin_projection(const TopoDS_Face& face);
Vector face_normal_at(const TopoDS_Face& face, const Vector& point);

// ---- enum conversions
gp_EulerSequence to_occ(Intrinsic ordering);
gp_EulerSequence to_occ(Extrinsic ordering);
GeomAbs_JoinType to_occ(Kind kind);
BRepBuilderAPI_TransitionMode to_occ(Transition transition);
gp_Trsf euler_trsf(const Vector& angles_deg, gp_EulerSequence sequence);

/// Check two vectors for equality like build123d (rounded to 5 digits).
bool rounded_equal(double a, double b, int digits = 5);
double py_round(double value, int digits);
/// Python's repr() of a float: the shortest round-tripping digits, e.g. 1.0, 0.0001, 1e-05.
std::string py_repr(double value);
/// Python's float % (the result has the sign of the divisor).
double py_float_mod(double a, double b);

/// Convert a Shape (Edge/Wire/Curve or other 1D) to a wire from its edges.
Wire as_wire(const Shape& shape);
/// The faces of a Face, Sketch, Shell, Solid, ...
std::vector<Face> as_faces(const Shape& shape);

// ---- attributes and history (boolean.cpp)

/// build123d's Shape.copy_attributes_to: copy `from`'s label, color, material and
/// topo_path to `to` where `to` has none set (joints: see the marked spot in the
/// definition). `exceptions` names attributes not to copy ("label", "color",
/// "material", "topo_path", "joints").
void copy_attributes_to(const Shape& from, Shape& to, std::initializer_list<const char*> exceptions = {});

/// A TopoDS boolean and the record of what it did (build123d's Shape._bool_op without
/// the result typing). `shape` is null for a COMMON with an empty side (build123d
/// returns an empty shape then).
struct BoolResult {
    TopoDS_Shape shape;
    std::shared_ptr<ShapeHistory> history;
};
BoolResult bool_op_history(BoolOp op, const std::vector<TopoDS_Shape>& args,
                           const std::vector<TopoDS_Shape>& tools, double fuzzy = 0.0,
                           bool glue = false);
/// build123d's Shape._bool_op: the result (unwrapped) carrying the history, with
/// args[0]'s attributes copied.
Shape bool_op_shape(BoolOp op, const std::vector<const Shape*>& args,
                    const std::vector<const Shape*>& tools, double fuzzy = 0.0, bool glue = false);
/// build123d's Shape._bool_op_list: the pieces of the result, each made by the operation.
ShapeList<Shape> bool_op_list(BoolOp op, const Shape& arg, const Shape& tool);
/// The shape with `history` as its record (build123d's _made_by).
template <class S>
S made_by(S shape, std::shared_ptr<ShapeHistory> history) {
    shape.set_history(std::move(history));
    return shape;
}
/// A copy of `shape` as type T (rewrapping the TopoDS_Shape) keeping all of its attributes.
template <class T>
T rewrap(const Shape& shape) {
    T result(shape.wrapped());
    ShapeAccess::copy_attributes(shape, result);
    return result;
}

}  // namespace build123d::detail
