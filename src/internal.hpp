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

#include "build123d/topology.hpp"

namespace build123d::detail {

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
enum class BoolOp { FUSE, CUT, COMMON };
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

/// Convert a Shape (Edge/Wire/Curve or other 1D) to a wire from its edges.
Wire as_wire(const Shape& shape);
/// The faces of a Face, Sketch, Shell, Solid, ...
std::vector<Face> as_faces(const Shape& shape);

}  // namespace build123d::detail
