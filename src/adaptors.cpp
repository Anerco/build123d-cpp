// geom_adaptor of Edge, Wire and Face: the OCCT objects behind the shapes.
#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS.hxx>

#include "internal.hpp"

namespace build123d {

opencascade::handle<BRepAdaptor_Curve> Edge::geom_adaptor() const {
    if (is_null()) throw std::invalid_argument("Can't find adaptor for empty edge");
    return new BRepAdaptor_Curve(TopoDS::Edge(obj_));
}

opencascade::handle<BRepAdaptor_CompCurve> Wire::geom_adaptor() const {
    if (is_null()) throw std::invalid_argument("Can't get geom adaptor of empty wire");
    return new BRepAdaptor_CompCurve(TopoDS::Wire(obj_));
}

opencascade::handle<Geom_Surface> Face::geom_adaptor() const {
    require_not_null("Can't get the surface of an empty face");
    return BRep_Tool::Surface(TopoDS::Face(obj_));
}

}  // namespace build123d
