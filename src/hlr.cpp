// Hidden-line removal: project_to_viewport (topology/one_d.py Mixin1D.project_to_viewport,
// shared by Mixin2D, Mixin3D and Compound).
#include <BRepLib.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>

#include "build123d/topology.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

void extract_edges(const TopoDS_Shape& compound, std::vector<TopoDS_Shape>& edges) {
    for (TopExp_Explorer explorer(compound, TopAbs_EDGE); explorer.More(); explorer.Next()) {
        edges.push_back(explorer.Current());
    }
}

std::pair<ShapeList<Edge>, ShapeList<Edge>> project_shape_to_viewport(const Shape& shape,
                                                                       const Vector& viewport_origin,
                                                                       const Vector& viewport_up,
                                                                       const std::optional<Vector>& look_at,
                                                                       const std::optional<double>& focus) {
    if (shape.is_null()) throw std::invalid_argument("Can't project empty edge/wire");
    return occt("project_to_viewport", [&] {
        // Setup the projector
        Handle(HLRBRep_Algo) hidden_line_removal = new HLRBRep_Algo();
        hidden_line_removal->Add(shape.wrapped());

        const Vector target = look_at ? *look_at : shape.center();
        const Vector projection_dir = (viewport_origin - target).normalized();
        const Vector up = viewport_up.normalized();
        gp_Ax2 camera_coordinate_system;
        camera_coordinate_system.SetAxis(gp_Ax1(viewport_origin.to_pnt(), projection_dir.to_dir()));
        camera_coordinate_system.SetYDirection(up.to_dir());
        // Python tests `if focus`, so a zero focal length is orthographic too
        const HLRAlgo_Projector projector = (focus && *focus != 0.0)
                                                ? HLRAlgo_Projector(camera_coordinate_system, *focus)
                                                : HLRAlgo_Projector(camera_coordinate_system);

        hidden_line_removal->Projector(projector);
        hidden_line_removal->Update();
        hidden_line_removal->Hide();

        HLRBRep_HLRToShape hlr_shapes(hidden_line_removal);

        std::vector<TopoDS_Shape> visible_edges;
        for (const TopoDS_Shape& edges :
             {hlr_shapes.VCompound(), hlr_shapes.Rg1LineVCompound(), hlr_shapes.OutLineVCompound()}) {
            if (!edges.IsNull()) extract_edges(edges, visible_edges);
        }
        std::vector<TopoDS_Shape> hidden_edges;
        for (const TopoDS_Shape& edges :
             {hlr_shapes.HCompound(), hlr_shapes.OutLineHCompound(), hlr_shapes.Rg1LineHCompound()}) {
            if (!edges.IsNull()) extract_edges(edges, hidden_edges);
        }

        // Fix the underlying geometry
        for (const auto& edge : visible_edges) BRepLib::BuildCurves3d(edge, TOLERANCE);
        for (const auto& edge : hidden_edges) BRepLib::BuildCurves3d(edge, TOLERANCE);

        std::pair<ShapeList<Edge>, ShapeList<Edge>> result;
        for (const auto& e : visible_edges) result.first.push_back(Edge(e));
        for (const auto& e : hidden_edges) result.second.push_back(Edge(e));
        return result;
    });
}

}  // namespace

std::pair<ShapeList<Edge>, ShapeList<Edge>> Mixin1D::project_to_viewport(const Vector& viewport_origin,
                                                                         const Vector& viewport_up,
                                                                         std::optional<Vector> look_at,
                                                                         std::optional<double> focus) const {
    return project_shape_to_viewport(*this, viewport_origin, viewport_up, look_at, focus);
}

std::pair<ShapeList<Edge>, ShapeList<Edge>> Mixin2D::project_to_viewport(const Vector& viewport_origin,
                                                                         const Vector& viewport_up,
                                                                         std::optional<Vector> look_at,
                                                                         std::optional<double> focus) const {
    return project_shape_to_viewport(*this, viewport_origin, viewport_up, look_at, focus);
}

std::pair<ShapeList<Edge>, ShapeList<Edge>> Mixin3D::project_to_viewport(const Vector& viewport_origin,
                                                                         const Vector& viewport_up,
                                                                         std::optional<Vector> look_at,
                                                                         std::optional<double> focus) const {
    return project_shape_to_viewport(*this, viewport_origin, viewport_up, look_at, focus);
}

}  // namespace build123d
