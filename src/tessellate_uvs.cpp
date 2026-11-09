// Private copies of Shape.tessellate_with_uvs, pack_utils._pack2d and Shape.to_splines, used by
// the exporters (topology/shape_core.py, pack_utils.py).
#include <algorithm>
#include <memory>

#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_Surface.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeCustom.hxx>
#include <ShapeCustom_RestrictionParameters.hxx>
#include <Standard_Failure.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <gp_Mat.hxx>

#include "internal.hpp"
#include "build123d/pack.hpp"
#include "io_internal.hpp"

namespace build123d::detail {

// pack2d: the shared port in pack.cpp (build123d/pack.hpp).

UvMesh tessellate_with_uvs(const Shape& shape, double tolerance, double angular_tolerance, bool atlas_packing,
                           double atlas_gutter) {
    if (shape.is_null()) throw std::invalid_argument("Cannot tessellate an empty shape");
    shape.mesh(tolerance, angular_tolerance);

    UvMesh mesh;
    struct UvRange {
        std::size_t start, count;
        double w, h;
    };
    std::vector<UvRange> face_uv_ranges;
    int offset = 0;

    for (const Face& face : shape.faces()) {
        const TopoDS_Face& topo_face = TopoDS::Face(face.wrapped());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) poly = BRep_Tool::Triangulation(topo_face, loc);
        if (poly.IsNull()) continue;
        const gp_Trsf trsf = loc.Transformation();
        const bool is_reversed = topo_face.Orientation() == TopAbs_REVERSED;
        const double reverse_factor = is_reversed ? -1.0 : 1.0;
        const int nb_nodes = poly->NbNodes();

        const gp_Mat mat = trsf.VectorialPart();
        const gp_XYZ tr = trsf.TranslationPart();
        auto rotate = [&mat](double x, double y, double z) {
            return Vector(x * mat.Value(1, 1) + y * mat.Value(1, 2) + z * mat.Value(1, 3),
                          x * mat.Value(2, 1) + y * mat.Value(2, 2) + z * mat.Value(2, 3),
                          x * mat.Value(3, 1) + y * mat.Value(3, 2) + z * mat.Value(3, 3));
        };

        // Vertices
        for (int i = 1; i <= nb_nodes; ++i) {
            const gp_Pnt p = poly->Node(i);
            const Vector r = rotate(p.X(), p.Y(), p.Z());
            mesh.vertices.emplace_back(r.X + tr.X(), r.Y + tr.Y(), r.Z + tr.Z());
        }

        // Normals
        if (!poly->HasNormals()) poly->ComputeNormals();
        for (int i = 1; i <= nb_nodes; ++i) {
            const gp_Dir d = poly->Normal(i);
            const Vector r = rotate(d.X(), d.Y(), d.Z());
            mesh.normals.emplace_back(r.X * reverse_factor, r.Y * reverse_factor, r.Z * reverse_factor);
        }

        // UV coordinates
        if (poly->HasUVNodes()) {
            std::vector<std::pair<double, double>> uv_raw;
            uv_raw.reserve(static_cast<std::size_t>(nb_nodes));
            for (int i = 1; i <= nb_nodes; ++i) {
                const gp_Pnt2d uv = poly->UVNode(i);
                uv_raw.emplace_back(uv.X(), uv.Y());
            }
            double u_min = uv_raw[0].first, v_min = uv_raw[0].second;
            double u_max = u_min, v_max = v_min;
            for (const auto& [u, v] : uv_raw) {
                u_min = std::min(u_min, u);
                u_max = std::max(u_max, u);
                v_min = std::min(v_min, v);
                v_max = std::max(v_max, v);
            }
            double u_range = u_max - u_min;
            double v_range = v_max - v_min;
            if (u_range < TOLERANCE) u_range = 1.0;
            if (v_range < TOLERANCE) v_range = 1.0;

            // Arc-lengths of isoparametric curves for the aspect ratio
            double arc_w = u_range, arc_h = v_range;
            try {
                const Handle(Geom_Surface) surface = BRep_Tool::Surface(topo_face);
                const double u_center = (u_min + u_max) * 0.5;
                const double v_center = (v_min + v_max) * 0.5;
                const GeomAdaptor_Curve u_adaptor(surface->VIso(v_center));
                const GeomAdaptor_Curve v_adaptor(surface->UIso(u_center));
                arc_w = GCPnts_AbscissaPoint::Length(u_adaptor, u_min, u_max, TOLERANCE);
                arc_h = GCPnts_AbscissaPoint::Length(v_adaptor, v_min, v_max, TOLERANCE);
            } catch (const Standard_Failure&) {
                // Degenerate/unsupported surface parameterization: keep the UV extents
            }
            if (arc_w < TOLERANCE) arc_w = 1.0;
            if (arc_h < TOLERANCE) arc_h = 1.0;

            face_uv_ranges.push_back({mesh.uvs.size(), static_cast<std::size_t>(nb_nodes), arc_w, arc_h});
            for (const auto& [u, v] : uv_raw) {
                double un = (u - u_min) / u_range;
                const double vn = (v - v_min) / v_range;
                if (is_reversed) un = 1.0 - un;
                mesh.uvs.emplace_back(un, vn);
            }
        } else {
            for (int i = 0; i < nb_nodes; ++i) mesh.uvs.emplace_back(0.0, 0.0);
            face_uv_ranges.push_back({mesh.uvs.size() - static_cast<std::size_t>(nb_nodes),
                                      static_cast<std::size_t>(nb_nodes), 1.0, 1.0});
        }

        // Triangles
        for (int i = 1; i <= poly->NbTriangles(); ++i) {
            int n1, n2, n3;
            poly->Triangle(i).Get(n1, n2, n3);
            if (is_reversed) std::swap(n1, n2);
            mesh.triangles.push_back({n1 + offset - 1, n2 + offset - 1, n3 + offset - 1});
        }
        offset += nb_nodes;
    }

    // Atlas packing: scale per-face UVs to world-space proportions and pack them into a single
    // [0, 1] texture atlas
    if (atlas_packing && !face_uv_ranges.empty()) {
        std::vector<std::pair<double, double>> sizes;
        for (const auto& r : face_uv_ranges) sizes.emplace_back(r.w, r.h);
        const auto positions = pack2d(sizes);
        double pack_w = -HUGE_VAL, pack_h = -HUGE_VAL;
        for (std::size_t i = 0; i < positions.size(); ++i) {
            pack_w = std::max(pack_w, positions[i].first + face_uv_ranges[i].w);
            pack_h = std::max(pack_h, positions[i].second + face_uv_ranges[i].h);
        }
        double scale = std::max(pack_w, pack_h);
        if (scale < TOLERANCE) scale = 1.0;
        const double inv_scale = 1.0 / scale;
        for (std::size_t i = 0; i < positions.size(); ++i) {
            const auto& info = face_uv_ranges[i];
            double bw = info.w * inv_scale;
            double bh = info.h * inv_scale;
            const double gutter_u = std::min(atlas_gutter, bw * 0.5);
            const double gutter_v = std::min(atlas_gutter, bh * 0.5);
            const double bx = positions[i].first * inv_scale + gutter_u;
            const double by = positions[i].second * inv_scale + gutter_v;
            bw -= 2.0 * gutter_u;
            bh -= 2.0 * gutter_v;
            for (std::size_t j = info.start; j < info.start + info.count; ++j) {
                auto& [u, v] = mesh.uvs[j];
                u = u * bw + bx;
                v = v * bh + by;
            }
        }
    }
    return mesh;
}

TopoDS_Shape to_splines(const TopoDS_Shape& shape, int degree, double tolerance, bool nurbs) {
    if (shape.IsNull()) throw std::invalid_argument("Cannot approximate an empty shape");
    Handle(ShapeCustom_RestrictionParameters) params = new ShapeCustom_RestrictionParameters();
    return ShapeCustom::BSplineRestriction(shape, tolerance, tolerance, degree, 1, GeomAbs_C0, GeomAbs_C0, true,
                                           !nurbs, params);
}

}  // namespace build123d::detail
