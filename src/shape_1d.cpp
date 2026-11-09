#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>

#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepLib.hxx>
#include <BRepLib_FindSurface.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepProj_Projection.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <ChFi2d_ChamferAPI.hxx>
#include <ChFi2d_FilletAPI.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeArcOfEllipse.hxx>
#include <GC_MakeArcOfHyperbola.hxx>
#include <GC_MakeArcOfParabola.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomFill_CorrectedFrenet.hxx>
#include <GeomFill_Frenet.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Wireframe.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Hypr.hxx>
#include <gp_Parab.hxx>
#include <gp_Pnt2d.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace detail {

std::vector<TopoDS_Wire> connect_edges_to_wires(const std::vector<TopoDS_Shape>& edges, double tol) {
    Handle(OccHSeqShape) edges_in = new OccHSeqShape();
    Handle(OccHSeqShape) wires_out = new OccHSeqShape();
    for (const auto& e : edges) {
        if (!e.IsNull()) edges_in->Append(e);
    }
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges_in, tol, false, wires_out);
    std::vector<TopoDS_Wire> out;
    for (int i = 1; i <= wires_out->Length(); ++i) out.push_back(TopoDS::Wire(wires_out->Value(i)));
    return out;
}

Wire as_wire(const Shape& shape) {
    if (shape.is_null()) throw std::invalid_argument("Can't make a wire from an empty shape");
    if (shape.wrapped().ShapeType() == TopAbs_WIRE) return Wire(shape.wrapped());
    if (shape.wrapped().ShapeType() == TopAbs_EDGE) return Wire(Edge(shape.wrapped()));
    return Wire(std::vector<Edge>(shape.edges()));
}

}  // namespace detail

namespace {

struct EdgeParam {
    TopoDS_Edge edge;
    double param;
    bool forward;
};

bool forward_of(const TopoDS_Shape& s) { return s.Orientation() == TopAbs_FORWARD; }

double edge_param_at(const TopoDS_Edge& edge, double position) {
    BRepAdaptor_Curve curve(edge);
    const double length = GCPnts_AbscissaPoint::Length(curve);
    return GCPnts_AbscissaPoint(curve, length * position, curve.FirstParameter()).Parameter();
}

EdgeParam edge_occt_param_at(const TopoDS_Edge& edge, double position, PositionMode mode) {
    BRepAdaptor_Curve curve(edge);
    const double length = GCPnts_AbscissaPoint::Length(curve);
    const bool forward = forward_of(edge);
    double value;
    if (mode == PositionMode::PARAMETER) {
        if (!forward) position = 1.0 - position;
        value = position;
    } else {
        const double self_length = linear_length(edge);
        if (!forward) position = self_length - position;
        value = position / self_length;
    }
    const double param =
        GCPnts_AbscissaPoint(curve, length * value, curve.FirstParameter()).Parameter();
    return {edge, param, forward};
}

std::vector<TopoDS_Edge> wire_edges(const TopoDS_Shape& wire) {
    std::vector<TopoDS_Edge> out;
    for (BRepTools_WireExplorer exp(TopoDS::Wire(wire)); exp.More(); exp.Next()) {
        out.push_back(exp.Current());
    }
    return out;
}

EdgeParam wire_occt_param_at(const TopoDS_Shape& wire, double position, PositionMode mode) {
    const bool w_forward = forward_of(wire);
    const double total_length = linear_length(wire);
    double distance;
    if (mode == PositionMode::PARAMETER) {
        if (!w_forward) position = 1.0 - position;
        distance = position * total_length;
    } else {
        if (!w_forward) position = total_length - position;
        distance = position;
    }
    const auto edges = wire_edges(wire);
    if (edges.empty()) throw std::invalid_argument("Wire has no edges");
    std::vector<double> lengths, cumulative;
    double total = 0.0;
    for (const auto& e : edges) {
        lengths.push_back(linear_length(e));
        total += lengths.back();
        cumulative.push_back(total);
    }
    std::size_t idx;
    double local_dist;
    if (distance <= 0.0) {
        idx = 0;
        local_dist = 0.0;
    } else if (distance >= total) {
        idx = edges.size() - 1;
        local_dist = lengths[idx];
    } else {
        idx = static_cast<std::size_t>(
            std::upper_bound(cumulative.begin(), cumulative.end(), distance) - cumulative.begin());
        const double prev = idx > 0 ? cumulative[idx - 1] : 0.0;
        local_dist = distance - prev;
    }
    const TopoDS_Edge& target = edges[idx];
    double local_frac = lengths[idx] == 0.0 ? 0.0 : local_dist / lengths[idx];
    if (!forward_of(target)) local_frac = 1.0 - local_frac;
    return {target, edge_param_at(target, local_frac), forward_of(target)};
}

EdgeParam occt_param_at(const TopoDS_Shape& s, double position, PositionMode mode) {
    if (s.IsNull()) throw std::invalid_argument("Empty edge or wire");
    if (s.ShapeType() == TopAbs_WIRE) return wire_occt_param_at(s, position, mode);
    if (s.ShapeType() == TopAbs_EDGE) return edge_occt_param_at(TopoDS::Edge(s), position, mode);
    throw std::invalid_argument("Expected an Edge or Wire");
}

/// Oriented normalized parameter of the point on the edge closest to `point`.
double edge_param_at_point(const Edge& edge, const Vector& pnt) {
    const TopoDS_Edge& e = TopoDS::Edge(edge.wrapped());
    BRepAdaptor_Curve adaptor(e);
    const double total = GCPnts_AbscissaPoint::Length(adaptor);
    double pmin, pmax;
    BRep_Tool::Range(e, pmin, pmax);
    auto oriented = [&](double u) { return forward_of(e) ? u : 1.0 - u; };

    // Method 1: the point is one of the edge's vertices
    double best = 1e300;
    TopoDS_Vertex nearest;
    for (TopExp_Explorer exp(e, TopAbs_VERTEX); exp.More(); exp.Next()) {
        const TopoDS_Vertex v = TopoDS::Vertex(exp.Current());
        const double d = BRep_Tool::Pnt(v).Distance(pnt.to_pnt());
        if (d < best) {
            best = d;
            nearest = v;
        }
    }
    if (!nearest.IsNull() && best <= TOLERANCE) {
        const double param = BRep_Tool::Parameter(nearest, e);
        return oriented(GCPnts_AbscissaPoint::Length(adaptor, pmin, param) / total);
    }
    const double separation = edge.distance_to(pnt);
    if (std::abs(separation) > TOLERANCE) {
        throw std::invalid_argument("point " + pnt.to_string() + " is " +
                                    std::to_string(separation) + " from edge");
    }
    // Method 2: project the point onto the curve
    double f, l;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(e, f, l);
    GeomAPI_ProjectPointOnCurve projector(pnt.to_pnt(), curve);
    if (projector.NbPoints() > 0) {
        double param = projector.LowerDistanceParameter();
        if (adaptor.IsPeriodic()) {
            param = pmin + std::fmod(std::fmod(param - pmin, adaptor.Period()) + adaptor.Period(),
                                     adaptor.Period());
        }
        const double u = oriented(GCPnts_AbscissaPoint::Length(adaptor, pmin, param) / total);
        if ((edge.position_at(u) - pnt).length() < TOLERANCE) return u;
    }
    // Method 3: bounded search for the closest point
    double best_u = 0.0, best_d = 1e300;
    const int samples = 200;
    for (int i = 0; i <= samples; ++i) {
        const double u = static_cast<double>(i) / samples;
        const double d = (edge.position_at(u) - pnt).length();
        if (d < best_d) {
            best_d = d;
            best_u = u;
        }
    }
    double lo = std::max(0.0, best_u - 1.0 / samples), hi = std::min(1.0, best_u + 1.0 / samples);
    const double gr = (std::sqrt(5.0) - 1) / 2;
    for (int it = 0; it < 100 && hi - lo > TOLERANCE / 2; ++it) {
        const double a = hi - gr * (hi - lo), b = lo + gr * (hi - lo);
        if ((edge.position_at(a) - pnt).length() < (edge.position_at(b) - pnt).length()) {
            hi = b;
        } else {
            lo = a;
        }
    }
    return py_round((lo + hi) / 2, 6);
}

Edge make_edge(const Handle(Geom_Curve)& curve) {
    return Edge(occt("Edge", [&] { return TopoDS_Shape(BRepBuilderAPI_MakeEdge(curve).Edge()); }));
}

TopoDS_Wire make_wire(const std::vector<TopoDS_Shape>& edges) {
    BRepBuilderAPI_MakeWire builder;
    builder.Add(to_list(edges));
    builder.Build();
    if (!builder.IsDone()) {
        switch (builder.Error()) {
            case BRepBuilderAPI_EmptyWire: throw std::runtime_error("Wire is empty");
            case BRepBuilderAPI_DisconnectedWire: throw std::invalid_argument("Edges are disconnected");
            case BRepBuilderAPI_NonManifoldWire:
                throw std::invalid_argument("Wire is non manifold (e.g. branching, self intersecting)");
            default: throw std::runtime_error("Unable to build wire");
        }
    }
    return builder.Wire();
}

}  // namespace

// ---------------------------------------------------------------------------
// Mixin1D
// ---------------------------------------------------------------------------

double Mixin1D::length() const {
    require_not_null("Can't find the length of an empty edge/wire");
    return linear_length(obj_);
}

double Mixin1D::radius() const {
    require_not_null("Can't find the radius of an empty edge/wire");
    if (obj_.ShapeType() == TopAbs_WIRE) return Edge(wire_edges(obj_).at(0)).radius();
    BRepAdaptor_Curve curve(TopoDS::Edge(obj_));
    if (curve.GetType() != GeomAbs_Circle) throw std::invalid_argument("Shape could not be reduced to a circle");
    return curve.Circle().Radius();
}

bool Mixin1D::is_closed() const {
    require_not_null("Can't determine if empty Edge or Wire is closed");
    return BRep_Tool::IsClosed(obj_);
}

bool Mixin1D::is_forward() const {
    require_not_null("Can't determine direction of empty Edge or Wire");
    return forward_of(obj_);
}

Vector Mixin1D::position_at(double position, PositionMode mode) const {
    const EdgeParam ep = occt_param_at(obj_, position, mode);
    return Vector(BRepAdaptor_Curve(ep.edge).Value(ep.param));
}

std::vector<Vector> Mixin1D::positions(const std::vector<double>& distances, PositionMode mode) const {
    std::vector<Vector> out;
    out.reserve(distances.size());
    for (double d : distances) out.push_back(position_at(d, mode));
    return out;
}

Vector Mixin1D::derivative_at(double position, int order, PositionMode mode) const {
    const EdgeParam ep = occt_param_at(obj_, position, mode);
    const gp_Vec d = BRepAdaptor_Curve(ep.edge).DN(ep.param, order);
    if (d.Magnitude() == 0.0) return Vector(0, 0, 0);
    Vector result(d);
    if (order % 2 == 1) {
        if (obj_.ShapeType() == TopAbs_WIRE) {
            if (ep.forward != is_forward()) result = -result;
        } else if (!is_forward()) {
            result = -result;
        }
    }
    return result;
}

Vector Mixin1D::derivative_at(const Vector& point, int order) const {
    Edge closest;
    if (obj_.ShapeType() == TopAbs_WIRE) {
        double best = 1e300;
        for (const auto& e : wire_edges(obj_)) {
            const double d = Edge(e).distance_to(point);
            if (d < best) {
                best = d;
                closest = Edge(e);
            }
        }
    } else {
        closest = Edge(obj_);
    }
    const double u = edge_param_at_point(closest, point);
    const EdgeParam ep = edge_occt_param_at(TopoDS::Edge(closest.wrapped()), u, PositionMode::PARAMETER);
    const gp_Vec d = BRepAdaptor_Curve(ep.edge).DN(ep.param, order);
    if (d.Magnitude() == 0.0) return Vector(0, 0, 0);
    Vector result(d);
    if (order % 2 == 1) {
        if (obj_.ShapeType() == TopAbs_WIRE) {
            if (ep.forward != is_forward()) result = -result;
        } else if (!is_forward()) {
            result = -result;
        }
    }
    return result;
}

Vector Mixin1D::tangent_at(double position, PositionMode mode) const {
    return derivative_at(position, 1, mode).normalized();
}

Vector Mixin1D::tangent_at(const Vector& point) const {
    return derivative_at(point, 1).normalized();
}

double Mixin1D::tangent_angle_at(double location_param, PositionMode mode, const Plane& plane) const {
    const Vector tan = tangent_at(location_param, mode);
    return std::fmod(plane.x_dir.get_signed_angle(tan, plane.z_dir) + 360.0, 360.0);
}

Location Mixin1D::location_at(double distance, PositionMode mode, FrameMethod frame_method,
                              std::optional<Vector> x_dir) const {
    require_not_null("Can't find a location on an empty edge/wire");
    const bool forward = is_forward();
    const double self_length = length();
    if (!forward) distance = mode == PositionMode::PARAMETER ? 1.0 - distance : self_length - distance;

    Handle(Adaptor3d_Curve) curve;
    double param;
    if (obj_.ShapeType() == TopAbs_WIRE) {
        if (frame_method == FrameMethod::CORRECTED) {
            const double u = mode == PositionMode::PARAMETER ? distance : distance / self_length;
            Handle(BRepAdaptor_CompCurve) comp = new BRepAdaptor_CompCurve(TopoDS::Wire(obj_));
            const Vector target = position_at(u);
            Extrema_ExtPC extrema(target.to_pnt(), *comp);
            if (!extrema.IsDone() || extrema.NbExt() == 0) throw std::runtime_error("Failed to find point on curve");
            double best = 1e300;
            param = comp->FirstParameter();
            for (int i = 1; i <= extrema.NbExt(); ++i) {
                if (extrema.SquareDistance(i) < best) {
                    best = extrema.SquareDistance(i);
                    param = extrema.Point(i).Parameter();
                }
            }
            curve = comp;
        } else {
            const double len_pos = mode == PositionMode::PARAMETER ? distance * self_length : distance;
            const EdgeParam ep = wire_occt_param_at(obj_, len_pos, PositionMode::LENGTH);
            curve = new BRepAdaptor_Curve(ep.edge);
            param = ep.param;
        }
    } else {
        const TopoDS_Edge& e = TopoDS::Edge(obj_);
        param = edge_param_at(e, mode == PositionMode::PARAMETER ? distance : distance / self_length);
        curve = new BRepAdaptor_Curve(e);
    }

    Handle(GeomFill_TrihedronLaw) law;
    if (frame_method == FrameMethod::FRENET) {
        law = new GeomFill_Frenet();
    } else {
        law = new GeomFill_CorrectedFrenet();
    }
    law->SetCurve(curve);
    gp_Vec tangent, normal, binormal;
    law->D0(param, tangent, normal, binormal);
    const gp_Pnt pnt = curve->Value(param);

    gp_Trsf trsf;
    if (x_dir) {
        try {
            trsf.SetTransformation(gp_Ax3(pnt, gp_Dir(tangent.XYZ()), x_dir->to_dir()), gp_Ax3());
        } catch (const Standard_Failure&) {
            throw std::invalid_argument("Unable to create location with given x_dir " +
                                        x_dir->to_string() +
                                        ". x_dir must be perpendicular to shape's tangent " +
                                        Vector(tangent).to_string() + ".");
        }
    } else {
        trsf.SetTransformation(gp_Ax3(pnt, gp_Dir(tangent.XYZ()), gp_Dir(normal.XYZ())), gp_Ax3());
    }
    const Location loc(trsf);
    return forward ? loc : -loc;
}

std::vector<Location> Mixin1D::locations(const std::vector<double>& distances, PositionMode mode,
                                         FrameMethod frame_method, std::optional<Vector> x_dir) const {
    std::vector<Location> out;
    for (double d : distances) out.push_back(location_at(d, mode, frame_method, x_dir));
    return out;
}

Vector Mixin1D::start_point() const {
    require_not_null("Empty edge/wire");
    if (obj_.ShapeType() == TopAbs_WIRE) {
        BRepAdaptor_CompCurve c(TopoDS::Wire(obj_));
        return Vector(c.Value(is_forward() ? c.FirstParameter() : c.LastParameter()));
    }
    BRepAdaptor_Curve c(TopoDS::Edge(obj_));
    return Vector(c.Value(is_forward() ? c.FirstParameter() : c.LastParameter()));
}

Vector Mixin1D::end_point() const {
    require_not_null("Empty edge/wire");
    if (obj_.ShapeType() == TopAbs_WIRE) {
        BRepAdaptor_CompCurve c(TopoDS::Wire(obj_));
        return Vector(c.Value(is_forward() ? c.LastParameter() : c.FirstParameter()));
    }
    BRepAdaptor_Curve c(TopoDS::Edge(obj_));
    return Vector(c.Value(is_forward() ? c.LastParameter() : c.FirstParameter()));
}

double Mixin1D::param_at_point(const Vector& point) const {
    require_not_null("Can't find a parameter on an empty edge/wire");
    if (obj_.ShapeType() == TopAbs_EDGE) return edge_param_at_point(Edge(obj_), point);

    const double separation = distance_to(point);
    if (std::abs(separation) > TOLERANCE) {
        throw std::invalid_argument("point " + point.to_string() + " is " +
                                    std::to_string(separation) + " from wire");
    }
    BRepExtrema_DistShapeShape extrema(Vertex(point).wrapped(), obj_);
    extrema.Perform();
    if (!extrema.IsDone() || extrema.NbSolution() == 0) throw std::invalid_argument("point is not on Wire");
    TopoDS_Edge closest;
    double param = 0.0;
    if (extrema.SupportTypeShape2(1) == BRepExtrema_IsOnEdge) {
        closest = TopoDS::Edge(extrema.SupportOnShape2(1));
        extrema.ParOnEdgeS2(1, param);
    } else {
        const TopoDS_Vertex hit = TopoDS::Vertex(extrema.SupportOnShape2(1));
        OccShapeListMap map;
        TopExp::MapShapesAndAncestors(obj_, TopAbs_VERTEX, TopAbs_EDGE, map);
        closest = TopoDS::Edge(map.FindFromKey(hit).First());
        param = BRep_Tool::Parameter(hit, closest);
    }
    BRepAdaptor_Curve adaptor(closest);
    double pmin, pmax;
    BRep_Tool::Range(closest, pmin, pmax);
    if (adaptor.IsPeriodic()) {
        param = std::fmod(std::fmod(param - pmin, adaptor.Period()) + adaptor.Period(), adaptor.Period()) + pmin;
    }
    double along = forward_of(closest) ? GCPnts_AbscissaPoint::Length(adaptor, pmin, param)
                                       : GCPnts_AbscissaPoint::Length(adaptor, param, pmax);
    for (BRepTools_WireExplorer exp(TopoDS::Wire(obj_)); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = exp.Current();
        if (BRep_Tool::Degenerated(e)) continue;
        if (e.IsEqual(closest)) break;
        along += GCPnts_AbscissaPoint::Length(BRepAdaptor_Curve(e));
    }
    return along / length();
}

Vector Mixin1D::normal() const {
    require_not_null("Empty edge/wire");
    if (obj_.ShapeType() == TopAbs_EDGE) {
        BRepAdaptor_Curve c(TopoDS::Edge(obj_));
        if (c.GetType() == GeomAbs_Circle) return Vector(c.Circle().Axis().Direction());
        if (c.GetType() == GeomAbs_Ellipse) return Vector(c.Ellipse().Axis().Direction());
    }
    BRepLib_FindSurface finder(obj_, -1, true);
    if (finder.Found()) {
        Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(finder.Surface());
        if (!plane.IsNull()) {
            gp_Pln pln = plane->Pln();
            if (!finder.Location().IsIdentity()) pln.Transform(finder.Location().Transformation());
            return Vector(pln.Axis().Direction());
        }
    }
    throw std::invalid_argument("Normal not defined");
}

std::optional<Plane> Mixin1D::common_plane(const std::vector<Shape>& lines, double tolerance) const {
    std::vector<Mixin1D> all{*this};
    for (const auto& l : lines) {
        if (l.is_null()) continue;
        const auto t = l.wrapped().ShapeType();
        if (t != TopAbs_EDGE && t != TopAbs_WIRE) throw std::invalid_argument("Only Edges or Wires are valid");
        all.emplace_back(l.wrapped());
    }
    std::vector<Edge> all_edges;
    for (const auto& l : all) {
        for (const auto& e : l.edges()) all_edges.push_back(e);
    }
    std::optional<Plane> result;
    bool all_lines = !all_edges.empty();
    for (const auto& e : all_edges) all_lines = all_lines && e.geom_type() == GeomType::LINE;
    if (all_lines) {
        std::vector<Axis> axes;
        for (const auto& e : all_edges) axes.emplace_back(e.position_at(0), e.tangent_at(0.0));
        bool coaxial = true;
        for (std::size_t i = 0; i < axes.size() && coaxial; ++i) {
            for (std::size_t j = i + 1; j < axes.size() && coaxial; ++j) {
                coaxial = axes[i].is_coaxial(axes[j]);
            }
        }
        if (coaxial) {
            const Vector z = Plane(axes[0]).x_dir;
            result = Plane(axes[0].position, std::nullopt, z);
        }
    }
    if (!result) {
        std::vector<Vector> points;
        for (const auto& line : all) {
            Mixin1D l = line;
            if (l.length() > 1e50) l = Edge(l.wrapped()).trim_to_length(0.5, 10);
            const int n = (l.wrapped().ShapeType() == TopAbs_EDGE && l.geom_type() == GeomType::LINE) ? 2 : 8;
            for (int i = 0; i < n; ++i) {
                const Vector p = l.position_at(static_cast<double>(i) / (n - 1));
                if (std::none_of(points.begin(), points.end(), [&](const Vector& q) { return q == p; })) {
                    points.push_back(p);
                }
            }
        }
        if (points.size() < 3) return std::nullopt;
        double best_area = -1.0;
        std::array<Vector, 3> extremes;
        for (std::size_t i = 0; i < points.size(); ++i) {
            for (std::size_t j = i + 1; j < points.size(); ++j) {
                for (std::size_t k = j + 1; k < points.size(); ++k) {
                    const double area =
                        0.5 * (points[j] - points[i]).cross(points[k] - points[i]).length();
                    if (area >= best_area) {
                        best_area = area;
                        extremes = {points[i], points[j], points[k]};
                    }
                }
            }
        }
        const Vector x = (extremes[1] - extremes[0]).normalized();
        const Vector z = (extremes[2] - extremes[0]).cross(x);
        if (z.length() <= gp::Resolution()) return std::nullopt;
        const Plane c_plane((extremes[0] + extremes[1] + extremes[2]) / 3.0, std::nullopt, z);
        for (const auto& p : points) {
            if (!c_plane.contains(p, tolerance)) return std::nullopt;
        }
        result = c_plane;
    }
    Vector global_center(0, 0, 0);
    for (const auto& l : all) global_center += l.position_at(0.5);
    global_center = global_center / static_cast<double>(all.size());
    const auto origin = result->intersect(Axis(global_center, result->z_dir));
    if (!origin || !std::holds_alternative<Vector>(*origin)) return result;
    return result->shift_origin(std::get<Vector>(*origin));
}

Wire Mixin1D::offset_2d(double distance, Kind kind, Side side, bool closed) const {
    require_not_null("Can't offset an empty edge/wire");
    const Wire line = obj_.ShapeType() == TopAbs_WIRE ? Wire(obj_) : Wire(Edge(obj_));
    TopoDS_Wire topods_wire;
    const auto line_edges = line.edges();
    if (line_edges.size() == 1) {
        const Edge& e = line_edges[0];
        topods_wire = TopoDS::Wire(Wire(std::vector<Edge>{e.trim(0.0, 0.5), e.trim(0.5, 1.0)}).wrapped());
    } else {
        topods_wire = TopoDS::Wire(line.wrapped());
    }
    Wire offset_wire = occt("offset_2d", [&] {
        BRepOffsetAPI_MakeOffset builder;
        builder.Init(to_occ(kind));
        builder.AddWire(topods_wire);
        builder.Perform(distance);
        if (!builder.IsDone()) throw std::runtime_error("2D offset failed with distance " + std::to_string(distance));
        TopoDS_Shape obj = unwrap(builder.Shape(), true);
        if (obj.ShapeType() != TopAbs_WIRE) throw std::runtime_error("Unexpected result type");
        return Wire(obj);
    });

    if (side != Side::BOTH) {
        const Vector p0 = line.position_at(0), p1 = line.position_at(1);
        std::vector<Edge> kept;
        for (const auto& e : offset_wire.edges()) {
            bool is_end_arc = false;
            if (e.geom_type() == GeomType::CIRCLE) {
                const Vector c = e.arc_center();
                is_end_arc = (c - p0).length() < TOLERANCE || (c - p1).length() < TOLERANCE;
            }
            if (!is_end_arc) kept.push_back(e);
        }
        std::vector<TopoDS_Shape> kept_topods;
        for (const auto& e : kept) kept_topods.push_back(e.wrapped());
        const auto wires = connect_edges_to_wires(kept_topods, 1e-6);
        if (wires.size() < 2) throw std::runtime_error("offset_2d: unable to separate the offset sides");
        std::vector<double> angles;
        for (const auto& w : wires) {
            const Vector c = Wire(w).position_at(0.5);
            angles.push_back(line.tangent_at(0.0).get_signed_angle(c - line.position_at(0)));
        }
        if (side == Side::LEFT) {
            offset_wire = Wire(wires[angles[0] > angles[1] ? 1 : 0]);
        } else {
            offset_wire = Wire(wires[angles[0] <= angles[1] ? 1 : 0]);
        }
        if (closed) {
            const Vector self0 = line.position_at(0), self1 = line.position_at(1);
            const Vector end0 = offset_wire.position_at(0), end1 = offset_wire.position_at(1);
            Edge edge0, edge1;
            if ((self0 - end0).length() - std::abs(distance) <= TOLERANCE) {
                edge0 = Edge::make_line(self0, end0);
                edge1 = Edge::make_line(self1, end1);
            } else {
                edge0 = Edge::make_line(self0, end1);
                edge1 = Edge::make_line(self1, end0);
            }
            std::vector<Edge> all_edges(line.edges());
            for (const auto& e : offset_wire.edges()) all_edges.push_back(e);
            all_edges.push_back(edge0);
            all_edges.push_back(edge1);
            offset_wire = Wire(all_edges);
        }
    }
    return offset_wire;
}

Edge Mixin1D::perpendicular_line(double length, double u_value, const Plane& plane) const {
    const Vector start = position_at(u_value);
    const Plane local(start, tangent_at(u_value), plane.z_dir);
    return Edge::make_line(start + local.y_dir * (length / 2), start - local.y_dir * (length / 2));
}

ShapeList<Edge> Mixin1D::project_to_shape(const Shape& target, const Vector& direction) const {
    require_not_null("Can't project an empty edge/wire");
    const Wire w = obj_.ShapeType() == TopAbs_WIRE ? Wire(obj_) : Wire(Edge(obj_));
    ShapeList<Edge> out;
    occt("project_to_shape", [&] {
        BRepProj_Projection proj(w.wrapped(), target.wrapped(), direction.to_dir());
        for (; proj.More(); proj.Next()) {
            for (const auto& e : entities(proj.Current(), TopAbs_EDGE)) out.push_back(Edge(e));
        }
        return 0;
    });
    return out;
}

Axis::Axis(const Edge& edge) {
    if (edge.is_null()) throw std::invalid_argument("Invalid edge argument");
    const TopoDS_Edge& e = TopoDS::Edge(edge.wrapped());
    double first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(e, first, last);
    if (curve.IsNull()) throw std::invalid_argument("Edge has no 3D curve");
    gp_Pnt origin;
    gp_Vec tangent;
    curve->D1(first, origin, tangent);
    position = Vector(origin);
    direction = Vector(gp_Dir(tangent));
}

Vector operator%(const Mixin1D& curve, double position) { return curve.tangent_at(position); }
Location operator^(const Mixin1D& curve, double position) { return curve.location_at(position); }

// ---------------------------------------------------------------------------
// Edge
// ---------------------------------------------------------------------------

Edge::Edge(const TopoDS_Shape& obj) : Mixin1D(obj) {
    if (!obj.IsNull() && obj.ShapeType() != TopAbs_EDGE) throw std::invalid_argument("Edge requires a TopoDS_Edge");
}

Edge::Edge(const Axis& axis)
    : Edge(TopoDS_Shape(BRepBuilderAPI_MakeEdge(Handle(Geom_Curve)(new Geom_Line(axis.wrapped()))).Edge())) {}

Edge Edge::make_line(const Vector& p1, const Vector& p2) {
    return Edge(occt("Edge.make_line",
                     [&] { return TopoDS_Shape(BRepBuilderAPI_MakeEdge(p1.to_pnt(), p2.to_pnt()).Edge()); }));
}

Edge Edge::make_circle(double radius, const Plane& plane, double start_angle, double end_angle,
                       AngularDirection angular_direction) {
    const gp_Circ circle(plane.to_gp_ax2(), radius);
    if (start_angle == end_angle) {
        return Edge(occt("Edge.make_circle", [&] { return TopoDS_Shape(BRepBuilderAPI_MakeEdge(circle).Edge()); }));
    }
    const bool ccw = angular_direction == AngularDirection::COUNTER_CLOCKWISE;
    const double start = (ccw ? start_angle : end_angle) * DEG2RAD;
    const double end = (ccw ? end_angle : start_angle) * DEG2RAD;
    return make_edge(occt("Edge.make_circle", [&] { return Handle(Geom_Curve)(GC_MakeArcOfCircle(circle, start, end, ccw).Value()); }));
}

Edge Edge::make_ellipse(double x_radius, double y_radius, const Plane& plane, double start_angle,
                        double end_angle, AngularDirection angular_direction) {
    const gp_Ax1 ax1(plane.origin.to_pnt(), plane.z_dir.to_dir());
    double correction = 0.0;
    gp_Elips ellipse;
    if (y_radius > x_radius) {
        correction = 90.0 * DEG2RAD;
        ellipse = gp_Elips(plane.to_gp_ax2(), y_radius, x_radius).Rotated(ax1, correction);
    } else {
        ellipse = gp_Elips(plane.to_gp_ax2(), x_radius, y_radius);
    }
    if (start_angle == end_angle) {
        return Edge(occt("Edge.make_ellipse", [&] { return TopoDS_Shape(BRepBuilderAPI_MakeEdge(ellipse).Edge()); }));
    }
    double a1 = start_angle * DEG2RAD - correction;
    double a2 = end_angle * DEG2RAD - correction;
    bool sense;
    double alpha1, alpha2;
    if (angular_direction == AngularDirection::COUNTER_CLOCKWISE) {
        if (a2 <= a1) a2 += 2 * PI;
        sense = true;
        alpha1 = a1;
        alpha2 = a2;
    } else {
        if (a2 >= a1) a2 -= 2 * PI;
        sense = false;
        alpha1 = a2;
        alpha2 = a1;
    }
    return make_edge(occt("Edge.make_ellipse", [&] {
        return Handle(Geom_Curve)(GC_MakeArcOfEllipse(ellipse, alpha1, alpha2, sense).Value());
    }));
}

Edge Edge::make_helix(double pitch, double height, double radius, const Vector& center,
                      const Vector& normal, double angle, bool lefthand) {
    return Edge(occt("Edge.make_helix", [&] {
        const gp_Ax3 ax3(center.to_pnt(), normal.to_dir());
        Handle(Geom_Surface) surface;
        if (angle == 0.0) {
            surface = new Geom_CylindricalSurface(ax3, radius);
        } else {
            surface = new Geom_ConicalSurface(ax3, angle * DEG2RAD, radius);
        }
        const double sign = lefthand ? -1.0 : 1.0;
        const Vector line_dir = Vector(sign * 2 * PI, pitch).normalized();
        const double line_len = (height / line_dir.Y) / std::cos(angle * DEG2RAD);
        Handle(Geom2d_Line) helix_line = new Geom2d_Line(gp_Pnt2d(0, 0), gp_Dir2d(line_dir.X, line_dir.Y));
        Handle(Geom2d_TrimmedCurve) helix_curve = new Geom2d_TrimmedCurve(helix_line, 0, line_len, true, true);
        BRepBuilderAPI_MakeEdge builder(helix_curve, surface);
        TopoDS_Edge edge = builder.Edge();
        BRepLib::BuildCurves3d(edge, 1e-9, GeomAbs_C1, 14, 2000);
        return TopoDS_Shape(edge);
    }));
}

Edge Edge::make_spline(const std::vector<Vector>& points, const std::vector<Vector>& tangents,
                       bool periodic, const std::vector<double>& parameters, bool scale, double tol) {
    if (points.size() < 2) throw std::invalid_argument("A spline needs at least two points");
    Handle(OccHArrayPnt) pnts = new OccHArrayPnt(1, static_cast<int>(points.size()));
    for (std::size_t i = 0; i < points.size(); ++i) pnts->SetValue(static_cast<int>(i) + 1, points[i].to_pnt());
    return occt("Edge.make_spline", [&] {
        std::unique_ptr<GeomAPI_Interpolate> builder;
        if (parameters.empty()) {
            builder = std::make_unique<GeomAPI_Interpolate>(pnts, periodic, tol);
        } else {
            if (parameters.size() != points.size() + (periodic ? 1 : 0)) {
                throw std::invalid_argument(
                    "There must be one parameter for each interpolation point (plus one if periodic), or none specified");
            }
            Handle(OccHArrayReal) params = new OccHArrayReal(1, static_cast<int>(parameters.size()));
            for (std::size_t i = 0; i < parameters.size(); ++i) params->SetValue(static_cast<int>(i) + 1, parameters[i]);
            builder = std::make_unique<GeomAPI_Interpolate>(pnts, params, periodic, tol);
        }
        if (!tangents.empty()) {
            if (tangents.size() == 2 && points.size() != 2) {
                builder->Load(tangents[0].to_vec(), tangents[1].to_vec(), scale);
            } else {
                if (tangents.size() != points.size()) {
                    throw std::invalid_argument(
                        "There must be one tangent for each interpolation point, or just two end point tangents");
                }
                OccArrayVec tans(1, static_cast<int>(tangents.size()));
                Handle(OccHArrayBool) flags = new OccHArrayBool(1, static_cast<int>(tangents.size()));
                for (std::size_t i = 0; i < tangents.size(); ++i) {
                    tans.SetValue(static_cast<int>(i) + 1, tangents[i].to_vec());
                    flags->SetValue(static_cast<int>(i) + 1, true);
                }
                builder->Load(tans, flags, scale);
            }
        }
        builder->Perform();
        if (!builder->IsDone()) throw std::invalid_argument("B-spline interpolation failed");
        return make_edge(builder->Curve());
    });
}

Edge Edge::make_spline_approx(const std::vector<Vector>& points, double tol,
                              std::optional<std::array<double, 3>> smoothing, int min_deg, int max_deg) {
    OccArrayPnt pnts(1, static_cast<int>(points.size()));
    for (std::size_t i = 0; i < points.size(); ++i) pnts.SetValue(static_cast<int>(i) + 1, points[i].to_pnt());
    return occt("Edge.make_spline_approx", [&] {
        std::unique_ptr<GeomAPI_PointsToBSpline> builder;
        if (smoothing) {
            builder = std::make_unique<GeomAPI_PointsToBSpline>(pnts, (*smoothing)[0], (*smoothing)[1], (*smoothing)[2],
                                                                std::max(max_deg, 6), GeomAbs_C2, tol);
        } else {
            builder = std::make_unique<GeomAPI_PointsToBSpline>(pnts, min_deg, max_deg, GeomAbs_C2, tol);
        }
        if (!builder->IsDone()) throw std::invalid_argument("B-spline approximation failed");
        return make_edge(builder->Curve());
    });
}

Edge Edge::make_bezier(const std::vector<Vector>& points, const std::vector<double>& weights) {
    if (points.size() < 2) throw std::invalid_argument("At least two control points must be provided (start, end)");
    if (points.size() > 25) throw std::invalid_argument("The maximum number of control points is 25");
    if (!weights.empty() && weights.size() != points.size()) {
        throw std::invalid_argument("A weight must be provided for each control point");
    }
    OccArrayPnt poles(1, static_cast<int>(points.size()));
    for (std::size_t i = 0; i < points.size(); ++i) poles.SetValue(static_cast<int>(i) + 1, points[i].to_pnt());
    return occt("Edge.make_bezier", [&] {
        Handle(Geom_BezierCurve) curve;
        if (!weights.empty()) {
            OccArrayReal w(1, static_cast<int>(weights.size()));
            for (std::size_t i = 0; i < weights.size(); ++i) w.SetValue(static_cast<int>(i) + 1, weights[i]);
            curve = new Geom_BezierCurve(poles, w);
        } else {
            curve = new Geom_BezierCurve(poles);
        }
        return make_edge(curve);
    });
}

Edge Edge::make_bspline(const std::vector<Vector>& control_points, const std::vector<double>& knots, int degree,
                        const std::vector<double>& weights, bool periodic) {
    if (knots.empty()) throw std::invalid_argument("B-spline requires at least one knot");
    std::vector<double> unique_knots{knots[0]};
    std::vector<int> multiplicities{1};
    for (std::size_t i = 1; i < knots.size(); ++i) {
        if (std::abs(knots[i] - unique_knots.back()) <= TOLERANCE) {
            ++multiplicities.back();
        } else {
            unique_knots.push_back(knots[i]);
            multiplicities.push_back(1);
        }
    }
    return occt("Edge.make_bspline", [&] {
        OccArrayPnt poles(1, static_cast<int>(control_points.size()));
        for (std::size_t i = 0; i < control_points.size(); ++i) {
            poles.SetValue(static_cast<int>(i) + 1, control_points[i].to_pnt());
        }
        OccArrayReal knot_array(1, static_cast<int>(unique_knots.size()));
        OccArrayInt multiplicity_array(1, static_cast<int>(multiplicities.size()));
        for (std::size_t i = 0; i < unique_knots.size(); ++i) {
            knot_array.SetValue(static_cast<int>(i) + 1, unique_knots[i]);
            multiplicity_array.SetValue(static_cast<int>(i) + 1, multiplicities[i]);
        }
        Handle(Geom_BSplineCurve) curve;
        if (!weights.empty()) {
            OccArrayReal weight_array(1, static_cast<int>(weights.size()));
            for (std::size_t i = 0; i < weights.size(); ++i) weight_array.SetValue(static_cast<int>(i) + 1, weights[i]);
            curve = new Geom_BSplineCurve(poles, weight_array, knot_array, multiplicity_array, degree, periodic);
        } else {
            curve = new Geom_BSplineCurve(poles, knot_array, multiplicity_array, degree, periodic);
        }
        return make_edge(curve);
    });
}

Edge Edge::make_parabola(double focal_length, const Plane& plane, double start_angle, double end_angle,
                         AngularDirection angular_direction) {
    return make_edge(occt("Edge.make_parabola", [&] {
        const gp_Parab parabola(plane.to_gp_ax2(), focal_length);
        return Handle(Geom_Curve)(GC_MakeArcOfParabola(parabola, start_angle * DEG2RAD, end_angle * DEG2RAD,
                                                       angular_direction == AngularDirection::COUNTER_CLOCKWISE)
                                      .Value());
    }));
}

Edge Edge::make_hyperbola(double x_radius, double y_radius, const Plane& plane, double start_angle,
                          double end_angle, AngularDirection angular_direction) {
    return occt("Edge.make_hyperbola", [&] {
        const gp_Ax1 ax1(plane.origin.to_pnt(), plane.z_dir.to_dir());
        double correction = 0.0;
        gp_Hypr hyperbola;
        if (y_radius > x_radius) {
            correction = 90.0 * DEG2RAD;
            hyperbola = gp_Hypr(plane.to_gp_ax2(), y_radius, x_radius).Rotated(ax1, correction);
        } else {
            hyperbola = gp_Hypr(plane.to_gp_ax2(), x_radius, y_radius);
        }
        if (start_angle == end_angle) return Edge(TopoDS_Shape(BRepBuilderAPI_MakeEdge(hyperbola).Edge()));
        return make_edge(GC_MakeArcOfHyperbola(hyperbola, start_angle * DEG2RAD - correction,
                                               end_angle * DEG2RAD - correction,
                                               angular_direction == AngularDirection::COUNTER_CLOCKWISE)
                             .Value());
    });
}

Edge Edge::make_three_point_arc(const Vector& p1, const Vector& p2, const Vector& p3) {
    return make_edge(occt("Edge.make_three_point_arc", [&] {
        return Handle(Geom_Curve)(GC_MakeArcOfCircle(p1.to_pnt(), p2.to_pnt(), p3.to_pnt()).Value());
    }));
}

Edge Edge::make_tangent_arc(const Vector& start, const Vector& tangent, const Vector& end) {
    return make_edge(occt("Edge.make_tangent_arc", [&] {
        return Handle(Geom_Curve)(GC_MakeArcOfCircle(start.to_pnt(), tangent.to_vec(), end.to_pnt()).Value());
    }));
}

Edge Edge::make_mid_way(const Edge& first, const Edge& second, double middle) {
    const bool flip = Axis(first).is_opposite(Axis(second));
    std::array<Vector, 2> pnts;
    for (int i = 0; i < 2; ++i) {
        pnts[static_cast<std::size_t>(i)] =
            Edge::make_line(first.position_at(i), second.position_at(flip ? 1 - i : i)).position_at(middle);
    }
    return Edge::make_line(pnts[0], pnts[1]);
}

Edge Edge::extrude(const Vertex& obj, const Vector& direction) {
    return Edge(extrude_shape(obj.wrapped(), direction));
}

Vector Edge::arc_center() const {
    BRepAdaptor_Curve c(TopoDS::Edge(obj_));
    if (c.GetType() == GeomAbs_Circle) return Vector(c.Circle().Position().Location());
    if (c.GetType() == GeomAbs_Ellipse) return Vector(c.Ellipse().Position().Location());
    throw std::invalid_argument(build123d::to_string(geom_type()) + " has no arc center");
}

Edge Edge::reversed(bool reconstruct) const {
    require_not_null("An empty edge can't be reversed");
    if (!reconstruct) return derive<Edge>(obj_.Reversed());
    const double first = param_at(0), last = param_at(1);
    double f, l;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(obj_), f, l);
    const double rf = curve->ReversedParameter(first), rl = curve->ReversedParameter(last);
    return derive<Edge>(occt("Edge.reversed", [&] {
        return TopoDS_Shape(BRepBuilderAPI_MakeEdge(curve->Reversed(), rl, rf).Edge());
    }));
}

Edge Edge::trim(double start, double end) const {
    require_not_null("Can't trim an empty edge");
    const Vector start_point = position_at(start);
    const EdgeParam ps = edge_occt_param_at(TopoDS::Edge(obj_), start, PositionMode::PARAMETER);
    const EdgeParam pe = edge_occt_param_at(TopoDS::Edge(obj_), end, PositionMode::PARAMETER);
    double first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(obj_), first, last);
    const double lo = std::min(ps.param, pe.param), hi = std::max(ps.param, pe.param);
    Edge trimmed(occt("Edge.trim", [&] { return TopoDS_Shape(BRepBuilderAPI_MakeEdge(curve, lo, hi).Edge()); }));
    const bool same_start = (trimmed.position_at(0) - start_point).length() < TOLERANCE;
    const bool same_dir = tangent_at(start).dot(trimmed.tangent_at(0.0)) > 1 - TOLERANCE;
    if (same_start && same_dir) return trimmed;
    return trimmed.reversed(true);
}

Edge Edge::trim(const Vector& start, const Vector& end) const {
    return trim(param_at_point(start), param_at_point(end));
}

Edge Edge::trim_to_length(double start, double length) const {
    require_not_null("Can't trim empty edge");
    double first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(obj_), first, last);
    GeomAdaptor_Curve adaptor(curve);
    const double parm_start = param_at(start);
    const double parm_end = GCPnts_AbscissaPoint(adaptor, length, parm_start).Parameter();
    Handle(Geom_TrimmedCurve) trimmed = new Geom_TrimmedCurve(curve, parm_start, parm_end);
    return make_edge(trimmed);
}

double Edge::param_at(double position) const {
    require_not_null("Empty edge");
    return edge_param_at(TopoDS::Edge(obj_), position);
}

Wire Edge::close() const { return Wire(*this).close(); }

bool Edge::is_infinite() const { return geom_type() == GeomType::LINE && length() > 1e100; }

bool Edge::geom_equal(const Edge& other, double tol, int num_interpolation_points) const {
    if (other.is_null() || is_null()) return false;
    const GeomType type = geom_type();
    if (type != other.geom_type()) return false;
    if (position_at(0) != other.position_at(0) || position_at(1) != other.position_at(1)) return false;
    const BRepAdaptor_Curve ga1(TopoDS::Edge(obj_)), ga2(TopoDS::Edge(other.obj_));
    switch (type) {
        case GeomType::LINE: return true;
        case GeomType::CIRCLE: {
            const gp_Circ c1 = ga1.Circle(), c2 = ga2.Circle();
            return std::abs(c1.Radius() - c2.Radius()) < tol && Vector(c1.Location()) == Vector(c2.Location()) &&
                   Vector(c1.Axis().Direction()) == Vector(c2.Axis().Direction());
        }
        case GeomType::ELLIPSE: {
            const gp_Elips e1 = ga1.Ellipse(), e2 = ga2.Ellipse();
            return std::abs(e1.MajorRadius() - e2.MajorRadius()) < tol &&
                   std::abs(e1.MinorRadius() - e2.MinorRadius()) < tol && Vector(e1.Location()) == Vector(e2.Location()) &&
                   Vector(e1.Axis().Direction()) == Vector(e2.Axis().Direction());
        }
        case GeomType::HYPERBOLA: {
            const gp_Hypr h1 = ga1.Hyperbola(), h2 = ga2.Hyperbola();
            return std::abs(h1.MajorRadius() - h2.MajorRadius()) < tol &&
                   std::abs(h1.MinorRadius() - h2.MinorRadius()) < tol && Vector(h1.Location()) == Vector(h2.Location()) &&
                   Vector(h1.Axis().Direction()) == Vector(h2.Axis().Direction());
        }
        case GeomType::PARABOLA: {
            const gp_Parab p1 = ga1.Parabola(), p2 = ga2.Parabola();
            return std::abs(p1.Focal() - p2.Focal()) < tol && Vector(p1.Location()) == Vector(p2.Location()) &&
                   Vector(p1.Axis().Direction()) == Vector(p2.Axis().Direction());
        }
        case GeomType::BEZIER: {
            const Handle(Geom_BezierCurve) b1 = ga1.Bezier(), b2 = ga2.Bezier();
            if (b1->Degree() != b2->Degree() || b1->NbPoles() != b2->NbPoles()) return false;
            for (int i = 1; i <= b1->NbPoles(); ++i) {
                if (Vector(b1->Pole(i)) != Vector(b2->Pole(i))) return false;
                if (b1->IsRational() && std::abs(b1->Weight(i) - b2->Weight(i)) >= tol) return false;
            }
            return true;
        }
        case GeomType::BSPLINE: {
            const Handle(Geom_BSplineCurve) s1 = ga1.BSpline(), s2 = ga2.BSpline();
            if (s1->Degree() != s2->Degree()) return false;
            if (s1->IsPeriodic() != s2->IsPeriodic()) return false;
            if (s1->NbPoles() != s2->NbPoles() || s1->NbKnots() != s2->NbKnots()) return false;
            for (int i = 1; i <= s1->NbPoles(); ++i) {
                if (Vector(s1->Pole(i)) != Vector(s2->Pole(i))) return false;
                if (s1->IsRational() && std::abs(s1->Weight(i) - s2->Weight(i)) >= tol) return false;
            }
            for (int i = 1; i <= s1->NbKnots(); ++i) {
                if (std::abs(s1->Knot(i) - s2->Knot(i)) >= tol) return false;
                if (s1->Multiplicity(i) != s2->Multiplicity(i)) return false;
            }
            return true;
        }
        case GeomType::OFFSET: {
            const Handle(Geom_OffsetCurve) oc1 = ga1.OffsetCurve(), oc2 = ga2.OffsetCurve();
            if (std::abs(oc1->Offset() - oc2->Offset()) >= tol) return false;
            if (Vector(oc1->Direction()) != Vector(oc2->Direction())) return false;
            const Edge basis1(TopoDS_Shape(BRepBuilderAPI_MakeEdge(oc1->BasisCurve()).Edge()));
            const Edge basis2(TopoDS_Shape(BRepBuilderAPI_MakeEdge(oc2->BasisCurve()).Edge()));
            return basis1.geom_equal(basis2, tol);
        }
        default:
            for (int i = 1; i <= num_interpolation_points; ++i) {
                const double t = static_cast<double>(i) / (num_interpolation_points + 1);
                if (position_at(t) != other.position_at(t)) return false;
            }
            return true;
    }
}

std::vector<Vector> Edge::find_intersection_points(const Edge& other, double tolerance) const {
    require_not_null("Can't find intersections of empty edge");
    std::vector<Vector> out;
    occt("find_intersection_points", [&] {
        BRepExtrema_DistShapeShape calc(obj_, other.wrapped());
        if (calc.IsDone() && calc.Value() <= tolerance) {
            for (int i = 1; i <= calc.NbSolution(); ++i) {
                const Vector p(calc.PointOnShape1(i));
                if (std::none_of(out.begin(), out.end(), [&](const Vector& q) { return q == p; })) out.push_back(p);
            }
        }
        return 0;
    });
    return out;
}

std::vector<Vector> Edge::find_intersection_points(const Axis& axis, double tolerance) const {
    const BoundBox bb = bounding_box().add(axis.position);
    const double d = bb.diagonal();
    return find_intersection_points(
        Edge::make_line(axis.position - axis.direction * d, axis.position + axis.direction * d), tolerance);
}

// ---------------------------------------------------------------------------
// Wire
// ---------------------------------------------------------------------------

Wire::Wire(const TopoDS_Shape& obj) : Mixin1D(obj) {
    if (!obj.IsNull() && obj.ShapeType() != TopAbs_WIRE) throw std::invalid_argument("Wire requires a TopoDS_Wire");
}

Wire::Wire(const Edge& edge) : Wire(std::vector<Edge>{edge}) {}

Wire::Wire(const std::vector<Edge>& edges_in, bool sequenced) {
    std::vector<Edge> edges = edges_in;
    if (sequenced && !edges.empty()) {
        std::vector<Edge> placed{edges.front()};
        std::vector<Edge> unplaced(edges.begin() + 1, edges.end());
        while (!unplaced.empty()) {
            const Vector target = Wire(placed).position_at(1);
            std::size_t best = 0;
            double best_d = 1e300;
            for (std::size_t i = 0; i < unplaced.size(); ++i) {
                const double d = std::min((target - unplaced[i].position_at(0)).length(),
                                          (target - unplaced[i].position_at(1)).length());
                if (d < best_d) {
                    best_d = d;
                    best = i;
                }
            }
            placed.push_back(unplaced[best]);
            unplaced.erase(unplaced.begin() + static_cast<std::ptrdiff_t>(best));
        }
        edges = placed;
    }
    std::vector<TopoDS_Shape> topods;
    for (const auto& e : edges) topods.push_back(e.wrapped());
    obj_ = occt("Wire", [&] { return TopoDS_Shape(make_wire(topods)); });
}

Wire::Wire(const Curve& curve) : Wire(std::vector<Edge>(curve.edges())) {}

ShapeList<Wire> Wire::combine(const std::vector<Edge>& edges, double tol) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& e : edges) topods.push_back(e.wrapped());
    ShapeList<Wire> out;
    for (const auto& w : connect_edges_to_wires(topods, tol)) out.push_back(Wire(w));
    return out;
}

ShapeList<Wire> Wire::combine(const std::vector<Wire>& wires, double tol) {
    std::vector<Edge> edges;
    for (const auto& w : wires) {
        for (const auto& e : w.edges()) edges.push_back(e);
    }
    return combine(edges, tol);
}

Wire Wire::make_polygon(const std::vector<Vector>& vertices, bool close) {
    if (vertices.empty()) throw std::invalid_argument("make_polygon requires points");
    std::vector<Vector> pts = vertices;
    if ((pts.front() - pts.back()).length() > TOLERANCE && close) pts.push_back(pts.front());
    return Wire(occt("Wire.make_polygon", [&] {
        BRepBuilderAPI_MakePolygon builder;
        for (const auto& p : pts) builder.Add(p.to_pnt());
        return TopoDS_Shape(builder.Wire());
    }));
}

Wire Wire::make_rect(double width, double height, const Plane& plane) {
    const std::vector<Vector> corners = {
        plane.from_local_coords(Vector(width / 2, height / 2)),
        plane.from_local_coords(Vector(width / 2, height / -2)),
        plane.from_local_coords(Vector(width / -2, height / -2)),
        plane.from_local_coords(Vector(width / -2, height / 2))};
    return make_polygon(corners, true);
}

Wire Wire::make_circle(double radius, const Plane& plane) {
    return Wire(Edge::make_circle(radius, plane));
}

Wire Wire::make_ellipse(double x_radius, double y_radius, const Plane& plane, double start_angle,
                        double end_angle, AngularDirection angular_direction, bool closed) {
    const Edge e = Edge::make_ellipse(x_radius, y_radius, plane, start_angle, end_angle, angular_direction);
    if (start_angle != end_angle && closed) {
        return Wire(std::vector<Edge>{e, Edge::make_line(e.end_point(), e.start_point())});
    }
    return Wire(e);
}

namespace {

/// Andrew's monotone chain; returns hull indices in counter-clockwise order.
std::vector<std::size_t> convex_hull_2d(const std::vector<std::array<double, 2>>& pts) {
    std::vector<std::size_t> idx(pts.size());
    for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
        return pts[a][0] < pts[b][0] || (pts[a][0] == pts[b][0] && pts[a][1] < pts[b][1]);
    });
    auto cross = [&](std::size_t o, std::size_t a, std::size_t b) {
        return (pts[a][0] - pts[o][0]) * (pts[b][1] - pts[o][1]) -
               (pts[a][1] - pts[o][1]) * (pts[b][0] - pts[o][0]);
    };
    std::vector<std::size_t> hull(2 * idx.size());
    std::size_t k = 0;
    for (std::size_t i : idx) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], i) <= 1e-12) --k;
        hull[k++] = i;
    }
    for (std::size_t j = idx.size() - 1, t = k + 1; j-- > 0;) {
        const std::size_t i = idx[j];
        while (k >= t && cross(hull[k - 2], hull[k - 1], i) <= 1e-12) --k;
        hull[k++] = i;
    }
    hull.resize(k > 0 ? k - 1 : 0);
    return hull;
}

}  // namespace

Wire Wire::make_convex_hull(const std::vector<Edge>& edges, double tolerance) {
    const int fragments = static_cast<int>(2 / tolerance);
    std::vector<std::array<double, 2>> points;
    std::vector<std::pair<std::size_t, double>> lookup;
    for (std::size_t ei = 0; ei < edges.size(); ++ei) {
        for (int i = 0; i < fragments; ++i) {
            const double param = static_cast<double>(i) / (fragments - 1);
            const Vector p = edges[ei].position_at(param);
            points.push_back({p.X, p.Y});
            lookup.emplace_back(ei, param);
        }
    }
    const auto hull = convex_hull_2d(points);
    if (hull.size() < 2) throw std::invalid_argument("Unable to create a convex hull");

    std::vector<std::pair<std::pair<std::size_t, double>, std::pair<std::size_t, double>>> connecting;
    std::map<std::size_t, std::vector<std::size_t>> trim_points;
    for (std::size_t h = 0; h < hull.size(); ++h) {
        const std::size_t s0 = hull[h], s1 = hull[(h + 1) % hull.size()];
        const std::size_t e0 = lookup[s0].first, e1 = lookup[s1].first;
        if (e0 != e1) {
            trim_points[e0].push_back(s0);
            trim_points[e1].push_back(s1);
            connecting.push_back({lookup[s0], lookup[s1]});
        } else if ((s0 > s1 ? s0 - s1 : s1 - s0) != 1) {
            const std::size_t a = std::min(s0, s1), b = std::max(s0, s1);
            trim_points[e0].push_back(a);
            trim_points[e0].push_back(b);
            connecting.push_back({lookup[a], lookup[b]});
        }
    }
    std::vector<Edge> all;
    for (const auto& c : connecting) {
        all.push_back(Edge::make_line(edges[c.first.first].position_at(c.first.second),
                                      edges[c.second.first].position_at(c.second.second)));
    }
    for (auto& [edge_index, pts] : trim_points) {
        std::sort(pts.begin(), pts.end());
        for (std::size_t i = 0; i + 1 < pts.size(); i += 2) {
            if (pts[i] != pts[i + 1]) {
                all.push_back(edges[edge_index].trim(lookup[pts[i]].second, lookup[pts[i + 1]].second));
            }
        }
    }
    return Wire(all, true);
}

ShapeList<Edge> Wire::edges(Select select) const {
    ShapeList<Edge> out;
    if (obj_.IsNull()) return select_shapes(*this, std::move(out), select);
    const auto path = extraction_path(*this);
    for (const auto& e : wire_edges(obj_)) {
        Edge edge(e);
        ShapeAccess::set_topo_path(edge, path);
        out.push_back(std::move(edge));
    }
    return select_shapes(*this, std::move(out), select);
}

ShapeList<Edge> Wire::order_edges() const {
    const auto sorted = edges().sort_by(*this);
    ShapeList<Edge> ordered;
    if (sorted.empty()) return ordered;
    ordered.push_back(sorted[0]);
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        const Edge& last = ordered.back();
        if ((last.position_at(1) - sorted[i].position_at(0)).length() < TOLERANCE) {
            ordered.push_back(sorted[i]);
        } else {
            ordered.push_back(sorted[i].reversed());
        }
    }
    return ordered;
}

Wire Wire::close() const {
    if (is_closed()) return *this;
    const Edge edge = Edge::make_line(end_point(), start_point());
    std::vector<Edge> all(edges());
    all.push_back(edge);
    return combine(all).at(0);
}

Wire Wire::fix_degenerate_edges(double precision) const {
    require_not_null("Can't fix an empty wire");
    ShapeFix_Wireframe fixer(obj_);
    fixer.SetPrecision(precision);
    fixer.SetMaxTolerance(1e-6);
    fixer.FixSmallEdges();
    fixer.FixWireGaps();
    return Wire(unwrap(fixer.Shape(), true));
}

Wire Wire::stitch(const Wire& other) const {
    return Wire(occt("Wire.stitch", [&] {
        BRepBuilderAPI_MakeWire builder;
        builder.Add(TopoDS::Wire(obj_));
        builder.Add(TopoDS::Wire(other.obj_));
        builder.Build();
        return TopoDS_Shape(builder.Wire());
    }));
}

Wire Wire::trim(double start, double end) const {
    if (start >= end) throw std::invalid_argument("start must be less than end");
    const auto es = edges();
    const double total = length();
    std::vector<Edge> kept;
    double acc = 0.0;
    for (const auto& e : es) {
        const double len = e.length();
        const double e0 = acc / total, e1 = (acc + len) / total;
        acc += len;
        if (e1 <= start + 1e-12 || e0 >= end - 1e-12) continue;
        const double s = std::max(0.0, (start - e0) / (e1 - e0));
        const double t = std::min(1.0, (end - e0) / (e1 - e0));
        kept.push_back((s <= 1e-12 && t >= 1 - 1e-12) ? e : e.trim(s, t));
    }
    if (kept.empty()) throw std::invalid_argument("Wire trim produced no edges");
    return Wire(kept);
}

}  // namespace build123d
