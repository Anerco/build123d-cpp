#include <algorithm>
#include <cmath>
#include <memory>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepFill.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BoundedSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

const TopoDS_Face& as_face(const TopoDS_Shape& s) { return TopoDS::Face(s); }

std::array<double, 4> uv_bounds(const TopoDS_Face& face) {
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    return {u0, u1, v0, v1};
}

std::vector<Face> to_faces(const ShapeList<Face>& list) { return std::vector<Face>(list.begin(), list.end()); }

}  // namespace

namespace detail {

std::vector<Face> as_faces(const Shape& shape) {
    if (shape.is_null()) return {};
    return to_faces(shape.faces());
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Face
// ---------------------------------------------------------------------------

Face::Face(const TopoDS_Shape& obj) : Mixin2D(obj) {
    if (!obj.IsNull() && obj.ShapeType() != TopAbs_FACE) throw std::invalid_argument("Face requires a TopoDS_Face");
}

Face::Face(const Wire& outer_wire, const std::vector<Wire>& inner_wires) {
    std::vector<TopoDS_Wire> inner;
    for (const auto& w : inner_wires) inner.push_back(TopoDS::Wire(w.wrapped()));
    if (outer_wire.is_null()) throw std::invalid_argument("Face can only be created with closed wires");
    bool all_closed = BRep_Tool::IsClosed(outer_wire.wrapped());
    for (const auto& w : inner) all_closed = all_closed && BRep_Tool::IsClosed(w);
    if (!all_closed) throw std::invalid_argument("Face can only be created with closed wires");
    obj_ = make_face_from_wires(TopoDS::Wire(outer_wire.wrapped()), inner);
}

Face Face::make_rect(double width, double height, const Plane& plane) {
    return Face(occt("Face.make_rect", [&] {
        return TopoDS_Shape(BRepBuilderAPI_MakeFace(plane.wrapped(), -width * 0.5, width * 0.5,
                                                    -height * 0.5, height * 0.5)
                                .Face());
    }));
}

Face Face::make_plane(const Plane& plane) {
    return Face(occt("Face.make_plane", [&] { return TopoDS_Shape(BRepBuilderAPI_MakeFace(plane.wrapped()).Face()); }));
}

Face Face::make_surface(const Wire& exterior, const std::vector<Vector>& surface_points,
                        const std::vector<Wire>& interior_wires) {
    Face surface_face = occt("Error building non-planar face with provided exterior", [&] {
        BRepOffsetAPI_MakeFilling surface(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
        for (const auto& e : exterior.edges()) surface.Add(TopoDS::Edge(e.wrapped()), GeomAbs_C0);
        surface.Build();
        if (!surface_points.empty()) {
            for (const auto& p : surface_points) surface.Add(p.to_pnt());
            surface.Build();
        }
        return Face(surface.Shape());
    });
    if (!interior_wires.empty()) {
        surface_face = occt("Error adding interior hole in non-planar face with provided interior_wires", [&] {
            BRepBuilderAPI_MakeFace maker(TopoDS::Face(surface_face.wrapped()));
            for (const auto& w : interior_wires) maker.Add(TopoDS::Wire(w.wrapped()));
            return Face(maker.Face());
        });
    }
    surface_face = surface_face.fix();
    if (!surface_face.is_valid()) throw std::runtime_error("non planar face is invalid");
    return surface_face;
}

Face Face::make_surface_from_curves(const Edge& curve1, const Edge& curve2) {
    return Face(occt("make_surface_from_curves", [&] {
        return TopoDS_Shape(BRepFill::Face(TopoDS::Edge(curve1.wrapped()), TopoDS::Edge(curve2.wrapped())));
    }));
}

Face Face::make_surface_from_curves(const Wire& curve1, const Wire& curve2) {
    const TopoDS_Shell shell = occt("make_surface_from_curves", [&] {
        return BRepFill::Shell(TopoDS::Wire(curve1.wrapped()), TopoDS::Wire(curve2.wrapped()));
    });
    const auto faces = entities(shell, TopAbs_FACE);
    if (faces.size() != 1) throw std::invalid_argument("The ruled surface between the wires has multiple faces; use Shell");
    return Face(faces[0]);
}

Face Face::make_bezier_surface(const std::vector<std::vector<Vector>>& points,
                               const std::vector<std::vector<double>>& weights) {
    if (points.size() < 2 || points[0].size() < 2) throw std::invalid_argument("At least 2x2 control points are required");
    const int rows = static_cast<int>(points.size()), cols = static_cast<int>(points[0].size());
    OccArray2Pnt poles(1, rows, 1, cols);
    for (int r = 0; r < rows; ++r) {
        if (static_cast<int>(points[static_cast<std::size_t>(r)].size()) != cols) {
            throw std::invalid_argument("All rows of control points must have the same length");
        }
        for (int c = 0; c < cols; ++c) {
            poles.SetValue(r + 1, c + 1, points[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)].to_pnt());
        }
    }
    return Face(occt("make_bezier_surface", [&] {
        Handle(Geom_BezierSurface) surface;
        if (!weights.empty()) {
            OccArray2Real w(1, rows, 1, cols);
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    w.SetValue(r + 1, c + 1, weights[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]);
                }
            }
            surface = new Geom_BezierSurface(poles, w);
        } else {
            surface = new Geom_BezierSurface(poles);
        }
        return TopoDS_Shape(BRepBuilderAPI_MakeFace(surface, Precision::Confusion()).Face());
    }));
}

Face Face::make_surface_from_array_of_points(const std::vector<std::vector<Vector>>& points, double tol,
                                             std::optional<std::array<double, 3>> smoothing, int min_deg,
                                             int max_deg) {
    if (points.empty() || points[0].empty()) throw std::invalid_argument("No points provided");
    const int rows = static_cast<int>(points.size()), cols = static_cast<int>(points[0].size());
    OccArray2Pnt grid(1, rows, 1, cols);
    for (int r = 0; r < rows; ++r) {
        const auto& row = points[static_cast<std::size_t>(r)];
        if (static_cast<int>(row.size()) != cols) throw std::invalid_argument("All rows of points must have the same length");
        for (int c = 0; c < cols; ++c) grid.SetValue(r + 1, c + 1, row[static_cast<std::size_t>(c)].to_pnt());
    }
    return Face(occt("make_surface_from_array_of_points", [&] {
        std::unique_ptr<GeomAPI_PointsToBSplineSurface> builder;
        if (smoothing) {
            // OCCT needs a work degree of at least 6 for the C2 smoothing variant.
            builder = std::make_unique<GeomAPI_PointsToBSplineSurface>(
                grid, (*smoothing)[0], (*smoothing)[1], (*smoothing)[2], std::max(max_deg, 6), GeomAbs_C2, tol);
        } else {
            builder = std::make_unique<GeomAPI_PointsToBSplineSurface>(grid, min_deg, max_deg, GeomAbs_C2, tol);
        }
        if (!builder->IsDone()) throw std::invalid_argument("B-spline approximation failed");
        return TopoDS_Shape(BRepBuilderAPI_MakeFace(builder->Surface(), Precision::Confusion()).Face());
    }));
}

Face Face::revolve(const Edge& profile, double angle, const Axis& axis) {
    return Face(occt("Face.revolve", [&] {
        BRepPrimAPI_MakeRevol builder(profile.wrapped(), axis.wrapped(), angle * DEG2RAD, true);
        return unwrap(builder.Shape(), true);
    }));
}

Face Face::extrude(const Edge& obj, const Vector& direction) {
    return Face(extrude_shape(obj.wrapped(), direction));
}

Face Face::sweep(const Shape& profile, const Shape& path, Transition transition) {
    const auto profile_edges = profile.edges();
    const auto path_edges = path.edges();
    if (profile_edges.size() != 1 || path_edges.size() != 1) {
        throw std::invalid_argument("Use Shell::sweep for multi Edge objects");
    }
    const Wire profile_wire(profile_edges[0]);
    const Wire path_wire(path_edges[0]);
    Face result(occt("Face.sweep", [&] {
        BRepOffsetAPI_MakePipeShell builder(TopoDS::Wire(path_wire.wrapped()));
        builder.Add(profile_wire.wrapped(), false, false);
        builder.SetTransitionMode(to_occ(transition));
        builder.Build();
        return unwrap(builder.Shape(), true);
    }));
    if (clean_enabled()) result = result.clean();
    return result;
}

Face Face::operator-() const {
    require_not_null("Invalid Shape");
    Face result = derive<Face>(obj_.Complemented());
    result.set_topo_parent(std::nullopt);
    return result;
}

Vector Face::normal_at() const { return normal_at(0.5, 0.5); }

Vector Face::normal_at(const Vector& surface_point) const {
    require_not_null("Empty face");
    return face_normal_at(as_face(obj_), surface_point);
}

Vector Face::normal_at(double u, double v) const {
    require_not_null("Empty face");
    const auto b = uv_bounds(as_face(obj_));
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(as_face(obj_)).Normal(b[0] + u * (b[1] - b[0]), b[2] + v * (b[3] - b[2]), p, n);
    return Vector(n).normalized();
}

Vector Face::position_at(double u, double v) const {
    require_not_null("Empty face");
    const auto b = uv_bounds(as_face(obj_));
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(as_face(obj_)).Normal(b[0] + u * (b[1] - b[0]), b[2] + v * (b[3] - b[2]), p, n);
    return Vector(p);
}

namespace {

Location face_location(const TopoDS_Face& face, double u_val, double v_val, const std::optional<Vector>& x_dir) {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    gp_Pnt pnt;
    gp_Vec du, dv;
    surface->D1(u_val, v_val, pnt, du, dv);
    Vector z = Vector(du).cross(Vector(dv)).normalized();
    if (face.Orientation() == TopAbs_REVERSED) z = -z;
    const Vector x = x_dir ? x_dir->normalized() : Vector(du).normalized();
    return Location(Plane(Vector(pnt), x, z));
}

}  // namespace

Location Face::location_at(double u, double v, std::optional<Vector> x_dir) const {
    require_not_null("Empty face");
    const auto b = uv_bounds(as_face(obj_));
    return face_location(as_face(obj_), b[0] + u * (b[1] - b[0]), b[2] + v * (b[3] - b[2]), x_dir);
}

Location Face::location_at(const Vector& surface_point, std::optional<Vector> x_dir) const {
    require_not_null("Empty face");
    GeomAPI_ProjectPointOnSurf projector(surface_point.to_pnt(), BRep_Tool::Surface(as_face(obj_)));
    double u, v;
    projector.LowerDistanceParameters(u, v);
    return face_location(as_face(obj_), u, v, x_dir);
}

Location Face::center_location() const {
    const Vector origin = position_at(0.5, 0.5);
    return Plane(origin, std::nullopt, normal_at(origin)).location();
}

Wire Face::outer_wire() const {
    require_not_null("Empty face");
    Wire outer(BRepTools::OuterWire(as_face(obj_)));
    outer.extracted_from(*this);
    return outer;
}

ShapeList<Wire> Face::inner_wires() const {
    const Wire outer = outer_wire();
    ShapeList<Wire> out;
    for (auto& w : wires()) {
        if (!w.is_same(outer)) out.push_back(w);
    }
    return out;
}

std::optional<Plane> Face::is_planar() const {
    require_not_null("Empty face");
    GeomLib_IsPlanarSurface searcher(BRep_Tool::Surface(as_face(obj_)), TOLERANCE);
    if (!searcher.IsPlanar()) return std::nullopt;
    gp_Pln pln = searcher.Plan();
    if (!pln.Position().Direct()) pln = gp_Pln(gp_Ax3(pln.Position().Ax2()));
    return Plane(pln);
}

bool Face::is_coplanar(const Plane& plane) const {
    const auto b = uv_bounds(as_face(obj_));
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(as_face(obj_)).Normal(b[0], b[2], p, n);
    return plane.contains(Vector(p)) && 1 - std::abs(plane.z_dir.dot(Vector(n))) < TOLERANCE;
}

bool Face::is_inside(const Vector& point, double tolerance) const {
    BRepClass3d_SolidClassifier classifier(obj_);
    classifier.Perform(point.to_pnt(), tolerance);
    return classifier.IsOnAFace();
}

std::optional<double> Face::width() const {
    const auto plane = is_planar();
    if (!plane) return std::nullopt;
    const Face flat = Plane(*this).to_local_coords(*this);
    const auto v = flat.vertices().sort_by(Axis::Y);
    return v[-1].Y() - v[0].Y();
}

std::optional<double> Face::length() const {
    const auto plane = is_planar();
    if (!plane) return std::nullopt;
    const Face flat = Plane(*this).to_local_coords(*this);
    const auto v = flat.vertices().sort_by(Axis::X);
    return v[-1].X() - v[0].X();
}

std::optional<double> Face::radius() const {
    const GeomType t = geom_type();
    Handle(Geom_Surface) s = BRep_Tool::Surface(as_face(obj_));
    if (!Handle(Geom_RectangularTrimmedSurface)::DownCast(s).IsNull()) return std::nullopt;
    if (t == GeomType::CYLINDER) {
        auto c = Handle(Geom_CylindricalSurface)::DownCast(s);
        if (!c.IsNull()) return c->Radius();
    }
    if (t == GeomType::SPHERE) {
        auto sp = Handle(Geom_SphericalSurface)::DownCast(s);
        if (!sp.IsNull()) return sp->Radius();
    }
    return std::nullopt;
}

std::optional<Axis> Face::axis_of_rotation() const {
    Handle(Geom_Surface) surf = BRep_Tool::Surface(as_face(obj_));
    for (;;) {
        auto trimmed = Handle(Geom_RectangularTrimmedSurface)::DownCast(surf);
        if (!trimmed.IsNull()) {
            surf = trimmed->BasisSurface();
            continue;
        }
        auto offset = Handle(Geom_OffsetSurface)::DownCast(surf);
        if (!offset.IsNull()) {
            surf = offset->BasisSurface();
            continue;
        }
        break;
    }
    if (auto c = Handle(Geom_ConicalSurface)::DownCast(surf); !c.IsNull()) return Axis(c->Cone().Axis());
    if (auto c = Handle(Geom_CylindricalSurface)::DownCast(surf); !c.IsNull()) return Axis(c->Cylinder().Axis());
    if (auto s = Handle(Geom_SphericalSurface)::DownCast(surf); !s.IsNull()) {
        const gp_Ax3 ax3 = s->Position();
        return Axis(gp_Ax1(ax3.Location(), ax3.Direction()));
    }
    if (auto t = Handle(Geom_ToroidalSurface)::DownCast(surf); !t.IsNull()) return Axis(t->Torus().Axis());
    if (auto r = Handle(Geom_SurfaceOfRevolution)::DownCast(surf); !r.IsNull()) return Axis(r->Axis());
    return std::nullopt;
}

std::optional<std::string> Face::geometry() const {
    if (!is_planar()) return std::nullopt;
    const Face flat = Plane(*this).to_local_coords(*this);
    const auto edges = flat.edges();
    for (const auto& e : edges) {
        if (e.geom_type() != GeomType::LINE) return std::nullopt;
    }
    std::string result = "POLYGON";
    if (edges.size() == 4) {
        bool right_angles = true;
        for (const auto& v : flat.vertices()) {
            std::vector<Vector> dirs;
            for (const auto& e : edges) {
                for (const auto& ev : e.vertices()) {
                    if (ev.is_same(v)) dirs.push_back(e.tangent_at(0.0));
                }
            }
            if (dirs.size() == 2 && std::abs(dirs[0].get_angle(dirs[1]) - 90.0) > 1e-9) right_angles = false;
        }
        if (right_angles) {
            result = "RECTANGLE";
            if (edges.group_by(SortBy::LENGTH).size() == 1) result = "SQUARE";
        }
    }
    return result;
}

double Face::area_without_holes() const {
    if (is_null()) return 0.0;
    return without_holes().area();
}

Face Face::fillet_2d(double radius, const std::vector<Vertex>& vertices) const {
    require_not_null("Can't fillet an empty face");
    std::vector<Vertex> valid;
    for (const auto& v : vertices) {
        if (!v.is_null()) valid.push_back(v);
    }
    if (valid.empty()) return *this;
    Face result = derive<Face>(occt("fillet_2d", [&] {
        BRepFilletAPI_MakeFillet2d builder(as_face(obj_));
        const auto face_vertices = entities(obj_, TopAbs_VERTEX);
        for (const auto& v : valid) {
            TopoDS_Shape match;
            for (const auto& fv : face_vertices) {
                if (fv.IsSame(v.wrapped())) {
                    match = fv;
                    break;
                }
            }
            if (match.IsNull()) {
                const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(v.wrapped()));
                for (const auto& fv : face_vertices) {
                    if (BRep_Tool::Pnt(TopoDS::Vertex(fv)).Distance(p) < TOLERANCE) {
                        match = fv;
                        break;
                    }
                }
            }
            if (match.IsNull()) throw std::invalid_argument("Vertex is not part of the face");
            builder.AddFillet(TopoDS::Vertex(match), radius);
        }
        builder.Build();
        if (!builder.IsDone()) throw std::invalid_argument("Fillet failed, try a smaller radius");
        return builder.Shape();
    }));
    result = result.fix();
    if (!(result.normal_at() == normal_at())) result = -result;
    return result;
}

Face Face::chamfer_2d(double distance, double distance2, const std::vector<Vertex>& vertices,
                      std::optional<Edge> edge) const {
    require_not_null("Can't chamfer an empty face");
    Face result = derive<Face>(occt("chamfer_2d", [&] {
        BRepFilletAPI_MakeFillet2d builder(as_face(obj_));
        OccShapeListMap map;
        TopExp::MapShapesAndAncestors(obj_, TopAbs_VERTEX, TopAbs_EDGE, map);
        for (const auto& v : vertices) {
            if (!map.Contains(v.wrapped())) throw std::invalid_argument("Vertex is not part of the face");
            const OccShapeList& edge_list = map.FindFromKey(v.wrapped());
            TopoDS_Edge e1 = TopoDS::Edge(edge_list.First());
            TopoDS_Edge e2 = TopoDS::Edge(edge_list.Last());
            if (edge) {
                if (e2.IsSame(edge->wrapped())) {
                    std::swap(e1, e2);
                } else if (!e1.IsSame(edge->wrapped())) {
                    throw std::invalid_argument("reference edge not in edges");
                }
            }
            builder.AddChamfer(e1, e2, distance, distance2);
        }
        builder.Build();
        if (!builder.IsDone()) throw std::invalid_argument("Cannot chamfer at this location");
        return builder.Shape();
    }));
    return result.fix();
}

Face Face::make_holes(const std::vector<Wire>& interior_wires) const {
    Face result = derive<Face>(occt("Error adding interior hole in non-planar face with provided interior_wires", [&] {
        BRepBuilderAPI_MakeFace maker(as_face(obj_));
        for (const auto& w : interior_wires) maker.Add(TopoDS::Wire(w.wrapped()));
        return TopoDS_Shape(maker.Face());
    }));
    return result.fix();
}

Face Face::without_holes() const {
    require_not_null("Cannot remove holes from an empty face");
    const auto inner = inner_wires();
    if (inner.empty()) return *this;
    Handle(BRepTools_ReShape) reshaper = new BRepTools_ReShape();
    for (const auto& w : inner) reshaper->Remove(w.wrapped());
    return derive<Face>(reshaper->Apply(obj_));
}

ShapeList<Shape> Face::project_to_shape(const Shape& target, const Vector& direction) const {
    require_not_null("Can't project an empty face");
    if (target.is_null()) throw std::invalid_argument("Can't project onto an empty shape");
    if (target.wrapped().ShapeType() == TopAbs_VERTEX) {
        throw std::invalid_argument("projection to a vertex is not supported");
    }
    const double max_dimension =
        BoundBox::from_topo_ds(make_compound({obj_, target.wrapped()}), TOLERANCE, true).diagonal();
    const TopoDS_Shape extruded = extrude_shape(obj_, direction * max_dimension);
    ShapeList<Shape> intersected;
    if (target.wrapped().ShapeType() == TopAbs_FACE) {
        const TopoDS_Shape common = bool_op(BoolOp::COMMON, {extruded}, {target.wrapped()}, 0.0, false);
        for (const auto& f : top_level(common)) intersected.push_back(Face(f));
    } else {
        for (const auto& shell : target.shells()) {
            const TopoDS_Shape common = bool_op(BoolOp::COMMON, {extruded}, {shell.wrapped()}, 0.0, false);
            for (const auto& s : top_level(common)) intersected.push_back(Shell(s));
        }
    }
    // A region crossing several faces of the target stays a Shell; single faces become Faces.
    ShapeList<Shape> projected;
    for (const auto& shape : intersected.sort_by(Axis(center(), direction))) {
        const auto faces = shape.faces();
        if (faces.size() == 1) {
            projected.push_back(faces[0]);
        } else {
            projected.push_back(shape);
        }
    }
    return projected;
}

ShapeList<Shape> Shape::project_faces(const Shape& faces, const Mixin1D& path, double start) const {
    return project_faces(to_faces(faces.faces()), path, start);
}

ShapeList<Shape> Shape::project_faces(const std::vector<Face>& faces, const Mixin1D& path, double start) const {
    require_not_null("Can't project onto an empty shape");
    ShapeList<Shape> projected;
    if (faces.empty()) return projected;
    const double path_length = path.length();
    const Vector shape_center = center();
    const double first_face_min_x = faces[0].bounding_box().min.X;
    for (const auto& face : faces) {
        const BoundBox bbox = face.bounding_box();
        const double face_center_x = (bbox.min.X + bbox.max.X) / 2;
        const double relative_position = start + (face_center_x - first_face_min_x) / path_length;
        const Vector path_position = path.position_at(relative_position);
        const Vector path_tangent = path.tangent_at(relative_position);
        const auto hits = find_intersection_points(Axis(path_position, shape_center - path_position));
        if (hits.empty()) throw std::invalid_argument("project_faces: the path is not on the shape");
        const auto& [surface_point, surface_normal] = hits[0];
        const Plane surface_normal_plane(surface_point, path_tangent, surface_normal);
        const Face projection_face =
            surface_normal_plane.from_local_coords(face.moved(Location(Vector(-face_center_x, 0, 0))));
        const auto result = projection_face.project_to_shape(*this, surface_normal * -1);
        if (result.empty()) throw std::invalid_argument("project_faces: a face missed the shape");
        projected.push_back(result[0]);
    }
    return projected;
}

// ---------------------------------------------------------------------------
// Plane from a Face
// ---------------------------------------------------------------------------

Plane::Plane(const Face& face, std::optional<Vector> x_dir_in) {
    if (face.is_null() || !face.is_planar()) throw std::invalid_argument("Planes can only be created from planar faces");
    const TopoDS_Face& f = TopoDS::Face(face.wrapped());
    const Vector o(properties(f, 2).CentreOfMass());
    Handle(Geom_Surface) surface = BRep_Tool::Surface(f);
    Vector face_x;
    if (!Handle(Geom_BoundedSurface)::DownCast(surface).IsNull()) {
        gp_Pnt p;
        gp_Vec du, dv;
        surface->D1(0.5, 0.5, p, du, dv);
        face_x = Vector(du);
    } else {
        auto elementary = Handle(Geom_ElementarySurface)::DownCast(surface);
        if (elementary.IsNull()) throw std::invalid_argument("Unsupported planar surface");
        face_x = Vector(elementary->Position().XDirection());
    }
    auto r14 = [](const Vector& v) { return Vector(py_round(v.X, 14), py_round(v.Y, 14), py_round(v.Z, 14)); };
    const Vector x = r14(x_dir_in ? *x_dir_in : face_x);
    const Vector z = r14(face_normal_at_origin_projection(f));
    init(o, x.normalized(), z.normalized());
}

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

Shell::Shell(const TopoDS_Shape& obj) : Mixin2D(obj) {
    if (!obj.IsNull() && obj.ShapeType() != TopAbs_SHELL) throw std::invalid_argument("Shell requires a TopoDS_Shell");
}

Shell::Shell(const Face& face) {
    if (face.is_null()) throw std::invalid_argument("Can't create a Shell from empty Face");
    BRep_Builder builder;
    TopoDS_Shell shell;
    builder.MakeShell(shell);
    builder.Add(shell, face.wrapped());
    obj_ = shell;
}

Shell::Shell(const std::vector<Face>& faces) {
    if (faces.size() == 1) {
        *this = Shell(faces[0]);
        return;
    }
    std::vector<TopoDS_Shape> topods;
    for (const auto& f : faces) topods.push_back(f.wrapped());
    const TopoDS_Shape sewed = sew_faces(topods);
    if (sewed.ShapeType() != TopAbs_SHELL) throw std::invalid_argument("Unable to create Shell, faces are not connected");
    obj_ = sewed;
}

Shell Shell::make_loft(const std::vector<Shape>& sections, bool ruled) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& s : sections) topods.push_back(s.wrapped());
    return Shell(unwrap(detail::make_loft(topods, false, ruled), true));
}

Shell Shell::extrude(const Wire& obj, const Vector& direction) {
    return Shell(extrude_shape(obj.wrapped(), direction));
}

Shell Shell::revolve(const Shape& profile, double angle, const Axis& axis) {
    const Wire wire(std::vector<Edge>(profile.edges()));
    return Shell(occt("Shell.revolve", [&] {
        BRepPrimAPI_MakeRevol builder(wire.wrapped(), axis.wrapped(), angle * DEG2RAD, true);
        return builder.Shape();
    }));
}

Shell Shell::sweep(const Shape& profile, const Shape& path, Transition transition) {
    const Wire profile_wire(std::vector<Edge>(profile.edges()));
    const Wire path_wire(std::vector<Edge>(Wire(std::vector<Edge>(path.edges())).order_edges()));
    Shell result(occt("Shell.sweep", [&] {
        BRepOffsetAPI_MakePipeShell builder(TopoDS::Wire(path_wire.wrapped()));
        builder.Add(profile_wire.wrapped(), false, false);
        builder.SetTransitionMode(to_occ(transition));
        builder.Build();
        return builder.Shape();
    }));
    if (clean_enabled()) result = result.clean();
    return result;
}

double Shell::volume() const { return Shape::volume(); }

Shell Shell::operator-() const {
    require_not_null("Invalid Shape");
    Shell result = derive<Shell>(obj_.Complemented());
    result.set_topo_parent(std::nullopt);
    return result;
}

}  // namespace build123d
