#include <algorithm>
#include <cmath>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <BRepTools_History.hxx>
#include <LocOpe_DPrism.hxx>
#include <ShapeFix_Solid.hxx>
#include <StdFail_NotDone.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

/// Wrap a 3D operation result like build123d's _make_3d_result.
TopoDS_Shape make_3d_result(const TopoDS_Shape& shape) {
    TopoDS_Shape result = unwrap(shape, true);
    if (result.ShapeType() == TopAbs_COMPOUND) {
        std::vector<TopoDS_Shape> solids;
        for (const auto& s : top_level(result)) solids.push_back(s);
        return make_compound(solids);
    }
    return result;
}

void set_sweep_mode(BRepOffsetAPI_MakePipeShell& builder, const Wire& path, bool is_frenet,
                    const std::optional<Vector>& binormal_dir, const std::optional<Wire>& binormal_wire,
                    bool& rotate) {
    rotate = false;
    if (binormal_dir) {
        gp_Ax2 cs;
        cs.SetLocation(path.start_point().to_pnt());
        cs.SetDirection(binormal_dir->to_dir());
        builder.SetMode(cs);
        rotate = true;
    } else if (binormal_wire) {
        builder.SetMode(TopoDS::Wire(binormal_wire->wrapped()), true);
    } else {
        builder.SetMode(is_frenet);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Mixin3D
// ---------------------------------------------------------------------------

bool Mixin3D::is_inside(const Vector& point, double tolerance) const {
    BRepClass3d_SolidClassifier classifier(obj_);
    classifier.Perform(point.to_pnt(), tolerance);
    return classifier.State() == TopAbs_IN || classifier.IsOnAFace();
}

TopoDS_Shape Mixin3D::fillet_topods(double radius, const std::vector<Edge>& edge_list,
                                    std::shared_ptr<ShapeHistory>* history) const {
    require_not_null("Can't fillet an empty shape");
    TopoDS_Shape result;
    bool ok = true;
    try {
        BRepFilletAPI_MakeFillet builder(obj_);
        for (const auto& e : edge_list) builder.Add(radius, TopoDS::Edge(e.wrapped()));
        builder.Build();
        ok = builder.IsDone();
        if (ok) {
            result = make_3d_result(builder.Shape());
            ok = Shape(result).is_valid();
        }
        if (ok && history) *history = ShapeHistory::from_algorithm(builder, {obj_}, result);
    } catch (const Standard_Failure&) {
        ok = false;
    }
    if (!ok) {
        throw std::invalid_argument("Failed creating a fillet with radius of " + std::to_string(radius) +
                                    ", try a smaller value or use max_fillet() to find the largest "
                                    "valid fillet radius");
    }
    return result;
}

TopoDS_Shape Mixin3D::chamfer_topods(double length, std::optional<double> length2,
                                     const std::vector<Edge>& edge_list,
                                     const std::optional<Face>& face,
                                     std::shared_ptr<ShapeHistory>* history) const {
    require_not_null("Can't chamfer an empty shape");
    if (face) {
        const auto face_edges = face->edges();
        for (const auto& e : edge_list) {
            if (!face_edges.contains(e)) throw std::invalid_argument("Some edges are not part of the face");
        }
    }
    OccShapeListMap edge_face_map;
    TopExp::MapShapesAndAncestors(obj_, TopAbs_EDGE, TopAbs_FACE, edge_face_map);
    const double d1 = length;
    const double d2 = (length2 && *length2 != 0.0) ? *length2 : length;
    TopoDS_Shape result;
    bool ok = true;
    try {
        BRepFilletAPI_MakeChamfer builder(obj_);
        for (const auto& e : edge_list) {
            const TopoDS_Edge& edge = TopoDS::Edge(e.wrapped());
            TopoDS_Face topo_face;
            if (face) {
                topo_face = TopoDS::Face(face->wrapped());
            } else {
                if (!edge_face_map.Contains(edge)) throw std::invalid_argument("Edge is not part of the shape");
                topo_face = TopoDS::Face(edge_face_map.FindFromKey(edge).First());
            }
            builder.Add(d1, d2, edge, topo_face);
        }
        builder.Build();
        ok = builder.IsDone();
        if (ok) {
            result = make_3d_result(builder.Shape());
            ok = Shape(result).is_valid();
        }
        if (ok && history) *history = ShapeHistory::from_algorithm(builder, {obj_}, result);
    } catch (const Standard_Failure&) {
        ok = false;
    }
    if (!ok) throw std::invalid_argument("Failed creating a chamfer, try a smaller length value(s)");
    return result;
}

TopoDS_Shape Mixin3D::offset_3d_topods(const std::vector<Face>& openings, double thickness,
                                       double tolerance, Kind kind) const {
    require_not_null("Can't offset an empty shape");
    if (kind == Kind::TANGENT) throw std::invalid_argument("Kind.TANGENT not supported");
    std::vector<TopoDS_Shape> faces;
    for (const auto& f : openings) faces.push_back(f.wrapped());
    TopoDS_Shape result;
    try {
        BRepOffsetAPI_MakeThickSolid builder;
        builder.MakeThickSolidByJoin(obj_, to_list(faces), thickness, tolerance, BRepOffset_Skin, true,
                                     false, to_occ(kind), true);
        builder.Build();
        result = builder.Shape();
    } catch (const Standard_Failure&) {
        throw std::runtime_error("offset Error, an alternative kind may resolve this error");
    }
    if (Shape(result).volume() < 0) result.Reverse();
    return result;
}

double Mixin3D::max_fillet(const std::vector<Edge>& edge_list, double tolerance, int max_iterations) const {
    if (!is_valid()) throw std::invalid_argument("Invalid Shape");
    std::function<double(double, double, int)> search = [&](double lo, double hi, int iteration) -> double {
        const double mid = (lo + hi) / 2;
        if (iteration == max_iterations) {
            throw std::runtime_error("Failed to find the max value within " + std::to_string(tolerance) +
                                     " in " + std::to_string(max_iterations));
        }
        bool works = true;
        try {
            BRepFilletAPI_MakeFillet builder(obj_);
            for (const auto& e : edge_list) builder.Add(mid, TopoDS::Edge(e.wrapped()));
            builder.Build();
            works = builder.IsDone() && Shape(make_3d_result(builder.Shape())).is_valid();
        } catch (const Standard_Failure&) {
            works = false;
        }
        if (!works) return search(lo, mid, iteration + 1);
        if (mid - lo <= tolerance) return mid;
        return search(mid, hi, iteration + 1);
    };
    return search(0.0, 2 * bounding_box().diagonal(), 0);
}

// ---------------------------------------------------------------------------
// Solid
// ---------------------------------------------------------------------------

Solid::Solid(const TopoDS_Shape& obj) : Mixin3D(obj) {
    if (!obj.IsNull() && obj.ShapeType() != TopAbs_SOLID) throw std::invalid_argument("Solid requires a TopoDS_Solid");
}

Solid::Solid(const Shell& shell) {
    ShapeFix_Solid fixer;
    obj_ = occt("Solid", [&] { return TopoDS_Shape(fixer.SolidFromShell(TopoDS::Shell(shell.wrapped()))); });
}

Solid Solid::make_box(double length, double width, double height, const Plane& plane) {
    return Solid(occt("Solid.make_box",
                      [&] { return BRepPrimAPI_MakeBox(plane.to_gp_ax2(), length, width, height).Shape(); }));
}

Solid Solid::make_cylinder(double radius, double height, const Plane& plane, double angle) {
    return Solid(occt("Solid.make_cylinder", [&] {
        return BRepPrimAPI_MakeCylinder(plane.to_gp_ax2(), radius, height, angle * DEG2RAD).Shape();
    }));
}

Solid Solid::make_cone(double base_radius, double top_radius, double height, const Plane& plane, double angle) {
    return Solid(occt("Solid.make_cone", [&] {
        return BRepPrimAPI_MakeCone(plane.to_gp_ax2(), base_radius, top_radius, height, angle * DEG2RAD).Shape();
    }));
}

Solid Solid::make_sphere(double radius, const Plane& plane, double angle1, double angle2, double angle3) {
    return Solid(occt("Solid.make_sphere", [&] {
        return BRepPrimAPI_MakeSphere(plane.to_gp_ax2(), radius, angle1 * DEG2RAD, angle2 * DEG2RAD,
                                      angle3 * DEG2RAD)
            .Shape();
    }));
}

Solid Solid::make_torus(double major_radius, double minor_radius, const Plane& plane, double start_angle,
                        double end_angle, double major_angle) {
    return Solid(occt("Solid.make_torus", [&] {
        return BRepPrimAPI_MakeTorus(plane.to_gp_ax2(), major_radius, minor_radius, start_angle * DEG2RAD,
                                     end_angle * DEG2RAD, major_angle * DEG2RAD)
            .Shape();
    }));
}

Solid Solid::make_wedge(double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax,
                        const Plane& plane) {
    return Solid(occt("Solid.make_wedge", [&] {
        return TopoDS_Shape(BRepPrimAPI_MakeWedge(plane.to_gp_ax2(), dx, dy, dz, xmin, zmin, xmax, zmax).Solid());
    }));
}

Solid Solid::make_loft(const std::vector<Shape>& sections, bool ruled) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& s : sections) topods.push_back(s.wrapped());
    return Solid(unwrap(detail::make_loft(topods, true, ruled), true));
}

Solid Solid::extrude(const Face& obj, const Vector& direction) {
    return Solid(extrude_shape(obj.wrapped(), direction));
}

Solid Solid::extrude_taper(const Face& profile, const Vector& direction, double taper, bool flip_inner) {
    const Plane profile_plane(profile);
    if (direction.normalized() == profile.normal_at() && profile_plane.z_dir.Z > 0 && taper > 0 &&
        profile.inner_wires().empty()) {
        return Solid(occt("extrude_taper", [&] {
            LocOpe_DPrism prism(TopoDS::Face(profile.wrapped()), direction.length() / std::cos(taper * DEG2RAD),
                                taper * DEG2RAD);
            return unwrap(prism.Shape(), true);
        }));
    }
    const double offset_amt = -direction.length() * std::tan(taper * DEG2RAD);
    std::vector<Wire> profile_wires{profile.outer_wire()};
    for (const auto& w : profile.inner_wires()) profile_wires.push_back(w);
    std::vector<Solid> solids;
    for (std::size_t i = 0; i < profile_wires.size(); ++i) {
        const double flip = (i > 0 && flip_inner) ? -1.0 : 1.0;
        const Wire local = profile_plane.to_local_coords(profile_wires[i]);
        const Wire local_taper = local.offset_2d(flip * offset_amt, Kind::INTERSECTION);
        Wire taper_wire = profile_plane.from_local_coords(local_taper);
        taper_wire.move(Location(direction));
        solids.push_back(Solid::make_loft({profile_wires[i], taper_wire}));
    }
    if (solids.size() == 1) return solids[0];
    std::vector<TopoDS_Shape> tools;
    for (std::size_t i = 1; i < solids.size(); ++i) tools.push_back(solids[i].wrapped());
    return Solid(bool_op(BoolOp::CUT, {solids[0].wrapped()}, tools));
}

Solid Solid::extrude_until(const Face& profile, const Shape& target, const Vector& direction_in, Until until) {
    Vector direction = direction_in;
    if (until == Until::PREVIOUS || until == Until::FIRST) {
        direction = direction * -1.0;
        until = until == Until::PREVIOUS ? Until::NEXT : Until::LAST;
    }
    const double max_dimension =
        BoundBox::from_topo_ds(make_compound({profile.wrapped(), target.wrapped()}), TOLERANCE, true).diagonal();
    const Solid extrusion = Solid::extrude(profile, direction * max_dimension);

    std::vector<TopoDS_Shape> modified_target_faces;
    occt("extrude_until", [&] {
        BRepAlgoAPI_Common common(target.wrapped(), extrusion.wrapped());
        common.Build();
        TopExp_Explorer face_exp(common.Shape(), TopAbs_FACE);
        if (!face_exp.More()) throw std::invalid_argument("No intersection: extrusion does not contact target");
        Handle(BRepTools_History) history = common.History();
        for (TopExp_Explorer exp(target.wrapped(), TopAbs_FACE); exp.More(); exp.Next()) {
            for (const auto& m : history->Modified(exp.Current())) modified_target_faces.push_back(m);
        }
        return 0;
    });
    const TopoDS_Shape sewed = sew_faces(modified_target_faces);
    std::vector<std::pair<double, Shape>> surfaces;
    for (const auto& s : top_level(sewed)) {
        if (s.ShapeType() != TopAbs_FACE && s.ShapeType() != TopAbs_SHELL) {
            throw std::runtime_error("Invalid sewn shape");
        }
        const Shape surface(s);
        surfaces.emplace_back(surface.distance_to(profile), surface);
    }
    if (surfaces.empty()) throw std::invalid_argument("No intersection: extrusion does not contact target");
    std::stable_sort(surfaces.begin(), surfaces.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    const Shape& limit = until == Until::NEXT ? surfaces.front().second : surfaces.back().second;
    const Keep keep = until == Until::NEXT ? Keep::TOP : Keep::BOTTOM;
    auto pieces = extrusion.split(limit, keep);
    if (pieces.empty()) throw std::runtime_error("Extrusion is None");
    if (pieces.size() == 1) return Solid(pieces[0].wrapped());
    const Axis axis(profile.center(), direction);
    return Solid(pieces.sort_by(axis)[0].wrapped());
}

Solid Solid::extrude_linear_with_rotation(const Face& section, const Vector& center, const Vector& normal,
                                          double angle) {
    auto extrude_aux_spine = [](const TopoDS_Wire& wire, const TopoDS_Wire& spine, const TopoDS_Wire& aux) {
        BRepOffsetAPI_MakePipeShell builder(spine);
        builder.SetMode(aux, false);
        builder.Add(wire);
        builder.Build();
        builder.MakeSolid();
        return builder.Shape();
    };
    return Solid(occt("extrude_linear_with_rotation", [&] {
        const Wire outer = section.outer_wire();
        const auto inner = section.inner_wires();
        const Wire straight = Wire::combine({Edge::make_line(center, center + normal)})[0];
        const double pitch = 360.0 / angle * normal.length();
        const Wire aux(Edge::make_helix(pitch, normal.length(), 1, center, normal));
        const TopoDS_Shape outer_solid = extrude_aux_spine(TopoDS::Wire(outer.wrapped()),
                                                           TopoDS::Wire(straight.wrapped()), TopoDS::Wire(aux.wrapped()));
        std::vector<TopoDS_Shape> inner_solids;
        for (const auto& w : inner) {
            inner_solids.push_back(extrude_aux_spine(TopoDS::Wire(w.wrapped()), TopoDS::Wire(straight.wrapped()),
                                                     TopoDS::Wire(aux.wrapped())));
        }
        if (inner_solids.empty()) return unwrap(outer_solid, true);
        return unwrap(BRepAlgoAPI_Cut(outer_solid, make_compound(inner_solids)).Shape(), true);
    }));
}

Solid Solid::revolve(const Face& section, double angle, const Axis& axis) {
    return Solid(occt("Solid.revolve", [&] {
        BRepPrimAPI_MakeRevol builder(section.wrapped(), axis.wrapped(), angle * DEG2RAD, true);
        return unwrap(builder.Shape(), true);
    }));
}

Solid Solid::sweep(const Shape& section, const Shape& path, bool make_solid, bool is_frenet,
                   std::optional<Vector> binormal_dir, const std::optional<Wire>& binormal_wire,
                   Transition transition) {
    const Wire path_wire = as_wire(path);
    std::vector<Wire> wires;
    if (section.wrapped().ShapeType() == TopAbs_FACE) {
        const Face face(section.wrapped());
        wires.push_back(face.outer_wire());
        for (const auto& w : face.inner_wires()) wires.push_back(w);
    } else {
        wires.push_back(as_wire(section));
    }
    std::vector<TopoDS_Shape> shapes;
    for (const auto& wire : wires) {
        shapes.push_back(occt("Solid.sweep", [&] {
            BRepOffsetAPI_MakePipeShell builder(TopoDS::Wire(path_wire.wrapped()));
            bool rotate = false;
            set_sweep_mode(builder, path_wire, is_frenet, binormal_dir, binormal_wire, rotate);
            builder.SetTransitionMode(to_occ(transition));
            builder.Add(wire.wrapped(), false, rotate);
            builder.Build();
            if (make_solid) builder.MakeSolid();
            return unwrap(builder.Shape(), true);
        }));
    }
    if (shapes.size() > 1) {
        std::vector<TopoDS_Shape> tools(shapes.begin() + 1, shapes.end());
        return Solid(bool_op(BoolOp::CUT, {shapes[0]}, tools));
    }
    return Solid(shapes[0]);
}

Solid Solid::sweep_multi(const std::vector<Shape>& profiles, const Shape& path, bool make_solid, bool is_frenet,
                         std::optional<Vector> binormal_dir, const std::optional<Wire>& binormal_wire) {
    const Wire path_wire = as_wire(path);
    return Solid(occt("Solid.sweep_multi", [&] {
        BRepOffsetAPI_MakePipeShell builder(TopoDS::Wire(path_wire.wrapped()));
        bool rotate = false;
        set_sweep_mode(builder, path_wire, is_frenet, binormal_dir, binormal_wire, rotate);
        for (const auto& profile : profiles) {
            const TopoDS_Shape wire = profile.wrapped().ShapeType() == TopAbs_WIRE
                                          ? profile.wrapped()
                                          : Face(profile.wrapped()).outer_wire().wrapped();
            builder.Add(wire, false, rotate);
        }
        builder.Build();
        if (make_solid) builder.MakeSolid();
        return unwrap(builder.Shape(), true);
    }));
}

Solid Solid::thicken(const Shape& surface, double depth, std::optional<Vector> normal_override) {
    double adjusted = depth;
    if (surface.wrapped().ShapeType() == TopAbs_FACE && normal_override) {
        const Face face(surface.wrapped());
        const Vector n = face.normal_at(face.center()).normalized();
        if (n.dot(normal_override->normalized()) < 0) adjusted = -depth;
    }
    return Solid(occt("Error applying thicken to given surface", [&] {
        BRepOffset_MakeOffset builder;
        builder.Initialize(surface.wrapped(), adjusted, 1.0e-5, BRepOffset_Skin, true, false, GeomAbs_Intersection,
                           true, true);
        builder.MakeOffsetShape();
        if (!builder.IsDone()) throw std::runtime_error("thicken failed");
        return unwrap(builder.Shape(), true);
    }));
}

Solid Solid::from_bounding_box(const BoundBox& bbox) {
    Solid box = make_box(bbox.size.X, bbox.size.Y, bbox.size.Z);
    box.locate(Location(bbox.min));
    return box;
}

double Solid::volume() const { return Shape::volume(); }

Solid Solid::draft(const std::vector<Face>& faces, const Plane& neutral_plane, double angle) const {
    for (const auto& f : faces) {
        const GeomType t = f.geom_type();
        if (t != GeomType::PLANE && t != GeomType::CYLINDER && t != GeomType::CONE) {
            throw std::invalid_argument("Face has unsupported geometry type " + build123d::to_string(t) +
                                        ". Only PLANAR, CYLINDRICAL, and CONICAL faces are supported.");
        }
    }
    BRepOffsetAPI_DraftAngle builder(obj_);
    for (const auto& f : faces) {
        occt("draft", [&] {
            builder.Add(TopoDS::Face(f.wrapped()), neutral_plane.z_dir.to_dir(), angle * DEG2RAD,
                        neutral_plane.wrapped(), true);
            return 0;
        });
        if (!builder.AddDone()) {
            throw DraftAngleError("Draft could not be added to a face.", f, Shape(builder.ProblematicShape()));
        }
    }
    TopoDS_Shape drafted;
    try {
        builder.Build();
        drafted = builder.Shape();
    } catch (const Standard_Failure&) {
        throw DraftAngleError("Draft build failed on the given solid.", std::nullopt, Shape(builder.ProblematicShape()));
    }
    if (drafted.IsNull() || drafted.ShapeType() != TopAbs_SOLID) {
        throw DraftAngleError("Draft build failed on the given solid.", std::nullopt, Shape(builder.ProblematicShape()));
    }
    Solid result(drafted);
    result.set_history(ShapeHistory::from_algorithm(builder, {obj_}, drafted));
    return result;
}

DraftAngleError::DraftAngleError(const std::string& message, std::optional<Face> face_in, Shape problematic)
    : std::runtime_error(message), face(std::move(face_in)), problematic_shape(std::move(problematic)) {}

}  // namespace build123d
