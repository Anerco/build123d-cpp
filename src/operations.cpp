#include <algorithm>
#include <cmath>
#include <numeric>

#include <TopoDS.hxx>

#include "build123d/operations.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

std::vector<TopoDS_Shape> topods_of(const std::vector<Solid>& solids) {
    std::vector<TopoDS_Shape> out;
    for (const auto& s : solids) out.push_back(s.wrapped());
    return out;
}

TopoDS_Shape compound_of_top_level(const std::vector<Shape>& shapes) {
    std::vector<TopoDS_Shape> out;
    for (const auto& s : shapes) {
        for (const auto& t : top_level(s.wrapped())) out.push_back(t);
    }
    return make_compound(out);
}

int static_dims_of(const std::vector<const Shape*>& objs) {
    int dim = -2;
    for (const Shape* o : objs) {
        const int d = o->dim();
        if (dim == -2) {
            dim = d;
        } else if (d != dim) {
            return -1;
        }
    }
    return dim;
}

/// The shape a selection of sub-shapes was taken from.
Shape parent_of(const Shape& s, const char* operation) {
    auto parent = s.topo_parent();
    if (!parent) {
        throw std::invalid_argument(std::string("Nothing to ") + operation +
                                    ": select edges/vertices from a shape, e.g. part.edges()");
    }
    return *parent;
}

Wire target_wire(const Shape& target) {
    const auto type = target.wrapped().ShapeType();
    if (type == TopAbs_WIRE) return Wire(target.wrapped());
    if (type == TopAbs_EDGE) return Wire(Edge(target.wrapped()));
    return Curve(target.wrapped()).wires().at(0);
}

std::vector<Vertex> without_end_vertices(const Wire& wire, const std::vector<Vertex>& vertices) {
    if (wire.is_closed()) return vertices;
    const Vector p0 = wire.position_at(0), p1 = wire.position_at(1);
    std::vector<Vertex> out;
    for (const auto& v : vertices) {
        const Vector p(v);
        if ((p - p0).length() > 1e-14 && (p - p1).length() > 1e-14) out.push_back(v);
    }
    return out;
}

bool contains_vertex(const std::vector<Vertex>& list, const Vertex& v) {
    return std::any_of(list.begin(), list.end(), [&](const Vertex& o) { return o.is_same(v); });
}

}  // namespace

// ---------------------------------------------------------------------------
// extrude / revolve / loft / section / thicken
// ---------------------------------------------------------------------------

Part extrude(const Shape& to_extrude, double amount, const ExtrudeOptions& options) {
    return extrude(as_faces(to_extrude), amount, options);
}

namespace {

std::vector<Plane> face_planes(const std::vector<Face>& faces, const std::optional<Vector>& dir) {
    std::vector<Plane> planes;
    for (const auto& face : faces) {
        if (dir) {
            planes.emplace_back(face.center(), face.center_location().x_axis().direction, *dir);
        } else {
            if (!face.is_planar()) throw std::invalid_argument("dir must be provided when extruding non-planar faces");
            planes.emplace_back(face);
        }
    }
    return planes;
}

Part finish_extrusion(std::vector<Solid> solids, bool both, bool clean) {
    std::vector<TopoDS_Shape> shapes = topods_of(solids);
    if (both && shapes.size() > 1) {
        const TopoDS_Shape last = shapes.back();
        shapes.pop_back();
        shapes = {bool_op(BoolOp::FUSE, {last}, shapes)};
    }
    std::vector<TopoDS_Shape> result;
    for (const auto& s : shapes) {
        for (const auto& solid : entities(s, TopAbs_SOLID)) result.push_back(clean ? clean_shape(solid) : solid);
    }
    return Part(make_compound(result));
}

}  // namespace

Part extrude(const std::vector<Face>& faces, double amount, const ExtrudeOptions& options) {
    // As build123d: no faces (an empty Sketch or list) give an empty Part
    const auto planes = face_planes(faces, options.dir);
    std::vector<Solid> solids;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (double direction : options.both ? std::vector<double>{1, -1} : std::vector<double>{1}) {
            const Vector d = planes[i].z_dir * (amount * direction);
            solids.push_back(options.taper == 0.0 ? Solid::extrude(faces[i], d)
                                                  : Solid::extrude_taper(faces[i], d, options.taper));
        }
    }
    return finish_extrusion(solids, options.both, options.clean);
}

Part extrude(const Shape& to_extrude, Until until, const Shape& target, const ExtrudeOptions& options) {
    return extrude(as_faces(to_extrude), until, target, options);
}

Part extrude(const std::vector<Face>& faces, Until until, const Shape& target, const ExtrudeOptions& options) {
    const auto planes = face_planes(faces, options.dir);
    std::vector<Solid> solids;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (double direction : options.both ? std::vector<double>{1, -1} : std::vector<double>{1}) {
            if (target.is_null()) throw std::invalid_argument("A target object must be provided");
            solids.push_back(Solid::extrude_until(faces[i], target, planes[i].z_dir * direction, until));
        }
    }
    return finish_extrusion(solids, options.both, options.clean);
}

namespace {

Part revolve_faces(const std::vector<Face>& profiles, const Axis& axis, double revolution_arc, bool clean) {
    const double sign = revolution_arc >= 0 ? 1.0 : -1.0;
    double angle = std::fmod(revolution_arc, sign * 360.0);
    if (angle == 0.0) angle = sign * 360.0;
    std::vector<TopoDS_Shape> solids;
    for (const auto& face : profiles) solids.push_back(Solid::revolve(face, angle, axis).wrapped());
    TopoDS_Shape result = make_compound(solids);
    if (clean) result = clean_shape(result);
    return Part(result);
}

}  // namespace

// As build123d: an empty list raises, an empty Sketch gives an empty Part
Part revolve(const Shape& profiles, const Axis& axis, double revolution_arc, bool clean) {
    return revolve_faces(as_faces(profiles), axis, revolution_arc, clean);
}

Part revolve(const std::vector<Face>& profiles, const Axis& axis, double revolution_arc, bool clean) {
    if (profiles.empty()) throw std::invalid_argument("No profiles provided");
    return revolve_faces(profiles, axis, revolution_arc, clean);
}

Part revolve(const std::vector<Shape>& profiles, const Axis& axis, double revolution_arc, bool clean) {
    if (profiles.empty()) throw std::invalid_argument("No profiles provided");
    std::vector<Face> faces;
    for (const auto& p : profiles) {
        for (const auto& f : as_faces(p)) faces.push_back(f);
    }
    return revolve_faces(faces, axis, revolution_arc, clean);
}

Part loft(const std::vector<Shape>& sections, bool ruled, bool clean) {
    if (sections.empty()) throw std::invalid_argument("No sections provided");
    const auto is_vertex = [](const Shape& s) { return !s.is_null() && s.wrapped().ShapeType() == TopAbs_VERTEX; };
    if (std::any_of(sections.begin(), sections.end(), is_vertex)) {
        if (!is_vertex(sections.front()) && !is_vertex(sections.back())) {
            throw std::invalid_argument("Vertices must be the first, last, or first and last elements");
        }
        for (std::size_t i = 1; i + 1 < sections.size(); ++i) {
            if (is_vertex(sections[i])) {
                throw std::invalid_argument("Vertices must be the first, last, or first and last elements");
            }
        }
    }
    std::vector<Shape> loft_sections;
    std::vector<Face> faces;
    for (const auto& s : sections) {
        if (is_vertex(s)) {
            loft_sections.push_back(s);
            continue;
        }
        for (const auto& face : s.faces()) {
            loft_sections.push_back(face.outer_wire());
            faces.push_back(face);
        }
    }
    std::vector<std::size_t> hole_counts;
    for (const auto& f : faces) hole_counts.push_back(f.inner_wires().size());
    const bool any_holes = std::any_of(hole_counts.begin(), hole_counts.end(), [](std::size_t c) { return c > 0; });
    if (any_holes && std::any_of(hole_counts.begin(), hole_counts.end(), [&](std::size_t c) { return c != hole_counts[0]; })) {
        throw std::invalid_argument("All sections must have the same number of inner wires");
    }
    std::vector<std::vector<Wire>> hole_sections;
    if (any_holes) {
        for (const auto& w : faces[0].inner_wires()) hole_sections.push_back({w});
        for (std::size_t fi = 1; fi < faces.size(); ++fi) {
            const auto inner = faces[fi].inner_wires();
            std::vector<std::size_t> perm(hole_sections.size());
            std::iota(perm.begin(), perm.end(), 0);
            std::vector<std::size_t> best = perm;
            double best_quality = 1e300;
            do {
                double quality = 0.0;
                for (std::size_t j = 0; j < perm.size(); ++j) {
                    quality += (hole_sections[j].back().center() - inner[perm[j]].center()).length();
                }
                if (quality < best_quality) {
                    best_quality = quality;
                    best = perm;
                }
            } while (std::next_permutation(perm.begin(), perm.end()));
            for (std::size_t j = 0; j < best.size(); ++j) hole_sections[j].push_back(inner[best[j]]);
        }
    }
    // Perform the lofts
    Shape new_solid = Solid::make_loft(loft_sections, ruled);
    if (!hole_sections.empty()) {
        std::vector<Shape> holes;
        for (const auto& hs : hole_sections) holes.push_back(Solid::make_loft(std::vector<Shape>(hs.begin(), hs.end()), ruled));
        new_solid = new_solid.cut(holes);
    }
    // Try to recover an invalid loft
    if (!new_solid.is_valid()) {
        try {
            const auto solid_faces = new_solid.faces();
            std::vector<Face> recovery(solid_faces.begin(), solid_faces.end());
            for (const auto& s : loft_sections) {
                if (s.wrapped().ShapeType() == TopAbs_WIRE) recovery.emplace_back(Wire(s.wrapped()));
            }
            new_solid = Solid(Shell(recovery));
            if (clean) new_solid = new_solid.clean();
            if (!new_solid.is_valid()) throw std::runtime_error("Recovery failed");
        } catch (const std::exception&) {
            throw std::runtime_error("Failed to create valid loft");
        }
    }
    // (build123d cleans here in algebra mode, also after a recovery)
    if (clean) new_solid = new_solid.clean();
    Part result(make_compound({new_solid.wrapped()}));
    result.set_history(ShapeHistory::of(new_solid));
    return result;
}

Part loft(const Shape& sections, bool ruled, bool clean) {
    return loft(std::vector<Shape>{sections}, ruled, clean);
}

Sketch section(const Shape& obj, const Plane& section_by, double height, bool clean) {
    return section(obj, std::vector<Plane>{section_by}, height, clean);
}

Sketch section(const Shape& obj, const std::vector<Plane>& section_by, double height, bool clean) {
    if (obj.is_null()) throw std::invalid_argument("No object to section");
    const BoundBox bbox = obj.bounding_box(std::nullopt, false);
    double max_size = 0.0;
    for (double v : {bbox.min.X, bbox.min.Y, bbox.min.Z, bbox.max.X, bbox.max.Y, bbox.max.Z}) {
        max_size = std::max(max_size, std::abs(v));
    }
    max_size += bbox.diagonal();
    std::vector<TopoDS_Shape> faces;
    for (const auto& plane : section_by) {
        const Face cutter = Face::make_rect(2 * max_size, 2 * max_size,
                                            Plane(plane.origin + plane.z_dir * height, std::nullopt, plane.z_dir));
        for (const auto& element : children(obj.wrapped())) {
            const TopoDS_Shape common = bool_op(BoolOp::COMMON, {element}, {cutter.wrapped()});
            for (const auto& f : entities(common, TopAbs_FACE)) faces.push_back(clean ? clean_shape(f) : f);
        }
    }
    return Sketch(make_compound(faces));
}

Part thicken(const Shape& to_thicken, double amount, std::optional<Vector> normal_override, bool both, bool clean) {
    return thicken(as_faces(to_thicken), amount, normal_override, both, clean);
}

Part thicken(const std::vector<Face>& faces, double amount, std::optional<Vector> normal_override, bool both,
             bool clean) {
    // As build123d: no faces (an empty Sketch or list) give an empty Part
    std::vector<TopoDS_Shape> solids;
    for (const auto& face : faces) {
        const Vector normal = normal_override ? *normal_override : face.normal_at();
        for (double direction : both ? std::vector<double>{1, -1} : std::vector<double>{1}) {
            solids.push_back(Solid::thicken(face, amount, normal * direction).wrapped());
        }
    }
    if (solids.size() > 1) {
        const std::vector<TopoDS_Shape> tools(solids.begin() + 1, solids.end());
        solids = {bool_op(BoolOp::FUSE, {solids[0]}, tools)};
    }
    if (clean) {
        for (auto& s : solids) s = clean_shape(s);
    }
    return Part(make_compound(solids));
}

Part draft(const std::vector<Face>& faces, const Plane& neutral_plane, double angle) {
    std::string unsupported;
    for (const auto& f : faces) {
        const GeomType t = f.geom_type();
        if (t != GeomType::PLANE && t != GeomType::CYLINDER && t != GeomType::CONE) {
            unsupported += (unsupported.empty() ? "" : ", ") + build123d::to_string(t);
        }
    }
    if (!unsupported.empty()) throw std::invalid_argument("Draft not supported on face(s) with geometry: " + unsupported);
    // All the faces must come from the same Solid
    std::vector<Shape> topo_parents;
    for (const auto& f : faces) {
        const auto p = f.topo_parent();
        if (!p) continue;
        if (std::none_of(topo_parents.begin(), topo_parents.end(), [&](const Shape& o) { return o.is_same(*p); })) {
            topo_parents.push_back(*p);
        }
    }
    if (topo_parents.size() != 1) throw std::invalid_argument("All faces must share the same topological parent (a Solid)");
    const auto parent_solids = topo_parents[0].solids();
    if (parent_solids.size() != 1) throw std::invalid_argument("Topological parent must be a single Solid");
    Solid new_solid;
    try {
        new_solid = parent_solids[0].draft(faces, neutral_plane, angle);
    } catch (const DraftAngleError& err) {
        throw DraftAngleError("Draft operation failed. Use `err.face` and `err.problematic_shape` for more information.",
                              err.face, err.problematic_shape);
    }
    Part result(make_compound({new_solid.wrapped()}));
    result.set_history(ShapeHistory::of(new_solid));
    return result;
}

Plane project_workplane(const Vector& origin, const Vector& x_dir_in, const Vector& projection_dir_in, double distance) {
    const Vector x_dir = x_dir_in.normalized();
    const Vector projection_dir = projection_dir_in.normalized();
    // A preliminary workplane without x direction set
    const Vector workplane_origin = origin + projection_dir * distance;
    Plane workplane(workplane_origin, std::nullopt, projection_dir);
    // Project a point off the origin to find the projected x direction
    const Face screen = Face::make_rect(1e9, 1e9, workplane);
    const auto projection = screen.find_intersection_points(Axis(origin + x_dir, projection_dir));
    if (projection.empty()) throw std::invalid_argument("x_dir perpendicular to projection_dir");
    workplane.set_x_dir(projection[0].first - workplane_origin);
    return workplane;
}

// ---------------------------------------------------------------------------
// Sketch operations
// ---------------------------------------------------------------------------

Sketch make_face(const Shape& edges) { return make_face(std::vector<Edge>(edges.edges())); }

Sketch make_face(const std::vector<Edge>& edges) {
    // (build123d's message for an empty list)
    if (edges.empty()) throw std::invalid_argument("No objects to create a hull");
    Face face(Wire::combine(edges).at(0));
    if (face.normal_at().Z < 0) face = -face;
    return Sketch(face);
}

Sketch make_face(const std::vector<Shape>& edges) {
    std::vector<Edge> outer_edges;
    for (const auto& item : edges) {
        for (const auto& e : item.edges()) outer_edges.push_back(e);
    }
    return make_face(outer_edges);
}

Sketch make_hull(const Shape& edges) { return make_hull(std::vector<Edge>(edges.edges())); }

Sketch make_hull(const std::vector<Edge>& edges) {
    if (edges.empty()) throw std::invalid_argument("No objects to create a hull");
    Face face(Wire::make_convex_hull(edges));
    if (face.normal_at().Z < 0) face = -face;
    return Sketch(face);
}

Sketch trace(const Shape& lines, double line_width) { return trace(std::vector<Edge>(lines.edges()), line_width); }

Sketch trace(const std::vector<Edge>& lines, double line_width) {
    // build123d indexes the (empty) list of traced faces
    if (lines.empty()) throw std::out_of_range("list index out of range");
    std::vector<TopoDS_Shape> faces;
    for (const auto& wire : Wire::combine(lines)) {
        const Edge pen = wire.perpendicular_line(line_width, 0);
        for (const auto& f : Shell::sweep(pen, wire, Transition::RIGHT).faces()) faces.push_back(f.wrapped());
    }
    if (faces.size() > 1) {
        const std::vector<TopoDS_Shape> tools(faces.begin() + 1, faces.end());
        return Sketch(bool_op(BoolOp::FUSE, {faces[0]}, tools));
    }
    return Sketch(make_compound(faces));
}

Sketch trace(const std::vector<Shape>& lines, double line_width) {
    std::vector<Edge> trace_edges;
    for (const auto& l : lines) {
        for (const auto& e : l.edges()) trace_edges.push_back(e);
    }
    return trace(trace_edges, line_width);
}

// ---------------------------------------------------------------------------
// fillet / chamfer
// ---------------------------------------------------------------------------

namespace {

/// target.fillet / target.chamfer on the topo_parent's own shape (a Solid or a Compound),
/// wrapped as build123d does: Part(Compound([new_part])) made by the new part's record.
template <class Op>
Part part_from_parent(const Shape& target, Op op) {
    Shape new_part;
    if (target.wrapped().ShapeType() == TopAbs_SOLID) {
        new_part = op(Solid(target.wrapped()));
    } else {
        new_part = op(Compound(target.wrapped()));
    }
    Part result(make_compound({new_part.wrapped()}));
    result.set_history(ShapeHistory::of(new_part));
    return result;
}

/// The faces of a 2D target with the given vertices filleted/chamfered, as a Compound
/// made by the merged records of the changed faces.
template <class FaceOp>
Compound faces_with_corners(const Shape& target, const std::vector<Vertex>& vertices, FaceOp op) {
    std::vector<Face> new_faces;
    for (const auto& face : target.faces()) {
        std::vector<Vertex> in_face;
        for (const auto& v : face.vertices()) {
            if (contains_vertex(vertices, v)) in_face.push_back(v);
        }
        new_faces.push_back(in_face.empty() ? face : op(face, in_face));
    }
    std::vector<TopoDS_Shape> shapes;
    std::vector<const Shape*> made;
    for (const auto& f : new_faces) {
        shapes.push_back(f.wrapped());
        made.push_back(&f);
    }
    Compound result(make_compound(shapes));
    result.set_history(ShapeHistory::of(made));
    return result;
}

bool truthy(const std::optional<double>& v) { return v && *v != 0.0; }

}  // namespace

Part fillet(const std::vector<Edge>& edges, double radius) {
    if (edges.empty()) throw std::invalid_argument("No objects provided");
    const Shape target = parent_of(edges[0], "fillet");
    if (target.dim() == 2 || target.dim() == 1) {
        throw std::invalid_argument(target.dim() == 2 ? "2D fillet operation takes only Vertices"
                                                      : "1D fillet operation takes only Vertices");
    }
    return part_from_parent(target, [&](const auto& t) -> Shape { return t.fillet(radius, edges); });
}

Part fillet(const Edge& edge, double radius) { return fillet(std::vector<Edge>{edge}, radius); }

Compound fillet(const std::vector<Vertex>& vertices, double radius) {
    if (vertices.empty()) throw std::invalid_argument("No objects provided");
    const Shape target = parent_of(vertices[0], "fillet");
    const int dim = target.dim();
    if (dim == 3) throw std::invalid_argument("3D fillet operation takes only Edges");
    if (dim == 2) {
        return faces_with_corners(target, vertices, [&](const Face& face, const std::vector<Vertex>& in_face) {
            return face.fillet_2d(radius, in_face);
        });
    }
    if (dim == 1) {
        const Wire wire = target_wire(target);
        const Wire new_wire = wire.fillet_2d(radius, without_end_vertices(wire, vertices));
        return made_by(Compound(new_wire.wrapped()), new_wire.history());
    }
    throw std::invalid_argument("Invalid object dimension");
}

Compound fillet(const Vertex& vertex, double radius) { return fillet(std::vector<Vertex>{vertex}, radius); }

Part chamfer(const std::vector<Edge>& edges, double length, std::optional<double> length2, std::optional<double> angle,
             const std::optional<Face>& reference) {
    if (truthy(length2) && truthy(angle)) throw std::invalid_argument("Only one of length2 or angle should be provided");
    if (truthy(angle)) length2 = length * std::tan(*angle * DEG2RAD);
    if (reference && !reference->is_null() && !(truthy(length2) || truthy(angle))) {
        throw std::invalid_argument("reference can only be used in conjunction with length2 or angle");
    }
    const double l2 = length2.value_or(length);
    if (edges.empty()) throw std::invalid_argument("No objects provided");
    const Shape target = parent_of(edges[0], "chamfer");
    if (target.dim() == 2 || target.dim() == 1) throw std::invalid_argument("2D/1D chamfer operation takes only Vertices");
    return part_from_parent(target, [&](const auto& t) -> Shape { return t.chamfer(length, l2, edges, reference); });
}

Part chamfer(const Edge& edge, double length, std::optional<double> length2, std::optional<double> angle,
             const std::optional<Face>& reference) {
    return chamfer(std::vector<Edge>{edge}, length, length2, angle, reference);
}

Compound chamfer(const std::vector<Vertex>& vertices, double length, std::optional<double> length2,
                 std::optional<double> angle, const std::optional<Edge>& reference) {
    if (truthy(length2) && truthy(angle)) throw std::invalid_argument("Only one of length2 or angle should be provided");
    if (truthy(angle)) length2 = length * std::tan(*angle * DEG2RAD);
    if (reference && !reference->is_null() && !(truthy(length2) || truthy(angle))) {
        throw std::invalid_argument("reference can only be used in conjunction with length2 or angle");
    }
    const double l2 = length2.value_or(length);
    if (vertices.empty()) throw std::invalid_argument("No objects provided");
    const Shape target = parent_of(vertices[0], "chamfer");
    const int dim = target.dim();
    if (dim == 3) throw std::invalid_argument("3D chamfer operation takes only Edges");
    if (dim == 2) {
        return faces_with_corners(target, vertices, [&](const Face& face, const std::vector<Vertex>& in_face) {
            return face.chamfer_2d(length, l2, in_face, reference);
        });
    }
    if (dim == 1) {
        const Wire wire = target_wire(target);
        const Wire new_wire = wire.chamfer_2d(length, l2, without_end_vertices(wire, vertices), reference);
        return made_by(Compound(new_wire.wrapped()), new_wire.history());
    }
    throw std::invalid_argument("Invalid object dimension");
}

Compound chamfer(const Vertex& vertex, double length, std::optional<double> length2, std::optional<double> angle,
                 const std::optional<Edge>& reference) {
    return chamfer(std::vector<Vertex>{vertex}, length, length2, angle, reference);
}

// ---------------------------------------------------------------------------
// mirror / scale / split / offset / bounding_box / sweep
// ---------------------------------------------------------------------------

namespace detail {

TopoDS_Shape mirror_impl(const std::vector<const Shape*>& objs, const Plane& about) {
    std::vector<Shape> out;
    for (const Shape* o : objs) {
        if (!o->is_null()) out.push_back(o->mirror(about));
    }
    return compound_of_top_level(out);
}

TopoDS_Shape scale_impl(const std::vector<const Shape*>& objs, double by, const std::optional<Vector>& about) {
    std::vector<Shape> out;
    for (const Shape* o : objs) {
        if (!o->is_null()) out.push_back(o->scale(by, about));
    }
    return compound_of_top_level(out);
}

TopoDS_Shape scale_impl(const std::vector<const Shape*>& objs, const Vector& by, const std::optional<Vector>& about) {
    std::vector<Shape> out;
    for (const Shape* o : objs) {
        if (!o->is_null()) out.push_back(o->scale(by, about));
    }
    return compound_of_top_level(out);
}

TopoDS_Shape split_impl(const std::vector<const Shape*>& objs, const Plane& bisect_by, Keep keep) {
    std::vector<Shape> out;
    for (const Shape* o : objs) {
        for (const auto& piece : o->split(bisect_by, keep)) out.push_back(piece);
    }
    return compound_of_top_level(out);
}

TopoDS_Shape split_impl(const std::vector<const Shape*>& objs, const Shape& bisect_by, Keep keep) {
    std::vector<Shape> out;
    for (const Shape* o : objs) {
        for (const auto& piece : o->split(bisect_by, keep)) out.push_back(piece);
    }
    return compound_of_top_level(out);
}

TopoDS_Shape offset_impl(const std::vector<const Shape*>& objs, double amount, const OffsetOptions& opt) {
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<Solid> solids;
    for (const Shape* o : objs) {
        if (o->is_null()) continue;
        const TopoDS_Shape& s = o->wrapped();
        switch (s.ShapeType()) {
            case TopAbs_COMPOUND: {
                // Compound.get_type: the children of that type of the child compounds and of the
                // compound itself (Compound.compounds()); the edges first, then those of the wires
                std::vector<TopoDS_Shape> compounds;
                for (const auto& child : children(s)) {
                    if (child.ShapeType() == TopAbs_COMPOUND) compounds.push_back(child);
                }
                compounds.push_back(s);
                const auto of_type = [&](TopAbs_ShapeEnum type) {
                    std::vector<TopoDS_Shape> out;
                    for (const auto& comp : compounds) {
                        for (const auto& child : children(comp)) {
                            if (child.ShapeType() == type) out.push_back(child);
                        }
                    }
                    return out;
                };
                for (const auto& e : of_type(TopAbs_EDGE)) edges.emplace_back(e);
                for (const auto& w : of_type(TopAbs_WIRE)) {
                    for (const auto& e : Wire(w).edges()) edges.push_back(e);
                }
                for (const auto& f : of_type(TopAbs_FACE)) faces.emplace_back(f);
                for (const auto& so : of_type(TopAbs_SOLID)) solids.emplace_back(so);
                break;
            }
            case TopAbs_SOLID: solids.emplace_back(s); break;
            case TopAbs_FACE: faces.emplace_back(s); break;
            case TopAbs_EDGE: edges.emplace_back(s); break;
            case TopAbs_WIRE:
                for (const auto& e : Wire(s).edges()) edges.push_back(e);
                break;
            default: throw std::invalid_argument("Unsupported type for offset");
        }
    }
    std::vector<TopoDS_Shape> results;
    for (const auto& face : faces) {
        Wire outer = face.outer_wire().offset_2d(amount, opt.kind);
        if (opt.min_edge_length) outer = outer.fix_degenerate_edges(*opt.min_edge_length);
        std::vector<Wire> inner_wires;
        for (const auto& inner : face.inner_wires()) {
            // build123d skips the holes whose offset fails in OCCT (not its ValueErrors)
            try {
                Wire w = inner.offset_2d(-amount, opt.kind);
                if (opt.min_edge_length) w = w.fix_degenerate_edges(*opt.min_edge_length);
                inner_wires.push_back(w);
            } catch (const std::runtime_error&) {
            }
        }
        Face new_face(outer);
        if ((new_face.normal_at() - face.normal_at()).length() > 0.001) new_face = -new_face;
        if (inner_wires.empty()) {
            results.push_back(new_face.wrapped());
        } else {
            std::vector<TopoDS_Shape> inner_faces;
            for (const auto& w : inner_wires) inner_faces.push_back(Face(w).wrapped());
            const TopoDS_Shape subtraction = bool_op(BoolOp::CUT, {new_face.wrapped()}, inner_faces);
            // build123d fails (calling Face.unwrap) when the holes leave other than one face
            if (subtraction.ShapeType() == TopAbs_COMPOUND) {
                throw std::invalid_argument("'Face' object has no attribute 'unwrap'");
            }
            results.push_back(subtraction);
        }
    }
    if (!edges.empty()) {
        Wire w;
        if (edges.size() == 1 && edges[0].geom_type() == GeomType::LINE) {
            const Edge& e = edges[0];
            w = Wire(std::vector<Edge>{Edge::make_line(e.position_at(0), e.position_at(0.5)),
                                       Edge::make_line(e.position_at(0.5), e.position_at(1))})
                    .offset_2d(amount, opt.kind, opt.side, opt.closed);
        } else {
            w = Wire(edges).offset_2d(amount, opt.kind, opt.side, opt.closed);
        }
        if (opt.min_edge_length) w = w.fix_degenerate_edges(*opt.min_edge_length);
        results.insert(results.begin(), w.wrapped());
    }
    for (const auto& solid : solids) {
        std::vector<Face> openings;
        const auto solid_faces = solid.faces();
        for (const auto& o : opt.openings) {
            if (solid_faces.contains(o)) openings.push_back(o);
        }
        results.push_back(solid.offset_3d(openings, amount, 1e-4, opt.kind).fix().wrapped());
    }
    return make_compound(results);
}

TopoDS_Shape bounding_box_impl(const std::vector<const Shape*>& objs) {
    std::vector<TopoDS_Shape> out;
    if (static_dims_of(objs) == 2) {
        for (const Shape* o : objs) {
            const BoundBox b = o->bounding_box();
            out.push_back(Face(Wire::make_polygon({Vector(b.min.X, b.min.Y), Vector(b.min.X, b.max.Y),
                                                   Vector(b.max.X, b.max.Y), Vector(b.max.X, b.min.Y),
                                                   Vector(b.min.X, b.min.Y)}))
                              .wrapped());
        }
        return make_compound(out);
    }
    for (const Shape* o : objs) {
        if (o->is_null() || o->wrapped().ShapeType() == TopAbs_VERTEX) continue;
        const BoundBox b = o->bounding_box();
        out.push_back(Solid::make_box(b.size.X, b.size.Y, b.size.Z, Plane(b.min)).wrapped());
    }
    return make_compound(out);
}

Wire sweep_path_wire(const std::vector<Edge>& path) {
    try {
        return Wire(path);
    } catch (const std::invalid_argument&) {
        throw std::invalid_argument("Unable to build path from edges");
    }
}

TopoDS_Shape sweep_one_impl(const Shape& sections, const Shape& path, const SweepOptions& options) {
    // build123d iterates a Compound (Curve, Sketch, Part) given as the sections: its children
    // are the sections
    std::vector<Shape> section_list;
    if (!sections.is_null() && sections.wrapped().ShapeType() == TopAbs_COMPOUND) {
        for (const auto& child : children(sections.wrapped())) section_list.emplace_back(child);
    } else {
        section_list.push_back(sections);
    }
    std::vector<const Shape*> ptrs;
    for (const auto& s : section_list) ptrs.push_back(&s);
    return sweep_impl(ptrs, path, options);
}

TopoDS_Shape sweep_impl(const std::vector<const Shape*>& sections, const Shape& path, const SweepOptions& opt) {
    if (path.is_null()) throw std::invalid_argument("path must be provided");
    Wire path_wire;
    if (path.wrapped().ShapeType() == TopAbs_WIRE) {
        path_wire = Wire(path.wrapped());
    } else if (path.wrapped().ShapeType() == TopAbs_COMPOUND) {
        path_wire = sweep_path_wire(path.edges());  // build123d's Wire(curve) of an iterable path
    } else {
        path_wire = Wire(std::vector<Edge>(path.edges()));
    }
    if (sections.empty()) throw std::invalid_argument("No sections provided");
    std::vector<const Shape*> curve_sections;
    std::vector<Face> face_list;
    for (const Shape* s : sections) {
        if (s->is_null()) continue;
        if (s->dim() == 1) {
            curve_sections.push_back(s);
        } else {
            for (const auto& f : s->faces()) face_list.push_back(f);
        }
    }
    std::vector<TopoDS_Shape> solids;
    // A binormal curve takes precedence over a fixed normal, as in build123d.
    const std::optional<Vector> normal = opt.binormal ? std::nullopt : opt.normal;
    if (!face_list.empty()) {
        if (opt.multisection) {
            std::vector<Shape> wires;
            for (const auto& f : face_list) wires.push_back(f.outer_wire());
            solids.push_back(Solid::sweep_multi(wires, path_wire, true, opt.is_frenet, normal, opt.binormal).wrapped());
        } else {
            for (const auto& f : face_list) {
                solids.push_back(
                    Solid::sweep(f, path_wire, true, opt.is_frenet, normal, opt.binormal, opt.transition).wrapped());
            }
        }
    }
    // With curves among the sections build123d sweeps every section (faces too) into a shell
    std::vector<TopoDS_Shape> faces;
    if (!curve_sections.empty()) {
        for (const Shape* s : sections) {
            for (const auto& f : Shell::sweep(*s, path_wire, opt.transition).faces()) faces.push_back(f.wrapped());
        }
    }
    if (opt.clean) {
        for (auto& s : solids) s = clean_shape(s);
        for (auto& f : faces) f = clean_shape(f);
    }
    return make_compound(solids.empty() ? faces : solids);
}

}  // namespace detail

}  // namespace build123d
