// Ports of build123d's Builder._add_to_context, _PublicationService.place and the builder-mode
// paths of the operations (build_common.py, operations_generic.py, operations_part.py,
// operations_sketch.py of build123d 0.13).
#include "builder.hpp"

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>

#include "../py_compat.hpp"
#include "data.hpp"

namespace bd_warehouse::detail {

namespace {

const char* tag_of(int dim) {
    switch (dim) {
        case 3: return "BuildPart";
        case 2: return "BuildSketch";
        default: return "BuildLine";
    }
}

/// The builder's shape class around a TopoDS shape (Python's self._sub_class(wrapped)).
Shape sub_class(int dim, const TopoDS_Shape& wrapped) {
    switch (dim) {
        case 3: return Part(wrapped);
        case 2: return Sketch(wrapped);
        default: return Curve(wrapped);
    }
}

Shape compound_of(int dim, const std::vector<Shape>& shapes) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& s : shapes) {
        if (!s.is_null()) topods.push_back(s.wrapped());
    }
    TopoDS_Compound comp;
    BRep_Builder builder;
    builder.MakeCompound(comp);
    for (const auto& s : topods) builder.Add(comp, s);
    return sub_class(dim, comp);
}

bool is_identity(const std::vector<Location>& locations) {
    return locations.size() == 1 && locations[0] == Location();
}

}  // namespace

void Builder::add(const std::vector<Shape>& objects, Mode mode, bool clean, bool faces_to_pending) {
    if (mode == Mode::PRIVATE || objects.empty()) return;

    // Typed lists: the objects by class, then the contents of the compounds
    std::vector<Shape> edges, wires, faces, solids, compounds;
    for (const auto& object : objects) {
        if (object.is_null()) {
            // an empty Compound (Python: a Part()/Sketch()/Curve() with no shape) adds nothing;
            // other empty shapes are None in Python, which is skipped too
            continue;
        }
        switch (object.wrapped().ShapeType()) {
            case TopAbs_EDGE: edges.push_back(object); break;
            case TopAbs_WIRE: wires.push_back(object); break;
            case TopAbs_FACE: faces.push_back(object); break;
            case TopAbs_SOLID: solids.push_back(object); break;
            case TopAbs_COMPOUND: compounds.push_back(object); break;
            default:
                throw std::invalid_argument(std::string(tag_of(dim)) + " doesn't accept {" +
                                            build123d::to_string(object.shape_type()) + "}");
        }
    }
    for (const auto& c : compounds) {
        const Compound compound(c.wrapped());
        for (const auto& e : compound.get_type<Edge>()) edges.push_back(e);
        for (const auto& w : compound.get_type<Wire>()) wires.push_back(w);
        for (const auto& f : compound.get_type<Face>()) faces.push_back(f);
        for (const auto& s : compound.get_type<Solid>()) solids.push_back(s);
    }

    // Align sketch planar faces with Plane.XY
    if (dim == 2) {
        std::vector<Shape> aligned;
        for (const auto& f : faces) {
            Face new_face(f.wrapped());
            if (!new_face.is_coplanar(Plane::XY)) {
                // Try to keep the x direction, if not allow it to be assigned automatically
                std::optional<Plane> plane;
                try {
                    plane = Plane(Vector(0, 0, 0), Vector(1, 0, 0), new_face.normal_at());
                } catch (const std::exception&) {
                    plane = Plane(Vector(0, 0, 0), std::nullopt, new_face.normal_at());
                }
                new_face = plane->to_local_coords(new_face);
                new_face = new_face.moved(Location(Vector(0, 0, -new_face.center().Z)));
            }
            if (new_face.normal_at().Z > 0) {
                aligned.push_back(new_face);
            } else {
                aligned.push_back(-new_face);
            }
        }
        faces = aligned;
    }

    // Convert wires to edges
    for (const auto& w : wires) {
        for (const auto& e : Wire(w.wrapped()).edges()) edges.push_back(e);
    }

    // Allow faces to be combined with solids for section operations
    if (!faces_to_pending) {
        solids.insert(solids.end(), faces.begin(), faces.end());
        faces.clear();
    }

    std::vector<Shape>& mine = dim == 3 ? solids : dim == 2 ? faces : edges;
    if (!mine.empty()) {
        std::optional<Shape> combined;
        bool needs_clean = clean;
        switch (mode) {
            case Mode::ADD:
                if (!obj) {
                    if (mine.size() == 1) {
                        combined = mine[0];
                    } else {
                        const Shape last = mine.back();
                        const std::vector<Shape> rest(mine.begin(), mine.end() - 1);
                        combined = last.fuse(rest);
                        needs_clean = false;
                    }
                } else {
                    combined = obj->fuse(mine);
                    needs_clean = false;
                }
                break;
            case Mode::SUBTRACT:
                if (!obj) throw std::runtime_error("Nothing to subtract from");
                combined = obj->cut(mine);
                needs_clean = false;
                break;
            case Mode::INTERSECT: {
                if (!obj) throw std::runtime_error("Nothing to intersect with");
                const ShapeList<Shape> common = obj->intersect(Compound(mine));
                if (!common.empty()) combined = compound_of(dim, std::vector<Shape>(common.begin(), common.end()));
                needs_clean = false;
                break;
            }
            case Mode::REPLACE: combined = compound_of(dim, mine); break;
            case Mode::PRIVATE: break;
        }
        obj = combined ? *combined : sub_class(dim, TopoDS_Shape());
        if (needs_clean && !obj->is_null()) obj = obj->clean();
    }

    // Cast to the builder's type (Curve, Sketch or Part)
    if (obj) {
        if (!obj->is_null() && obj->wrapped().ShapeType() == TopAbs_COMPOUND) {
            obj = sub_class(dim, obj->wrapped());
        } else if (obj->is_null()) {
            obj = sub_class(dim, TopoDS_Shape());
        } else {
            std::vector<Shape> shapes;
            if (dim == 3) {
                for (const auto& s : obj->solids()) shapes.push_back(s);
            } else if (dim == 2) {
                for (const auto& f : obj->faces()) shapes.push_back(f);
            } else {
                for (const auto& e : obj->edges()) shapes.push_back(e);
            }
            obj = compound_of(dim, shapes);
        }
    }

    // Add to pending
    if (dim == 3) {
        for (const auto& e : edges) pending_edges.emplace_back(e.wrapped());
        for (const auto& f : faces) {
            const Face face(f.wrapped());
            Plane plane = Plane::XY;
            try {
                plane = Plane(face);
            } catch (const std::invalid_argument&) {
            }
            pending_faces.push_back(face);
            pending_face_planes.push_back(plane);
        }
    } else if (dim == 2) {
        for (const auto& e : edges) pending_edges.emplace_back(e.wrapped());
    }
}

Part Builder::part() const { return obj ? Part(obj->wrapped()) : Part(); }
Sketch Builder::sketch() const { return obj ? Sketch(obj->wrapped()) : Sketch(); }
Curve Builder::line() const { return obj ? Curve(obj->wrapped()) : Curve(); }
Shape Builder::product() const { return obj ? sub_class(dim, obj->wrapped()) : sub_class(dim, TopoDS_Shape()); }

ShapeList<Vertex> Builder::vertices() const { return obj ? obj->vertices() : ShapeList<Vertex>(); }
ShapeList<Edge> Builder::edges() const { return obj ? obj->edges() : ShapeList<Edge>(); }
ShapeList<Face> Builder::faces() const { return obj ? obj->faces() : ShapeList<Face>(); }
ShapeList<Solid> Builder::solids() const { return obj ? obj->solids() : ShapeList<Solid>(); }
ShapeList<Wire> Builder::wires() const {
    if (!obj) return {};
    if (dim == 1) return Wire::combine(std::vector<Edge>(obj->edges()));
    return obj->wires();
}

Face Builder::face() const {
    const auto all = faces();
    if (all.size() != 1) throw std::invalid_argument("Expected exactly one face, found " + std::to_string(all.size()));
    return all[0];
}

Edge Builder::edge() const {
    const auto all = edges();
    if (all.size() != 1) throw std::invalid_argument("Expected exactly one edge, found " + std::to_string(all.size()));
    return all[0];
}

Wire Builder::wire() const {
    const auto all = wires();
    if (all.size() != 1) throw std::invalid_argument("Expected exactly one wire, found " + std::to_string(all.size()));
    return all[0];
}

void Builder::publish(const Shape& product, Mode mode, const std::vector<Location>& publication,
                      const std::vector<Location>& output) {
    if (product.is_null() || mode == Mode::PRIVATE) return;
    add(place(product, publication, output), mode);
}

Shape place(const Shape& product, const std::vector<Location>& publication, const std::vector<Location>& output) {
    if (product.is_null()) return product;
    if (is_identity(publication) && is_identity(output)) return product;
    std::vector<Shape> placed;
    for (const auto& p : publication) {
        for (const auto& o : output) placed.push_back(product.moved(p * o));
    }
    if (placed.size() == 1) return placed[0];
    const int dim = product.dim();
    TopoDS_Compound comp;
    BRep_Builder builder;
    builder.MakeCompound(comp);
    for (const auto& s : placed) builder.Add(comp, s.wrapped());
    Shape result = dim == 3 ? Shape(Part(comp)) : dim == 2 ? Shape(Sketch(comp)) : dim == 1 ? Shape(Curve(comp)) : Shape(Compound(comp));
    result.label = product.label;
    return result;
}

std::vector<Location> compose(const std::vector<Location>& outer, const std::vector<Location>& inner) {
    std::vector<Location> out;
    for (const auto& o : outer) {
        for (const auto& i : inner) out.push_back(o * i);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Operations in builder mode
// ---------------------------------------------------------------------------

void make_face(Builder& sketch, Mode mode) {
    if (sketch.pending_edges.empty()) throw std::invalid_argument("No objects to create a hull");
    make_face(sketch, sketch.pending_edges, mode);
}

void make_face(Builder& sketch, const std::vector<Edge>& edges, Mode mode) {
    if (edges.empty()) throw std::invalid_argument("No objects to create a hull");
    Face pending_face(Wire::combine(edges).at(0));
    if (pending_face.normal_at().Z < 0) pending_face = -pending_face;  // flip up-side-down faces
    sketch.add(pending_face, mode);
    sketch.pending_edges.clear();
}

void loft(Builder& part, bool ruled, bool clean, Mode mode) {
    if (part.pending_faces.empty()) throw std::invalid_argument("No sections provided");
    const std::vector<Shape> sections(part.pending_faces.begin(), part.pending_faces.end());
    part.pending_faces.clear();
    part.pending_face_planes.clear();
    // build123d's loft makes the solid (cleaned when it's recovered), then adds it with `clean`
    const Part lofted = build123d::loft(sections, ruled, false);
    part.add(lofted, mode, clean);
}

namespace {

void extrude_faces(Builder& part, const std::vector<Face>& faces, const std::vector<Plane>& planes, double amount,
                   const ExtrudeOptions& o, Mode mode) {
    std::vector<Shape> new_solids;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (const double direction : o.both ? std::vector<double>{1, -1} : std::vector<double>{1}) {
            if (o.taper == 0) {
                new_solids.push_back(Solid::extrude(faces[i], planes[i].z_dir * amount * direction));
            } else {
                new_solids.push_back(Solid::extrude_taper(faces[i], planes[i].z_dir * amount * direction, o.taper));
            }
        }
    }
    if (o.both && new_solids.size() > 1) {
        const Shape last = new_solids.back();
        new_solids.pop_back();
        const Shape fused = last.fuse(new_solids);
        new_solids = {fused};
    }
    if (o.clean) {
        for (auto& s : new_solids) s = s.clean();
    }
    part.add(new_solids, mode, o.clean);
}

}  // namespace

void extrude(Builder& part, double amount, const ExtrudeOptions& options, Mode mode) {
    if (part.pending_faces.empty()) throw std::invalid_argument("A face or sketch must be provided");
    std::vector<Face> faces = part.pending_faces;
    std::vector<Plane> planes = part.pending_face_planes;
    part.pending_faces.clear();
    part.pending_face_planes.clear();
    if (options.dir) {
        planes.clear();
        for (const auto& face : faces) planes.emplace_back(face.center(), face.center_location().x_axis().direction, *options.dir);
    }
    if (planes.size() != faces.size()) throw std::invalid_argument("dir must be provided when extruding non-planar faces");
    extrude_faces(part, faces, planes, amount, options, mode);
}

void extrude(Builder& part, const Shape& to_extrude, double amount, const ExtrudeOptions& options, Mode mode) {
    std::vector<Face> faces;
    for (const auto& f : to_extrude.faces()) faces.push_back(f);
    std::vector<Plane> planes;
    if (options.dir) {
        for (const auto& face : faces) planes.emplace_back(face.center(), face.center_location().x_axis().direction, *options.dir);
    } else {
        for (const auto& face : faces) {
            try {
                planes.emplace_back(face);
            } catch (const std::invalid_argument&) {
            }
        }
    }
    if (planes.size() != faces.size()) throw std::invalid_argument("dir must be provided when extruding non-planar faces");
    extrude_faces(part, faces, planes, amount, options, mode);
}

void revolve(Builder& part, const Axis& axis, double revolution_arc, bool clean, Mode mode) {
    // for OCCT not assuming that a 0 degree revolve means a 360 degree revolve
    const double sign = revolution_arc >= 0 ? 1.0 : -1.0;
    double angle = build123d::detail::py_mod(revolution_arc, sign * 360.0);
    angle = angle == 0 ? sign * 360.0 : angle;
    if (part.pending_faces.empty()) throw std::invalid_argument("No profiles provided");
    const std::vector<Face> profiles = part.pending_faces;
    part.pending_faces.clear();
    part.pending_face_planes.clear();
    std::vector<Shape> new_solids;
    for (const auto& profile : profiles) new_solids.push_back(Solid::revolve(profile, angle, axis));
    part.add(new_solids, mode, clean);
}

void fillet(Builder& builder, const std::vector<Shape>& objects, double radius) {
    if (!builder.obj || builder.obj->is_null()) throw std::invalid_argument("Nothing to fillet");
    if (builder.dim == 3) {
        std::vector<Edge> edges;
        for (const auto& o : objects) {
            if (o.shape_type() != ShapeType::EDGE) throw std::invalid_argument("3D fillet operation takes only Edges");
            edges.emplace_back(o.wrapped());
        }
        const Part target(builder.obj->wrapped());
        builder.add(target.fillet(radius, edges), Mode::REPLACE);
    } else if (builder.dim == 2) {
        std::vector<Vertex> vertices;
        for (const auto& o : objects) {
            if (o.shape_type() != ShapeType::VERTEX) throw std::invalid_argument("2D fillet operation takes only Vertices");
            vertices.emplace_back(o.wrapped());
        }
        std::vector<Shape> new_faces;
        for (const auto& face : builder.obj->faces()) {
            std::vector<Vertex> in_face;
            for (const auto& v : face.vertices()) {
                if (std::find(vertices.begin(), vertices.end(), v) != vertices.end()) in_face.push_back(v);
            }
            new_faces.push_back(in_face.empty() ? Shape(face) : Shape(face.fillet_2d(radius, in_face)));
        }
        builder.add(compound_of(2, new_faces), Mode::REPLACE);
    } else {
        const Wire target = builder.wires().at(0);
        std::vector<Vertex> vertices;
        for (const auto& o : objects) {
            if (o.shape_type() != ShapeType::VERTEX) throw std::invalid_argument("1D fillet operation takes only Vertices");
            const Vertex v(o.wrapped());
            if (!target.is_closed() && ((Vector(v) - target.position_at(0)).length() <= 1e-9 ||
                                        (Vector(v) - target.position_at(1)).length() <= 1e-9)) {
                continue;  // end vertices can't be filleted
            }
            vertices.push_back(v);
        }
        builder.add(target.fillet_2d(radius, vertices), Mode::REPLACE);
    }
}

void chamfer(Builder& builder, const std::vector<Shape>& objects, double length, std::optional<double> length2,
             std::optional<double> angle, const std::optional<Shape>& reference) {
    if (!builder.obj || builder.obj->is_null()) throw std::invalid_argument("Nothing to chamfer");
    // (Python tests the truthiness of length2 and angle)
    const bool has_length2 = length2 && *length2 != 0;
    const bool has_angle = angle && *angle != 0;
    if (has_length2 && has_angle) throw std::invalid_argument("Only one of length2 or angle should be provided");
    if (has_angle) length2 = length * std::tan(radians(*angle));
    if (reference && !(has_length2 || has_angle)) {
        throw std::invalid_argument("reference can only be used in conjunction with length2 or angle");
    }
    const double second = length2 ? *length2 : length;
    if (builder.dim == 3) {
        std::vector<Edge> edges;
        for (const auto& o : objects) {
            if (o.shape_type() != ShapeType::EDGE) throw std::invalid_argument("3D chamfer operation takes only Edges");
            edges.emplace_back(o.wrapped());
        }
        std::optional<Face> face;
        if (reference) face = Face(reference->wrapped());
        const Part target(builder.obj->wrapped());
        builder.add(target.chamfer(length, second, edges, face), Mode::REPLACE);
    } else if (builder.dim == 2) {
        std::vector<Vertex> vertices;
        for (const auto& o : objects) {
            if (o.shape_type() != ShapeType::VERTEX) throw std::invalid_argument("2D chamfer operation takes only Vertices");
            vertices.emplace_back(o.wrapped());
        }
        std::vector<Shape> new_faces;
        for (const auto& face : builder.obj->faces()) {
            std::vector<Vertex> in_face;
            for (const auto& v : face.vertices()) {
                if (std::find(vertices.begin(), vertices.end(), v) != vertices.end()) in_face.push_back(v);
            }
            std::optional<Edge> edge_reference;
            if (reference) edge_reference = Edge(reference->wrapped());
            new_faces.push_back(in_face.empty() ? Shape(face)
                                                : Shape(face.chamfer_2d(length, second, in_face, edge_reference)));
        }
        builder.add(compound_of(2, new_faces), Mode::REPLACE);
    } else {
        const Wire target = builder.wires().at(0);
        std::vector<Vertex> vertices;
        for (const auto& o : objects) {
            if (o.shape_type() != ShapeType::VERTEX) throw std::invalid_argument("1D fillet operation takes only Vertices");
            const Vertex v(o.wrapped());
            if (!target.is_closed() && ((Vector(v) - target.position_at(0)).length() <= 1e-9 ||
                                        (Vector(v) - target.position_at(1)).length() <= 1e-9)) {
                continue;
            }
            vertices.push_back(v);
        }
        std::optional<Edge> edge_reference;
        if (reference) edge_reference = Edge(reference->wrapped());
        builder.add(target.chamfer_2d(length, second, vertices, edge_reference), Mode::REPLACE);
    }
}

void mirror(Builder& builder, const std::optional<std::vector<Shape>>& objects, const Plane& about, Mode mode) {
    std::vector<Shape> object_list;
    if (!objects) {
        if (!builder.obj) throw std::invalid_argument("objects must be provided");
        object_list = {*builder.obj};
    } else {
        object_list = *objects;
    }
    std::vector<Shape> mirrored;
    for (const auto& o : object_list) mirrored.push_back(o.mirror(about));
    builder.add(mirrored, mode);
}

void split(Builder& builder, const std::optional<std::vector<Shape>>& objects, const Plane& bisect_by, Keep keep,
           Mode mode) {
    std::vector<Shape> object_list;
    if (!objects) {
        if (!builder.obj) throw std::invalid_argument("objects must be provided");
        object_list = {*builder.obj};
    } else {
        object_list = *objects;
    }
    std::vector<Shape> new_objects;
    for (const auto& o : object_list) {
        for (const auto& piece : o.split(bisect_by, keep)) new_objects.push_back(piece);
    }
    builder.add(new_objects, mode);
}

void offset(Builder& builder, const std::optional<std::vector<Shape>>& objects, double amount,
            const OffsetOptions& options, Mode mode) {
    std::vector<Shape> object_list;
    if (!objects) {
        if (!builder.obj) throw std::invalid_argument("objects must be provided");
        object_list = {*builder.obj};
    } else {
        object_list = *objects;
    }
    const Compound result(build123d::offset(object_list, amount, options).wrapped());
    std::vector<Shape> new_objects;
    for (TopoDS_Iterator it(result.wrapped()); it.More(); it.Next()) new_objects.emplace_back(it.Value());
    builder.add(new_objects, mode);
}

void scale(Builder& builder, const std::optional<std::vector<Shape>>& objects, double by, Mode mode) {
    std::vector<Shape> object_list;
    if (!objects) {
        if (!builder.obj) throw std::invalid_argument("objects must be provided");
        object_list = {*builder.obj};
    } else {
        object_list = *objects;
    }
    std::vector<Shape> new_objects;
    for (const auto& o : object_list) {
        if (!o.is_null()) new_objects.push_back(o.scale(by));
    }
    builder.add(new_objects, mode);
}

void hole(Builder& part, double radius, std::optional<double> depth, const std::vector<Location>& locations,
          Mode mode) {
    double hole_depth;
    if (depth) {
        hole_depth = 2 * *depth;
    } else {
        // _BaseHole._through_hole_depth
        if (!part.obj || part.obj->is_null()) throw std::invalid_argument("No depth provided");
        const Vector part_origin = part.obj->bounding_box().center();
        double origin_distance = 0.0;
        bool first = true;
        for (const auto& location : locations) {
            const double distance = (part_origin - location.position()).length();
            if (first || distance > origin_distance) origin_distance = distance;
            first = false;
        }
        if (first) throw std::invalid_argument("max() arg is an empty sequence");
        hole_depth = 2 * (part.obj->bounding_box().diagonal() + origin_distance);
    }
    // To ensure the hole goes all the way through the part when no depth is given, it starts above
    // the part
    const Vector hole_start = depth ? Vector(0, 0, 0) : Vector(0, 0, hole_depth / 2);
    const Solid solid = Solid::make_cylinder(radius, hole_depth, Plane(hole_start, std::nullopt, Vector(0, 0, -1)));
    const BasePartObject hole_object(solid, Rotation(), Align3(Align::CENTER, Align::CENTER, Align::CENTER));
    part.publish(hole_object, mode, locations);
}

Sketch sweep(Builder& builder, const std::vector<Shape>& sections, const Shape& path, bool clean, Transition transition,
             Mode mode) {
    if (builder.dim != 2) throw std::invalid_argument("sweep of edge sections is only ported for BuildSketch");
    const Wire path_wire =
        path.wrapped().ShapeType() == TopAbs_WIRE ? Wire(path.wrapped()) : Wire(std::vector<Edge>(path.edges()));
    std::vector<Shape> new_faces;
    for (const auto& section : sections) {
        const Shell swept = Shell::sweep(section, path_wire, transition);
        for (const auto& face : swept.faces()) new_faces.push_back(face);
    }
    builder.add(new_faces, mode, clean);
    return Sketch(compound_of(2, new_faces).wrapped());
}

void insert(Builder& builder, const std::vector<Shape>& objects, const std::vector<Location>& locations, Mode mode) {
    // Compounds are inserted unwrapped one level (Compound.unwrap(fully=False))
    std::vector<Shape> object_iter;
    for (const auto& o : objects) {
        if (o.is_null()) continue;
        if (o.wrapped().ShapeType() == TopAbs_COMPOUND) {
            object_iter.push_back(Compound(o.wrapped()).unwrap(false));
        } else {
            object_iter.push_back(o);
        }
    }
    if (builder.dim == 3) {
        std::vector<Edge> new_edges;
        std::vector<Face> new_faces;
        std::vector<Shape> new_solids;
        std::vector<Shape> wires;
        for (const auto& o : object_iter) {
            switch (o.wrapped().ShapeType()) {
                case TopAbs_EDGE: new_edges.emplace_back(o.wrapped()); break;
                case TopAbs_WIRE: wires.push_back(o); break;
                case TopAbs_FACE: new_faces.emplace_back(o.wrapped()); break;
                case TopAbs_SOLID: new_solids.push_back(o); break;
                default: break;
            }
        }
        for (const auto& o : object_iter) {
            if (o.wrapped().ShapeType() != TopAbs_COMPOUND) continue;
            const Compound c(o.wrapped());
            for (const auto& e : c.get_type<Edge>()) new_edges.push_back(e);
            for (const auto& w : c.get_type<Wire>()) wires.push_back(w);
            for (const auto& f : c.get_type<Face>()) new_faces.push_back(f);
            for (const auto& s : c.get_type<Solid>()) new_solids.push_back(s);
        }
        for (const auto& w : wires) {
            for (const auto& e : Wire(w.wrapped()).edges()) new_edges.push_back(e);
        }
        for (const auto& e : new_edges) {
            for (const auto& l : locations) builder.pending_edges.push_back(e.moved(l));
        }
        for (const auto& l : locations) {
            for (const auto& f : new_faces) {
                builder.pending_faces.push_back(f.moved(l));
                builder.pending_face_planes.push_back(Plane::XY);
            }
        }
        std::vector<Shape> located_solids;
        for (const auto& s : new_solids) {
            for (const auto& l : locations) located_solids.push_back(s.moved(l));
        }
        builder.add(located_solids, mode);
    } else {
        std::vector<Shape> new_objects;
        for (const auto& o : object_iter) {
            for (const auto& l : locations) new_objects.push_back(o.moved(l));
        }
        builder.add(new_objects, mode);
    }
}

}  // namespace bd_warehouse::detail
