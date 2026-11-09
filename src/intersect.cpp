// Shape.intersect / _intersect / touch of build123d 0.13 (shape_core.py, zero_d.py,
// one_d.py, two_d.py, three_d.py, composite.py). build123d dispatches on the Python
// class of each operand; here the TopoDS type selects the same implementation.
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepTools.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

TopAbs_ShapeEnum type_of(const Shape& s) { return s.wrapped().ShapeType(); }
bool is_vertex(const Shape& s) { return !s.is_null() && type_of(s) == TopAbs_VERTEX; }
bool is_edge(const Shape& s) { return !s.is_null() && type_of(s) == TopAbs_EDGE; }
bool is_1d(const Shape& s) { return !s.is_null() && (type_of(s) == TopAbs_EDGE || type_of(s) == TopAbs_WIRE); }
bool is_face(const Shape& s) { return !s.is_null() && type_of(s) == TopAbs_FACE; }
bool is_2d(const Shape& s) { return !s.is_null() && (type_of(s) == TopAbs_FACE || type_of(s) == TopAbs_SHELL); }
bool is_solid(const Shape& s) { return !s.is_null() && type_of(s) == TopAbs_SOLID; }
bool is_compound(const Shape& s) {
    return !s.is_null() && (type_of(s) == TopAbs_COMPOUND || type_of(s) == TopAbs_COMPSOLID);
}

/// Shape.as_shape: Vector and Location become a Vertex, Axis an Edge and Plane a Face.
Shape as_shape(const IntersectArg& arg) {
    if (const auto* s = std::get_if<Shape>(&arg)) return *s;
    if (const auto* v = std::get_if<Vector>(&arg)) return Vertex(*v);
    if (const auto* l = std::get_if<Location>(&arg)) return Vertex(l->position());
    if (const auto* a = std::get_if<Axis>(&arg)) return Edge(*a);
    return Face(BRepBuilderAPI_MakeFace(std::get<Plane>(arg).wrapped()).Face());
}

/// Edge.is_infinite / trim_infinite, applied the way every _intersect does: an infinite
/// line is trimmed to a length that spans `self`.
Shape trim_infinite_against(const Shape& self, const Shape& other) {
    if (!is_edge(other)) return other;
    const Edge edge(other.wrapped());
    if (!edge.is_infinite()) return other;
    const BoundBox bbox = self.bounding_box(std::nullopt, false);
    const double half_length = bbox.diagonal() + (edge.center() - bbox.center()).length();
    const Vector origin = edge.center();
    const Vector direction = (edge.end_point() - edge.start_point()).normalized();
    return Edge::make_line(origin - direction * half_length, origin + direction * half_length);
}

/// The direct elements of a compound (build123d's list(compound)), or, for an assembly,
/// its children placed at their global locations.
std::vector<Shape> compound_elements(const Shape& compound) {
    std::vector<Shape> elements;
    if (!compound.children.empty()) {
        for (const auto& c : compound.children) {
            // global_location of a child of this (root) compound
            const Location global = compound.location() * c.location();
            elements.push_back(c.moved(c.location().inverse() * global));
        }
        return elements;
    }
    for (TopoDS_Iterator it(compound.wrapped()); it.More(); it.Next()) elements.emplace_back(it.Value());
    return elements;
}

/// ShapeList.expand: compounds dissolved to their children, wires to edges, shells to
/// faces, nulls removed.
ShapeList<Shape> expand(const ShapeList<Shape>& shapes) {
    ShapeList<Shape> expanded;
    for (const auto& shape : shapes) {
        if (shape.is_null()) continue;
        switch (type_of(shape)) {
            case TopAbs_COMPOUND: {
                ShapeList<Shape> kids;
                for (TopoDS_Iterator it(shape.wrapped()); it.More(); it.Next()) kids.emplace_back(it.Value());
                for (auto& s : expand(kids)) expanded.push_back(std::move(s));
                break;
            }
            case TopAbs_SHELL:
                for (const auto& f : shape.faces()) expanded.push_back(f);
                break;
            case TopAbs_WIRE:
                for (const auto& e : Wire(shape.wrapped()).edges()) expanded.push_back(e);
                break;
            default: expanded.push_back(shape);
        }
    }
    return expanded;
}

/// ShapeList(set(shapes)): duplicates (is_same) removed. build123d's set order is
/// arbitrary; the first occurrence order is kept here.
ShapeList<Shape> unique(const ShapeList<Shape>& shapes) {
    ShapeList<Shape> out;
    OccShapeMap seen;
    for (const auto& s : shapes) {
        if (s.is_null()) continue;
        if (seen.Contains(s.wrapped())) continue;
        seen.Add(s.wrapped());
        out.push_back(s);
    }
    return out;
}

void append(ShapeList<Shape>& to, const ShapeList<Shape>& from) { to.insert(to.end(), from.begin(), from.end()); }

double distance(const Shape& a, const Shape& b) { return a.distance_to(b); }

ShapeList<Shape> intersect_one(const Shape& self, const IntersectArg& other, double tol, bool touched);
ShapeList<Shape> touch_any(const Shape& self, const Shape& other, double tol);
ShapeList<Shape> touch_2d(const Shape& self, const Shape& other, double tol, const ShapeList<Shape>* found_faces,
                          const ShapeList<Shape>* found_edges);

// ---- Vertex._intersect

ShapeList<Shape> intersect_vertex(const Shape& self, const IntersectArg& other_arg, double tol, bool touched) {
    const Vector center = Vertex(self.wrapped()).center();
    if (const auto* axis = std::get_if<Axis>(&other_arg)) {
        return axis->intersect(center) ? ShapeList<Shape>{self} : ShapeList<Shape>{};
    }
    if (const auto* plane = std::get_if<Plane>(&other_arg)) {
        return plane->contains(center, tol) ? ShapeList<Shape>{self} : ShapeList<Shape>{};
    }
    const Shape other = as_shape(other_arg);
    if (is_vertex(other)) return distance(self, other) <= tol ? ShapeList<Shape>{self} : ShapeList<Shape>{};
    return intersect_one(other, self, tol, touched);
}

// ---- Mixin1D._intersect

ShapeList<Shape> intersect_1d(const Shape& self, const IntersectArg& other_arg, double tol, bool touched) {
    ShapeList<Shape> results;
    if (const auto* plane = std::get_if<Plane>(&other_arg)) {
        const Shape face(BRepBuilderAPI_MakeFace(plane->wrapped()).Face());
        append(results, expand(bool_op_list(BoolOp::SECTION, self, face)));
        return results;
    }
    const Shape other = trim_infinite_against(self, as_shape(other_arg));
    if (is_1d(other)) {
        append(results, expand(bool_op_list(BoolOp::COMMON, self, other)));
        for (const auto& s : bool_op_list(BoolOp::SECTION, self, other)) {
            if (is_vertex(s)) results.push_back(s);
        }
    } else if (is_vertex(other)) {
        if (distance(other, self) <= tol) results.push_back(other);
    } else {
        append(results, intersect_one(other, self, tol, touched));
    }
    return results;
}

// ---- Mixin2D._intersect

ShapeList<Shape> intersect_2d(const Shape& self, const IntersectArg& other_arg, double tol, bool touched) {
    const Shape other = trim_infinite_against(self, as_shape(other_arg));
    ShapeList<Shape> results;
    if (is_2d(other)) {
        // Common for coplanar overlap
        const ShapeList<Shape> common_faces = expand(bool_op_list(BoolOp::COMMON, self, other));
        append(results, common_faces);
        // Section for crossing curves (only edges; section vertices are touches)
        ShapeList<Shape> section_edges_in;
        for (const auto& s : bool_op_list(BoolOp::SECTION, self, other)) {
            if (is_edge(s)) section_edges_in.push_back(s);
        }
        const ShapeList<Shape> section_edges = expand(section_edges_in);
        if (common_faces.empty()) {
            append(results, section_edges);
        } else {
            // drop the section edges that bound the common faces
            std::vector<std::pair<Shape, BoundBox>> common_edges;
            for (const auto& face : common_faces) {
                for (const auto& e : face.edges()) common_edges.emplace_back(e, e.bounding_box(std::nullopt, false));
            }
            for (const auto& edge : section_edges) {
                const BoundBox edge_bbox = edge.bounding_box(std::nullopt, false);
                bool is_common = false;
                for (const auto& [ce, ce_bbox] : common_edges) {
                    if (edge_bbox.intersects(ce_bbox, tol) && distance(edge, ce) <= tol) {
                        is_common = true;
                        break;
                    }
                }
                if (!is_common) results.push_back(edge);
            }
        }
    } else if (is_1d(other)) {
        append(results, bool_op_list(BoolOp::SECTION, self, other));
    } else if (is_vertex(other)) {
        if (distance(other, self) <= tol) results.push_back(other);
    } else {
        append(results, intersect_one(other, self, tol, touched));
    }
    if (touched && is_2d(other)) {
        ShapeList<Shape> found_faces, found_edges;
        for (const auto& r : results) {
            if (is_face(r)) found_faces.push_back(r);
            if (is_edge(r)) found_edges.push_back(r);
        }
        append(results, touch_2d(self, other, tol, &found_faces, &found_edges));
    }
    return results;
}

// ---- Mixin3D._intersect (Solid)

ShapeList<Shape> touch_solid(const Shape& self, const Shape& other, double tol, const ShapeList<Shape>* found_solids);

ShapeList<Shape> filter_redundant_touches(const ShapeList<Shape>& items, double tol) {
    ShapeList<Shape> edges, faces, solids;
    for (const auto& r : items) {
        if (is_edge(r)) edges.push_back(r);
        if (is_face(r)) faces.push_back(r);
        if (is_solid(r)) solids.push_back(r);
    }
    ShapeList<Shape> out;
    for (const auto& r : items) {
        if (is_vertex(r)) {
            bool redundant = false;
            for (const auto& e : edges) redundant = redundant || distance(e, r) <= tol;
            for (const auto& f : faces) redundant = redundant || distance(f, r) <= tol;
            for (const auto& s : solids) {
                for (const auto& sf : s.faces()) redundant = redundant || distance(sf, r) <= tol;
            }
            if (redundant) continue;
        }
        if (is_edge(r)) {
            bool on_face = false;
            for (const auto& f : faces) on_face = on_face || f.distance_to(r.center()) <= tol;
            if (on_face) continue;
        }
        out.push_back(r);
    }
    return out;
}

ShapeList<Shape> intersect_3d(const Shape& self, const IntersectArg& other_arg, double tol, bool touched) {
    const Shape other = trim_infinite_against(self, as_shape(other_arg));
    ShapeList<Shape> results;
    if (is_solid(other) || is_2d(other) || is_1d(other)) {
        append(results, expand(bool_op_list(BoolOp::COMMON, self, other)));
    } else if (is_vertex(other)) {
        if (Solid(self.wrapped()).is_inside(Vector(Vertex(other.wrapped())), tol)) results.push_back(other);
    } else {
        // Compound: delegate; the touches are handled by the outer caller
        append(results, intersect_one(other, self, tol, false));
    }
    if (touched && is_solid(self)) {
        append(results, touch_solid(self, other, tol, nullptr));
        results = filter_redundant_touches(unique(results), tol);
    }
    return results;
}

// ---- Compound._intersect (OR semantics over the elements)

ShapeList<Shape> intersect_compound(const Shape& self, const IntersectArg& other_arg, double tol, bool touched) {
    const Shape other = as_shape(other_arg);
    const std::vector<Shape> self_elements = compound_elements(self);
    if (self_elements.empty()) return {};
    const std::vector<Shape> other_elements = is_compound(other) ? compound_elements(other) : std::vector<Shape>{other};
    ShapeList<Shape> results;
    for (const auto& self_elem : self_elements) {
        for (const auto& other_elem : other_elements) append(results, intersect_one(self_elem, other_elem, tol, touched));
    }
    return unique(results);
}

ShapeList<Shape> intersect_one(const Shape& self, const IntersectArg& other, double tol, bool touched) {
    if (self.is_null()) return {};
    switch (type_of(self)) {
        case TopAbs_VERTEX: return intersect_vertex(self, other, tol, touched);
        case TopAbs_EDGE:
        case TopAbs_WIRE: return intersect_1d(self, other, tol, touched);
        case TopAbs_FACE:
        case TopAbs_SHELL: return intersect_2d(self, other, tol, touched);
        case TopAbs_SOLID: return intersect_3d(self, other, tol, touched);
        case TopAbs_COMPOUND:
        case TopAbs_COMPSOLID: return intersect_compound(self, other, tol, touched);
        default: return {};
    }
}

// ---- touch

bool vertex_on_shapes(const Shape& v, const ShapeList<Shape>& shapes, double tol) {
    for (const auto& s : shapes) {
        if (distance(v, s) <= tol) return true;
    }
    return false;
}

bool vertex_duplicate(const Shape& v, const std::vector<Vector>& points) {
    const Vector p = Vertex(v.wrapped()).center();
    for (const auto& q : points) {
        if (p == q) return true;
    }
    return false;
}

std::vector<Vector> vertex_points(const Shape& shape) {
    std::vector<Vector> out;
    for (const auto& v : shape.vertices()) out.push_back(Vector(v));
    return out;
}

ShapeList<Shape> touch_2d(const Shape& self, const Shape& other, double tol, const ShapeList<Shape>* found_faces_in,
                          const ShapeList<Shape>* found_edges_in) {
    ShapeList<Shape> results;
    if (!is_2d(other)) {
        append(results, touch_any(other, self, tol));
        return results;
    }
    ShapeList<Shape> found_faces, found_edges;
    if (found_faces_in == nullptr) {
        for (const auto& r : intersect_2d(self, other, tol, false)) {
            if (is_face(r)) found_faces.push_back(r);
            if (is_edge(r)) found_edges.push_back(r);
        }
    } else {
        found_faces = *found_faces_in;
        if (found_edges_in != nullptr) found_edges = *found_edges_in;
    }
    // all contact points (vertex-vertex, vertex-edge, vertex-face)
    std::vector<Vector> found_vertices;
    occt("touch", [&] {
        BRepExtrema_DistShapeShape extrema;
        extrema.SetDeflection(tol * 1e-3);  // higher precision avoids duplicate solutions
        extrema.LoadS1(self.wrapped());
        extrema.LoadS2(other.wrapped());
        extrema.Perform();
        if (!extrema.IsDone() || extrema.Value() > tol) return 0;
        ShapeList<Shape> self_edges, other_edges;
        for (const auto& e : self.edges()) self_edges.push_back(e);
        for (const auto& e : other.edges()) other_edges.push_back(e);
        const auto self_points = vertex_points(self), other_points = vertex_points(other);
        for (int i = 1; i <= extrema.NbSolution(); ++i) {
            const gp_Pnt p1 = extrema.PointOnShape1(i), p2 = extrema.PointOnShape2(i);
            if (p1.Distance(p2) > tol) continue;
            const Vertex new_vertex(p1.X(), p1.Y(), p1.Z());
            if (vertex_duplicate(new_vertex, found_vertices)) continue;
            // skip edge-edge crossings, but keep corner touches
            if (vertex_on_shapes(new_vertex, self_edges, tol) && vertex_on_shapes(new_vertex, other_edges, tol) &&
                !vertex_duplicate(new_vertex, self_points) && !vertex_duplicate(new_vertex, other_points)) {
                continue;
            }
            if (!vertex_on_shapes(new_vertex, found_faces, tol) && !vertex_on_shapes(new_vertex, found_edges, tol)) {
                results.push_back(new_vertex);
                found_vertices.push_back(new_vertex.center());
            }
        }
        return 0;
    });
    return results;
}

std::pair<Vector, Vector> face_point_normal(const Face& face, double u, double v) {
    double u0, u1, v0, v1;
    BRepTools::UVBounds(TopoDS::Face(face.wrapped()), u0, u1, v0, v1);
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(TopoDS::Face(face.wrapped())).Normal(u0 + u * (u1 - u0), v0 + v * (v1 - v0), p, n);
    return {Vector(p), Vector(n)};
}

bool faces_equal(const Face& f1, const Face& f2, double tol, int grid_size = 4) {
    if (!f1.bounding_box(std::nullopt, false).intersects(f2.bounding_box(std::nullopt, false), tol)) return false;
    for (int i = 0; i < grid_size; ++i) {
        const double u = static_cast<double>(i) / (grid_size - 1);
        for (int j = 0; j < grid_size; ++j) {
            const double v = static_cast<double>(j) / (grid_size - 1);
            const auto [pos1, norm1] = face_point_normal(f1, u, v);
            const auto [pos2, norm2] = face_point_normal(f2, u, v);
            if ((pos1 - pos2).length() > tol || std::abs(norm1.dot(norm2)) < 0.99) return false;
        }
    }
    return true;
}

bool touch_duplicate(const Shape& shape, const ShapeList<Shape>& existing, double tol) {
    if (is_vertex(shape)) {
        const Vector p = Vertex(shape.wrapped()).center();
        for (const auto& v : existing) {
            if (is_vertex(v) && Vertex(v.wrapped()).center() == p) return true;
        }
        return false;
    }
    if (is_edge(shape)) {
        const Edge e(shape.wrapped());
        for (const auto& o : existing) {
            if (is_edge(o) && e.geom_equal(Edge(o.wrapped()), tol)) return true;
        }
        return false;
    }
    if (is_face(shape)) {
        const Face f(shape.wrapped());
        for (const auto& o : existing) {
            if (is_face(o) && faces_equal(f, Face(o.wrapped()), tol)) return true;
        }
        return false;
    }
    return false;
}

ShapeList<Shape> touch_solid(const Shape& self, const Shape& other, double tol, const ShapeList<Shape>* found_solids_in) {
    const auto edge_on_face = [&](const Shape& e, const Shape& f) { return !bool_op_list(BoolOp::COMMON, e, f).empty(); };
    const auto edge_on_faces = [&](const Shape& e, const ShapeList<Shape>& faces) {
        for (const auto& f : faces) {
            if (edge_on_face(e, f)) return true;
        }
        return false;
    };
    ShapeList<Shape> results;
    if (is_solid(other) || is_2d(other)) {
        ShapeList<Shape> intersect_faces;
        if (is_solid(other)) {
            ShapeList<Shape> found_solids;
            if (found_solids_in == nullptr) {
                found_solids = intersect_3d(self, other, tol, false);
            } else {
                found_solids = *found_solids_in;
            }
            for (const auto& s : found_solids) {
                for (const auto& f : s.faces()) intersect_faces.push_back(f);
            }
        }
        std::vector<std::pair<Face, BoundBox>> self_faces, other_faces;
        for (const auto& f : self.faces()) self_faces.emplace_back(f, f.bounding_box(std::nullopt, false));
        for (const auto& f : other.faces()) other_faces.emplace_back(f, f.bounding_box(std::nullopt, false));
        // First pass: touch/intersect results of the face pairs, filtered against the
        // faces of the intersection solids
        ShapeList<Shape> raw_results;
        for (const auto& [sf, sf_bb] : self_faces) {
            for (const auto& [of, of_bb] : other_faces) {
                if (!sf_bb.intersects(of_bb, tol)) continue;
                for (const auto& r : touch_2d(sf, of, tol, nullptr, nullptr)) {
                    if (!touch_duplicate(r, raw_results, tol) && !vertex_on_shapes(r, intersect_faces, tol)) {
                        raw_results.push_back(r);
                    }
                }
                for (const auto& r : sf.intersect(IntersectArg(of), tol)) {
                    if (!touch_duplicate(r, raw_results, tol) && !edge_on_faces(r, intersect_faces)) {
                        raw_results.push_back(r);
                    }
                }
            }
        }
        // Second pass: drop lower-dimensional results bounding higher-dimensional ones
        ShapeList<Shape> all_faces, all_edges;
        for (const auto& r : raw_results) {
            if (is_face(r)) all_faces.push_back(r);
            if (is_edge(r)) all_edges.push_back(r);
        }
        for (const auto& r : raw_results) {
            bool maximal = false;
            if (is_face(r)) {
                maximal = true;
            } else if (is_edge(r)) {
                maximal = !edge_on_faces(r, all_faces);
            } else if (is_vertex(r)) {
                maximal = !vertex_on_shapes(r, all_faces, tol) && !vertex_on_shapes(r, all_edges, tol);
            }
            if (maximal) results.push_back(r);
        }
    } else if (is_1d(other)) {
        std::vector<std::pair<Face, BoundBox>> self_faces;
        for (const auto& f : self.faces()) self_faces.emplace_back(f, f.bounding_box(std::nullopt, false));
        const BoundBox other_bb = other.bounding_box(std::nullopt, false);
        for (const auto& ov : other.vertices()) {
            for (const auto& [sf, bb] : self_faces) {
                if (distance(ov, sf) <= tol) {
                    results.push_back(ov);
                    break;
                }
            }
        }
        // tangent contacts (edge tangent to the surface)
        for (const auto& [sf, sf_bb] : self_faces) {
            if (!sf_bb.intersects(other_bb, tol)) continue;
            occt("touch", [&] {
                BRepExtrema_DistShapeShape extrema(sf.wrapped(), other.wrapped());
                if (!extrema.IsDone() || extrema.Value() > tol) return 0;
                for (int i = 1; i <= extrema.NbSolution(); ++i) {
                    const gp_Pnt p1 = extrema.PointOnShape1(i), p2 = extrema.PointOnShape2(i);
                    if (p1.Distance(p2) > tol) continue;
                    const Vertex new_vertex(p1.X(), p1.Y(), p1.Z());
                    if (!touch_duplicate(new_vertex, results, tol)) results.push_back(new_vertex);
                }
                return 0;
            });
        }
    } else if (is_vertex(other)) {
        for (const auto& sf : self.faces()) {
            if (distance(other, sf) <= tol) {
                results.push_back(other);
                break;
            }
        }
    } else {
        append(results, touch_any(other, self, tol));
    }
    return unique(results);
}

ShapeList<Shape> touch_compound(const Shape& self, const Shape& other, double tol) {
    ShapeList<Shape> results;
    const std::vector<Shape> elements = !self.children.empty() ? self.children : compound_elements(self);
    for (const auto& elem : elements) append(results, touch_any(elem, other, tol));
    return unique(results);
}

ShapeList<Shape> touch_any(const Shape& self, const Shape& other, double tol) {
    if (self.is_null()) return {};
    switch (type_of(self)) {
        case TopAbs_FACE:
        case TopAbs_SHELL: return touch_2d(self, other, tol, nullptr, nullptr);
        case TopAbs_SOLID: return touch_solid(self, other, tol, nullptr);
        case TopAbs_COMPOUND:
        case TopAbs_COMPSOLID: return touch_compound(self, other, tol);
        default: return {};
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Shape.intersect / touch
// ---------------------------------------------------------------------------

ShapeList<Shape> Shape::intersect(const IntersectArg& to_intersect, double tolerance, bool include_touched) const {
    return intersect(std::vector<IntersectArg>{to_intersect}, tolerance, include_touched);
}

ShapeList<Shape> Shape::intersect(const std::vector<IntersectArg>& to_intersect, double tolerance,
                                  bool include_touched) const {
    if (to_intersect.empty()) return {};
    // Chained for AND semantics: c.intersect(s1, s2) = c ∩ s1 ∩ s2
    ShapeList<Shape> common_set{*this};
    for (const auto& other : to_intersect) {
        ShapeList<Shape> next_set;
        for (const auto& obj : common_set) {
            const ShapeList<Shape> result = intersect_one(obj, other, tolerance, include_touched);
            if (!result.empty()) append(next_set, expand(result));
        }
        if (next_set.empty()) return {};  // AND: if any step fails, no intersection
        common_set = unique(next_set);
    }
    return common_set;
}

ShapeList<Shape> Shape::touch(const Shape& other, double tolerance) const { return touch_any(*this, other, tolerance); }

// ---- geometry.intersect(shape) is shape.intersect(geometry)

ShapeList<Shape> Vector::intersect(const Shape& shape) const { return shape.intersect(IntersectArg(*this)); }
ShapeList<Shape> Axis::intersect(const Shape& shape) const { return shape.intersect(IntersectArg(*this)); }
ShapeList<Shape> Location::intersect(const Shape& shape) const { return shape.intersect(IntersectArg(*this)); }
ShapeList<Shape> Plane::intersect(const Shape& shape) const { return shape.intersect(IntersectArg(*this)); }

}  // namespace build123d
