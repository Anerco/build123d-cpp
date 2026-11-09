// The +, -, & operators: build123d's Shape.__add__/__sub__/__and__, Compound.__add__/
// __sub__/__and__ and Mixin1D.__add__ (shape_core.py, composite.py, one_d.py).
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp_Face.hxx>
#include <Geom_Curve.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <TopoDS.hxx>

#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d::detail {

namespace {

const char* dim_name(int d) {
    switch (d) {
        case 0: return "0D";
        case 1: return "1D";
        case 2: return "2D";
        case 3: return "3D";
        default: return "mixed";
    }
}

/// The top-level (non-compound) shapes of the operands, as fresh shapes.
std::vector<Shape> top_level_shapes(const std::vector<const Shape*>& operands) {
    std::vector<Shape> out;
    for (const Shape* s : operands) {
        if (s == nullptr || s->is_null()) continue;
        for (const auto& t : top_level(s->wrapped())) out.emplace_back(t);
    }
    return out;
}

std::vector<TopoDS_Shape> wrapped_of(const std::vector<Shape>& shapes) {
    std::vector<TopoDS_Shape> out;
    for (const auto& s : shapes) out.push_back(s.wrapped());
    return out;
}

/// Shape.make_composite for the pieces of one operation: a compound of them that keeps
/// their record when they all share one.
Shape make_composite(const std::vector<Shape>& pieces) {
    std::vector<TopoDS_Shape> shapes;
    std::shared_ptr<ShapeHistory> record;
    bool single_record = true;
    for (const auto& p : pieces) {
        shapes.push_back(p.wrapped());
        if (p.history()) {
            if (record && record != p.history()) single_record = false;
            record = p.history();
        }
    }
    Shape composite(make_compound(shapes));
    if (record && single_record) composite.set_history(record);
    return composite;
}

// ---- Mixin1D.__add__

/// Wire._make_wire(edges, sequenced=False).
TopoDS_Shape make_wire_py(const std::vector<Edge>& edges) {
    return occt("Wire", [&]() -> TopoDS_Shape {
        BRepBuilderAPI_MakeWire builder;
        OccShapeList combined;
        for (const auto& e : edges) {
            if (!e.is_null()) combined.Append(e.wrapped());
        }
        builder.Add(combined);
        builder.Build();
        if (!builder.IsDone()) {
            switch (builder.Error()) {
                case BRepBuilderAPI_NonManifoldWire:
                    warn("Wire is non manifold (e.g. branching, self intersecting)");
                    break;
                case BRepBuilderAPI_EmptyWire: throw std::runtime_error("Wire is empty");
                case BRepBuilderAPI_DisconnectedWire: throw std::invalid_argument("Edges are disconnected");
                default: break;
            }
        }
        return builder.Wire();
    });
}

/// _record_matching_vertices: the vertices of a rebuilt edge as modified from those at
/// the same points.
void record_matching_vertices(ShapeHistory& history, const TopoDS_Shape& source, const TopoDS_Shape& placed) {
    std::vector<TopoDS_Shape> placed_vertices;
    for (const auto& v : tracked_subshapes({placed})) {
        if (v.ShapeType() == TopAbs_VERTEX) placed_vertices.push_back(v);
    }
    for (const auto& old : tracked_subshapes({source})) {
        if (old.ShapeType() != TopAbs_VERTEX) continue;
        const gp_Pnt old_point = BRep_Tool::Pnt(TopoDS::Vertex(old));
        for (const auto& added : placed_vertices) {
            if (added.IsSame(old)) break;
            if (BRep_Tool::Pnt(TopoDS::Vertex(added)).Distance(old_point) < TOLERANCE) {
                history.wrapped()->AddModified(old, added);
                break;
            }
        }
    }
}

/// _joined_wire: a wire from connected edges, with the record of what became of each.
Shape joined_wire(const std::vector<Edge>& edges, std::vector<TopoDS_Shape> before,
                  std::vector<TopoDS_Shape> brought) {
    const Wire wire(make_wire_py(edges));
    auto history = std::make_shared<ShapeHistory>(nullptr, std::move(before), std::move(brought));
    const auto placed = wire.edges();
    std::vector<Vector> midpoints;
    for (const auto& e : placed) midpoints.push_back(e.position_at(0.5));
    for (const auto& source : edges) {
        const Vector midpoint = source.position_at(0.5);
        std::size_t best = 0;
        double best_d = 1e300;
        for (std::size_t i = 0; i < midpoints.size(); ++i) {
            const double d = (midpoints[i] - midpoint).length();
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        if (midpoints.empty() || best_d > TOLERANCE) continue;
        const TopoDS_Shape& match = placed[best].wrapped();
        if (!match.IsSame(source.wrapped())) history->wrapped()->AddModified(source.wrapped(), match);
        record_matching_vertices(*history, source.wrapped(), match);
    }
    return made_by(Shape(wire.wrapped()), history);
}

/// The edges of a 1D shape as build123d lists them (Wire.edges() is in connection order).
std::vector<Edge> edges_of(const Shape& s) {
    std::vector<Edge> out;
    if (s.is_null()) return out;
    if (s.wrapped().ShapeType() == TopAbs_WIRE) {
        for (const auto& e : Wire(s.wrapped()).edges()) out.push_back(e);
    } else {
        for (const auto& e : s.edges()) out.push_back(e);
    }
    return out;
}

/// summands[0].fuse(*summands[1:]) with the record told which inputs were brought in.
Shape summands_fuse(const std::vector<Shape>& summands, const std::vector<TopoDS_Shape>& brought) {
    const std::vector<Shape> rest(summands.begin() + 1, summands.end());
    Shape fused = summands[0].fuse(rest);
    if (fused.history()) fused.history()->with_inputs({}, brought);
    return fused;
}

/// Mixin1D.__add__ (also Curve.__add__): the result is an Edge, Wire or Curve (here a
/// Shape holding the edge, wire or compound) carrying the record.
Shape mixin1d_add(const Shape& self, const std::vector<const Shape*>& others) {
    std::vector<Shape> summands = top_level_shapes(others);
    if (summands.empty()) return self;
    for (const auto& s : summands) {
        if (topods_dim(s.wrapped()) != 1) throw std::invalid_argument("Only shapes with the same dimension can be added");
    }
    std::vector<Edge> summand_edges;
    for (const auto& s : summands) {
        for (const auto& e : edges_of(s)) summand_edges.push_back(e);
    }
    const std::vector<TopoDS_Shape> brought = wrapped_of(summands);

    Shape sum_shape;
    if (self.is_null()) {
        if (summands.size() == 1) {
            sum_shape = summands[0];
        } else {
            try {
                sum_shape = joined_wire(summand_edges, {}, brought);
            } catch (const std::invalid_argument&) {
                sum_shape = summands_fuse(summands, brought);
            } catch (const std::runtime_error&) {
                sum_shape = summands_fuse(summands, brought);
            }
        }
    } else {
        std::vector<Edge> all_edges = edges_of(self);
        all_edges.insert(all_edges.end(), summand_edges.begin(), summand_edges.end());
        try {
            sum_shape = joined_wire(all_edges, {self.wrapped()}, brought);
        } catch (const std::invalid_argument&) {
            sum_shape = self.fuse(summands);
        } catch (const std::runtime_error&) {
            sum_shape = self.fuse(summands);
        }
    }
    if (clean_enabled()) sum_shape = sum_shape.clean();
    // A single Edge is returned as that Edge, with the record of how it was made
    const auto edges = sum_shape.edges();
    if (edges.size() == 1) sum_shape = made_by(Shape(edges[0].wrapped()), sum_shape.history());
    return sum_shape;
}

}  // namespace

Shape algebra(AlgebraOp op, const Shape& a, const std::vector<const Shape*>& b, int static_dim, SelfKind kind) {
    switch (op) {
        case AlgebraOp::ADD: {
            const int a_dim = static_dim >= 0 ? static_dim : (a ? a.dim() : -1);
            const bool is_1d_shape = !a.is_null() && (a.wrapped().ShapeType() == TopAbs_EDGE ||
                                                      a.wrapped().ShapeType() == TopAbs_WIRE);
            // Curve.__add__ is Mixin1D.__add__, as is Edge's and Wire's
            if (kind == SelfKind::CURVE || (kind == SelfKind::SHAPE && is_1d_shape)) return mixin1d_add(a, b);
            if (kind == SelfKind::COMPOUND && a_dim == 1) {
                // Compound.__add__ of a 1D compound: through a fresh Curve
                Shape sum1d = mixin1d_add(a.is_null() ? Shape() : Shape(a.wrapped()), b);
                if (!sum1d.is_null() && sum1d.wrapped().ShapeType() == TopAbs_EDGE) {
                    Shape as_curve(make_compound({sum1d.wrapped()}));
                    as_curve.set_history(ShapeHistory::of(sum1d));
                    sum1d = as_curve;
                }
                copy_attributes_to(a, sum1d);
                return sum1d;
            }
            const bool compound_self = kind != SelfKind::SHAPE;
            std::vector<Shape> summands = top_level_shapes(b);
            if (summands.empty()) return a;
            if (compound_self) {
                // Compound.__add__
                const std::vector<Shape> brought = summands;
                std::vector<Shape> all;
                if (!a.is_null()) {
                    for (const auto& t : top_level(a.wrapped())) all.emplace_back(t);
                }
                const std::vector<Shape> self_pieces = all;
                all.insert(all.end(), summands.begin(), summands.end());
                const int dim = a_dim >= 0 ? a_dim : topods_dim(all[0].wrapped());
                for (const auto& s : all) {
                    if (topods_dim(s.wrapped()) != dim) {
                        throw std::invalid_argument(std::string("Only shapes with the same dimension can be added: not ") +
                                                    dim_name(dim) + " and " + dim_name(topods_dim(s.wrapped())));
                    }
                }
                if (all.size() <= 1) return make_composite({all[0]});
                copy_attributes_to(a, all[0]);
                std::vector<const Shape*> tools;
                for (std::size_t i = 1; i < all.size(); ++i) tools.push_back(&all[i]);
                Shape result = bool_op_shape(BoolOp::FUSE, {&all[0]}, tools, TOLERANCE);
                if (result.history()) result.history()->with_inputs(wrapped_of(self_pieces), wrapped_of(brought));
                if (result.wrapped().ShapeType() != TopAbs_COMPOUND) result = make_composite({result});
                return result;
            }
            // Shape.__add__
            if (a_dim < 0) throw std::invalid_argument("Dimensions of objects to add to are inconsistent");
            for (const auto& s : summands) {
                if (topods_dim(s.wrapped()) != a_dim) {
                    throw std::invalid_argument(std::string("Only shapes with the same dimension can be added: not ") +
                                                dim_name(a_dim) + " and " + dim_name(topods_dim(s.wrapped())));
                }
            }
            if (a.is_null()) {
                if (summands.size() == 1) return summands[0];
                return summands[0].fuse(std::vector<Shape>(summands.begin() + 1, summands.end()));
            }
            return a.fuse(summands);
        }
        case AlgebraOp::SUB: {
            if (a.is_null()) throw std::invalid_argument("Cannot subtract shape from empty compound");
            const std::vector<Shape> subtrahends = top_level_shapes(b);
            if (subtrahends.empty()) return a;
            const int a_dim = static_dim >= 0 ? static_dim : a.dim();
            if (a_dim < 0) throw std::invalid_argument("Dimensions of objects to subtract from are inconsistent");
            for (const auto& o : subtrahends) {
                const int d = topods_dim(o.wrapped());
                if (d < 0) throw std::invalid_argument("Dimensions of objects to subtract from are inconsistent");
                if (d < a_dim) {
                    throw std::invalid_argument(
                        std::string("Only shapes with equal or greater dimension can be subtracted: not ") +
                        dim_name(a_dim) + " and " + dim_name(d));
                }
            }
            Shape difference = a.cut(subtrahends);
            if (kind != SelfKind::SHAPE) {
                if (!difference.is_null() && difference.wrapped().ShapeType() != TopAbs_COMPOUND) {
                    difference = make_composite({difference});
                }
                copy_attributes_to(a, difference);
            }
            return difference;
        }
        case AlgebraOp::AND: {
            std::vector<IntersectArg> others;
            for (const Shape* s : b) others.emplace_back(*s);
            return algebra_and(a, others, kind);
        }
    }
    return Shape();
}

Shape algebra_and(const Shape& a, const std::vector<IntersectArg>& others, SelfKind kind) {
    // Shape.__and__
    if (!a) throw std::invalid_argument("Cannot intersect shape with empty compound");
    for (const auto& o : others) {
        if (const auto* s = std::get_if<Shape>(&o); s && !*s) {
            throw std::invalid_argument("Cannot intersect shape with empty compound");
        }
    }
    const ShapeList<Shape> pieces = a.intersect(others);
    Shape new_shape;
    bool found = !pieces.empty();
    if (found) new_shape = pieces.size() == 1 ? pieces[0] : make_composite(pieces);
    if (found && !new_shape.is_null() && clean_enabled()) new_shape = new_shape.clean();
    if (kind == SelfKind::SHAPE) return new_shape;
    // Compound.__and__
    if (!found) return Shape();
    if (new_shape.wrapped().ShapeType() != TopAbs_COMPOUND) new_shape = make_composite({new_shape});
    copy_attributes_to(a, new_shape);
    return new_shape;
}

std::vector<TopoDS_Shape> list_entities(const TopoDS_Shape& shape, int topabs_type) {
    return entities(shape, static_cast<TopAbs_ShapeEnum>(topabs_type));
}

double round_digits(double value, int digits) { return py_round(value, digits); }

Vector shape_center(const Shape& s) { return s.center(); }

double shape_distance(const Shape& s, const Shape& other) { return s.distance_to(other); }
double shape_distance(const Shape& s, const Vector& point) { return s.distance_to(point); }

double sort_key_axis(const Shape& s, const Axis& axis) {
    return (s.center() - axis.position).dot(axis.direction);
}

double sort_key(const Shape& s, SortBy sort_by) {
    if (s.is_null()) throw std::invalid_argument("Can't sort an empty shape");
    const auto type = s.wrapped().ShapeType();
    switch (sort_by) {
        case SortBy::LENGTH:
            if (type == TopAbs_FACE) {
                auto l = Face(s.wrapped()).length();
                if (!l) throw std::invalid_argument("length is only defined for planar faces");
                return *l;
            }
            return linear_length(s.wrapped());
        case SortBy::RADIUS:
            if (type == TopAbs_FACE) {
                auto r = Face(s.wrapped()).radius();
                if (!r) throw std::invalid_argument("Face has no radius");
                return *r;
            }
            if (type == TopAbs_EDGE || type == TopAbs_WIRE) return Mixin1D(s.wrapped()).radius();
            throw std::invalid_argument("Shape has no radius");
        case SortBy::AREA: return s.area();
        case SortBy::VOLUME: return s.volume();
        case SortBy::DISTANCE: return s.center().length();
    }
    return 0.0;
}

double sort_key_path(const Shape& s, const Mixin1D& path) {
    const auto pts = path.closest_points(s.center());
    return path.param_at_point(pts.first);
}

namespace {

bool planar_face_axis(const TopoDS_Shape& shape, const Shape& s, Axis& out) {
    const TopoDS_Face& face = TopoDS::Face(shape);
    if (!GeomLib_IsPlanarSurface(BRep_Tool::Surface(face), TOLERANCE).IsPlanar()) return false;
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face(face).Normal(u0, v0, p, n);
    out = Axis(s.center(), Vector(n).normalized());
    return true;
}

}  // namespace

bool axis_parallel_predicate(const Shape& s, const Axis& axis, double tolerance) {
    if (s.is_null()) return false;
    const auto type = s.wrapped().ShapeType();
    Axis shape_axis = Axis::Z;
    if (type == TopAbs_FACE) {
        if (!planar_face_axis(s.wrapped(), s, shape_axis)) return false;
    } else if (type == TopAbs_EDGE && s.geom_type() == GeomType::LINE) {
        BRepAdaptor_Curve curve(TopoDS::Edge(s.wrapped()));
        gp_Pnt p;
        gp_Vec d;
        curve.D1(curve.FirstParameter(), p, d);
        shape_axis = Axis(Vector(p), Vector(gp_Dir(d)));
    } else {
        return false;
    }
    return axis.is_parallel(shape_axis, tolerance);
}

bool plane_parallel_predicate(const Shape& s, const Plane& plane, double tolerance) {
    if (s.is_null()) return false;
    const Axis plane_axis(plane.origin, plane.z_dir);
    const auto type = s.wrapped().ShapeType();
    if (type == TopAbs_FACE) {
        Axis shape_axis = Axis::Z;
        if (!planar_face_axis(s.wrapped(), s, shape_axis)) return false;
        return plane_axis.is_parallel(shape_axis, tolerance);
    }
    if (type == TopAbs_WIRE) {
        for (BRepTools_WireExplorer exp(TopoDS::Wire(s.wrapped())); exp.More(); exp.Next()) {
            if (!plane_parallel_predicate(Shape(exp.Current()), plane, tolerance)) return false;
        }
        return true;
    }
    if (type == TopAbs_EDGE) {
        TopLoc_Location loc;
        double f, l;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(s.wrapped()), loc, f, l);
        if (curve.IsNull()) return false;
        gp_XYZ normal = plane.z_dir.to_dir().Transformed(loc.Transformation().Inverted()).XYZ();
        return ShapeAnalysis_Curve::IsPlanar(curve, normal, tolerance);
    }
    return false;
}

}  // namespace build123d::detail
