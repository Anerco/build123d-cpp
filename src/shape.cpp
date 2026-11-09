#include <cstdio>
#include <sstream>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <gce_MakeLin.hxx>
#include <gp_GTrsf.hxx>

#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

GeomType curve_type(GeomAbs_CurveType t) {
    switch (t) {
        case GeomAbs_Line: return GeomType::LINE;
        case GeomAbs_Circle: return GeomType::CIRCLE;
        case GeomAbs_Ellipse: return GeomType::ELLIPSE;
        case GeomAbs_Hyperbola: return GeomType::HYPERBOLA;
        case GeomAbs_Parabola: return GeomType::PARABOLA;
        case GeomAbs_BezierCurve: return GeomType::BEZIER;
        case GeomAbs_BSplineCurve: return GeomType::BSPLINE;
        case GeomAbs_OffsetCurve: return GeomType::OFFSET;
        default: return GeomType::OTHER;
    }
}

GeomType surface_type(GeomAbs_SurfaceType t) {
    switch (t) {
        case GeomAbs_Plane: return GeomType::PLANE;
        case GeomAbs_Cylinder: return GeomType::CYLINDER;
        case GeomAbs_Cone: return GeomType::CONE;
        case GeomAbs_Sphere: return GeomType::SPHERE;
        case GeomAbs_Torus: return GeomType::TORUS;
        case GeomAbs_BezierSurface: return GeomType::BEZIER;
        case GeomAbs_BSplineSurface: return GeomType::BSPLINE;
        case GeomAbs_SurfaceOfRevolution: return GeomType::REVOLUTION;
        case GeomAbs_SurfaceOfExtrusion: return GeomType::EXTRUSION;
        case GeomAbs_OffsetSurface: return GeomType::OFFSET;
        default: return GeomType::OTHER;
    }
}

/// Properties dimension for a compound, like build123d's unwrapped_shapetype.
int compound_props_dim(const TopoDS_Shape& compound) {
    int type = -2;
    for (const auto& child : children(compound)) {
        const int t = static_cast<int>(child.ShapeType());
        if (type == -2) {
            type = t;
        } else if (t != type) {
            type = static_cast<int>(TopAbs_COMPOUND);
            break;
        }
    }
    switch (static_cast<TopAbs_ShapeEnum>(type < 0 ? TopAbs_COMPOUND : type)) {
        case TopAbs_VERTEX: return 0;
        case TopAbs_EDGE:
        case TopAbs_WIRE: return 1;
        case TopAbs_FACE:
        case TopAbs_SHELL: return 2;
        default: return 3;
    }
}

template <class T>
ShapeList<T> sub_shapes(const Shape& self, TopAbs_ShapeEnum type, Select select) {
    ShapeList<T> out;
    if (self.is_null()) return select_shapes(self, std::move(out), select);
    const auto path = extraction_path(self);
    for (const auto& s : entities(self.wrapped(), type)) {
        if (type == TopAbs_EDGE && BRep_Tool::Degenerated(TopoDS::Edge(s))) continue;
        T item(s);
        ShapeAccess::set_topo_path(item, path);
        out.push_back(std::move(item));
    }
    return select_shapes(self, std::move(out), select);
}

template <class T>
T single(const ShapeList<T>& list, const char* name) {
    if (list.size() != 1) throw_count(name, list.size());
    return list[0];
}

double shell_volume(const TopoDS_Shape& shell) {
    if (!Shape(shell).is_manifold()) return 0.0;
    ShapeFix_Solid fixer;
    const TopoDS_Solid solid = fixer.SolidFromShell(TopoDS::Shell(shell));
    return properties(solid, 3).Mass();
}

}  // namespace

Shape::operator bool() const {
    if (obj_.IsNull()) return false;
    if (obj_.ShapeType() == TopAbs_COMPOUND) return TopoDS_Iterator(obj_).More();
    return true;
}

void Shape::require_not_null(const char* what) const {
    if (obj_.IsNull()) throw std::invalid_argument(what);
}

ShapeType Shape::shape_type() const {
    if (obj_.IsNull()) return ShapeType::NONE;
    switch (obj_.ShapeType()) {
        case TopAbs_VERTEX: return ShapeType::VERTEX;
        case TopAbs_EDGE: return ShapeType::EDGE;
        case TopAbs_WIRE: return ShapeType::WIRE;
        case TopAbs_FACE: return ShapeType::FACE;
        case TopAbs_SHELL: return ShapeType::SHELL;
        case TopAbs_SOLID: return ShapeType::SOLID;
        case TopAbs_COMPSOLID: return ShapeType::COMPSOLID;
        case TopAbs_COMPOUND: return ShapeType::COMPOUND;
        default: return ShapeType::NONE;
    }
}

int Shape::dim() const { return topods_dim(obj_); }

GeomType Shape::geom_type() const {
    require_not_null("Cannot determine geometry type of an empty shape");
    if (obj_.ShapeType() == TopAbs_EDGE) {
        return curve_type(BRepAdaptor_Curve(TopoDS::Edge(obj_)).GetType());
    }
    if (obj_.ShapeType() == TopAbs_FACE) {
        return surface_type(BRepAdaptor_Surface(TopoDS::Face(obj_)).GetType());
    }
    return GeomType::OTHER;
}

bool Shape::is_valid() const {
    if (obj_.IsNull()) return true;
    BRepCheck_Analyzer checker(obj_);
    return checker.IsValid();
}

bool Shape::is_manifold() const {
    if (obj_.IsNull()) return false;
    for (const auto& shape : top_level(obj_)) {
        OccShapeListMap map;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, map);
        for (int i = 1; i <= map.Extent(); ++i) {
            const TopoDS_Edge edge = TopoDS::Edge(map.FindKey(i));
            TopoDS_Vertex v0, v1;
            TopExp::Vertices(edge, v0, v1);
            if (v0.IsNull() && v1.IsNull() && v0.IsSame(v1)) continue;
            if (map.FindFromIndex(i).Extent() != 2) return false;
        }
    }
    return true;
}

bool Shape::is_same(const Shape& other) const {
    if (obj_.IsNull() || other.obj_.IsNull()) return obj_.IsNull() && other.obj_.IsNull();
    return obj_.IsSame(other.obj_);
}

bool Shape::is_equal(const Shape& other) const {
    if (obj_.IsNull() || other.obj_.IsNull()) return obj_.IsNull() && other.obj_.IsNull();
    return obj_.IsEqual(other.obj_);
}

double Shape::area() const {
    if (obj_.IsNull()) return 0.0;
    return properties(obj_, 2).Mass();
}

double Shape::volume() const {
    if (obj_.IsNull()) return 0.0;
    switch (obj_.ShapeType()) {
        case TopAbs_SOLID: return properties(obj_, 3).Mass();
        case TopAbs_SHELL: return shell_volume(obj_);
        case TopAbs_COMPOUND:
        case TopAbs_COMPSOLID: {
            // Compound.volume: the solids and shells of get_type (this compound's children
            // and those of its direct sub-compounds)
            const Compound compound(obj_);
            double total = 0.0;
            for (const auto& s : compound.get_type<Solid>()) total += properties(s.wrapped(), 3).Mass();
            for (const auto& s : compound.get_type<Shell>()) total += shell_volume(s.wrapped());
            return total;
        }
        default: return 0.0;
    }
}

Vector Shape::center() const {
    if (obj_.IsNull()) throw std::invalid_argument("Can't find the center of an empty shape");
    switch (obj_.ShapeType()) {
        case TopAbs_VERTEX: return Vector(BRep_Tool::Pnt(TopoDS::Vertex(obj_)));
        case TopAbs_EDGE:
        case TopAbs_WIRE:
        case TopAbs_FACE: return center(CenterOf::GEOMETRY);
        case TopAbs_SHELL: return Vector(properties(obj_, 1).CentreOfMass());
        default: return center(CenterOf::MASS);
    }
}

Vector Shape::center(CenterOf center_of) const {
    if (obj_.IsNull()) throw std::invalid_argument("Can't find the center of an empty shape");
    if (center_of == CenterOf::BOUNDING_BOX) return bounding_box().center();
    switch (obj_.ShapeType()) {
        case TopAbs_VERTEX: return Vector(BRep_Tool::Pnt(TopoDS::Vertex(obj_)));
        case TopAbs_EDGE:
        case TopAbs_WIRE: {
            if (center_of == CenterOf::GEOMETRY) return Mixin1D(obj_).position_at(0.5);
            return Vector(properties(obj_, 1).CentreOfMass());
        }
        case TopAbs_FACE: {
            const Face face(obj_);
            if (center_of == CenterOf::MASS || face.is_planar()) {
                return Vector(properties(obj_, 2).CentreOfMass());
            }
            double u0, u1, v0, v1;
            BRepTools::UVBounds(TopoDS::Face(obj_), u0, u1, v0, v1);
            gp_Pnt p;
            gp_Vec n;
            BRepGProp_Face(TopoDS::Face(obj_)).Normal(0.5 * (u0 + u1), 0.5 * (v0 + v1), p, n);
            return Vector(p);
        }
        case TopAbs_SHELL:
            if (center_of == CenterOf::GEOMETRY) return Vector(properties(obj_, 1).CentreOfMass());
            return Vector(properties(obj_, 2).CentreOfMass());
        case TopAbs_SOLID:
        case TopAbs_COMPSOLID:
            if (center_of == CenterOf::GEOMETRY) {
                throw std::invalid_argument("Center of GEOMETRY is not supported for this object");
            }
            return Vector(properties(obj_, 3).CentreOfMass());
        default: {
            if (center_of == CenterOf::GEOMETRY) {
                throw std::invalid_argument("Center of GEOMETRY is not supported for this object");
            }
            const int dim = compound_props_dim(obj_);
            if (dim == 0) throw std::runtime_error("Unable to calculate center of mass of vertices");
            return Vector(properties(obj_, dim).CentreOfMass());
        }
    }
}

BoundBox Shape::bounding_box(std::optional<double> tolerance, bool optimal) const {
    return BoundBox::from_topo_ds(obj_, tolerance, optimal);
}

Location Shape::location() const { return Location(obj_.Location()); }

void Shape::set_location(const Location& loc) {
    require_not_null("Cannot locate an empty shape");
    obj_.Location(loc.wrapped());
}

Vector Shape::position() const { return location().position(); }
Vector Shape::orientation() const { return location().orientation(); }

ShapeList<Vertex> Shape::vertices(Select select) const {
    return sub_shapes<Vertex>(*this, TopAbs_VERTEX, select);
}
ShapeList<Edge> Shape::edges(Select select) const { return sub_shapes<Edge>(*this, TopAbs_EDGE, select); }
ShapeList<Wire> Shape::wires(Select select) const { return sub_shapes<Wire>(*this, TopAbs_WIRE, select); }
ShapeList<Face> Shape::faces(Select select) const { return sub_shapes<Face>(*this, TopAbs_FACE, select); }
ShapeList<Shell> Shape::shells(Select select) const { return sub_shapes<Shell>(*this, TopAbs_SHELL, select); }
ShapeList<Solid> Shape::solids(Select select) const { return sub_shapes<Solid>(*this, TopAbs_SOLID, select); }

namespace detail {

std::shared_ptr<const std::vector<Shape>> extraction_path(const Shape& source) {
    auto path = std::make_shared<std::vector<Shape>>();
    if (const auto& own = ShapeAccess::topo_path(source)) *path = *own;
    path->push_back(source);
    return path;
}

const ShapeHistory* history_for_select(const Shape& owner, Select select) {
    if (select != Select::LAST && select != Select::NEW) {
        throw std::invalid_argument("Invalid input, must be one of ['ALL', 'LAST', 'NEW']");
    }
    if (!owner.history()) {
        throw std::invalid_argument(std::string("Select.") + (select == Select::LAST ? "LAST" : "NEW") +
                                    " needs a record of how this shape was made, and this one has none - "
                                    "it is the result of no recorded operation");
    }
    return owner.history().get();
}

}  // namespace detail

ShapeList<Compound> Shape::compounds() const {
    ShapeList<Compound> out;
    if (obj_.IsNull() || obj_.ShapeType() != TopAbs_COMPOUND) return out;
    for (const auto& child : detail::children(obj_)) {
        if (child.ShapeType() == TopAbs_COMPOUND) out.push_back(Compound(child));
    }
    out.push_back(Compound(obj_));
    return out;
}

Vertex Shape::vertex() const { return single(vertices(), "vertex"); }
Edge Shape::edge() const { return single(edges(), "edge"); }
Wire Shape::wire() const { return single(wires(), "wire"); }
Face Shape::face() const { return single(faces(), "face"); }
Shell Shape::shell() const { return single(shells(), "shell"); }
Solid Shape::solid() const { return single(solids(), "solid"); }
Compound Shape::compound() const { return single(compounds(), "compound"); }

ShapeList<Shape> Shape::get_top_level_shapes() const {
    ShapeList<Shape> out;
    for (const auto& s : top_level(obj_)) out.emplace_back(s);
    return out;
}

std::vector<Shape> Shape::topo_path() const {
    return topo_path_ ? *topo_path_ : std::vector<Shape>{};
}

void Shape::set_topo_path(std::vector<Shape> path) {
    if (path.empty()) {
        topo_path_.reset();
    } else {
        topo_path_ = std::make_shared<const std::vector<Shape>>(std::move(path));
    }
}

std::optional<Shape> Shape::topo_parent() const {
    if (!topo_path_ || topo_path_->empty()) return std::nullopt;
    return topo_path_->front();
}

void Shape::set_topo_parent(const std::optional<Shape>& parent) {
    if (parent) {
        set_topo_path({*parent});
    } else {
        topo_path_.reset();
    }
}

std::optional<Shape> Shape::topo_owner() const {
    if (!topo_path_ || topo_path_->empty()) return std::nullopt;
    return topo_path_->back();
}

void Shape::extracted_from(const Shape& source) { topo_path_ = extraction_path(source); }

double Shape::distance(const Shape& other) const {
    if (obj_.IsNull() || other.obj_.IsNull()) {
        throw std::invalid_argument("Cannot calculate distance to or from an empty shape");
    }
    return occt("distance", [&] { return BRepExtrema_DistShapeShape(obj_, other.obj_).Value(); });
}

double Shape::distance_to(const Shape& other) const {
    return std::get<0>(distance_to_with_closest_points(other));
}

double Shape::distance_to(const Vector& point) const {
    return std::get<0>(distance_to_with_closest_points(point));
}

std::tuple<double, Vector, Vector> Shape::distance_to_with_closest_points(const Shape& other) const {
    if (obj_.IsNull() || other.obj_.IsNull()) {
        throw std::invalid_argument("Cannot calculate distance to or from an empty shape");
    }
    return occt("distance", [&] {
        BRepExtrema_DistShapeShape calc;
        calc.LoadS1(obj_);
        calc.LoadS2(other.obj_);
        calc.Perform();
        return std::make_tuple(calc.Value(), Vector(calc.PointOnShape1(1)),
                               Vector(calc.PointOnShape2(1)));
    });
}

std::tuple<double, Vector, Vector> Shape::distance_to_with_closest_points(const Vector& point) const {
    return distance_to_with_closest_points(
        Shape(BRepBuilderAPI_MakeVertex(point.to_pnt()).Vertex()));
}

std::pair<Vector, Vector> Shape::closest_points(const Shape& other) const {
    auto r = distance_to_with_closest_points(other);
    return {std::get<1>(r), std::get<2>(r)};
}

std::pair<Vector, Vector> Shape::closest_points(const Vector& point) const {
    auto r = distance_to_with_closest_points(point);
    return {std::get<1>(r), std::get<2>(r)};
}

ShapeList<Face> Shape::faces_intersected_by_axis(const Axis& axis, double tol) const {
    ShapeList<Face> out;
    if (obj_.IsNull()) return out;
    const gp_Lin line = gce_MakeLin(axis.wrapped()).Value();
    BRepIntCurveSurface_Inter inter;
    inter.Init(obj_, line, tol);
    std::vector<std::pair<double, TopoDS_Face>> found;
    while (inter.More()) {
        const double d = axis.position.to_pnt().SquareDistance(inter.Pnt());
        found.emplace_back(std::abs(d), inter.Face());
        inter.Next();
    }
    std::stable_sort(found.begin(), found.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& f : found) out.push_back(Face(f.second));
    return out;
}

std::vector<std::pair<Vector, Vector>> Shape::find_intersection_points(const Axis& axis,
                                                                       double tolerance) const {
    std::vector<std::pair<Vector, Vector>> result;
    if (obj_.IsNull()) return result;
    struct Hit {
        TopoDS_Face face;
        Vector point;
        double distance;
    };
    std::vector<Hit> hits;
    occt("find_intersection_points", [&] {
        const gp_Lin line = gce_MakeLin(axis.wrapped()).Value();
        BRepIntCurveSurface_Inter inter;
        inter.Init(obj_, line, tolerance);
        const Plane axis_plane(axis);
        for (; inter.More(); inter.Next()) {
            const Vector point(inter.Pnt());
            hits.push_back({inter.Face(), point, axis_plane.to_local_coords(point).Z});
        }
        return 0;
    });
    std::stable_sort(hits.begin(), hits.end(),
                     [](const Hit& a, const Hit& b) { return a.distance < b.distance; });
    for (const auto& h : hits) result.emplace_back(h.point, face_normal_at(h.face, h.point));
    return result;
}

Shape Shape::fuse(const Shape& other, bool glue, std::optional<double> tol) const {
    return fuse(std::vector<Shape>{other}, glue, tol);
}

Shape Shape::fuse(const std::vector<Shape>& to_fuse, bool glue, std::optional<double> tol) const {
    std::vector<const Shape*> tools;
    for (const auto& s : to_fuse) tools.push_back(&s);
    return bool_op_shape(BoolOp::FUSE, {this}, tools, tol.value_or(0.0), glue);
}

Shape Shape::cut(const Shape& other) const { return cut(std::vector<Shape>{other}); }

Shape Shape::cut(const std::vector<Shape>& to_cut) const {
    std::vector<const Shape*> tools;
    for (const auto& s : to_cut) tools.push_back(&s);
    return bool_op_shape(BoolOp::CUT, {this}, tools);
}

namespace {

SplitResult split_shape(const Shape& self, const TopoDS_Shape& tool,
                        const std::optional<Plane>& plane, Keep keep) {
    if (self.is_null() || tool.IsNull()) throw std::invalid_argument("Can't split an empty shape/tool");
    if (keep == Keep::INSIDE || keep == Keep::OUTSIDE) {
        throw std::invalid_argument("Keep.INSIDE/OUTSIDE are invalid for split");
    }
    return occt("split", [&] {
        BRepAlgoAPI_Splitter splitter;
        splitter.SetArguments(to_list({self.wrapped()}));
        splitter.SetTools(to_list({tool}));
        splitter.Build();
        const TopoDS_Shape result = unwrap(splitter.Shape(), true);

        SplitResult out;
        if (keep == Keep::ALL) {
            for (const auto& s : top_level(result)) out.emplace_back(s);
            return out;
        }
        TopoDS_Shape tool_solid;
        if (!plane) {
            TopoDS_Face tool_face;
            if (tool.ShapeType() == TopAbs_SHELL) {
                TopExp_Explorer exp(tool, TopAbs_FACE);
                tool_face = TopoDS::Face(exp.Current());
            } else {
                tool_face = TopoDS::Face(tool);
            }
            double u0, u1, v0, v1;
            BRepTools::UVBounds(tool_face, u0, u1, v0, v1);
            gp_Pnt p;
            gp_Vec n;
            BRepGProp_Face(tool_face).Normal((u0 + u1) / 2, (v0 + v1) / 2, p, n);
            const Vector ref = Vector(p) + Vector(n).normalized();
            if (tool.ShapeType() == TopAbs_SHELL) {
                tool_solid = BRepPrimAPI_MakeHalfSpace(TopoDS::Shell(tool), ref.to_pnt()).Solid();
            } else {
                tool_solid = BRepPrimAPI_MakeHalfSpace(tool_face, ref.to_pnt()).Solid();
            }
        }
        ShapeList<Shape> tops, bottoms;
        for (const auto& part : top_level(result)) {
            Shape sub(part);
            bool is_up;
            if (plane) {
                is_up = plane->to_local_coords(sub.center()).Z >= 0;
            } else {
                const TopoDS_Shape common = bool_op(BoolOp::COMMON, {part}, {tool_solid}, 0.0, false);
                is_up = linear_length(common) >= TOLERANCE;
            }
            (is_up ? tops : bottoms).push_back(sub);
        }
        if (keep == Keep::TOP || keep == Keep::BOTH) {
            out.insert(out.end(), tops.begin(), tops.end());
            out.top = tops;
        }
        if (keep == Keep::BOTTOM || keep == Keep::BOTH) {
            out.insert(out.end(), bottoms.begin(), bottoms.end());
            out.bottom = bottoms;
        }
        return out;
    });
}

}  // namespace

SplitResult Shape::split(const Plane& tool, Keep keep) const {
    return split_shape(*this, BRepBuilderAPI_MakeFace(tool.wrapped()).Face(), tool, keep);
}

SplitResult Shape::split(const Shape& tool, Keep keep) const {
    if (!tool.is_null() && tool.wrapped().ShapeType() != TopAbs_FACE &&
        tool.wrapped().ShapeType() != TopAbs_SHELL) {
        // A compound holding a single face or shell is accepted too.
        const TopoDS_Shape inner = unwrap(tool.wrapped(), true);
        if (inner.ShapeType() == TopAbs_FACE || inner.ShapeType() == TopAbs_SHELL) {
            return split_shape(*this, inner, std::nullopt, keep);
        }
        throw std::invalid_argument("split tool must be a Plane, Face or Shell");
    }
    return split_shape(*this, tool.wrapped(), std::nullopt, keep);
}

void Shape::move(const Location& loc) {
    require_not_null("Cannot move an empty shape");
    obj_.Move(loc.wrapped());
}

void Shape::locate(const Location& loc) {
    require_not_null("Cannot locate an empty shape");
    obj_.Location(loc.wrapped());
}

void Shape::mesh(double tolerance, double angular_tolerance) const {
    require_not_null("Cannot mesh an empty shape");
    if (!BRepTools::Triangulation(obj_, tolerance)) {
        BRepMesh_IncrementalMesh(obj_, tolerance, true, angular_tolerance, true);
    }
}

std::pair<std::vector<Vector>, std::vector<std::array<int, 3>>> Shape::tessellate(
    double tolerance, double angular_tolerance) const {
    require_not_null("Cannot tessellate an empty shape");
    mesh(tolerance, angular_tolerance);
    std::vector<Vector> vertices;
    std::vector<std::array<int, 3>> triangles;
    int offset = 0;
    for (const auto& face : faces()) {
        TopLoc_Location loc;
        const TopoDS_Face& f = TopoDS::Face(face.wrapped());
        Handle(Poly_Triangulation) poly = BRep_Tool::Triangulation(f, loc);
        if (poly.IsNull()) continue;
        const gp_Trsf trsf = loc.Transformation();
        const bool reverse = f.Orientation() == TopAbs_REVERSED;
        for (int i = 1; i <= poly->NbNodes(); ++i) {
            vertices.emplace_back(poly->Node(i).Transformed(trsf));
        }
        for (int i = 1; i <= poly->NbTriangles(); ++i) {
            int a, b, c;
            poly->Triangle(i).Get(a, b, c);
            if (reverse) {
                triangles.push_back({a + offset - 1, c + offset - 1, b + offset - 1});
            } else {
                triangles.push_back({a + offset - 1, b + offset - 1, c + offset - 1});
            }
        }
        offset += poly->NbNodes();
    }
    return {vertices, triangles};
}

std::string Shape::to_string() const {
    std::ostringstream os;
    os << build123d::to_string(shape_type());
    if (!label.empty()) os << " '" << label << "'";
    if (!obj_.IsNull()) {
        const BoundBox bb = bounding_box();
        os << " bbox=[" << bb.min << " .. " << bb.max << "]";
    }
    return os.str();
}

std::ostream& operator<<(std::ostream& os, const Shape& s) { return os << s.to_string(); }

// ---- transformation helpers

TopoDS_Shape Shape::moved_topods(const Location& loc) const {
    require_not_null("Cannot move an empty shape");
    return obj_.Moved(loc.wrapped());
}

TopoDS_Shape Shape::located_topods(const Location& loc) const {
    require_not_null("Cannot locate an empty shape");
    return obj_.Located(loc.wrapped());
}

TopoDS_Shape Shape::rotate_topods(const Axis& axis, double angle, bool transform_shape) const {
    if (obj_.IsNull()) return obj_;
    gp_Trsf t;
    t.SetRotation(axis.wrapped(), angle * DEG2RAD);
    if (transform_shape) return transform(obj_, t, true);
    return obj_.Moved(TopLoc_Location(t));
}

TopoDS_Shape Shape::translate_topods(const Vector& vector, bool transform_shape) const {
    if (obj_.IsNull()) return obj_;
    gp_Trsf t;
    t.SetTranslation(vector.to_vec());
    if (transform_shape) return transform(obj_, t, true);
    return obj_.Moved(TopLoc_Location(t));
}

TopoDS_Shape Shape::scale_topods(double factor, const std::optional<Vector>& about) const {
    if (obj_.IsNull()) return obj_;
    const Vector about_point = about ? *about : location().position();
    gp_Trsf t;
    t.SetScale(about_point.to_pnt(), factor);
    return transform(obj_, t, true);
}

TopoDS_Shape Shape::scale_topods(const Vector& factors, const std::optional<Vector>& about) const {
    if (obj_.IsNull()) return obj_;
    const Vector p = about ? *about : location().position();
    gp_GTrsf g;
    g.SetValue(1, 1, factors.X);
    g.SetValue(2, 2, factors.Y);
    g.SetValue(3, 3, factors.Z);
    g.SetValue(1, 4, p.X * (1 - factors.X));
    g.SetValue(2, 4, p.Y * (1 - factors.Y));
    g.SetValue(3, 4, p.Z * (1 - factors.Z));
    return occt("scale", [&] { return BRepBuilderAPI_GTransform(obj_, g, true).Shape(); });
}

TopoDS_Shape Shape::mirror_topods(const Plane& plane) const {
    if (obj_.IsNull()) return obj_;
    gp_Trsf t;
    t.SetMirror(gp_Ax2(plane.origin.to_pnt(), plane.z_dir.to_dir()));
    return transform(obj_, t, true);
}

TopoDS_Shape Shape::transformed_topods(const Vector& rotate, const Vector& offset) const {
    if (obj_.IsNull()) return obj_;
    gp_Trsf rx, ry, rz, to;
    rx.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), rotate.X * DEG2RAD);
    ry.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), rotate.Y * DEG2RAD);
    rz.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), rotate.Z * DEG2RAD);
    to.SetTranslation(offset.to_vec());
    return transform(obj_, to * rx * ry * rz, true);
}

TopoDS_Shape Shape::transform_topods(const gp_Trsf& trsf, bool copy) const {
    if (obj_.IsNull()) return obj_;
    return transform(obj_, trsf, copy);
}

TopoDS_Shape Shape::clean_topods(std::shared_ptr<ShapeHistory>* history) const {
    if (history) *history = history_;
    if (obj_.IsNull()) return obj_;
    try {
        ShapeUpgrade_UnifySameDomain upgrader(obj_, true, true, true);
        upgrader.AllowInternalEdges(false);
        upgrader.Build();
        const TopoDS_Shape cleaned = upgrader.Shape();
        if (history) {
            const auto unified = ShapeHistory::from_unify(upgrader);
            // build123d merges into the record in place; a copy keeps this shape's own intact
            *history = history_ ? std::shared_ptr<ShapeHistory>(history_->clone()) : unified;
            if (history_) (*history)->merge(unified);
        }
        return cleaned;
    } catch (const Standard_Failure&) {
        warn("Unable to clean " + to_string());
        return obj_;
    }
}

TopoDS_Shape Shape::fix_topods() const {
    if (obj_.IsNull() || is_valid()) return obj_;
    return fix_shape(obj_);
}

TopoDS_Shape Shape::copy_topods() const {
    if (obj_.IsNull()) return obj_;
    return BRepBuilderAPI_Copy(obj_).Shape();
}

// ---------------------------------------------------------------------------
// Vertex
// ---------------------------------------------------------------------------

Vertex::Vertex() : Vertex(0.0, 0.0, 0.0) {}

Vertex::Vertex(const TopoDS_Shape& obj) : Shape(obj) {
    if (!obj.IsNull() && obj.ShapeType() != TopAbs_VERTEX) {
        throw std::invalid_argument("Vertex requires a TopoDS_Vertex");
    }
}

Vertex::Vertex(double x, double y, double z)
    : Shape(BRepBuilderAPI_MakeVertex(gp_Pnt(x, y, z)).Vertex()) {}

Vertex::Vertex(const Vector& v) : Vertex(v.X, v.Y, v.Z) {}

double Vertex::X() const { return BRep_Tool::Pnt(TopoDS::Vertex(obj_)).X(); }
double Vertex::Y() const { return BRep_Tool::Pnt(TopoDS::Vertex(obj_)).Y(); }
double Vertex::Z() const { return BRep_Tool::Pnt(TopoDS::Vertex(obj_)).Z(); }

Vertex Vertex::operator+(const Vertex& other) const { return Vertex(X() + other.X(), Y() + other.Y(), Z() + other.Z()); }
Vertex Vertex::operator+(const Vector& other) const { return Vertex(X() + other.X, Y() + other.Y, Z() + other.Z); }
Vertex Vertex::operator-(const Vertex& other) const { return Vertex(X() - other.X(), Y() - other.Y(), Z() - other.Z()); }
Vertex Vertex::operator-(const Vector& other) const { return Vertex(X() - other.X, Y() - other.Y, Z() - other.Z); }

ShapeList<Vertex> Vertex::vertices(Select select) const {
    return select_shapes(*this, ShapeList<Vertex>{*this}, select);
}

Vector::Vector(const Vertex& v) {
    if (v.is_null()) throw std::invalid_argument("Cannot convert an empty Vertex to a Vector");
    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(v.wrapped()));
    X = p.X();
    Y = p.Y();
    Z = p.Z();
}

}  // namespace build123d
