// build123d-cpp: topology (Shape, Vertex, Edge, Wire, Face, Shell, Solid, Compound,
// Curve, Sketch, Part), ShapeList selectors and the algebra operators.
// Mirrors build123d/topology/*.py.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <gp_Trsf.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>

#include "build123d/geometry.hpp"
#include "build123d/materials.hpp"

namespace build123d {

class Shape;
class Vertex;
class Mixin1D;
class Edge;
class Wire;
class Face;
class Shell;
class Solid;
class Compound;
class Curve;
class Sketch;
class Part;
class ShapeHistory;
template <class T>
class ShapeList;

class SplitResult;

/// An operand of Shape::intersect: a shape or a geometry object (Vector and Location
/// act as a Vertex, Axis as an infinite Edge, Plane as an infinite Face).
using IntersectArg = std::variant<Shape, Vector, Location, Axis, Plane>;
template <class T>
class GroupBy;
class TangencyTarget;
struct ConstrainedArcOptions;
struct ConstrainedLineOptions;
class Joint;
namespace detail {
struct JointState;
}

/// The children of a shape in an assembly tree, used like a std::vector<Shape>.
/// Each child knows its parent (Shape::parent()). As when assigning `children` in build123d,
/// changing the children of a compound (or of an empty shape) rebuilds its geometry as a
/// compound of the children. The tree owns its children: adding a shape stores a copy.
class ShapeChildren {
public:
    using value_type = Shape;
    using size_type = std::size_t;
    using reference = Shape&;
    using const_reference = const Shape&;
    using iterator = std::vector<Shape>::iterator;
    using const_iterator = std::vector<Shape>::const_iterator;

    ShapeChildren() = default;
    ShapeChildren(const ShapeChildren& other);
    ShapeChildren(ShapeChildren&& other) noexcept;
    ShapeChildren& operator=(const ShapeChildren& other);
    ShapeChildren& operator=(ShapeChildren&& other);
    ShapeChildren& operator=(const std::vector<Shape>& shapes);
    ShapeChildren& operator=(std::initializer_list<Shape> shapes);
    ~ShapeChildren();

    operator const std::vector<Shape>&() const { return items_; }  // NOLINT: read like a vector
    const std::vector<Shape>& as_vector() const { return items_; }

    size_type size() const;
    bool empty() const;
    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;
    const_iterator cbegin() const;
    const_iterator cend() const;
    Shape& operator[](size_type i);
    const Shape& operator[](size_type i) const;
    Shape& at(size_type i);
    const Shape& at(size_type i) const;
    Shape& front();
    const Shape& front() const;
    Shape& back();
    const Shape& back() const;

    void push_back(const Shape& child);
    void push_back(Shape&& child);
    template <class... Args>
    Shape& emplace_back(Args&&... args);
    iterator insert(const_iterator pos, const Shape& child);
    iterator erase(const_iterator pos);
    iterator erase(const_iterator first, const_iterator last);
    void pop_back();
    void clear();
    void reserve(size_type n);

private:
    friend class Shape;
    friend struct ShapeAccess;
    std::vector<Shape> items_;
    Shape* owner_ = nullptr;

    /// Set each child's parent and, if the children changed, rebuild the owner's geometry.
    void changed(bool had_children);
    void relink();
    /// Replace the children without touching the owner's geometry.
    void assign_raw(const std::vector<Shape>& shapes);
};

/// The joints bound to a shape, by label: build123d's `joints` dict (see joints.hpp).
/// A copy of a shape gets copies of its joints, bound to the copy.
class JointMap {
public:
    JointMap() = default;
    JointMap(const JointMap& other);
    JointMap(JointMap&& other) noexcept;
    JointMap& operator=(const JointMap& other);
    JointMap& operator=(JointMap&& other) noexcept;
    ~JointMap();

    /// The joint with this label; std::out_of_range if there is none.
    Joint operator[](const std::string& label) const;
    Joint at(const std::string& label) const;
    /// The joint with this label as a specific joint type (std::invalid_argument if it isn't).
    template <class J>
    J get(const std::string& label) const;
    bool contains(const std::string& label) const;
    std::size_t count(const std::string& label) const;
    std::size_t size() const;
    bool empty() const;
    /// Labels in insertion order.
    std::vector<std::string> keys() const;
    std::vector<Joint> values() const;
    std::vector<std::pair<std::string, Joint>> items() const;
    /// Bind `joint` to this map's shape under `label` (build123d: `part.joints[label] = joint`).
    void insert(const std::string& label, const Joint& joint);
    void erase(const std::string& label);
    void clear();

private:
    friend class Shape;
    friend class Joint;
    std::vector<std::pair<std::string, std::shared_ptr<detail::JointState>>> items_;
    Shape* owner_ = nullptr;

    void rebind(Shape* owner);
    void detach_all();
};

/// RAII guard that disables the automatic clean() after boolean operations,
/// like build123d's `with SkipClean():`.
class SkipClean {
public:
    SkipClean();
    ~SkipClean();
    SkipClean(const SkipClean&) = delete;
    SkipClean& operator=(const SkipClean&) = delete;
    static bool clean_enabled();

private:
    bool previous_;
};

// Adds the build123d methods that return "Self" with the right C++ type.
#define BUILD123D_TYPED_OPS(T)                                                                 \
public:                                                                                        \
    /** Copy relocated relative to the current location. */                                   \
    T moved(const Location& loc) const { return derive<T>(moved_topods(loc)); }               \
    T moved(const Plane& plane) const { return derive<T>(moved_topods(plane.location())); }   \
    /** Copy placed at an absolute location. */                                                \
    T located(const Location& loc) const { return derive<T>(located_topods(loc)); }           \
    /** Copy rotated about an axis by `angle` degrees; `transform` regenerates the shape  */   \
    /** instead of only changing its location. */                                              \
    T rotate(const Axis& axis, double angle, bool transform = false) const {                   \
        return derive<T>(rotate_topods(axis, angle, transform));                               \
    }                                                                                          \
    T translate(const Vector& vector, bool transform = false) const {                          \
        return derive<T>(translate_topods(vector, transform));                                 \
    }                                                                                          \
    /** Uniform scale about `about` (defaults to the shape's location). */                    \
    T scale(double factor, std::optional<Vector> about = std::nullopt) const {                 \
        return derive<T>(scale_topods(factor, about));                                         \
    }                                                                                          \
    /** Non-uniform scale (may convert geometry to splines). */                                \
    T scale(const Vector& factors, std::optional<Vector> about = std::nullopt) const {         \
        return derive<T>(scale_topods(factors, about));                                        \
    }                                                                                          \
    /** Mirror about a plane (does not keep the original). */                                  \
    T mirror(const Plane& mirror_plane = Plane::XY) const {                                    \
        return derive<T>(mirror_topods(mirror_plane));                                         \
    }                                                                                          \
    /** Rotate by Euler angles (degrees, X then Y then Z) and then translate. */               \
    T transformed(const Vector& rotate = Vector(), const Vector& offset = Vector()) const {     \
        return derive<T>(transformed_topods(rotate, offset));                                  \
    }                                                                                          \
    T transform_shape(const gp_Trsf& trsf) const {                                             \
        return derive<T>(transform_topods(trsf, false));                                      \
    }                                                                                          \
    /** Transform without changing the geometry type (rigid part of the matrix). */            \
    T transform_shape(const Matrix& t_matrix) const {                                          \
        return derive<T>(transform_topods(t_matrix.wrapped().Trsf(), false));                 \
    }                                                                                          \
    /** Affine transform (may convert lines and circles to splines; handles skew/stretch). */ \
    T transform_geometry(const Matrix& t_matrix) const {                                       \
        return derive<T>(gtransform_topods(t_matrix));                                         \
    }                                                                                          \
    /** Remove internal edges/faces (ShapeUpgrade_UnifySameDomain); the history records it. */ \
    T clean() const {                                                                          \
        std::shared_ptr<ShapeHistory> clean_history;                                           \
        T cleaned = derive<T>(clean_topods(&clean_history));                                   \
        cleaned.set_history(std::move(clean_history));                                         \
        return cleaned;                                                                        \
    }                                                                                          \
    /** Repair an invalid shape (no-op if valid). */                                           \
    T fix() const { return derive<T>(fix_topods()); }                                         \
    /** Deep copy (geometry duplicated). */                                                    \
    T copy() const { return derive<T>(copy_topods()); }                                       \
    /** All geometry approximated by B-splines (ShapeCustom::BSplineRestriction). */           \
    T to_splines(int degree = 3, double tolerance = 1e-3, bool nurbs = false) const {          \
        return T(to_splines_topods(degree, tolerance, nurbs));                                 \
    }

// ---------------------------------------------------------------------------
// Shape
// ---------------------------------------------------------------------------

/// Base class of all topological objects. Holds an OCCT TopoDS_Shape.
class Shape {
public:
    static constexpr int kDim = -1;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_SHAPE;
    using topo_type = Shape;

    std::string label;
    std::optional<Color> color;
    /// What the shape is made of (mass() reads its density); see materials.hpp.
    std::optional<FinishedMaterial> material;
    /// Assembly children (with their own labels/colors); see Compound::make_assembly.
    ShapeChildren children;
    /// Joints bound to this shape, by label (see joints.hpp).
    JointMap joints;

    Shape() { bind(); }
    explicit Shape(const TopoDS_Shape& obj) : obj_(obj) { bind(); }
    Shape(const Shape& other);
    Shape(Shape&& other) noexcept;
    Shape& operator=(const Shape& other);
    Shape& operator=(Shape&& other) noexcept;
    ~Shape() = default;

    // ---- Assembly tree (build123d's anytree NodeMixin) ----
    /// The shape whose children hold this one, or nullptr.
    Shape* parent() { return tree_parent_; }
    const Shape* parent() const { return tree_parent_; }
    /// The top of this shape's assembly tree (this shape if it has no parent).
    Shape& root();
    const Shape& root() const;
    /// The shapes from the root down to this one.
    std::vector<Shape*> path();
    std::vector<const Shape*> path() const;
    /// The shapes from this one up to the root.
    std::vector<const Shape*> iter_path_reverse() const;
    /// The shapes from the root down to the parent.
    std::vector<Shape*> ancestors();
    std::vector<const Shape*> ancestors() const;
    /// Every shape below this one, in pre-order.
    std::vector<Shape*> descendants();
    std::vector<const Shape*> descendants() const;
    /// The shapes without children in this subtree (this shape if it has none).
    std::vector<Shape*> leaves();
    std::vector<const Shape*> leaves() const;
    /// The other children of this shape's parent.
    std::vector<Shape*> siblings();
    std::vector<const Shape*> siblings() const;
    bool is_root() const { return tree_parent_ == nullptr; }
    bool is_leaf() const { return children.empty(); }
    /// Number of steps from the root down to this shape.
    std::size_t depth() const;
    /// Number of steps on the longest path from this shape down to a leaf.
    std::size_t height() const;
    /// Number of shapes in the subtree starting at this one (anytree's `size`).
    std::size_t tree_size() const;
    /// Location in world coordinates: the locations along the path from the root, combined.
    Location global_location() const;

    const TopoDS_Shape& wrapped() const { return obj_; }
    bool is_null() const { return obj_.IsNull(); }
    /// Not null and, for compounds, not empty.
    explicit operator bool() const;

    ShapeType shape_type() const;
    /// Topological dimension (0..3), or -1 if empty or mixed.
    int dim() const;
    GeomType geom_type() const;
    /// How the shape this one was selected from sits around it (build123d's convexity):
    /// defined for a Vertex, Edge or Face taken out of a larger shape (see
    /// Vertex/Mixin1D/Face); std::invalid_argument for anything else.
    Convexity convexity() const;
    bool is_valid() const;
    bool is_manifold() const;
    /// Same TShape and location (orientation may differ).
    bool is_same(const Shape& other) const;
    /// Same TShape, location and orientation.
    bool is_equal(const Shape& other) const;
    bool operator==(const Shape& other) const { return is_same(other); }
    bool operator!=(const Shape& other) const { return !is_same(other); }

    double area() const;
    double volume() const;
    /// build123d's compute_volume: the GProp mass for the shape type (length for edges and
    /// wires, area for faces and shells, volume otherwise; 0 for a vertex).
    double compute_volume() const;
    /// Mass from `material`'s density (std::invalid_argument if there is none): volume()
    /// times the density converted to `mass_unit` per `length_unit`³.
    double compute_mass(Unit mass_unit = Unit::G, Unit length_unit = Unit::MM) const;
    /// The mass as build123d's per-class `mass`: 0 for vertices, edges, wires and faces;
    /// compute_mass for shells and solids; for a compound the sum over its assembly
    /// children (each with its own material, else the compound's) or its solids and shells.
    double mass(Unit mass_unit = Unit::G, Unit length_unit = Unit::MM) const;
    /// Inertia tensor about the center of mass (VolumeProperties), rows Ixx..Izz.
    std::array<std::array<double, 3>, 3> matrix_of_inertia() const;
    /// The principal axes of inertia with their moments.
    std::vector<std::pair<Vector, double>> principal_properties() const;
    /// First moments of mass (Mx, My, Mz).
    std::array<double, 3> static_moments() const;
    double radius_of_gyration(const Axis& axis) const;
    /// Center of several objects: weighted by compute_volume (CenterOf::MASS) or the mean of
    /// the bounding box centers (BOUNDING_BOX); GEOMETRY throws.
    static Vector combined_center(const std::vector<Shape>& objects, CenterOf center_of = CenterOf::MASS);
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S> && !std::is_same_v<S, Shape>, int> = 0>
    static Vector combined_center(const std::vector<S>& objects, CenterOf center_of = CenterOf::MASS) {
        return combined_center(std::vector<Shape>(objects.begin(), objects.end()), center_of);
    }
    /// Center using the default method for the shape type
    /// (GEOMETRY for 1D/2D, MASS for 3D and compounds).
    Vector center() const;
    Vector center(CenterOf center_of) const;
    BoundBox bounding_box(std::optional<double> tolerance = std::nullopt, bool optimal = true) const;
    /// The oriented bounding box (an empty one for an empty shape).
    OrientedBoundBox oriented_bounding_box() const;
    Location location() const;
    void set_location(const Location& loc);
    Vector position() const;
    Vector orientation() const;

    /// Sub-shapes; `select` narrows them to those the operation that made this shape
    /// brought in or created (Select::LAST) or created outright (Select::NEW).
    ShapeList<Vertex> vertices(Select select = Select::ALL) const;
    ShapeList<Edge> edges(Select select = Select::ALL) const;
    ShapeList<Wire> wires(Select select = Select::ALL) const;
    ShapeList<Face> faces(Select select = Select::ALL) const;
    ShapeList<Shell> shells(Select select = Select::ALL) const;
    ShapeList<Solid> solids(Select select = Select::ALL) const;
    ShapeList<Compound> compounds() const;
    /// Exactly one sub-shape of the given type, else std::invalid_argument.
    Vertex vertex() const;
    Edge edge() const;
    Wire wire() const;
    Face face() const;
    Shell shell() const;
    Solid solid() const;
    Compound compound() const;
    /// First level non-compound children (nested compounds are flattened).
    ShapeList<Shape> get_top_level_shapes() const;

    /// The shapes this one was extracted through (edges(), faces(), ...), outermost first.
    std::vector<Shape> topo_path() const;
    void set_topo_path(std::vector<Shape> path);
    /// The shape this one was ultimately taken out of: the outermost step of topo_path().
    std::optional<Shape> topo_parent() const;
    /// Record a single step of provenance, discarding any longer path.
    void set_topo_parent(const std::optional<Shape>& parent);
    /// The shape this one was taken directly out of: the innermost step of topo_path().
    std::optional<Shape> topo_owner() const;
    /// Record that this shape was taken out of `source`: topo_path = source's path + source.
    void extracted_from(const Shape& source);

    /// The record of how the operation that made this shape treated its inputs'
    /// sub-shapes (nullptr if no recorded operation made it). See history.hpp.
    const std::shared_ptr<ShapeHistory>& history() const { return history_; }
    void set_history(std::shared_ptr<ShapeHistory> history) { history_ = std::move(history); }

    double distance(const Shape& other) const;
    double distance_to(const Shape& other) const;
    double distance_to(const Vector& point) const;
    std::tuple<double, Vector, Vector> distance_to_with_closest_points(const Shape& other) const;
    std::tuple<double, Vector, Vector> distance_to_with_closest_points(const Vector& point) const;
    std::pair<Vector, Vector> closest_points(const Shape& other) const;
    std::pair<Vector, Vector> closest_points(const Vector& point) const;
    /// Faces crossed by an axis, sorted by distance from the axis position.
    ShapeList<Face> faces_intersected_by_axis(const Axis& axis, double tol = 1e-4) const;
    /// Points where an axis crosses the faces of this shape, with the face normals,
    /// sorted along the axis.
    std::vector<std::pair<Vector, Vector>> find_intersection_points(const Axis& axis,
                                                                    double tolerance = TOLERANCE) const;
    /// Wrap faces (e.g. Text) onto this shape: each face is placed normal to the surface at
    /// its position along `path` (starting at normalized `start`) and projected.
    ShapeList<Shape> project_faces(const std::vector<Face>& faces, const Mixin1D& path,
                                   double start = 0.0) const;
    ShapeList<Shape> project_faces(const Shape& faces, const Mixin1D& path, double start = 0.0) const;

    /// Untyped booleans that record their history; prefer the +, -, & operators.
    Shape fuse(const Shape& other, bool glue = false, std::optional<double> tol = std::nullopt) const;
    Shape fuse(const std::vector<Shape>& to_fuse, bool glue = false,
               std::optional<double> tol = std::nullopt) const;
    Shape cut(const Shape& other) const;
    Shape cut(const std::vector<Shape>& to_cut) const;
    /// Where bodies or interiors meet (overlap or crossing geometry), as build123d 0.13:
    /// several operands chain (a ∩ b ∩ c) and a Compound operand distributes over its
    /// elements ((a ∩ c1) ∪ (a ∩ c2)). `include_touched` adds boundary contacts where
    /// Solids are involved. Python's None (no intersection) is an empty list here.
    ShapeList<Shape> intersect(const IntersectArg& to_intersect, double tolerance = 1e-6,
                               bool include_touched = false) const;
    ShapeList<Shape> intersect(const std::vector<IntersectArg>& to_intersect, double tolerance = 1e-6,
                               bool include_touched = false) const;
    /// Boundary contacts with another shape (vertices, edges, faces) without interior overlap.
    ShapeList<Shape> touch(const Shape& other, double tolerance = 1e-6) const;
    /// Split by a plane, face or shell, keeping Keep::TOP, BOTTOM, BOTH or ALL. The result
    /// is the list of kept pieces (tops then bottoms for BOTH); `.top` and `.bottom` hold
    /// each side, and `auto [top, bottom] = s.split(tool, Keep::BOTH);` unpacks them like
    /// build123d's tuple (an empty list stands for None).
    SplitResult split(const Plane& tool, Keep keep = Keep::TOP) const;
    SplitResult split(const Shape& tool, Keep keep = Keep::TOP) const;

    /// In-place relative move / absolute placement.
    void move(const Location& loc);
    void locate(const Location& loc);

    void mesh(double tolerance, double angular_tolerance = 0.1) const;
    std::pair<std::vector<Vector>, std::vector<std::array<int, 3>>> tessellate(
        double tolerance, double angular_tolerance = 0.1) const;
    /// Triangles with per-vertex normals and UVs: (vertices, triangles, normals, uvs). The
    /// face UVs are normalized to [0, 1] and, with `atlas_packing`, packed into one atlas
    /// (each island inset by `atlas_gutter`).
    std::tuple<std::vector<Vector>, std::vector<std::array<int, 3>>, std::vector<Vector>,
               std::vector<std::array<double, 2>>>
    tessellate_with_uvs(double tolerance, double angular_tolerance = 0.1, bool atlas_packing = true,
                        double atlas_gutter = 0.0) const;

    /// The internal structure as a tree, down to `limit_class`. `show_center` defaults to
    /// the location for assemblies (compounds with children) and the bounding box center
    /// otherwise. (The addresses printed are those of the C++ objects.)
    std::string show_topology(ShapeType limit_class = ShapeType::VERTEX,
                              std::optional<bool> show_center = std::nullopt) const;
    std::string to_string() const;
    /// build123d's __hash__: the OCCT shape (TShape and location); see std::hash<Shape>.
    std::size_t hash() const;

    BUILD123D_TYPED_OPS(Shape)

protected:
    TopoDS_Shape obj_;
    /// topo_path (shared, never modified once set).
    std::shared_ptr<const std::vector<Shape>> topo_path_;
    std::shared_ptr<ShapeHistory> history_;

    /// A copy of this shape's attributes around a new TopoDS_Shape, like build123d's
    /// deepcopy-then-replace-wrapped (moved, located, transformed, ...): label, color,
    /// material, children, joints (bound to the copy), topo_path and history are carried over.
    template <class T>
    T derive(const TopoDS_Shape& shape) const {
        T result(shape);
        Shape& base = result;
        base.label = label;
        base.color = color;
        base.material = material;
        base.children.assign_raw(children.items_);
        base.joints = joints;
        base.topo_path_ = topo_path_;
        base.history_ = history_;
        return result;
    }
    void copy_metadata_from(const Shape& other) {
        label = other.label;
        color = other.color;
        material = other.material;
    }
    void require_not_null(const char* what) const;

    TopoDS_Shape moved_topods(const Location& loc) const;
    TopoDS_Shape located_topods(const Location& loc) const;
    TopoDS_Shape rotate_topods(const Axis& axis, double angle, bool transform = false) const;
    TopoDS_Shape translate_topods(const Vector& vector, bool transform = false) const;
    TopoDS_Shape scale_topods(double factor, const std::optional<Vector>& about) const;
    TopoDS_Shape scale_topods(const Vector& factors, const std::optional<Vector>& about) const;
    TopoDS_Shape mirror_topods(const Plane& plane) const;
    TopoDS_Shape transformed_topods(const Vector& rotate, const Vector& offset) const;
    TopoDS_Shape transform_topods(const gp_Trsf& trsf, bool copy) const;
    TopoDS_Shape gtransform_topods(const Matrix& t_matrix) const;
    /// The cleaned shape; `history` (if given) receives this shape's record merged with
    /// the clean's.
    TopoDS_Shape clean_topods(std::shared_ptr<ShapeHistory>* history = nullptr) const;
    TopoDS_Shape fix_topods() const;
    TopoDS_Shape copy_topods() const;
    TopoDS_Shape to_splines_topods(int degree, double tolerance, bool nurbs) const;

    template <class T>
    friend class ShapeList;
    friend class Compound;
    friend class Vertex;
    friend class ShapeChildren;
    friend class JointMap;
    friend struct ShapeAccess;

private:
    // The shape whose `children` hold this one. Copies start as roots; ShapeChildren sets it.
    Shape* tree_parent_ = nullptr;
    /// Point the children and joints at this object (after construction, copy or move).
    void bind();
    /// build123d's Compound._post_attach_children: the geometry becomes a compound of the children.
    void rebuild_from_children();
};

/// Access to Shape's protected parts for the library's own sources (not for users).
struct ShapeAccess {
    /// Copy every attribute (label, color, material, children, joints, topo_path, history)
    /// but the shape. Children are copied without rebuilding `to`'s geometry; the joints are
    /// copies bound to `to`.
    static void copy_attributes(const Shape& from, Shape& to) {
        to.label = from.label;
        to.color = from.color;
        to.material = from.material;
        to.children.assign_raw(from.children.items_);
        to.joints = from.joints;
        to.topo_path_ = from.topo_path_;
        to.history_ = from.history_;
    }
    static const std::shared_ptr<const std::vector<Shape>>& topo_path(const Shape& s) { return s.topo_path_; }
    static void set_topo_path(Shape& s, std::shared_ptr<const std::vector<Shape>> path) {
        s.topo_path_ = std::move(path);
    }
    static TopoDS_Shape& wrapped(Shape& s) { return s.obj_; }
    /// Replace the children without rebuilding the geometry.
    static void set_children_raw(Shape& s, const std::vector<Shape>& children) { s.children.assign_raw(children); }
};

std::ostream& operator<<(std::ostream& os, const Shape& s);

template <class... Args>
Shape& ShapeChildren::emplace_back(Args&&... args) {
    const bool had_children = !items_.empty();
    items_.emplace_back(std::forward<Args>(args)...);
    changed(had_children);
    return items_.back();
}

// ---------------------------------------------------------------------------
// 0D
// ---------------------------------------------------------------------------

class Vertex : public Shape {
public:
    static constexpr int kDim = 0;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_VERTEX;
    using topo_type = Vertex;

    /// The origin, as in build123d.
    Vertex();
    explicit Vertex(const TopoDS_Shape& obj);
    explicit Vertex(double x, double y, double z = 0.0);
    explicit Vertex(const Vector& v);

    double X() const;
    double Y() const;
    double Z() const;
    Vector to_vector() const { return Vector(*this); }
    /// The coordinates (build123d's iteration over a Vertex).
    std::array<double, 3> to_tuple() const { return {X(), Y(), Z()}; }

    /// Coordinate arithmetic, as in build123d: a new Vertex at the sum or difference.
    /// A Python tuple operand is a Vector here (v + Vector(0, 0, 15)); the compound
    /// assignments also take braces (v += {0, 0, 15}) and Vertices.
    Vertex operator+(const Vertex& other) const;
    Vertex operator+(const Vector& other) const;
    Vertex operator-(const Vertex& other) const;
    Vertex operator-(const Vector& other) const;
    Vertex& operator+=(const Vector& other) { return *this = *this + other; }
    Vertex& operator-=(const Vector& other) { return *this = *this - other; }

    /// This vertex (narrowed by `select` like any sub-shape list).
    ShapeList<Vertex> vertices(Select select = Select::ALL) const;
    Vertex vertex() const { return *this; }

    BUILD123D_TYPED_OPS(Vertex)
};

// ---------------------------------------------------------------------------
// 1D
// ---------------------------------------------------------------------------

/// Methods shared by Edge and Wire.
class Mixin1D : public Shape {
public:
    static constexpr int kDim = 1;
    using Shape::Shape;

    double length() const;
    /// Radius of a circular edge (first edge for a wire).
    double radius() const;
    bool is_closed() const;
    bool is_forward() const;
    /// Is this a concave edge of the shape it was selected from (convexity() == CONCAVE)?
    bool is_interior() const;

    Vector position_at(double position, PositionMode mode = PositionMode::PARAMETER) const;
    std::vector<Vector> positions(const std::vector<double>& distances,
                                  PositionMode mode = PositionMode::PARAMETER) const;
    Vector tangent_at(double position = 0.5, PositionMode mode = PositionMode::PARAMETER) const;
    /// Tangent at the point on the curve closest to `point`.
    Vector tangent_at(const Vector& point) const;
    Vector derivative_at(double position, int order = 2,
                         PositionMode mode = PositionMode::PARAMETER) const;
    Vector derivative_at(const Vector& point, int order = 2) const;
    /// Tangent angle (degrees, 0..360) measured in `plane`.
    double tangent_angle_at(double location_param = 0.5,
                            PositionMode mode = PositionMode::PARAMETER,
                            const Plane& plane = Plane::XY) const;
    Location location_at(double distance, PositionMode mode = PositionMode::PARAMETER,
                         FrameMethod frame_method = FrameMethod::FRENET,
                         std::optional<Vector> x_dir = std::nullopt) const;
    std::vector<Location> locations(const std::vector<double>& distances,
                                    PositionMode mode = PositionMode::PARAMETER,
                                    FrameMethod frame_method = FrameMethod::FRENET,
                                    std::optional<Vector> x_dir = std::nullopt) const;
    Vector start_point() const;
    Vector end_point() const;
    /// Normalized parameter (0..1) of the point on the curve closest to `point`.
    double param_at_point(const Vector& point) const;
    /// Normal of a planar curve.
    Vector normal() const;
    /// Plane containing this and the other lines, if any.
    std::optional<Plane> common_plane(const std::vector<Shape>& lines = {},
                                      double tolerance = TOLERANCE) const;
    /// 2D offset of a planar edge/wire.
    Wire offset_2d(double distance, Kind kind = Kind::ARC, Side side = Side::BOTH,
                   bool closed = true) const;
    /// Line in `plane` perpendicular to this curve at `u_value`, centered on it.
    Edge perpendicular_line(double length, double u_value, const Plane& plane = Plane::XY) const;
    /// Project onto a target shape along `direction` (the edges of Edge/Wire::project_to_shape).
    ShapeList<Edge> project_to_shape(const Shape& target, const Vector& direction) const;
    /// Project onto a face along `direction`: the projection closest to this curve (an Edge or
    /// a Wire; check shape_type()).
    Mixin1D project(const Face& face, const Vector& direction) const;
    /// All the projections onto the face (closest = false), or just the closest one.
    ShapeList<Mixin1D> project(const Face& face, const Vector& direction, bool closest) const;
    /// Hidden-line projection onto a viewport: the (visible, hidden) edges, in viewport
    /// coordinates. `look_at` defaults to the center of the shape; a `focus` gives a
    /// perspective projection.
    std::pair<ShapeList<Edge>, ShapeList<Edge>> project_to_viewport(
        const Vector& viewport_origin, const Vector& viewport_up = Vector(0, 0, 1),
        std::optional<Vector> look_at = std::nullopt,
        std::optional<double> focus = std::nullopt) const;
};

/// Tangent at a position, build123d's `%` operator.
Vector operator%(const Mixin1D& curve, double position);
/// Location at a position, build123d's `^` operator.
Location operator^(const Mixin1D& curve, double position);

class Edge : public Mixin1D {
public:
    static constexpr int kDim = 1;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_EDGE;
    using topo_type = Edge;

    Edge() = default;
    explicit Edge(const TopoDS_Shape& obj);
    /// Infinite line along the axis.
    explicit Edge(const Axis& axis);

    static Edge make_line(const Vector& point1, const Vector& point2);
    static Edge make_circle(double radius, const Plane& plane = Plane::XY,
                            double start_angle = 360.0, double end_angle = 360.0,
                            AngularDirection angular_direction = AngularDirection::COUNTER_CLOCKWISE);
    static Edge make_ellipse(double x_radius, double y_radius, const Plane& plane = Plane::XY,
                             double start_angle = 360.0, double end_angle = 360.0,
                             AngularDirection angular_direction = AngularDirection::COUNTER_CLOCKWISE);
    static Edge make_helix(double pitch, double height, double radius,
                           const Vector& center = Vector(0, 0, 0),
                           const Vector& normal = Vector(0, 0, 1), double angle = 0.0,
                           bool lefthand = false);
    /// Interpolating spline. `tangents` may be empty, two (start/end) or one per point.
    static Edge make_spline(const std::vector<Vector>& points,
                            const std::vector<Vector>& tangents = {}, bool periodic = false,
                            const std::vector<double>& parameters = {}, bool scale = true,
                            double tol = 1e-6);
    static Edge make_spline_approx(const std::vector<Vector>& points, double tol = 1e-3,
                                   std::optional<std::array<double, 3>> smoothing = std::nullopt,
                                   int min_deg = 1, int max_deg = 6);
    static Edge make_bezier(const std::vector<Vector>& points,
                            const std::vector<double>& weights = {});
    /// Exact B-spline; repeated knot values give the knot multiplicity.
    static Edge make_bspline(const std::vector<Vector>& control_points, const std::vector<double>& knots,
                             int degree, const std::vector<double>& weights = {}, bool periodic = false);
    /// Parabola arc with its vertex at the plane origin; angles are curve parameters in degrees.
    static Edge make_parabola(double focal_length, const Plane& plane = Plane::XY,
                              double start_angle = 0.0, double end_angle = 90.0,
                              AngularDirection angular_direction = AngularDirection::COUNTER_CLOCKWISE);
    /// Hyperbola (or an arc of it) centered at the plane origin.
    static Edge make_hyperbola(double x_radius, double y_radius, const Plane& plane = Plane::XY,
                               double start_angle = 360.0, double end_angle = 360.0,
                               AngularDirection angular_direction = AngularDirection::COUNTER_CLOCKWISE);
    static Edge make_three_point_arc(const Vector& point1, const Vector& point2,
                                     const Vector& point3);
    static Edge make_tangent_arc(const Vector& start, const Vector& tangent, const Vector& end);
    static Edge make_mid_way(const Edge& first, const Edge& second, double middle = 0.5);
    static Edge extrude(const Vertex& obj, const Vector& direction);
    /// Arcs (or circles) on Plane::XY tangent to curves or through points: two targets with a
    /// radius or a center_on locus, three targets, or one target with a center or with a
    /// radius and center_on (see ConstrainedArcOptions and constrained_lines.py).
    static ShapeList<Edge> make_constrained_arcs(const TangencyTarget& tangency_one,
                                                 const ConstrainedArcOptions& options);
    static ShapeList<Edge> make_constrained_arcs(const TangencyTarget& tangency_one,
                                                 const TangencyTarget& tangency_two,
                                                 const ConstrainedArcOptions& options);
    static ShapeList<Edge> make_constrained_arcs(const TangencyTarget& tangency_one,
                                                 const TangencyTarget& tangency_two,
                                                 const TangencyTarget& tangency_three);
    static ShapeList<Edge> make_constrained_arcs(const TangencyTarget& tangency_one,
                                                 const TangencyTarget& tangency_two,
                                                 const TangencyTarget& tangency_three,
                                                 const ConstrainedArcOptions& options);
    /// Lines on Plane::XY tangent to two curves, to a curve and through a point, or tangent
    /// to a curve at an angle (or direction) to a reference Axis.
    static ShapeList<Edge> make_constrained_lines(const TangencyTarget& tangency_one,
                                                  const TangencyTarget& tangency_two);
    static ShapeList<Edge> make_constrained_lines(const TangencyTarget& tangency_one,
                                                  const TangencyTarget& tangency_two,
                                                  const ConstrainedLineOptions& options);

    /// Center of an underlying circle or ellipse.
    Vector arc_center() const;
    Edge reversed(bool reconstruct = false) const;
    /// Keep the section between normalized positions `start` and `end`.
    Edge trim(double start, double end) const;
    Edge trim(const Vector& start, const Vector& end) const;
    Edge trim_to_length(double start, double length) const;
    /// The shortest part of this edge from its start to where it meets `other` (a Shape, Axis,
    /// Location, Plane or point), or nothing when they don't meet.
    std::optional<Edge> trim_to_other(const Shape& other) const;
    std::optional<Edge> trim_to_other(const Axis& other) const;
    std::optional<Edge> trim_to_other(const Location& other) const;
    std::optional<Edge> trim_to_other(const Plane& other) const;
    std::optional<Edge> trim_to_other(const Vector& other) const;
    /// An infinite line trimmed to `half_length` on each side of its center; other edges are
    /// returned unchanged.
    Edge trim_infinite(double half_length) const;
    /// OCCT curve parameter at a normalized arc-length position.
    double param_at(double position) const;
    Wire close() const;
    bool is_infinite() const;
    /// Geometric (not topological) equality within `tol`: same curve type, end points and
    /// defining data; unknown curve types compare `num_interpolation_points` samples.
    bool geom_equal(const Edge& other, double tol = 1e-6, int num_interpolation_points = 5) const;
    /// Intersection points with another edge or an axis.
    std::vector<Vector> find_intersection_points(const Edge& other,
                                                 double tolerance = TOLERANCE) const;
    std::vector<Vector> find_intersection_points(const Axis& axis,
                                                 double tolerance = TOLERANCE) const;
    /// Project onto a shape along `direction` (parallel) or from `center` (conical); exactly
    /// one of them must be given. The edges of the projected wires.
    ShapeList<Edge> project_to_shape(const Shape& target_object, std::optional<Vector> direction = std::nullopt,
                                     std::optional<Vector> center = std::nullopt) const;

    BUILD123D_TYPED_OPS(Edge)
};

class Wire : public Mixin1D {
public:
    static constexpr int kDim = 1;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_WIRE;
    using topo_type = Wire;

    Wire() = default;
    explicit Wire(const TopoDS_Shape& obj);
    explicit Wire(const Edge& edge);
    /// Wire from (possibly unsorted) connected edges.
    explicit Wire(const std::vector<Edge>& edges, bool sequenced = false);
    explicit Wire(const Curve& curve);

    /// Combine edges/wires into as few wires as possible.
    static ShapeList<Wire> combine(const std::vector<Edge>& edges, double tol = 1e-9);
    static ShapeList<Wire> combine(const std::vector<Wire>& wires, double tol = 1e-9);
    static Wire make_polygon(const std::vector<Vector>& vertices, bool close = true);
    static Wire make_rect(double width, double height, const Plane& plane = Plane::XY);
    static Wire make_circle(double radius, const Plane& plane = Plane::XY);
    static Wire make_ellipse(double x_radius, double y_radius, const Plane& plane = Plane::XY,
                             double start_angle = 360.0, double end_angle = 360.0,
                             AngularDirection angular_direction = AngularDirection::COUNTER_CLOCKWISE,
                             bool closed = true);
    /// Shortest wire enclosing the given planar (XY) edges.
    static Wire make_convex_hull(const std::vector<Edge>& edges, double tolerance = 1e-3);

    /// Edges in connection order (narrowed by `select`, see Shape::edges).
    ShapeList<Edge> edges(Select select = Select::ALL) const;
    ShapeList<Edge> order_edges() const;
    Wire fillet_2d(double radius, const std::vector<Vertex>& vertices) const;
    Wire chamfer_2d(double distance, double distance2, const std::vector<Vertex>& vertices,
                    std::optional<Edge> edge = std::nullopt) const;
    Wire close() const;
    Wire fix_degenerate_edges(double precision) const;
    Wire stitch(const Wire& other) const;
    Wire trim(double start, double end) const;
    /// Project onto a shape along `direction` (parallel) or from `center` (conical); exactly
    /// one of them must be given. The wires keep this wire's orientation and are sorted by
    /// distance (from this wire, or from the center).
    ShapeList<Wire> project_to_shape(const Shape& target_object, std::optional<Vector> direction = std::nullopt,
                                     std::optional<Vector> center = std::nullopt) const;

    BUILD123D_TYPED_OPS(Wire)
};

/// A target of Edge::make_constrained_arcs/lines, ConstrainedArcs and ConstrainedLines: a
/// curve the result is tangent to (an Edge or an Axis, optionally qualified with a Tangency,
/// Python's `(edge, Tangency.ENCLOSING)` tuple) or a point it passes through (a Vector, a
/// Vertex or {x, y}).
class TangencyTarget {
public:
    enum class Kind { EDGE, AXIS, VERTEX, POINT };

    TangencyTarget(const Edge& edge);  // NOLINT(google-explicit-constructor)
    TangencyTarget(const Edge& edge, Tangency tangency);
    TangencyTarget(const Axis& axis);  // NOLINT
    TangencyTarget(const Axis& axis, Tangency tangency);
    TangencyTarget(const Vertex& vertex);  // NOLINT
    TangencyTarget(const Vector& point);   // NOLINT
    TangencyTarget(double x, double y, double z = 0.0);

    Kind kind() const { return kind_; }
    /// True when given with a Tangency (a tuple in Python).
    bool qualified() const { return qualified_; }
    Tangency tangency() const { return tangency_; }
    const Edge& edge() const;
    const Axis& axis() const;
    const Vertex& vertex() const;
    const Vector& point() const;

private:
    Kind kind_;
    bool qualified_ = false;
    Tangency tangency_ = Tangency::UNQUALIFIED;
    std::optional<Edge> edge_;
    std::optional<Axis> axis_;
    std::optional<Vertex> vertex_;
    Vector point_;
};

/// The keyword arguments of Edge::make_constrained_arcs (and ConstrainedArcs).
struct ConstrainedArcOptions {
    std::optional<double> radius;
    /// The locus the arc's center lies on (an Edge or Axis).
    std::optional<TangencyTarget> center_on;
    /// The fixed center (a point).
    std::optional<TangencyTarget> center;
    Sagitta sagitta = Sagitta::SHORT;
};

/// The keyword arguments of Edge::make_constrained_lines (and ConstrainedLines): the line's
/// angle in degrees from the reference Axis, or its direction.
struct ConstrainedLineOptions {
    std::optional<double> angle;
    std::optional<Vector> direction;
};

// ---------------------------------------------------------------------------
// 2D
// ---------------------------------------------------------------------------

/// Methods shared by Face and Shell.
class Mixin2D : public Shape {
public:
    static constexpr int kDim = 2;
    using Shape::Shape;

    /// Hidden-line projection onto a viewport (see Mixin1D::project_to_viewport).
    std::pair<ShapeList<Edge>, ShapeList<Edge>> project_to_viewport(
        const Vector& viewport_origin, const Vector& viewport_up = Vector(0, 0, 1),
        std::optional<Vector> look_at = std::nullopt,
        std::optional<double> focus = std::nullopt) const;
};

class Face : public Mixin2D {
public:
    static constexpr int kDim = 2;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_FACE;
    using topo_type = Face;

    Face() = default;
    explicit Face(const TopoDS_Shape& obj);
    /// Planar face from a closed outer wire and optional hole wires.
    explicit Face(const Wire& outer_wire, const std::vector<Wire>& inner_wires = {});

    static Face make_rect(double width, double height, const Plane& plane = Plane::XY);
    /// Infinite planar face.
    static Face make_plane(const Plane& plane = Plane::XY);
    /// Non-planar surface bounded by `exterior`, optionally through `surface_points`.
    static Face make_surface(const Wire& exterior, const std::vector<Vector>& surface_points = {},
                             const std::vector<Wire>& interior_wires = {});
    static Face make_surface_from_curves(const Edge& curve1, const Edge& curve2);
    static Face make_surface_from_curves(const Wire& curve1, const Wire& curve2);
    static Face make_bezier_surface(const std::vector<std::vector<Vector>>& points,
                                    const std::vector<std::vector<double>>& weights = {});
    /// Approximating spline surface through a grid of points (rows are V, columns U).
    static Face make_surface_from_array_of_points(
        const std::vector<std::vector<Vector>>& points, double tol = 1e-2,
        std::optional<std::array<double, 3>> smoothing = std::nullopt, int min_deg = 1,
        int max_deg = 3);
    static Face revolve(const Edge& profile, double angle, const Axis& axis);
    static Face extrude(const Edge& obj, const Vector& direction);
    static Face sweep(const Shape& profile, const Shape& path,
                      Transition transition = Transition::TRANSFORMED);

    /// Flip the normal (build123d's unary minus).
    Face operator-() const;

    Vector normal_at() const;
    Vector normal_at(const Vector& surface_point) const;
    Vector normal_at(double u, double v) const;
    Vector position_at(double u, double v) const;
    Location location_at(double u = 0.5, double v = 0.5,
                         std::optional<Vector> x_dir = std::nullopt) const;
    Location location_at(const Vector& surface_point,
                         std::optional<Vector> x_dir = std::nullopt) const;
    Location center_location() const;
    Wire outer_wire() const;
    ShapeList<Wire> inner_wires() const;
    /// The outer wire; warns when the face has holes (build123d's Face.wire).
    Wire wire() const;
    /// Does the face curve away from the material behind it (convexity() == CONVEX)?
    bool is_circular_convex() const;
    /// Does the face curve into the material behind it (convexity() == CONCAVE)?
    bool is_circular_concave() const;
    /// The plane of a planar face (std::nullopt if not planar). Usable as a bool.
    std::optional<Plane> is_planar() const;
    bool is_coplanar(const Plane& plane) const;
    bool is_inside(const Vector& point, double tolerance = 1.0e-6) const;
    std::optional<double> width() const;
    std::optional<double> length() const;
    std::optional<double> radius() const;
    std::optional<Axis> axis_of_rotation() const;
    /// "POLYGON", "RECTANGLE" or "SQUARE" for planar straight-edged faces.
    std::optional<std::string> geometry() const;
    double area_without_holes() const;

    Face fillet_2d(double radius, const std::vector<Vertex>& vertices) const;
    Face chamfer_2d(double distance, double distance2, const std::vector<Vertex>& vertices,
                    std::optional<Edge> edge = std::nullopt) const;
    Face make_holes(const std::vector<Wire>& interior_wires) const;
    Face without_holes() const;
    /// Project onto a target along `direction`. Results are sorted along the direction;
    /// each is a Face, or a Shell where the projection spans several target faces.
    ShapeList<Shape> project_to_shape(const Shape& target, const Vector& direction) const;

    BUILD123D_TYPED_OPS(Face)
};

class Shell : public Mixin2D {
public:
    static constexpr int kDim = 2;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_SHELL;
    using topo_type = Shell;

    Shell() = default;
    explicit Shell(const TopoDS_Shape& obj);
    explicit Shell(const Face& face);
    /// Sew faces into a shell.
    explicit Shell(const std::vector<Face>& faces);

    static Shell make_loft(const std::vector<Shape>& sections, bool ruled = false);
    static Shell extrude(const Wire& obj, const Vector& direction);
    static Shell revolve(const Shape& profile, double angle, const Axis& axis);
    static Shell sweep(const Shape& profile, const Shape& path,
                       Transition transition = Transition::TRANSFORMED);

    /// Enclosed volume of a closed shell, else 0.
    double volume() const;

    Shell operator-() const;

    BUILD123D_TYPED_OPS(Shell)
};

// ---------------------------------------------------------------------------
// 3D
// ---------------------------------------------------------------------------

/// Methods shared by Solid and Compound.
class Mixin3D : public Shape {
public:
    using Shape::Shape;

    bool is_inside(const Vector& point, double tolerance = 1.0e-6) const;
    /// Largest working fillet radius for the edges (binary search).
    double max_fillet(const std::vector<Edge>& edge_list, double tolerance = 0.1,
                      int max_iterations = 10) const;
    /// Hidden-line projection onto a viewport (see Mixin1D::project_to_viewport).
    std::pair<ShapeList<Edge>, ShapeList<Edge>> project_to_viewport(
        const Vector& viewport_origin, const Vector& viewport_up = Vector(0, 0, 1),
        std::optional<Vector> look_at = std::nullopt,
        std::optional<double> focus = std::nullopt) const;

protected:
    TopoDS_Shape fillet_topods(double radius, const std::vector<Edge>& edge_list,
                               std::shared_ptr<ShapeHistory>* history = nullptr) const;
    TopoDS_Shape chamfer_topods(double length, std::optional<double> length2,
                                const std::vector<Edge>& edge_list,
                                const std::optional<Face>& face,
                                std::shared_ptr<ShapeHistory>* history = nullptr) const;
    TopoDS_Shape offset_3d_topods(const std::vector<Face>& openings, double thickness,
                                  double tolerance, Kind kind) const;
};

#define BUILD123D_3D_OPS(T)                                                                    \
public:                                                                                        \
    /** Fillet edges of this shape; a new shape (no attributes) recording its history. */     \
    T fillet(double radius, const std::vector<Edge>& edge_list) const {                        \
        std::shared_ptr<ShapeHistory> fillet_history;                                          \
        T result(fillet_topods(radius, edge_list, &fillet_history));                           \
        result.set_history(std::move(fillet_history));                                         \
        return result;                                                                         \
    }                                                                                          \
    T chamfer(double length, std::optional<double> length2, const std::vector<Edge>& edge_list, \
              const std::optional<Face>& face = std::nullopt) const {                          \
        std::shared_ptr<ShapeHistory> chamfer_history;                                         \
        T result(chamfer_topods(length, length2, edge_list, face, &chamfer_history));          \
        result.set_history(std::move(chamfer_history));                                        \
        return result;                                                                         \
    }                                                                                          \
    /** Hollow (shell) this object; negative thickness offsets inwards. */                     \
    T offset_3d(const std::vector<Face>& openings, double thickness, double tolerance = 1e-4,  \
                Kind kind = Kind::ARC) const {                                                 \
        return T(offset_3d_topods(openings, thickness, tolerance, kind));                     \
    }

class Solid : public Mixin3D {
public:
    static constexpr int kDim = 3;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_SOLID;
    using topo_type = Solid;

    Solid() = default;
    explicit Solid(const TopoDS_Shape& obj);
    explicit Solid(const Shell& shell);

    static Solid make_box(double length, double width, double height,
                          const Plane& plane = Plane::XY);
    static Solid make_cylinder(double radius, double height, const Plane& plane = Plane::XY,
                               double angle = 360.0);
    static Solid make_cone(double base_radius, double top_radius, double height,
                           const Plane& plane = Plane::XY, double angle = 360.0);
    static Solid make_sphere(double radius, const Plane& plane = Plane::XY, double angle1 = -90.0,
                             double angle2 = 90.0, double angle3 = 360.0);
    static Solid make_torus(double major_radius, double minor_radius,
                            const Plane& plane = Plane::XY, double start_angle = 0.0,
                            double end_angle = 360.0, double major_angle = 360.0);
    static Solid make_wedge(double delta_x, double delta_y, double delta_z, double min_x,
                            double min_z, double max_x, double max_z,
                            const Plane& plane = Plane::XY);
    /// Loft through wires; a Vertex may start and/or end the sequence.
    static Solid make_loft(const std::vector<Shape>& sections, bool ruled = false);
    static Solid extrude(const Face& obj, const Vector& direction);
    static Solid extrude_taper(const Face& profile, const Vector& direction, double taper,
                               bool flip_inner = true);
    static Solid extrude_until(const Face& profile, const Shape& target, const Vector& direction,
                               Until until = Until::NEXT);
    static Solid extrude_linear_with_rotation(const Face& section, const Vector& center,
                                              const Vector& normal, double angle);
    static Solid revolve(const Face& section, double angle, const Axis& axis);
    /// Sweep a face (or closed wire) along a path. `binormal` fixes the profile
    /// orientation (a fixed direction or an auxiliary wire).
    static Solid sweep(const Shape& section, const Shape& path, bool make_solid = true,
                       bool is_frenet = false, std::optional<Vector> binormal_dir = std::nullopt,
                       const std::optional<Wire>& binormal_wire = std::nullopt,
                       Transition transition = Transition::TRANSFORMED);
    static Solid sweep_multi(const std::vector<Shape>& profiles, const Shape& path,
                             bool make_solid = true, bool is_frenet = false,
                             std::optional<Vector> binormal_dir = std::nullopt,
                             const std::optional<Wire>& binormal_wire = std::nullopt);
    static Solid thicken(const Shape& surface, double depth,
                         std::optional<Vector> normal_override = std::nullopt);
    static Solid from_bounding_box(const BoundBox& bbox);
    /// A box of the oriented bounding box's size, placed at its location.
    static Solid from_bounding_box(const OrientedBoundBox& bbox);

    double volume() const;
    /// Draft angle (degrees) applied to planar, cylindrical or conical faces; throws
    /// DraftAngleError when OCCT can't draft a face or build the result.
    Solid draft(const std::vector<Face>& faces, const Plane& neutral_plane, double angle) const;

    BUILD123D_TYPED_OPS(Solid)
    BUILD123D_3D_OPS(Solid)
};

/// build123d's DraftAngleError (a RuntimeError): the face that failed (if known) and the
/// shape OCCT reported as problematic.
class DraftAngleError : public std::runtime_error {
public:
    DraftAngleError(const std::string& message, std::optional<Face> face = std::nullopt,
                    Shape problematic_shape = Shape());
    std::optional<Face> face;
    Shape problematic_shape;
};

// ---------------------------------------------------------------------------
// Composites
// ---------------------------------------------------------------------------

/// A collection of shapes. Curve, Sketch and Part are Compounds of 1D, 2D and 3D shapes.
class Compound : public Mixin3D {
public:
    static constexpr int kDim = -1;
    static constexpr TopAbs_ShapeEnum kTopAbs = TopAbs_COMPOUND;
    using topo_type = Compound;

    Compound() = default;
    /// Wraps a non-compound shape into a single element compound.
    explicit Compound(const TopoDS_Shape& obj);
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    explicit Compound(const std::vector<S>& shapes);
    /// An assembly whose children keep their labels and colors.
    static Compound make_assembly(const std::vector<Shape>& children, const std::string& label = "");

    /// Number of direct children.
    std::size_t size() const;
    /// Direct children.
    ShapeList<Shape> elements() const;
    /// Children of the given topological type, of this compound and of its direct
    /// sub-compounds (build123d's get_type).
    template <class T>
    ShapeList<T> get_type() const;
    /// Remove unnecessary compound wrappers.
    Shape unwrap(bool fully = true) const;
    double volume() const;

    /// Text as faces. See Text in objects.hpp for the full set of options.
    static Compound make_text(const std::string& txt, double font_size,
                              const std::string& font = "Arial", const std::string& font_path = "",
                              FontStyle font_style = FontStyle::REGULAR,
                              std::pair<TextAlign, TextAlign> text_align = {TextAlign::CENTER,
                                                                            TextAlign::CENTER},
                              std::optional<Align2> align = std::nullopt,
                              double position_on_path = 0.0,
                              const std::optional<Wire>& text_path = std::nullopt);

    BUILD123D_TYPED_OPS(Compound)
    BUILD123D_3D_OPS(Compound)

protected:
    static TopoDS_Shape make_compound(const std::vector<TopoDS_Shape>& shapes);
    static TopoDS_Shape ensure_compound(const TopoDS_Shape& obj);
    void check_dim(int expected, const char* name) const;
};

/// A Compound of edges/wires.
class Curve : public Compound {
public:
    static constexpr int kDim = 1;
    using topo_type = Curve;

    Curve() = default;
    explicit Curve(const TopoDS_Shape& obj);
    Curve(const Compound& compound);  // NOLINT: checked at runtime
    Curve(const Edge& edge);          // NOLINT
    Curve(const Wire& wire);          // NOLINT
    Curve(const Sketch&) = delete;
    Curve(const Part&) = delete;
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    explicit Curve(const std::vector<S>& shapes) : Curve(Compound(shapes)) {}

    /// Wires made by combining the edges (those `select` keeps).
    ShapeList<Wire> wires(Select select = Select::ALL) const;
    double length() const;
    Vector position_at(double position, PositionMode mode = PositionMode::PARAMETER) const;
    Vector tangent_at(double position = 0.5, PositionMode mode = PositionMode::PARAMETER) const;
    Location location_at(double position, PositionMode mode = PositionMode::PARAMETER,
                         FrameMethod frame_method = FrameMethod::FRENET) const;

    BUILD123D_TYPED_OPS(Curve)
};

Vector operator%(const Curve& curve, double position);
Location operator^(const Curve& curve, double position);

/// A Compound of faces.
class Sketch : public Compound {
public:
    static constexpr int kDim = 2;
    using topo_type = Sketch;

    Sketch() = default;
    explicit Sketch(const TopoDS_Shape& obj);
    Sketch(const Compound& compound);  // NOLINT: checked at runtime
    Sketch(const Face& face);          // NOLINT
    Sketch(const Curve&) = delete;
    Sketch(const Part&) = delete;
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    explicit Sketch(const std::vector<S>& shapes) : Sketch(Compound(shapes)) {}

    BUILD123D_TYPED_OPS(Sketch)
};

/// A Compound of solids.
class Part : public Compound {
public:
    static constexpr int kDim = 3;
    using topo_type = Part;

    Part() = default;
    explicit Part(const TopoDS_Shape& obj);
    Part(const Compound& compound);  // NOLINT: checked at runtime
    Part(const Solid& solid);        // NOLINT
    Part(const Curve&) = delete;
    Part(const Sketch&) = delete;
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    explicit Part(const std::vector<S>& shapes) : Part(Compound(shapes)) {}

    BUILD123D_TYPED_OPS(Part)
    BUILD123D_3D_OPS(Part)
};

// ---------------------------------------------------------------------------
// Internal helpers used by the templates below (defined in the library).
// ---------------------------------------------------------------------------
namespace detail {

template <class T>
inline constexpr bool is_shape_v = std::is_base_of_v<Shape, std::decay_t<T>>;

template <int D>
struct algebra_for {
    using type = Compound;
};
template <>
struct algebra_for<1> {
    using type = Curve;
};
template <>
struct algebra_for<2> {
    using type = Sketch;
};
template <>
struct algebra_for<3> {
    using type = Part;
};

enum class AlgebraOp { ADD, SUB, AND };

/// Which build123d class implements an operator for the left operand: Shape's own,
/// Compound's (Sketch, Part, Compound) or Curve's (Compound's with Mixin1D's `+`).
enum class SelfKind { SHAPE, COMPOUND, CURVE };

/// The +, -, & operations; `dim` is the static dimension of `a` (-1 if dynamic). The result
/// carries the operation's record (history) and the attributes build123d copies.
Shape algebra(AlgebraOp op, const Shape& a, const std::vector<const Shape*>& b, int dim, SelfKind kind);
/// build123d's Shape.__and__ / Compound.__and__ with any intersect operands (chained).
Shape algebra_and(const Shape& a, const std::vector<IntersectArg>& others, SelfKind kind);

std::vector<TopoDS_Shape> list_entities(const TopoDS_Shape& shape, int topabs_type);
double sort_key_axis(const Shape& s, const Axis& axis);
double sort_key(const Shape& s, SortBy sort_by);
double sort_key_path(const Shape& s, const Mixin1D& path);
bool axis_parallel_predicate(const Shape& s, const Axis& axis, double tolerance);
bool plane_parallel_predicate(const Shape& s, const Plane& plane, double tolerance);
double round_digits(double value, int digits);
Vector shape_center(const Shape& s);
double shape_distance(const Shape& s, const Shape& other);
double shape_distance(const Shape& s, const Vector& point);
TopoDS_Shape make_compound_topods(const std::vector<TopoDS_Shape>& shapes);
int topods_type(const TopoDS_Shape& shape);
[[noreturn]] void throw_count(const char* type, std::size_t count);

}  // namespace detail

/// Curve, Sketch or Part depending on the static dimension of T (Compound if dynamic).
template <class T>
using algebra_t = typename detail::algebra_for<std::decay_t<T>::kDim>::type;

// ---------------------------------------------------------------------------
// GroupBy / ShapeList
// ---------------------------------------------------------------------------

/// Tag for ShapeList::group_by(by_convexity), build123d's group_by(Convexity).
struct ConvexityKey {};
inline constexpr ConvexityKey by_convexity{};

/// Result of ShapeList::group_by: sorted groups, indexable (negative indices allowed).
template <class T>
class GroupBy : public std::vector<ShapeList<T>> {
public:
    using Base = std::vector<ShapeList<T>>;
    std::vector<double> keys;
    std::function<double(const T&)> key_f;

    ShapeList<T>& operator[](std::ptrdiff_t i) { return Base::at(norm(i)); }
    const ShapeList<T>& operator[](std::ptrdiff_t i) const { return Base::at(norm(i)); }
    /// The group with the given key.
    const ShapeList<T>& group(double key) const {
        for (std::size_t i = 0; i < keys.size(); ++i) {
            if (keys[i] == key) return Base::at(i);
        }
        throw std::out_of_range("GroupBy: no group with key " + std::to_string(key));
    }
    /// The group containing a shape like the given one.
    const ShapeList<T>& group_for(const T& shape) const { return group(key_f(shape)); }

private:
    std::size_t norm(std::ptrdiff_t i) const {
        const auto n = static_cast<std::ptrdiff_t>(Base::size());
        if (i < 0) i += n;
        if (i < 0 || i >= n) throw std::out_of_range("GroupBy index out of range");
        return static_cast<std::size_t>(i);
    }
};

/// A std::vector of shapes with build123d's selectors (sort_by, group_by, filter_by, ...).
/// Supports Python-style negative indices: list[-1] is the last element.
template <class T>
class ShapeList : public std::vector<T> {
public:
    using Base = std::vector<T>;
    using Base::Base;
    ShapeList() = default;
    ShapeList(const Base& v) : Base(v) {}        // NOLINT
    ShapeList(Base&& v) : Base(std::move(v)) {}  // NOLINT

    T& operator[](std::ptrdiff_t i) { return Base::at(norm(i)); }
    const T& operator[](std::ptrdiff_t i) const { return Base::at(norm(i)); }
    const T& first() const { return (*this)[0]; }
    const T& last() const { return (*this)[-1]; }
    /// Python slicing list[start:stop].
    ShapeList slice(std::ptrdiff_t start,
                    std::optional<std::ptrdiff_t> stop = std::nullopt) const {
        const auto n = static_cast<std::ptrdiff_t>(this->size());
        auto clamp = [n](std::ptrdiff_t i) {
            if (i < 0) i += n;
            return std::min(std::max<std::ptrdiff_t>(i, 0), n);
        };
        const std::ptrdiff_t b = clamp(start);
        const std::ptrdiff_t e = stop ? clamp(*stop) : n;
        ShapeList out;
        for (std::ptrdiff_t i = b; i < e; ++i) out.push_back(Base::at(static_cast<std::size_t>(i)));
        return out;
    }

    // ---- sorting
    ShapeList sort_by(const Axis& axis = Axis::Z, bool reverse = false) const {
        return sorted([&](const T& s) { return detail::sort_key_axis(s, axis); }, reverse);
    }
    ShapeList sort_by(SortBy sort_by, bool reverse = false) const {
        return sorted([&](const T& s) { return detail::sort_key(s, sort_by); }, reverse);
    }
    /// Sort by the parameter of the closest point on a path (Edge or Wire).
    ShapeList sort_by(const Mixin1D& path, bool reverse = false) const {
        return sorted([&](const T& s) { return detail::sort_key_path(s, path); }, reverse);
    }
    template <class F, std::enable_if_t<std::is_invocable_v<F, const T&>, int> = 0>
    ShapeList sort_by(F key, bool reverse = false) const {
        return sorted([&](const T& s) { return static_cast<double>(key(s)); }, reverse);
    }
    ShapeList sort_by_distance(const Shape& other, bool reverse = false) const {
        return sorted([&](const T& s) { return detail::shape_distance(s, other); }, reverse);
    }
    ShapeList sort_by_distance(const Vector& other, bool reverse = false) const {
        return sorted([&](const T& s) { return detail::shape_distance(s, other); }, reverse);
    }

    // ---- grouping
    GroupBy<T> group_by(const Axis& axis = Axis::Z, bool reverse = false,
                        int tol_digits = 6) const {
        return grouped([axis](const T& s) { return detail::sort_key_axis(s, axis); }, reverse,
                       tol_digits);
    }
    GroupBy<T> group_by(SortBy sort_by, bool reverse = false, int tol_digits = 6) const {
        return grouped([sort_by](const T& s) { return detail::sort_key(s, sort_by); }, reverse,
                       tol_digits);
    }
    GroupBy<T> group_by(const Mixin1D& path, bool reverse = false, int tol_digits = 6) const {
        const Mixin1D p = path;
        return grouped([p](const T& s) { return detail::sort_key_path(s, p); }, reverse,
                       tol_digits);
    }
    template <class F, std::enable_if_t<std::is_invocable_v<F, const T&>, int> = 0>
    GroupBy<T> group_by(F key, bool reverse = false, int tol_digits = 6) const {
        return grouped([key](const T& s) { return static_cast<double>(key(s)); }, reverse,
                       tol_digits);
    }
    /// build123d's group_by(Convexity): groups of equal convexity in the enum's order
    /// (the group keys are static_cast<double>(Convexity)).
    GroupBy<T> group_by(ConvexityKey, bool reverse = false, int tol_digits = 6) const {
        return grouped([](const T& s) { return static_cast<double>(s.convexity()); }, reverse, tol_digits);
    }

    // ---- filtering
    /// Planar faces with normals, or lines with directions, parallel to the axis.
    ShapeList filter_by(const Axis& axis, bool reverse = false, double tolerance = 1e-5) const {
        return filtered(
            [&](const T& s) { return detail::axis_parallel_predicate(s, axis, tolerance); },
            reverse);
    }
    /// Faces/edges/wires parallel to the plane.
    ShapeList filter_by(const Plane& plane, bool reverse = false, double tolerance = 1e-5) const {
        return filtered(
            [&](const T& s) { return detail::plane_parallel_predicate(s, plane, tolerance); },
            reverse);
    }
    ShapeList filter_by(GeomType geom_type, bool reverse = false) const {
        return filtered([&](const T& s) { return s.geom_type() == geom_type; }, reverse);
    }
    /// Objects with the given convexity relative to the shape they were selected from.
    ShapeList filter_by(Convexity convexity, bool reverse = false) const {
        return filtered([&](const T& s) { return s.convexity() == convexity; }, reverse);
    }
    template <class F, std::enable_if_t<std::is_invocable_r_v<bool, F, const T&>, int> = 0>
    ShapeList filter_by(F predicate, bool reverse = false) const {
        return filtered([&](const T& s) { return static_cast<bool>(predicate(s)); }, reverse);
    }
    /// Objects whose centers lie between minimum and maximum along the axis, sorted by it.
    ShapeList filter_by_position(const Axis& axis, double minimum, double maximum,
                                 std::pair<bool, bool> inclusive = {true, true}) const {
        ShapeList out;
        for (const T& s : *this) {
            const double z = detail::sort_key_axis(s, axis);
            const bool lo = inclusive.first ? (minimum <= z) : (minimum < z);
            const bool hi = inclusive.second ? (z <= maximum) : (z < maximum);
            if (lo && hi) out.push_back(s);
        }
        return out.sort_by(axis);
    }

    // ---- sub-shapes
    ShapeList<Vertex> vertices() const { return gather([](const T& s) { return s.vertices(); }); }
    ShapeList<Edge> edges() const { return gather([](const T& s) { return s.edges(); }); }
    ShapeList<Wire> wires() const { return gather([](const T& s) { return s.wires(); }); }
    ShapeList<Face> faces() const { return gather([](const T& s) { return s.faces(); }); }
    ShapeList<Shell> shells() const { return gather([](const T& s) { return s.shells(); }); }
    ShapeList<Solid> solids() const { return gather([](const T& s) { return s.solids(); }); }
    ShapeList<Compound> compounds() const {
        return gather([](const T& s) { return s.compounds(); });
    }
    Vertex vertex() const;
    Edge edge() const;
    Wire wire() const;
    Face face() const;
    Shell shell() const;
    Solid solid() const;
    /// Average of the centers.
    Vector center() const {
        if (this->empty()) return Vector(0, 0, 0);
        Vector total(0, 0, 0);
        for (const T& s : *this) total += detail::shape_center(s);
        return total / static_cast<double>(this->size());
    }

    // ---- set-like operators
    ShapeList operator+(const ShapeList& other) const {
        ShapeList out(*this);
        out.insert(out.end(), other.begin(), other.end());
        return out;
    }
    ShapeList& operator+=(const ShapeList& other) {
        this->insert(this->end(), other.begin(), other.end());
        return *this;
    }
    ShapeList& operator+=(const T& item) {
        this->push_back(item);
        return *this;
    }
    /// Elements not present (is_same) in `other`.
    ShapeList operator-(const ShapeList& other) const {
        ShapeList out;
        for (const T& s : *this) {
            if (!contains_shape(other, s) && !contains_shape(out, s)) out.push_back(s);
        }
        return out;
    }
    /// Elements present (is_same) in both lists.
    ShapeList operator&(const ShapeList& other) const {
        ShapeList out;
        for (const T& s : *this) {
            if (contains_shape(other, s) && !contains_shape(out, s)) out.push_back(s);
        }
        return out;
    }
    bool contains(const T& shape) const { return contains_shape(*this, shape); }

private:
    std::size_t norm(std::ptrdiff_t i) const {
        const auto n = static_cast<std::ptrdiff_t>(this->size());
        if (i < 0) i += n;
        if (i < 0 || i >= n) {
            throw std::out_of_range("ShapeList index " + std::to_string(i) + " out of range (size " +
                                    std::to_string(n) + ")");
        }
        return static_cast<std::size_t>(i);
    }
    template <class K>
    ShapeList sorted(K key, bool reverse) const {
        std::vector<std::pair<double, std::size_t>> keyed;
        keyed.reserve(this->size());
        for (std::size_t i = 0; i < this->size(); ++i) keyed.emplace_back(key(Base::at(i)), i);
        if (reverse) {
            std::stable_sort(keyed.begin(), keyed.end(),
                             [](const auto& a, const auto& b) { return a.first > b.first; });
        } else {
            std::stable_sort(keyed.begin(), keyed.end(),
                             [](const auto& a, const auto& b) { return a.first < b.first; });
        }
        ShapeList out;
        out.reserve(this->size());
        for (const auto& k : keyed) out.push_back(Base::at(k.second));
        return out;
    }
    template <class K>
    GroupBy<T> grouped(K key, bool reverse, int tol_digits) const {
        GroupBy<T> groups;
        groups.key_f = [key, tol_digits](const T& s) {
            return detail::round_digits(key(s), tol_digits);
        };
        std::vector<std::pair<double, std::size_t>> keyed;
        for (std::size_t i = 0; i < this->size(); ++i) {
            keyed.emplace_back(groups.key_f(Base::at(i)), i);
        }
        if (reverse) {
            std::stable_sort(keyed.begin(), keyed.end(),
                             [](const auto& a, const auto& b) { return a.first > b.first; });
        } else {
            std::stable_sort(keyed.begin(), keyed.end(),
                             [](const auto& a, const auto& b) { return a.first < b.first; });
        }
        for (const auto& k : keyed) {
            if (groups.keys.empty() || groups.keys.back() != k.first) {
                groups.keys.push_back(k.first);
                groups.emplace_back();
            }
            groups.back().push_back(Base::at(k.second));
        }
        return groups;
    }
    template <class P>
    ShapeList filtered(P pred, bool reverse) const {
        ShapeList out;
        for (const T& s : *this) {
            if (pred(s) != reverse) out.push_back(s);
        }
        return out;
    }
    template <class F>
    auto gather(F f) const {
        decltype(f(std::declval<const T&>())) out;
        for (const T& s : *this) {
            auto sub = f(s);
            out.insert(out.end(), sub.begin(), sub.end());
        }
        return out;
    }
    template <class L>
    static bool contains_shape(const L& list, const T& s) {
        for (const auto& o : list) {
            if (o.is_same(s)) return true;
        }
        return false;
    }
};

/// The result of Shape::split: the kept pieces, with each side apart (build123d returns
/// `(top, bottom)` for Keep.BOTH; here `auto [top, bottom] = shape.split(tool, Keep::BOTH)`).
class SplitResult : public ShapeList<Shape> {
public:
    using ShapeList<Shape>::ShapeList;
    SplitResult() = default;
    /// The pieces above the tool (none for Keep::BOTTOM and Keep::ALL).
    ShapeList<Shape> top;
    /// The pieces below the tool (none for Keep::TOP and Keep::ALL).
    ShapeList<Shape> bottom;
};

template <std::size_t I>
const ShapeList<Shape>& get(const SplitResult& r) {
    static_assert(I < 2, "SplitResult unpacks into (top, bottom)");
    if constexpr (I == 0) {
        return r.top;
    } else {
        return r.bottom;
    }
}

template <class T>
ShapeList<T> operator>(const ShapeList<T>& l, const Axis& axis) { return l.sort_by(axis); }
template <class T>
ShapeList<T> operator>(const ShapeList<T>& l, SortBy s) { return l.sort_by(s); }
template <class T>
ShapeList<T> operator<(const ShapeList<T>& l, const Axis& axis) { return l.sort_by(axis, true); }
template <class T>
ShapeList<T> operator<(const ShapeList<T>& l, SortBy s) { return l.sort_by(s, true); }
/// Last (largest) group.
template <class T>
ShapeList<T> operator>>(const ShapeList<T>& l, const Axis& axis) { return l.group_by(axis)[-1]; }
template <class T>
ShapeList<T> operator>>(const ShapeList<T>& l, SortBy s) { return l.group_by(s)[-1]; }
/// First (smallest) group.
template <class T>
ShapeList<T> operator<<(const ShapeList<T>& l, const Axis& axis) { return l.group_by(axis)[0]; }
template <class T>
ShapeList<T> operator<<(const ShapeList<T>& l, SortBy s) { return l.group_by(s)[0]; }
template <class T>
ShapeList<T> operator|(const ShapeList<T>& l, const Axis& axis) { return l.filter_by(axis); }
template <class T>
ShapeList<T> operator|(const ShapeList<T>& l, const Plane& plane) { return l.filter_by(plane); }
template <class T>
ShapeList<T> operator|(const ShapeList<T>& l, GeomType g) { return l.filter_by(g); }

template <class T>
Vertex ShapeList<T>::vertex() const {
    auto v = vertices();
    if (v.size() != 1) detail::throw_count("vertex", v.size());
    return v[0];
}
template <class T>
Edge ShapeList<T>::edge() const {
    auto v = edges();
    if (v.size() != 1) detail::throw_count("edge", v.size());
    return v[0];
}
template <class T>
Wire ShapeList<T>::wire() const {
    auto v = wires();
    if (v.size() != 1) detail::throw_count("wire", v.size());
    return v[0];
}
template <class T>
Face ShapeList<T>::face() const {
    auto v = faces();
    if (v.size() != 1) detail::throw_count("face", v.size());
    return v[0];
}
template <class T>
Shell ShapeList<T>::shell() const {
    auto v = shells();
    if (v.size() != 1) detail::throw_count("shell", v.size());
    return v[0];
}
template <class T>
Solid ShapeList<T>::solid() const {
    auto v = solids();
    if (v.size() != 1) detail::throw_count("solid", v.size());
    return v[0];
}

// ---------------------------------------------------------------------------
// Template member definitions
// ---------------------------------------------------------------------------

template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int>>
Compound::Compound(const std::vector<S>& shapes) {
    std::vector<TopoDS_Shape> topods;
    topods.reserve(shapes.size());
    for (const auto& s : shapes) {
        if (!s.is_null()) topods.push_back(s.wrapped());
    }
    obj_ = make_compound(topods);
}

template <class T>
ShapeList<T> Compound::get_type() const {
    static_assert(T::kTopAbs != TopAbs_SHAPE, "get_type needs a topological type");
    ShapeList<T> out;
    // as build123d: the children of each of compounds() (the direct sub-compounds, then this)
    for (const Compound& comp : compounds()) {
        for (const Shape& child : comp.elements()) {
            if (!child.is_null() && child.wrapped().ShapeType() == T::kTopAbs) {
                out.push_back(T(child.wrapped()));
            }
        }
    }
    return out;
}

template <class S>
S Plane::to_local_coords(const S& shape) const {
    return shape.transform_shape(forward_transform());
}

template <class S>
S Plane::from_local_coords(const S& shape) const {
    return shape.transform_shape(reverse_transform());
}

// ---------------------------------------------------------------------------
// Algebra operators
// ---------------------------------------------------------------------------

namespace detail {

/// The algebra result as type R, keeping the attributes and record of the operation.
template <class R>
R algebra_result(const Shape& shape) {
    R result(shape.wrapped());
    ShapeAccess::copy_attributes(shape, result);
    return result;
}

template <class A>
SelfKind self_kind(const A& a) {
    if constexpr (std::is_base_of_v<Curve, A>) {
        return SelfKind::CURVE;
    } else if constexpr (std::is_base_of_v<Compound, A>) {
        return SelfKind::COMPOUND;
    } else if constexpr (std::is_same_v<A, Shape>) {
        return !a.is_null() && a.wrapped().ShapeType() == TopAbs_COMPOUND ? SelfKind::COMPOUND : SelfKind::SHAPE;
    } else {
        return SelfKind::SHAPE;
    }
}

/// Geometry operands of & (Vector and Location act as 0D, Axis as 1D, Plane as 2D).
template <class G>
inline constexpr bool is_geometry_v =
    std::is_same_v<std::decay_t<G>, Vector> || std::is_base_of_v<Location, std::decay_t<G>> ||
    std::is_same_v<std::decay_t<G>, Axis> || std::is_same_v<std::decay_t<G>, Plane>;
template <class G>
inline constexpr int geometry_dim_v =
    std::is_same_v<std::decay_t<G>, Axis> ? 1 : (std::is_same_v<std::decay_t<G>, Plane> ? 2 : 0);

/// Static type of a & b: an intersection with a 3D shape has the other operand's dimension
/// (Part, Sketch or Curve); every other combination may mix dimensions (crossing curves
/// give vertices, crossing faces give edges, ...) and is a Compound.
template <int DA, int DB>
struct and_result_for {
    using type = Compound;
};
template <>
struct and_result_for<3, 3> {
    using type = Part;
};
template <>
struct and_result_for<3, 2> {
    using type = Sketch;
};
template <>
struct and_result_for<2, 3> {
    using type = Sketch;
};
template <>
struct and_result_for<3, 1> {
    using type = Curve;
};
template <>
struct and_result_for<1, 3> {
    using type = Curve;
};

template <class B>
std::vector<const Shape*> shape_ptrs(const std::vector<B>& bs) {
    std::vector<const Shape*> out;
    out.reserve(bs.size());
    for (const auto& b : bs) out.push_back(&b);
    return out;
}

template <class A, class B>
inline constexpr bool same_dim_v = A::kDim < 0 || B::kDim < 0 || A::kDim == B::kDim;

template <class A, class B>
inline constexpr bool sub_dim_ok_v = A::kDim < 0 || B::kDim < 0 || B::kDim >= A::kDim;

template <class A, class B>
using and_result_t = typename and_result_for<A::kDim, B::kDim>::type;
template <class A, class G>
using and_geometry_result_t = typename and_result_for<A::kDim, geometry_dim_v<G>>::type;

template <class A>
inline constexpr bool not_vertex_v = !std::is_same_v<std::decay_t<A>, Vertex>;

}  // namespace detail

/// Fuse: a + b (shapes must have the same dimension).
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B>, int> = 0>
algebra_t<A> operator+(const A& a, const B& b) {
    static_assert(detail::not_vertex_v<A>, "Vertex addition only supports a Vertex or a Vector");
    static_assert(detail::same_dim_v<A, B>, "Only shapes with the same dimension can be added");
    return detail::algebra_result<algebra_t<A>>(
        detail::algebra(detail::AlgebraOp::ADD, a, {&b}, A::kDim, detail::self_kind(a)));
}
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B>, int> = 0>
algebra_t<A> operator+(const A& a, const std::vector<B>& b) {
    static_assert(detail::not_vertex_v<A>, "Vertex addition only supports a Vertex or a Vector");
    static_assert(detail::same_dim_v<A, B>, "Only shapes with the same dimension can be added");
    return detail::algebra_result<algebra_t<A>>(
        detail::algebra(detail::AlgebraOp::ADD, a, detail::shape_ptrs(b), A::kDim, detail::self_kind(a)));
}

/// Cut: a - b (b must have the same or a higher dimension).
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B>, int> = 0>
algebra_t<A> operator-(const A& a, const B& b) {
    static_assert(detail::not_vertex_v<A>, "Vertex subtraction only supports a Vertex or a Vector");
    static_assert(detail::sub_dim_ok_v<A, B>,
                  "Only shapes with equal or greater dimension can be subtracted");
    return detail::algebra_result<algebra_t<A>>(
        detail::algebra(detail::AlgebraOp::SUB, a, {&b}, A::kDim, detail::self_kind(a)));
}
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B>, int> = 0>
algebra_t<A> operator-(const A& a, const std::vector<B>& b) {
    static_assert(detail::not_vertex_v<A>, "Vertex subtraction only supports a Vertex or a Vector");
    static_assert(detail::sub_dim_ok_v<A, B>,
                  "Only shapes with equal or greater dimension can be subtracted");
    return detail::algebra_result<algebra_t<A>>(
        detail::algebra(detail::AlgebraOp::SUB, a, detail::shape_ptrs(b), A::kDim, detail::self_kind(a)));
}

/// Intersect: a & b, build123d's intersect followed by clean. A list operand chains
/// (a & {b, c} is a ∩ b ∩ c); a Compound operand distributes over its elements.
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B>, int> = 0>
detail::and_result_t<A, B> operator&(const A& a, const B& b) {
    static_assert(detail::not_vertex_v<A>, "Vertices can't be intersected");
    return detail::algebra_result<detail::and_result_t<A, B>>(
        detail::algebra(detail::AlgebraOp::AND, a, {&b}, A::kDim, detail::self_kind(a)));
}
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B>, int> = 0>
detail::and_result_t<A, B> operator&(const A& a, const std::vector<B>& b) {
    static_assert(detail::not_vertex_v<A>, "Vertices can't be intersected");
    return detail::algebra_result<detail::and_result_t<A, B>>(
        detail::algebra(detail::AlgebraOp::AND, a, detail::shape_ptrs(b), A::kDim, detail::self_kind(a)));
}
/// Intersect a shape with a Vector, Location, Axis or Plane.
template <class A, class G,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_geometry_v<G>, int> = 0>
detail::and_geometry_result_t<A, G> operator&(const A& a, const G& g) {
    static_assert(detail::not_vertex_v<A>, "Vertices can't be intersected");
    return detail::algebra_result<detail::and_geometry_result_t<A, G>>(
        detail::algebra_and(a, {IntersectArg(static_cast<const std::conditional_t<std::is_base_of_v<Location, G>, Location, G>&>(g))},
                            detail::self_kind(a)));
}

/// A Vector, Location, Axis or Plane & a shape: Shape::intersect (an empty list for None).
template <class G, class S,
          std::enable_if_t<detail::is_geometry_v<G> && detail::is_shape_v<S>, int> = 0>
ShapeList<Shape> operator&(const G& g, const S& shape) {
    return g.intersect(static_cast<const Shape&>(shape));
}

namespace detail {
template <class B, class = void>
struct is_shape_vector : std::false_type {};
template <class B>
struct is_shape_vector<B, std::void_t<typename B::value_type>>
    : std::bool_constant<std::is_base_of_v<Shape, typename B::value_type> &&
                         std::is_base_of_v<std::vector<typename B::value_type>, B>> {};
template <class A, class B>
inline constexpr bool compound_assignable_v =
    is_shape_v<A> && (is_shape_v<B> || is_shape_vector<B>::value);
}  // namespace detail

template <class A, class B,
          std::enable_if_t<detail::compound_assignable_v<A, B>, int> = 0,
          std::enable_if_t<std::is_assignable_v<A&, algebra_t<A>>, int> = 0>
A& operator+=(A& a, const B& b) {
    a = a + b;
    return a;
}
template <class A, class B,
          std::enable_if_t<detail::compound_assignable_v<A, B>, int> = 0,
          std::enable_if_t<std::is_assignable_v<A&, algebra_t<A>>, int> = 0>
A& operator-=(A& a, const B& b) {
    a = a - b;
    return a;
}
template <class A, class B,
          std::enable_if_t<detail::is_shape_v<A> && detail::is_shape_v<B> &&
                               std::is_assignable_v<A&, detail::and_result_t<A, B>>,
                           int> = 0>
A& operator&=(A& a, const B& b) {
    a = a & b;
    return a;
}

// ---- placement: Location * shape, Plane * shape, [locations] * shape

template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
typename T::topo_type operator*(const Location& loc, const T& shape) {
    return shape.moved(loc);
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
typename T::topo_type operator*(const Plane& plane, const T& shape) {
    return shape.moved(plane.location());
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
ShapeList<typename T::topo_type> operator*(const std::vector<Location>& locs, const T& shape) {
    ShapeList<typename T::topo_type> out;
    out.reserve(locs.size());
    for (const auto& loc : locs) out.push_back(shape.moved(loc));
    return out;
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
ShapeList<typename T::topo_type> operator*(const std::vector<Plane>& planes, const T& shape) {
    ShapeList<typename T::topo_type> out;
    out.reserve(planes.size());
    for (const auto& p : planes) out.push_back(shape.moved(p.location()));
    return out;
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
ShapeList<typename T::topo_type> operator*(const Location& loc, const std::vector<T>& shapes) {
    ShapeList<typename T::topo_type> out;
    out.reserve(shapes.size());
    for (const auto& s : shapes) out.push_back(s.moved(loc));
    return out;
}
template <class T, std::enable_if_t<detail::is_shape_v<T>, int> = 0>
ShapeList<typename T::topo_type> operator*(const Plane& plane, const std::vector<T>& shapes) {
    return plane.location() * shapes;
}

}  // namespace build123d

// ---------------------------------------------------------------------------
// std::hash and tuple-like support (build123d's __hash__ and tuple results)
// ---------------------------------------------------------------------------

template <>
struct std::tuple_size<build123d::SplitResult> : std::integral_constant<std::size_t, 2> {};
template <std::size_t I>
struct std::tuple_element<I, build123d::SplitResult> {
    using type = const build123d::ShapeList<build123d::Shape>;
};

/// Shapes hash like build123d (TShape and location); equality is is_same().
#define BUILD123D_SHAPE_HASH(T)                                                  \
    template <>                                                                  \
    struct std::hash<build123d::T> {                                             \
        std::size_t operator()(const build123d::T& s) const { return s.hash(); } \
    };
BUILD123D_SHAPE_HASH(Shape)
BUILD123D_SHAPE_HASH(Vertex)
BUILD123D_SHAPE_HASH(Mixin1D)
BUILD123D_SHAPE_HASH(Edge)
BUILD123D_SHAPE_HASH(Wire)
BUILD123D_SHAPE_HASH(Mixin2D)
BUILD123D_SHAPE_HASH(Face)
BUILD123D_SHAPE_HASH(Shell)
BUILD123D_SHAPE_HASH(Mixin3D)
BUILD123D_SHAPE_HASH(Solid)
BUILD123D_SHAPE_HASH(Compound)
BUILD123D_SHAPE_HASH(Curve)
BUILD123D_SHAPE_HASH(Sketch)
BUILD123D_SHAPE_HASH(Part)
#undef BUILD123D_SHAPE_HASH
