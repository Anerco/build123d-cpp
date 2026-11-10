// The parts of build123d's builder mode (BuildPart, BuildSketch, BuildLine) that bd_warehouse's
// code relies on, for translating it to the algebra exactly: Builder._add_to_context (how an
// object combines with the builder's object in each mode, and the pending faces and edges), the
// publication of an object or a finished builder at the active locations and workplanes, and
// the builder-mode paths of the operations bd_warehouse calls. Each ported block keeps Python's
// order of operations; this file holds what builder mode does implicitly.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <build123d/build123d.hpp>

namespace bd_warehouse::detail {

using namespace build123d;

/// build123d's Mode (the builder combination modes).
enum class Mode { ADD, SUBTRACT, INTERSECT, REPLACE, PRIVATE };

/// A builder: BuildPart (dim 3), BuildSketch (dim 2) or BuildLine (dim 1) with its object in
/// local coordinates (Python's `_obj`) and its pending lists.
class Builder {
public:
    explicit Builder(int dim) : dim(dim) {}

    /// 3: BuildPart, 2: BuildSketch, 1: BuildLine.
    int dim;
    /// The builder's object (`part_local`, `sketch_local`, `line_local`); nullopt for None.
    std::optional<Shape> obj;
    std::vector<Edge> pending_edges;
    std::vector<Face> pending_faces;
    std::vector<Plane> pending_face_planes;

    /// Builder._add_to_context(*objects, faces_to_pending, clean, mode).
    void add(const std::vector<Shape>& objects, Mode mode = Mode::ADD, bool clean = true,
             bool faces_to_pending = true);
    void add(const Shape& object, Mode mode = Mode::ADD, bool clean = true) { add(std::vector<Shape>{object}, mode, clean); }

    /// The object as the builder's type (Part, Sketch or Curve); an empty one for None.
    Part part() const;
    Sketch sketch() const;
    Curve line() const;
    /// The object as a Compound of the builder's type (for publishing).
    Shape product() const;

    /// builder.vertices() / edges() / faces() / solids() / wires() (of the local object).
    ShapeList<Vertex> vertices() const;
    ShapeList<Edge> edges() const;
    ShapeList<Face> faces() const;
    ShapeList<Solid> solids() const;
    ShapeList<Wire> wires() const;
    /// builder.face() / edge(): the only one, else ValueError.
    Face face() const;
    Edge edge() const;
    /// builder.wire() of a BuildLine: Wire(line.edges()) (the line's edges in one wire).
    Wire wire() const;

    /// A finished child builder (or its object) published into this builder: placed at the
    /// publication locations x output placements (see place) and added with `mode`.
    void publish(const Shape& product, Mode mode = Mode::ADD, const std::vector<Location>& publication = {Location()},
                 const std::vector<Location>& output = {Location()});
};

/// _PublicationService.place: the product at every publication location x output placement
/// (`publication * output * product`); several make one Compound of the product's type. With
/// only identity locations, the product itself.
Shape place(const Shape& product, const std::vector<Location>& publication,
            const std::vector<Location>& output = {Location()});

/// The locations of nested Locations contexts: each of the outer locations times each of the
/// inner ones (outer first).
std::vector<Location> compose(const std::vector<Location>& outer, const std::vector<Location>& inner);

// ---- builder-mode paths of the operations

/// make_face() in a BuildSketch: a face of the pending edges, added with `mode`.
void make_face(Builder& sketch, Mode mode = Mode::ADD);
/// make_face(edges) in a BuildSketch.
void make_face(Builder& sketch, const std::vector<Edge>& edges, Mode mode = Mode::ADD);
/// loft() in a BuildPart: the pending faces lofted, added with `mode`.
void loft(Builder& part, bool ruled = false, bool clean = true, Mode mode = Mode::ADD);
/// extrude(amount=...) in a BuildPart: the pending faces extruded along their planes.
void extrude(Builder& part, double amount, const ExtrudeOptions& options = {}, Mode mode = Mode::ADD);
/// extrude(to_extrude, amount, ...) of explicit faces in a BuildPart.
void extrude(Builder& part, const Shape& to_extrude, double amount, const ExtrudeOptions& options = {},
             Mode mode = Mode::ADD);
/// revolve(axis=..., revolution_arc=...) in a BuildPart: the pending faces revolved.
void revolve(Builder& part, const Axis& axis = Axis::Z, double revolution_arc = 360.0, bool clean = true,
             Mode mode = Mode::ADD);
/// fillet(objects, radius) in a builder (the result replaces the builder's object).
void fillet(Builder& builder, const std::vector<Shape>& objects, double radius);
/// chamfer(objects, length, length2, angle, reference) in a builder.
void chamfer(Builder& builder, const std::vector<Shape>& objects, double length,
             std::optional<double> length2 = std::nullopt, std::optional<double> angle = std::nullopt,
             const std::optional<Shape>& reference = std::nullopt);
/// mirror(objects (None: the builder's object), about, mode) in a builder.
void mirror(Builder& builder, const std::optional<std::vector<Shape>>& objects, const Plane& about = Plane::XZ,
            Mode mode = Mode::ADD);
/// split(objects (None: the builder's object), bisect_by, keep, mode=REPLACE) in a builder.
void split(Builder& builder, const std::optional<std::vector<Shape>>& objects, const Plane& bisect_by = Plane::XZ,
           Keep keep = Keep::TOP, Mode mode = Mode::REPLACE);
/// offset(objects (None: the builder's object), amount, ...) in a builder.
void offset(Builder& builder, const std::optional<std::vector<Shape>>& objects, double amount,
            const OffsetOptions& options = {}, Mode mode = Mode::REPLACE);
/// scale(objects (None: the builder's object), by, mode=REPLACE) in a builder.
void scale(Builder& builder, const std::optional<std::vector<Shape>>& objects, double by, Mode mode = Mode::REPLACE);
/// insert(objects) in a builder at the given locations (the builder's location context).
void insert(Builder& builder, const std::vector<Shape>& objects, const std::vector<Location>& locations = {Location()},
            Mode mode = Mode::ADD);

}  // namespace bd_warehouse::detail
