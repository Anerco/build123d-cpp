// build123d-cpp: ShapeHistory, the record of what an operation did to the sub-shapes of
// its inputs. Mirrors build123d/topology/history.py; it is what Select::LAST and
// Select::NEW read (part.edges(Select::NEW)).
#pragma once

#include <memory>
#include <vector>

#include <BRepTools_History.hxx>
#include <Standard_Handle.hxx>
#include <TopoDS_Shape.hxx>

class BRepAlgoAPI_BuilderAlgo;
class BRepBuilderAPI_MakeShape;
class BRepBuilderAPI_Sewing;
class ShapeUpgrade_UnifySameDomain;

namespace build123d {

class Shape;
class ShapeHistory;

/// Every distinct vertex, edge, face and solid of the given shapes (the sub-shape kinds
/// BRepTools_History records), in map order.
std::vector<TopoDS_Shape> tracked_subshapes(const std::vector<TopoDS_Shape>& shapes);

/// A ShapeHistory read from the result's side, for one set of inputs.
class Trace {
public:
    Trace(const ShapeHistory& history, const std::vector<TopoDS_Shape>& inputs);

    /// Was this result sub-shape an input sub-shape, unchanged?
    bool is_untouched(const TopoDS_Shape& shape) const;
    /// Was this result sub-shape rebuilt from an input sub-shape?
    bool is_modified(const TopoDS_Shape& shape) const;
    /// Was this result sub-shape made from an input sub-shape of another kind?
    bool is_generated(const TopoDS_Shape& shape) const;
    /// Did this result sub-shape come from the inputs in any way?
    bool is_descendant(const TopoDS_Shape& shape) const;

private:
    struct Maps;
    std::shared_ptr<const Maps> maps_;
};

/// The record of what an operation did to its inputs' sub-shapes.
///
/// Each tracked sub-shape of an input ends up untouched, modified, generating or removed.
/// Records chain with merge(). `before` holds the inputs that were already there and
/// `brought` those the operation brought in (for a + b: a and b); is_last() and is_new()
/// classify a sub-shape of the result with them.
class ShapeHistory {
public:
    explicit ShapeHistory(Handle(BRepTools_History) history = nullptr,
                          std::vector<TopoDS_Shape> before = {},
                          std::vector<TopoDS_Shape> brought = {});

    std::vector<TopoDS_Shape> before;
    std::vector<TopoDS_Shape> brought;

    // ---- constructors (as build123d's class methods)
    /// The record a boolean operation (or splitter) kept while building.
    static std::shared_ptr<ShapeHistory> from_boolean(BRepAlgoAPI_BuilderAlgo& operation,
                                                      std::vector<TopoDS_Shape> before = {},
                                                      std::vector<TopoDS_Shape> brought = {});
    /// The record of a clean (ShapeUpgrade_UnifySameDomain).
    static std::shared_ptr<ShapeHistory> from_unify(ShapeUpgrade_UnifySameDomain& upgrader);
    /// Collect the per-sub-shape answers (Modified/Generated) of a BRepBuilderAPI_MakeShape.
    static std::shared_ptr<ShapeHistory> from_algorithm(BRepBuilderAPI_MakeShape& algorithm,
                                                        const std::vector<TopoDS_Shape>& inputs,
                                                        const TopoDS_Shape& result);
    /// The record of sewing faces into a shell.
    static std::shared_ptr<ShapeHistory> from_sewing(BRepBuilderAPI_Sewing& sewing,
                                                     const std::vector<TopoDS_Shape>& inputs);
    /// The record of moving a shape: each sub-shape modified into its moved self.
    static std::shared_ptr<ShapeHistory> from_relocation(const TopoDS_Shape& before,
                                                         const TopoDS_Shape& after);
    /// The records the shapes carry, merged into one; nullptr if none carries one.
    static std::shared_ptr<ShapeHistory> of(const std::vector<const Shape*>& shapes);
    static std::shared_ptr<ShapeHistory> of(const Shape& shape);

    // ---- instance methods
    /// Record that `before` was rebuilt into `after`.
    ShapeHistory& add_modified(const TopoDS_Shape& before, const TopoDS_Shape& after);
    /// Extend this record with the operation that came after it.
    ShapeHistory& merge(const ShapeHistory* later);
    ShapeHistory& merge(const std::shared_ptr<ShapeHistory>& later) { return merge(later.get()); }
    /// Say which inputs were already there and which were brought in.
    ShapeHistory& with_inputs(std::vector<TopoDS_Shape> before, std::vector<TopoDS_Shape> brought);
    /// Did the operation bring this sub-shape of the result in, or create it?
    bool is_last(const TopoDS_Shape& shape) const;
    /// Did this sub-shape of the result exist in no input, in any form?
    bool is_new(const TopoDS_Shape& shape) const;
    /// What an input sub-shape was rebuilt into, if anything.
    std::vector<TopoDS_Shape> modified(const TopoDS_Shape& shape) const;
    /// What was made from an input sub-shape, if anything.
    std::vector<TopoDS_Shape> generated(const TopoDS_Shape& shape) const;
    /// Was an input sub-shape removed by the operation?
    bool is_removed(const TopoDS_Shape& shape) const;
    /// Index the record from the result's side for the given inputs.
    Trace trace(const std::vector<TopoDS_Shape>& inputs) const;

    /// An independent copy (the OCCT record is duplicated).
    std::shared_ptr<ShapeHistory> clone() const;

    const Handle(BRepTools_History)& wrapped() const { return wrapped_; }

private:
    Handle(BRepTools_History) wrapped_;
    mutable std::shared_ptr<std::pair<Trace, Trace>> traces_;
    const std::pair<Trace, Trace>& from() const;
};

}  // namespace build123d
