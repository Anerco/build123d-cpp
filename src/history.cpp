// ShapeHistory: a port of build123d/topology/history.py.
#include "build123d/history.hpp"

#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <NCollection_DataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <NCollection_Map.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include "internal.hpp"

namespace build123d {

namespace {

using ShapeSet = NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>;
using ShapeIndex = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using ShapeToShape = NCollection_DataMap<TopoDS_Shape, TopoDS_Shape, TopTools_ShapeMapHasher>;

const TopAbs_ShapeEnum kTrackedTypes[] = {TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID};

std::vector<TopoDS_Shape> list_shapes(const NCollection_List<TopoDS_Shape>& shapes) {
    std::vector<TopoDS_Shape> out;
    for (const auto& s : shapes) out.push_back(s);
    return out;
}

/// What an algorithm says about one sub-shape, or nothing if it will not say.
template <class Query>
std::vector<TopoDS_Shape> answers(Query query, const TopoDS_Shape& shape) {
    try {
        return list_shapes(query(shape));
    } catch (const Standard_Failure&) {
        return {};
    } catch (const std::exception&) {
        return {};
    }
}

ShapeSet present_in(const TopoDS_Shape& result) {
    ShapeSet present;
    for (const auto& sub : tracked_subshapes({result})) present.Add(sub);
    return present;
}

}  // namespace

std::vector<TopoDS_Shape> tracked_subshapes(const std::vector<TopoDS_Shape>& shapes) {
    ShapeIndex found;
    for (const auto& shape : shapes) {
        if (shape.IsNull()) continue;
        for (TopAbs_ShapeEnum type : kTrackedTypes) TopExp::MapShapes(shape, type, found);
    }
    std::vector<TopoDS_Shape> out;
    out.reserve(static_cast<std::size_t>(found.Extent()));
    for (int i = 1; i <= found.Extent(); ++i) out.push_back(found(i));
    return out;
}

// ---------------------------------------------------------------------------
// Trace
// ---------------------------------------------------------------------------

struct Trace::Maps {
    ShapeSet untouched;
    ShapeSet modified_from;
    ShapeSet generated_from;
};

Trace::Trace(const ShapeHistory& history, const std::vector<TopoDS_Shape>& inputs) {
    auto maps = std::make_shared<Maps>();
    for (const auto& sub : tracked_subshapes(inputs)) {
        maps->untouched.Add(sub);
        for (const auto& shape : history.modified(sub)) maps->modified_from.Add(shape);
        for (const auto& shape : history.generated(sub)) maps->generated_from.Add(shape);
    }
    maps_ = std::move(maps);
}

bool Trace::is_untouched(const TopoDS_Shape& shape) const { return maps_->untouched.Contains(shape); }
bool Trace::is_modified(const TopoDS_Shape& shape) const { return maps_->modified_from.Contains(shape); }
bool Trace::is_generated(const TopoDS_Shape& shape) const { return maps_->generated_from.Contains(shape); }
bool Trace::is_descendant(const TopoDS_Shape& shape) const {
    return is_untouched(shape) || is_modified(shape) || is_generated(shape);
}

// ---------------------------------------------------------------------------
// ShapeHistory
// ---------------------------------------------------------------------------

ShapeHistory::ShapeHistory(Handle(BRepTools_History) history, std::vector<TopoDS_Shape> before_in,
                           std::vector<TopoDS_Shape> brought_in)
    : before(std::move(before_in)),
      brought(std::move(brought_in)),
      wrapped_(history.IsNull() ? Handle(BRepTools_History)(new BRepTools_History()) : history) {}

std::shared_ptr<ShapeHistory> ShapeHistory::from_boolean(BRepAlgoAPI_BuilderAlgo& operation,
                                                         std::vector<TopoDS_Shape> before_in,
                                                         std::vector<TopoDS_Shape> brought_in) {
    return std::make_shared<ShapeHistory>(operation.History(), std::move(before_in), std::move(brought_in));
}

std::shared_ptr<ShapeHistory> ShapeHistory::from_unify(ShapeUpgrade_UnifySameDomain& upgrader) {
    return std::make_shared<ShapeHistory>(upgrader.History());
}

std::shared_ptr<ShapeHistory> ShapeHistory::from_algorithm(BRepBuilderAPI_MakeShape& algorithm,
                                                           const std::vector<TopoDS_Shape>& inputs,
                                                           const TopoDS_Shape& result) {
    auto history = std::make_shared<ShapeHistory>(nullptr, inputs);
    const ShapeSet present = present_in(result);
    for (const auto& sub : tracked_subshapes(inputs)) {
        const auto modified = answers([&](const TopoDS_Shape& s) -> const NCollection_List<TopoDS_Shape>& {
            return algorithm.Modified(s);
        }, sub);
        for (const auto& shape : modified) history->wrapped_->AddModified(sub, shape);
        for (const auto& shape : answers([&](const TopoDS_Shape& s) -> const NCollection_List<TopoDS_Shape>& {
                 return algorithm.Generated(s);
             }, sub)) {
            if (BRepTools_History::IsSupportedType(shape)) history->wrapped_->AddGenerated(sub, shape);
        }
        if (modified.empty() && !present.Contains(sub)) history->wrapped_->Remove(sub);
    }
    return history;
}

std::shared_ptr<ShapeHistory> ShapeHistory::from_sewing(BRepBuilderAPI_Sewing& sewing,
                                                        const std::vector<TopoDS_Shape>& inputs) {
    auto history = std::make_shared<ShapeHistory>(nullptr, inputs);
    const ShapeSet present = present_in(sewing.SewedShape());
    for (const auto& sub : tracked_subshapes(inputs)) {
        if (sub.ShapeType() == TopAbs_FACE) {
            if (sewing.IsModified(sub)) {
                history->wrapped_->AddModified(sub, sewing.Modified(sub));
            } else if (!present.Contains(sub)) {
                history->wrapped_->Remove(sub);
            }
        } else if (sewing.IsModifiedSubShape(sub)) {
            history->wrapped_->AddModified(sub, sewing.ModifiedSubShape(sub));
        } else if (!present.Contains(sub)) {
            history->wrapped_->Remove(sub);
        }
    }
    return history;
}

std::shared_ptr<ShapeHistory> ShapeHistory::from_relocation(const TopoDS_Shape& before_shape,
                                                            const TopoDS_Shape& after_shape) {
    auto history = std::make_shared<ShapeHistory>(nullptr, std::vector<TopoDS_Shape>{before_shape});
    const TopLoc_Location identity;
    ShapeToShape placed;
    for (const auto& sub : tracked_subshapes({after_shape})) placed.Bind(sub.Located(identity), sub);
    for (const auto& sub : tracked_subshapes({before_shape})) {
        const TopoDS_Shape key = sub.Located(identity);
        if (placed.IsBound(key)) {
            const TopoDS_Shape& moved = placed.Find(key);
            if (!moved.IsSame(sub)) history->wrapped_->AddModified(sub, moved);
        }
    }
    return history;
}

std::shared_ptr<ShapeHistory> ShapeHistory::of(const std::vector<const Shape*>& shapes) {
    std::vector<std::shared_ptr<ShapeHistory>> found;
    for (const Shape* shape : shapes) {
        if (shape != nullptr && shape->history()) found.push_back(shape->history());
    }
    if (found.empty()) return nullptr;
    if (found.size() == 1) return found[0];
    std::vector<TopoDS_Shape> all_before, all_brought;
    for (const auto& h : found) {
        all_before.insert(all_before.end(), h->before.begin(), h->before.end());
        all_brought.insert(all_brought.end(), h->brought.begin(), h->brought.end());
    }
    auto merged = std::make_shared<ShapeHistory>(nullptr, std::move(all_before), std::move(all_brought));
    for (const auto& h : found) merged->merge(h);
    return merged;
}

std::shared_ptr<ShapeHistory> ShapeHistory::of(const Shape& shape) { return of(std::vector<const Shape*>{&shape}); }

ShapeHistory& ShapeHistory::add_modified(const TopoDS_Shape& before_shape, const TopoDS_Shape& after_shape) {
    if (BRepTools_History::IsSupportedType(before_shape)) wrapped_->AddModified(before_shape, after_shape);
    return *this;
}

ShapeHistory& ShapeHistory::merge(const ShapeHistory* later) {
    if (later != nullptr) {
        wrapped_->Merge(later->wrapped_);
        traces_.reset();
    }
    return *this;
}

ShapeHistory& ShapeHistory::with_inputs(std::vector<TopoDS_Shape> before_in, std::vector<TopoDS_Shape> brought_in) {
    before = std::move(before_in);
    brought = std::move(brought_in);
    traces_.reset();
    return *this;
}

const std::pair<Trace, Trace>& ShapeHistory::from() const {
    if (!traces_) traces_ = std::make_shared<std::pair<Trace, Trace>>(Trace(*this, before), Trace(*this, brought));
    return *traces_;
}

bool ShapeHistory::is_last(const TopoDS_Shape& shape) const {
    const auto& [from_before, from_brought] = from();
    if (from_brought.is_untouched(shape) || from_brought.is_modified(shape)) return true;
    return !(from_before.is_untouched(shape) || from_before.is_modified(shape));
}

bool ShapeHistory::is_new(const TopoDS_Shape& shape) const {
    const auto& [from_before, from_brought] = from();
    for (const Trace* trace : {&from_brought, &from_before}) {
        if (trace->is_untouched(shape) || trace->is_modified(shape)) return false;
    }
    return true;
}

std::vector<TopoDS_Shape> ShapeHistory::modified(const TopoDS_Shape& shape) const {
    if (!BRepTools_History::IsSupportedType(shape)) return {};
    return list_shapes(wrapped_->Modified(shape));
}

std::vector<TopoDS_Shape> ShapeHistory::generated(const TopoDS_Shape& shape) const {
    if (!BRepTools_History::IsSupportedType(shape)) return {};
    return list_shapes(wrapped_->Generated(shape));
}

bool ShapeHistory::is_removed(const TopoDS_Shape& shape) const {
    return BRepTools_History::IsSupportedType(shape) && wrapped_->IsRemoved(shape);
}

Trace ShapeHistory::trace(const std::vector<TopoDS_Shape>& inputs) const { return Trace(*this, inputs); }

std::shared_ptr<ShapeHistory> ShapeHistory::clone() const {
    return std::make_shared<ShapeHistory>(Handle(BRepTools_History)(new BRepTools_History(*wrapped_)), before,
                                          brought);
}

}  // namespace build123d
