// Booleans that keep the record of what they did (build123d's Shape._bool_op and
// _bool_op_list), and the attribute copying build123d does around them.
#include <cstring>
#include <memory>

#include <BOPAlgo_GlueEnum.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopoDS_Iterator.hxx>

#include "build123d/history.hpp"
#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d::detail {

void copy_attributes_to(const Shape& from, Shape& to, const std::vector<std::string>& exceptions) {
    const auto excluded = [&](const char* name) {
        for (const auto& e : exceptions) {
            // the C++ names, or build123d's attribute names ("_color", "_material")
            if (e == name || e == std::string("_") + name) return true;
        }
        return false;
    };
    // As build123d: an attribute is copied only when the target has none of its own.
    if (!excluded("label") && to.label.empty()) to.label = from.label;
    if (!excluded("color") && !to.color.raw()) to.color.set_raw(from.color.raw());
    if (!excluded("material") && !to.material.raw()) to.material.set_raw(from.material.raw());
    if (!excluded("topo_path") && !ShapeAccess::topo_path(to)) {
        ShapeAccess::set_topo_path(to, ShapeAccess::topo_path(from));
    }
    // build123d deep-copies `joints` when the target has none; copies of a JointMap are bound
    // to the shape that holds them. The assembly tree (parent, children) is never copied.
    if (!excluded("joints") && to.joints.empty()) to.joints = from.joints;
}

BoolResult bool_op_history(BoolOp op, const std::vector<TopoDS_Shape>& args_in,
                           const std::vector<TopoDS_Shape>& tools_in, double fuzzy, bool glue) {
    std::vector<TopoDS_Shape> args, tools;
    for (const auto& s : args_in) {
        if (!s.IsNull()) args.push_back(s);
    }
    for (const auto& s : tools_in) {
        if (!s.IsNull()) tools.push_back(s);
    }

    // Operations with "zero" shapes
    TopoDS_Shape topo_result;
    switch (op) {
        case BoolOp::CUT:
            if (tools.empty()) topo_result = args.size() == 1 ? args[0] : TopoDS_Shape(make_compound(args));
            break;
        case BoolOp::FUSE:
            if (tools.empty()) {
                topo_result = args.size() == 1 ? args[0] : TopoDS_Shape(make_compound(args));
            } else if (args.empty()) {
                topo_result = tools.size() == 1 ? tools[0] : TopoDS_Shape(make_compound(tools));
            }
            break;
        case BoolOp::COMMON:
            if (tools.empty() || args.empty()) return {TopoDS_Shape(), nullptr};
            break;
        case BoolOp::SECTION: break;
    }

    // The arguments were there before and the tools are brought in; an empty record
    // means every input sub-shape came through untouched (the shortcuts above).
    auto history = std::make_shared<ShapeHistory>(nullptr, args, tools);
    if (topo_result.IsNull()) {
        occt("Boolean operation", [&] {
            std::unique_ptr<BRepAlgoAPI_BooleanOperation> operation;
            switch (op) {
                case BoolOp::FUSE: operation = std::make_unique<BRepAlgoAPI_Fuse>(); break;
                case BoolOp::CUT: operation = std::make_unique<BRepAlgoAPI_Cut>(); break;
                case BoolOp::COMMON: operation = std::make_unique<BRepAlgoAPI_Common>(); break;
                case BoolOp::SECTION: operation = std::make_unique<BRepAlgoAPI_Section>(); break;
            }
            if (glue) operation->SetGlue(BOPAlgo_GlueShift);
            if (fuzzy != 0.0) operation->SetFuzzyValue(fuzzy);
            operation->SetArguments(to_list(args));
            operation->SetTools(to_list(tools));
            operation->SetRunParallel(true);
            operation->Build();
            topo_result = operation->Shape();
            history = ShapeHistory::from_boolean(*operation, args, tools);
            return 0;
        });
        if (topo_result.IsNull()) throw std::invalid_argument("Null TopoDS_Shape object");
    }

    if (clean_enabled()) {
        try {
            ShapeUpgrade_UnifySameDomain upgrader(topo_result, true, true, true);
            upgrader.AllowInternalEdges(false);
            upgrader.Build();
            const TopoDS_Shape cleaned = upgrader.Shape();
            history->merge(ShapeHistory::from_unify(upgrader));
            topo_result = cleaned;
        } catch (const Standard_Failure&) {
            warn("Boolean operation unable to clean");
        }
    }
    if (topo_result.ShapeType() == TopAbs_COMPOUND) topo_result = unwrap(topo_result, true);
    return {topo_result, history};
}

Shape bool_op_shape(BoolOp op, const std::vector<const Shape*>& args, const std::vector<const Shape*>& tools,
                    double fuzzy, bool glue) {
    std::vector<TopoDS_Shape> arg_shapes, tool_shapes;
    for (const Shape* s : args) arg_shapes.push_back(s->wrapped());
    for (const Shape* s : tools) tool_shapes.push_back(s->wrapped());
    const BoolResult r = bool_op_history(op, arg_shapes, tool_shapes, fuzzy, glue);
    if (r.shape.IsNull()) return Shape();
    Shape result(r.shape);
    if (!args.empty()) copy_attributes_to(*args[0], result);
    result.set_history(r.history);
    return result;
}

ShapeList<Shape> bool_op_list(BoolOp op, const Shape& arg, const Shape& tool) {
    const Shape result = bool_op_shape(op, {&arg}, {&tool});
    ShapeList<Shape> pieces;
    if (result.is_null()) return pieces;
    if (result.wrapped().ShapeType() == TopAbs_COMPOUND) {
        // fresh pieces, as build123d's get_top_level_shapes, all made by the operation
        for (const auto& s : top_level(result.wrapped())) pieces.push_back(made_by(Shape(s), result.history()));
        return pieces;
    }
    pieces.push_back(result);
    return pieces;
}

}  // namespace build123d::detail
