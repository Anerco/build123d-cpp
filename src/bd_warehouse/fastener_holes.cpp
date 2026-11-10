// Port of bd_warehouse's fastener.py: the holes (_make_fastener_hole, ClearanceHole, TapHole,
// ThreadedHole, InsertHole).
#include <cmath>

#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;

namespace {

/// _make_fastener_hole (in algebra mode, where bd_warehouse doesn't record hole locations).
Shape make_fastener_hole(const ParamDict& hole_diameters, const Fastener& fastener,
                         const std::optional<Shape>& countersink_profile, double depth,
                         const std::optional<std::string>& fit, const std::optional<std::string>& material,
                         bool counter_sunk = true, bool captive_nut = false, bool threaded_hole = false) {
    const Vector bore_direction(0, 0, -1);
    const Vector origin(0, 0, 0);
    std::optional<Shape> countersink_cutter;
    double head_offset = 0.0;

    if (captive_nut) {
        if (!fit) throw std::invalid_argument("fit is required for a captive nut hole");
        const double clearance = fastener.clearance_hole_diameters().at(*fit).number() - fastener.thread_diameter;
        head_offset = countersink_profile->vertices().sort_by(Axis::Z)[-1].Z();
        double fillet_radius, rect_width, rect_height;
        if (dynamic_cast<const DomedCapNut*>(&fastener) || dynamic_cast<const HexNut*>(&fastener) ||
            dynamic_cast<const UnchamferedHexagonNut*>(&fastener)) {
            const double nut_diameter = dynamic_cast<const Nut&>(fastener).nut_diameter();
            fillet_radius = nut_diameter / 4;
            rect_width = nut_diameter + clearance;
            rect_height = nut_diameter * std::sin(M_PI / 3) + clearance;
        } else if (dynamic_cast<const SquareNut*>(&fastener)) {
            const double nut_diameter = dynamic_cast<const Nut&>(fastener).nut_diameter();
            fillet_radius = nut_diameter / 8;
            rect_height = nut_diameter * std::sqrt(2.0) / 2 + clearance;
            rect_width = rect_height + 2 * fillet_radius + clearance;
        } else {
            throw std::invalid_argument("captive nuts must be hex, domed cap or square nuts");
        }
        Builder countersink_cutter_builder(3);
        {
            Builder sketch(2);
            sketch.add(RectangleRounded(rect_width, rect_height, fillet_radius));
            countersink_cutter_builder.publish(sketch.product());
        }
        detail::extrude(countersink_cutter_builder, -head_offset);
        countersink_cutter = countersink_cutter_builder.part();
    } else if (counter_sunk && countersink_profile) {
        head_offset = countersink_profile->vertices().sort_by(Axis::Z)[-1].Z();
        countersink_cutter = build123d::revolve(*countersink_profile).moved(Pos(0, 0, -head_offset));
    } else {
        head_offset = 0;
    }

    double hole_radius;
    if (threaded_hole) {
        hole_radius = fastener.thread_diameter / 2;
    } else {
        const std::optional<std::string> key = material ? material : fit;
        const auto found = key ? hole_diameters.get(*key) : std::nullopt;
        if (!found) {
            throw std::invalid_argument((key ? *key : std::string("None")) + " invalid, must be one of " +
                                        detail::fits_repr(hole_diameters));
        }
        hole_radius = found->number() / 2;
    }

    Shape shank_hole;
    if (const auto* screw = dynamic_cast<const Screw*>(&fastener)) {
        shank_hole = screw->make_body_hole(hole_radius, depth, head_offset);
    } else {
        shank_hole = Solid::make_cylinder(hole_radius, depth, Plane(origin, std::nullopt, bore_direction));
    }
    Shape fastener_hole = counter_sunk && countersink_cutter ? countersink_cutter->fuse(shank_hole) : shank_hole;

    const double csk_angle = 180 - 82;  // 82 is a common tip angle
    const double h = hole_radius / std::tan(detail::radians(csk_angle / 2.0));
    const Solid drill_tip =
        Solid::make_cone(hole_radius, 0.0, h, Plane(bore_direction * depth, std::nullopt, bore_direction));
    fastener_hole = fastener_hole.fuse(drill_tip);
    return fastener_hole;
}

double required_depth(std::optional<double> depth) {
    // bd_warehouse takes twice the size of the builder's part in builder mode
    if (!depth) throw std::invalid_argument("No depth provided");
    return *depth;
}

}  // namespace

ClearanceHole::ClearanceHole(const Fastener& fastener, const std::string& fit, std::optional<double> depth,
                             bool counter_sunk, bool captive_nut, const Rotation& rotation) {
    if (dynamic_cast<const HeatSetNut*>(&fastener)) {
        throw std::invalid_argument(
            "ClearanceHole doesn't accept fasteners of type HeatSetNut - use insertHole instead");
    }
    if (captive_nut && !(dynamic_cast<const DomedCapNut*>(&fastener) || dynamic_cast<const HexNut*>(&fastener) ||
                         dynamic_cast<const UnchamferedHexagonNut*>(&fastener) ||
                         dynamic_cast<const SquareNut*>(&fastener))) {
        throw std::invalid_argument("Only DomedCapNut, HexNut, UnchamferedHexagonNut or SquareNut can be captive");
    }
    hole_depth = required_depth(depth);
    const ParamDict diameters = fastener.clearance_hole_diameters();
    const auto profile = fastener.countersink_profile(fit);
    const Shape hole_part = make_fastener_hole(diameters, fastener, profile ? std::optional<Shape>(*profile) : std::nullopt,
                                               hole_depth, fit, std::nullopt, counter_sunk, captive_nut);
    assign(hole_part, rotation, std::nullopt);
}

TapHole::TapHole(const Fastener& fastener, const std::string& material, const std::string& fit,
                 std::optional<double> depth, bool counter_sunk) {
    if (dynamic_cast<const HeatSetNut*>(&fastener)) {
        throw std::invalid_argument("TapHole doesn't accept fasteners of type HeatSetNut - use insertHole instead");
    }
    hole_depth = required_depth(depth);
    const ParamDict diameters = fastener.tap_hole_diameters();
    const auto profile = fastener.countersink_profile(fit);
    const Shape hole_part = make_fastener_hole(diameters, fastener, profile ? std::optional<Shape>(*profile) : std::nullopt,
                                               hole_depth, fit, material, counter_sunk);
    assign(hole_part, Rotation(), std::nullopt);
}

ThreadedHole::ThreadedHole(const Fastener& fastener, const std::string& material, const std::string& fit,
                           std::optional<double> depth, bool counter_sunk, bool simple) {
    if (dynamic_cast<const HeatSetNut*>(&fastener)) {
        throw std::invalid_argument(
            "ThreadedHole doesn't accept fasteners of type HeatSetNut - use InsertHole instead");
    }
    hole_depth = required_depth(depth);
    if (hole_depth <= 0) throw std::invalid_argument("Hole depth must be greater than zero");

    const ParamDict tap_diameters = fastener.tap_hole_diameters();
    const auto core = tap_diameters.get(material);
    if (!core) throw std::invalid_argument(material + " invalid, must be one of " + detail::fits_repr(tap_diameters));
    const double core_diameter = core->number();

    // Construct the cutter at the origin
    const std::optional<Face> head_profile = counter_sunk ? fastener.countersink_profile(fit) : std::nullopt;
    const double head_height = head_profile ? head_profile->bounding_box().max.Z : 0.0;
    const double head_depth = counter_sunk ? head_height : 0.0;
    const double thread_length = hole_depth - head_depth;
    if (thread_length <= 0) {
        throw std::invalid_argument("Hole depth " + detail::py_str(hole_depth) + " is too shallow for a countersink depth of " +
                                    detail::py_str(head_depth));
    }
    const auto* screw = dynamic_cast<const Screw*>(&fastener);
    if (!screw) throw std::invalid_argument("'" + fastener.class_name() + "' object has no attribute 'make_body_hole'");
    const Solid body_hole = screw->make_body_hole(core_diameter / 2, hole_depth, head_depth);
    std::vector<Shape> cutter_solids;
    for (const auto& s : body_hole.solids()) cutter_solids.push_back(s);

    if (counter_sunk && head_profile) {
        const Solid head_cutter = Solid::revolve(head_profile->moved(Pos(0, 0, -head_depth)), 360, Axis::Z);
        for (const auto& s : head_cutter.solids()) cutter_solids.push_back(s);
    }

    if (!simple) {
        IsoThread cut_thread(fastener.thread_diameter, screw->thread_pitch, thread_length,
                             IsoThreadOptions{.external = true,
                                              .hand = screw->hand,
                                              .end_finishes = {"square", "square"},
                                              .interference = 0.2,
                                              .simple = false});
        cut_thread.move(Pos(0, 0, -hole_depth));
        for (const auto& s : cut_thread.solids()) cutter_solids.push_back(s);
        thread = cut_thread;
    }

    Compound hole_part;
    hole_part.label = "threaded hole cutter";
    hole_part.children = cutter_solids;
    assign(hole_part, Rotation(), std::nullopt);
}

InsertHole::InsertHole(const HeatSetNut& fastener, const std::string& fit, std::optional<double> depth,
                       double manufacturing_compensation) {
    hole_depth = required_depth(depth);
    const ParamDict diameters = fastener.clearance_hole_diameters();
    const Sketch profile = fastener.countersink_profile(manufacturing_compensation);
    const Shape hole_part = make_fastener_hole(diameters, fastener, profile, hole_depth, fit, std::nullopt);
    assign(hole_part, Rotation(), std::nullopt);
}

}  // namespace bd_warehouse
