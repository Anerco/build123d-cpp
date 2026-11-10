// Port of bd_warehouse's thread.py.
#include "bd_warehouse/thread.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "../py_compat.hpp"
#include "builder.hpp"
#include "data.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;

bool is_safe(const std::string& value) {
    if (value.size() > 10) return false;
    for (const char c : value) {
        if (std::string("0123456789./ ").find(c) == std::string::npos) return false;
    }
    return true;
}

Param imperial_str_to_float(const std::string& measure) {
    if (is_safe(measure)) {
        // Before eval() the string is verified as safe
        return detail::py_eval_number(detail::py_replace(detail::py_strip(measure), " ", "+")).value * IN;
    }
    return Param(measure);
}

namespace {

// ---------------------------------------------------------------------------
// Tables
// ---------------------------------------------------------------------------

struct AstmD2911Dimensions {
    double diameter_max;
    double diameter_min;
    int tpi;
};

struct AstmD2911Profile {
    double root_width;
    double thread_height;
};

struct AstmD2911Finish {
    double min_turns;
    std::vector<int> diameters;
    Dict<std::string, std::pair<double, double>> angles;
};

int as_int(const std::string& text) { return static_cast<int>(detail::py_int_str(text)); }

std::vector<detail::DictRow> read_plastic_bottle_thread_csv(const std::string& filename) {
    return detail::dict_reader(detail::data_file("data/" + filename)).rows;
}

const Dict<int, AstmD2911Dimensions>& astm_d2911_dimensions() {
    static const Dict<int, AstmD2911Dimensions> table = [] {
        Dict<int, AstmD2911Dimensions> t;
        for (const auto& row : read_plastic_bottle_thread_csv("plastic_bottle_thread_astm_d2911_dimensions.csv")) {
            t[as_int(row.str("diameter"))] = {detail::py_float(row.str("diameter_max")),
                                              detail::py_float(row.str("diameter_min")), as_int(row.str("tpi"))};
        }
        return t;
    }();
    return table;
}

const Dict<std::pair<std::string, int>, AstmD2911Profile>& astm_d2911_profiles() {
    static const Dict<std::pair<std::string, int>, AstmD2911Profile> table = [] {
        Dict<std::pair<std::string, int>, AstmD2911Profile> t;
        for (const auto& row : read_plastic_bottle_thread_csv("plastic_bottle_thread_astm_d2911_profiles.csv")) {
            t[{row.str("style"), as_int(row.str("tpi"))}] = {detail::py_float(row.str("root_width")),
                                                             detail::py_float(row.str("thread_height"))};
        }
        return t;
    }();
    return table;
}

const Dict<int, AstmD2911Finish>& astm_d2911_finishes() {
    static const Dict<int, AstmD2911Finish> table = [] {
        Dict<int, AstmD2911Finish> t;
        for (const auto& row : read_plastic_bottle_thread_csv("plastic_bottle_thread_astm_d2911_finishes.csv")) {
            AstmD2911Finish finish;
            finish.min_turns = detail::py_float(row.str("min_turns"));
            for (const auto& d : detail::py_split(row.str("diameters"))) finish.diameters.push_back(as_int(d));
            finish.angles["L"] = {detail::py_float(row.str("l_angle_1")), detail::py_float(row.str("l_angle_2"))};
            finish.angles["M"] = {detail::py_float(row.str("m_angle_1")), detail::py_float(row.str("m_angle_2"))};
            t[as_int(row.str("finish"))] = finish;
        }
        return t;
    }();
    return table;
}

const Dict<std::string, Dict<std::string, double>>& pco1881_data() {
    static const Dict<std::string, Dict<std::string, double>> table = [] {
        Dict<std::string, Dict<std::string, double>> t;
        for (const auto& row : read_plastic_bottle_thread_csv("plastic_bottle_thread_pco1881.csv")) {
            Dict<std::string, double> values;
            for (const auto& [key, value] : row.fields) {
                if (key == "size") continue;
                values[key] = key != "starts" ? detail::py_float(value.value()) : static_cast<double>(as_int(value.value()));
            }
            t[row.str("size")] = values;
        }
        return t;
    }();
    return table;
}

const Dict<std::string, Dict<std::string, double>>& iso_228_1_data() {
    static const Dict<std::string, Dict<std::string, double>> table = [] {
        Dict<std::string, Dict<std::string, double>> t;
        for (const auto& row : detail::dict_reader(detail::data_file("data/iso_228_1.csv")).rows) {
            Dict<std::string, double> values;
            values["tpi"] = as_int(row.str("tpi"));
            for (const auto& [key, value] : row.fields) {
                if (key != "size" && key != "tpi") values[key] = detail::py_float(value.value());
            }
            t[row.str("size")] = values;
        }
        return t;
    }();
    return table;
}

void check_end_finishes(const EndFinishes& end_finishes) {
    for (const auto* finish : {&end_finishes.first, &end_finishes.second}) {
        if (*finish != "raw" && *finish != "square" && *finish != "fade" && *finish != "chamfer") {
            throw std::invalid_argument(
                "end_finishes invalid, must be tuple() of \"raw, square, fade, or chamfer\"");
        }
    }
}

void check_hand(const std::string& hand) {
    if (hand != "right" && hand != "left") {
        throw std::invalid_argument("hand must be one of \"right\" or \"left\" not " + hand);
    }
}

// ---------------------------------------------------------------------------
// _ThreadSweep
// ---------------------------------------------------------------------------

/// Create a helical thread from a completed two-dimensional profile (bd_warehouse's private
/// _ThreadSweep): the loops and fade tips, as the children of `part` (labelled "thread").
class ThreadSweep {
public:
    ThreadSweep(const Face& thread_profile, double apex_radius, double root_radius, double pitch, double length,
                const std::string& hand, const EndFinishes& end_finishes, double manufacturing_compensation);

    bool external;
    double apex_radius;
    double root_radius;
    double manufacturing_compensation;
    double pitch;
    double length;
    bool right_hand;
    EndFinishes end_finishes;
    double tooth_height;
    Face thread_profile;
    double thread_loops = 0.0;
    Compound part;

private:
    using Child = std::shared_ptr<Shape>;  // Python's object references

    static std::optional<Shape> chamfer_loop(const Shape& loop, const std::optional<Solid>& chamfer_shape);
    std::vector<Child> finish_start(std::vector<Child> children, const std::string& finish,
                                    const std::optional<Solid>& chamfer_shape) const;
    std::vector<Child> finish_end(std::vector<Child> children, const std::string& finish,
                                  const std::optional<Solid>& chamfer_shape) const;
    Solid make_thread_loop(double loop_height) const;
    Solid make_fade_end(bool bottom) const;
    Solid make_chamfer_shape() const;
};

ThreadSweep::ThreadSweep(const Face& profile, double apex_radius_in, double root_radius_in, double pitch_in,
                         double length_in, const std::string& hand, const EndFinishes& finishes,
                         double manufacturing_compensation_in) {
    external = apex_radius_in > root_radius_in;
    const double radial_shift = manufacturing_compensation_in * (external ? -1 : 1);
    apex_radius = apex_radius_in + radial_shift;
    root_radius = root_radius_in + radial_shift;
    manufacturing_compensation = manufacturing_compensation_in;
    pitch = pitch_in;
    length = length_in;
    right_hand = hand == "right";
    end_finishes = finishes;
    tooth_height = std::abs(apex_radius - root_radius);
    thread_profile = profile;

    // Create base cylindrical thread
    const int number_faded_ends = (end_finishes.first == "fade" ? 1 : 0) + (end_finishes.second == "fade" ? 1 : 0);
    const double cylindrical_thread_length = length + pitch * (1 - 1 * number_faded_ends);
    thread_loops = cylindrical_thread_length / pitch;
    const double cylindrical_thread_displacement = end_finishes.first == "fade" ? pitch / 2 : -pitch / 2;

    std::vector<Child> loops;
    if (thread_loops >= 1.0) {
        Solid full_loop = make_thread_loop(1.0);
        full_loop.label = "loop";
        // copy.copy: each copy shares the geometry and has its own joints
        for (long i = 0; i < static_cast<long>(thread_loops); ++i) loops.push_back(std::make_shared<Shape>(full_loop));
    }
    const double partial = build123d::detail::py_mod(thread_loops, 1.0);
    if (partial > 0.0) {
        Solid last_loop = make_thread_loop(partial);
        last_loop.label = "partial";
        loops.push_back(std::make_shared<Shape>(last_loop));
    }
    if (loops.empty()) throw std::out_of_range("list index out of range");
    loops[0]->locate(Location(Vector(0, 0, cylindrical_thread_displacement)));
    // Connect each loop to position it
    for (std::size_t i = 1; i < loops.size(); ++i) {
        loops[i - 1]->joints.get<RigidJoint>("1").connect_to(loops[i]->joints.get<RigidJoint>("0"));
    }

    std::vector<Child> children = loops;

    // Apply the end finishes
    const bool chamfered = end_finishes.first == "chamfer" || end_finishes.second == "chamfer";
    const std::optional<Solid> chamfer_shape = chamfered ? std::optional<Solid>(make_chamfer_shape()) : std::nullopt;
    Child start_tip, end_tip;
    // Bottom
    if (end_finishes.first == "fade") {
        start_tip = std::make_shared<Shape>(make_fade_end(true));
        start_tip->label = "bottom_tip";
        children.push_back(start_tip);
    } else if (end_finishes.first == "square" || end_finishes.first == "chamfer") {
        children = finish_start(children, end_finishes.first, chamfer_shape);
    }
    // Top
    if (end_finishes.second == "fade") {
        end_tip = std::make_shared<Shape>(make_fade_end(false));
        end_tip->label = "top_tip";
        children.push_back(end_tip);
    } else if (end_finishes.second == "square" || end_finishes.second == "chamfer") {
        children = finish_end(children, end_finishes.second, chamfer_shape);
    }

    // Locate the final loop objects in axial order so fade tips can be positioned
    std::vector<Child> final_loops;
    for (const auto& child : children) {
        if (child->label == "loop" || child->label == "partial") final_loops.push_back(child);
    }
    std::vector<double> min_z;
    for (const auto& loop : final_loops) min_z.push_back(loop->bounding_box().min.Z);
    std::vector<std::size_t> order(final_loops.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return min_z[a] < min_z[b]; });
    std::vector<Child> sorted;
    for (const auto i : order) sorted.push_back(final_loops[i]);
    if (end_finishes.first == "fade") {
        if (sorted.empty()) throw std::out_of_range("list index out of range");
        sorted.front()->joints.get<RigidJoint>("0").connect_to(start_tip->joints.get<RigidJoint>("0"));
    }
    if (end_finishes.second == "fade") {
        if (sorted.empty()) throw std::out_of_range("list index out of range");
        sorted.back()->joints.get<RigidJoint>("1").connect_to(end_tip->joints.get<RigidJoint>("1"));
    }

    // Compound(label="thread", children=...): the compound of the children as they are now
    part.label = "thread";
    std::vector<Shape> kids;
    for (const auto& child : children) kids.push_back(*child);
    part.children = kids;
}

std::optional<Shape> ThreadSweep::chamfer_loop(const Shape& loop, const std::optional<Solid>& chamfer_shape) {
    if (!chamfer_shape) throw std::invalid_argument("a chamfered end needs a chamfer shape");
    const ShapeList<Shape> pieces = loop.intersect(*chamfer_shape);
    if (pieces.empty()) return std::nullopt;
    return Shape(pieces[0].wrapped());
}

std::vector<ThreadSweep::Child> ThreadSweep::finish_start(std::vector<Child> children, const std::string& finish,
                                                          const std::optional<Solid>& chamfer_shape) const {
    if (children.empty()) throw std::out_of_range("pop from empty list");
    const Child bottom_loop = children.front();
    children.erase(children.begin());
    const std::string label = bottom_loop->label;
    std::optional<Shape> kept;
    if (finish == "square") {
        kept = Part(build123d::split(*bottom_loop, Plane::XY, Keep::TOP).wrapped());
    } else {
        kept = chamfer_loop(*bottom_loop, chamfer_shape);
    }
    if (!kept) return children;
    kept->label = label;
    children.insert(children.begin(), std::make_shared<Shape>(*kept));
    return children;
}

std::vector<ThreadSweep::Child> ThreadSweep::finish_end(std::vector<Child> children, const std::string& finish,
                                                        const std::optional<Solid>& chamfer_shape) const {
    std::vector<Child> top_loops;
    bool last_square = false;
    for (int n = 0; n < 3; ++n) {
        if (children.empty()) continue;
        const Child top_loop = children.back();
        children.pop_back();
        const std::string label = top_loop->label;
        // A loop entirely above the cut plane is dropped
        const BoundBox bbox = top_loop->bounding_box();
        if (bbox.min.Z > length) continue;
        std::optional<Child> kept = top_loop;
        if (finish == "square") {
            // A loop entirely below the plane is kept without splitting; stop checking
            if (bbox.max.Z < length) {
                last_square = true;
            } else {
                kept = std::make_shared<Shape>(
                    Part(build123d::split(*top_loop, Plane::XY.offset(length), Keep::BOTTOM).wrapped()));
            }
        } else {
            const auto piece = chamfer_loop(*top_loop, chamfer_shape);
            kept = piece ? std::optional<Child>(std::make_shared<Shape>(*piece)) : std::nullopt;
        }
        if (kept && (*kept)->volume() != 0) {
            (*kept)->label = label;
            top_loops.push_back(*kept);
        }
        if (last_square) break;
    }
    children.insert(children.end(), top_loops.begin(), top_loops.end());
    return children;
}

Solid ThreadSweep::make_thread_loop(double loop_height) const {
    if (!(0.0 < loop_height && loop_height <= 1.0)) {
        throw std::invalid_argument("Invalid loop_height (" + detail::py_str(loop_height) + ")");
    }
    Builder thread_loop(3);
    // The thread path, in a BuildLine (published to the BuildPart as a pending edge)
    const Helix thread_path_wire(pitch, loop_height * pitch, root_radius, Vector(0, 0, 0), Vector(0, 0, 1), 0.0,
                                 !right_hand);
    {
        Builder line(1);
        line.add(thread_path_wire);
        thread_loop.publish(line.product());
    }
    for (int i = 0; i < 11; ++i) {
        const double u_value = i / 10.0;
        const Vector native_z_dir = thread_path_wire.tangent_at(u_value);
        const Vector z_dir_thread_axis = Vector(native_z_dir.X, native_z_dir.Y, 0).normalized();
        const Plane plane(thread_path_wire.position_at(u_value), Vector(0, 0, 1), z_dir_thread_axis);
        Builder sketch(2);
        detail::insert(sketch, {thread_profile});
        thread_loop.publish(sketch.product(), Mode::ADD, {Location()}, {Location(plane)});
    }
    detail::loft(thread_loop);

    Solid loop = thread_loop.solids()[0];
    for (int i = 0; i < 2; ++i) RigidJoint(std::to_string(i), loop, thread_path_wire.location_at(i));
    return loop;
}

Solid ThreadSweep::make_fade_end(bool bottom) const {
    const double direction = bottom ? -1 : 1;
    const double height = std::min(pitch / 4, length / 2);
    Builder fade_tip(3);
    const Helix fade_path_wire(pitch, direction * height, root_radius, Vector(0, 0, 0), Vector(0, 0, 1), 0.0,
                               !right_hand);
    {
        Builder line(1);
        line.add(fade_path_wire);
        fade_tip.publish(line.product());
    }
    for (int i = 0; i < 11; ++i) {
        const double u_value = i / 10.0;
        const Vector native_z_dir = fade_path_wire.tangent_at(u_value);
        Vector z_dir_thread_axis = Vector(native_z_dir.X, native_z_dir.Y, 0).normalized();
        if (bottom) z_dir_thread_axis = z_dir_thread_axis.reverse();
        const Plane plane(fade_path_wire.position_at(u_value), Vector(0, 0, 1), z_dir_thread_axis);
        Builder sketch(2);
        detail::insert(sketch, {thread_profile});
        detail::scale(sketch, std::nullopt, (11 - i) / 11.0);
        fade_tip.publish(sketch.product(), Mode::ADD, {Location()}, {Location(plane)});
    }
    detail::loft(fade_tip);

    Solid tip = fade_tip.solids()[0];
    RigidJoint("0", tip, fade_path_wire.location_at(0) * Location(Vector(0, 0, 0), Vector(1, 0, 0), 180));
    RigidJoint("1", tip, fade_path_wire.location_at(0));
    return tip;
}

Solid ThreadSweep::make_chamfer_shape() const {
    double inside_radius = std::min(apex_radius, root_radius);
    const double outside_radius = std::max(apex_radius, root_radius) + 0.001;
    Solid chamfer_shape;
    if (external) {
        chamfer_shape = Solid::extrude(Face(Wire::make_circle(outside_radius)), Vector(0, 0, length));
    } else {
        // Decreasing inside_radius fixes the broken/missing first row chamfer
        inside_radius -= 0.01;
        chamfer_shape = Solid::extrude(Face(Wire::make_circle(2 * outside_radius), {Wire::make_circle(inside_radius)}),
                                       Vector(0, 0, length));
    }
    const double thickness = outside_radius - inside_radius;
    for (int i = 0; i < 2; ++i) {
        if ((i == 0 ? end_finishes.first : end_finishes.second) != "chamfer") continue;
        const auto edges = chamfer_shape.edges()
                               .group_by(Axis::Z, i != 0)[0]
                               .sort_by(SortBy::RADIUS, apex_radius > root_radius);
        chamfer_shape = chamfer_shape.chamfer(thickness / 2, thickness / 2, std::vector<Edge>{edges.at(0)});
    }
    return chamfer_shape;
}

/// The thread profile of Thread: a trapezoid (with the interference overlap) made into a face
/// in a private BuildSketch, mirrored about XZ for a left hand thread.
Face thread_face(double apex_radius, double apex_width, double root_radius, double root_width, double apex_offset,
                 double interference, bool right_hand) {
    Builder thread_face(2);
    const double height = apex_radius - root_radius;
    const double overlap = -interference * std::copysign(1.0, height);
    {
        Builder line(1);
        if (overlap == 0) {
            line.add(Polyline({{root_width / 2, 0},
                               {apex_width / 2 + apex_offset, height},
                               {-apex_width / 2 + apex_offset, height},
                               {-root_width / 2, 0}},
                              true));
        } else {
            line.add(Polyline({{root_width / 2, overlap},
                               {root_width / 2, 0},
                               {apex_width / 2 + apex_offset, height},
                               {-apex_width / 2 + apex_offset, height},
                               {-root_width / 2, 0},
                               {-root_width / 2, overlap}},
                              true));
        }
        if (!right_hand) detail::mirror(line, std::nullopt, Plane::XZ, Mode::REPLACE);
        thread_face.publish(line.product());
    }
    detail::make_face(thread_face);
    return thread_face.faces()[0];
}

}  // namespace

// ---------------------------------------------------------------------------
// Thread
// ---------------------------------------------------------------------------

Thread::Thread(double apex_radius_, double apex_width_, double root_radius_, double root_width_, double pitch_,
               double length_, double apex_offset_, double interference_, const std::string& hand,
               std::optional<double> taper_angle, const EndFinishes& end_finishes_, bool simple_,
               double manufacturing_compensation_, const Rotation& rotation, std::optional<Align3> align)
    : Thread(apex_radius_, apex_width_, root_radius_, root_width_, pitch_, length_,
             ThreadOptions{apex_offset_, interference_, hand, taper_angle, end_finishes_, simple_,
                           manufacturing_compensation_, rotation, align}) {}

Thread::Thread(double apex_radius_, double apex_width_, double root_radius_, double root_width_, double pitch_,
               double length_, const ThreadOptions& o) {
    check_end_finishes(o.end_finishes);
    if (o.taper_angle) throw std::invalid_argument("taper_angle is not currently supported");

    external = apex_radius_ > root_radius_;
    apex_radius = apex_radius_;
    apex_width = apex_width_;
    root_radius = root_radius_;
    root_width = root_width_;
    pitch = pitch_;
    length = length_;
    apex_offset = o.apex_offset;
    interference = o.interference;
    right_hand = o.hand == "right";
    end_finishes = o.end_finishes;
    tooth_height = std::abs(apex_radius - root_radius);
    taper = o.taper_angle ? *o.taper_angle : 0;
    simple = o.simple;
    manufacturing_compensation = o.manufacturing_compensation;

    thread_profile = thread_face(apex_radius, apex_width, root_radius, root_width, apex_offset, interference, right_hand);

    if (simple) {
        // Initialize with a valid shape then nullify
        assign(Solid::make_box(1, 1, 1), Rotation(), std::nullopt);
        obj_ = TopoDS_Shape();
        return;
    }

    ThreadSweep thread_sweep(thread_profile, apex_radius, root_radius, pitch, length, o.hand, end_finishes,
                             manufacturing_compensation);
    thread_loops = thread_sweep.thread_loops;
    assign(thread_sweep.part, o.rotation, o.align);
}

// ---------------------------------------------------------------------------
// WhitworthThread, BSPPThread
// ---------------------------------------------------------------------------

WhitworthThread::WhitworthThread(double major_diameter_, double pitch_, double length_, bool external_,
                                 const std::string& hand_, const EndFinishes& end_finishes_, double interference_,
                                 bool simple_, double crest_truncation_, double manufacturing_compensation_,
                                 const Rotation& rotation, std::optional<Align3> align) {
    init(major_diameter_, pitch_, length_,
         WhitworthThreadOptions{external_, hand_, end_finishes_, interference_, simple_, crest_truncation_,
                                manufacturing_compensation_, rotation, align});
}

WhitworthThread::WhitworthThread(double major_diameter_, double pitch_, double length_,
                                 const WhitworthThreadOptions& options) {
    init(major_diameter_, pitch_, length_, options);
}

void WhitworthThread::init(double major_diameter_, double pitch_, double length_, const WhitworthThreadOptions& o) {
    if (o.hand != "right" && o.hand != "left") {
        throw std::invalid_argument("hand must be one of \"right\" or \"left\" not " + o.hand);
    }
    check_end_finishes(o.end_finishes);

    major_diameter = major_diameter_;
    pitch = pitch_;
    length = length_;
    external = o.external;
    hand = o.hand;
    end_finishes = o.end_finishes;
    interference = o.interference;
    simple = o.simple;
    manufacturing_compensation = o.manufacturing_compensation;
    if (!(0 <= o.crest_truncation && o.crest_truncation < 0.640327 * pitch)) {
        throw std::invalid_argument("crest_truncation must be at least 0 and less than h");
    }
    crest_truncation = o.crest_truncation;
    fundamental_triangle_height = 0.960491 * pitch;
    thread_height = 0.640327 * pitch;
    rounding_radius = 0.137329 * pitch;
    minor_diameter = major_diameter - 2 * thread_height;
    if (external) {
        apex_radius = major_diameter / 2 - crest_truncation;
        root_radius = minor_diameter / 2;
        crest_diameter = major_diameter - 2 * crest_truncation;
    } else {
        apex_radius = minor_diameter / 2 + crest_truncation;
        root_radius = major_diameter / 2;
        crest_diameter = minor_diameter + 2 * crest_truncation;
    }
    tooth_height = thread_height - crest_truncation;

    const double radial_direction = external ? 1 : -1;
    const double half_angle = thread_angle / 2;
    const double flank_angle = 90 - half_angle;
    const double crest_arc_angle = 180 - thread_angle;
    const double radius = rounding_radius;
    const double root_center_y = radial_direction * radius;
    const double crest_center_y = radial_direction * (thread_height - radius);
    const double flank_x = radius * std::cos(detail::radians(half_angle));
    const double root_tangent_y = radial_direction * (radius - radius * std::sin(detail::radians(half_angle)));
    const double crest_tangent_y =
        radial_direction * (thread_height - radius + radius * std::sin(detail::radians(half_angle)));
    const Vector right_root_tangent(pitch / 2 - flank_x, root_tangent_y);
    const Vector right_crest_tangent(flank_x, crest_tangent_y);
    const Vector left_crest_tangent(-flank_x, crest_tangent_y);
    const Vector left_root_tangent(-pitch / 2 + flank_x, root_tangent_y);
    const Vector right_valley(pitch / 2, 0);
    const Vector left_valley(-pitch / 2, 0);
    const double overlap = -interference * radial_direction;

    Builder thread_face(2);
    {
        Builder line(1);
        line.add(CenterArc(Vector(pitch / 2, root_center_y), radius, -90 * radial_direction,
                           -flank_angle * radial_direction));
        line.add(Line(right_root_tangent, right_crest_tangent));
        line.add(CenterArc(Vector(0, crest_center_y), radius, half_angle * radial_direction,
                           crest_arc_angle * radial_direction));
        line.add(Line(left_crest_tangent, left_root_tangent));
        line.add(CenterArc(Vector(-pitch / 2, root_center_y), radius, -half_angle * radial_direction,
                           -flank_angle * radial_direction));
        if (overlap == 0) {
            line.add(Line(left_valley, right_valley));
        } else {
            line.add(Polyline({left_valley, {-pitch / 2, overlap}, {pitch / 2, overlap}, right_valley}));
        }
        thread_face.publish(line.product());
    }
    detail::make_face(thread_face);
    thread_profile = thread_face.faces()[0];
    if (crest_truncation > 0) {
        const double crest_y = radial_direction * tooth_height;
        const Plane crest_plane = Plane::ZX.offset(crest_y);
        const auto truncated = thread_profile.split(crest_plane, external ? Keep::BOTTOM : Keep::TOP);
        if (truncated.size() != 1 || truncated[0].shape_type() != ShapeType::FACE) {
            throw std::runtime_error("Unable to create the truncated Whitworth profile");
        }
        thread_profile = Face(truncated[0].wrapped());
        // A flat crest centred exactly at profile X=0 is coincident with the bottom end plane
        // and can produce a null OCCT square/chamfer boolean; a negligible axial phase shift
        // avoids that coincidence.
        thread_profile.move(Location(Vector(1e-6, 0, 0)));
    }

    if (simple) {
        assign(Solid::make_box(1, 1, 1), Rotation(), std::nullopt);
        obj_ = TopoDS_Shape();
        return;
    }

    ThreadSweep thread_sweep(thread_profile, apex_radius, root_radius, pitch, length, hand, end_finishes,
                             manufacturing_compensation);
    thread_loops = thread_sweep.thread_loops;
    assign(thread_sweep.part, o.rotation, o.align);
}

std::vector<std::string> BSPPThread::sizes() {
    std::vector<std::string> out;
    for (const auto& [size, data] : iso_228_1_data()) out.push_back("G" + size);
    return out;
}

std::string BSPPThread::_normalize_size(const std::string& size) {
    std::string normalized = detail::py_strip(size);
    for (auto& c : normalized) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    normalized = detail::py_replace(detail::py_replace(normalized, "\"", ""), "\xE2\x80\xB3", "");  // ″
    if (!normalized.empty() && normalized[0] == 'G') normalized = normalized.substr(1);
    std::string joined;
    for (const auto& part : detail::py_split(normalized)) joined += (joined.empty() ? "" : " ") + part;
    return joined;
}

BSPPThread::BSPPThread(const std::string& size, double length_, bool external_,
                       std::optional<std::string> tolerance_class_, const std::string& hand_,
                       const EndFinishes& end_finishes_, double interference_, bool simple_,
                       std::optional<double> crest_diameter_, double manufacturing_compensation_,
                       const Rotation& rotation, std::optional<Align3> align)
    : BSPPThread(size, length_,
                 BSPPThreadOptions{external_, tolerance_class_, hand_, end_finishes_, interference_, simple_,
                                   crest_diameter_, manufacturing_compensation_, rotation, align}) {}

BSPPThread::BSPPThread(const std::string& size, double length_, const BSPPThreadOptions& o) {
    const std::string normalized_size = _normalize_size(size);
    const auto found = iso_228_1_data().get(normalized_size);
    if (!found) throw std::invalid_argument("size invalid, must be one of " + detail::py_list_repr(sizes()));
    const auto& data = *found;

    std::optional<std::string> selected_class;
    if (o.external) {
        std::string upper = o.tolerance_class ? *o.tolerance_class : "A";
        for (auto& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (upper != "A" && upper != "B") throw std::invalid_argument("tolerance_class must be one of \"A\" or \"B\"");
        selected_class = upper;
    } else if (o.tolerance_class) {
        throw std::invalid_argument("internal BSPP threads have one tolerance class");
    }

    thread_size = "G" + normalized_size;
    tpi = static_cast<int>(data.at("tpi"));
    tolerance_class = selected_class;
    standard = "ISO 228-1:1994";
    basic_major_diameter = data.at("major_diameter");
    basic_pitch_diameter = data.at("pitch_diameter");
    basic_minor_diameter = data.at("minor_diameter");

    const std::string designation_class = o.external ? *selected_class : "";
    const std::string designation_hand = o.hand == "left" ? " LH" : "";
    designation = thread_size + designation_class + designation_hand;

    const double pitch_tolerance = data.at("pitch_tolerance");
    if (o.external) {
        const double class_factor = *selected_class == "A" ? 1 : 2;
        pitch_diameter_limits = {basic_pitch_diameter - class_factor * pitch_tolerance, basic_pitch_diameter};
        major_diameter_limits = std::make_pair(basic_major_diameter - data.at("external_major_tolerance"),
                                               basic_major_diameter);
        minor_diameter_limits = std::nullopt;
    } else {
        pitch_diameter_limits = {basic_pitch_diameter, basic_pitch_diameter + pitch_tolerance};
        major_diameter_limits = std::nullopt;
        minor_diameter_limits = std::make_pair(basic_minor_diameter,
                                               basic_minor_diameter + data.at("internal_minor_tolerance"));
    }

    const std::pair<double, double> crest_limits = o.external ? *major_diameter_limits : *minor_diameter_limits;
    double selected_crest_diameter = o.external ? crest_limits.second : crest_limits.first;
    if (o.crest_diameter) selected_crest_diameter = *o.crest_diameter;
    if (!(crest_limits.first <= selected_crest_diameter && selected_crest_diameter <= crest_limits.second)) {
        throw std::invalid_argument("crest_diameter must be within " +
                                    detail::py_tuple_repr(std::vector<double>{crest_limits.first, crest_limits.second}));
    }
    double truncation;
    if (o.external) {
        truncation = (basic_major_diameter - selected_crest_diameter) / 2;
    } else {
        const double full_form_minor_diameter = basic_major_diameter - (2 * 0.640327 * (25.4 * MM / tpi));
        truncation = std::max(0.0, (selected_crest_diameter - full_form_minor_diameter) / 2);
    }

    init(basic_major_diameter, 25.4 * MM / tpi, length_,
         WhitworthThreadOptions{o.external, o.hand, o.end_finishes, o.interference, o.simple, truncation,
                                o.manufacturing_compensation, o.rotation, o.align});
}

// ---------------------------------------------------------------------------
// IsoThread
// ---------------------------------------------------------------------------

IsoThread::IsoThread(double major_diameter_, double pitch_, double length_, bool external_, const std::string& hand_,
                     const EndFinishes& end_finishes_, double interference_, bool simple_,
                     double manufacturing_compensation_, const Rotation& rotation, std::optional<Align3> align)
    : IsoThread(major_diameter_, pitch_, length_,
                IsoThreadOptions{external_, hand_, end_finishes_, interference_, simple_, manufacturing_compensation_,
                                 rotation, align}) {}

IsoThread::IsoThread(double major_diameter_, double pitch_, double length_, const IsoThreadOptions& o) {
    major_diameter = major_diameter_;
    pitch = pitch_;
    length = length_;
    external = o.external;
    thread_angle = 60;
    check_hand(o.hand);
    hand = o.hand;
    check_end_finishes(o.end_finishes);
    end_finishes = o.end_finishes;
    interference = o.interference;
    simple = o.simple;
    manufacturing_compensation = o.manufacturing_compensation;
    apex_radius = external ? major_diameter / 2 : min_radius();
    const double apex_width = external ? pitch / 8 : pitch / 4;
    root_radius = external ? min_radius() : major_diameter / 2;
    const double root_width = external ? 3 * pitch / 4 : 7 * pitch / 8;
    if (simple) {
        // Initialize with a valid shape then nullify
        assign(Solid::make_box(1, 1, 1), Rotation(), std::nullopt);
        obj_ = TopoDS_Shape();
        return;
    }
    const Thread bd_object(apex_radius, apex_width, root_radius, root_width, pitch, length,
                           ThreadOptions{.interference = interference,
                                         .hand = hand,
                                         .end_finishes = end_finishes,
                                         .simple = simple,
                                         .manufacturing_compensation = manufacturing_compensation});
    thread_profile = bd_object.thread_profile;
    assign(Compound(bd_object.solids()), o.rotation, o.align);
}

double IsoThread::h_parameter() const { return (pitch / 2) / std::tan(detail::radians(thread_angle / 2)); }

double IsoThread::min_radius() const { return (major_diameter - 2 * (5.0 / 8) * h_parameter()) / 2; }

// ---------------------------------------------------------------------------
// TrapezoidalThread, AcmeThread, MetricTrapezoidalThread
// ---------------------------------------------------------------------------

TrapezoidalThread::TrapezoidalThread(double diameter_, double pitch_, double thread_angle_, double length_,
                                     bool external_, int starts, const std::string& hand_,
                                     const EndFinishes& end_finishes_, double interference_,
                                     double manufacturing_compensation_, const Rotation& rotation,
                                     std::optional<Align3> align) {
    init(diameter_, pitch_, thread_angle_, length_,
         TrapezoidalThreadOptions{external_, starts, hand_, end_finishes_, interference_, manufacturing_compensation_,
                                  rotation, align});
}

TrapezoidalThread::TrapezoidalThread(double diameter_, double pitch_, double thread_angle_, double length_,
                                     const TrapezoidalThreadOptions& options) {
    init(diameter_, pitch_, thread_angle_, length_, options);
}

void TrapezoidalThread::init(double diameter_, double pitch_, double thread_angle_, double length_,
                             const TrapezoidalThreadOptions& o) {
    thread_size = diameter_;
    external = o.external;
    length = length_;
    diameter = diameter_;
    thread_angle = thread_angle_;
    pitch = pitch_;
    lead = pitch * o.starts;
    const double shoulder_width = (pitch / 2) * std::tan(detail::radians(thread_angle / 2));
    const double apex_width = (pitch / 2) - shoulder_width;
    const double root_width = (pitch / 2) + shoulder_width;
    if (external) {
        apex_radius = diameter / 2;
        root_radius = diameter / 2 - pitch / 2;
    } else {
        apex_radius = diameter / 2 - pitch / 2;
        root_radius = diameter / 2;
    }
    check_hand(o.hand);
    hand = o.hand;
    check_end_finishes(o.end_finishes);
    end_finishes = o.end_finishes;
    interference = o.interference;
    manufacturing_compensation = o.manufacturing_compensation;
    const Thread bd_object(apex_radius, apex_width, root_radius, root_width, lead, length,
                           ThreadOptions{.interference = interference,
                                         .hand = hand,
                                         .end_finishes = end_finishes,
                                         .manufacturing_compensation = manufacturing_compensation});
    thread_profile = bd_object.thread_profile;
    std::vector<Shape> starts_list;
    for (int i = 0; i < o.starts; ++i) {
        Shape start = bd_object;  // copy.copy
        start.move(Rotation(0, 0, i * (360.0 / o.starts)));
        starts_list.push_back(start);
    }
    Compound thread_object;
    thread_object.children = starts_list;
    assign(thread_object, o.rotation, o.align);
}

const Dict<std::string, double> AcmeThread::acme_pitch = {
    {"1/4", (1.0 / 16) * IN},  {"5/16", (1.0 / 14) * IN}, {"3/8", (1.0 / 12) * IN}, {"1/2", (1.0 / 10) * IN},
    {"5/8", (1.0 / 8) * IN},   {"3/4", (1.0 / 6) * IN},   {"7/8", (1.0 / 6) * IN},  {"1", (1.0 / 5) * IN},
    {"1 1/4", (1.0 / 5) * IN}, {"1 1/2", (1.0 / 4) * IN}, {"1 3/4", (1.0 / 4) * IN}, {"2", (1.0 / 4) * IN},
    {"2 1/2", (1.0 / 3) * IN}, {"3", (1.0 / 2) * IN}};

std::vector<std::string> AcmeThread::sizes() { return acme_pitch.keys(); }

AcmeThread::AcmeThread(const std::string& size, double length_, bool external_, const std::string& hand_,
                       const EndFinishes& end_finishes_, double interference_, double manufacturing_compensation_,
                       const Rotation& rotation, std::optional<Align3> align)
    : AcmeThread(size, length_,
                 AcmeThreadOptions{external_, hand_, end_finishes_, interference_, manufacturing_compensation_,
                                   rotation, align}) {}

AcmeThread::AcmeThread(const std::string& size, double length_, const AcmeThreadOptions& o) {
    const Param diameter_param = imperial_str_to_float(size);
    const auto pitch_ = acme_pitch.get(size);
    if (!pitch_) throw std::invalid_argument("Invalid screw size");
    init(diameter_param.number(), *pitch_, thread_angle, length_,
         TrapezoidalThreadOptions{o.external, 1, o.hand, o.end_finishes, o.interference, o.manufacturing_compensation,
                                  o.rotation, o.align});
}

const std::vector<std::string> MetricTrapezoidalThread::standard_sizes = {
    "8x1.5", "9x1.5", "9x2", "10x1.5", "10x2", "11x2", "11x3", "12x2", "12x3", "14x2",
    "14x3", "16x2", "16x3", "16x4", "18x2", "18x3", "18x4", "20x2", "20x3", "20x4",
    "22x3", "22x5", "22x8", "24x3", "24x5", "24x8", "26x3", "26x5", "26x8", "28x3",
    "28x5", "28x8", "30x3", "30x6", "30x10", "32x3", "32x6", "32x10", "34x3", "34x6",
    "34x10", "36x3", "36x6", "36x10", "38x3", "38x7", "38x10", "40x3", "40x7", "40x10",
    "42x3", "42x7", "42x10", "44x3", "44x7", "44x12", "46x3", "46x8", "46x12", "48x3",
    "48x8", "48x12", "50x3", "50x8", "50x12", "52x3", "52x8", "52x12", "55x3", "55x9",
    "55x14", "60x3", "60x9", "60x14", "65x4", "65x10", "65x16", "70x4", "70x10", "70x16",
    "75x4", "75x10", "75x16", "80x4", "80x10", "80x16", "85x4", "85x12", "85x18", "90x4",
    "90x12", "90x18", "95x4", "95x12", "95x18", "100x4", "100x12", "100x20", "105x4", "105x12",
    "105x20", "110x4", "110x12", "110x20", "115x6", "115x12", "115x14", "115x22", "120x6", "120x12",
    "120x14", "120x22", "125x6", "125x12", "125x14", "125x22", "130x6", "130x12", "130x14", "130x22",
    "135x6", "135x12", "135x14", "135x24", "140x6", "140x12", "140x14", "140x24", "145x6", "145x12",
    "145x14", "145x24", "150x6", "150x12", "150x16", "150x24", "155x6", "155x12", "155x16", "155x24",
    "160x6", "160x12", "160x16", "160x28", "165x6", "165x12", "165x16", "165x28", "170x6", "170x12",
    "170x16", "170x28", "175x8", "175x12", "175x16", "175x28", "180x8", "180x12", "180x18", "180x28",
    "185x8", "185x12", "185x18", "185x24", "185x32", "190x8", "190x12", "190x18", "190x24", "190x32",
    "195x8", "195x12", "195x18", "195x24", "195x32", "200x8", "200x12", "200x18", "200x24", "200x32",
    "205x4", "210x4", "210x8", "210x12", "210x20", "210x24", "210x36", "215x4", "220x4", "220x8",
    "220x12", "220x20", "220x24", "220x36", "230x4", "230x8", "230x12", "230x20", "230x24", "230x36",
    "235x4", "240x4", "240x8", "240x12", "240x20", "240x22", "240x24", "240x36", "250x4", "250x12",
    "250x22", "250x24", "250x40", "260x4", "260x12", "260x20", "260x22", "260x24", "260x40", "270x12",
    "270x24", "270x40", "275x4", "280x4", "280x12", "280x24", "280x40", "290x4", "290x12", "290x24",
    "290x44", "295x4", "300x4", "300x12", "300x24", "300x44", "310x5", "315x5"};

std::vector<std::string> MetricTrapezoidalThread::sizes() { return standard_sizes; }

MetricTrapezoidalThread::MetricTrapezoidalThread(const std::string& size, double length_, bool external_,
                                                 const std::string& hand_, const EndFinishes& end_finishes_,
                                                 double interference_, double manufacturing_compensation_,
                                                 const Rotation& rotation, std::optional<Align3> align)
    : MetricTrapezoidalThread(size, length_,
                              MetricTrapezoidalThreadOptions{external_, hand_, end_finishes_, interference_,
                                                             manufacturing_compensation_, rotation, align}) {}

MetricTrapezoidalThread::MetricTrapezoidalThread(const std::string& size, double length_,
                                                 const MetricTrapezoidalThreadOptions& o) {
    if (std::find(standard_sizes.begin(), standard_sizes.end(), size) == standard_sizes.end()) {
        throw std::invalid_argument("size invalid, must be one of " + detail::py_list_repr(standard_sizes));
    }
    const auto parts = detail::py_split(size, "x");
    if (parts.size() != 2) throw std::invalid_argument("size invalid");
    init(detail::py_float(parts[0]), detail::py_float(parts[1]), thread_angle, length_,
         TrapezoidalThreadOptions{o.external, 1, o.hand, o.end_finishes, o.interference, o.manufacturing_compensation,
                                  o.rotation, o.align});
}

// ---------------------------------------------------------------------------
// PlasticBottleThread
// ---------------------------------------------------------------------------

std::vector<std::string> PlasticBottleThread::bottle_types() { return {"astm_d2911", "pco1881"}; }

std::vector<std::string> PlasticBottleThread::sizes(const std::string& bottle_type) {
    if (bottle_type == "pco1881") return pco1881_data().keys();
    if (bottle_type != "astm_d2911") {
        throw std::invalid_argument("bottle_type must be one of " + detail::py_tuple_repr(bottle_types()));
    }
    std::vector<std::string> out;
    for (const auto& [finish, data] : astm_d2911_finishes()) {
        for (const int diameter : data.diameters) {
            for (const char* style : {"L", "M"}) {
                out.push_back(std::string(style) + std::to_string(diameter) + "SP" + std::to_string(finish));
            }
        }
    }
    return out;
}

std::vector<PlasticBottleThreadMatch> PlasticBottleThread::identify(std::optional<double> neck_diameter,
                                                                    std::optional<double> major_diameter,
                                                                    std::optional<double> pitch,
                                                                    std::optional<double> tpi,
                                                                    std::optional<double> turns,
                                                                    const std::string& style, double tolerance) {
    if (!neck_diameter && !major_diameter) {
        throw std::invalid_argument("one of neck_diameter or major_diameter is required");
    }
    if (pitch && tpi) throw std::invalid_argument("give pitch or tpi, not both");
    if (style != "L" && style != "M") {
        throw std::invalid_argument("style must be one of \"L\" or \"M\" not " + style);
    }
    if (tpi) {
        if (*tpi == 0) throw std::domain_error("float division by zero");
        pitch = 25.4 * MM / *tpi;
    }

    // How far a measurement falls outside a range
    const auto outside = [](std::optional<double> measured, double low, double high) {
        if (!measured) return 0.0;
        return std::max({low - *measured, *measured - high, 0.0});
    };

    // each standard thread, and how many sizes its finish is made in
    std::vector<std::pair<PlasticBottleThreadMatch, int>> standards;
    for (const auto& [diameter, dimensions] : astm_d2911_dimensions()) {
        const double height = astm_d2911_profiles().at({style, dimensions.tpi}).thread_height;
        const std::pair<double, double> major{dimensions.diameter_min, dimensions.diameter_max};
        const std::pair<double, double> neck{detail::py_round(major.first - 2 * height, 4),
                                             detail::py_round(major.second - 2 * height, 4)};
        for (const auto& [finish, data] : astm_d2911_finishes()) {
            if (std::find(data.diameters.begin(), data.diameters.end(), diameter) == data.diameters.end()) continue;
            standards.push_back({PlasticBottleThreadMatch{style + std::to_string(diameter) + "SP" + std::to_string(finish),
                                                          "astm_d2911", major, neck, 25.4 * MM / dimensions.tpi,
                                                          data.min_turns, 0.0},
                                 static_cast<int>(data.diameters.size())});
        }
    }
    for (const auto& [size, data] : pco1881_data()) {
        standards.push_back(
            {PlasticBottleThreadMatch{size, "pco1881",
                                      {data.at("bottle_major_diameter"), data.at("bottle_major_diameter")},
                                      {data.at("bottle_minor_diameter"), data.at("bottle_minor_diameter")},
                                      data.at("pitch"), std::nullopt, 0.0},
             1});
    }

    struct Ranked {
        double error;
        int negative_sizes_made;
        PlasticBottleThreadMatch standard;
    };
    std::vector<Ranked> matches;
    for (const auto& [standard, sizes_made] : standards) {
        const double errors[] = {outside(major_diameter, standard.major_diameter.first, standard.major_diameter.second),
                                 outside(neck_diameter, standard.neck_diameter.first, standard.neck_diameter.second),
                                 outside(pitch, standard.pitch, standard.pitch)};
        // a quarter turn is as close as turns can be counted
        const bool too_few = turns && standard.min_turns && *turns < *standard.min_turns - 0.25;
        if (std::max({errors[0], errors[1], errors[2]}) <= tolerance && !too_few) {
            matches.push_back({errors[0] + errors[1] + errors[2], -sizes_made, standard});
        }
    }
    std::stable_sort(matches.begin(), matches.end(), [](const Ranked& a, const Ranked& b) {
        if (a.error != b.error) return a.error < b.error;
        return a.negative_sizes_made < b.negative_sizes_made;
    });
    std::vector<PlasticBottleThreadMatch> out;
    for (auto& match : matches) {
        match.standard.error = match.error;
        out.push_back(match.standard);
    }
    return out;
}

PlasticBottleThread::PlasticBottleThread(const std::string& size, bool external_, const std::string& hand_,
                                         double interference_, const std::string& bottle_type_,
                                         double manufacturing_compensation_, const Rotation& rotation,
                                         std::optional<Align3> align)
    : PlasticBottleThread(size, PlasticBottleThreadOptions{external_, hand_, interference_, bottle_type_,
                                                           manufacturing_compensation_, rotation, align}) {}

PlasticBottleThread::PlasticBottleThread(const std::string& size, const PlasticBottleThreadOptions& o) {
    thread_size = size;
    external = o.external;
    bottle_type = o.bottle_type;
    if (bottle_type != "astm_d2911" && bottle_type != "pco1881") {
        throw std::invalid_argument("bottle_type must be one of " + detail::py_tuple_repr(bottle_types()));
    }
    check_hand(o.hand);
    hand = o.hand;
    interference = o.interference;
    manufacturing_compensation = o.manufacturing_compensation;

    if (bottle_type == "pco1881") {
        std::string pco_size = size;
        for (auto& c : pco_size) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (pco_size.size() >= 2 && pco_size.compare(pco_size.size() - 2, 2, "mm") == 0) {
            pco_size = pco_size.substr(0, pco_size.size() - 2);
        }
        const auto found = pco1881_data().get(pco_size);
        if (!found) {
            throw std::invalid_argument("size invalid, must be one of " + detail::py_list_repr(pco1881_data().keys()) +
                                        " for PCO1881");
        }
        const auto& data = *found;
        bottle_major_diameter = data.at("bottle_major_diameter");
        bottle_minor_diameter = data.at("bottle_minor_diameter");
        closure_minor_diameter = data.at("closure_minor_diameter");
        diameter = *bottle_major_diameter;
        major_diameter = diameter;
        minor_diameter = external ? *bottle_minor_diameter : *closure_minor_diameter;
        pitch = data.at("pitch");
        starts = static_cast<int>(data.at("starts"));
        lead = pitch * starts;
        length = data.at("length");
        root_width = data.at("root_width");
        apex_width = data.at("apex_width");
        apex_offset = data.at("apex_offset");
        if (!external) apex_offset = -apex_offset;
        const double compensation = manufacturing_compensation;
        if (external) {
            apex_radius = major_diameter / 2 - compensation;
            root_radius = minor_diameter / 2 - compensation;
        } else {
            root_radius = major_diameter / 2 + compensation;
            apex_radius = minor_diameter / 2 + compensation;
        }
        tpi = 25.4 * MM / pitch;
        finish = std::nullopt;
        style = std::nullopt;
        const Thread bd_object(apex_radius, apex_width, root_radius, root_width, pitch, length,
                               ThreadOptions{.apex_offset = apex_offset,
                                             .interference = interference,
                                             .hand = hand,
                                             .end_finishes = {"fade", "fade"}});
        thread_profile = bd_object.thread_profile;
        assign(bd_object, o.rotation, o.align);
        return;
    }

    // re.match(r"([LM])(\d+)SP(\d+)", size)
    std::size_t pos = 0;
    bool matched = !size.empty() && (size[0] == 'L' || size[0] == 'M');
    std::string diameter_digits, finish_digits;
    if (matched) {
        pos = 1;
        while (pos < size.size() && std::isdigit(static_cast<unsigned char>(size[pos]))) diameter_digits += size[pos++];
        matched = !diameter_digits.empty() && size.compare(pos, 2, "SP") == 0;
    }
    if (matched) {
        pos += 2;
        while (pos < size.size() && std::isdigit(static_cast<unsigned char>(size[pos]))) finish_digits += size[pos++];
        matched = !finish_digits.empty();
    }
    if (!matched) {
        throw std::invalid_argument(
            "size invalid, must match                     "
            "[L|M][diameter(mm)]SP[100|103|110|200|400|410|415:425|444]");
    }
    style = std::string(1, size[0]);
    const int diameter_int = static_cast<int>(detail::py_int_str(diameter_digits));
    diameter = diameter_int;
    finish = static_cast<int>(detail::py_int_str(finish_digits));
    if (!astm_d2911_finishes().contains(*finish)) {
        throw std::invalid_argument("finish (" + std::to_string(*finish) + ") invalid, must be one of " +
                                    detail::py_list_repr(astm_d2911_finishes().keys()));
    }
    const auto& finish_data = astm_d2911_finishes().at(*finish);
    if (std::find(finish_data.diameters.begin(), finish_data.diameters.end(), diameter_int) ==
        finish_data.diameters.end()) {
        throw std::invalid_argument("diameter (" + std::to_string(diameter_int) + ") invalid, must be one of " +
                                    detail::py_list_repr(finish_data.diameters));
    }
    const auto& dimensions = astm_d2911_dimensions().at(diameter_int);
    const double diameter_max = dimensions.diameter_max;
    const double diameter_min = dimensions.diameter_min;
    tpi = dimensions.tpi;
    const auto& profile = astm_d2911_profiles().at({*style, dimensions.tpi});
    root_width = profile.root_width;
    const double thread_height = profile.thread_height;
    major_diameter = diameter_max;
    minor_diameter = diameter_min;
    starts = 1;
    if (external) {
        apex_radius = diameter_min / 2 - manufacturing_compensation;
        root_radius = diameter_min / 2 - thread_height - manufacturing_compensation;
    } else {
        root_radius = diameter_max / 2 + manufacturing_compensation;
        apex_radius = diameter_max / 2 - thread_height + manufacturing_compensation;
    }
    const auto thread_angles = finish_data.angles.at(*style);
    const double shoulders[] = {thread_height * std::tan(detail::radians(thread_angles.first)),
                                thread_height * std::tan(detail::radians(thread_angles.second))};
    apex_width = root_width - (shoulders[0] + shoulders[1]);
    apex_offset = shoulders[0] + apex_width / 2 - root_width / 2;
    if (!external) apex_offset = -apex_offset;
    pitch = 25.4 * MM / tpi;
    length = (finish_data.min_turns + 0.75) * pitch;
    lead = pitch;
    const Thread bd_object(apex_radius, apex_width, root_radius, root_width, pitch, length,
                           ThreadOptions{.apex_offset = apex_offset,
                                         .interference = interference,
                                         .hand = hand,
                                         .end_finishes = {"fade", "fade"}});
    thread_profile = bd_object.thread_profile;
    assign(bd_object, o.rotation, o.align);
}

}  // namespace bd_warehouse
