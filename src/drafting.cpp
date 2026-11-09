// Drafting objects (build123d's drafting.py). Python builds ArrowHead in builder mode
// (BuildSketch/BuildLine); the algebra below gives the same shapes (checked).
#include "build123d/drafting.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>
#include <numeric>

#include "build123d/operations.hpp"
#include "internal.hpp"
#include "py_compat.hpp"

namespace build123d {

using namespace detail;

namespace {

/// Python's round(x, ndigits): the double nearest to x correctly rounded (half to even) to
/// `ndigits` decimals (from the exact decimal expansion of x, as dtoa does).
double python_round(double x, int ndigits) {
    if (!std::isfinite(x) || x == 0.0) return x;
    if (ndigits > 323) return x;
    if (ndigits < -308) return 0.0 * x;
    char buffer[1200];
    std::snprintf(buffer, sizeof(buffer), "%.1100f", std::fabs(x));  // exact
    const std::string text(buffer);
    const std::size_t dot = text.find('.');
    const std::string digits = text.substr(0, dot) + text.substr(dot + 1);
    const long point = static_cast<long>(dot);  // digits before the decimal point
    const long keep = point + ndigits;          // digits kept
    if (keep < 0) return std::copysign(0.0, x);
    if (keep >= static_cast<long>(digits.size())) return x;
    std::string kept = digits.substr(0, static_cast<std::size_t>(keep));
    const char first_dropped = digits[static_cast<std::size_t>(keep)];
    const bool rest_nonzero = digits.find_first_not_of('0', static_cast<std::size_t>(keep) + 1) != std::string::npos;
    bool up = first_dropped > '5' || (first_dropped == '5' && rest_nonzero);
    if (first_dropped == '5' && !rest_nonzero) {
        const int last = kept.empty() ? 0 : kept.back() - '0';
        up = last % 2 == 1;  // a tie goes to the even digit
    }
    if (up) {
        long i = static_cast<long>(kept.size()) - 1;
        while (i >= 0 && kept[static_cast<std::size_t>(i)] == '9') kept[static_cast<std::size_t>(i--)] = '0';
        if (i < 0) {
            kept.insert(kept.begin(), '1');
        } else {
            ++kept[static_cast<std::size_t>(i)];
        }
    }
    if (kept.empty()) kept = "0";
    const std::string decimal = kept + "e" + std::to_string(-ndigits);
    return std::copysign(std::strtod(decimal.c_str(), nullptr), x);
}

/// Python's f"{x:.{n}f}".
std::string fixed(double x, int n) {
    char buffer[400];
    std::snprintf(buffer, sizeof(buffer), "%.*f", n, x);
    return buffer;
}

/// Python's str(int(x)) of a whole number.
std::string int_str(double x) {
    if (std::isnan(x)) throw std::invalid_argument("cannot convert float NaN to integer");
    if (std::isinf(x)) throw std::overflow_error("cannot convert float infinity to integer");
    if (x == 0.0) return "0";
    return fixed(x, 0);
}

/// Python's round(x) of a float: the nearest integer, ties to even.
double py_round_int(double x) {
    const double r = std::round(x);
    if (std::fabs(x - r) == 0.5) return 2.0 * std::round(x / 2.0);
    return r;
}

/// The edges of a path in order (a Wire's in connection order).
ShapeList<Edge> path_edges(const Mixin1D& path) {
    if (path.shape_type() == ShapeType::WIRE) return Wire(path.wrapped()).edges();
    return path.edges();
}

/// The area of Sketch.intersect(a, b): of their common faces (0 when they don't meet).
double intersection_area(const Shape& a, const Shape& b) {
    const TopoDS_Shape common = bool_op(BoolOp::COMMON, {a.wrapped()}, {b.wrapped()});
    double area = 0.0;
    for (const auto& f : entities(common, TopAbs_FACE)) area += Face(f).area();
    return area;
}

// ---------------------------------------------------------------------------
// ArrowHead, Arrow
// ---------------------------------------------------------------------------

Sketch arrow_head_shape(double size, HeadType head_type) {
    if (head_type == HeadType::STRAIGHT) {
        return Polygon({{-size, size / 3}, {-size, -size / 3}, {0, 0}}, 0.0, Align::NONE);
    }
    // CURVED: a side of tangent arc and line, mirrored about XZ, made into a face
    const TangentArc side({0, 0}, {-size, size / 3}, {-size, size / 6});
    const Line line(side.position_at(1), {-7 * size / 8, 0});
    const Curve half = side + line;
    const Sketch curved = make_face(half + build123d::mirror(half, Plane::XZ));
    if (head_type == HeadType::CURVED) return curved;
    // FILLETED: the curved head with its back corners rounded
    return Sketch(fillet(curved.vertices().filter_by_position(Axis::X, -2 * size, -size / 5), size / 20));
}

template <class Path>
Shape arrow_shape(double arrow_size, const Path& path, double shaft_width, bool head_at_start, HeadType head_type) {
    const double angle = head_at_start ? path.tangent_angle_at(0) + 180 : path.tangent_angle_at(1);
    // Create the arrow head
    const Sketch arrow_head =
        ArrowHead(arrow_size, head_type, angle).moved(Location(path.position_at(head_at_start ? 0 : 1)));
    // Trim the path so the tip of the arrow isn't lost
    const double trim_amount = (arrow_size / 2) / path.length();
    const Path shaft_path = head_at_start ? path.trim(trim_amount, 1.0) : path.trim(0.0, 1.0 - trim_amount);
    // A perpendicular line swept along the shaft path
    const Edge shaft_pen = shaft_path.perpendicular_line(shaft_width, 0);
    const Sketch shaft = sweep(shaft_pen, shaft_path);
    return Shape(clean_shape(bool_op(BoolOp::FUSE, {arrow_head.wrapped()}, {shaft.wrapped()})));
}

}  // namespace

ArrowHead::ArrowHead(double size, HeadType head_type, double rotation)
    : BaseSketchObject(arrow_head_shape(size, head_type), rotation, std::nullopt) {}

Arrow::Arrow(double arrow_size, const Edge& shaft_path, double shaft_width, bool head_at_start, HeadType head_type)
    : BaseSketchObject(arrow_shape(arrow_size, shaft_path, shaft_width, head_at_start, head_type), 0, std::nullopt) {}

Arrow::Arrow(double arrow_size, const Wire& shaft_path, double shaft_width, bool head_at_start, HeadType head_type)
    : BaseSketchObject(arrow_shape(arrow_size, shaft_path, shaft_width, head_at_start, head_type), 0, std::nullopt) {}

// ---------------------------------------------------------------------------
// PathDescriptor, DimensionTolerance
// ---------------------------------------------------------------------------

PathDescriptor::PathDescriptor(const std::vector<Vertex>& points) {
    std::vector<Vector> pnts;
    for (const auto& p : points) pnts.emplace_back(p);
    value = pnts;
}

std::optional<double> DimensionTolerance::symmetric() const {
    if (const double* v = std::get_if<double>(&value_)) return *v;
    return std::nullopt;
}

std::optional<std::pair<double, double>> DimensionTolerance::plus_minus() const {
    if (const auto* v = std::get_if<std::pair<double, double>>(&value_)) return *v;
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Draft
// ---------------------------------------------------------------------------

const std::map<bool, std::string> Draft::unit_LUT = {{true, "mm"}, {false, "\""}};

Draft::Draft(const DraftOptions& options) : DraftOptions(options) {
    // Validate inputs: log2(fractional_precision) must be a whole number
    if (fractional_precision <= 0) throw std::invalid_argument("math domain error");
    const double exponent = std::log2(static_cast<double>(fractional_precision));
    if (exponent != std::floor(exponent)) {
        throw std::invalid_argument("fractional_precision values must be a factor of 2 not " +
                                    std::to_string(fractional_precision));
    }
}

bool Draft::is_metric() const { return unit == Unit::MM || unit == Unit::CM || unit == Unit::M || unit == Unit::MC; }

std::string Draft::_round_to_str(double number) const {
    if (decimal_precision > 0) return fixed(python_round(number, decimal_precision), decimal_precision);
    return int_str(python_round(number, decimal_precision));
}

std::string Draft::_number_with_units(double number, const DimensionTolerance& tolerance,
                                      std::optional<bool> display_units) const {
    const bool qualified_display_units =
        display_units ? *display_units : (tolerance.has_value() ? false : this->display_units);
    const std::string unit_str = qualified_display_units ? unit_LUT.at(is_metric()) : "";
    std::string tolerance_str;
    if (const auto value = tolerance.symmetric()) {
        tolerance_str = " \xC2\xB1" + _number_with_units(*value);  // ±
    } else if (const auto pair = tolerance.plus_minus()) {
        tolerance_str = " +" + _number_with_units(pair->first, {}, false) + " -" + _number_with_units(pair->second);
    }
    if (is_metric() || number_display == NumberDisplay::DECIMAL) {
        const std::string measurement = _round_to_str(number / (is_metric() ? MM : IN));
        return measurement + unit_str + tolerance_str;
    }
    // Fractions of an inch, simplified
    const double whole_part = std::floor(number / IN);
    const auto numerator0 = static_cast<long long>(py_round_int((number / IN - whole_part) * fractional_precision));
    const long long greatest_common_denominator = std::gcd(numerator0, static_cast<long long>(fractional_precision));
    const long long numerator = numerator0 / greatest_common_denominator;
    const long long denominator = fractional_precision / greatest_common_denominator;
    const std::string fraction = std::to_string(numerator) + "/" + std::to_string(denominator);
    if (whole_part == 0) return fraction + unit_str + tolerance_str;
    return int_str(whole_part) + " " + fraction + unit_str + tolerance_str;
}

std::variant<Edge, Wire> Draft::_process_path(const PathDescriptor& path) {
    if (const Edge* edge = std::get_if<Edge>(&path.value)) return *edge;
    if (const Wire* wire = std::get_if<Wire>(&path.value)) return *wire;
    const auto& pnts = std::get<std::vector<Vector>>(path.value);
    if (pnts.size() == 2) return Edge::make_line(pnts[0], pnts[1]);
    return Wire::make_polygon(pnts, false);
}

std::string Draft::_label_to_str(const std::optional<std::string>& label, const Mixin1D& line_wire, bool label_angle,
                                 const DimensionTolerance& tolerance) const {
    const double line_length = line_wire.length();
    if (label) return *label;
    if (label_angle) {
        const ShapeList<Edge> arc_edges = path_edges(line_wire).filter_by(GeomType::CIRCLE);
        if (arc_edges.empty()) throw std::invalid_argument("label_angle requested but the path is not part of a circle");
        const double arc_size = 360 * line_length / (2 * M_PI * arc_edges[0].radius());
        return _round_to_str(arc_size) + "\xC2\xB0";  // °
    }
    return _number_with_units(line_length, tolerance);
}

Location Draft::_sketch_location(const Mixin1D& path, double u_value, bool flip) {
    const double angle = path.tangent_angle_at(u_value) + (flip ? 180 : 0);
    return Location(path.position_at(u_value), Vector(0, 0, 1), angle);
}

// ---------------------------------------------------------------------------
// DimensionLine
// ---------------------------------------------------------------------------

namespace {

template <class Path>
Sketch dimension_line(const Path& path_obj, const Draft& draft, const DimensionLineOptions& o, double& dimension) {
    const double path_length = path_obj.length();
    dimension = path_length;

    // Generate the label
    const std::string label_str = draft._label_to_str(o.label, path_obj, o.label_angle, o.tolerance);
    TextOptions text_options;
    text_options.font = draft.font;
    text_options.font_style = draft.font_style;
    text_options.align = Align::CENTER;
    const Text label_shape(label_str, draft.font_size, text_options);
    const double label_length = label_shape.bounding_box().size.X;

    // Calculate the arrow shaft length for up to three types
    const int arrow_count = static_cast<int>(o.arrows.first) + static_cast<int>(o.arrows.second);
    if (arrow_count == 0) throw std::invalid_argument("No output - no arrows selected");
    double shaft_length = (path_length - label_length) / 2 - draft.pad_around_text;
    std::vector<Sketch> arrow_shapes;
    const auto add_arrow = [&](const auto& shaft, int i) {
        const bool flip_head = (shaft.position_at(i) != path_obj.position_at(i)) == (i == 1);
        arrow_shapes.push_back(Arrow(draft.arrow_length, shaft, draft.line_width, flip_head, draft.head_type));
    };
    if (label_length + arrow_count * draft.arrow_length < path_length && shaft_length > draft.arrow_length / 2) {
        const Path first = path_obj.trim(0.0, shaft_length / path_length);
        const Path second = path_obj.trim(1.0 - shaft_length / path_length, 1.0);
        add_arrow(first, 0);
        add_arrow(second, 1);
    } else {
        shaft_length = 2 * draft.arrow_length;
        const Edge first = Edge::make_line(path_obj.position_at(0),
                                           path_obj.position_at(0) - path_obj.tangent_at(0) * 2 * draft.arrow_length);
        const Edge second = Edge::make_line(path_obj.position_at(1) + path_obj.tangent_at(1) * 2 * draft.arrow_length,
                                            path_obj.position_at(1));
        add_arrow(first, 0);
        add_arrow(second, 1);
    }

    // The possible locations of the label; keep the best scoring one (the last of equals)
    const double overage = shaft_length + draft.pad_around_text + label_length / 2;
    const double label_u_values[] = {0.5, -overage / path_length, 1 + overage / path_length};
    std::optional<Sketch> best;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const double u_value : label_u_values) {
        std::vector<Shape> select_arrow_shapes;
        if (o.arrows.first) select_arrow_shapes.push_back(arrow_shapes[0]);
        if (o.arrows.second) select_arrow_shapes.push_back(arrow_shapes[1]);
        Sketch d_line(Compound(select_arrow_shapes).wrapped());
        const bool flip_label = path_obj.tangent_at(u_value).get_angle(Vector(1, 0, 0)) >= 180;
        const Location loc = Draft::_sketch_location(path_obj, u_value, flip_label);
        const Sketch placed_label = label_shape.located(loc);
        const double self_intersection_area = intersection_area(d_line, placed_label);
        d_line += placed_label;
        const double bbox_size = d_line.bounding_box().diagonal();

        // Minimize size while avoiding intersections
        double common_area = o.sketch ? intersection_area(d_line, *o.sketch) : 0.0;
        common_area += self_intersection_area;
        const double score = (d_line.area() - 10 * common_area) / bbox_size;
        if (!best || score >= best_score) {
            best = d_line;
            best_score = score;
        }
    }
    return *best;
}

}  // namespace

DimensionLine::DimensionLine(const PathDescriptor& path, const Draft& draft, const DimensionLineOptions& options) {
    const auto* points = std::get_if<std::vector<Vector>>(&path.value);
    if (points != nullptr && points->size() > 2) {
        throw std::invalid_argument("Only two points are allowed for dimension lines");
    }
    const std::variant<Edge, Wire> path_obj = Draft::_process_path(path);
    const Sketch sketch =
        std::visit([&](const auto& p) { return dimension_line(p, draft, options, dimension); }, path_obj);
    assign(sketch, 0, std::nullopt);
}

// ---------------------------------------------------------------------------
// ExtensionLine
// ---------------------------------------------------------------------------

namespace {

// How close to perpendicular to the dimension line a vector offset must be (1 degree)
const double kOffsetAngularDotTol = std::sin(1.0 * M_PI / 180.0);

Sketch extension_line(const PathDescriptor& border, std::optional<double> scalar_offset,
                      std::optional<Vector> offset_vector, const Draft& draft, const ExtensionLineOptions& o,
                      double& dimension) {
    // offset is a signed distance or an explicit displacement in the drawing plane
    if (scalar_offset) {
        if (*scalar_offset == 0) throw std::invalid_argument("A dimension line should be used if offset is 0");
    } else {
        if (offset_vector->length() < TOLERANCE) throw std::invalid_argument("offset vector must be non-zero");
        if (std::abs(offset_vector->Z) > TOLERANCE) {
            throw std::invalid_argument("offset vector must lie in the drawing (XY) plane");
        }
        offset_vector = Vector(offset_vector->X, offset_vector->Y);
    }

    // The path being measured
    const std::variant<Edge, Wire> object_to_measure = Draft::_process_path(border);
    const Mixin1D& measured = std::visit([](const auto& p) -> const Mixin1D& { return p; }, object_to_measure);
    if (measured.position_at(0) == measured.position_at(1)) {
        throw std::invalid_argument("Start and end points of border must be different.");
    }

    std::optional<Vector> measurement_direction;
    if (o.measurement_direction) {
        const Vector& direction = *o.measurement_direction;
        if (direction.length() < TOLERANCE) throw std::invalid_argument("measurement_direction must be non-zero");
        if (std::abs(direction.Z) > TOLERANCE) {
            throw std::invalid_argument("measurement_direction must lie in the drawing (XY) plane");
        }
        measurement_direction = Vector(direction.X, direction.Y);
    } else if (offset_vector) {
        // The dimension line runs perpendicular to the offset
        measurement_direction = offset_vector->cross(Vector(0, 0, 1));
    }

    std::variant<Edge, Wire> object_to_dimension = object_to_measure;
    if (measurement_direction) {
        const Vector measure_object_span = measured.position_at(1) - measured.position_at(0);
        const Vector extent_along_wire = measure_object_span.project_to_line(*measurement_direction);
        if (extent_along_wire.length() < TOLERANCE) {
            throw std::invalid_argument(
                "border has no extent along the measurement direction; the dimension would be degenerate");
        }
        object_to_dimension = Edge::make_line(measured.position_at(0), measured.position_at(0) + extent_along_wire);
    }
    const Mixin1D& to_dimension = std::visit([](const auto& p) -> const Mixin1D& { return p; }, object_to_dimension);

    double signed_offset = 0.0;
    if (scalar_offset) {
        signed_offset = *scalar_offset;
    } else {
        // The vector reduced to a signed distance: the side is its sign against the right normal
        const Vector path_direction = (to_dimension.position_at(1) - to_dimension.position_at(0)).normalized();
        const double along_line = offset_vector->normalized().dot(path_direction);
        if (std::abs(along_line) > kOffsetAngularDotTol) {
            throw std::invalid_argument(
                "offset vector must be perpendicular to the dimension line (when measurement_direction is given, "
                "perpendicular to it)");
        }
        const Vector right_normal = path_direction.cross(Vector(0, 0, 1));
        const double side_dot = offset_vector->dot(right_normal);
        signed_offset = std::copysign(offset_vector->length(), side_dot);
    }
    const double dimension_distance = signed_offset;
    const Side dimension_side = std::copysign(1.0, signed_offset) > 0 ? Side::RIGHT : Side::LEFT;

    const Wire dimension_path = to_dimension.offset_2d(dimension_distance, Kind::ARC, dimension_side, false);
    const std::string dimension_label_str =
        o.label ? *o.label : draft._label_to_str(o.label, to_dimension, o.label_angle, o.tolerance);

    std::vector<Edge> extension_lines;
    for (int e : {0, 1}) extension_lines.push_back(Edge::make_line(measured.position_at(e), dimension_path.position_at(e)));
    // If the dimension path was created backwards, flip the extension lines
    if (std::abs(extension_lines[0].length() - std::abs(dimension_distance)) > 1e-4) {
        extension_lines.clear();
        for (int e : {0, 1}) {
            extension_lines.push_back(Edge::make_line(measured.position_at(e), dimension_path.position_at(1 - e)));
        }
    }
    // Move the extension lines away from the object
    for (auto& extension_line : extension_lines) {
        extension_line = extension_line.moved(Location(extension_line.tangent_at(0) * draft.extension_gap));
    }

    // Build the extension line sketch
    std::vector<TopoDS_Shape> faces;
    for (const auto& extension_line : extension_lines) {
        const Edge line_pen = extension_line.perpendicular_line(draft.line_width, 0);
        for (const auto& f : sweep(line_pen, extension_line).faces()) faces.push_back(f.wrapped());
    }
    const DimensionLine d_line(dimension_path, draft,
                               {.sketch = o.sketch, .label = dimension_label_str, .arrows = o.arrows,
                                .tolerance = o.tolerance, .label_angle = o.label_angle});
    dimension = d_line.dimension;
    for (const auto& f : d_line.faces()) faces.push_back(f.wrapped());
    return Sketch(make_compound(faces));
}

}  // namespace

ExtensionLine::ExtensionLine(const PathDescriptor& border, double offset, const Draft& draft,
                             const ExtensionLineOptions& options) {
    assign(extension_line(border, offset, std::nullopt, draft, options, dimension), 0, std::nullopt);
}

ExtensionLine::ExtensionLine(const PathDescriptor& border, const Vector& offset, const Draft& draft,
                             const ExtensionLineOptions& options) {
    assign(extension_line(border, std::nullopt, offset, draft, options, dimension), 0, std::nullopt);
}

// ---------------------------------------------------------------------------
// Date, TechnicalDrawing
// ---------------------------------------------------------------------------

Date::Date(int year_, int month_, int day_) : year(year_), month(month_), day(day_) {
    if (year < 1 || year > 9999) throw std::invalid_argument("year " + std::to_string(year) + " is out of range");
    if (month < 1 || month > 12) throw std::invalid_argument("month must be in 1..12");
    static const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
    const int in_month = days[month - 1] + (month == 2 && leap ? 1 : 0);
    if (day < 1 || day > in_month) throw std::invalid_argument("day is out of range for month");
}

Date Date::today() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    return Date(local.tm_year + 1900, local.tm_mon + 1, local.tm_mday);
}

std::string Date::isoformat() const {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d", year, month, day);
    return buffer;
}

const std::map<PageSize, std::pair<double, double>> TechnicalDrawing::page_sizes = {
    {PageSize::A0, {1189 * MM, 841 * MM}},   {PageSize::A1, {841 * MM, 594 * MM}},
    {PageSize::A2, {594 * MM, 420 * MM}},    {PageSize::A3, {420 * MM, 297 * MM}},
    {PageSize::A4, {297 * MM, 210 * MM}},    {PageSize::A5, {210 * MM, 148.5 * MM}},
    {PageSize::A6, {148.5 * MM, 105 * MM}},  {PageSize::A7, {105 * MM, 74 * MM}},
    {PageSize::A8, {74 * MM, 52 * MM}},      {PageSize::A9, {52 * MM, 37 * MM}},
    {PageSize::A10, {37 * MM, 26 * MM}},     {PageSize::LETTER, {11 * IN, 8.5 * IN}},
    {PageSize::LEGAL, {14 * IN, 8.5 * IN}},  {PageSize::LEDGER, {17 * IN, 11 * IN}}};

namespace {

/// Sketch(Compound.make_text(txt, size, align=...).wrapped)
Sketch text_sketch(const std::string& txt, double size, std::optional<Align2> align = std::nullopt) {
    return Sketch(Compound::make_text(txt, size, "Arial", "", FontStyle::REGULAR, {TextAlign::CENTER, TextAlign::CENTER},
                                      align)
                      .wrapped());
}

Sketch technical_drawing(const TechnicalDrawingOptions& o) {
    const Date design_date = o.design_date ? *o.design_date : Date::today();
    const double nominal_text_size = o.nominal_text_size;
    const auto page_dim = TechnicalDrawing::page_sizes.at(o.page_size);

    // Frame
    const double frame_width = page_dim.first - 2 * TechnicalDrawing::margin - 2 * nominal_text_size;
    const double frame_height = 2 * frame_width / 3;
    const Wire frame_wire = Wire::make_polygon({{-frame_width / 2, frame_height / 2},
                                                {frame_width / 2, frame_height / 2},
                                                {frame_width / 2, -frame_height / 2},
                                                {-frame_width / 2, -frame_height / 2}});
    const Sketch frame = trace(frame_wire, o.line_width);

    // Ticks
    std::vector<Edge> tick_lines;
    for (int i = 0; i < 20; ++i) {
        if (i == 0 || i == 6 || i == 10 || i == 16) continue;  // corners
        const double u_value = i / 20.0;
        const Vector pos = frame_wire.position_at(u_value);
        tick_lines.push_back(Edge::make_line(
            pos, pos + Vector(nominal_text_size, 0).rotate(Axis::Z, frame_wire.tangent_angle_at(u_value) + 90)));
    }
    const Sketch ticks = trace(tick_lines, o.line_width);

    // Numbers
    Sketch grid_labels;
    const double y_centers[] = {-3.0 / 8, -1.0 / 8, 1.0 / 8, 3.0 / 8};
    for (int label = 0; label < 4; ++label) {
        for (const double x_index : {-0.5, 0.5}) {
            grid_labels += Pos(x_index * (frame_width + 1.5 * nominal_text_size), y_centers[label] * frame_height) *
                           text_sketch(std::to_string(label + 1), nominal_text_size);
        }
    }
    // Letters
    const double x_centers[] = {-5.0 / 12, -3.0 / 12, -1.0 / 12, 1.0 / 12, 3.0 / 12, 5.0 / 12};
    const char* letters[] = {"F", "E", "D", "C", "B", "A"};
    for (int i = 0; i < 6; ++i) {
        for (const double y_index : {-0.5, 0.5}) {
            grid_labels += Pos(x_centers[i] * frame_width, y_index * (frame_height + 1.5 * nominal_text_size)) *
                           text_sketch(letters[i], nominal_text_size);
        }
    }

    // Text Box Frame
    const Vector bf_pnt1 = frame_wire.edges().sort_by(Axis::Y)[0].position_at(0.5);
    const Vector bf_pnt2 = frame_wire.edges().sort_by(Axis::X)[-1].position_at(0.75);
    const Wire box_frame_wire = Wire::make_polygon({bf_pnt1, {bf_pnt1.X, bf_pnt2.Y}, bf_pnt2}, false);
    const Vector bf_pnt3 = box_frame_wire.edges().sort_by(Axis::X)[0].position_at(1.0 / 3);
    const Vector bf_pnt4 = box_frame_wire.edges().sort_by(Axis::X)[0].position_at(2.0 / 3);
    Curve box_frame_curve =
        Curve() + std::vector<Shape>{box_frame_wire, Edge::make_line(bf_pnt3, {bf_pnt2.X, bf_pnt3.Y})};
    box_frame_curve += Edge::make_line(bf_pnt4, {bf_pnt2.X, bf_pnt4.Y});
    const Vector bf_pnt5 = box_frame_curve.edges().sort_by(Axis::Y)[-1].position_at(1.0 / 3);
    const Vector bf_pnt6 = box_frame_curve.edges().sort_by(Axis::Y)[-1].position_at(2.0 / 3);
    box_frame_curve += Edge::make_line(bf_pnt5, {bf_pnt5.X, bf_pnt1.Y});
    const Vector start(bf_pnt6.X, bf_pnt1.Y);
    box_frame_curve += Edge::make_line(start, start + Vector(0, (bf_pnt2.Y - bf_pnt1.Y) / 3));
    const Sketch box_frame = trace(box_frame_curve, o.line_width);

    // Text
    Sketch labels;
    const Align2 min_center(Align::MIN, Align::CENTER);
    const Edge t_base_line1 =
        Edge::make_line(bf_pnt1, {bf_pnt1.X, bf_pnt2.Y}).moved(Location(Vector(nominal_text_size / 5, 0)));
    const Edge t_base_line2 = t_base_line1.moved(Location(Vector(frame_width / 6, 0)));
    const Edge t_base_line3 = t_base_line1.moved(Location(Vector(2 * frame_width / 6, 0)));
    const auto place = [&](const Edge& base, double u, const std::string& txt, double size) {
        labels += Pos(base.position_at(u)) * text_sketch(txt, size, min_center);
    };
    place(t_base_line1, 11.0 / 12, "DESIGNED BY:", nominal_text_size / 3);
    place(t_base_line1, 9.0 / 12, o.designed_by, nominal_text_size / 2);
    place(t_base_line1, 7.0 / 12, "DATE:", nominal_text_size / 3);
    place(t_base_line1, 5.0 / 12, design_date.isoformat(), nominal_text_size / 2);
    place(t_base_line1, 3.0 / 12, "SCALE:", nominal_text_size / 3);
    place(t_base_line1, 1.0 / 12, "1:" + py_float_repr(o.drawing_scale), nominal_text_size / 2);
    place(t_base_line2, 10.0 / 12, o.title, nominal_text_size);
    place(t_base_line2, 6.0 / 12, o.sub_title, nominal_text_size);
    place(t_base_line2, 3.0 / 12, "DRAWING NUMBER:", nominal_text_size / 3);
    place(t_base_line2, 1.0 / 12, o.drawing_number, nominal_text_size / 2);
    place(t_base_line3, 3.0 / 12, "SHEET:", nominal_text_size / 3);
    if (o.sheet_number) place(t_base_line3, 1.0 / 12, std::to_string(*o.sheet_number), nominal_text_size / 2);

    std::vector<TopoDS_Shape> faces;
    for (const Sketch* part : std::vector<const Sketch*>{&frame, &ticks, &grid_labels, &box_frame, &labels}) {
        for (const auto& f : part->faces()) faces.push_back(f.wrapped());
    }
    return Sketch(make_compound(faces));
}

}  // namespace

TechnicalDrawing::TechnicalDrawing(const TechnicalDrawingOptions& options)
    : BaseSketchObject(technical_drawing(options), 0, std::nullopt) {}

}  // namespace build123d
