// build123d-cpp: drafting objects (build123d's drafting.py): arrows, dimension and extension
// lines, and the frame and title block of a technical drawing, as Sketches on Plane.XY.
#pragma once

#include <initializer_list>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "build123d/objects.hpp"

namespace build123d {

/// Sketch Object: ArrowHead, with its tip at the origin pointing along +X before `rotation`
/// (degrees).
class ArrowHead : public BaseSketchObject {
public:
    explicit ArrowHead(double size, HeadType head_type = HeadType::CURVED, double rotation = 0);
};

/// Sketch Object: Arrow with a shaft `shaft_width` wide along `shaft_path`, the head at the
/// start (or the end) of the path.
class Arrow : public BaseSketchObject {
public:
    Arrow(double arrow_size, const Edge& shaft_path, double shaft_width, bool head_at_start = true,
          HeadType head_type = HeadType::CURVED);
    Arrow(double arrow_size, const Wire& shaft_path, double shaft_width, bool head_at_start = true,
          HeadType head_type = HeadType::CURVED);
};

/// The path of a dimension (Python's PathDescriptor): an Edge, a Wire or points (two points
/// give a line, more an open polyline).
class PathDescriptor {
public:
    PathDescriptor(const Edge& edge) : value(edge) {}                     // NOLINT
    PathDescriptor(const Wire& wire) : value(wire) {}                     // NOLINT
    PathDescriptor(const std::vector<Vector>& points) : value(points) {}  // NOLINT
    PathDescriptor(std::initializer_list<Vector> points) : value(std::vector<Vector>(points)) {}
    PathDescriptor(const std::vector<Vertex>& points);  // NOLINT

    std::variant<Edge, Wire, std::vector<Vector>> value;
};

/// A dimension's tolerance (Python's float | tuple[float, float] | None): none, ±value
/// (`0.1`) or +plus -minus (`{0.1, 0.2}`).
class DimensionTolerance {
public:
    DimensionTolerance() = default;
    DimensionTolerance(double value) : value_(value) {}  // NOLINT
    DimensionTolerance(double plus, double minus) : value_(std::make_pair(plus, minus)) {}

    bool has_value() const { return value_.index() != 0; }
    /// The ± tolerance, if it is one.
    std::optional<double> symmetric() const;
    /// The (+, -) pair, if it is one.
    std::optional<std::pair<double, double>> plus_minus() const;

private:
    std::variant<std::monostate, double, std::pair<double, double>> value_;
};

/// Settings of the drafting objects (the fields of Python's Draft dataclass).
struct DraftOptions {
    /// Size of the text in dimension lines and callouts.
    double font_size = 5.0;
    std::string font = "Arial";
    FontStyle font_style = FontStyle::REGULAR;
    HeadType head_type = HeadType::CURVED;
    double arrow_length = 3.0;
    /// Thickness of all lines.
    double line_width = 0.5;
    double pad_around_text = 2.0;
    Unit unit = Unit::MM;
    NumberDisplay number_display = NumberDisplay::DECIMAL;
    bool display_units = true;
    int decimal_precision = 2;
    /// Largest fraction denominator; a power of 2.
    int fractional_precision = 64;
    /// Gap between the point measured and the start of an extension line.
    double extension_gap = 2.0;
};

/// Draft: the settings for documenting designs with dimension and extension lines
/// (Python's Draft): Draft draft({.font_size = 3, .unit = Unit::IN}).
class Draft : public DraftOptions {
public:
    /// Python's unit_LUT: "mm" for metric units, '"' otherwise.
    static const std::map<bool, std::string> unit_LUT;

    /// Throws std::invalid_argument if fractional_precision isn't a power of 2.
    Draft(const DraftOptions& options = {});  // NOLINT: a DraftOptions converts to a Draft

    /// Are metric units being used.
    bool is_metric() const;

    // The helpers the drafting objects use (Python's "private" methods)
    /// A number rounded to decimal_precision digits, as a string.
    std::string _round_to_str(double number) const;
    /// A length as a measurement string with units and tolerance.
    std::string _number_with_units(double number, const DimensionTolerance& tolerance = {},
                                   std::optional<bool> display_units = std::nullopt) const;
    /// The Edge or Wire of a path.
    static std::variant<Edge, Wire> _process_path(const PathDescriptor& path);
    /// The label of a dimension of `line_wire`: `label`, the angle of its arc or its length.
    std::string _label_to_str(const std::optional<std::string>& label, const Mixin1D& line_wire, bool label_angle,
                              const DimensionTolerance& tolerance) const;
    /// Where to place an object at `u_value` along a path on Plane.XY (turned by 180° with flip).
    static Location _sketch_location(const Mixin1D& path, double u_value, bool flip = false);
};

/// Options of DimensionLine (Python's keyword arguments).
struct DimensionLineOptions {
    /// The sketch being dimensioned, so that the label avoids it.
    std::optional<Sketch> sketch;
    /// Text replacing the measured length (or angle).
    std::optional<std::string> label;
    /// Arrows at the start and the end.
    std::pair<bool, bool> arrows = {true, true};
    DimensionTolerance tolerance;
    /// Show the angle of the arc of the path instead of its length.
    bool label_angle = false;
};

/// Sketch Object: DimensionLine, arrows on either side of a measurement label along `path`
/// (typically for inside dimensions). The label is placed where it fits best.
class DimensionLine : public BaseSketchObject {
public:
    DimensionLine(const PathDescriptor& path, const Draft& draft, const DimensionLineOptions& options = {});
    /// The length of the dimension.
    double dimension = 0.0;
};

/// Options of ExtensionLine (Python's keyword arguments).
struct ExtensionLineOptions {
    std::optional<Sketch> sketch;
    std::optional<std::string> label;
    std::pair<bool, bool> arrows = {true, true};
    DimensionTolerance tolerance;
    bool label_angle = false;
    /// Direction to measure along (the border's start is the start of the dimension).
    std::optional<Vector> measurement_direction;
};

/// Sketch Object: ExtensionLine, a dimension line `offset` from `border` with two lines
/// extending to it (typically for outside dimensions). `offset` is a signed distance (the sign
/// selects the side) or a displacement perpendicular to the dimension line.
class ExtensionLine : public BaseSketchObject {
public:
    ExtensionLine(const PathDescriptor& border, double offset, const Draft& draft,
                  const ExtensionLineOptions& options = {});
    ExtensionLine(const PathDescriptor& border, const Vector& offset, const Draft& draft,
                  const ExtensionLineOptions& options = {});
    /// The length of the dimension.
    double dimension = 0.0;
};

/// A calendar date (Python's datetime.date).
struct Date {
    int year = 1;
    int month = 1;
    int day = 1;

    /// Throws std::invalid_argument for a date that doesn't exist, as Python does.
    Date(int year, int month, int day);
    /// Today's (local) date.
    static Date today();
    /// "YYYY-MM-DD".
    std::string isoformat() const;
    bool operator==(const Date& o) const { return year == o.year && month == o.month && day == o.day; }
};

/// Options of TechnicalDrawing (Python's keyword arguments).
struct TechnicalDrawingOptions {
    std::string designed_by = "build123d";
    /// Defaults to today.
    std::optional<Date> design_date;
    PageSize page_size = PageSize::A4;
    std::string title = "Title";
    std::string sub_title = "Sub Title";
    std::string drawing_number = "B3D-1";
    std::optional<int> sheet_number;
    /// Shown as 1:value.
    double drawing_scale = 1.0;
    /// Size of the title text.
    double nominal_text_size = 10.0;
    double line_width = 0.5;
};

/// Sketch Object: TechnicalDrawing, the frame of a drawing page with its grid references and
/// title block.
class TechnicalDrawing : public BaseSketchObject {
public:
    explicit TechnicalDrawing(const TechnicalDrawingOptions& options = {});
    /// Page (width, height) of each PageSize.
    static const std::map<PageSize, std::pair<double, double>> page_sizes;
    static constexpr double margin = 5 * MM;
};

}  // namespace build123d
