// build123d-cpp: 2D drawing export to SVG and DXF, and hidden-line drawings. Mirrors
// exporters.py. The files are written without ezdxf or svgpathtools: the SVG is what Python
// writes, byte for byte, and the DXF has the same document structure, layers, line types
// and entities as ezdxf's.
//
//   ExportSVG svg({.unit = Unit::MM, .line_weight = 0.5});
//   svg.add_layer("hidden", {.line_color = Color("gray"), .line_type = LineType::ISO_DOT});
//   svg.add_shape(visible);
//   svg.add_shape(hidden, "hidden");
//   svg.write("drawing.svg");
#pragma once

#include <cstdint>
#include <iosfwd>
#include <map>
#include <set>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "build123d/geometry.hpp"
#include "build123d/topology.hpp"

namespace build123d {

/// Line types (exporters.py LineType); value() gives Python's .value ("CONTINUOUS",
/// "ACAD_ISO02W100", ...).
enum class LineType {
    CONTINUOUS,
    BORDER, BORDER2, BORDERX2,
    CENTER, CENTER2, CENTERX2,
    DASHDOT, DASHDOT2, DASHDOTX2,
    DASHED, DASHED2, DASHEDX2,
    DIVIDE, DIVIDE2, DIVIDEX2,
    DOT, DOT2, DOTX2,
    HIDDEN, HIDDEN2, HIDDENX2,
    PHANTOM, PHANTOM2, PHANTOMX2,
    ISO_DASH, ISO_DASH_SPACE, ISO_LONG_DASH_DOT, ISO_LONG_DASH_DOUBLE_DOT, ISO_LONG_DASH_TRIPLE_DOT,
    ISO_DOT, ISO_LONG_DASH_SHORT_DASH, ISO_LONG_DASH_DOUBLE_SHORT_DASH, ISO_DASH_DOT,
    ISO_DOUBLE_DASH_DOT, ISO_DASH_DOUBLE_DOT, ISO_DOUBLE_DASH_DOUBLE_DOT, ISO_DASH_TRIPLE_DOT,
    ISO_DOUBLE_DASH_TRIPLE_DOT
};
/// The DXF line type name (Python's LineType.value).
std::string value(LineType line_type);
/// "LineType.DASHED"
std::string to_string(LineType line_type);

/// AutoCAD color index of the DXF exporter (exporters.py ColorIndex).
enum class ColorIndex { RED = 1, YELLOW = 2, GREEN = 3, CYAN = 4, BLUE = 5, MAGENTA = 6, BLACK = 7, GRAY = 8, LIGHT_GRAY = 9 };
std::string to_string(ColorIndex color);

/// Width of the dots of SVG line types, in tenths of an inch (exporters.py DotLength).
enum class DotLength { TRUE_DOT, INKSCAPE_COMPAT, QCAD_IMPERIAL };
/// 0.0, 0.01 or 0.2 (Python's DotLength.value).
double value(DotLength dot_length);

/// An ANSI line pattern for ezdxf: inches in, tenths of an inch out, total length first.
std::vector<double> ansi_pattern(const std::vector<double>& pattern);
/// An ISO line pattern for ezdxf: millimeters in, tenths of an inch out, total length first.
std::vector<double> iso_pattern(const std::vector<double>& pattern);
/// The factor converting lengths in from_unit to to_unit.
double unit_conversion_scale(Unit from_unit, Unit to_unit);

/// A hidden-line drawing of a shape (exporters.py Drawing): the visible and hidden edges,
/// projected onto the plane through `look_at` (default: the shape's center) whose normal
/// points to `look_from`.
struct DrawingOptions {
    std::optional<Vector> look_at = std::nullopt;
    Vector look_from = Vector(1, -1, 1);
    Vector look_up = Vector(0, 0, 1);
    bool with_hidden = true;
    std::optional<double> focus = std::nullopt;
};

class Drawing {
public:
    using Options = DrawingOptions;
    explicit Drawing(const Shape& shape, const Options& options = {});
    Drawing(const Shape& shape, const std::optional<Vector>& look_at, const Vector& look_from = Vector(1, -1, 1),
            const Vector& look_up = Vector(0, 0, 1), bool with_hidden = true,
            std::optional<double> focus = std::nullopt);

    Compound visible_lines;
    Compound hidden_lines;
};

/// Base class of the 2D exporters (exporters.py Export2D).
class Export2D {
public:
    /// Parameters closer than this are equal for OCCT (spline segments).
    static constexpr double PARAMETRIC_TOLERANCE = 1e-9;
    static constexpr ColorIndex DEFAULT_COLOR_INDEX = ColorIndex::BLACK;
    static constexpr double DEFAULT_LINE_WEIGHT = 0.09;
    static constexpr LineType DEFAULT_LINE_TYPE = LineType::CONTINUOUS;

    /// Line type definitions: value() name -> (description, pattern in tenths of an inch,
    /// total length first).
    static const std::map<std::string, std::pair<std::string, std::vector<double>>>& LINETYPE_DEFS();
    /// Scale factor from line type units (1/10 inch) to the unit (IN, FT, MM, CM, M).
    static double LTYPE_SCALE(Unit unit);

    virtual ~Export2D() = default;
};

// ---------------------------------------------------------------------------
// SVG
// ---------------------------------------------------------------------------

/// The fill and line colors of ExportSVG and its layers: INHERIT_COLOR (the default: Face
/// colors fill faces and Edge/Wire colors draw lines, falling back to no fill and black
/// lines), std::nullopt (none, Python's None), a color, or an (r, g, b[, a]) tuple. A tuple
/// with a component above 1 (Python's deprecated 0-255 tuples) and a ColorIndex (deprecated)
/// are converted with a warning, as in Python.
class SvgColor {
public:
    SvgColor() = default;
    SvgColor(std::nullopt_t) : inherit_(false) {}  // NOLINT
    SvgColor(const Color& color) : inherit_(false), color_(color) {}  // NOLINT
    SvgColor(const std::string& name) : SvgColor(Color(name)) {}  // NOLINT
    SvgColor(const char* name) : SvgColor(Color(name)) {}  // NOLINT
    SvgColor(ColorIndex index) : inherit_(false), index_(index) {}  // NOLINT
    SvgColor(const std::array<double, 3>& rgb) : inherit_(false), tuple_({rgb[0], rgb[1], rgb[2], 1.0}) {}  // NOLINT
    SvgColor(const std::array<double, 4>& rgba) : inherit_(false), tuple_(rgba), tuple_alpha_(true) {}  // NOLINT

    bool inherit() const { return inherit_; }
    /// The color (none for std::nullopt), after converting deprecated inputs.
    std::optional<Color> resolve() const;

private:
    bool inherit_ = true;
    std::optional<Color> color_;
    std::optional<ColorIndex> index_;
    std::optional<std::array<double, 4>> tuple_;
    bool tuple_alpha_ = false;
};

/// Use shape colors (the default of ExportSVG's color arguments).
inline const SvgColor INHERIT_COLOR{};

/// Python's `DotLength | float` argument.
class DotLengthArg {
public:
    DotLengthArg(DotLength d) : value_(build123d::value(d)) {}  // NOLINT
    DotLengthArg(double v) : value_(v) {}  // NOLINT
    double value() const { return value_; }

private:
    double value_;
};

/// The keyword arguments of ExportSVG.
struct ExportSVGOptions {
    Unit unit = Unit::MM;
    double scale = 1;
    double margin = 0;
    bool fit_to_stroke = true;
    int precision = 6;
    SvgColor fill_color = INHERIT_COLOR;
    SvgColor line_color = INHERIT_COLOR;
    /// In millimeters.
    double line_weight = 0.09;
    LineType line_type = LineType::CONTINUOUS;
    DotLengthArg dot_length = DotLength::INKSCAPE_COMPAT;
};

/// The keyword arguments of ExportSVG::add_layer.
struct SvgLayerOptions {
    SvgColor fill_color = INHERIT_COLOR;
    SvgColor line_color = INHERIT_COLOR;
    /// In millimeters.
    double line_weight = 0.09;
    LineType line_type = LineType::CONTINUOUS;
};

/// SVG export (exporters.py ExportSVG). Units MM, CM or IN.
class ExportSVG : public Export2D {
public:
    using Options = ExportSVGOptions;
    using LayerOptions = SvgLayerOptions;

    /// Raises std::invalid_argument for a unit other than MM, CM or IN.
    explicit ExportSVG(const Options& options = {});
    ExportSVG(Unit unit, double scale = 1, double margin = 0, bool fit_to_stroke = true, int precision = 6,
              const SvgColor& fill_color = INHERIT_COLOR, const SvgColor& line_color = INHERIT_COLOR,
              double line_weight = DEFAULT_LINE_WEIGHT, LineType line_type = DEFAULT_LINE_TYPE,
              DotLengthArg dot_length = DotLength::INKSCAPE_COMPAT);
    ~ExportSVG() override;
    ExportSVG(ExportSVG&&) noexcept;
    ExportSVG& operator=(ExportSVG&&) noexcept;

    /// Raises std::invalid_argument for a duplicate layer name.
    ExportSVG& add_layer(const std::string& name, const LayerOptions& options = {});
    /// Add a shape (its faces, loose wires and loose edges) to a layer ("" is the default
    /// layer). Raises std::invalid_argument for an undefined layer.
    void add_shape(const Shape& shape, const std::string& layer = "", bool reverse_wires = false);
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    void add_shape(const std::vector<S>& shapes, const std::string& layer = "", bool reverse_wires = false) {
        check_layer(layer);
        for (const auto& s : shapes) add_shape(s, layer, reverse_wires);
    }
    /// Raises std::invalid_argument if no shape was added.
    void write(const std::string& path) const;
    void write(std::ostream& stream) const;
    /// The SVG document as text.
    std::string to_string() const;

    Unit unit;
    double scale;
    double margin;
    bool fit_to_stroke;
    int precision;
    double dot_length;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void check_layer(const std::string& layer) const;
};

// ---------------------------------------------------------------------------
// DXF
// ---------------------------------------------------------------------------

/// The keyword arguments of ExportDXF::add_layer.
struct DxfLayerOptions {
    std::optional<ColorIndex> color = std::nullopt;
    /// In millimeters.
    std::optional<double> line_weight = std::nullopt;
    std::optional<LineType> line_type = std::nullopt;
};

/// DXF export (exporters.py ExportDXF), written without ezdxf.
class ExportDXF : public Export2D {
public:
    /// DXF versions (ezdxf's constants).
    static constexpr const char* DXF12 = "AC1009";
    static constexpr const char* DXF2000 = "AC1015";
    static constexpr const char* DXF2004 = "AC1018";
    static constexpr const char* DXF2007 = "AC1021";
    static constexpr const char* DXF2010 = "AC1024";
    static constexpr const char* DXF2013 = "AC1027";
    static constexpr const char* DXF2018 = "AC1032";
    /// The units ExportDXF writes as metric.
    static inline const std::set<Unit> METRIC_UNITS{Unit::MM, Unit::CM, Unit::M};

    using LayerOptions = DxfLayerOptions;

    /// `version` is one of the constants above (or "R12", "R2000", ...). `color`,
    /// `line_weight` and `line_type` change the default layer "0". Raises
    /// std::invalid_argument for an unsupported unit (MC, MM, CM, M, IN, FT are).
    explicit ExportDXF(const std::string& version = DXF2013, Unit unit = Unit::MM,
                       std::optional<ColorIndex> color = std::nullopt,
                       std::optional<double> line_weight = std::nullopt,
                       std::optional<LineType> line_type = std::nullopt);
    ~ExportDXF() override;
    ExportDXF(ExportDXF&&) noexcept;
    ExportDXF& operator=(ExportDXF&&) noexcept;

    /// Raises std::invalid_argument for a duplicate layer name.
    ExportDXF& add_layer(const std::string& name, const LayerOptions& options = {});
    /// Add the edges of a shape to a layer ("" is the default layer "0").
    ExportDXF& add_shape(const Shape& shape, const std::string& layer = "");
    template <class S, std::enable_if_t<std::is_base_of_v<Shape, S>, int> = 0>
    ExportDXF& add_shape(const std::vector<S>& shapes, const std::string& layer = "") {
        for (const auto& s : shapes) add_single_shape(s, layer);
        warn_non_planar();
        return *this;
    }
    /// ASCII (default) or binary DXF.
    void write(const std::string& file_name, bool ascii_format = true);
    void write(std::ostream& stream, bool ascii_format = true);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void add_single_shape(const Shape& shape, const std::string& layer);
    void warn_non_planar();
};

}  // namespace build123d
