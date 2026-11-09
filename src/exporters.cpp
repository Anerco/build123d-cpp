// Shared parts of the 2D exporters (exporters.py): line types, patterns, Drawing.
#include "build123d/exporters.hpp"

#include <cmath>
#include <stdexcept>

#include <BRepLib.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <gp_Ax2.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

struct LineTypeInfo {
    LineType type;
    const char* name;
    const char* value;
};

const LineTypeInfo kLineTypes[] = {
    {LineType::CONTINUOUS, "CONTINUOUS", "CONTINUOUS"},
    {LineType::BORDER, "BORDER", "BORDER"},
    {LineType::BORDER2, "BORDER2", "BORDER2"},
    {LineType::BORDERX2, "BORDERX2", "BORDERX2"},
    {LineType::CENTER, "CENTER", "CENTER"},
    {LineType::CENTER2, "CENTER2", "CENTER2"},
    {LineType::CENTERX2, "CENTERX2", "CENTERX2"},
    {LineType::DASHDOT, "DASHDOT", "DASHDOT"},
    {LineType::DASHDOT2, "DASHDOT2", "DASHDOT2"},
    {LineType::DASHDOTX2, "DASHDOTX2", "DASHDOTX2"},
    {LineType::DASHED, "DASHED", "DASHED"},
    {LineType::DASHED2, "DASHED2", "DASHED2"},
    {LineType::DASHEDX2, "DASHEDX2", "DASHEDX2"},
    {LineType::DIVIDE, "DIVIDE", "DIVIDE"},
    {LineType::DIVIDE2, "DIVIDE2", "DIVIDE2"},
    {LineType::DIVIDEX2, "DIVIDEX2", "DIVIDEX2"},
    {LineType::DOT, "DOT", "DOT"},
    {LineType::DOT2, "DOT2", "DOT2"},
    {LineType::DOTX2, "DOTX2", "DOTX2"},
    {LineType::HIDDEN, "HIDDEN", "HIDDEN"},
    {LineType::HIDDEN2, "HIDDEN2", "HIDDEN2"},
    {LineType::HIDDENX2, "HIDDENX2", "HIDDENX2"},
    {LineType::PHANTOM, "PHANTOM", "PHANTOM"},
    {LineType::PHANTOM2, "PHANTOM2", "PHANTOM2"},
    {LineType::PHANTOMX2, "PHANTOMX2", "PHANTOMX2"},
    {LineType::ISO_DASH, "ISO_DASH", "ACAD_ISO02W100"},
    {LineType::ISO_DASH_SPACE, "ISO_DASH_SPACE", "ACAD_ISO03W100"},
    {LineType::ISO_LONG_DASH_DOT, "ISO_LONG_DASH_DOT", "ACAD_ISO04W100"},
    {LineType::ISO_LONG_DASH_DOUBLE_DOT, "ISO_LONG_DASH_DOUBLE_DOT", "ACAD_ISO05W100"},
    {LineType::ISO_LONG_DASH_TRIPLE_DOT, "ISO_LONG_DASH_TRIPLE_DOT", "ACAD_ISO06W100"},
    {LineType::ISO_DOT, "ISO_DOT", "ACAD_ISO07W100"},
    {LineType::ISO_LONG_DASH_SHORT_DASH, "ISO_LONG_DASH_SHORT_DASH", "ACAD_ISO08W100"},
    {LineType::ISO_LONG_DASH_DOUBLE_SHORT_DASH, "ISO_LONG_DASH_DOUBLE_SHORT_DASH", "ACAD_ISO09W100"},
    {LineType::ISO_DASH_DOT, "ISO_DASH_DOT", "ACAD_ISO10W100"},
    {LineType::ISO_DOUBLE_DASH_DOT, "ISO_DOUBLE_DASH_DOT", "ACAD_ISO11W100"},
    {LineType::ISO_DASH_DOUBLE_DOT, "ISO_DASH_DOUBLE_DOT", "ACAD_ISO12W100"},
    {LineType::ISO_DOUBLE_DASH_DOUBLE_DOT, "ISO_DOUBLE_DASH_DOUBLE_DOT", "ACAD_ISO13W100"},
    {LineType::ISO_DASH_TRIPLE_DOT, "ISO_DASH_TRIPLE_DOT", "ACAD_ISO14W100"},
    {LineType::ISO_DOUBLE_DASH_TRIPLE_DOT, "ISO_DOUBLE_DASH_TRIPLE_DOT", "ACAD_ISO15W100"},
};

const LineTypeInfo& info(LineType t) {
    for (const auto& i : kLineTypes) {
        if (i.type == t) return i;
    }
    throw std::invalid_argument("Unknown line type");
}

/// Python 3.12's sum() of floats (Neumaier's compensated summation).
double py_sum(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double f_result = values[0];
    double c = 0.0;
    for (std::size_t i = 1; i < values.size(); ++i) {
        const double x = values[i];
        const double t = f_result + x;
        if (std::fabs(f_result) >= std::fabs(x)) {
            c += (f_result - t) + x;
        } else {
            c += (x - t) + f_result;
        }
        f_result = t;
    }
    if (c != 0.0 && std::isfinite(c)) f_result += c;
    return f_result;
}

}  // namespace

std::string value(LineType line_type) { return info(line_type).value; }

std::string to_string(LineType line_type) { return std::string("LineType.") + info(line_type).name; }

std::string to_string(ColorIndex color) {
    switch (color) {
        case ColorIndex::RED: return "ColorIndex.RED";
        case ColorIndex::YELLOW: return "ColorIndex.YELLOW";
        case ColorIndex::GREEN: return "ColorIndex.GREEN";
        case ColorIndex::CYAN: return "ColorIndex.CYAN";
        case ColorIndex::BLUE: return "ColorIndex.BLUE";
        case ColorIndex::MAGENTA: return "ColorIndex.MAGENTA";
        case ColorIndex::BLACK: return "ColorIndex.BLACK";
        case ColorIndex::GRAY: return "ColorIndex.GRAY";
        case ColorIndex::LIGHT_GRAY: return "ColorIndex.LIGHT_GRAY";
    }
    return "ColorIndex.?";
}

double value(DotLength dot_length) {
    switch (dot_length) {
        case DotLength::TRUE_DOT: return 0.0;
        case DotLength::INKSCAPE_COMPAT: return 0.01;
        case DotLength::QCAD_IMPERIAL: return 0.2;
    }
    return 0.0;
}

std::vector<double> ansi_pattern(const std::vector<double>& pattern) {
    std::vector<double> abs_args;
    for (double l : pattern) abs_args.push_back(std::fabs(l));
    std::vector<double> result{py_sum(abs_args) * 10};
    for (double l : pattern) result.push_back(l * 10);
    return result;
}

std::vector<double> iso_pattern(const std::vector<double>& pattern) {
    std::vector<double> abs_args;
    for (double l : pattern) abs_args.push_back(std::fabs(l));
    std::vector<double> result{py_sum(abs_args) / 2.54};
    for (double l : pattern) result.push_back(l / 2.54);
    return result;
}

double unit_conversion_scale(Unit from_unit, Unit to_unit) {
    return units_per_meter(to_unit) / units_per_meter(from_unit);
}

const std::map<std::string, std::pair<std::string, std::vector<double>>>& Export2D::LINETYPE_DEFS() {
    static const std::map<std::string, std::pair<std::string, std::vector<double>>> defs = {
        {"CONTINUOUS", {"Solid", {0.0}}},
        {"BORDER", {"Border __ __ . __ __ . __ __ . __ __ . __ __ .", ansi_pattern({0.5, -0.25, 0.5, -0.25, 0, -0.25})}},
        {"BORDER2", {"Border (.5x) __.__.__.__.__.__.__.__.__.__.__.",
                     ansi_pattern({0.25, -0.125, 0.25, -0.125, 0, -0.125})}},
        {"BORDERX2", {"Border (2x) ____  ____  .  ____  ____  .  ___", ansi_pattern({1.0, -0.5, 1.0, -0.5, 0, -0.5})}},
        {"CENTER", {"Center ____ _ ____ _ ____ _ ____ _ ____ _ ____", ansi_pattern({1.25, -0.25, 0.25, -0.25})}},
        {"CENTER2", {"Center (.5x) ___ _ ___ _ ___ _ ___ _ ___ _ ___", ansi_pattern({0.75, -0.125, 0.125, -0.125})}},
        {"CENTERX2", {"Center (2x) ________  __  ________  __  _____", ansi_pattern({2.5, -0.5, 0.5, -0.5})}},
        {"DASHDOT", {"Dash dot __ . __ . __ . __ . __ . __ . __ . __", ansi_pattern({0.5, -0.25, 0, -0.25})}},
        {"DASHDOT2", {"Dash dot (.5x) _._._._._._._._._._._._._._._.", ansi_pattern({0.25, -0.125, 0, -0.125})}},
        {"DASHDOTX2", {"Dash dot (2x) ____  .  ____  .  ____  .  ___", ansi_pattern({1.0, -0.5, 0, -0.5})}},
        {"DASHED", {"Dashed __ __ __ __ __ __ __ __ __ __ __ __ __ _", ansi_pattern({0.5, -0.25})}},
        {"DASHED2", {"Dashed (.5x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ ", ansi_pattern({0.25, -0.125})}},
        {"DASHEDX2", {"Dashed (2x) ____  ____  ____  ____  ____  ___", ansi_pattern({1.0, -0.5})}},
        {"DIVIDE", {"Divide ____ . . ____ . . ____ . . ____ . . ____", ansi_pattern({0.5, -0.25, 0, -0.25, 0, -0.25})}},
        {"DIVIDE2", {"Divide (.5x) __..__..__..__..__..__..__..__.._",
                     ansi_pattern({0.25, -0.125, 0, -0.125, 0, -0.125})}},
        {"DIVIDEX2", {"Divide (2x) ________  .  .  ________  .  .  _", ansi_pattern({1.0, -0.5, 0, -0.5, 0, -0.5})}},
        {"DOT", {"Dot . . . . . . . . . . . . . . . . . . . . . . . .", ansi_pattern({0, -0.25})}},
        {"DOT2", {"Dot (.5x) ........................................", ansi_pattern({0, -0.125})}},
        {"DOTX2", {"Dot (2x) .  .  .  .  .  .  .  .  .  .  .  .  .  .", ansi_pattern({0, -0.5})}},
        {"HIDDEN", {"Hidden __ __ __ __ __ __ __ __ __ __ __ __ __ __", ansi_pattern({0.25, -0.125})}},
        {"HIDDEN2", {"Hidden (.5x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ ", ansi_pattern({0.125, -0.0625})}},
        {"HIDDENX2", {"Hidden (2x) ____ ____ ____ ____ ____ ____ ____ ", ansi_pattern({0.5, -0.25})}},
        {"PHANTOM", {"Phantom ______  __  __  ______  __  __  ______ ",
                     ansi_pattern({1.25, -0.25, 0.25, -0.25, 0.25, -0.25})}},
        {"PHANTOM2", {"Phantom (.5x) ___ _ _ ___ _ _ ___ _ _ ___ _ _",
                      ansi_pattern({0.625, -0.125, 0.125, -0.125, 0.125, -0.125})}},
        {"PHANTOMX2", {"Phantom (2x) ____________    ____    ____   _", ansi_pattern({2.5, -0.5, 0.5, -0.5, 0.5, -0.5})}},
        {"ACAD_ISO02W100", {"ISO dash __ __ __ __ __ __ __ __ __ __ __ __ __", iso_pattern({12, -3})}},
        {"ACAD_ISO03W100", {"ISO dash space __    __    __    __    __    __", iso_pattern({12, -18})}},
        {"ACAD_ISO04W100", {"ISO long-dash dot ____ . ____ . ____ . ____ . _", iso_pattern({24, -3, 0, -3})}},
        {"ACAD_ISO05W100", {"ISO long-dash double-dot ____ .. ____ .. ____ . ", iso_pattern({24, -3, 0, -3, 0, -3})}},
        {"ACAD_ISO06W100",
         {"ISO long-dash triple-dot ____ ... ____ ... ____", iso_pattern({24, -3, 0, -3, 0, -3, 0, -3})}},
        {"ACAD_ISO07W100", {"ISO dot . . . . . . . . . . . . . . . . . . . . ", iso_pattern({0, -3})}},
        {"ACAD_ISO08W100", {"ISO long-dash short-dash ____ __ ____ __ ____ _", iso_pattern({24, -3, 6, -3})}},
        {"ACAD_ISO09W100",
         {"ISO long-dash double-short-dash ____ __ __ ____", iso_pattern({24, -3, 6, -3, 6, -3})}},
        {"ACAD_ISO10W100", {"ISO dash dot __ . __ . __ . __ . __ . __ . __ . ", iso_pattern({12, -3, 0, -3})}},
        {"ACAD_ISO11W100", {"ISO double-dash dot __ __ . __ __ . __ __ . __ _", iso_pattern({12, -3, 12, -3, 0, -3})}},
        {"ACAD_ISO12W100", {"ISO dash double-dot __ . . __ . . __ . . __ . . ", iso_pattern({12, -3, 0, -3, 0, -3})}},
        {"ACAD_ISO13W100",
         {"ISO double-dash double-dot __ __ . . __ __ . . _", iso_pattern({12, -3, 12, -3, 0, -3, 0, -3})}},
        {"ACAD_ISO14W100",
         {"ISO dash triple-dot __ . . . __ . . . __ . . . _", iso_pattern({12, -3, 0, -3, 0, -3, 0, -3})}},
        {"ACAD_ISO15W100",
         {"ISO double-dash triple-dot __ __ . . . __ __ . .", iso_pattern({12, -3, 12, -3, 0, -3, 0, -3, 0, -3})}},
    };
    return defs;
}

double Export2D::LTYPE_SCALE(Unit unit) {
    switch (unit) {
        case Unit::IN: return 0.1;
        case Unit::FT: return 0.1 / 12;
        case Unit::MM: return 2.54;
        case Unit::CM: return 0.254;
        case Unit::M: return 0.00254;
        default: break;
    }
    throw std::invalid_argument("No line type scale for this unit");
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

Drawing::Drawing(const Shape& shape, const Options& options)
    : Drawing(shape, options.look_at, options.look_from, options.look_up, options.with_hidden, options.focus) {}

Drawing::Drawing(const Shape& shape, const std::optional<Vector>& look_at, const Vector& look_from,
                 const Vector& look_up, bool with_hidden, std::optional<double> focus) {
    if (shape.is_null()) throw std::invalid_argument("Can't draw an empty shape");
    occt("Drawing", [&] {
        Handle(HLRBRep_Algo) hlr = new HLRBRep_Algo();
        hlr->Add(shape.wrapped());

        const Vector projection_origin = look_at ? *look_at : shape.center();
        const Vector projection_dir = look_from.normalized();
        const Vector projection_x = look_up.normalized().cross(projection_dir);
        const gp_Ax2 coordinate_system(projection_origin.to_pnt(), projection_dir.to_dir(), projection_x.to_dir());

        const HLRAlgo_Projector projector =
            focus ? HLRAlgo_Projector(coordinate_system, *focus) : HLRAlgo_Projector(coordinate_system);
        hlr->Projector(projector);
        hlr->Update();
        hlr->Hide();

        HLRBRep_HLRToShape hlr_shapes(hlr);

        std::vector<TopoDS_Shape> visible;
        for (const TopoDS_Shape& s : {hlr_shapes.VCompound(), hlr_shapes.Rg1LineVCompound(), hlr_shapes.OutLineVCompound()}) {
            if (!s.IsNull()) visible.push_back(s);
        }
        std::vector<TopoDS_Shape> hidden;
        if (with_hidden) {
            for (const TopoDS_Shape& s : {hlr_shapes.HCompound(), hlr_shapes.OutLineHCompound()}) {
                if (!s.IsNull()) hidden.push_back(s);
            }
        }
        for (const auto& el : visible) BRepLib::BuildCurves3d(el, TOLERANCE);
        for (const auto& el : hidden) BRepLib::BuildCurves3d(el, TOLERANCE);

        std::vector<Shape> v, h;
        for (const auto& s : visible) v.emplace_back(s);
        for (const auto& s : hidden) h.emplace_back(s);
        visible_lines = Compound(v);
        hidden_lines = Compound(h);
        return 0;
    });
}

}  // namespace build123d
