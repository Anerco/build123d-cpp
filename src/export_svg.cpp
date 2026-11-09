// ExportSVG (exporters.py), writing the same SVG text as Python's ElementTree + svgpathtools.
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GeomConvert_BSplineCurveToBezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>

#include "build123d/exporters.hpp"
#include "build123d/warnings.hpp"
#include "internal.hpp"
#include "io_internal.hpp"
#include "pyformat.hpp"
#include "svgpath.hpp"

namespace build123d {

using namespace detail;
using svgpath::Cx;
using svgpath::Segment;

namespace {

// ---------------------------------------------------------------------------
// A minimal xml.etree.ElementTree: ordered attributes, indent() and write()
// ---------------------------------------------------------------------------

struct XmlElement {
    std::string tag;
    std::vector<std::pair<std::string, std::string>> attrib;
    std::vector<XmlElement> children;
    std::string text;
    std::string tail;

    XmlElement() = default;
    XmlElement(std::string t, std::vector<std::pair<std::string, std::string>> a = {})
        : tag(std::move(t)), attrib(std::move(a)) {}

    /// Element.set: replaces a value in place, else appends.
    void set(const std::string& key, const std::string& value) {
        for (auto& kv : attrib) {
            if (kv.first == key) {
                kv.second = value;
                return;
            }
        }
        attrib.emplace_back(key, value);
    }
};

std::string escape_attrib(const std::string& text) {
    std::string out;
    for (char c : text) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\r': out += "&#13;"; break;
            case '\n': out += "&#10;"; break;
            case '\t': out += "&#09;"; break;
            default: out += c;
        }
    }
    return out;
}

std::string escape_cdata(const std::string& text) {
    std::string out;
    for (char c : text) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            default: out += c;
        }
    }
    return out;
}

bool is_blank(const std::string& s) {
    for (char c : s) {
        if (!std::isspace(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

/// ET.indent(tree, space)
void indent(XmlElement& root, const std::string& space) {
    std::vector<std::string> indentations{"\n"};
    std::function<void(XmlElement&, std::size_t)> indent_children = [&](XmlElement& elem, std::size_t level) {
        if (level >= indentations.size()) indentations.push_back(indentations[level - 1] + space);
        const std::string child_indentation = indentations[level];
        if (elem.text.empty() || is_blank(elem.text)) elem.text = child_indentation;
        for (auto& child : elem.children) {
            if (!child.children.empty()) indent_children(child, level + 1);
            if (child.tail.empty() || is_blank(child.tail)) child.tail = child_indentation;
        }
        auto& last = elem.children.back();
        if (is_blank(last.tail)) last.tail = indentations[level - 1];
    };
    if (!root.children.empty()) indent_children(root, 1);
}

void serialize(const XmlElement& elem, std::string& out) {
    out += "<" + elem.tag;
    for (const auto& [k, v] : elem.attrib) out += " " + k + "=\"" + escape_attrib(v) + "\"";
    if (!elem.text.empty() || !elem.children.empty()) {
        out += ">";
        if (!elem.text.empty()) out += escape_cdata(elem.text);
        for (const auto& child : elem.children) serialize(child, out);
        out += "</" + elem.tag + ">";
    } else {
        out += " />";
    }
    if (!elem.tail.empty()) out += escape_cdata(elem.tail);
}

std::string str(double x) { return pyfmt::repr(x); }

constexpr double kPi = 3.141592653589793;

}  // namespace

// ---------------------------------------------------------------------------
// Colors
// ---------------------------------------------------------------------------

std::optional<Color> SvgColor::resolve() const {
    if (index_) {
        warn("ExportSVG ColorIndex values are deprecated; use ColorLike values such as 'red' or 0xFF0000 instead.");
        switch (*index_) {
            case ColorIndex::BLACK: return Color("black");
            case ColorIndex::RED: return Color(1.0, 0.0, 0.0);
            case ColorIndex::YELLOW: return Color(1.0, 1.0, 0.0);
            case ColorIndex::GREEN: return Color(0.0, 1.0, 0.0);
            case ColorIndex::CYAN: return Color(0.0, 1.0, 1.0);
            case ColorIndex::BLUE: return Color(0.0, 0.0, 1.0);
            case ColorIndex::MAGENTA: return Color(1.0, 0.0, 1.0);
            case ColorIndex::GRAY: return Color(128 / 255.0, 128 / 255.0, 128 / 255.0);
            case ColorIndex::LIGHT_GRAY: return Color(192 / 255.0, 192 / 255.0, 192 / 255.0);
        }
    }
    if (tuple_) {
        const auto& t = *tuple_;
        const bool legacy = t[0] > 1 || t[1] > 1 || t[2] > 1 || (tuple_alpha_ && t[3] > 1);
        if (!legacy) return Color(t[0], t[1], t[2], tuple_alpha_ ? t[3] : 1.0);
        warn("ExportSVG 0-255 RGB tuples are deprecated; use normalized ColorLike tuples, a hexadecimal "
             "integer, or a color name instead.");
        const double alpha = tuple_alpha_ ? t[3] : 255.0;
        const double normalized_alpha = alpha > 1 ? alpha / 255 : alpha;
        return Color(t[0] / 255, t[1] / 255, t[2] / 255, normalized_alpha);
    }
    if (!color_) return std::nullopt;
    return *color_;
}

// ---------------------------------------------------------------------------
// ExportSVG
// ---------------------------------------------------------------------------

struct ExportSVG::Impl {
    struct Layer {
        std::string name;
        bool fill_color_override = false;
        bool line_color_override = false;
        std::optional<Color> fill_color;
        std::optional<Color> line_color;
        double line_weight = DEFAULT_LINE_WEIGHT;
        LineType line_type = DEFAULT_LINE_TYPE;
        std::vector<XmlElement> elements;
    };
    std::vector<Layer> layers;
    std::optional<BoundBox> bounds;
    int non_planar_point_count = 0;

    Layer* find(const std::string& name) {
        for (auto& l : layers) {
            if (l.name == name) return &l;
        }
        return nullptr;
    }
};

namespace {

/// The (color, opacity) attribute values of a Color, as Python computes them from the
/// float32 color OCCT stores.
std::pair<std::string, std::optional<std::string>> color_attribs(const std::optional<Color>& color) {
    if (!color) return {"none", std::nullopt};
    // tuple(color): OCCT's float color as sRGB, each value rounded to 7 digits
    const Quantity_ColorRGBA q = color->wrapped();
    double r, g, b;
    q.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
    r = pyfmt::round(r, 7);
    g = pyfmt::round(g, 7);
    b = pyfmt::round(b, 7);
    const double alpha = pyfmt::round(q.Alpha(), 7);
    const std::string rgb = "rgb(" + std::to_string(static_cast<long long>(r * 255)) + "," +
                            std::to_string(static_cast<long long>(g * 255)) + "," +
                            std::to_string(static_cast<long long>(b * 255)) + ")";
    std::optional<std::string> opacity;
    if (alpha < 1) opacity = str(pyfmt::round(alpha, 3));
    return {rgb, opacity};
}

void set_element_color(XmlElement& element, const std::string& attribute, const Color& color) {
    const auto [value, opacity] = color_attribs(color);
    element.set(attribute, value);
    if (opacity) element.set(attribute + "-opacity", *opacity);
}

/// Converts edges to svgpathtools segments and SVG elements.
class SvgConverter {
public:
    SvgConverter(int precision, int& non_planar) : precision_(precision), non_planar_(non_planar) {}

    Cx path_point(const gp_Pnt& pt) {
        const double x = pyfmt::round(pt.X(), precision_);
        const double y = pyfmt::round(pt.Y(), precision_);
        const double z = pyfmt::round(pt.Z(), precision_);
        if (z != 0) ++non_planar_;
        return {x, y};
    }

    static std::vector<Edge> wire_edges(const Wire& wire, bool reverse) {
        std::vector<Edge> edges;
        for (BRepTools_WireExplorer explorer(TopoDS::Wire(wire.wrapped())); explorer.More(); explorer.Next()) {
            edges.emplace_back(explorer.Current());
        }
        if (reverse) std::reverse(edges.begin(), edges.end());
        return edges;
    }

    std::vector<Segment> wire_segments(const Wire& wire, bool reverse) {
        std::vector<Segment> segments;
        for (const Edge& edge : wire_edges(wire, reverse)) {
            auto s = edge_segments(edge, reverse);
            segments.insert(segments.end(), s.begin(), s.end());
        }
        return segments;
    }

    XmlElement wire_element(const Wire& wire, bool reverse) {
        const auto edges = wire_edges(wire, reverse);
        if (edges.size() == 1) return edge_element(edges[0]);
        std::vector<Segment> segments;
        for (const Edge& edge : edges) {
            auto s = edge_segments(edge, reverse);
            segments.insert(segments.end(), s.begin(), s.end());
        }
        return XmlElement("path", {{"d", svgpath::path_d(segments)}});
    }

    std::vector<Segment> edge_segments(const Edge& edge, bool reverse) {
        if (edge.is_null()) throw std::invalid_argument("Edge is empty");
        const bool edge_reversed = edge.wrapped().Orientation() == TopAbs_REVERSED;
        const bool r = reverse != edge_reversed;
        switch (edge.geom_type()) {
            case GeomType::LINE: return {line_segment(edge, r)};
            case GeomType::CIRCLE: return circle_segments(edge, r);
            case GeomType::ELLIPSE: return ellipse_segments(edge, r);
            default: return bspline_segments(edge, r);
        }
    }

    XmlElement edge_element(const Edge& edge) {
        switch (edge.geom_type()) {
            case GeomType::LINE: {
                const Segment s = line_segment(edge, false);
                return XmlElement("line", {{"x1", str(s.start.re)}, {"y1", str(s.start.im)}, {"x2", str(s.end.re)},
                                           {"y2", str(s.end.im)}});
            }
            case GeomType::CIRCLE: return circle_element(edge);
            case GeomType::ELLIPSE: return XmlElement("path", {{"d", svgpath::path_d(ellipse_segments(edge, false))}});
            default: return XmlElement("path", {{"d", svgpath::path_d(bspline_segments(edge, false))}});
        }
    }

private:
    int precision_;
    int& non_planar_;

    Segment line_segment(const Edge& edge, bool reverse) {
        const BRepAdaptor_Curve curve(TopoDS::Edge(edge.wrapped()));
        const double fp = curve.FirstParameter(), lp = curve.LastParameter();
        const double u0 = reverse ? lp : fp, u1 = reverse ? fp : lp;
        const Cx p0 = path_point(curve.Value(u0));
        const Cx p1 = path_point(curve.Value(u1));
        return Segment::line(p0, p1);
    }

    template <class Conic>
    std::vector<Segment> conic_segments(const Edge& edge, bool reverse, const Conic& conic, Cx radius,
                                        const BRepAdaptor_Curve& curve) {
        const gp_Dir x_axis = conic.XAxis().Direction();
        const gp_Dir z_axis = conic.Axis().Direction();
        const double fp = curve.FirstParameter(), lp = curve.LastParameter();
        const double du = lp - fp;
        const bool large_arc = (du < -kPi) || (du > kPi);
        const bool sweep = (z_axis.Z() > 0) != reverse;
        const double u0 = reverse ? lp : fp, u1 = reverse ? fp : lp;
        const Cx start = path_point(curve.Value(u0));
        const Cx end = path_point(curve.Value(u1));
        const double rotation = gp_Dir(1, 0, 0).AngleWithRef(x_axis, gp_Dir(0, 0, 1)) * (180.0 / kPi);
        if (curve.IsClosed()) {
            const Cx midway = path_point(curve.Value((u0 + u1) / 2));
            return {Segment::make(svgpath::make_arc(start, radius, rotation, false, sweep, midway)),
                    Segment::make(svgpath::make_arc(midway, radius, rotation, false, sweep, end))};
        }
        return {Segment::make(svgpath::make_arc(start, radius, rotation, large_arc, sweep, end))};
    }

    std::vector<Segment> circle_segments(const Edge& edge, bool reverse) {
        if (edge.length() < 1e-6) {
            warn("Skipping arc that is too small to export safely (length < 1e-6).");
            return {};
        }
        const BRepAdaptor_Curve curve(TopoDS::Edge(edge.wrapped()));
        const gp_Circ circle = curve.Circle();
        return conic_segments(edge, reverse, circle, Cx{circle.Radius(), circle.Radius()}, curve);
    }

    XmlElement circle_element(const Edge& edge) {
        if (edge.is_closed()) {
            const BRepAdaptor_Curve curve(TopoDS::Edge(edge.wrapped()));
            const gp_Circ circle = curve.Circle();
            const Cx c = path_point(circle.Location());
            return XmlElement("circle", {{"cx", str(c.re)}, {"cy", str(c.im)}, {"r", str(circle.Radius())}});
        }
        return XmlElement("path", {{"d", svgpath::path_d(circle_segments(edge, false))}});
    }

    std::vector<Segment> ellipse_segments(const Edge& edge, bool reverse) {
        if (edge.length() < 1e-6) {
            warn("Skipping ellipse that is too small to export safely (length < 1e-6).");
            return {};
        }
        const BRepAdaptor_Curve curve(TopoDS::Edge(edge.wrapped()));
        const gp_Elips ellipse = curve.Ellipse();
        return conic_segments(edge, reverse, ellipse, Cx{ellipse.MajorRadius(), ellipse.MinorRadius()}, curve);
    }

    std::vector<Segment> bspline_segments(const Edge& edge_in, bool reverse) {
        // Reduce to degree 3 B-splines (converts almost any edge)
        const Edge edge = edge_in.to_splines();
        if (edge.is_null()) throw std::invalid_argument("Edge is empty");
        const BRepAdaptor_Curve adaptor(TopoDS::Edge(edge.wrapped()));
        Handle(Geom_BSplineCurve) spline =
            Handle(Geom_BSplineCurve)::DownCast(adaptor.Curve().Curve()->Copy());
        if (spline.IsNull()) throw std::runtime_error("Expected a B-spline curve");
        const double u1 = adaptor.FirstParameter(), u2 = adaptor.LastParameter();
        spline->Transform(edge.wrapped().Location().Transformation());
        GeomConvert_BSplineCurveToBezierCurve converter(spline, u1, u2, Export2D::PARAMETRIC_TOLERANCE);
        std::vector<Segment> result;
        for (int i = 1; i <= converter.NbArcs(); ++i) {
            const Handle(Geom_BezierCurve) bezier = converter.Arc(i);
            std::vector<Cx> p;
            for (int k = 1; k <= bezier->NbPoles(); ++k) p.push_back(path_point(bezier->Pole(k)));
            if (reverse) std::reverse(p.begin(), p.end());
            if (p.size() == 2) {
                result.push_back(Segment::line(p[0], p[1]));
            } else if (p.size() == 3) {
                result.push_back(Segment::quadratic(p[0], p[1], p[2]));
            } else if (p.size() == 4) {
                result.push_back(Segment::cubic(p[0], p[1], p[2], p[3]));
            } else {
                throw std::invalid_argument("Surprising B\xc3\xa9zier of degree " + std::to_string(bezier->Degree()) + "!");
            }
        }
        if (reverse) std::reverse(result.begin(), result.end());
        return result;
    }
};

}  // namespace

ExportSVG::ExportSVG(const Options& o)
    : ExportSVG(o.unit, o.scale, o.margin, o.fit_to_stroke, o.precision, o.fill_color, o.line_color, o.line_weight,
                o.line_type, o.dot_length) {}

ExportSVG::ExportSVG(Unit unit_, double scale_, double margin_, bool fit_to_stroke_, int precision_,
                     const SvgColor& fill_color, const SvgColor& line_color, double line_weight, LineType line_type,
                     DotLengthArg dot_length_)
    : unit(unit_),
      scale(scale_),
      margin(margin_),
      fit_to_stroke(fit_to_stroke_),
      precision(precision_),
      dot_length(dot_length_.value()),
      impl_(std::make_unique<Impl>()) {
    if (unit != Unit::MM && unit != Unit::CM && unit != Unit::IN) {
        throw std::invalid_argument("Invalid unit.  Supported units are mm, cm, in.");
    }
    add_layer("", {fill_color, line_color, line_weight, line_type});
}

ExportSVG::~ExportSVG() = default;
ExportSVG::ExportSVG(ExportSVG&&) noexcept = default;
ExportSVG& ExportSVG::operator=(ExportSVG&&) noexcept = default;

ExportSVG& ExportSVG::add_layer(const std::string& name, const LayerOptions& options) {
    if (impl_->find(name)) throw std::invalid_argument("Duplicate layer name '" + name + "'.");
    if (LINETYPE_DEFS().count(value(options.line_type)) == 0) {
        throw std::invalid_argument("Unknown linetype `" + value(options.line_type) + "`.");
    }
    Impl::Layer layer;
    layer.name = name;
    layer.fill_color_override = !options.fill_color.inherit();
    layer.line_color_override = !options.line_color.inherit();
    layer.fill_color = options.fill_color.inherit() ? std::nullopt : options.fill_color.resolve();
    layer.line_color = options.line_color.inherit() ? std::optional<Color>(Color("black")) : options.line_color.resolve();
    layer.line_weight = options.line_weight;
    layer.line_type = options.line_type;
    impl_->layers.push_back(std::move(layer));
    return *this;
}

void ExportSVG::check_layer(const std::string& layer) const {
    if (!impl_->find(layer)) throw std::invalid_argument("Undefined layer: " + layer + ".");
}

void ExportSVG::add_shape(const Shape& shape, const std::string& layer_name, bool reverse_wires) {
    check_layer(layer_name);
    Impl::Layer& layer = *impl_->find(layer_name);
    impl_->non_planar_point_count = 0;
    const BoundBox bb = shape.bounding_box();
    impl_->bounds = impl_->bounds ? impl_->bounds->add(bb) : bb;
    std::vector<XmlElement> elements;
    const std::optional<Color> shape_color = shape.color;
    SvgConverter conv(precision, impl_->non_planar_point_count);

    occt("ExportSVG.add_shape", [&] {
        // Faces
        for (const Face& face : shape.faces()) {
            const Wire outer = face.outer_wire();
            const auto inner = face.inner_wires();
            XmlElement face_element;
            if (inner.empty()) {
                // Faces without holes as bare wires, keeping circles and ellipses
                face_element = conv.wire_element(outer, reverse_wires);
            } else {
                auto face_segments = conv.wire_segments(outer, reverse_wires);
                for (const Wire& w : inner) {
                    auto s = conv.wire_segments(w, reverse_wires);
                    face_segments.insert(face_segments.end(), s.begin(), s.end());
                }
                face_element = XmlElement("path", {{"d", svgpath::path_d(face_segments)}});
            }
            if (shape_color && !layer.fill_color_override) set_element_color(face_element, "fill", *shape_color);
            elements.push_back(std::move(face_element));
        }
        // Wires that are not part of faces
        for (TopExp_Explorer explorer(shape.wrapped(), TopAbs_WIRE, TopAbs_FACE); explorer.More(); explorer.Next()) {
            XmlElement wire_element = conv.wire_element(Wire(explorer.Current()), reverse_wires);
            if (shape_color && !layer.line_color_override) set_element_color(wire_element, "stroke", *shape_color);
            elements.push_back(std::move(wire_element));
        }
        // Edges that are not part of wires
        std::vector<XmlElement> loose_edge_elements;
        for (TopExp_Explorer explorer(shape.wrapped(), TopAbs_EDGE, TopAbs_WIRE); explorer.More(); explorer.Next()) {
            loose_edge_elements.push_back(conv.edge_element(Edge(explorer.Current())));
        }
        if (shape_color && !layer.line_color_override) {
            for (auto& e : loose_edge_elements) set_element_color(e, "stroke", *shape_color);
        }
        elements.insert(elements.end(), loose_edge_elements.begin(), loose_edge_elements.end());
        return 0;
    });

    layer.elements.insert(layer.elements.end(), elements.begin(), elements.end());
    if (impl_->non_planar_point_count > 0) {
        warn("Exporting non-planar shape to 2D format; " + std::to_string(impl_->non_planar_point_count) +
             " points found outside the XY plane.");
    }
}

std::string ExportSVG::to_string() const {
    if (!impl_->bounds) throw std::invalid_argument("No shapes to export.");
    const BoundBox& bb = *impl_->bounds;
    double doc_margin = margin;
    if (fit_to_stroke) {
        double max_line_weight = impl_->layers.front().line_weight;
        for (const auto& l : impl_->layers) max_line_weight = std::max(max_line_weight, l.line_weight);
        doc_margin += max_line_weight / 2;
    }
    const double view_margin = doc_margin / scale;
    const double view_left = pyfmt::round(+bb.min.X - view_margin, precision);
    const double view_top = pyfmt::round(-bb.max.Y - view_margin, precision);
    const double view_width = pyfmt::round(bb.size.X + 2 * view_margin, precision);
    const double view_height = pyfmt::round(bb.size.Y + 2 * view_margin, precision);
    const double doc_width = pyfmt::round(view_width * scale, precision);
    const double doc_height = pyfmt::round(view_height * scale, precision);
    const std::string doc_unit = unit == Unit::MM ? "mm" : unit == Unit::CM ? "cm" : "in";

    XmlElement svg("svg", {{"width", str(doc_width) + doc_unit},
                           {"height", str(doc_height) + doc_unit},
                           {"viewBox", str(view_left) + " " + str(view_top) + " " + str(view_width) + " " +
                                           str(view_height)},
                           {"version", "1.1"},
                           {"xmlns", "http://www.w3.org/2000/svg"}});
    XmlElement container("g", {{"transform", "scale(1,-1)"}, {"stroke-linecap", "round"}});

    for (const auto& layer : impl_->layers) {
        if (layer.elements.empty()) continue;
        XmlElement group("g");
        const auto [fill, fill_opacity] = color_attribs(layer.fill_color);
        group.set("fill", fill);
        if (fill_opacity) group.set("fill-opacity", *fill_opacity);
        const auto [stroke, stroke_opacity] = color_attribs(layer.line_color);
        group.set("stroke", stroke);
        if (stroke_opacity && !stroke_opacity->empty()) group.set("stroke-opacity", *stroke_opacity);
        const double lwscale = unit_conversion_scale(Unit::MM, unit) / scale;
        group.set("stroke-width", str(layer.line_weight * lwscale));
        if (!layer.name.empty()) group.set("id", layer.name);
        if (layer.line_type != LineType::CONTINUOUS) {
            // _stroke_dasharray
            std::vector<double> pattern = LINETYPE_DEFS().at(value(layer.line_type)).second;
            const double d = dot_length;
            const std::size_t plen = pattern.size();
            for (std::size_t i = 0; i < plen; ++i) {
                if (pattern[i] == 0) {
                    pattern[i] = d;
                    pattern[(i + plen - 1) % plen] -= d / 2;
                    pattern[(i + 1) % plen] -= d / 2;
                }
            }
            const double ltscale = LTYPE_SCALE(unit) * layer.line_weight / scale;
            std::string dash;
            for (std::size_t i = 1; i < plen; ++i) {
                if (i > 1) dash += " ";
                dash += str(pyfmt::round(ltscale * std::fabs(pattern[i]), precision));
            }
            group.set("stroke-dasharray", dash);
        }
        group.children = layer.elements;
        container.children.push_back(std::move(group));
    }
    svg.children.push_back(std::move(container));
    indent(svg, "  ");
    std::string out = "<?xml version='1.0' encoding='utf-8'?>\n";
    serialize(svg, out);
    return out;
}

void ExportSVG::write(std::ostream& stream) const { stream << to_string(); }

void ExportSVG::write(const std::string& path) const {
    const std::string text = to_string();
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Can't write " + path);
    out << text;
}

}  // namespace build123d
