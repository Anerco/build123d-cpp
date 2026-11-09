// SVG import (importers.py import_svg and import_svg_as_buildline_code). The document is read
// by the svgelements port in svgdoc.cpp; this file ports ocpsvg (svg.py, ocp.py), which turns
// its shapes into OCCT wires and faces, and build123d's own part.
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFeat.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepLib_FindSurface.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeArcOfEllipse.hxx>
#include <GC_MakeCircle.hxx>
#include <GC_MakeEllipse.hxx>
#include <GC_MakeSegment.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <ShapeExtend_WireData.hxx>
#include <ShapeFix_Face.hxx>
#include <Standard_Version.hxx>
#include <Standard_Failure.hxx>
#include <StdFail_NotDone.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "build123d/importers.hpp"
#include "build123d/warnings.hpp"
#include "internal.hpp"
#include "pyformat.hpp"
#include "svgdoc.hpp"
#include "svgpath.hpp"
#include "xml_reader.hpp"

namespace build123d {

using namespace detail;

namespace {

constexpr double kPi = 3.141592653589793;

double radians(double x) { return x * (kPi / 180.0); }
double degrees(double x) { return x * (180.0 / kPi); }

/// Python's ValueError inside ocpsvg (caught where ocpsvg catches it).
struct OcpValueError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

/// ocpsvg's InvalidWiresForFace.
struct InvalidWiresForFace : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// ---------------------------------------------------------------------------
// ocpsvg/ocp.py
// ---------------------------------------------------------------------------

constexpr double kOcpTolerance = 1e-8;

TopoDS_Compound wires_compound(const std::vector<TopoDS_Wire>& shapes) {
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& s : shapes) builder.Add(compound, s);
    return compound;
}

bool are_wires_coplanar(const std::vector<TopoDS_Wire>& wires) {
    return wires.empty() || BRepLib_FindSurface(wires_compound(wires), -1, true).Found();
}

TopoDS_Edge edge_from_curve(const Handle(Geom_Curve)& curve) { return BRepBuilderAPI_MakeEdge(curve).Edge(); }

Handle(Geom_Curve) segment_curve(const gp_Pnt& start, const gp_Pnt& end) {
    try {
        return GC_MakeSegment(start, end).Value();
    } catch (const Standard_Failure&) {
        throw OcpValueError("could not make segment curve");
    }
}

Handle(Geom_Curve) bezier_curve(const std::vector<gp_Pnt>& controls) {
    const int n = static_cast<int>(controls.size());
    if (n < 2 || n > Geom_BezierCurve::MaxDegree()) {
        throw OcpValueError("bezier curve must have between 2 and 25 control points");
    }
    TColgp_Array1OfPnt poles(1, n);
    for (int i = 0; i < n; ++i) poles.SetValue(i + 1, controls[static_cast<std::size_t>(i)]);
    return new Geom_BezierCurve(poles);
}

Handle(Geom_Curve) circle_curve(double radius, const gp_Pnt& center) {
    const gp_Circ circle_gp(gp_Ax2(gp_Pnt(), gp_Dir(0, 0, 1)), radius);
    Handle(Geom_Curve) circle = GC_MakeCircle(circle_gp).Value();
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(center.XYZ()));
    circle->Transform(trsf);
    return circle;
}

Handle(Geom_Curve) ellipse_curve(double major_radius, double minor_radius, double start_angle, double end_angle,
                                 bool clockwise, double rotation, const gp_Pnt& center) {
    double angle_adjustment = 0;
    if (minor_radius > major_radius) {
        std::swap(major_radius, minor_radius);
        angle_adjustment = 90;
    }
    const gp_Dir normal(0, 0, 1);
    gp_Elips ellipse_gp(gp_Ax2(gp_Pnt(), normal), major_radius, minor_radius);
    ellipse_gp.Rotate(gp_Ax1(), radians(angle_adjustment));
    Handle(Geom_Curve) ellipse;
    if (start_angle == end_angle) {
        ellipse = GC_MakeEllipse(ellipse_gp).Value();
    } else {
        ellipse = GC_MakeArcOfEllipse(ellipse_gp, radians(start_angle - angle_adjustment),
                                      radians(end_angle - angle_adjustment), clockwise)
                      .Value();
    }
    ellipse->Rotate(gp_Ax1(gp_Pnt(), normal), radians(rotation));
    ellipse->Translate(gp_Vec(center.XYZ()));
    return ellipse;
}

TopoDS_Wire closed_wire(const TopoDS_Wire& wire) {
    if (BRep_Tool::IsClosed(wire)) return wire;
    TopoDS_Iterator it(wire);
    if (!it.More()) return wire;  // no edges
    const TopoDS_Shape first_edge = it.Value();
    TopoDS_Shape last_edge = first_edge;
    for (it.Next(); it.More(); it.Next()) last_edge = it.Value();
    const BRepAdaptor_Curve first_adaptor(TopoDS::Edge(first_edge));
    const gp_Pnt start = first_adaptor.Value(first_adaptor.FirstParameter());
    const BRepAdaptor_Curve last_adaptor(TopoDS::Edge(last_edge));
    const gp_Pnt end = last_adaptor.Value(last_adaptor.LastParameter());
    if (!start.IsEqual(end, kOcpTolerance)) {
        Handle(ShapeExtend_WireData) extend = new ShapeExtend_WireData();
        extend->AddOriented(wire, 0);
        extend->AddOriented(edge_from_curve(segment_curve(end, start)), 0);
        return extend->WireAPIMake();
    }
    return wire;
}

TopoDS_Wire wire_from_continuous_edges(const std::vector<TopoDS_Edge>& edges, bool closed) {
    Handle(ShapeExtend_WireData) extend = new ShapeExtend_WireData(TopoDS_Wire());
    for (const auto& edge : edges) extend->AddOriented(edge, 0);
    const TopoDS_Wire wire = extend->WireAPIMake();
    return closed ? closed_wire(wire) : wire;
}

TopoDS_Face face_from_wires(const TopoDS_Wire& outer_wire, const std::vector<TopoDS_Wire>& inner_wires = {}) {
    BRepBuilderAPI_MakeFace face_builder(closed_wire(outer_wire), true);
    for (const auto& inner : inner_wires) face_builder.Add(closed_wire(inner));
    ShapeFix_Face face_fix(face_builder.Face());
    face_fix.FixOrientation();
    return face_fix.Face();
}

TopoDS_Wire face_outer_wire(const TopoDS_Face& face) { return BRepTools::OuterWire(face); }

TopoDS_Face ensure_face_normal_up(TopoDS_Face face) {
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    gp_Pnt point;
    gp_Vec normal;
    BRepGProp_Face(face).Normal((u0 + u1) / 2, (v0 + v1) / 2, point, normal);
    if (normal.Z() < 0) face.Reverse();
    return face;
}

/// faces_from_wire_soup: faces from unorganized, possibly nested (but non-intersecting) wires.
std::vector<TopoDS_Face> faces_from_wire_soup(const std::vector<TopoDS_Wire>& wires) {
    for (const auto& wire : wires) {
        if (!are_wires_coplanar({wire})) throw InvalidWiresForFace("invalid wire");
    }
    if (!are_wires_coplanar(wires)) throw InvalidWiresForFace("wires are not coplanar");
    std::vector<TopoDS_Face> faces;
    for (const auto& wire : wires) {
        faces.push_back(BRepBuilderAPI_MakeFace(face_outer_wire(face_from_wires(wire)), true).Face());
    }
    std::vector<TopoDS_Face> out;
    if (faces.size() < 2) {
        for (auto& f : faces) out.push_back(ensure_face_normal_up(f));
        return out;
    }
    const int n = static_cast<int>(faces.size());
    std::map<int, std::set<int>> included_in;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i != j && BRepFeat::IsInside(faces[static_cast<std::size_t>(i)], faces[static_cast<std::size_t>(j)])) {
                included_in[i].insert(j);
            }
        }
    }
    auto depth = [&included_in](int k) {
        const auto it = included_in.find(k);
        return it == included_in.end() ? std::size_t{0} : it->second.size();
    };
    // outers_and_inners, in the order its keys are first used (a Python dict)
    std::vector<int> order;
    std::map<int, std::pair<std::vector<TopoDS_Wire>, std::vector<TopoDS_Wire>>> outers_and_inners;
    auto entry = [&](int key) -> std::pair<std::vector<TopoDS_Wire>, std::vector<TopoDS_Wire>>& {
        if (outers_and_inners.find(key) == outers_and_inners.end()) order.push_back(key);
        return outers_and_inners[key];
    };
    for (int i = 0; i < n; ++i) {
        const auto it = included_in.find(i);
        const std::set<int> ancestors = it == included_in.end() ? std::set<int>() : it->second;
        if (ancestors.size() % 2) {  // odd depth: inner ring
            int parent_i = *ancestors.begin();
            for (int k : ancestors) {
                if (depth(k) > depth(parent_i)) parent_i = k;
            }
            entry(parent_i).second.push_back(face_outer_wire(faces[static_cast<std::size_t>(i)]));
        } else {  // even depth: outer ring
            entry(i).first.push_back(face_outer_wire(faces[static_cast<std::size_t>(i)]));
        }
    }
    for (int key : order) {
        const auto& [outers, inners] = outers_and_inners[key];
        if (outers.size() == 1) {
            out.push_back(ensure_face_normal_up(face_from_wires(outers[0], inners)));
        } else {
            for (const auto& w : outers) out.push_back(ensure_face_normal_up(face_from_wires(w)));
            for (const auto& w : inners) out.push_back(ensure_face_normal_up(face_from_wires(w)));
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// ocpsvg/svg.py
// ---------------------------------------------------------------------------

using svgdoc::SegKind;
using svgdoc::Segment;

gp_Pnt p(const std::optional<svgdoc::Point>& c) {
    if (!c) return gp_Pnt(0, 0, 0);  // ocpsvg logs "found None point"
    return gp_Pnt(c->x, c->y, 0);
}

/// Point != Point, where either may be None.
bool points_differ(const std::optional<svgdoc::Point>& a, const std::optional<svgdoc::Point>& b) {
    if (a && b) return !svgdoc::same_point(*a, *b);
    return a.has_value() != b.has_value();
}

/// curves_from_segment: the curve of a segment, or none.
Handle(Geom_Curve) curve_from_segment(const Segment& segment) {
    switch (segment.kind) {
        case SegKind::MOVE: return nullptr;
        case SegKind::LINE:
        case SegKind::CLOSE:
            if (points_differ(segment.start, segment.end)) return segment_curve(p(segment.start), p(segment.end));
            return nullptr;
        case SegKind::QUAD: return bezier_curve({p(segment.start), p(segment.control), p(segment.end)});
        case SegKind::CUBIC:
            return bezier_curve({p(segment.start), p(segment.control1), p(segment.control2), p(segment.end)});
        case SegKind::ARC: {
            const double start_angle = segment.theta();
            const double end_angle = segment.theta() + segment.delta();
            return ellipse_curve(segment.rx(), segment.ry(), std::min(start_angle, end_angle),
                                 std::max(end_angle, start_angle), end_angle > start_angle,
                                 degrees(segment.rotation()), p(segment.center));
        }
    }
    return nullptr;
}

/// wires_from_svg_path: a wire for each continuous subpath.
std::vector<TopoDS_Wire> wires_from_svg_path(const std::vector<Segment>& path) {
    // Path.as_subpaths
    std::vector<std::pair<std::size_t, std::size_t>> subpaths;
    std::size_t start = 0;
    for (std::size_t current = 0; current < path.size(); ++current) {
        if (path[current].kind == SegKind::MOVE && current != start) {
            subpaths.emplace_back(start, current - 1);
            start = current;
        }
        if (path[current].kind == SegKind::CLOSE) {
            subpaths.emplace_back(start, current);
            start = current + 1;
        }
    }
    if (start != path.size()) subpaths.emplace_back(start, path.size() - 1);

    std::vector<TopoDS_Wire> wires;
    for (const auto& [first, last] : subpaths) {
        const bool closed = path[last].kind == SegKind::CLOSE;
        std::vector<TopoDS_Edge> edges;
        for (std::size_t i = first; i <= last; ++i) {
            try {
                const Handle(Geom_Curve) curve = curve_from_segment(path[i]);
                if (!curve.IsNull()) edges.push_back(edge_from_curve(curve));
            } catch (const StdFail_NotDone&) {
                // ocpsvg logs the invalid segment and skips it
            } catch (const OcpValueError&) {
            }
        }
        if (!edges.empty()) wires.push_back(wire_from_continuous_edges(edges, closed));
    }
    return wires;
}

TopoDS_Wire transform_if_needed(const TopoDS_Wire& wire, const svgdoc::Matrix& t) {
    if (t.is_identity()) return wire;
    // check_unskewed_transform
    const svgdoc::Point o = t.apply({0, 0});
    const svgdoc::Point x = t.apply({1, 0});
    const svgdoc::Point y = t.apply({0, 1});
    const double angle_x = std::atan2(x.y - o.y, x.x - o.x);
    const double angle_y = std::atan2(y.y - o.y, y.x - o.x);
    const bool is_skewed = std::fabs(angle_y - angle_x) != kPi / 2;
    if (!is_skewed) {
        gp_Trsf trsf;
        trsf.SetValues(t.a, t.c, 0.0, t.e, t.b, t.d, 0.0, t.f, 0.0, 0.0, 1.0, 0.0);
        return TopoDS::Wire(BRepBuilderAPI_Transform(wire, trsf).Shape());
    }
    gp_GTrsf gtrsf;
    gtrsf.SetValue(1, 1, t.a);
    gtrsf.SetValue(2, 1, t.b);
    gtrsf.SetValue(1, 2, t.c);
    gtrsf.SetValue(2, 2, t.d);
    gtrsf.SetValue(1, 4, t.e);
    gtrsf.SetValue(1, 4, t.f);  // sic: ocpsvg sets (1, 4) twice
    return TopoDS::Wire(BRepBuilderAPI_GTransform(wire, gtrsf).Shape());
}

std::vector<TopoDS_Wire> wires_from_svg_element(const svgdoc::ShapeElement& element) {
    std::vector<TopoDS_Wire> out;
    if (element.kind == svgdoc::ShapeKind::CIRCLE || element.kind == svgdoc::ShapeKind::ELLIPSE) {
        const gp_Pnt center(element.cx, element.cy, 0);
        const Handle(Geom_Curve) curve = element.rx == element.ry
                                             ? circle_curve(element.rx, center)
                                             : ellipse_curve(element.rx, element.ry, 360, 360, false, 0, center);
        out.push_back(transform_if_needed(wire_from_continuous_edges({edge_from_curve(curve)}, false), element.transform));
        return out;
    }
    for (const auto& wire : wires_from_svg_path(element.path)) out.push_back(transform_if_needed(wire, element.transform));
    return out;
}

/// ColorAndLabel._color: RGBA in [0, 1], or nothing for none and transparent black.
std::optional<std::array<double, 4>> element_color(const svgdoc::Color& color) {
    if (!color.value || *color.value == 0) return std::nullopt;
    return std::array<double, 4>{static_cast<double>(color.red()) / 255, static_cast<double>(color.green()) / 255,
                                 static_cast<double>(color.blue()) / 255, static_cast<double>(color.alpha()) / 255};
}

std::string expand_user(const std::string& path) {
    if (path.empty() || path[0] != '~') return path;
    if (path.size() > 1 && path[1] != '/') return path;
    const char* home = std::getenv("HOME");
    return home ? std::string(home) + path.substr(1) : path;
}

std::string read_file(const std::string& file_name) {
    std::ifstream in(expand_user(file_name), std::ios::binary);
    if (!in) throw std::runtime_error("No such file or directory: '" + file_name + "'");
    std::ostringstream content;
    content << in.rdbuf();
    return content.str();
}

ShapeList<Shape> import_svg_text(const std::string& text, const ImportSvgOptions& options) {
    // label_by: inkscape:xxx is the attribute in Inkscape's namespace
    std::string label_by = options.label_by;
    static const std::regex inkscape("^inkscape:(.+)");
    label_by = std::regex_replace(label_by, inkscape, "{http://www.inkscape.org/namespaces/inkscape}$1",
                                  std::regex_constants::format_first_only);

    return occt("import_svg", [&] {
        const svgdoc::Document doc = svgdoc::parse(text, options.ignore_visibility, 25.4);

        // find_shapes_svg_in_document: the document's size and position
        if (doc.has_viewbox) {
            auto absolute = [&doc](const char* key) {
                const auto it = doc.values.find(key);
                return it != doc.values.end() && !it->second.empty() && it->second.back() != '%';
            };
            const bool has_absolute_width = absolute("width");
            const bool has_absolute_height = absolute("height");
            if ((has_absolute_width && !has_absolute_height && doc.width != doc.vb_width) ||
                (has_absolute_height && !has_absolute_width && doc.height != doc.vb_height)) {
                warn("position of SVG document with only one of `width` or `height` may be incorrect");
            }
        }
        const bool has_width = doc.values.count("width") > 0;
        const bool has_height = doc.values.count("height") > 0;
        double width = 0.0, height = 0.0;
        if (has_width && has_height) {
            width = doc.width;
            height = doc.height;
        } else if (has_width && doc.has_viewbox) {
            width = doc.width;
            height = doc.vb_height * doc.width / doc.vb_width;
        } else if (has_height && doc.has_viewbox) {
            width = doc.vb_width * doc.height / doc.vb_height;
            height = doc.height;
        } else if (doc.has_viewbox) {
            width = doc.vb_width;
            height = doc.vb_height;
        }
        double x = 0.0, y = 0.0;
        if (doc.has_viewbox) {
            if (doc.vb_width == 0 || doc.vb_height == 0) throw std::invalid_argument("SVG import: float division by zero");
            x = doc.vb_x * width / doc.vb_width;
            y = doc.vb_y * height / doc.vb_height;
        }

        // import_svg_document's transform of the document
        gp_Trsf transform;
        transform.SetTranslation(gp_Vec(x, y, 0));
        if (options.flip_y) {
            gp_Trsf mirror;
            mirror.SetMirror(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)));
            transform.PreMultiply(mirror);
        }
        // transform_viewbox
        const double view_x = x;
        const double view_y = options.flip_y ? -y - height : y;

        // build123d: alignment of the viewbox
        const Vector doc_xy(view_x, view_y);
        const Vector doc_wh(width, height);
        const Vector offset = to_align_offset(doc_xy, doc_xy + doc_wh, Align3(options.align, Align::NONE));

        ShapeList<Shape> shapes;
        for (const auto& element : doc.shapes) {
            // ColorAndLabel
            const auto fill_color = element_color(element.fill);
            const auto stroke = element_color(element.stroke);
            const std::array<double, 4> stroke_color = stroke ? *stroke : std::array<double, 4>{0, 0, 0, 1};
            std::string label;
            if (const auto it = element.values.find(label_by); it != element.values.end()) label = it->second;

            const std::vector<TopoDS_Wire> wires = wires_from_svg_element(element);
            const bool is_filled = element.fill.value.has_value();
            std::vector<TopoDS_Shape> faces_or_wires;
            bool made_faces = false;
            if (is_filled) {
                try {
                    for (const auto& f : faces_from_wire_soup(wires)) faces_or_wires.push_back(f);
                    made_faces = true;
                } catch (const InvalidWiresForFace&) {
                    // ocpsvg logs "filled shape could not be converted to face"
                }
            }
            if (!made_faces) {
                for (const auto& w : wires) faces_or_wires.push_back(w);
            }
            for (const TopoDS_Shape& face_or_wire : faces_or_wires) {
#if OCC_VERSION_HEX >= 0x070700
                TopoDS_Shape mirrored = BRepBuilderAPI_Transform(face_or_wire, transform, false, false).Shape();
#else
                TopoDS_Shape mirrored = BRepBuilderAPI_Transform(face_or_wire, transform, false).Shape();
#endif
                if (options.flip_y) mirrored.Reverse();
                Shape shape(mirrored);
                if (offset.X != 0 || offset.Y != 0) shape = shape.translate(offset);
                const bool filled = mirrored.ShapeType() != TopAbs_WIRE && mirrored.ShapeType() != TopAbs_EDGE;
                const std::array<double, 4>& c = filled && fill_color ? *fill_color : stroke_color;
                shape.color = Color(c[0], c[1], c[2], c[3]);
                shape.label = label;
                shapes.push_back(shape);
            }
        }
        return shapes;
    });
}

// ---------------------------------------------------------------------------
// import_svg_as_buildline_code (svgpathtools.svg2paths)
// ---------------------------------------------------------------------------

/// Python's float(text), raising ValueError (std::invalid_argument) as Python does.
double py_float(const std::string& text) {
    std::string s = text;
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    s = s.substr(b);
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (s.empty() || end != s.c_str() + s.size() || s.find("0x") != std::string::npos ||
        s.find("0X") != std::string::npos) {
        throw std::invalid_argument("could not convert string to float: '" + text + "'");
    }
    return v;
}

using Attributes = std::vector<std::pair<std::string, std::string>>;

const std::string* attr(const Attributes& attributes, const std::string& key) {
    for (const auto& [k, v] : attributes) {
        if (k == key) return &v;
    }
    return nullptr;
}

/// minidom's getElementsByTagName: the elements with this name as written, in document order.
void elements_by_tag_name(const xml::Node& node, const std::string& name, std::vector<const xml::Node*>& out) {
    if (node.qname == name) out.push_back(&node);
    for (const auto& child : node.children) elements_by_tag_name(*child, name, out);
}

std::string ellipse2pathd(const Attributes& ellipse) {
    const std::string* cx_text = attr(ellipse, "cx");
    const std::string* cy_text = attr(ellipse, "cy");
    const std::string* rx_text = attr(ellipse, "rx");
    const std::string* ry_text = attr(ellipse, "ry");
    const std::string* r_text = attr(ellipse, "r");
    double rx, ry;
    if (r_text) {
        rx = ry = py_float(*r_text);
    } else {
        if (!rx_text || !ry_text) throw std::invalid_argument("float() argument must be a string or a real number");
        rx = py_float(*rx_text);
        ry = py_float(*ry_text);
    }
    const double cx = cx_text ? py_float(*cx_text) : 0.0;
    const double cy = cy_text ? py_float(*cy_text) : 0.0;
    auto str = [](double v) { return pyfmt::repr(v); };
    std::string d;
    d += "M" + str(cx - rx) + "," + str(cy);
    d += "a" + str(rx) + "," + str(ry) + " 0 1,0 " + str(2 * rx) + ",0";
    d += "a" + str(rx) + "," + str(ry) + " 0 1,0 " + str(-2 * rx) + ",0";
    return d + "z";
}

std::string polyline2pathd(const Attributes& polyline, bool is_polygon) {
    static const std::regex coord_pair(
        R"(([\+-]?\d*[\.\d]\d*[eE][\+-]?\d+|[\+-]?\d*[\.\d]\d*)(?:\s*,\s*|\s+|(?=-))([\+-]?\d*[\.\d]\d*[eE][\+-]?\d+|[\+-]?\d*[\.\d]\d*))");
    const std::string* text = attr(polyline, "points");
    const std::string points_text = text ? *text : std::string();
    std::vector<std::pair<std::string, std::string>> points;
    for (auto it = std::sregex_iterator(points_text.begin(), points_text.end(), coord_pair);
         it != std::sregex_iterator(); ++it) {
        points.emplace_back((*it)[1].str(), (*it)[2].str());
    }
    if (points.empty()) return "";
    const bool closed = py_float(points.front().first) == py_float(points.back().first) &&
                        py_float(points.front().second) == py_float(points.back().second);
    if (is_polygon && closed) points.push_back(points.front());
    std::string d = "M";
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i) d += "L";
        d += points[i].first + " " + points[i].second;
    }
    if (is_polygon || closed) d += "z";
    return d;
}

std::string rect2pathd(const Attributes& rect) {
    auto value = [&rect](const char* key) {
        const std::string* v = attr(rect, key);
        return v ? py_float(*v) : 0.0;
    };
    const double x = value("x"), y = value("y");
    const double w = value("width"), h = value("height");
    auto str = [](double v) { return pyfmt::repr(v); };
    const std::string* rx_text = attr(rect, "rx");
    const std::string* ry_text = attr(rect, "ry");
    if (rx_text || ry_text) {
        // rx = ry or 0. (an empty string is false)
        std::optional<std::string> rx_s = rx_text ? std::optional<std::string>(*rx_text) : std::nullopt;
        std::optional<std::string> ry_s = ry_text ? std::optional<std::string>(*ry_text) : std::nullopt;
        if (!rx_s) rx_s = (ry_s && !ry_s->empty()) ? *ry_s : std::string("0.");
        if (!ry_s) ry_s = (rx_s && !rx_s->empty()) ? *rx_s : std::string("0.");
        const double rx = py_float(*rx_s), ry = py_float(*ry_s);
        std::string d = "M " + str(x + rx) + " " + str(y) + " ";
        d += "L " + str(x + w - rx) + " " + str(y) + " ";
        d += "A " + str(rx) + " " + str(ry) + " 0 0 1 " + str(x + w) + " " + str(y + ry) + " ";
        d += "L " + str(x + w) + " " + str(y + h - ry) + " ";
        d += "A " + str(rx) + " " + str(ry) + " 0 0 1 " + str(x + w - rx) + " " + str(y + h) + " ";
        d += "L " + str(x + rx) + " " + str(y + h) + " ";
        d += "A " + str(rx) + " " + str(ry) + " 0 0 1 " + str(x) + " " + str(y + h - ry) + " ";
        d += "L " + str(x) + " " + str(y + ry) + " ";
        d += "A " + str(rx) + " " + str(ry) + " 0 0 1 " + str(x + rx) + " " + str(y) + " z";
        return d;
    }
    const double x1 = x + w, y2 = y + h;
    return "M" + str(x) + " " + str(y) + " L " + str(x1) + " " + str(y) + " L " + str(x1) + " " + str(y2) + " L " +
           str(x) + " " + str(y2) + " z";
}

/// Python's str.isidentifier for ASCII names (other characters are taken as letters).
bool is_identifier(const std::string& s) {
    if (s.empty()) return false;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        const bool ok = std::isalpha(c) || c == '_' || c >= 0x80 || (i > 0 && std::isdigit(c));
        if (!ok) return false;
    }
    return true;
}

}  // namespace

ShapeList<Shape> import_svg(const std::string& svg_file, const ImportSvgOptions& options) {
    return import_svg_text(read_file(svg_file), options);
}

ShapeList<Shape> import_svg(std::istream& svg_stream, const ImportSvgOptions& options) {
    std::ostringstream content;
    content << svg_stream.rdbuf();
    return import_svg_text(content.str(), options);
}

std::pair<std::string, std::string> import_svg_as_buildline_code(const std::string& file_name, int precision) {
    const auto root = xml::parse(read_file(file_name));
    auto elements = [&root](const char* name) {
        std::vector<const xml::Node*> out;
        elements_by_tag_name(*root, name, out);
        return out;
    };
    // svgpathtools.svg2paths
    std::vector<std::string> d_strings;
    for (const auto* el : elements("path")) {
        const std::string* d = attr(el->raw_attrib, "d");
        if (!d) throw std::invalid_argument("KeyError: 'd'");
        d_strings.push_back(*d);
    }
    for (const auto* el : elements("polyline")) d_strings.push_back(polyline2pathd(el->raw_attrib, false));
    for (const auto* el : elements("polygon")) d_strings.push_back(polyline2pathd(el->raw_attrib, true));
    for (const auto* el : elements("line")) {
        std::string d;
        for (const char* key : {"x1", "y1", "x2", "y2"}) {
            const std::string* v = attr(el->raw_attrib, key);
            if (!v) throw std::invalid_argument(std::string("KeyError: '") + key + "'");
        }
        d = "M" + *attr(el->raw_attrib, "x1") + " " + *attr(el->raw_attrib, "y1") + "L" + *attr(el->raw_attrib, "x2") +
            " " + *attr(el->raw_attrib, "y2");
        d_strings.push_back(d);
    }
    for (const auto* el : elements("ellipse")) d_strings.push_back(ellipse2pathd(el->raw_attrib));
    for (const auto* el : elements("circle")) d_strings.push_back(ellipse2pathd(el->raw_attrib));
    for (const auto* el : elements("rect")) d_strings.push_back(rect2pathd(el->raw_attrib));

    auto g = [precision](double v) { return pyfmt::format_g(v, precision); };
    auto fmt_point = [&g](svgpath::Cx v) { return "(" + g(v.re) + ", " + g(v.im) + ")"; };

    const std::string base = std::filesystem::path(file_name).filename().string();
    std::string builder_name = base.substr(0, base.find('.'));
    if (!is_identifier(builder_name)) builder_name = "builder";
    std::vector<std::string> code{"from build123d import *", "with BuildLine() as " + builder_name + ":"};
    for (const auto& d : d_strings) {
        for (const svgpath::Segment& curve : svgpath::parse_path(d)) {
            switch (curve.type) {
                case svgpath::SegmentType::ARC: {
                    const svgpath::Arc& arc = curve.arc;
                    const double x_radius = std::fabs(arc.radius.re);
                    const double y_radius = std::fabs(arc.radius.im);
                    if (std::fabs(x_radius - y_radius) <= TOLERANCE) {
                        code.push_back("    RadiusArc(" + fmt_point(arc.start) + ", " + fmt_point(arc.end) + ", " +
                                       g(x_radius) + ", " + (arc.large_arc ? "False" : "True") + ")");
                    } else {
                        code.push_back("    EllipticalCenterArc(" + fmt_point(arc.center) + ", " + g(x_radius) + ", " +
                                       g(y_radius) + ", start_angle=" + g(arc.theta) + ", arc_size=" + g(arc.delta) +
                                       ", rotation=" + g(arc.rotation) + ")");
                    }
                    break;
                }
                case svgpath::SegmentType::LINE:
                    code.push_back("    Line(" + fmt_point(curve.start) + "," + fmt_point(curve.end) + ")");
                    break;
                case svgpath::SegmentType::CUBIC:
                    code.push_back("    Bezier(" + fmt_point(curve.start) + "," + fmt_point(curve.c1) + "," +
                                   fmt_point(curve.c2) + "," + fmt_point(curve.end) + ")");
                    break;
                case svgpath::SegmentType::QUADRATIC:
                    code.push_back("    Bezier(" + fmt_point(curve.start) + "," + fmt_point(curve.c1) + "," +
                                   fmt_point(curve.end) + ")");
                    break;
            }
        }
    }
    std::string text;
    for (std::size_t i = 0; i < code.size(); ++i) text += (i ? "\n" : "") + code[i];
    return {text, builder_name};
}

}  // namespace build123d
