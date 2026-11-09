#include <algorithm>

#include <Font_FontMgr.hxx>
#include <Font_SystemFont.hxx>
#include <Graphic3d_HorizontalTextAlignment.hxx>
#include <Graphic3d_VerticalTextAlignment.hxx>
#include <NCollection_String.hxx>
#include <StdPrs_BRepFont.hxx>
#include <StdPrs_BRepTextBuilder.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopoDS_Iterator.hxx>

#include "build123d/objects.hpp"
#include "build123d/text.hpp"
#include "internal.hpp"
#include "py_compat.hpp"

namespace build123d {

using namespace detail;

// ---------------------------------------------------------------------------
// Compound
// ---------------------------------------------------------------------------

TopoDS_Shape Compound::make_compound(const std::vector<TopoDS_Shape>& shapes) {
    return detail::make_compound(shapes);
}

TopoDS_Shape Compound::ensure_compound(const TopoDS_Shape& obj) {
    if (obj.IsNull() || obj.ShapeType() == TopAbs_COMPOUND) return obj;
    return detail::make_compound({obj});
}

Compound::Compound(const TopoDS_Shape& obj) : Mixin3D(ensure_compound(obj)) {}

Compound Compound::make_assembly(const std::vector<Shape>& kids, const std::string& name) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& c : kids) {
        if (!c.is_null()) topods.push_back(c.wrapped());
    }
    Compound result(make_compound(topods));
    result.children = kids;
    result.label = name;
    return result;
}

void Compound::check_dim(int expected, const char* name) const {
    if (obj_.IsNull() || !*this) return;
    const int d = dim();
    if (d >= 0 && d != expected) {
        throw std::invalid_argument("Can't convert a Compound of dimension " + std::to_string(d) + " into a " +
                                    name);
    }
}

std::size_t Compound::size() const {
    std::size_t n = 0;
    if (obj_.IsNull()) return n;
    for (TopoDS_Iterator it(obj_); it.More(); it.Next()) ++n;
    return n;
}

ShapeList<Shape> Compound::elements() const {
    ShapeList<Shape> out;
    if (obj_.IsNull()) return out;
    for (TopoDS_Iterator it(obj_); it.More(); it.Next()) out.emplace_back(it.Value());
    return out;
}

Shape Compound::unwrap(bool fully) const {
    if (size() == 1) {
        TopoDS_Iterator it(obj_);
        Shape single_element(it.Value());
        detail::copy_attributes_to(*this, single_element);
        // If the single element is another Compound, unwrap it recursively
        if (it.Value().ShapeType() == TopAbs_COMPOUND) {
            Shape unwrapped = detail::rewrap<Compound>(single_element).unwrap(fully);
            if (!fully) unwrapped = Shape(unwrapped.wrapped());  // build123d's type(self)(...)
            detail::copy_attributes_to(*this, unwrapped);
            return unwrapped;
        }
        if (fully) return single_element;
        return *this;
    }
    // If there are no elements or more than one element, return self
    return *this;
}

double Compound::volume() const { return Shape::volume(); }

// ---------------------------------------------------------------------------
// Curve / Sketch / Part
// ---------------------------------------------------------------------------

Curve::Curve(const TopoDS_Shape& obj) : Compound(obj) {}
Curve::Curve(const Compound& compound) : Compound(compound) { check_dim(1, "Curve"); }
Curve::Curve(const Edge& edge) : Compound(edge.wrapped()) {
    label = edge.label;
    color.set_raw(edge.color.raw());
}
Curve::Curve(const Wire& wire) : Compound(wire.wrapped()) {
    label = wire.label;
    color.set_raw(wire.color.raw());
}

ShapeList<Wire> Curve::wires(Select select) const { return Wire::combine(std::vector<Edge>(edges(select))); }
double Curve::length() const { return obj_.IsNull() ? 0.0 : linear_length(obj_); }
Vector Curve::position_at(double position, PositionMode mode) const {
    return Wire(std::vector<Edge>(edges())).position_at(position, mode);
}
Vector Curve::tangent_at(double position, PositionMode mode) const {
    return Wire(std::vector<Edge>(edges())).tangent_at(position, mode);
}
Location Curve::location_at(double position, PositionMode mode, FrameMethod frame_method) const {
    return Wire(std::vector<Edge>(edges())).location_at(position, mode, frame_method);
}

Vector operator%(const Curve& curve, double position) { return curve.tangent_at(position); }
Location operator^(const Curve& curve, double position) { return curve.location_at(position); }

Sketch::Sketch(const TopoDS_Shape& obj) : Compound(obj) {}
Sketch::Sketch(const Compound& compound) : Compound(compound) { check_dim(2, "Sketch"); }
Sketch::Sketch(const Face& face) : Compound(face.wrapped()) {
    label = face.label;
    color.set_raw(face.color.raw());
}

Part::Part(const TopoDS_Shape& obj) : Compound(obj) {}
Part::Part(const Compound& compound) : Compound(compound) { check_dim(3, "Part"); }
Part::Part(const Solid& solid) : Compound(solid.wrapped()) {
    label = solid.label;
    color.set_raw(solid.color.raw());
}

// ---------------------------------------------------------------------------
// Text
// ---------------------------------------------------------------------------

namespace {

Font_FontAspect to_aspect(FontStyle style) {
    switch (style) {
        case FontStyle::REGULAR: return Font_FA_Regular;
        case FontStyle::BOLD: return Font_FA_Bold;
        case FontStyle::ITALIC: return Font_FA_Italic;
        case FontStyle::BOLDITALIC: return Font_FA_BoldItalic;
    }
    return Font_FA_Regular;
}

Shape position_glyph(const Shape& glyph, const Mixin1D& path, double position) {
    const BoundBox bbox = glyph.bounding_box();
    const Vector face_bottom_center((bbox.min.X + bbox.max.X) / 2, 0, 0);
    const double relative = position + face_bottom_center.X / path.length();
    const Vector tangent = path.tangent_at(relative);
    const double angle = Vector(1, 0, 0).get_signed_angle(tangent);
    const Vector wire_position = path.position_at(relative);
    return glyph.translate(wire_position - face_bottom_center).rotate(Axis(wire_position, Vector(0, 0, 1)), -angle);
}

}  // namespace

namespace {

std::string text_align_name(TextAlign a) {
    switch (a) {
        case TextAlign::BOTTOM: return "TextAlign.BOTTOM";
        case TextAlign::CENTER: return "TextAlign.CENTER";
        case TextAlign::LEFT: return "TextAlign.LEFT";
        case TextAlign::RIGHT: return "TextAlign.RIGHT";
        case TextAlign::TOP: return "TextAlign.TOP";
        case TextAlign::TOPFIRSTLINE: return "TextAlign.TOPFIRSTLINE";
    }
    return "TextAlign";
}

/// make_text's _make_face: the face of the combined edges, facing +Z.
Face make_text_face(const std::vector<Edge>& edges) {
    Face face(Wire::combine(edges).at(0));
    if (face.normal_at().Z < 0) face = -face;
    return face;
}

}  // namespace

std::tuple<std::string, std::string, Handle(Font_SystemFont)> Compound::resolve_font(const std::string& font,
                                                                                    const std::string& font_path,
                                                                                    FontStyle font_style) {
    FontManager manager;
    std::string selected_name;
    if (!font_path.empty() && !manager.check_font(font_path).IsNull()) {
        const std::vector<std::string> face_names = manager.register_font(font_path, true, false);
        if (std::find(face_names.begin(), face_names.end(), font) != face_names.end()) {
            selected_name = font;
        } else if (face_names.empty()) {
            throw std::out_of_range("list index out of range");
        } else {
            selected_name = face_names[0];
        }
    } else {
        selected_name = font;
    }
    const Handle(Font_SystemFont) system_font = manager.find_font(selected_name, font_style);
    if (system_font.IsNull()) throw std::invalid_argument("No font found for '" + selected_name + "'");
    return {system_font->FontName().ToCString(), system_font->FontPath(to_aspect(font_style)).ToCString(),
            system_font};
}

Compound Compound::make_text(const std::string& txt, double font_size, const std::string& font,
                             const std::string& font_path, FontStyle font_style,
                             std::pair<TextAlign, TextAlign> text_align, std::optional<Align2> align,
                             double position_on_path, const std::optional<Mixin1D>& text_path,
                             double single_line_width) {
    const auto [resolved_font, resolved_font_path, system_font] = resolve_font(font, font_path, font_style);

    // Validate TextAlign parameters
    Graphic3d_HorizontalTextAlignment h_align;
    switch (text_align.first) {
        case TextAlign::LEFT: h_align = Graphic3d_HTA_LEFT; break;
        case TextAlign::CENTER: h_align = Graphic3d_HTA_CENTER; break;
        case TextAlign::RIGHT: h_align = Graphic3d_HTA_RIGHT; break;
        default:
            throw std::invalid_argument("Horizontal TextAlign must be LEFT, CENTER, or RIGHT. Got " +
                                        text_align_name(text_align.first));
    }
    Graphic3d_VerticalTextAlignment v_align;
    switch (text_align.second) {
        case TextAlign::BOTTOM: v_align = Graphic3d_VTA_BOTTOM; break;
        case TextAlign::CENTER: v_align = Graphic3d_VTA_CENTER; break;
        case TextAlign::TOP: v_align = Graphic3d_VTA_TOP; break;
        case TextAlign::TOPFIRSTLINE: v_align = Graphic3d_VTA_TOPFIRSTLINE; break;
        default:
            throw std::invalid_argument("Vertical TextAlign must be BOTTOM, CENTER, TOP, or TOPFIRSTLINE. Got " +
                                        text_align_name(text_align.second));
    }

    // Write text to shape
    Compound text_flat = occt("Text", [&] {
        StdPrs_BRepFont brep_font(NCollection_String(system_font->FontName().ToCString()), to_aspect(font_style),
                                  font_size, Font_StrictLevel_Any);
        if (system_font->IsSingleStrokeFont()) brep_font.SetCompositeCurveMode(false);
        StdPrs_BRepTextBuilder builder;
        return Compound(builder.Perform(brep_font, NCollection_String(txt.c_str()), gp_Ax3(), h_align, v_align));
    });

    // Align the text from the bounding box
    if (align) text_flat = text_flat.translate(text_flat.bounding_box().to_align_offset(*align));

    // Place text on path
    if (text_path && !text_path->is_null()) {
        std::vector<Shape> glyphs;
        for (const auto& g : text_flat.get_top_level_shapes()) {
            glyphs.push_back(position_glyph(g, *text_path, position_on_path));
        }
        text_flat = Compound(glyphs);
    }

    // Outline single line text; offset_2d's distance is a radius, single_line_width the width
    if (system_font->IsSingleStrokeFont() && single_line_width > 0) {
        std::vector<TopoDS_Shape> outline;
        for (const auto& e : text_flat.edges()) {
            outline.push_back(make_text_face(e.offset_2d(single_line_width / 2).edges()).wrapped());
        }
        // Compound([]) + outline: the faces fused (fuzzy) and cleaned
        if (outline.size() <= 1) {
            text_flat = Compound(detail::make_compound(outline));
        } else {
            text_flat = Compound(bool_op(BoolOp::FUSE, {outline[0]},
                                         std::vector<TopoDS_Shape>(outline.begin() + 1, outline.end()), TOLERANCE));
        }
        for (const auto& f : text_flat.get_top_level_shapes()) {
            if (!f.is_valid()) {
                throw std::invalid_argument("single_line_width (" + py_float_repr(single_line_width) +
                                            ") is too large for the text and produces invalid faces. Try a "
                                            "smaller width");
            }
        }
    }
    return text_flat;
}

}  // namespace build123d
