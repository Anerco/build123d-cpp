// build123d-cpp: SVG and DXF import. Mirrors importers.py (import_svg,
// import_svg_as_buildline_code) and import_dxf.py. The SVG reader is a port of the parts of
// svgelements and ocpsvg that build123d uses; no Python packages are needed.
//
//   ShapeList<Shape> shapes = import_svg("drawing.svg");
//   for (const Shape& s : shapes) { ... s.label, s.color ... }
//   Sketch sketch = Sketch(shapes.faces());
#pragma once

#include <iosfwd>
#include <string>
#include <utility>

#include "build123d/topology.hpp"

namespace build123d {

/// The keyword arguments of import_svg.
struct ImportSvgOptions {
    /// Flip objects to compensate for SVG's top-left origin (y down).
    bool flip_y = true;
    /// Alignment of the SVG's viewBox; Align::NONE keeps its origin at (0, 0, 0) (Python's
    /// align=None).
    Align2 align = Align::MIN;
    /// Also import hidden elements (display: none, visibility: hidden).
    bool ignore_visibility = false;
    /// The XML attribute used for the shapes' labels: "id", "class", "inkscape:label" (the
    /// labels of Inkscape's "Layers and Objects" panel) or any other attribute name.
    std::string label_by = "id";
};

/// The shapes of an SVG file (importers.py import_svg): a Face for each filled shape, a Wire
/// for each outline, with the shape's color and label. Each element of the list is a Face or
/// a Wire (get the type with cast_shape<Face>(s) / cast_shape<Wire>(s), or use .faces() and
/// .wires()). Raises std::runtime_error if the file can't be read and std::invalid_argument
/// if it isn't an SVG document svgelements could read.
ShapeList<Shape> import_svg(const std::string& svg_file, const ImportSvgOptions& options = {});
/// import_svg of an SVG document read from a stream (Python's TextIO argument).
ShapeList<Shape> import_svg(std::istream& svg_stream, const ImportSvgOptions& options = {});

/// The paths of an SVG file as Python build123d BuildLine code (importers.py
/// import_svg_as_buildline_code, through svgpathtools): returns the code and the name of the
/// BuildLine builder, exactly as Python does. Values are rounded to `precision` significant
/// digits.
std::pair<std::string, std::string> import_svg_as_buildline_code(const std::string& file_name,
                                                                  int precision = 6);

/// The entities of a DXF file's model space as shapes (import_dxf.py import_dxf; the file is
/// read as ezdxf reads it): LINE, ARC, CIRCLE, ELLIPSE, SPLINE and the (LW)POLYLINE segments
/// become edges or wires, POINT a Vertex, TEXT and MTEXT Text sketches, HATCH its boundary
/// paths, and INSERT the transformed objects of its block. Other entities are skipped with a
/// warning ("Unable to convert ..."). As in build123d 0.13, 3DFACE, SOLID and TRACE make it
/// raise (std::out_of_range). Raises std::runtime_error if the file can't be read or isn't a
/// DXF file, std::invalid_argument if its structure is invalid.
ShapeList<Shape> import_dxf(const std::string& dxf_file);
/// import_dxf of DXF data read from a stream (ASCII or binary).
ShapeList<Shape> import_dxf(std::istream& dxf_stream);

}  // namespace build123d
