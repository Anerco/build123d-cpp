// DXF import (import_dxf.py), reading the file as ezdxf does (dxf_reader.cpp).
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "build123d/importers.hpp"
#include "build123d/objects.hpp"
#include "build123d/operations.hpp"
#include "build123d/warnings.hpp"
#include "dxf_reader.hpp"
#include "internal.hpp"

namespace build123d {

namespace {

using dxf::Entity;
using dxf::Tag;

constexpr double kPi = 3.141592653589793;
constexpr double kTau = 2 * kPi;

double degrees(double x) { return x * (180.0 / kPi); }

double py_mod(double x, double y) {
    double mod = std::fmod(x, y);
    if (mod != 0.0) {
        if ((y < 0) != (mod < 0)) mod += y;
    } else {
        mod = std::copysign(0.0, y);
    }
    return mod;
}

Vector point(const Tag* t, const Vector& default_value = Vector(0, 0, 0)) {
    if (t == nullptr) return default_value;
    if (t->kind != Tag::Kind::POINT) throw std::invalid_argument("DXF: expected a point for group code " + std::to_string(t->code));
    return Vector(t->x, t->y, t->dims == 3 ? t->z : 0.0);
}

double number(const Tag* t, double default_value) {
    if (t == nullptr) return default_value;
    if (t->kind == Tag::Kind::INT) return static_cast<double>(t->i);
    if (t->kind == Tag::Kind::FLOAT || t->kind == Tag::Kind::POINT) return t->x;
    throw std::invalid_argument("DXF: expected a number for group code " + std::to_string(t->code));
}

long long integer(const Tag* t, long long default_value) {
    if (t == nullptr) return default_value;
    if (t->kind == Tag::Kind::INT) return t->i;
    if (t->kind == Tag::Kind::FLOAT) return static_cast<long long>(t->x);
    throw std::invalid_argument("DXF: expected an integer for group code " + std::to_string(t->code));
}

std::string text(const Tag* t, const std::string& default_value) {
    if (t == nullptr) return default_value;
    return t->text;
}

/// Any of the shapes import_dxf creates.
using Shapes = std::vector<Shape>;

// ---------------------------------------------------------------------------
// Entity handlers
// ---------------------------------------------------------------------------

Shapes process_arc(const Entity& e) {
    // entity.angles(3): start and end in [0, 360), counter-clockwise
    const double start_angle = py_mod(number(e.get(50), 0), 360);
    double stop = py_mod(number(e.get(51), 360), 360);
    if (stop <= start_angle) stop += 360;
    const double start = py_mod(start_angle, 360);
    const double end = py_mod(stop, 360);
    const double arc_size = py_mod(end - start + 360.0, 360.0);
    return {CenterArc(point(e.get(10)), number(e.get(40), 1), start, arc_size)};
}

Shapes process_circle(const Entity& e) {
    return {Circle(number(e.get(40), 1)).edge().moved(Pos(point(e.get(10))))};
}

Shapes process_ellipse(const Entity& e) {
    const Vector center = point(e.get(10));
    const Vector major_axis = point(e.get(11), Vector(1, 0, 0));
    const double x_radius = std::pow(std::pow(major_axis.X, 2) + std::pow(major_axis.Y, 2), 0.5);
    const double y_radius = x_radius * number(e.get(40), 1.0);
    const double rotation = degrees(std::atan2(major_axis.Y, major_axis.X));
    const double start_param = number(e.get(41), 0);
    const double end_param = number(e.get(42), kTau);
    const double start_angle = degrees(start_param);
    double arc_size = degrees(end_param - start_param);
    arc_size = py_mod(arc_size + 360.0, 360.0);
    return {EllipticalCenterArc(center, x_radius, y_radius, start_angle, arc_size, rotation)};
}

Shapes process_line(const Entity& e) {
    const Vector start = point(e.get(10));
    const Vector end = point(e.get(11));
    if ((start - end).length() < TOLERANCE) {
        warn("Skipping degenerate LINE");
        return {};
    }
    return {Line(start, end)};
}

/// _convert_bulge_polyline: lines and arcs between (x, y, bulge) vertices.
Shapes convert_bulge_polyline(const std::vector<std::array<double, 3>>& points, bool closed, double z_value,
                              const std::string& label) {
    std::vector<Edge> edges;
    const std::size_t n = points.size();
    const std::size_t segment_count = closed ? n : (n == 0 ? 0 : n - 1);
    for (std::size_t i = 0; i < segment_count; ++i) {
        const auto& start_data = points[i];
        const auto& end_data = points[(i + 1) % n];
        const Vector start_point(start_data[0], start_data[1], z_value);
        const Vector end_point(end_data[0], end_data[1], z_value);
        const double bulge = -1 * start_data[2];
        const double dist = std::hypot(start_point.X - end_point.X, start_point.Y - end_point.Y,
                                       start_point.Z - end_point.Z);
        if (dist < TOLERANCE) continue;
        if (std::fabs(bulge) < TOLERANCE) {
            edges.push_back(Line(start_point, end_point));
        } else {
            const double sagitta = bulge * dist / 2;
            edges.push_back(SagittaArc(start_point, end_point, sagitta));
        }
    }
    if (edges.empty()) {
        warn("Skipping degenerate " + label);
        return {};
    }
    if (edges.size() == 1) return {edges[0]};
    return {Wire(edges)};
}

Shapes process_lwpolyline(const Entity& e) {
    const double z_value = number(e.get(38), 0);
    // LWPolylinePoints.from_tags: a vertex per 10, with the 40/41/42 values after it
    std::vector<std::array<double, 3>> points;
    bool have_point = false;
    std::array<double, 3> current{0, 0, 0};
    double bulge = 0;
    const auto data = e.subclass(2);  // checked when the file was read
    for (const Tag& t : *data) {
        if (t.code == 10) {
            if (have_point) points.push_back({current[0], current[1], bulge});
            current = {t.x, t.y, 0};
            bulge = 0;
            have_point = true;
        } else if (t.code == 42) {
            bulge = number(&t, 0);
        }
    }
    if (have_point) points.push_back({current[0], current[1], bulge});
    if (points.size() < 2) {
        warn("Skipping degenerate LWPOLYLINE");
        return {};
    }
    const bool closed = (integer(e.get(70), 0) & 1) != 0;
    return convert_bulge_polyline(points, closed, z_value, "LWPOLYLINE");
}

Shapes process_point(const Entity& e) {
    const Vector p = point(e.get(10));
    return {Vertex(p.X, p.Y, p.Z)};
}

Shapes process_polyline(const Entity& e) {
    const long long flags = integer(e.get(70), 0);
    std::string mode = "AcDb2dPolyline";
    if (flags & 8) {
        mode = "AcDb3dPolyline";
    } else if (flags & 16) {
        mode = "AcDbPolygonMesh";
    } else if (flags & 64) {
        mode = "AcDbPolyFaceMesh";
    }
    if (mode != "AcDb2dPolyline") throw std::invalid_argument("Unsupported POLYLINE mode: " + mode);
    const auto& vertices = e.linked;
    if (vertices.size() < 2) {
        warn("Skipping degenerate POLYLINE");
        return {};
    }
    std::vector<std::array<double, 3>> points;
    for (const Entity& v : vertices) {
        const Tag* location = v.get(10);
        if (location == nullptr) throw std::invalid_argument("DXF: VERTEX without location");
        points.push_back({location->x, location->y, number(v.get(42), 0)});
    }
    const double z_value = point(vertices[0].get(10)).Z;
    return convert_bulge_polyline(points, (flags & 1) != 0, z_value, "POLYLINE");
}

// ---- HATCH

struct HatchEdge {
    int type = 0;  // 1 line, 2 arc, 3 ellipse, 4 spline
    double sx = 0, sy = 0, ex = 0, ey = 0;  // line
    double cx = 0, cy = 0, radius = 1.0, start_angle = 0.0, end_angle = 360.0;
    double mx = 1.0, my = 0.0, ratio = 1.0;
    bool ccw = true;
    long long degree = 3, periodic = 0;
    std::vector<double> knots, weights;
    std::vector<std::array<double, 2>> control_points;
};

struct HatchPath {
    long long flags = 0;
    bool polyline = false;
    bool is_closed = false;
    std::vector<std::array<double, 3>> vertices;
    std::vector<HatchEdge> edges;
};

bool is_path_code(int code) {
    switch (code) {
        case 10: case 11: case 12: case 13: case 40: case 42: case 50: case 51: case 72: case 73: case 74:
        case 92: case 93: case 94: case 95: case 96: case 97: case 330: return true;
        default: return false;
    }
}

/// pop_source_boundary_objects_tags
void pop_source_boundary_objects(std::vector<Tag>& tags) {
    std::vector<Tag> popped;
    while (!tags.empty()) {
        const Tag last = tags.back();
        if (last.code == 330 || last.code == 97) {
            tags.pop_back();
            if (last.code == 97) return;
            popped.push_back(last);
        } else {
            break;
        }
    }
    // no length tag: the tags stay removed (ezdxf pops them) but nothing is returned
}

HatchEdge load_hatch_edge(int type, const std::vector<Tag>& tags) {
    HatchEdge edge;
    edge.type = type;
    double start = 0.0, end = 0.0;
    for (const Tag& t : tags) {
        switch (type) {
            case 1:
                if (t.code == 10) edge.sx = t.x, edge.sy = t.y;
                if (t.code == 11) edge.ex = t.x, edge.ey = t.y;
                break;
            case 2:
            case 3:
                if (t.code == 10) edge.cx = t.x, edge.cy = t.y;
                if (type == 2 && t.code == 40) edge.radius = number(&t, 0);
                if (type == 3 && t.code == 11) edge.mx = t.x, edge.my = t.y;
                if (type == 3 && t.code == 40) edge.ratio = number(&t, 0);
                if (t.code == 50) start = number(&t, 0);
                if (t.code == 51) end = number(&t, 0);
                if (t.code == 73) edge.ccw = integer(&t, 0) != 0;
                break;
            case 4:
                if (t.code == 94) edge.degree = integer(&t, 3);
                if (t.code == 74) edge.periodic = integer(&t, 0);
                if (t.code == 40) edge.knots.push_back(number(&t, 0));
                if (t.code == 42) edge.weights.push_back(number(&t, 0));
                if (t.code == 10) edge.control_points.push_back({t.x, t.y});
                break;
            default: break;
        }
    }
    if (type == 2 || type == 3) {
        if (edge.ccw) {
            edge.start_angle = start;
            edge.end_angle = end;
        } else {
            edge.start_angle = 360.0 - end;
            edge.end_angle = 360.0 - start;
        }
    }
    return edge;
}

std::vector<HatchPath> load_hatch_paths(const Entity& e) {
    const auto data = e.subclass(2);  // checked when the file was read
    const std::vector<Tag>& tags = *data;
    std::size_t start = tags.size();
    for (std::size_t k = 0; k < tags.size(); ++k) {
        if (tags[k].code == 91) {
            start = k;
            break;
        }
    }
    if (start == tags.size()) {
        throw dxf::StructureError("HATCH: Missing required DXF tag 'Number of boundary paths (loops)' (code=91).");
    }
    std::vector<Tag> path_tags;
    for (std::size_t k = start + 1; k < tags.size() && is_path_code(tags[k].code); ++k) path_tags.push_back(tags[k]);
    // group_tags(splitcode=92)
    std::vector<std::vector<Tag>> groups;
    for (const Tag& t : path_tags) {
        if (t.code == 92) {
            groups.push_back({t});
        } else if (!groups.empty()) {
            groups.back().push_back(t);
        }
    }
    std::vector<HatchPath> paths;
    for (auto& group : groups) {
        HatchPath path;
        path.flags = group[0].i;
        path.polyline = (path.flags & 2) != 0;
        pop_source_boundary_objects(group);
        if (path.polyline) {
            for (const Tag& t : group) {
                if (t.code == 10) {
                    path.vertices.push_back({t.x, t.y, 0.0});
                } else if (t.code == 42) {
                    if (path.vertices.empty()) throw std::out_of_range("pop from empty list");
                    path.vertices.back()[2] = number(&t, 0);
                } else if (t.code == 73) {
                    path.is_closed = integer(&t, 0) != 0;
                } else if (t.code == 92) {
                    path.flags = integer(&t, 0);
                }
            }
        } else {
            // group_tags(splitcode=72): an edge per 72 tag
            std::vector<std::vector<Tag>> edge_groups;
            for (const Tag& t : group) {
                if (t.code == 72) {
                    edge_groups.push_back({t});
                } else if (!edge_groups.empty()) {
                    edge_groups.back().push_back(t);
                }
            }
            for (const auto& edge_tags : edge_groups) {
                const long long edge_type = integer(&edge_tags[0], 0);
                if (0 < edge_type && edge_type < 5) {
                    path.edges.push_back(load_hatch_edge(static_cast<int>(edge_type),
                                                         std::vector<Tag>(edge_tags.begin() + 1, edge_tags.end())));
                }
            }
        }
        paths.push_back(std::move(path));
    }
    return paths;
}

Edge convert_hatch_edge(const HatchEdge& edge, double z_value) {
    switch (edge.type) {
        case 1: return Line(Vector(edge.sx, edge.sy, z_value), Vector(edge.ex, edge.ey, z_value));
        case 2: {
            double arc_size = edge.end_angle - edge.start_angle;
            if (!edge.ccw) arc_size = -arc_size;
            return CenterArc(Vector(edge.cx, edge.cy, z_value), edge.radius, edge.start_angle, arc_size);
        }
        case 3: {
            const Vector major_axis(edge.mx, edge.my, 0);
            const double x_radius = major_axis.length();
            const double rotation = degrees(std::atan2(major_axis.Y, major_axis.X));
            double arc_size = edge.end_angle - edge.start_angle;
            if (!edge.ccw) arc_size = -arc_size;
            return EllipticalCenterArc(Vector(edge.cx, edge.cy, z_value), x_radius, x_radius * edge.ratio,
                                       edge.start_angle, arc_size, rotation);
        }
        default: {
            std::vector<Vector> control_points;
            for (const auto& p : edge.control_points) control_points.emplace_back(p[0], p[1], z_value);
            return BSpline(control_points, edge.knots, static_cast<int>(edge.degree), edge.weights, edge.periodic != 0);
        }
    }
}

Shapes process_hatch(const Entity& e) {
    const double z_value = point(e.get(10)).Z;
    const long long hatch_style = integer(e.get(75), 0);
    const std::vector<HatchPath> paths = load_hatch_paths(e);
    // BoundaryPaths.rendering_paths
    struct Ranked {
        int type;
        std::size_t index;
    };
    std::vector<Ranked> ranked;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const long long flags = paths[i].flags;
        const int type = (flags & 1) ? 0 : ((flags & 16) ? 1 : 2);
        ranked.push_back({type, i});
    }
    std::stable_sort(ranked.begin(), ranked.end(), [](const Ranked& a, const Ranked& b) {
        return a.type != b.type ? a.type < b.type : a.index < b.index;
    });
    int ignore = 1;
    if (hatch_style == 0) {
        ignore = 3;
    } else if (hatch_style == 1) {
        ignore = 2;
    }
    Shapes boundaries;
    for (const Ranked& r : ranked) {
        if (r.type >= ignore) continue;
        const HatchPath& path = paths[r.index];
        if (path.polyline) {
            for (const Shape& s : convert_bulge_polyline(path.vertices, path.is_closed, z_value, "HATCH")) {
                boundaries.push_back(s);
            }
        } else {
            std::vector<Edge> edges;
            for (const auto& edge : path.edges) edges.push_back(convert_hatch_edge(edge, z_value));
            if (edges.empty()) continue;
            if (edges.size() == 1) {
                boundaries.push_back(edges[0]);
            } else {
                boundaries.push_back(Wire(edges));
            }
        }
    }
    return boundaries;
}

Shapes process_solid_trace_3dface(const Entity&) {
    // build123d 0.13 reads the vertices as v0..v3, which aren't DXF attributes of these
    // entities (they are vtx0..vtx3): no vertex is found and Polygon() raises IndexError.
    throw std::out_of_range("list index out of range (3DFACE, SOLID and TRACE can't be imported)");
}

Shapes process_spline(const Entity& e) {
    const auto data = e.subclass(2);
    if (!data) throw dxf::StructureError("missing 'AcDbSpline' subclass in SPLINE");
    std::vector<Vector> control_points, fit_points;
    std::vector<double> knots, weights;
    std::optional<Vector> start_tangent, end_tangent;
    for (const Tag& t : *data) {
        if (t.code == 10) control_points.push_back(point(&t));
        if (t.code == 11) fit_points.push_back(point(&t));
        if (t.code == 40) knots.push_back(number(&t, 0));
        if (t.code == 41) weights.push_back(number(&t, 0));
        if (t.code == 12 || t.code == 13) {
            const Vector v = point(&t);
            auto close = [](double a) { return std::fabs(a) <= 1e-12; };
            if (close(v.X) && close(v.Y) && close(v.Z)) continue;  // null tangents are ignored
            if (t.code == 12) start_tangent = v;
            if (t.code == 13) end_tangent = v;
        }
    }
    const int degree = static_cast<int>(integer(e.get(71), 3));
    const bool periodic = (integer(e.get(70), 0) & 2) != 0;
    if (!control_points.empty() && !knots.empty()) {
        return {BSpline(control_points, knots, degree, weights, periodic)};
    }
    if (!fit_points.empty()) {
        std::vector<Vector> tangents;
        if (start_tangent && end_tangent) tangents = {*start_tangent, *end_tangent};
        return {Spline(fit_points, tangents)};
    }
    throw std::invalid_argument("Unsupported SPLINE entity: missing control points and knots");
}

Shapes process_text(const Entity& e) {
    static const std::map<long long, TextAlign> v_alignment = {
        {0, TextAlign::BOTTOM}, {1, TextAlign::BOTTOM}, {2, TextAlign::CENTER}, {3, TextAlign::TOP}};
    static const std::map<long long, TextAlign> h_alignment = {{0, TextAlign::LEFT},   {1, TextAlign::CENTER},
                                                               {2, TextAlign::RIGHT},  {3, TextAlign::LEFT},
                                                               {4, TextAlign::CENTER}, {5, TextAlign::LEFT}};
    const long long halign = integer(e.get(72), 0);
    const long long valign = integer(e.get(73), 0);
    Vector position = point(e.get(10));
    if ((halign != 0 || valign != 0) && e.get(11) != nullptr) position = point(e.get(11));
    TextOptions options;
    options.rotation = number(e.get(50), 0);
    const auto h = h_alignment.find(halign);
    const auto v = v_alignment.find(valign);
    options.text_align = {h == h_alignment.end() ? TextAlign::LEFT : h->second,
                          v == v_alignment.end() ? TextAlign::BOTTOM : v->second};
    return {Text(text(e.get(1), ""), number(e.get(40), 2.5), options).moved(Pos(position))};
}

Shapes process_mtext(const Entity& e) {
    static const std::map<long long, std::pair<TextAlign, TextAlign>> attachment_align = {
        {1, {TextAlign::LEFT, TextAlign::TOPFIRSTLINE}},  {2, {TextAlign::CENTER, TextAlign::TOPFIRSTLINE}},
        {3, {TextAlign::RIGHT, TextAlign::TOPFIRSTLINE}}, {4, {TextAlign::LEFT, TextAlign::CENTER}},
        {5, {TextAlign::CENTER, TextAlign::CENTER}},      {6, {TextAlign::RIGHT, TextAlign::CENTER}},
        {7, {TextAlign::LEFT, TextAlign::BOTTOM}},        {8, {TextAlign::CENTER, TextAlign::BOTTOM}},
        {9, {TextAlign::RIGHT, TextAlign::BOTTOM}}};
    const auto data = e.subclass(2);
    if (!data) throw dxf::StructureError("missing 'AcDbMText' subclass in MTEXT");
    // load_mtext_content: the 3 chunks, then the 1 tag
    std::string tail;
    std::string parts;
    for (const Tag& t : *data) {
        if (t.code == 1) tail = t.text;
        if (t.code == 3) parts += t.text;
    }
    std::string content = parts + tail;
    // escape_dxf_line_endings
    std::string escaped;
    for (char c : content) {
        if (c == '\r') continue;
        if (c == '\n') {
            escaped += "\\P";
        } else {
            escaped += c;
        }
    }
    TextOptions options;
    options.rotation = number(e.get(50), 0);
    const auto it = attachment_align.find(integer(e.get(71), 1));
    options.text_align = it == attachment_align.end() ? std::pair{TextAlign::LEFT, TextAlign::TOPFIRSTLINE} : it->second;
    return {Text(dxf::fast_plain_mtext(escaped), number(e.get(40), 2.5), options).moved(Pos(point(e.get(10))))};
}

Shapes process_entity(const Entity& entity, const dxf::Document& doc, int depth);

Shapes process_insert(const Entity& e, const dxf::Document& doc, int depth) {
    if (depth > 100) throw std::invalid_argument("DXF: INSERTs nest too deeply");
    const std::string block_name = text(e.get(2), "");
    const Tag* insert = e.get(10);
    if (insert == nullptr) throw std::invalid_argument("DXF: INSERT without insertion point");
    const Vector insert_point = point(insert);
    const double zscale = number(e.get(43), 1);
    const Vector scale_factors(number(e.get(41), 1), number(e.get(42), 1), zscale != 0 ? zscale : 1.0);
    const double rotation_angle = number(e.get(50), 0);
    const long long column_count = integer(e.get(70), 1);
    const long long row_count = integer(e.get(71), 1);
    const double column_spacing = number(e.get(44), 0);
    const double row_spacing = number(e.get(45), 0);

    const dxf::Block* block = doc.block(block_name);
    if (block == nullptr) throw std::invalid_argument("DXF: unknown block '" + block_name + "'");
    const Vector block_base_point = point(block->block.get(10));
    Shapes transformed_entities;
    for (const Entity& block_entity : block->entities) {
        for (const Shape& entity_object : process_entity(block_entity, doc, depth + 1)) {
            for (long long row_index = 0; row_index < row_count; ++row_index) {
                for (long long column_index = 0; column_index < column_count; ++column_index) {
                    Vector array_offset(static_cast<double>(column_index) * column_spacing,
                                        static_cast<double>(row_index) * row_spacing, 0);
                    array_offset = Vector(array_offset.X * scale_factors.X, array_offset.Y * scale_factors.Y,
                                          array_offset.Z * scale_factors.Z)
                                       .rotate(Axis::Z, rotation_angle);
                    Shape transformed = entity_object.translate(-block_base_point);
                    transformed = scale(transformed, scale_factors);
                    transformed = transformed.rotate(Axis::Z, rotation_angle);
                    transformed = transformed.translate(insert_point + array_offset);
                    transformed_entities.push_back(transformed);
                }
            }
        }
    }
    return transformed_entities;
}

Shapes process_entity(const Entity& entity, const dxf::Document& doc, int depth) {
    const std::string& type = entity.type;
    if (type == "3DFACE" || type == "SOLID" || type == "TRACE") return process_solid_trace_3dface(entity);
    if (type == "ARC") return process_arc(entity);
    if (type == "CIRCLE") return process_circle(entity);
    if (type == "ELLIPSE") return process_ellipse(entity);
    if (type == "HATCH") return process_hatch(entity);
    if (type == "INSERT") return process_insert(entity, doc, depth);
    if (type == "LINE") return process_line(entity);
    if (type == "LWPOLYLINE") return process_lwpolyline(entity);
    if (type == "MTEXT") return process_mtext(entity);
    if (type == "POINT") return process_point(entity);
    if (type == "POLYLINE") return process_polyline(entity);
    if (type == "SPLINE") return process_spline(entity);
    if (type == "TEXT") return process_text(entity);
    warn("Unable to convert " + type);
    return {};
}

ShapeList<Shape> import_document(const dxf::Document& doc) {
    ShapeList<Shape> shapes;
    for (const Entity& entity : doc.modelspace) {
        for (const Shape& s : process_entity(entity, doc, 0)) shapes.push_back(s);
    }
    return shapes;
}

}  // namespace

ShapeList<Shape> import_dxf(const std::string& dxf_file) {
    dxf::Document doc;
    try {
        doc = dxf::read_file(dxf_file);
    } catch (const dxf::StructureError&) {
        throw std::invalid_argument("Failed to read " + dxf_file);
    }
    return detail::occt("import_dxf", [&] { return import_document(doc); });
}

ShapeList<Shape> import_dxf(std::istream& dxf_stream) {
    // build123d reads a binary stream as Latin-1 text: data.decode("latin1"), then
    // ezdxf.read(StringIO(text))
    std::ostringstream content;
    content << dxf_stream.rdbuf();
    dxf::Document doc;
    try {
        doc = dxf::read_text(dxf::latin1_to_utf8(content.str()));
    } catch (const dxf::StructureError&) {
        throw std::invalid_argument("Failed to read the DXF stream");
    }
    return detail::occt("import_dxf", [&] { return import_document(doc); });
}

}  // namespace build123d
