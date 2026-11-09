// full_round (operations_sketch.py), make_brake_formed and draft of one face (operations_part.py).
#include <algorithm>
#include <cmath>
#include <limits>

#include <BRepAdaptor_Curve.hxx>
#include <BRepLProp.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>

#include "build123d/operations.hpp"
#include "build123d/topology_utils.hpp"
#include "internal.hpp"
#include "voronoi.hpp"

namespace build123d {

using namespace detail;

namespace {



}  // namespace

// ---------------------------------------------------------------------------
// full_round
// ---------------------------------------------------------------------------

Sketch full_round(const Edge& edge, bool invert, int voronoi_point_count) {
    const auto topo_parent = edge.topo_parent();
    if (!topo_parent) throw std::invalid_argument("edge must be extracted from shape");

    // Evenly spaced points along the edge and the edges connected to it give the Voronoi
    // vertices, the possible centers of the largest empty circle.
    const ShapeList<Edge> connected_edges = topo_explore_connected_edges(edge, *topo_parent);
    if (connected_edges.size() != 2) throw std::invalid_argument("Invalid geometry - 3 or more edges required");

    const std::vector<Edge> edge_group{edge, connected_edges[0], connected_edges[1]};
    std::vector<Point2> numpy_style_pnts;
    for (const auto& e : edge_group) {
        for (int i = 0; i <= voronoi_point_count; ++i) {
            const Vector v = e.position_at(static_cast<double>(i) / voronoi_point_count);
            numpy_style_pnts.push_back({v.X, v.Y});
        }
    }
    std::vector<Vector> voronoi_vertices;
    for (const auto& v : detail::voronoi_vertices(numpy_style_pnts)) voronoi_vertices.emplace_back(v[0], v[1]);

    // Refine the largest empty circle center estimate by averaging the best three candidates
    std::vector<std::pair<double, std::size_t>> best_three(3, {std::numeric_limits<double>::infinity(), 0});
    for (std::size_t i = 0; i < voronoi_vertices.size(); ++i) {
        const Vector& v = voronoi_vertices[i];
        double distances[3];
        for (int k = 0; k < 3; ++k) distances[k] = edge_group[static_cast<std::size_t>(k)].distance_to(v);
        const double avg_distance = (distances[0] + distances[1] + distances[2]) / 3;
        double difference = 0.0;
        for (double d : distances) difference = std::max(difference, std::abs(d - avg_distance));
        // Prefer vertices with minimal difference
        if (difference < best_three.back().first) {
            best_three.back() = {difference, i};
            std::stable_sort(best_three.begin(), best_three.end(),
                             [](const auto& a, const auto& b) { return a.first < b.first; });
        }
    }
    Vector sum(0, 0, 0);
    for (const auto& b : best_three) sum = sum + voronoi_vertices.at(b.second);
    const Vector voronoi_circle_center = sum / 3;

    // Where the connected edges and the target edge meet the largest empty circle
    std::vector<Vector> connected_edges_end_points;
    for (const auto& e : connected_edges) {
        connected_edges_end_points.push_back(std::get<1>(e.distance_to_with_closest_points(voronoi_circle_center)));
    }
    Vector middle_edge_arc_point = std::get<1>(edge.distance_to_with_closest_points(voronoi_circle_center));

    // Trim the connected edges to allow room for the circular feature
    const Vector origin = (Vector(0, 0, 0) + connected_edges_end_points[0] + connected_edges_end_points[1]) / 2;
    const Vector x_dir = (connected_edges_end_points[1] - connected_edges_end_points[0]).normalized();
    const Vector to_arc_vec = origin - middle_edge_arc_point;
    // Project `to_arc_vec` onto the plane perpendicular to `x_dir`
    const Vector z_dir = (to_arc_vec - x_dir * to_arc_vec.dot(x_dir)).normalized();
    const Plane split_pln(origin, x_dir, z_dir);
    std::vector<Edge> trimmed_connected_edges;
    for (const auto& e : connected_edges) {
        const auto tops = e.split(split_pln);
        if (tops.empty()) throw std::invalid_argument("Invalid geometry to create the end arc");
        if (tops.size() != 1 || tops[0].wrapped().ShapeType() != TopAbs_EDGE) {
            throw std::runtime_error("Invalid geometry to create the end arc: the edge is split in several pieces");
        }
        trimmed_connected_edges.emplace_back(tops[0].wrapped());
    }

    // Flip the middle point if the user wants the concave solution
    if (invert) middle_edge_arc_point = voronoi_circle_center * 2 - middle_edge_arc_point;

    // Generate the new circular edge
    const Edge new_arc =
        Edge::make_three_point_arc(connected_edges_end_points[0], middle_edge_arc_point, connected_edges_end_points[1]);

    // Recover other edges
    const ShapeList<Edge> again_connected = topo_explore_connected_edges(edge, *topo_parent);
    std::vector<Edge> all_edges = trimmed_connected_edges;
    all_edges.push_back(new_arc);
    for (const auto& e : topo_parent->edges()) {
        const bool is_connected =
            std::any_of(again_connected.begin(), again_connected.end(), [&](const Edge& c) { return c.is_same(e); });
        if (!is_connected && !e.is_same(edge)) all_edges.push_back(e);
    }

    // Rebuild the face; the longest wire is the perimeter and the others holes
    const ShapeList<Wire> face_wires = Wire::combine(all_edges).sort_by(SortBy::LENGTH, true);
    Face pending_face(face_wires[0], std::vector<Wire>(face_wires.begin() + 1, face_wires.end()));

    // Flip the face to match the original parent
    if (topo_parent->faces()[0].normal_at() != pending_face.normal_at()) pending_face = -pending_face;
    return Sketch(std::vector<Face>{pending_face});
}

// ---------------------------------------------------------------------------
// draft of a single face
// ---------------------------------------------------------------------------

Part draft(const Face& faces, const Plane& neutral_plane, double angle) {
    return draft(std::vector<Face>{faces}, neutral_plane, angle);
}

// ---------------------------------------------------------------------------
// make_brake_formed
// ---------------------------------------------------------------------------

namespace {

Part brake_formed(double thickness, const std::optional<double>& width, const std::vector<double>& widths,
                  const Mixin1D& line, Side side, Kind kind, bool clean) {
    // Create the offset
    const Wire offset_line = line.offset_2d(thickness, kind, side, true);
    const ShapeList<Vertex> offset_vertices = offset_line.vertices();

    std::optional<Plane> plane;
    try {
        plane = Plane(Face(offset_line));
    } catch (const std::exception&) {
        throw std::invalid_argument("line not suitable - probably straight");
    }

    // Make edge pairs
    const ShapeList<Vertex> line_vertices = line.vertices();
    const std::vector<double> station_widths_list =
        width ? std::vector<double>(line_vertices.size(), *width) : widths;

    ShapeList<Edge> station_edges;
    for (const auto& vertex : line_vertices) {
        const auto others = offset_vertices.sort_by_distance(Vector(vertex));
        for (std::size_t i = 1; i < others.size(); ++i) {
            if (std::abs((Vector(vertex) - Vector(others[i])).length() - thickness) < 1e-2) {
                station_edges.push_back(Edge::make_line(vertex, others[i]));
                break;
            }
        }
    }
    station_edges = station_edges.sort_by(line);

    if (station_widths_list.size() != line_vertices.size()) {
        throw std::invalid_argument(
            "widths must either be a single number or an iterable with a length of the # vertices in line (" +
            std::to_string(line_vertices.size()) + ")");
    }
    std::vector<Face> station_faces;
    for (std::size_t i = 0; i < station_edges.size() && i < station_widths_list.size(); ++i) {
        station_faces.push_back(Face::extrude(station_edges[i], plane->z_dir * station_widths_list[i]));
    }
    const ShapeList<Edge> line_edges =
        line.shape_type() == ShapeType::WIRE ? Wire(line.wrapped()).edges() : line.edges();
    const ShapeList<Edge> sweep_paths = line_edges.sort_by(line);
    std::vector<Shape> sections;
    for (std::size_t i = 0; i + 1 < station_faces.size(); ++i) {
        sections.push_back(
            Solid::sweep_multi(std::vector<Shape>{station_faces[i], station_faces[i + 1]}, sweep_paths[i]));
    }
    if (sections.empty()) throw std::out_of_range("list index out of range");
    // Part.fuse(*sections) and clean() record their history, as in build123d
    Shape new_solid = sections[0];
    if (sections.size() > 1) new_solid = sections[0].fuse(std::vector<Shape>(sections.begin() + 1, sections.end()));
    if (clean) new_solid = new_solid.clean();
    Part result(make_compound({new_solid.wrapped()}));
    result.set_history(ShapeHistory::of(new_solid));
    return result;
}

}  // namespace

Part make_brake_formed(double thickness, double station_widths, const Mixin1D& line, Side side, Kind kind, bool clean) {
    if (line.is_null()) throw std::invalid_argument("A line must be provided");
    return brake_formed(thickness, station_widths, {}, line, side, kind, clean);
}

Part make_brake_formed(double thickness, const std::vector<double>& station_widths, const Mixin1D& line, Side side,
                       Kind kind, bool clean) {
    if (line.is_null()) throw std::invalid_argument("A line must be provided");
    return brake_formed(thickness, std::nullopt, station_widths, line, side, kind, clean);
}

Part make_brake_formed(double thickness, double station_widths, const Curve& line, Side side, Kind kind, bool clean) {
    return brake_formed(thickness, station_widths, {}, line.wires().at(0), side, kind, clean);
}

Part make_brake_formed(double thickness, const std::vector<double>& station_widths, const Curve& line, Side side,
                       Kind kind, bool clean) {
    return brake_formed(thickness, std::nullopt, station_widths, line.wires().at(0), side, kind, clean);
}

}  // namespace build123d
