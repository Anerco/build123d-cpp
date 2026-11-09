// Free topology helpers: topo_explore_connected_edges/faces, topo_explore_common_vertex
// (one_d.py, zero_d.py), find_same_topods, relocation_between, topo_distance_to
// (shape_core.py), new_edges, polar (utils.py), edges_to_wires (one_d.py).
#include <cmath>
#include <deque>
#include <limits>
#include <map>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepLProp.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_Shape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include "build123d/topology_utils.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace detail {

std::optional<TopoDS_Shape> find_same_topods(const TopoDS_Shape& query, const std::vector<TopoDS_Shape>& candidates) {
    for (const auto& c : candidates) {
        if (c.IsSame(query)) return c;
    }
    for (const auto& c : candidates) {
        if (c.IsPartner(query)) return c;
    }
    return std::nullopt;
}

TopLoc_Location relocation_between(const TopoDS_Shape& source, const TopoDS_Shape& target) {
    return target.Location().Multiplied(source.Location().Inverted());
}

std::vector<TopoDS_Face> topo_explore_connected_faces(const Shape& edge, const std::optional<Shape>& parent_in) {
    if (edge.is_null()) throw std::invalid_argument("Can't explore from an empty edge");
    const std::optional<Shape> parent = parent_in ? parent_in : edge.topo_parent();
    if (!parent || !*parent) throw std::invalid_argument("edge has no valid parent");
    OccShapeListMap edge_face_map;
    TopExp::MapShapesAndAncestors(parent->wrapped(), TopAbs_EDGE, TopAbs_FACE, edge_face_map);
    // the parent's own copy of the edge (the map keys on TShape and Location)
    std::vector<TopoDS_Shape> keys;
    for (int i = 1; i <= edge_face_map.Extent(); ++i) keys.push_back(edge_face_map.FindKey(i));
    const auto parent_edge = find_same_topods(edge.wrapped(), keys);
    if (!parent_edge) return {};
    const TopLoc_Location relocation = relocation_between(*parent_edge, edge.wrapped());
    OccShapeMap unique_faces;
    for (const auto& face : edge_face_map.FindFromKey(*parent_edge)) unique_faces.Add(face);
    std::vector<TopoDS_Face> out;
    for (int i = 1; i <= unique_faces.Extent(); ++i) out.push_back(TopoDS::Face(unique_faces(i).Moved(relocation)));
    return out;
}

}  // namespace detail

std::optional<Vertex> topo_explore_common_vertex(const Edge& edge1, const Edge& edge2) {
    for (TopExp_Explorer v1(edge1.wrapped(), TopAbs_VERTEX); v1.More(); v1.Next()) {
        for (TopExp_Explorer v2(edge2.wrapped(), TopAbs_VERTEX); v2.More(); v2.Next()) {
            if (v1.Current().IsSame(v2.Current())) return Vertex(TopoDS::Vertex(v1.Current()));
        }
    }
    return std::nullopt;
}

ShapeList<Edge> topo_explore_connected_edges(const Edge& edge, const std::optional<Shape>& parent_in,
                                             ContinuityLevel continuity) {
    const std::optional<Shape> parent = parent_in ? parent_in : edge.topo_parent();
    if (!parent) throw std::invalid_argument("edge has no valid parent");
    if (edge.is_null()) throw std::invalid_argument("edge is empty");
    std::vector<TopoDS_Shape> topods_edges;
    for (const auto& e : parent->edges()) topods_edges.push_back(e.wrapped());
    // work with the parent's own copy of the edge so vertices match
    const auto parent_edge = find_same_topods(edge.wrapped(), topods_edges);
    if (!parent_edge) return {};
    const TopoDS_Edge given = TopoDS::Edge(*parent_edge);
    const TopLoc_Location relocation = relocation_between(given, edge.wrapped());

    const auto level_of = [](GeomAbs_Shape c) {
        switch (c) {
            case GeomAbs_C0: return ContinuityLevel::C0;
            case GeomAbs_G1:
            case GeomAbs_C1: return ContinuityLevel::C1;
            default: return ContinuityLevel::C2;
        }
    };
    std::vector<TopoDS_Shape> connected;
    for (const auto& topods_edge : topods_edges) {
        if (given.IsSame(topods_edge)) continue;
        const auto common = topo_explore_common_vertex(Edge(given), Edge(topods_edge));
        if (!common || common->is_null()) continue;
        const TopoDS_Vertex& v = TopoDS::Vertex(common->wrapped());
        const double u1 = BRep_Tool::Parameter(v, given);
        const double u2 = BRep_Tool::Parameter(v, TopoDS::Edge(topods_edge));
        const BRepAdaptor_Curve curve1(given), curve2(TopoDS::Edge(topods_edge));
        const GeomAbs_Shape actual = BRepLProp::Continuity(curve1, curve2, u1, u2, TOLERANCE, TOLERANCE);
        if (static_cast<int>(level_of(actual)) >= static_cast<int>(continuity)) {
            if (std::none_of(connected.begin(), connected.end(), [&](const TopoDS_Shape& c) { return c.IsSame(topods_edge); })) {
                connected.push_back(topods_edge);
            }
        }
    }
    ShapeList<Edge> out;
    for (const auto& e : connected) out.push_back(Edge(relocation.IsIdentity() ? e : e.Moved(relocation)));
    return out;
}

std::function<double(const Shape&)> topo_distance_to(const Shape& other) {
    return topo_distance_to(std::vector<Shape>{other});
}

std::function<double(const Shape&)> topo_distance_to(const std::vector<Shape>& sources) {
    if (sources.empty()) throw std::invalid_argument("Cannot measure topological distance to an empty object");
    const ShapeType peer_type = sources[0].shape_type();
    for (const auto& s : sources) {
        if (s.shape_type() != peer_type) throw std::invalid_argument("Topological distance requires shapes of the same type");
    }
    TopAbs_ShapeEnum peer_enum, connector_enum = TopAbs_SHAPE;
    switch (peer_type) {
        case ShapeType::VERTEX: peer_enum = TopAbs_VERTEX; break;
        case ShapeType::EDGE: peer_enum = TopAbs_EDGE, connector_enum = TopAbs_VERTEX; break;
        case ShapeType::WIRE: peer_enum = TopAbs_WIRE, connector_enum = TopAbs_VERTEX; break;
        case ShapeType::FACE: peer_enum = TopAbs_FACE, connector_enum = TopAbs_EDGE; break;
        case ShapeType::SHELL: peer_enum = TopAbs_SHELL, connector_enum = TopAbs_FACE; break;
        case ShapeType::SOLID: peer_enum = TopAbs_SOLID, connector_enum = TopAbs_FACE; break;
        default:
            throw std::invalid_argument("Topological distance is not supported for " + build123d::to_string(peer_type));
    }
    std::vector<Shape> parents;
    for (const auto& s : sources) {
        const auto p = s.topo_parent();
        if (!p) throw std::invalid_argument("Topological distance requires shapes with a topo_parent");
        parents.push_back(*p);
    }
    const Shape parent = parents[0];
    for (std::size_t i = 1; i < parents.size(); ++i) {
        if (!parent.is_same(parents[i])) throw std::invalid_argument("Topological distance requires a shared topo_parent");
    }

    // the peers: the parent's sub-shapes of the type, indexed by TShape+Location
    std::vector<TopoDS_Shape> peers;
    switch (peer_type) {
        case ShapeType::VERTEX:
            for (const auto& s : parent.vertices()) peers.push_back(s.wrapped());
            break;
        case ShapeType::EDGE:
            for (const auto& s : parent.edges()) peers.push_back(s.wrapped());
            break;
        case ShapeType::WIRE:
            for (const auto& s : parent.wires()) peers.push_back(s.wrapped());
            break;
        case ShapeType::FACE:
            for (const auto& s : parent.faces()) peers.push_back(s.wrapped());
            break;
        case ShapeType::SHELL:
            for (const auto& s : parent.shells()) peers.push_back(s.wrapped());
            break;
        default:
            for (const auto& s : parent.solids()) peers.push_back(s.wrapped());
            break;
    }
    OccShapeMap peer_index;  // 1-based index into peers
    for (const auto& p : peers) peer_index.Add(p);
    const auto index_of = [&](const TopoDS_Shape& s) -> int { return peer_index.FindIndex(s); };
    const auto peer_of = [peers, peer_index, index_of](const TopoDS_Shape& s) -> int {
        int i = index_of(s);
        if (i == 0) {
            const auto match = find_same_topods(s, peers);
            if (match) i = index_of(*match);
        }
        return i;
    };

    std::vector<std::vector<int>> neighbors(peers.size() + 1);
    if (peer_type == ShapeType::VERTEX) {
        OccShapeListMap vertex_edge_map;
        TopExp::MapShapesAndAncestors(parent.wrapped(), TopAbs_VERTEX, TopAbs_EDGE, vertex_edge_map);
        for (int i = 1; i <= vertex_edge_map.Extent(); ++i) {
            const int vertex_peer = index_of(vertex_edge_map.FindKey(i));
            if (vertex_peer == 0) continue;
            for (const auto& e : vertex_edge_map(i)) {
                TopoDS_Vertex v0, v1;
                TopExp::Vertices(TopoDS::Edge(e), v0, v1);
                for (const auto& nv : {v0, v1}) {
                    const int neighbor = nv.IsNull() ? 0 : index_of(nv);
                    if (neighbor != 0 && neighbor != vertex_peer) neighbors[static_cast<std::size_t>(vertex_peer)].push_back(neighbor);
                }
            }
        }
    } else {
        OccShapeListMap connector_peer_map;
        TopExp::MapShapesAndAncestors(parent.wrapped(), connector_enum, peer_enum, connector_peer_map);
        for (int i = 1; i <= connector_peer_map.Extent(); ++i) {
            std::vector<int> connected;
            for (const auto& p : connector_peer_map(i)) {
                const int peer = index_of(p);
                if (peer != 0) connected.push_back(peer);
            }
            for (int a : connected) {
                for (int b : connected) {
                    if (a != b) neighbors[static_cast<std::size_t>(a)].push_back(b);
                }
            }
        }
    }

    // breadth-first distances from the sources
    std::map<int, double> distances;
    std::deque<int> frontier;
    for (const auto& source : sources) {
        const int peer = peer_of(source.wrapped());
        if (peer != 0 && distances.count(peer) == 0) {
            distances[peer] = 0;
            frontier.push_back(peer);
        }
    }
    while (!frontier.empty()) {
        const int current = frontier.front();
        frontier.pop_front();
        for (int neighbor : neighbors[static_cast<std::size_t>(current)]) {
            if (distances.count(neighbor) != 0) continue;
            distances[neighbor] = distances[current] + 1;
            frontier.push_back(neighbor);
        }
    }

    return [=](const Shape& obj) -> double {
        if (obj.shape_type() != peer_type) {
            throw std::invalid_argument("Topological distance requires shapes of the same type");
        }
        const auto obj_parent = obj.topo_parent();
        if (!obj_parent) throw std::invalid_argument("Topological distance requires shapes with a topo_parent");
        if (!parent.is_same(*obj_parent)) throw std::invalid_argument("Topological distance requires a shared topo_parent");
        const int peer = peer_of(obj.wrapped());
        if (peer == 0) return std::numeric_limits<double>::infinity();
        const auto it = distances.find(peer);
        return it == distances.end() ? std::numeric_limits<double>::infinity() : it->second;
    };
}

ShapeList<Edge> new_edges(const std::vector<Shape>& objects, const Shape& combined) {
    std::vector<TopoDS_Shape> combined_edges, original_edges;
    for (const auto& e : combined.edges()) combined_edges.push_back(e.wrapped());
    for (const auto& obj : objects) {
        for (const auto& e : obj.edges()) original_edges.push_back(e.wrapped());
    }
    ShapeList<Edge> edges;
    occt("new_edges", [&] {
        BRepAlgoAPI_Cut operation;
        operation.SetArguments(to_list(combined_edges));
        operation.SetTools(to_list(original_edges));
        operation.SetRunParallel(true);
        operation.Build();
        for (TopExp_Explorer exp(operation.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
            Edge found(exp.Current());
            found.set_topo_parent(combined);
            edges.push_back(found);
        }
        return 0;
    });
    return edges;
}

Vector polar(double length, double angle) {
    return Vector(length * std::cos(angle * DEG2RAD), length * std::sin(angle * DEG2RAD));
}

ShapeList<Wire> edges_to_wires(const std::vector<Edge>& edges, double tol) {
    std::vector<TopoDS_Shape> topods;
    for (const auto& e : edges) {
        if (!e.is_null()) topods.push_back(e.wrapped());
    }
    ShapeList<Wire> out;
    for (const auto& w : connect_edges_to_wires(topods, tol)) out.push_back(Wire(w));
    return out;
}

}  // namespace build123d
