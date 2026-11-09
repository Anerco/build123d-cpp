// Shape.__hash__, to_splines, tessellate_with_uvs (with pack_utils._pack2d),
// show_topology (shape_core.py) and Face.wire (two_d.py).
#include <algorithm>
#include <cstdio>
#include <memory>
#include <sstream>

#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeCustom.hxx>
#include <ShapeCustom_RestrictionParameters.hxx>
#include <Standard_Version.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Mat.hxx>

#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

std::size_t Shape::hash() const {
    if (obj_.IsNull()) return 0;
#if OCC_VERSION_HEX >= 0x070800
    return std::hash<TopoDS_Shape>{}(obj_);
#else
    return static_cast<std::size_t>(obj_.HashCode(IntegerLast()));
#endif
}

TopoDS_Shape Shape::to_splines_topods(int degree, double tolerance, bool nurbs) const {
    if (obj_.IsNull()) throw std::invalid_argument("Cannot approximate an empty shape");
    return occt("to_splines", [&] {
        Handle(ShapeCustom_RestrictionParameters) params = new ShapeCustom_RestrictionParameters();
        return ShapeCustom::BSplineRestriction(obj_, tolerance, tolerance, degree, 1, GeomAbs_C0, GeomAbs_C0, true,
                                               !nurbs, params);
    });
}

// ---------------------------------------------------------------------------
// tessellate_with_uvs
// ---------------------------------------------------------------------------

namespace {

/// pack_utils._pack2d: a growing bin packer (jakesgordon/bin-packing); the positions of
/// the rectangles in input order.
std::vector<std::array<double, 2>> pack2d(const std::vector<std::array<double, 2>>& sizes) {
    struct Node {
        bool used = false;
        double x = 0, y = 0, w = 0, h = 0;
        Node* down = nullptr;
        Node* right = nullptr;
    };
    std::vector<std::unique_ptr<Node>> pool;
    const auto make = [&](Node n) {
        pool.push_back(std::make_unique<Node>(n));
        return pool.back().get();
    };
    std::function<Node*(Node*, double, double)> find_node = [&](Node* start, double w, double h) -> Node* {
        if (start->used) {
            Node* found = find_node(start->right, w, h);
            return found ? found : find_node(start->down, w, h);
        }
        if (w <= start->w && h <= start->h) return start;
        return nullptr;
    };
    const auto split_node = [&](Node* node, double w, double h) {
        node->used = true;
        node->down = make({false, node->x, node->y + h, node->w, node->h - h});
        node->right = make({false, node->x + w, node->y, node->w - w, h});
        return node;
    };
    if (sizes.empty()) throw std::invalid_argument("no objects to pack");
    struct Item {
        std::size_t index;
        double w, h;
    };
    std::vector<Item> items;
    for (std::size_t i = 0; i < sizes.size(); ++i) items.push_back({i, sizes[i][0], sizes[i][1]});
    std::stable_sort(items.begin(), items.end(),
                     [](const Item& a, const Item& b) { return std::min(a.w, a.h) > std::min(b.w, b.h); });
    std::stable_sort(items.begin(), items.end(),
                     [](const Item& a, const Item& b) { return std::max(a.w, a.h) > std::max(b.w, b.h); });
    Node* root = make({false, 0, 0, items[0].w, items[0].h});
    const auto grow_right = [&](double w, double h) {
        root = make({true, 0, 0, root->w + w, root->h, root, make({false, root->w, 0, w, root->h})});
        Node* node = find_node(root, w, h);
        if (!node) throw std::runtime_error("Failed to grow right!");
        return split_node(node, w, h);
    };
    const auto grow_down = [&](double w, double h) {
        root = make({true, 0, 0, root->w, root->h + h, make({false, 0, root->h, root->w, h}), root});
        Node* node = find_node(root, w, h);
        if (!node) throw std::runtime_error("Failed to grow down!");
        return split_node(node, w, h);
    };
    const auto grow_node = [&](double w, double h) {
        const bool can_grow_down = w <= root->w;
        const bool can_grow_right = h <= root->h;
        const bool should_grow_right = can_grow_right && (root->h >= (root->w + w));
        const bool should_grow_down = can_grow_down && (root->w >= (root->h + h));
        if (should_grow_right) return grow_right(w, h);
        if (should_grow_down) return grow_down(w, h);
        if (can_grow_right) return grow_right(w, h);
        if (can_grow_down) return grow_down(w, h);
        throw std::runtime_error("Failed to grow!");
    };
    std::vector<std::array<double, 2>> positions(sizes.size());
    for (const auto& item : items) {
        Node* node = find_node(root, item.w, item.h);
        node = node ? split_node(node, item.w, item.h) : grow_node(item.w, item.h);
        positions[item.index] = {node->x, node->y};
    }
    return positions;
}

}  // namespace

std::tuple<std::vector<Vector>, std::vector<std::array<int, 3>>, std::vector<Vector>, std::vector<std::array<double, 2>>>
Shape::tessellate_with_uvs(double tolerance, double angular_tolerance, bool atlas_packing, double atlas_gutter) const {
    require_not_null("Cannot tessellate an empty shape");
    mesh(tolerance, angular_tolerance);
    std::vector<Vector> all_vertices, all_normals;
    std::vector<std::array<int, 3>> all_triangles;
    std::vector<std::array<double, 2>> all_uvs;
    struct UvRange {
        std::size_t start, count;
        double w, h;
    };
    std::vector<UvRange> face_uv_ranges;
    int offset = 0;
    for (const auto& face : faces()) {
        const TopoDS_Face& f = TopoDS::Face(face.wrapped());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) poly = BRep_Tool::Triangulation(f, loc);
        if (poly.IsNull()) continue;
        const gp_Trsf trsf = loc.Transformation();
        const bool is_reversed = f.Orientation() == TopAbs_REVERSED;
        const double reverse_factor = is_reversed ? -1.0 : 1.0;
        const int nb_nodes = poly->NbNodes();
        const gp_Mat rot = trsf.VectorialPart();
        const gp_XYZ tr = trsf.TranslationPart();
        // vertices: rot * p + translation
        for (int i = 1; i <= nb_nodes; ++i) {
            const gp_Pnt p = poly->Node(i);
            double c[3];
            for (int r = 0; r < 3; ++r) {
                c[r] = p.X() * rot.Value(r + 1, 1) + p.Y() * rot.Value(r + 1, 2) + p.Z() * rot.Value(r + 1, 3);
            }
            all_vertices.emplace_back(c[0] + tr.X(), c[1] + tr.Y(), c[2] + tr.Z());
        }
        // normals: rotated, flipped on reversed faces
        if (!poly->HasNormals()) poly->ComputeNormals();
        for (int i = 1; i <= nb_nodes; ++i) {
            const gp_Dir d = poly->Normal(i);
            double c[3];
            for (int r = 0; r < 3; ++r) {
                c[r] = (d.X() * rot.Value(r + 1, 1) + d.Y() * rot.Value(r + 1, 2) + d.Z() * rot.Value(r + 1, 3)) *
                       reverse_factor;
            }
            all_normals.emplace_back(c[0], c[1], c[2]);
        }
        // UV coordinates normalized per face; the face's physical aspect from the arc
        // lengths of its center isoparametric curves
        if (poly->HasUVNodes()) {
            std::vector<std::array<double, 2>> uv_raw;
            double uv_min[2] = {1e300, 1e300}, uv_max[2] = {-1e300, -1e300};
            for (int i = 1; i <= nb_nodes; ++i) {
                const gp_Pnt2d uv = poly->UVNode(i);
                uv_raw.push_back({uv.X(), uv.Y()});
                uv_min[0] = std::min(uv_min[0], uv.X());
                uv_min[1] = std::min(uv_min[1], uv.Y());
                uv_max[0] = std::max(uv_max[0], uv.X());
                uv_max[1] = std::max(uv_max[1], uv.Y());
            }
            double uv_range[2] = {uv_max[0] - uv_min[0], uv_max[1] - uv_min[1]};
            for (double& r : uv_range) {
                if (r < TOLERANCE) r = 1.0;
            }
            double arc_w = uv_range[0], arc_h = uv_range[1];
            try {
                Handle(Geom_Surface) surface = BRep_Tool::Surface(f);
                const double u_center = (uv_min[0] + uv_max[0]) * 0.5;
                const double v_center = (uv_min[1] + uv_max[1]) * 0.5;
                const GeomAdaptor_Curve u_adaptor(surface->VIso(v_center));
                const GeomAdaptor_Curve v_adaptor(surface->UIso(u_center));
                // (assigned one at a time, as build123d: a failure keeps the first)
                arc_w = GCPnts_AbscissaPoint::Length(u_adaptor, uv_min[0], uv_max[0], TOLERANCE);
                arc_h = GCPnts_AbscissaPoint::Length(v_adaptor, uv_min[1], uv_max[1], TOLERANCE);
            } catch (const Standard_Failure&) {
                // degenerate parameterization: keep the UV extents
            }
            if (arc_w < TOLERANCE) arc_w = 1.0;
            if (arc_h < TOLERANCE) arc_h = 1.0;
            face_uv_ranges.push_back({all_uvs.size(), static_cast<std::size_t>(nb_nodes), arc_w, arc_h});
            for (const auto& uv : uv_raw) {
                double u = (uv[0] - uv_min[0]) / uv_range[0];
                const double v = (uv[1] - uv_min[1]) / uv_range[1];
                if (is_reversed) u = 1.0 - u;
                all_uvs.push_back({u, v});
            }
        } else {
            for (int i = 0; i < nb_nodes; ++i) all_uvs.push_back({0.0, 0.0});
            face_uv_ranges.push_back({all_uvs.size() - static_cast<std::size_t>(nb_nodes), static_cast<std::size_t>(nb_nodes),
                                      1.0, 1.0});
        }
        for (int i = 1; i <= poly->NbTriangles(); ++i) {
            int n1, n2, n3;
            poly->Triangle(i).Get(n1, n2, n3);
            if (is_reversed) std::swap(n1, n2);
            all_triangles.push_back({n1 + offset - 1, n2 + offset - 1, n3 + offset - 1});
        }
        offset += nb_nodes;
    }
    // Atlas packing: per-face UVs scaled to their physical proportions and packed
    if (atlas_packing && !face_uv_ranges.empty()) {
        std::vector<std::array<double, 2>> sizes;
        for (const auto& r : face_uv_ranges) sizes.push_back({r.w, r.h});
        const auto positions = pack2d(sizes);
        double pack_w = -1e300, pack_h = -1e300;
        for (std::size_t i = 0; i < positions.size(); ++i) {
            pack_w = std::max(pack_w, positions[i][0] + face_uv_ranges[i].w);
            pack_h = std::max(pack_h, positions[i][1] + face_uv_ranges[i].h);
        }
        double scale = std::max(pack_w, pack_h);
        if (scale < TOLERANCE) scale = 1.0;
        const double inv_scale = 1.0 / scale;
        for (std::size_t i = 0; i < positions.size(); ++i) {
            const UvRange& info = face_uv_ranges[i];
            double bw = info.w * inv_scale, bh = info.h * inv_scale;
            const double gutter_u = std::min(atlas_gutter, bw * 0.5);
            const double gutter_v = std::min(atlas_gutter, bh * 0.5);
            const double bx = positions[i][0] * inv_scale + gutter_u;
            const double by = positions[i][1] * inv_scale + gutter_v;
            bw -= 2.0 * gutter_u;
            bh -= 2.0 * gutter_v;
            for (std::size_t j = info.start; j < info.start + info.count; ++j) {
                all_uvs[j] = {all_uvs[j][0] * bw + bx, all_uvs[j][1] * bh + by};
            }
        }
    }
    return {all_vertices, all_triangles, all_normals, all_uvs};
}

// ---------------------------------------------------------------------------
// show_topology
// ---------------------------------------------------------------------------

namespace {

int shape_order(TopAbs_ShapeEnum t) {
    switch (t) {
        case TopAbs_COMPOUND: return 0;
        case TopAbs_COMPSOLID: return 1;
        case TopAbs_SOLID: return 2;
        case TopAbs_SHELL: return 3;
        case TopAbs_FACE: return 4;
        case TopAbs_WIRE: return 5;
        case TopAbs_EDGE: return 6;
        default: return 7;
    }
}

TopAbs_ShapeEnum to_topabs(ShapeType t) {
    switch (t) {
        case ShapeType::COMPOUND: return TopAbs_COMPOUND;
        case ShapeType::COMPSOLID: return TopAbs_COMPSOLID;
        case ShapeType::SOLID: return TopAbs_SOLID;
        case ShapeType::SHELL: return TopAbs_SHELL;
        case ShapeType::FACE: return TopAbs_FACE;
        case ShapeType::WIRE: return TopAbs_WIRE;
        case ShapeType::EDGE: return TopAbs_EDGE;
        default: return TopAbs_VERTEX;
    }
}

/// Python's f"{x:.6g}" of a Vector coordinate (values within TOLERANCE of 0 print 0).
std::string fmt_g6(double x) {
    const double v = std::abs(x) > TOLERANCE ? py_round(x, 6) : 0.0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

std::string vector_g6(const Vector& v) { return "(" + fmt_g6(v.X) + ", " + fmt_g6(v.Y) + ", " + fmt_g6(v.Z) + ")"; }

struct TreeNode {
    std::string label;       // left column
    std::string name;        // class column ("" for TopoDS trees)
    const void* address = nullptr;
    std::string location;    // "Center(...)" / "Position(...)" / "Location(...)"
    std::vector<TreeNode> children;
    int height() const {
        int h = 0;
        for (const auto& c : children) h = std::max(h, c.height() + 1);
        return h;
    }
};

std::size_t utf8_length(const std::string& s) {
    std::size_t n = 0;
    for (unsigned char c : s) n += (c & 0xC0) != 0x80;
    return n;
}

void label_sizes(const TreeNode& node, std::vector<std::pair<int, std::size_t>>& out) {
    out.emplace_back(node.height(), utf8_length(node.label));
    for (const auto& c : node.children) label_sizes(c, out);
}

void render(const TreeNode& node, const std::string& pre, const std::string& fill, std::size_t width,
            std::ostringstream& out) {
    std::string treestr = pre + node.label;
    const std::size_t len = utf8_length(treestr);
    if (len < width) treestr += std::string(width - len, ' ');
    char address[32];
    std::snprintf(address, sizeof(address), "%#zx", reinterpret_cast<std::size_t>(node.address));
    out << treestr << node.name << "at " << address << ", " << node.location << "\n";
    for (std::size_t i = 0; i < node.children.size(); ++i) {
        const bool last = i + 1 == node.children.size();
        render(node.children[i], fill + (last ? "└── " : "├── "), fill + (last ? "    " : "│   "), width, out);
    }
}

std::string show_tree(const TreeNode& root) {
    std::vector<std::pair<int, std::size_t>> sizes;
    label_sizes(root, sizes);
    const int root_height = root.height();
    std::vector<std::size_t> max_sizes_per_level;
    for (int level = 0; level <= root_height; ++level) {
        std::size_t m = 0;
        for (const auto& [h, len] : sizes) {
            if (h == level) m = std::max(m, len);
        }
        max_sizes_per_level.push_back(std::max<std::size_t>(4, m));
    }
    std::size_t width = 0;
    for (std::size_t i = 0; i < max_sizes_per_level.size(); ++i) {
        width = std::max(width, max_sizes_per_level[max_sizes_per_level.size() - 1 - i] + i * 4);
    }
    std::ostringstream out;
    render(root, "", "", width + 1, out);
    return out.str();
}

TreeNode topods_tree(const TopoDS_Shape& shape, int limit, bool show_center) {
    TreeNode node;
    node.label = build123d::to_string(Shape(shape).shape_type());
    node.address = shape.TShape().get();
    node.location = (show_center ? "Center" : "Position") + vector_g6(Shape(shape).bounding_box().center());
    for (TopoDS_Iterator it(shape); it.More(); it.Next()) {
        if (shape_order(it.Value().ShapeType()) <= limit) node.children.push_back(topods_tree(it.Value(), limit, show_center));
    }
    return node;
}

std::string class_name(const Shape& s) {
    std::string name = build123d::to_string(s.shape_type());
    if (name.size() < 9) name += std::string(9 - name.size(), ' ');
    return name;
}

TreeNode assembly_tree(const Shape& shape, bool show_center) {
    TreeNode node;
    node.label = shape.label;
    node.name = class_name(shape);
    node.address = &shape;
    if (show_center) {
        node.location = "Center" + vector_g6(shape.center());
    } else {
        const Location loc = shape.location();
        node.location = "Location(" + vector_g6(loc.position()) + ", " + vector_g6(loc.orientation()) + ")";
    }
    for (const auto& c : shape.children) node.children.push_back(assembly_tree(c, show_center));
    return node;
}

}  // namespace

std::string Shape::show_topology(ShapeType limit_class, std::optional<bool> show_center) const {
    require_not_null("Can't show the topology of an empty shape");
    if (obj_.ShapeType() == TopAbs_COMPOUND && !children.empty()) {
        return show_tree(assembly_tree(*this, show_center.value_or(false)));
    }
    return show_tree(topods_tree(obj_, shape_order(to_topabs(limit_class)), show_center.value_or(true)));
}

// ---------------------------------------------------------------------------
// Face.wire
// ---------------------------------------------------------------------------

Wire Face::wire() const {
    if (!inner_wires().empty()) warn("Found holes, returning outer_wire");
    return outer_wire();
}

}  // namespace build123d
