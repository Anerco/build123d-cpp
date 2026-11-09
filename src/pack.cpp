// pack (pack.py) and the rectangle packer _pack2d (pack_utils.py).
#include "build123d/pack.hpp"

#include <algorithm>
#include <deque>
#include <stdexcept>
#include <string>

namespace build123d {

namespace detail {

namespace {

/// A rectangle of the packing tree: once used, its space right of and below the placed
/// rectangle are the children.
struct PackNode {
    bool used = false;
    double x = 0;
    double y = 0;
    double w = 0;
    double h = 0;
    PackNode* down = nullptr;
    PackNode* right = nullptr;
};

/// Port of the growing binary-tree packer (https://codeincomplete.com/articles/bin-packing/).
class Packer {
public:
    Packer(double w, double h) : root_(make({.w = w, .h = h})) {}

    PackNode* place(double w, double h) {
        if (PackNode* node = find_node(root_, w, h)) return split_node(node, w, h);
        return grow_node(w, h);
    }

private:
    PackNode* make(const PackNode& node) {
        nodes_.push_back(node);
        return &nodes_.back();
    }

    PackNode* find_node(PackNode* start, double w, double h) {
        if (start->used) {
            if (PackNode* node = find_node(start->right, w, h)) return node;
            return find_node(start->down, w, h);
        }
        if (w <= start->w && h <= start->h) return start;
        return nullptr;
    }

    PackNode* split_node(PackNode* node, double w, double h) {
        node->used = true;
        node->down = make({.x = node->x, .y = node->y + h, .w = node->w, .h = node->h - h});
        node->right = make({.x = node->x + w, .y = node->y, .w = node->w - w, .h = h});
        return node;
    }

    PackNode* grow_node(double w, double h) {
        const bool can_grow_down = w <= root_->w;
        const bool can_grow_right = h <= root_->h;
        const bool should_grow_right = can_grow_right && (root_->h >= (root_->w + w));
        const bool should_grow_down = can_grow_down && (root_->w >= (root_->h + h));
        if (should_grow_right) return grow_right(w, h);
        if (should_grow_down) return grow_down(w, h);
        if (can_grow_right) return grow_right(w, h);
        if (can_grow_down) return grow_down(w, h);
        throw std::runtime_error("Failed to grow! root: " + describe(*root_) + ", w: " + std::to_string(w) +
                                 ", h: " + std::to_string(h));
    }

    PackNode* grow_right(double w, double h) {
        PackNode* old_root = root_;
        PackNode* right = make({.x = old_root->w, .w = w, .h = old_root->h});
        root_ = make({.used = true, .w = old_root->w + w, .h = old_root->h, .down = old_root, .right = right});
        PackNode* node = find_node(root_, w, h);
        if (node == nullptr) throw std::runtime_error("Failed to grow right!");
        return split_node(node, w, h);
    }

    PackNode* grow_down(double w, double h) {
        PackNode* old_root = root_;
        PackNode* down = make({.y = old_root->h, .w = old_root->w, .h = h});
        root_ = make({.used = true, .w = old_root->w, .h = old_root->h + h, .down = down, .right = old_root});
        PackNode* node = find_node(root_, w, h);
        if (node == nullptr) throw std::runtime_error("Failed to grow down!");
        return split_node(node, w, h);
    }

    static std::string describe(const PackNode& node) {
        return "_Node(used=" + std::string(node.used ? "True" : "False") + ", x=" + std::to_string(node.x) +
               ", y=" + std::to_string(node.y) + ", w=" + std::to_string(node.w) + ", h=" + std::to_string(node.h) +
               ")";
    }

    std::deque<PackNode> nodes_;  // owns every node; a deque keeps the pointers stable
    PackNode* root_;
};

}  // namespace

std::vector<std::pair<double, double>> pack2d(const std::vector<std::pair<double, double>>& sizes) {
    if (sizes.empty()) throw std::invalid_argument("no objects to pack");
    struct Item {
        std::size_t index;
        double w, h;
    };
    std::vector<Item> sorted_objects;
    sorted_objects.reserve(sizes.size());
    for (std::size_t i = 0; i < sizes.size(); ++i) sorted_objects.push_back({i, sizes[i].first, sizes[i].second});
    // Python's sorted(..., reverse=True) is stable: equal keys keep their order
    std::stable_sort(sorted_objects.begin(), sorted_objects.end(),
                     [](const Item& a, const Item& b) { return std::min(a.w, a.h) > std::min(b.w, b.h); });
    std::stable_sort(sorted_objects.begin(), sorted_objects.end(),
                     [](const Item& a, const Item& b) { return std::max(a.w, a.h) > std::max(b.w, b.h); });

    Packer packer(sorted_objects[0].w, sorted_objects[0].h);
    std::vector<std::pair<double, double>> translations(sizes.size());
    for (const Item& o : sorted_objects) {
        const PackNode* node = packer.place(o.w, o.h);
        translations[o.index] = {node->x, node->y};
    }
    return translations;
}

std::vector<Location> pack_locations(const std::vector<Shape>& objects, double padding, bool align_z) {
    std::vector<std::pair<double, double>> sizes;
    sizes.reserve(objects.size());
    for (const Shape& o : objects) {
        const Vector size = o.bounding_box().size + Vector(padding, padding);
        sizes.emplace_back(size.X, size.Y);
    }
    const std::vector<std::pair<double, double>> translations = pack2d(sizes);
    std::vector<Location> locations;
    locations.reserve(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const BoundBox bb = objects[i].bounding_box();
        locations.push_back(Location(Vector(translations[i].first - bb.min.X, translations[i].second - bb.min.Y, 0)) *
                            Pos(Vector(0, 0, align_z ? -bb.min.Z : 0)));
    }
    return locations;
}

void check_pack_overlap(const std::vector<Shape>& packed) {
    // Whether the intersection of the two bounding boxes has an area
    const auto overlapping = [](const BoundBox& bb1, const BoundBox& bb2) {
        const double min_x = std::max(bb1.min.X, bb2.min.X);
        const double min_y = std::max(bb1.min.Y, bb2.min.Y);
        const double max_x = std::min(bb1.max.X, bb2.max.X);
        const double max_y = std::min(bb1.max.Y, bb2.max.Y);
        return max_x > min_x && max_y > min_y;
    };
    std::vector<BoundBox> bb;
    bb.reserve(packed.size());
    for (const Shape& t : packed) bb.push_back(t.bounding_box());
    for (std::size_t i = 0; i < bb.size(); ++i) {
        for (std::size_t j = i + 1; j < bb.size(); ++j) {
            // Python reports j as an index into the boxes after i
            if (overlapping(bb[i], bb[j])) {
                throw std::runtime_error("Objects at indexes " + std::to_string(i) + " and " +
                                         std::to_string(j - i - 1) + " overlap!");
            }
        }
    }
}

}  // namespace detail

}  // namespace build123d
