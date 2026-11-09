// The assembly tree: Shape::children (ShapeChildren), parent links and tree navigation,
// mirroring the anytree NodeMixin that build123d's Shape inherits.
#include <algorithm>

#include <TopoDS.hxx>

#include "internal.hpp"

namespace build123d {

// ---------------------------------------------------------------------------
// ShapeChildren
// ---------------------------------------------------------------------------

// The owner is set by Shape::bind() once the containing shape exists.
ShapeChildren::ShapeChildren(const ShapeChildren& other) : items_(other.items_) {}
ShapeChildren::ShapeChildren(ShapeChildren&& other) noexcept : items_(std::move(other.items_)) {}
ShapeChildren::~ShapeChildren() = default;

ShapeChildren& ShapeChildren::operator=(const ShapeChildren& other) {
    if (this != &other) {
        const bool had_children = !items_.empty();
        items_ = other.items_;
        changed(had_children);
    }
    return *this;
}

ShapeChildren& ShapeChildren::operator=(ShapeChildren&& other) {
    if (this != &other) {
        const bool had_children = !items_.empty();
        items_ = std::move(other.items_);
        other.items_.clear();
        changed(had_children);
    }
    return *this;
}

ShapeChildren& ShapeChildren::operator=(const std::vector<Shape>& shapes) {
    const bool had_children = !items_.empty();
    items_ = shapes;
    changed(had_children);
    return *this;
}

ShapeChildren& ShapeChildren::operator=(std::initializer_list<Shape> shapes) {
    const bool had_children = !items_.empty();
    items_.assign(shapes.begin(), shapes.end());
    changed(had_children);
    return *this;
}

ShapeChildren::size_type ShapeChildren::size() const { return items_.size(); }
bool ShapeChildren::empty() const { return items_.empty(); }
ShapeChildren::iterator ShapeChildren::begin() { return items_.begin(); }
ShapeChildren::iterator ShapeChildren::end() { return items_.end(); }
ShapeChildren::const_iterator ShapeChildren::begin() const { return items_.begin(); }
ShapeChildren::const_iterator ShapeChildren::end() const { return items_.end(); }
ShapeChildren::const_iterator ShapeChildren::cbegin() const { return items_.cbegin(); }
ShapeChildren::const_iterator ShapeChildren::cend() const { return items_.cend(); }
Shape& ShapeChildren::operator[](size_type i) { return items_[i]; }
const Shape& ShapeChildren::operator[](size_type i) const { return items_[i]; }
Shape& ShapeChildren::at(size_type i) { return items_.at(i); }
const Shape& ShapeChildren::at(size_type i) const { return items_.at(i); }
Shape& ShapeChildren::front() { return items_.front(); }
const Shape& ShapeChildren::front() const { return items_.front(); }
Shape& ShapeChildren::back() { return items_.back(); }
const Shape& ShapeChildren::back() const { return items_.back(); }

void ShapeChildren::push_back(const Shape& child) {
    const bool had_children = !items_.empty();
    items_.push_back(child);
    changed(had_children);
}

void ShapeChildren::push_back(Shape&& child) {
    const bool had_children = !items_.empty();
    items_.push_back(std::move(child));
    changed(had_children);
}

ShapeChildren::iterator ShapeChildren::insert(const_iterator pos, const Shape& child) {
    const bool had_children = !items_.empty();
    const auto index = pos - items_.cbegin();
    items_.insert(pos, child);
    changed(had_children);
    return items_.begin() + index;
}

ShapeChildren::iterator ShapeChildren::erase(const_iterator pos) {
    const bool had_children = !items_.empty();
    const auto index = pos - items_.cbegin();
    items_.erase(pos);
    changed(had_children);
    return items_.begin() + index;
}

ShapeChildren::iterator ShapeChildren::erase(const_iterator first, const_iterator last) {
    const bool had_children = !items_.empty();
    const auto index = first - items_.cbegin();
    items_.erase(first, last);
    changed(had_children);
    return items_.begin() + index;
}

void ShapeChildren::pop_back() {
    items_.pop_back();
    changed(true);
}

void ShapeChildren::clear() {
    const bool had_children = !items_.empty();
    items_.clear();
    changed(had_children);
}

void ShapeChildren::reserve(size_type n) {
    items_.reserve(n);
    relink();
}

void ShapeChildren::relink() {
    for (auto& child : items_) child.tree_parent_ = owner_;
}

void ShapeChildren::changed(bool had_children) {
    relink();
    // build123d rebuilds the compound when children are attached, and when some are detached.
    if (owner_ != nullptr && (had_children || !items_.empty())) owner_->rebuild_from_children();
}

void ShapeChildren::assign_raw(const std::vector<Shape>& shapes) {
    if (&shapes != &items_) items_ = shapes;
    relink();
}

// ---------------------------------------------------------------------------
// Shape: copies and the tree
// ---------------------------------------------------------------------------

Shape::Shape(const Shape& other)
    : label(other.label),
      color(other.color),
      children(other.children),
      joints(other.joints),
      obj_(other.obj_),
      parent_(other.parent_) {
    bind();
}

Shape::Shape(Shape&& other) noexcept
    : label(std::move(other.label)),
      color(std::move(other.color)),
      children(std::move(other.children)),
      joints(std::move(other.joints)),
      obj_(std::move(other.obj_)),
      parent_(std::move(other.parent_)) {
    bind();
}

// Assignment replaces the content; the shape keeps its own place in a tree.
Shape& Shape::operator=(const Shape& other) {
    if (this != &other) {
        label = other.label;
        color = other.color;
        children.assign_raw(other.children.items_);
        joints = other.joints;
        obj_ = other.obj_;
        parent_ = other.parent_;
        bind();
    }
    return *this;
}

Shape& Shape::operator=(Shape&& other) noexcept {
    if (this != &other) {
        label = std::move(other.label);
        color = std::move(other.color);
        children.items_ = std::move(other.children.items_);
        other.children.items_.clear();
        joints = std::move(other.joints);
        obj_ = std::move(other.obj_);
        parent_ = std::move(other.parent_);
        bind();
    }
    return *this;
}

void Shape::bind() {
    children.owner_ = this;
    children.relink();
    joints.rebind(this);
}

void Shape::rebuild_from_children() {
    if (!obj_.IsNull() && obj_.ShapeType() != TopAbs_COMPOUND) return;
    std::vector<TopoDS_Shape> shapes;
    for (const auto& child : children.items_) {
        if (!child.is_null()) shapes.push_back(child.wrapped());
    }
    obj_ = detail::make_compound(shapes);
}

Shape& Shape::root() {
    Shape* node = this;
    while (node->tree_parent_ != nullptr) node = node->tree_parent_;
    return *node;
}

const Shape& Shape::root() const {
    const Shape* node = this;
    while (node->tree_parent_ != nullptr) node = node->tree_parent_;
    return *node;
}

std::vector<const Shape*> Shape::iter_path_reverse() const {
    std::vector<const Shape*> nodes;
    for (const Shape* node = this; node != nullptr; node = node->tree_parent_) nodes.push_back(node);
    return nodes;
}

std::vector<const Shape*> Shape::path() const {
    auto nodes = iter_path_reverse();
    std::reverse(nodes.begin(), nodes.end());
    return nodes;
}

std::vector<Shape*> Shape::path() {
    std::vector<Shape*> nodes;
    for (Shape* node = this; node != nullptr; node = node->tree_parent_) nodes.push_back(node);
    std::reverse(nodes.begin(), nodes.end());
    return nodes;
}

std::vector<const Shape*> Shape::ancestors() const {
    auto nodes = path();
    nodes.pop_back();
    return nodes;
}

std::vector<Shape*> Shape::ancestors() {
    auto nodes = path();
    nodes.pop_back();
    return nodes;
}

namespace {

template <class S, class Out>
void preorder(S& node, Out& out, bool include_self) {
    if (include_self) out.push_back(&node);
    for (auto& child : node.children) preorder(child, out, true);
}

}  // namespace

std::vector<const Shape*> Shape::descendants() const {
    std::vector<const Shape*> nodes;
    preorder(*this, nodes, false);
    return nodes;
}

std::vector<Shape*> Shape::descendants() {
    std::vector<Shape*> nodes;
    preorder(*this, nodes, false);
    return nodes;
}

std::vector<const Shape*> Shape::leaves() const {
    std::vector<const Shape*> nodes;
    preorder(*this, nodes, true);
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](const Shape* n) { return !n->is_leaf(); }),
                nodes.end());
    return nodes;
}

std::vector<Shape*> Shape::leaves() {
    std::vector<Shape*> nodes;
    preorder(*this, nodes, true);
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](const Shape* n) { return !n->is_leaf(); }),
                nodes.end());
    return nodes;
}

std::vector<const Shape*> Shape::siblings() const {
    std::vector<const Shape*> nodes;
    if (tree_parent_ == nullptr) return nodes;
    for (const auto& child : tree_parent_->children) {
        if (&child != this) nodes.push_back(&child);
    }
    return nodes;
}

std::vector<Shape*> Shape::siblings() {
    std::vector<Shape*> nodes;
    if (tree_parent_ == nullptr) return nodes;
    for (auto& child : tree_parent_->children) {
        if (&child != this) nodes.push_back(&child);
    }
    return nodes;
}

std::size_t Shape::depth() const {
    std::size_t d = 0;
    for (const Shape* node = tree_parent_; node != nullptr; node = node->tree_parent_) ++d;
    return d;
}

std::size_t Shape::height() const {
    std::size_t h = 0;
    for (const auto& child : children) h = std::max(h, child.height() + 1);
    return h;
}

std::size_t Shape::tree_size() const {
    std::size_t n = 1;
    for (const auto& child : children) n += child.tree_size();
    return n;
}

Location Shape::global_location() const {
    Location result;
    for (const Shape* node : path()) {
        if (node->is_null()) throw std::invalid_argument("Can't find the location of an empty shape");
        result = result * node->location();
    }
    return result;
}

}  // namespace build123d
