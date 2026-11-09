// Joints (build123d/joints.py and topology/shape_core.py Joint) and the JointMap that holds
// them in Shape::joints.
#include "build123d/joints.hpp"

#include <algorithm>
#include <cstdio>

namespace build123d {

using detail::JointKind;
using detail::JointState;

namespace {

std::string number(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

std::string range_text(const std::pair<double, double>& r) {
    return "(" + number(r.first) + ", " + number(r.second) + ")";
}

const char* kind_name(JointKind kind) {
    switch (kind) {
        case JointKind::RIGID: return "RigidJoint";
        case JointKind::REVOLUTE: return "RevoluteJoint";
        case JointKind::LINEAR: return "LinearJoint";
        case JointKind::CYLINDRICAL: return "CylindricalJoint";
        case JointKind::BALL: return "BallJoint";
    }
    return "Joint";
}

// Python raises TypeError for keyword arguments a keyword-only signature doesn't accept.
void reject_args(const JointArgs& args, bool position, bool angle, bool angles) {
    auto bad = [](const char* name) {
        throw std::invalid_argument(std::string("connect_to() got an unexpected keyword argument '") + name + "'");
    };
    if (!position && args.position) bad("position");
    if (!angle && args.angle) bad("angle");
    if (!angles && args.angles) bad("angles");
}

Location axis_location(const Axis& axis) { return axis.location(); }

// Plane(origin=(0, 0, 0), x_dir=..., z_dir=...).location
Location rotation_about(const Vector& x_dir, const Vector& z_dir) {
    return Location(Plane(Vector(0, 0, 0), x_dir, z_dir));
}

// Revolute: relative_to(other: RigidJoint, angle)
Location revolute_relative_to(JointState& self, const JointState& other, std::optional<double> angle) {
    if (other.kind != JointKind::RIGID) {
        throw std::invalid_argument(std::string("other must of type RigidJoint not ") + kind_name(other.kind));
    }
    double angle_degrees = angle ? *angle : self.angular_range.first;
    if (angle_degrees < self.angular_range.first || angle_degrees > self.angular_range.second) {
        throw std::invalid_argument("angle (" + number(angle_degrees) + ") must in range of " +
                                    range_text(self.angular_range));
    }
    self.angle = angle_degrees;
    angle_degrees = angle_degrees == 0.0 ? 360.0 : angle_degrees;
    Location rotation;
    if (self.angle_reference_provided) {
        rotation = rotation_about(self.angle_reference.rotate(self.relative_axis, angle_degrees),
                                  self.relative_axis.direction);
    } else {
        rotation = Rotation(0, 0, angle_degrees);
    }
    return axis_location(self.relative_axis) * rotation * other.relative_location.inverse();
}

// Linear: relative_to(other: RigidJoint | RevoluteJoint, position, angle)
Location linear_relative_to(JointState& self, const JointState& other, std::optional<double> position,
                            std::optional<double> angle) {
    if (other.kind != JointKind::RIGID && other.kind != JointKind::REVOLUTE) {
        throw std::invalid_argument(std::string("other must of type RigidJoint or RevoluteJoint not ") +
                                    kind_name(other.kind));
    }
    const double pos = position ? *position : (self.linear_range.first + self.linear_range.second) / 2;
    if (!(self.linear_range.first <= pos && pos <= self.linear_range.second)) {
        throw std::invalid_argument("position (" + number(pos) + ") must in range of " +
                                    range_text(self.linear_range));
    }
    self.position = pos;
    Location rotation;
    double angle_value = 0.0;
    if (other.kind == JointKind::REVOLUTE) {
        angle_value = angle ? *angle : other.angular_range.first;
        if (!(other.angular_range.first <= angle_value && angle_value <= other.angular_range.second)) {
            throw std::invalid_argument("angle (" + number(angle_value) + ") must in range of " +
                                        range_text(other.angular_range));
        }
        rotation = rotation_about(other.angle_reference.rotate(other.relative_axis, angle_value),
                                  other.relative_axis.direction);
    }
    self.angle = angle_value;
    const Location joint_relative_position =
        Location(self.relative_axis.position + self.relative_axis.direction * pos) * rotation;
    const Location other_relative_location = other.kind == JointKind::REVOLUTE
                                                 ? Location(other.relative_axis.position)
                                                 : other.relative_location;
    return joint_relative_position * other_relative_location.inverse();
}

// Cylindrical: relative_to(other: RigidJoint, position, angle)
Location cylindrical_relative_to(JointState& self, const JointState& other, std::optional<double> position,
                                 std::optional<double> angle) {
    if (other.kind != JointKind::RIGID) {
        throw std::invalid_argument(std::string("other must of type RigidJoint not ") + kind_name(other.kind));
    }
    const double pos = position ? *position : (self.linear_range.first + self.linear_range.second) / 2;
    if (!(self.linear_range.first <= pos && pos <= self.linear_range.second)) {
        throw std::invalid_argument("position (" + number(pos) + ") must in range of " +
                                    range_text(self.linear_range));
    }
    self.position = pos;
    const double angle_value = angle ? *angle : (self.angular_range.first + self.angular_range.second) / 2;
    if (!(self.angular_range.first <= angle_value && angle_value <= self.angular_range.second)) {
        throw std::invalid_argument("angle (" + number(angle_value) + ") must in range of " +
                                    range_text(self.angular_range));
    }
    self.angle = angle_value;
    const Location joint_relative_position(self.relative_axis.position + self.relative_axis.direction * pos);
    const Location joint_rotation =
        rotation_about(self.angle_reference.rotate(self.relative_axis, angle_value), self.relative_axis.direction);
    return joint_relative_position * joint_rotation * other.relative_location.inverse();
}

// Ball: relative_to(other: RigidJoint, angles)
Location ball_relative_to(JointState& self, const JointState& other, const std::optional<Rotation>& angles) {
    if (other.kind != JointKind::RIGID) {
        throw std::invalid_argument(std::string("other must of type RigidJoint not ") + kind_name(other.kind));
    }
    const Rotation angle_rotation = angles ? *angles
                                           : Rotation(self.ball_angular_range[0].first,
                                                      self.ball_angular_range[1].first,
                                                      self.ball_angular_range[2].first);
    const Location rotation = angle_rotation * self.ball_angle_reference.location();
    const Vector orientation = rotation.orientation();
    const double values[3] = {orientation.X, orientation.Y, orientation.Z};
    for (int i = 0; i < 3; ++i) {
        if (!(self.ball_angular_range[i].first <= values[i] && values[i] <= self.ball_angular_range[i].second)) {
            const Vector a = angle_rotation.orientation();
            throw std::invalid_argument("angles ((" + number(a.X) + ", " + number(a.Y) + ", " + number(a.Z) +
                                        ")) must in range of (" + range_text(self.ball_angular_range[0]) + ", " +
                                        range_text(self.ball_angular_range[1]) + ", " +
                                        range_text(self.ball_angular_range[2]) + ")");
        }
    }
    return self.relative_location * rotation * other.relative_location.inverse();
}

// Rigid: relative_to(other: any joint, **kwargs)
Location rigid_relative_to(JointState& self, JointState& other, const JointArgs& args) {
    switch (other.kind) {
        case JointKind::RIGID: return self.relative_location * other.relative_location.inverse();
        case JointKind::REVOLUTE: return revolute_relative_to(other, self, args.angle).inverse();
        case JointKind::LINEAR: return linear_relative_to(other, self, args.position, std::nullopt).inverse();
        case JointKind::CYLINDRICAL:
            return cylindrical_relative_to(other, self, args.position, args.angle).inverse();
        case JointKind::BALL: return ball_relative_to(other, self, args.angles).inverse();
    }
    throw std::invalid_argument("other must one of type BallJoint, CylindricalJoint, LinearJoint, "
                                "RevoluteJoint, RigidJoint");
}

}  // namespace

// ---------------------------------------------------------------------------
// Joint
// ---------------------------------------------------------------------------

JointState& Joint::state() const {
    if (!state_) throw std::invalid_argument("Empty joint");
    return *state_;
}

std::shared_ptr<JointState> Joint::attach(JointState state, Shape& to_part) {
    auto shared = std::make_shared<JointState>(std::move(state));
    shared->parent = &to_part;
    to_part.joints.insert(shared->label, Joint(shared));
    return shared;
}

const std::string& Joint::label() const { return state().label; }

Shape& Joint::parent() const {
    JointState& s = state();
    if (s.parent == nullptr) {
        throw std::runtime_error("The joint '" + s.label + "' is no longer bound to a shape");
    }
    return *s.parent;
}

std::optional<Joint> Joint::connected_to() const {
    if (auto other = state().connected_to.lock()) return Joint(other);
    return std::nullopt;
}

Location Joint::location() const {
    const JointState& s = state();
    const Location parent_location = parent().location();
    switch (s.kind) {
        case JointKind::RIGID:
        case JointKind::BALL: return parent_location * s.relative_location;
        case JointKind::REVOLUTE:
        case JointKind::LINEAR:
        case JointKind::CYLINDRICAL: return parent_location * axis_location(s.relative_axis);
    }
    return parent_location;
}

Compound Joint::symbol() const {
    const JointState& s = state();
    switch (s.kind) {
        case JointKind::REVOLUTE: {
            const double radius = parent().bounding_box(std::nullopt, false).diagonal() / 30;
            return Compound(std::vector<Edge>{Edge::make_line(Vector(0, 0, 0), Vector(0, 0, radius * 10)),
                                              Edge::make_circle(radius),
                                              Edge::make_line(Vector(0, 0, 0), Vector(radius, 0, 0))})
                .moved(location());
        }
        case JointKind::LINEAR:
        case JointKind::CYLINDRICAL: {
            const double radius = (s.linear_range.second - s.linear_range.first) / 15;
            std::vector<Edge> edges{
                Edge::make_line(Vector(0, 0, s.linear_range.first), Vector(0, 0, s.linear_range.second)),
                Edge::make_circle(radius)};
            if (s.kind == JointKind::CYLINDRICAL) edges.push_back(Edge::make_line(Vector(0, 0, 0), Vector(radius, 0, 0)));
            return Compound(edges).moved(location());
        }
        case JointKind::RIGID:
        case JointKind::BALL: break;
    }
    throw std::logic_error("symbol() of this joint type needs Compound::make_triad and single-line text");
}

void Joint::connect_to(const Joint& other, const JointArgs& args) const {
    JointState& self = state();
    JointState& other_state = other.state();
    switch (self.kind) {
        case JointKind::REVOLUTE: reject_args(args, false, true, false); break;
        case JointKind::CYLINDRICAL: reject_args(args, true, true, false); break;
        case JointKind::BALL: reject_args(args, false, false, true); break;
        case JointKind::RIGID:
        case JointKind::LINEAR: break;  // **kwargs in build123d
    }
    const Location relative_location = relative_to(other, args);
    Shape& other_parent = other.parent();
    other_parent.locate(parent().location() * relative_location);
    self.connected_to = other.state_;
    (void)other_state;
}

Location Joint::relative_to(const Joint& other, const JointArgs& args) const {
    JointState& self = state();
    JointState& other_state = other.state();
    switch (self.kind) {
        case JointKind::RIGID: return rigid_relative_to(self, other_state, args);
        case JointKind::REVOLUTE: return revolute_relative_to(self, other_state, args.angle);
        case JointKind::LINEAR: return linear_relative_to(self, other_state, args.position, args.angle);
        case JointKind::CYLINDRICAL: return cylindrical_relative_to(self, other_state, args.position, args.angle);
        case JointKind::BALL: return ball_relative_to(self, other_state, args.angles);
    }
    return Location();
}

void Joint::reparent(Shape& new_parent) const {
    JointState& s = state();
    const Location relative_to_new_parent = new_parent.location().inverse() * parent().location();
    switch (s.kind) {
        case JointKind::RIGID:
        case JointKind::BALL: s.relative_location = relative_to_new_parent * s.relative_location; break;
        case JointKind::REVOLUTE:
        case JointKind::LINEAR:
        case JointKind::CYLINDRICAL: s.relative_axis = s.relative_axis.located(relative_to_new_parent); break;
    }
    if (s.parent != &new_parent) {
        Shape* old_parent = s.parent;
        new_parent.joints.insert(s.label, *this);
        if (old_parent != nullptr && old_parent->joints.contains(s.label) &&
            old_parent->joints[s.label] == *this) {
            // Moving the joint: take it out of the old part without detaching it.
            auto& items = old_parent->joints.items_;
            items.erase(std::remove_if(items.begin(), items.end(),
                                       [&](const auto& item) { return item.second == state_; }),
                        items.end());
        }
    }
    s.parent = &new_parent;
}

// ---------------------------------------------------------------------------
// The joint types
// ---------------------------------------------------------------------------

RigidJoint::RigidJoint(const std::string& label, Shape& to_part, const Location& joint_location) {
    JointState s;
    s.kind = JointKind::RIGID;
    s.label = label;
    s.relative_location = to_part.location().inverse() * joint_location;
    state_ = attach(std::move(s), to_part);
}

Location RigidJoint::relative_location() const { return state().relative_location; }
void RigidJoint::set_relative_location(const Location& location) const { state().relative_location = location; }

void RigidJoint::connect_to(const RigidJoint& other) const { Joint::connect_to(other); }
void RigidJoint::connect_to(const RevoluteJoint& other, std::optional<double> angle) const {
    Joint::connect_to(other, {.angle = angle});
}
void RigidJoint::connect_to(const LinearJoint& other, std::optional<double> position) const {
    Joint::connect_to(other, {.position = position});
}
void RigidJoint::connect_to(const CylindricalJoint& other, std::optional<double> position,
                            std::optional<double> angle) const {
    Joint::connect_to(other, {.position = position, .angle = angle});
}
void RigidJoint::connect_to(const BallJoint& other, std::optional<Rotation> angles) const {
    Joint::connect_to(other, {.angles = angles});
}

Location RigidJoint::relative_to(const RigidJoint& other) const { return Joint::relative_to(other); }
Location RigidJoint::relative_to(const RevoluteJoint& other, std::optional<double> angle) const {
    return Joint::relative_to(other, {.angle = angle});
}
Location RigidJoint::relative_to(const LinearJoint& other, std::optional<double> position) const {
    return Joint::relative_to(other, {.position = position});
}
Location RigidJoint::relative_to(const CylindricalJoint& other, std::optional<double> position,
                                 std::optional<double> angle) const {
    return Joint::relative_to(other, {.position = position, .angle = angle});
}
Location RigidJoint::relative_to(const BallJoint& other, std::optional<Rotation> angles) const {
    return Joint::relative_to(other, {.angles = angles});
}

RevoluteJoint::RevoluteJoint(const std::string& label, Shape& to_part, const Axis& axis,
                             std::optional<Vector> angle_reference, std::pair<double, double> angular_range) {
    JointState s;
    s.kind = JointKind::REVOLUTE;
    s.label = label;
    s.angular_range = angular_range;
    // build123d tests the truth of the vector: a zero vector counts as not provided.
    s.angle_reference_provided = angle_reference.has_value();
    if (angle_reference && angle_reference->length() != 0.0) {
        if (!axis.is_normal(Axis(Vector(0, 0, 0), *angle_reference))) {
            throw std::invalid_argument("angle_reference must be normal to axis");
        }
        s.angle_reference = *angle_reference;
    } else {
        s.angle_reference = Plane(Vector(0, 0, 0), std::nullopt, axis.direction).x_dir;
    }
    s.relative_axis = axis.located(to_part.location().inverse());
    state_ = attach(std::move(s), to_part);
}

Axis RevoluteJoint::relative_axis() const { return state().relative_axis; }
void RevoluteJoint::set_relative_axis(const Axis& axis) const { state().relative_axis = axis; }
Vector RevoluteJoint::angle_reference() const { return state().angle_reference; }
std::pair<double, double> RevoluteJoint::angular_range() const { return state().angular_range; }
void RevoluteJoint::set_angular_range(std::pair<double, double> range) const { state().angular_range = range; }
std::optional<double> RevoluteJoint::angle() const { return state().angle; }

void RevoluteJoint::connect_to(const RigidJoint& other, std::optional<double> angle) const {
    Joint::connect_to(other, {.angle = angle});
}
Location RevoluteJoint::relative_to(const RigidJoint& other, std::optional<double> angle) const {
    return Joint::relative_to(other, {.angle = angle});
}

LinearJoint::LinearJoint(const std::string& label, Shape& to_part, const Axis& axis,
                         std::pair<double, double> linear_range) {
    JointState s;
    s.kind = JointKind::LINEAR;
    s.label = label;
    s.axis = axis;
    s.linear_range = linear_range;
    s.relative_axis = axis.located(to_part.location().inverse());
    state_ = attach(std::move(s), to_part);
}

Axis LinearJoint::axis() const { return state().axis; }
Axis LinearJoint::relative_axis() const { return state().relative_axis; }
void LinearJoint::set_relative_axis(const Axis& axis) const { state().relative_axis = axis; }
std::pair<double, double> LinearJoint::linear_range() const { return state().linear_range; }
void LinearJoint::set_linear_range(std::pair<double, double> range) const { state().linear_range = range; }
std::optional<double> LinearJoint::position() const { return state().position; }
std::optional<double> LinearJoint::angle() const { return state().angle; }

void LinearJoint::connect_to(const RigidJoint& other, std::optional<double> position) const {
    Joint::connect_to(other, {.position = position});
}
void LinearJoint::connect_to(const RevoluteJoint& other, std::optional<double> position,
                             std::optional<double> angle) const {
    Joint::connect_to(other, {.position = position, .angle = angle});
}
Location LinearJoint::relative_to(const RigidJoint& other, std::optional<double> position) const {
    return Joint::relative_to(other, {.position = position});
}
Location LinearJoint::relative_to(const RevoluteJoint& other, std::optional<double> position,
                                  std::optional<double> angle) const {
    return Joint::relative_to(other, {.position = position, .angle = angle});
}

CylindricalJoint::CylindricalJoint(const std::string& label, Shape& to_part, const Axis& axis,
                                   std::optional<Vector> angle_reference, std::pair<double, double> linear_range,
                                   std::pair<double, double> angular_range) {
    JointState s;
    s.kind = JointKind::CYLINDRICAL;
    s.label = label;
    s.axis = axis;
    if (angle_reference && angle_reference->length() != 0.0) {
        if (!axis.is_normal(Axis(Vector(0, 0, 0), *angle_reference))) {
            throw std::invalid_argument("angle_reference must be normal to axis");
        }
        s.angle_reference = *angle_reference;
    } else {
        s.angle_reference = Plane(Vector(0, 0, 0), std::nullopt, axis.direction).x_dir;
    }
    s.angular_range = angular_range;
    s.linear_range = linear_range;
    s.relative_axis = axis.located(to_part.location().inverse());
    state_ = attach(std::move(s), to_part);
}

Axis CylindricalJoint::axis() const { return state().axis; }
Axis CylindricalJoint::relative_axis() const { return state().relative_axis; }
void CylindricalJoint::set_relative_axis(const Axis& axis) const { state().relative_axis = axis; }
Vector CylindricalJoint::angle_reference() const { return state().angle_reference; }
std::pair<double, double> CylindricalJoint::linear_range() const { return state().linear_range; }
void CylindricalJoint::set_linear_range(std::pair<double, double> range) const { state().linear_range = range; }
std::pair<double, double> CylindricalJoint::angular_range() const { return state().angular_range; }
void CylindricalJoint::set_angular_range(std::pair<double, double> range) const { state().angular_range = range; }
std::optional<double> CylindricalJoint::position() const { return state().position; }
std::optional<double> CylindricalJoint::angle() const { return state().angle; }

void CylindricalJoint::connect_to(const RigidJoint& other, std::optional<double> position,
                                  std::optional<double> angle) const {
    Joint::connect_to(other, {.position = position, .angle = angle});
}
Location CylindricalJoint::relative_to(const RigidJoint& other, std::optional<double> position,
                                       std::optional<double> angle) const {
    return Joint::relative_to(other, {.position = position, .angle = angle});
}

BallJoint::BallJoint(const std::string& label, Shape& to_part, const Location& joint_location,
                     const AngularRange& angular_range, const Plane& angle_reference) {
    JointState s;
    s.kind = JointKind::BALL;
    s.label = label;
    s.relative_location = to_part.location().inverse() * joint_location;
    s.ball_angular_range = angular_range;
    s.ball_angle_reference = angle_reference;
    state_ = attach(std::move(s), to_part);
}

Location BallJoint::relative_location() const { return state().relative_location; }
void BallJoint::set_relative_location(const Location& location) const { state().relative_location = location; }
BallJoint::AngularRange BallJoint::angular_range() const { return state().ball_angular_range; }
void BallJoint::set_angular_range(const AngularRange& range) const { state().ball_angular_range = range; }
Plane BallJoint::angle_reference() const { return state().ball_angle_reference; }

void BallJoint::connect_to(const RigidJoint& other, std::optional<Rotation> angles) const {
    Joint::connect_to(other, {.angles = angles});
}
Location BallJoint::relative_to(const RigidJoint& other, std::optional<Rotation> angles) const {
    return Joint::relative_to(other, {.angles = angles});
}

// ---------------------------------------------------------------------------
// JointMap
// ---------------------------------------------------------------------------

JointMap::JointMap(const JointMap& other) {
    items_.reserve(other.items_.size());
    for (const auto& [label, state] : other.items_) {
        items_.emplace_back(label, std::make_shared<JointState>(*state));
    }
}

JointMap::JointMap(JointMap&& other) noexcept : items_(std::move(other.items_)) { other.items_.clear(); }

JointMap& JointMap::operator=(const JointMap& other) {
    if (this != &other) {
        detach_all();
        items_.clear();
        for (const auto& [label, state] : other.items_) {
            items_.emplace_back(label, std::make_shared<JointState>(*state));
        }
        rebind(owner_);
    }
    return *this;
}

JointMap& JointMap::operator=(JointMap&& other) noexcept {
    if (this != &other) {
        detach_all();
        items_ = std::move(other.items_);
        other.items_.clear();
        rebind(owner_);
    }
    return *this;
}

JointMap::~JointMap() { detach_all(); }

void JointMap::rebind(Shape* owner) {
    owner_ = owner;
    for (auto& item : items_) item.second->parent = owner;
}

void JointMap::detach_all() {
    for (auto& item : items_) {
        if (item.second->parent == owner_) item.second->parent = nullptr;
    }
}

Joint JointMap::at(const std::string& label) const {
    for (const auto& item : items_) {
        if (item.first == label) return Joint(item.second);
    }
    throw std::out_of_range("No joint labelled '" + label + "'");
}

Joint JointMap::operator[](const std::string& label) const { return at(label); }

bool JointMap::contains(const std::string& label) const { return count(label) > 0; }

std::size_t JointMap::count(const std::string& label) const {
    return static_cast<std::size_t>(
        std::count_if(items_.begin(), items_.end(), [&](const auto& item) { return item.first == label; }));
}

std::size_t JointMap::size() const { return items_.size(); }
bool JointMap::empty() const { return items_.empty(); }

std::vector<std::string> JointMap::keys() const {
    std::vector<std::string> out;
    for (const auto& item : items_) out.push_back(item.first);
    return out;
}

std::vector<Joint> JointMap::values() const {
    std::vector<Joint> out;
    for (const auto& item : items_) out.push_back(Joint(item.second));
    return out;
}

std::vector<std::pair<std::string, Joint>> JointMap::items() const {
    std::vector<std::pair<std::string, Joint>> out;
    for (const auto& item : items_) out.emplace_back(item.first, Joint(item.second));
    return out;
}

void JointMap::insert(const std::string& label, const Joint& joint) {
    if (!joint.state_) throw std::invalid_argument("Empty joint");
    for (auto& item : items_) {
        if (item.first == label) {
            if (item.second != joint.state_ && item.second->parent == owner_) item.second->parent = nullptr;
            item.second = joint.state_;
            joint.state_->parent = owner_;
            return;
        }
    }
    items_.emplace_back(label, joint.state_);
    joint.state_->parent = owner_;
}

void JointMap::erase(const std::string& label) {
    for (auto it = items_.begin(); it != items_.end(); ++it) {
        if (it->first == label) {
            if (it->second->parent == owner_) it->second->parent = nullptr;
            items_.erase(it);
            return;
        }
    }
}

void JointMap::clear() {
    detach_all();
    items_.clear();
}

}  // namespace build123d
