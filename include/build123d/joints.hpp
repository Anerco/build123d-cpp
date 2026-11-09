// Joints: connection points on parts that position other parts (build123d/joints.py).
//
//   Part base = Box(10, 10, 10);
//   Part lid = Box(10, 10, 1);
//   RevoluteJoint hinge("hinge", base, Axis({5, 0, 5}, {0, 1, 0}), {0, 180});
//   RigidJoint("edge", lid, Location({5, 0, 0}));
//   hinge.connect_to(lid.joints.get<RigidJoint>("edge"), 30);   // moves lid
//   base.joints["hinge"].connect_to(lid.joints["edge"], {.angle = 30});  // the same
//
// A joint lives in the `joints` of the shape it is bound to; joint objects are handles to it,
// so every copy of a handle refers to the same joint. Copying a shape gives the copy its own
// joints, bound to the copy (like build123d's deepcopy). Connecting two joints moves the part
// of the second joint. A handle whose part has been destroyed throws std::runtime_error.
#pragma once

#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "build123d/topology.hpp"

namespace build123d {

class RigidJoint;
class RevoluteJoint;
class LinearJoint;
class CylindricalJoint;
class BallJoint;

/// build123d's keyword arguments of connect_to / relative_to.
struct JointArgs {
    /// Linear position along the axis (LinearJoint, CylindricalJoint).
    std::optional<double> position = std::nullopt;
    /// Rotation angle in degrees (RevoluteJoint, CylindricalJoint, LinearJoint to a RevoluteJoint).
    std::optional<double> angle = std::nullopt;
    /// Rotation angles in degrees (BallJoint).
    std::optional<Rotation> angles = std::nullopt;
};

namespace detail {

enum class JointKind { RIGID, REVOLUTE, LINEAR, CYLINDRICAL, BALL };

struct JointState {
    JointKind kind = JointKind::RIGID;
    std::string label;
    Shape* parent = nullptr;
    std::weak_ptr<JointState> connected_to;
    // RigidJoint, BallJoint
    Location relative_location;
    // RevoluteJoint, LinearJoint, CylindricalJoint
    Axis relative_axis = Axis::Z;
    Axis axis = Axis::Z;
    Vector angle_reference;
    bool angle_reference_provided = false;
    std::pair<double, double> angular_range{0.0, 360.0};
    std::pair<double, double> linear_range{0.0, std::numeric_limits<double>::infinity()};
    std::optional<double> angle;
    std::optional<double> position;
    // BallJoint
    Plane ball_angle_reference = Plane::XY;
    std::array<std::pair<double, double>, 3> ball_angular_range{{{0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0}}};
};

}  // namespace detail

/// A joint bound to a shape (build123d's Joint). Use the derived classes to create joints.
class Joint {
public:
    /// An empty handle.
    Joint() = default;
    explicit operator bool() const { return static_cast<bool>(state_); }
    bool operator==(const Joint& other) const { return state_ == other.state_; }
    bool operator!=(const Joint& other) const { return state_ != other.state_; }

    const std::string& label() const;
    /// The shape the joint is bound to.
    Shape& parent() const;
    /// The joint this one was last connected to, if it still exists.
    std::optional<Joint> connected_to() const;
    /// Global location of the joint.
    Location location() const;
    /// A small compound showing the joint, for viewers.
    Compound symbol() const;

    /// Move the part of `other` so that it is connected to this joint, with the keyword
    /// arguments build123d accepts for this pair of joint types.
    void connect_to(const Joint& other, const JointArgs& args = {}) const;
    /// The location of `other`'s part relative to this joint's part when connected.
    Location relative_to(const Joint& other, const JointArgs& args = {}) const;

    /// Bind the joint to another shape, keeping its position in space (build123d's
    /// Joint._reparent); it moves into `new_parent.joints`.
    void reparent(Shape& new_parent) const;

    /// Whether the joint is of type J (RigidJoint, RevoluteJoint, ...).
    template <class J>
    bool is() const;
    /// This joint as type J; std::invalid_argument if it is another type.
    template <class J>
    J as() const;

protected:
    std::shared_ptr<detail::JointState> state_;

    explicit Joint(std::shared_ptr<detail::JointState> state) : state_(std::move(state)) {}
    detail::JointState& state() const;
    /// Register the new joint in `to_part.joints[label]`.
    static std::shared_ptr<detail::JointState> attach(detail::JointState state, Shape& to_part);

    friend class JointMap;
};

/// A fixed connection at a location on the part.
class RigidJoint : public Joint {
public:
    static constexpr detail::JointKind kKind = detail::JointKind::RIGID;

    RigidJoint(const std::string& label, Shape& to_part, const Location& joint_location = Location());

    /// Joint location relative to the part.
    Location relative_location() const;
    void set_relative_location(const Location& location) const;

    void connect_to(const RigidJoint& other) const;
    void connect_to(const RevoluteJoint& other, std::optional<double> angle = std::nullopt) const;
    void connect_to(const LinearJoint& other, std::optional<double> position = std::nullopt) const;
    void connect_to(const CylindricalJoint& other, std::optional<double> position = std::nullopt,
                    std::optional<double> angle = std::nullopt) const;
    void connect_to(const BallJoint& other, std::optional<Rotation> angles = std::nullopt) const;
    using Joint::connect_to;

    Location relative_to(const RigidJoint& other) const;
    Location relative_to(const RevoluteJoint& other, std::optional<double> angle = std::nullopt) const;
    Location relative_to(const LinearJoint& other, std::optional<double> position = std::nullopt) const;
    Location relative_to(const CylindricalJoint& other, std::optional<double> position = std::nullopt,
                         std::optional<double> angle = std::nullopt) const;
    Location relative_to(const BallJoint& other, std::optional<Rotation> angles = std::nullopt) const;
    using Joint::relative_to;

private:
    friend class Joint;
    explicit RigidJoint(std::shared_ptr<detail::JointState> state) : Joint(std::move(state)) {}
};

/// A hinge: rotation about an axis, within an angular range (degrees).
class RevoluteJoint : public Joint {
public:
    static constexpr detail::JointKind kKind = detail::JointKind::REVOLUTE;

    RevoluteJoint(const std::string& label, Shape& to_part, const Axis& axis = Axis::Z,
                  std::optional<Vector> angle_reference = std::nullopt,
                  std::pair<double, double> angular_range = {0.0, 360.0});

    Axis relative_axis() const;
    void set_relative_axis(const Axis& axis) const;
    Vector angle_reference() const;
    std::pair<double, double> angular_range() const;
    void set_angular_range(std::pair<double, double> range) const;
    /// The angle of the last connection (build123d's `_angle`).
    std::optional<double> angle() const;

    void connect_to(const RigidJoint& other, std::optional<double> angle = std::nullopt) const;
    using Joint::connect_to;
    Location relative_to(const RigidJoint& other, std::optional<double> angle = std::nullopt) const;
    using Joint::relative_to;

private:
    friend class Joint;
    explicit RevoluteJoint(std::shared_ptr<detail::JointState> state) : Joint(std::move(state)) {}
};

/// A slider: translation along an axis, within a linear range.
class LinearJoint : public Joint {
public:
    static constexpr detail::JointKind kKind = detail::JointKind::LINEAR;

    LinearJoint(const std::string& label, Shape& to_part, const Axis& axis = Axis::Z,
                std::pair<double, double> linear_range = {0.0, std::numeric_limits<double>::infinity()});

    Axis axis() const;
    Axis relative_axis() const;
    void set_relative_axis(const Axis& axis) const;
    std::pair<double, double> linear_range() const;
    void set_linear_range(std::pair<double, double> range) const;
    /// Position and angle of the last connection.
    std::optional<double> position() const;
    std::optional<double> angle() const;

    void connect_to(const RigidJoint& other, std::optional<double> position = std::nullopt) const;
    void connect_to(const RevoluteJoint& other, std::optional<double> position = std::nullopt,
                    std::optional<double> angle = std::nullopt) const;
    using Joint::connect_to;
    Location relative_to(const RigidJoint& other, std::optional<double> position = std::nullopt) const;
    Location relative_to(const RevoluteJoint& other, std::optional<double> position = std::nullopt,
                         std::optional<double> angle = std::nullopt) const;
    using Joint::relative_to;

private:
    friend class Joint;
    explicit LinearJoint(std::shared_ptr<detail::JointState> state) : Joint(std::move(state)) {}
};

/// Translation along and rotation about an axis, each within a range.
class CylindricalJoint : public Joint {
public:
    static constexpr detail::JointKind kKind = detail::JointKind::CYLINDRICAL;

    CylindricalJoint(const std::string& label, Shape& to_part, const Axis& axis = Axis::Z,
                     std::optional<Vector> angle_reference = std::nullopt,
                     std::pair<double, double> linear_range = {0.0, std::numeric_limits<double>::infinity()},
                     std::pair<double, double> angular_range = {0.0, 360.0});

    Axis axis() const;
    Axis relative_axis() const;
    void set_relative_axis(const Axis& axis) const;
    Vector angle_reference() const;
    std::pair<double, double> linear_range() const;
    void set_linear_range(std::pair<double, double> range) const;
    std::pair<double, double> angular_range() const;
    void set_angular_range(std::pair<double, double> range) const;
    /// Position and angle of the last connection.
    std::optional<double> position() const;
    std::optional<double> angle() const;

    void connect_to(const RigidJoint& other, std::optional<double> position = std::nullopt,
                    std::optional<double> angle = std::nullopt) const;
    using Joint::connect_to;
    Location relative_to(const RigidJoint& other, std::optional<double> position = std::nullopt,
                         std::optional<double> angle = std::nullopt) const;
    using Joint::relative_to;

private:
    friend class Joint;
    explicit CylindricalJoint(std::shared_ptr<detail::JointState> state) : Joint(std::move(state)) {}
};

/// A ball and socket: rotation about three axes, each within a range (degrees).
class BallJoint : public Joint {
public:
    static constexpr detail::JointKind kKind = detail::JointKind::BALL;
    using AngularRange = std::array<std::pair<double, double>, 3>;

    BallJoint(const std::string& label, Shape& to_part, const Location& joint_location = Location(),
              const AngularRange& angular_range = {{{0.0, 360.0}, {0.0, 360.0}, {0.0, 360.0}}},
              const Plane& angle_reference = Plane::XY);

    Location relative_location() const;
    void set_relative_location(const Location& location) const;
    AngularRange angular_range() const;
    void set_angular_range(const AngularRange& range) const;
    Plane angle_reference() const;

    void connect_to(const RigidJoint& other, std::optional<Rotation> angles = std::nullopt) const;
    using Joint::connect_to;
    Location relative_to(const RigidJoint& other, std::optional<Rotation> angles = std::nullopt) const;
    using Joint::relative_to;

private:
    friend class Joint;
    explicit BallJoint(std::shared_ptr<detail::JointState> state) : Joint(std::move(state)) {}
};

template <class J>
bool Joint::is() const {
    return state_ && state_->kind == J::kKind;
}

template <class J>
J Joint::as() const {
    if (!is<J>()) throw std::invalid_argument("The joint '" + label() + "' is of another type");
    return J(state_);
}

template <class J>
J JointMap::get(const std::string& label) const {
    return at(label).as<J>();
}

}  // namespace build123d
