// bd_warehouse-cpp: chain sprockets (bd_warehouse's sprocket.py).
//
//   using namespace bd_warehouse;
//   Sprocket sprocket(32);                       // a bicycle chain sprocket (1/2" pitch)
//   // sprocket.pitch_radius() is 64.78458745735234
//   Sprocket drive(20, 0.5 * IN, 5.0 / 16 * IN, 0, 0.084 * IN, 60, 4, 5, 25);  // with bolt holes and bore
#pragma once

#include <optional>

#include <build123d/build123d.hpp>

namespace bd_warehouse {

using build123d::Align;
using build123d::Align3;
using build123d::Face;
using build123d::Rotation;
using build123d::Wire;

/// The keyword arguments of Sprocket (Python's order and defaults).
struct SprocketOptions {
    /// Distance between the centers of two adjacent rollers. Defaults to 1/2 inch.
    double chain_pitch = 0.5 * build123d::IN;
    /// Size of the cylindrical rollers within the chain. Defaults to 5/16 inch.
    double roller_diameter = (5.0 / 16) * build123d::IN;
    /// Size of the gap between the chain's rollers and the sprocket's teeth. Defaults to 0.
    double clearance = 0.0;
    /// Thickness of the sprocket. Defaults to 0.084 inch.
    double thickness = 0.084 * build123d::IN;
    /// Diameter of the mounting bolt hole pattern. Defaults to 0.
    double bolt_circle_diameter = 0.0;
    /// Number of bolt holes - if 0, no bolt holes are added to the sprocket. Defaults to 0.
    int num_mount_bolts = 0;
    /// Size of the bolt holes used to mount the sprocket. Defaults to 0.
    double mount_bolt_diameter = 0.0;
    /// Size of the central hole in the sprocket - if 0, no bore hole is added. Defaults to 0.
    double bore_diameter = 0.0;
    /// Angles to rotate about axes. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Align MIN, CENTER, or MAX of object. Defaults to Align::CENTER.
    std::optional<Align3> align = Align3(Align::CENTER);
};

/// Create a new sprocket object as defined by the given parameters. The input parameter defaults
/// are appropriate for a standard bicycle chain.
///
/// Args:
///     num_teeth: number of teeth on the perimeter of the sprocket (greater than 2).
///     chain_pitch .. align: see SprocketOptions.
///
/// Attributes:
///     pitch_radius(): radius of the circle formed by the center of the chain rollers.
///     outer_radius(): size of the sprocket from center to tip of the teeth.
///     pitch_circumference(): circumference of the sprocket at the pitch radius.
///     plan(): 2D plan of the base sprocket without any cutouts.
///
/// Raises:
///     std::invalid_argument: roller_diameter is too large for chain_pitch; num_teeth must be an
///         integer greater than 2.
///
/// Example: Sprocket(32).pitch_radius() is 64.78458745735234.
class Sprocket : public build123d::BasePartObject {
public:
    /// Calculate and return the pitch radius of a sprocket with the given number of teeth and
    /// chain pitch (the distance between two adjacent pins in a single link).
    static double sprocket_pitch_radius(int num_teeth, double chain_pitch);
    /// Calculate and return the pitch circumference of a sprocket with the given number of teeth
    /// and chain pitch.
    static double sprocket_circumference(int num_teeth, double chain_pitch);

    explicit Sprocket(int num_teeth, double chain_pitch = 0.5 * build123d::IN,
                      double roller_diameter = (5.0 / 16) * build123d::IN, double clearance = 0.0,
                      double thickness = 0.084 * build123d::IN, double bolt_circle_diameter = 0.0,
                      int num_mount_bolts = 0, double mount_bolt_diameter = 0.0, double bore_diameter = 0.0,
                      const Rotation& rotation = Rotation(), std::optional<Align3> align = Align3(Align::CENTER));
    Sprocket(int num_teeth, const SprocketOptions& options);

    /// The radius of the circle formed by the center of the chain rollers.
    double pitch_radius() const;
    /// The size of the sprocket from center to tip of the teeth.
    double outer_radius() const;
    /// The circumference of the sprocket at the pitch radius.
    double pitch_circumference() const;
    /// 2D plan of the base sprocket without any cutouts.
    Face plan() const;

    int num_teeth = 0;
    double chain_pitch = 0.0;
    double roller_diameter = 0.0;
    double clearance = 0.0;
    double thickness = 0.0;
    double bolt_circle_diameter = 0.0;
    int num_mount_bolts = 0;
    double mount_bolt_diameter = 0.0;
    double bore_diameter = 0.0;

private:
    /// The outline of one tooth (from roller to roller).
    static Wire _make_tooth_outline(int num_teeth, double chain_pitch, double roller_diameter, double clearance = 0.0);
    build123d::Shape _make_sprocket();
    bool flat_teeth_ = false;
};

}  // namespace bd_warehouse
