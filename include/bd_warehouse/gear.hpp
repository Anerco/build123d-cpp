// bd_warehouse-cpp: gears (bd_warehouse's gear.py).
//
// Involute spur and helical gears, metric racks, and worm gears (ISO 54 / DIN 3975 worms with
// exact ruled-helicoid flanks, worm wheels, and the meshed pair).
//
//   using namespace bd_warehouse;
//   SpurGear spur(2, 12, 14.5, 5, {.root_fillet = 0.5});   // module 2, 12 teeth, 5 thick
//   HelicalGear helical(2, 13, 20, 45, 10);                 // 45° helix
//   RackGear rack(2, 10, 20, 8);
//   WormGear pair(2, 2, 10, 30, 30, 12);                    // children "worm" and "wheel"
#pragma once

#include <optional>
#include <string>

#include <build123d/build123d.hpp>

namespace bd_warehouse {

using build123d::Align;
using build123d::Align2;
using build123d::Align3;
using build123d::Rotation;

// ---------------------------------------------------------------------------
// Involute gears and racks
// ---------------------------------------------------------------------------

/// The keyword arguments of InvoluteToothProfile.
struct InvoluteToothProfileOptions {
    /// Radius of the fillet at the root of the tooth. Defaults to none.
    std::optional<double> root_fillet = std::nullopt;
    /// The radial distance between the pitch circle and the top of the gear tooth. Defaults to
    /// the module.
    std::optional<double> addendum = std::nullopt;
    /// The radial distance between the pitch circle and the bottom of the gear tooth space.
    /// Defaults to 1.25 module.
    std::optional<double> dedendum = std::nullopt;
    /// Create a closed wire. Defaults to false. (As in bd_warehouse 0.4.0 with this build123d,
    /// true throws: "'Vertex' object has no attribute 'to_tuple'".)
    bool closed = false;
};

/// InvoluteToothProfile: the outline of a single involute tooth (a Wire on XY, the tooth along
/// +X).
///
/// Args:
///     module: the ratio of the pitch diameter to the number of teeth, in millimeters (mm).
///     tooth_count: number of teeth in complete gear.
///     pressure_angle: the angle between the line of action (the line along which the force is
///         transmitted between meshing gear teeth) and the tangent to the pitch circle. Common
///         values are 14.5 or 20.
///     root_fillet, addendum, dedendum, closed: see InvoluteToothProfileOptions.
///
/// Raises:
///     std::invalid_argument: the root fillet fails ("Fillet algorithm failed ...").
class InvoluteToothProfile : public build123d::BaseLineObject {
public:
    InvoluteToothProfile(double module, int tooth_count, double pressure_angle,
                         std::optional<double> root_fillet = std::nullopt, std::optional<double> addendum = std::nullopt,
                         std::optional<double> dedendum = std::nullopt, bool closed = false);
    InvoluteToothProfile(double module, int tooth_count, double pressure_angle,
                         const InvoluteToothProfileOptions& options);

    double module = 0.0;
    int tooth_count = 0;
    /// module * tooth_count / 2.
    double pitch_radius = 0.0;
    /// pitch_radius * cos(pressure_angle).
    double base_radius = 0.0;
    double addendum = 0.0;
    /// pitch_radius + addendum.
    double addendum_radius = 0.0;
    double dedendum = 0.0;
    /// pitch_radius - dedendum.
    double root_radius = 0.0;
};

/// The keyword arguments of RackGearPlan.
struct RackGearPlanOptions {
    /// Signed rack tooth angle in degrees. Defaults to 0.
    double helix_angle = 0;
    /// Measurement plane for module and pressure_angle, "normal" or "transverse". Defaults to
    /// "normal".
    std::string module_system = "normal";
    /// Optional radius of each concave tooth-root fillet. Zero leaves the roots unfilleted.
    std::optional<double> root_fillet = std::nullopt;
    /// Tooth height above the pitch line. Defaults to module.
    std::optional<double> addendum = std::nullopt;
    /// Tooth depth below the pitch line. Defaults to 1.25 * module.
    std::optional<double> dedendum = std::nullopt;
    /// Material below the root line. Defaults to 2 * module.
    std::optional<double> base_height = std::nullopt;
    /// In-plane rotation in degrees. Defaults to 0.
    double rotation = 0;
    /// Sketch alignment. Defaults to Align::NONE (Y=0 at the pitch line).
    std::optional<Align2> align = Align2(Align::NONE);
};

/// Create a finite metric basic-rack profile in the transverse XY plane.
///
/// A rack tooth does not have a finite involute curve. It is the infinite-radius limiting form
/// of an involute gear tooth, so each flank is a straight line at the transverse pressure angle.
/// End boundaries lie on tooth-space centerlines, giving an exact pitch length of
/// tooth_count * pi * transverse_module.
///
/// Args:
///     module: Normal or transverse metric module in millimeters, selected by module_system.
///     tooth_count: Number of complete teeth in the finite rack.
///     pressure_angle: Normal or transverse pressure angle in degrees, selected by module_system.
///     helix_angle .. align: see RackGearPlanOptions.
///
/// Raises:
///     std::invalid_argument: If a dimension, angle, module system, or resulting tooth proportion
///         is invalid.
class RackGearPlan : public build123d::BaseSketchObject {
public:
    RackGearPlan(double module, int tooth_count, double pressure_angle, double helix_angle = 0,
                 const std::string& module_system = "normal", std::optional<double> root_fillet = std::nullopt,
                 std::optional<double> addendum = std::nullopt, std::optional<double> dedendum = std::nullopt,
                 std::optional<double> base_height = std::nullopt, double rotation = 0,
                 std::optional<Align2> align = Align2(Align::NONE));
    RackGearPlan(double module, int tooth_count, double pressure_angle, const RackGearPlanOptions& options);

    double normal_module = 0.0;
    double transverse_module = 0.0;
    double normal_pressure_angle = 0.0;
    double transverse_pressure_angle = 0.0;
    double module = 0.0;
    std::string module_system;
    int tooth_count = 0;
    double pressure_angle = 0.0;
    double helix_angle = 0.0;
    double addendum = 0.0;
    double dedendum = 0.0;
    std::optional<double> root_fillet;
    double base_height = 0.0;
    /// pi * transverse_module.
    double pitch = 0.0;
    /// tooth_count * pitch.
    double pitch_length = 0.0;
    /// addendum + dedendum.
    double tooth_height = 0.0;
};

/// The keyword arguments of RackGear.
struct RackGearOptions {
    /// Signed rack tooth angle in degrees. Defaults to 0.
    double helix_angle = 0;
    /// Measurement plane for module and pressure_angle. Defaults to "normal".
    std::string module_system = "normal";
    /// Optional concave tooth-root fillet radius.
    std::optional<double> root_fillet = std::nullopt;
    /// Tooth height above the pitch line. Defaults to module.
    std::optional<double> addendum = std::nullopt;
    /// Tooth depth below the pitch line. Defaults to 1.25 * module.
    std::optional<double> dedendum = std::nullopt;
    /// Material below the tooth root. Defaults to 2 * module.
    std::optional<double> base_height = std::nullopt;
    /// Object rotation. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Part alignment. Defaults to Align::NONE (Z=0 at the pitch line).
    std::optional<Align3> align = Align3(Align::NONE);
};

/// Create a finite straight or helical metric rack gear: the rack plan extruded (or, with a helix
/// angle, ruled-lofted to its offset copy) through the thickness, standing on XZ.
///
/// Args:
///     module: Normal or transverse metric module in millimeters, selected by module_system.
///     tooth_count: Number of complete rack teeth.
///     pressure_angle: Pressure angle in degrees in the selected measurement plane.
///     thickness: Rack face width along Y in millimeters.
///     helix_angle .. align: see RackGearOptions.
///
/// Raises:
///     std::invalid_argument: If thickness or a rack-plan parameter is invalid.
class RackGear : public build123d::BasePartObject {
public:
    RackGear(double module, int tooth_count, double pressure_angle, double thickness, double helix_angle = 0,
             const std::string& module_system = "normal", std::optional<double> root_fillet = std::nullopt,
             std::optional<double> addendum = std::nullopt, std::optional<double> dedendum = std::nullopt,
             std::optional<double> base_height = std::nullopt, const Rotation& rotation = Rotation(),
             std::optional<Align3> align = Align3(Align::NONE));
    RackGear(double module, int tooth_count, double pressure_angle, double thickness, const RackGearOptions& options);

    double module = 0.0;
    std::string module_system;
    int tooth_count = 0;
    double pressure_angle = 0.0;
    double helix_angle = 0.0;
    double normal_module = 0.0;
    double transverse_module = 0.0;
    double normal_pressure_angle = 0.0;
    double transverse_pressure_angle = 0.0;
    double addendum = 0.0;
    double dedendum = 0.0;
    std::optional<double> root_fillet;
    double base_height = 0.0;
    double pitch = 0.0;
    double pitch_length = 0.0;
    double tooth_height = 0.0;
    double thickness = 0.0;
    /// thickness * tan(helix_angle): the shift of the far face's teeth.
    double lateral_offset = 0.0;
};

/// The keyword arguments of SpurGearPlan.
struct SpurGearPlanOptions {
    /// Radius of the fillet at the root of the tooth. Defaults to none.
    std::optional<double> root_fillet = std::nullopt;
    /// Defaults to the module.
    std::optional<double> addendum = std::nullopt;
    /// Defaults to 1.25 module.
    std::optional<double> dedendum = std::nullopt;
    /// In-plane rotation in degrees. Defaults to 0.
    double rotation = 0;
    /// Alignment. Defaults to (Align::CENTER, Align::CENTER).
    std::optional<Align2> align = Align2(Align::CENTER, Align::CENTER);
};

/// SpurGearPlan: the 2D plan of the gear (the involute teeth around the gear, as a Face).
///
/// Args: module, tooth_count, pressure_angle (see InvoluteToothProfile), and root_fillet,
/// addendum, dedendum, rotation, align (see SpurGearPlanOptions).
class SpurGearPlan : public build123d::BaseSketchObject {
public:
    SpurGearPlan(double module, int tooth_count, double pressure_angle, std::optional<double> root_fillet = std::nullopt,
                 std::optional<double> addendum = std::nullopt, std::optional<double> dedendum = std::nullopt,
                 double rotation = 0, std::optional<Align2> align = Align2(Align::CENTER, Align::CENTER));
    SpurGearPlan(double module, int tooth_count, double pressure_angle, const SpurGearPlanOptions& options);

    double pitch_radius = 0.0;
    double base_radius = 0.0;
    double addendum_radius = 0.0;
    double root_radius = 0.0;
};

/// The keyword arguments of SpurGear.
struct SpurGearOptions {
    /// Radius of the fillet at the root of the tooth. Defaults to none.
    std::optional<double> root_fillet = std::nullopt;
    /// Defaults to the module.
    std::optional<double> addendum = std::nullopt;
    /// Defaults to 1.25 module.
    std::optional<double> dedendum = std::nullopt;
    /// Object rotation. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Alignment. Defaults to Align::CENTER.
    std::optional<Align3> align = Align3(Align::CENTER);
};

/// SpurGear: the 3D representation of the gear (its plan extruded by the thickness), alloy steel
/// with a black oxide finish.
///
/// Args: module, tooth_count, pressure_angle (see InvoluteToothProfile), thickness (gear
/// thickness), and root_fillet, addendum, dedendum, rotation, align (see SpurGearOptions).
class SpurGear : public build123d::BasePartObject {
public:
    SpurGear(double module, int tooth_count, double pressure_angle, double thickness,
             std::optional<double> root_fillet = std::nullopt, std::optional<double> addendum = std::nullopt,
             std::optional<double> dedendum = std::nullopt, const Rotation& rotation = Rotation(),
             std::optional<Align3> align = Align3(Align::CENTER));
    SpurGear(double module, int tooth_count, double pressure_angle, double thickness, const SpurGearOptions& options);

    double pitch_radius = 0.0;
    double base_radius = 0.0;
    double addendum_radius = 0.0;
    double root_radius = 0.0;
};

/// The keyword arguments of HelicalGear.
struct HelicalGearOptions {
    /// Measurement plane for both module and pressure_angle: "normal" is perpendicular to the
    /// tooth helix; "transverse" is perpendicular to the gear axis. Defaults to "normal".
    std::string module_system = "normal";
    /// Radius of the tooth-root fillet. Defaults to none.
    std::optional<double> root_fillet = std::nullopt;
    /// Radial addendum. When omitted, the module in the selected reference system is used.
    std::optional<double> addendum = std::nullopt;
    /// Radial dedendum. When omitted, 1.25 times the module in the selected reference system.
    std::optional<double> dedendum = std::nullopt;
    /// Object rotation. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Object alignment. Defaults to Align::CENTER.
    std::optional<Align3> align = Align3(Align::CENTER);
};

/// A cylindrical involute gear with helical teeth.
///
/// Helical-gear module and pressure angle can be specified in either of two measurement planes:
/// "normal" means the plane perpendicular (normal) to the direction of a tooth helix ("normal" is
/// a geometric term and does not mean ordinary or default); normal-system dimensions correspond to
/// the rack or cutting-tool profile commonly used to manufacture the gear. "transverse" means the
/// plane perpendicular to the gear axis, equivalent to looking directly at the circular end of
/// the gear; the transverse module determines pitch diameter directly: pitch_diameter = module *
/// tooth_count.
///
/// For a helix angle β, the modules are related by transverse_module = normal_module / cos(β).
/// The pressure angle is also converted between the selected normal or transverse plane.
/// Catalogue gears may use either convention, so module_system should match the convention used
/// by the source dimensions. A positive helix angle produces a positive rotation about the Z axis
/// through the gear thickness; a negative angle produces the opposite hand.
///
/// Args:
///     module: normal or transverse module, as selected by module_system.
///     tooth_count: number of teeth.
///     pressure_angle: normal or transverse pressure angle, as selected by module_system.
///     helix_angle: signed helix angle in degrees, in (-90, 90).
///     thickness: gear face width.
///     module_system .. align: see HelicalGearOptions.
///
/// Raises:
///     std::invalid_argument: an invalid module, tooth count, thickness, angle or module system.
class HelicalGear : public build123d::BasePartObject {
public:
    HelicalGear(double module, int tooth_count, double pressure_angle, double helix_angle, double thickness,
                const std::string& module_system = "normal", std::optional<double> root_fillet = std::nullopt,
                std::optional<double> addendum = std::nullopt, std::optional<double> dedendum = std::nullopt,
                const Rotation& rotation = Rotation(), std::optional<Align3> align = Align3(Align::CENTER));
    HelicalGear(double module, int tooth_count, double pressure_angle, double helix_angle, double thickness,
                const HelicalGearOptions& options);

    double normal_module = 0.0;
    double transverse_module = 0.0;
    double normal_pressure_angle = 0.0;
    double transverse_pressure_angle = 0.0;
    double module = 0.0;
    std::string module_system;
    int tooth_count = 0;
    double pressure_angle = 0.0;
    double helix_angle = 0.0;
    double thickness = 0.0;
    double pitch_radius = 0.0;
    double base_radius = 0.0;
    double addendum_radius = 0.0;
    double root_radius = 0.0;
    /// The rotation (degrees) of the teeth through the thickness.
    double twist_angle = 0.0;
    /// 2 pi pitch_radius / |tan(helix_angle)|; infinity for a straight gear.
    double lead = 0.0;

protected:
    HelicalGear() = default;
    void init(double module, int tooth_count, double pressure_angle, double helix_angle, double thickness,
              const HelicalGearOptions& options);
};

// ---------------------------------------------------------------------------
// Worm gears
// ---------------------------------------------------------------------------

/// The derived dimensions of a cylindrical worm (ISO 54 / DIN 3975 nomenclature) that WormFlank
/// and Worm carry (bd_warehouse copies them from its private _WormGeometry).
struct WormDimensions {
    /// Axial module.
    double module = 0.0;
    /// Number of threads.
    int starts = 0;
    /// q = reference diameter / module.
    double diametral_quotient = 0.0;
    /// "ZI" or "ZA".
    std::string flank_form;
    double normal_pressure_angle = 0.0;
    double axial_pressure_angle = 0.0;
    /// ZI only (nullopt for ZA).
    std::optional<double> transverse_pressure_angle;
    double addendum = 0.0;
    double dedendum = 0.0;
    double pitch_radius = 0.0;
    double addendum_radius = 0.0;
    double root_radius = 0.0;
    /// ZI only (nullopt for ZA).
    std::optional<double> base_radius;
    /// pi * module.
    double axial_pitch = 0.0;
    /// pi * module * starts.
    double lead = 0.0;
    /// atan(starts / diametral_quotient) in degrees.
    double lead_angle = 0.0;
    /// ZI only (nullopt for ZA).
    std::optional<double> base_lead_angle;
};

/// The keyword arguments of WormFlank.
struct WormFlankOptions {
    /// Normal pressure angle in degrees. Defaults to 20.
    double pressure_angle = 20;
    /// "ZI" involute helicoid or "ZA" Archimedean helicoid. Defaults to "ZI".
    std::string flank_form = "ZI";
    /// Thread hand, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// "lower" is the flank facing -Z, "upper" the flank facing +Z. Defaults to "lower".
    std::string side = "lower";
    /// Radial addendum. Defaults to module.
    std::optional<double> addendum = std::nullopt;
    /// Radial dedendum. Defaults to 1.2 * module (DIN 3975).
    std::optional<double> dedendum = std::nullopt;
};

/// The working flank surface of one worm thread.
///
/// Both supported flank forms are exact ruled surfaces between two helices of the worm's lead, so
/// the face is an OCCT ruled surface rather than a swept or lofted approximation. The tooth is
/// centred on the +X axis at z = 0 and the face spans the axial length [-length/2, length/2].
///
/// Args:
///     module: Axial module in millimeters (equal to the worm wheel's transverse module).
///     starts: Number of threads.
///     diametral_quotient: q = reference diameter / module (ISO 54). The lead angle follows from
///         tan(γ) = starts / q.
///     length: Axial length of the flank.
///     pressure_angle .. dedendum: see WormFlankOptions.
///
/// Raises:
///     std::invalid_argument: If a parameter is invalid or the tooth proportions are impossible.
///
/// Note: A ZI flank exists only outside the base cylinder. When the root radius is smaller than
/// the base radius the face starts at the base radius; Worm closes the tooth below it with a
/// radial helicoid.
class WormFlank : public build123d::Face, public WormDimensions {
public:
    WormFlank(double module, int starts, double diametral_quotient, double length, double pressure_angle = 20,
              const std::string& flank_form = "ZI", const std::string& hand = "right",
              const std::string& side = "lower", std::optional<double> addendum = std::nullopt,
              std::optional<double> dedendum = std::nullopt);
    WormFlank(double module, int starts, double diametral_quotient, double length, const WormFlankOptions& options);

    /// The length.
    double axial_length = 0.0;
    std::string hand;
    std::string side;
    /// The radius the flank starts at (the root radius, or the base radius of a ZI flank).
    double inner_radius = 0.0;
};

/// The keyword arguments of Worm.
struct WormOptions {
    /// Normal pressure angle in degrees. Defaults to 20.
    double pressure_angle = 20;
    /// "ZI" involute helicoid (conjugate to an involute worm wheel) or "ZA" Archimedean helicoid.
    /// Defaults to "ZI".
    std::string flank_form = "ZI";
    /// Thread hand. Defaults to "right".
    std::string hand = "right";
    /// Radial addendum. Defaults to module.
    std::optional<double> addendum = std::nullopt;
    /// Radial dedendum. Defaults to 1.2 * module (DIN 3975).
    std::optional<double> dedendum = std::nullopt;
    /// Object rotation. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Part alignment. Defaults to Align::CENTER.
    std::optional<Align3> align = Align3(Align::CENTER);
};

/// A cylindrical worm with ISO 54 / DIN 3975 proportions.
///
/// The worm is built as a single exact boundary representation: ruled-helicoid flanks (see
/// WormFlank), cylindrical tip and root surfaces, and planar ends - no sweeps or boolean
/// operations are involved, so construction takes milliseconds and the transverse section matches
/// the analytic tooth profile. The axis is Z, the part is centred on the origin, and thread 0 is
/// centred on the +X axis at z = 0. Alloy steel with a black oxide finish.
///
/// Args:
///     module: Axial module in millimeters (equal to the worm wheel's transverse module).
///     starts: Number of threads.
///     diametral_quotient: q = reference diameter / module (ISO 54); typical values are 8 to 12.
///         The lead angle follows from tan(γ) = starts / q.
///     length: Axial length of the worm.
///     pressure_angle .. align: see WormOptions.
///
/// Raises:
///     std::invalid_argument: If a parameter is invalid or the tooth proportions are impossible.
///     std::runtime_error: worm surfaces did not sew into a closed shell.
class Worm : public build123d::BasePartObject, public WormDimensions {
public:
    Worm(double module, int starts, double diametral_quotient, double length, double pressure_angle = 20,
         const std::string& flank_form = "ZI", const std::string& hand = "right",
         std::optional<double> addendum = std::nullopt, std::optional<double> dedendum = std::nullopt,
         const Rotation& rotation = Rotation(), std::optional<Align3> align = Align3(Align::CENTER));
    Worm(double module, int starts, double diametral_quotient, double length, const WormOptions& options);

    double length = 0.0;
    std::string hand;
};

/// The keyword arguments of WormWheel.
struct WormWheelOptions {
    /// Number of worm threads. Defaults to 1.
    int starts = 1;
    /// Worm q = reference diameter / module. Defaults to 10.
    double diametral_quotient = 10;
    /// Normal pressure angle in degrees. Defaults to 20.
    double pressure_angle = 20;
    /// Hand of the worm and wheel. Defaults to "right".
    std::string hand = "right";
    /// Optional radius of the tooth-root fillet.
    std::optional<double> root_fillet = std::nullopt;
    /// Radial addendum. Defaults to module.
    std::optional<double> addendum = std::nullopt;
    /// Radial dedendum. Defaults to 1.2 * module (DIN 3975).
    std::optional<double> dedendum = std::nullopt;
    /// Object rotation. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Part alignment. Defaults to Align::CENTER.
    std::optional<Align3> align = Align3(Align::CENTER);
};

/// A worm wheel: an involute helical gear matched to a Worm (phosphor bronze).
///
/// For a 90° shaft angle the wheel's transverse module equals the worm's axial module, its helix
/// angle equals the worm's lead angle (same hand), and its transverse pressure angle equals the
/// worm's axial pressure angle. This is a plain (non-throated) wheel, exactly conjugate to a "ZI"
/// worm.
///
/// Args:
///     module: Worm axial module = wheel transverse module in millimeters.
///     tooth_count: Number of wheel teeth.
///     thickness: Wheel face width.
///     starts .. align: see WormWheelOptions.
class WormWheel : public HelicalGear {
public:
    WormWheel(double module, int tooth_count, double thickness, int starts = 1, double diametral_quotient = 10,
              double pressure_angle = 20, const std::string& hand = "right",
              std::optional<double> root_fillet = std::nullopt, std::optional<double> addendum = std::nullopt,
              std::optional<double> dedendum = std::nullopt, const Rotation& rotation = Rotation(),
              std::optional<Align3> align = Align3(Align::CENTER));
    WormWheel(double module, int tooth_count, double thickness, const WormWheelOptions& options);

    int starts = 1;
    double diametral_quotient = 0.0;
    std::string hand;
    /// The worm's lead angle (degrees).
    double lead_angle = 0.0;
    /// The worm's axial pressure angle (degrees).
    double axial_pressure_angle = 0.0;
    /// module * (diametral_quotient + tooth_count) / 2.
    double center_distance = 0.0;
};

/// The keyword arguments of WormGear.
struct WormGearOptions {
    /// Normal pressure angle in degrees. Defaults to 20.
    double pressure_angle = 20;
    /// Worm flank form. Defaults to "ZI".
    std::string flank_form = "ZI";
    /// Hand of the worm and wheel. Defaults to "right".
    std::string hand = "right";
    /// Optional wheel tooth-root fillet radius.
    std::optional<double> root_fillet = std::nullopt;
};

/// Assembly: a Worm meshed with a WormWheel at a 90° shaft angle.
///
/// The wheel is centred on the origin with its axis along Z; the worm axis is parallel to Y
/// through (center_distance, 0, 0) with a thread space facing the wheel tooth on the +X axis.
/// The children are "worm" and "wheel"; RigidJoints "wheel" and "worm" mark the two axes.
///
/// Args:
///     module: Worm axial module = wheel transverse module in millimeters.
///     starts: Number of worm threads.
///     diametral_quotient: Worm q = reference diameter / module.
///     tooth_count: Number of wheel teeth.
///     worm_length: Axial length of the worm.
///     wheel_thickness: Wheel face width.
///     pressure_angle .. root_fillet: see WormGearOptions.
class WormGear : public build123d::Compound {
public:
    WormGear(double module, int starts, double diametral_quotient, int tooth_count, double worm_length,
             double wheel_thickness, double pressure_angle = 20, const std::string& flank_form = "ZI",
             const std::string& hand = "right", std::optional<double> root_fillet = std::nullopt);
    WormGear(double module, int starts, double diametral_quotient, int tooth_count, double worm_length,
             double wheel_thickness, const WormGearOptions& options);

    /// The wheel's center distance.
    double center_distance = 0.0;
    /// tooth_count / starts.
    double ratio = 0.0;
};

}  // namespace bd_warehouse
