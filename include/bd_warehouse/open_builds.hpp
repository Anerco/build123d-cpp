// bd_warehouse-cpp: OpenBuilds parts (bd_warehouse's open_builds.py).
//
// Parameterized models of parts from https://openbuildspartstore.com/: rails (C-Beam and V-Slot
// linear rails and their profiles), plates, spacers, washers, wheels, couplers, lead screws,
// stepper motors, and the assemblies built of them (C-Beam actuators, gantries, wheel
// assemblies). Parts carry their material and joints; assemblies are Compounds whose children
// are positioned by connecting their joints.
//
//   using namespace bd_warehouse;
//   CBeamLinearRail rail(500);                         // joints "screw_axis", "bottom_center", ...
//   XtremeSolidVWheelAssembly wheel(true);             // RevoluteJoint "mount"
//   CBeamAssembly actuator(300, 100);                  // with an XLarge gantry at 100 mm
//   VSlotLinearRail vslot("20x40", 250);
//
// bd_warehouse makes these parts in its builder mode; the port builds the same geometry in
// algebra mode, so the `mode` arguments (how the object combines with an enclosing builder)
// aren't ported.
#pragma once

#include <optional>
#include <string>

#include <build123d/build123d.hpp>

#include "bd_warehouse/fastener.hpp"

namespace bd_warehouse {

using build123d::Align;
using build123d::Align2;
using build123d::Align3;
using build123d::Rotation;

/// open_builds.py's module values (bd_warehouse.open_builds.CAVITY_RADIUS, ...).
namespace open_builds {
/// The radius of the round cavities of the rail profiles: 2.3 mm.
inline constexpr double CAVITY_RADIUS = 2.3;
/// The radius of the rounded corners of the rail profiles: 1.5 mm.
inline constexpr double FILLET_RADIUS = 1.5;
/// The screw the parts' M5 threaded holes are made for: SocketHeadCapScrew("M5-0.8", 20).
///
/// bd_warehouse appends the location of every threaded hole made with it to its
/// hole_locations (in builder mode, every time a part is made); the shared screw here is left
/// unchanged.
const SocketHeadCapScrew& m5();
}  // namespace open_builds

/// Part Object: OpenBuilds 8mm Acme Anti Backlash Nut Block.
///
/// This Anti-Backlash Nut Block for 8mm Lead Screw is a great choice for many build projects
/// requiring lead screw linear motion where high precision and repeatability with zero play or
/// slop is needed.
///
/// Product Features: screw and nut for secure and precise backlash adjustment; recesses screw
/// holes for non obtrusive placement; can be mounted on plates or V-Slot; works best with
/// OpenBuilds Lead Screw.
///
/// Specifications: Tr8*8(p2) Metric Acme Tap (compatible with the customized 8mm Metric Acme
/// Lead Screws); mounting hole spacing 20mm; pitch 2mm; lead 8mm; Delrin; color black.
///
/// Joints: RigidJoints "a" and "b" (the mounting faces), "nut_a" and "nut_b" (the nut pockets),
/// "screw" (the threaded hole) and "bottom".
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt (bd_warehouse's
///         docstring says (CENTER, CENTER, MIN); its code uses None).
class AcmeAntiBacklashNutBlock8mm : public build123d::BasePartObject {
public:
    explicit AcmeAntiBacklashNutBlock8mm(const Rotation& rotation = Rotation(),
                                         std::optional<Align3> align = std::nullopt);
};

/// Assembly: OpenBuilds 8mm Acme Anti Backlash Nut Block Assembly.
///
/// All of the components in anti backlash nut assembly: AcmeAntiBacklashNutBlock8mm x 1,
/// AluminumSpacer x 2, M5 HexNut x 2, SocketHeadCapScrew x 1.
///
/// The RigidJoint "a" or "b" - positioned at the top of the spaces - can be used to connect this
/// assembly to a gantry plate.
class AcmeAntiBacklashNutBlock8mmAssembly : public build123d::Compound {
public:
    AcmeAntiBacklashNutBlock8mmAssembly();
};

/// Part Object: OpenBuilds AluminumSpacer.
///
/// Aluminum spacers used throughout the system to ensure precision alignment and spacing
/// between components. Rigid to maintain stability during motion; lightweight, non-magnetic and
/// corrosion resistant. M5 ID, 10mm OD, aluminum, silver.
///
/// Joints: "a" and "b" are the bottom and top mounting faces; "center" is the top-face
/// mounting/rotation axis used by wheel assemblies (not the geometric midpoint).
///
/// Args:
///     length: one of "3mm", "1/8in", "6mm", "1/4in", "9mm", "10mm", "13.2mm", "20mm", "35mm",
///         "1-1/2in" or "40mm".
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: Invalid length.
class AluminumSpacer : public build123d::BasePartObject {
public:
    explicit AluminumSpacer(const std::string& length, const Rotation& rotation = Rotation(),
                            std::optional<Align3> align = std::nullopt);
};

/// The keyword arguments of CBeamAssembly (Python's order and defaults).
struct CBeamAssemblyOptions {
    /// Position of the gantry along the rail; nullopt for no gantry. Defaults to nullopt.
    std::optional<double> gantry_position = std::nullopt;
    /// Lead screw length. If omitted, the length is calculated from the end assemblies when
    /// possible (else the rail length). Defaults to nullopt.
    std::optional<double> screw_length = std::nullopt;
    /// Include the additional C-Beam end plate and bearing. Defaults to true.
    bool end_plate = true;
    /// Use a simplified lead screw representation. Defaults to true.
    bool simple = true;
};

/// Assembly: OpenBuilds C-Beam Linear Actuator Assembly.
///
/// Complete C-Beam linear actuator assembly consisting of a C-Beam rail, lead screw, motorized
/// end assembly, and optional end plate and XLarge C-Beam gantry.
///
/// Product Features: combines linear motion and structural framing components; supports an
/// optional adjustable gantry position; provides a linear joint for positioning along the lead
/// screw axis; includes mounting joints for the rear, front, and cross-front rail faces.
///
/// Product Specifications: C-Beam Linear Rail; Metric Tr8*8-2p lead screw; optional XLarge
/// C-Beam Gantry; optional C-Beam End Mount and bearing.
///
/// Joints: RigidJoints "gantry_center" (with a gantry), "rail_back", "rail_front",
/// "rail_cross_front"; LinearJoint "screw_axis".
///
/// Args:
///     rail_length: length of the C-Beam rail.
///     gantry_position .. simple: see CBeamAssemblyOptions.
class CBeamAssembly : public build123d::Compound {
public:
    explicit CBeamAssembly(double rail_length, std::optional<double> gantry_position = std::nullopt,
                           std::optional<double> screw_length = std::nullopt, bool end_plate = true,
                           bool simple = true);
    CBeamAssembly(double rail_length, const CBeamAssemblyOptions& options);
};

/// Assembly: OpenBuilds C-Beam Capped Linear Rail.
///
/// C-Beam Linear Rail assembly with a C-Beam End Mount and bearing at each end. The end mounts
/// are secured with M5 low-profile screws and provide end support for a separately modeled lead
/// screw.
///
/// Product Features: end-mounted bearings for lead screw support; capped ends for a compact
/// C-Beam assembly; rigid joints for the top and bottom rail centers.
///
/// Product Specifications: C-Beam Linear Rail; two C-Beam End Mounts; two M8-16-5 capped
/// deep-groove ball bearings; eight M5 low-profile screws. This assembly does not include the
/// lead screw itself.
///
/// Joints: RigidJoints "bottom_center" and "top_center".
///
/// Args:
///     length: length of the C-Beam rail.
class CBeamCapped : public build123d::Compound {
public:
    explicit CBeamCapped(double length);
};

/// Assembly: OpenBuilds C-Beam End Assembly.
///
/// Motorized end assembly for driving an OpenBuilds C-Beam lead screw. The assembly combines a
/// C-Beam End Mount, NEMA 23 stepper motor, flexible coupler, bearing, mounting spacers, and the
/// required fasteners.
///
/// Product Features: supports and drives a C-Beam lead screw; includes motor mounting spacers
/// for alignment; provides a rigid joint ("screw") at the coupler's lead screw connection.
///
/// Product Specifications: NEMA 23 stepper motor; 8mm flexible coupler; M8-16-5 capped
/// deep-groove ball bearing; two 40mm aluminum spacers; four M5 low-profile screws.
class CBeamEndAssembly : public build123d::Compound {
public:
    CBeamEndAssembly();
};

/// CBeamEndMount.
///
/// Ensure seamless integration of your motor with the C-Beam Linear Rail using the versatile
/// C-Beam End Mount. Designed with pre-threaded holes and a bearing recess for lead screw
/// transmission, this mount also features additional countersunk holes for flush mounting.
///
/// Product Features: pre-tapped holes; countersunk holes; bearing recess for lead screw
/// transmission.
///
/// Joints: RigidJoints "motor_mount-0", "motor_mount-1", "screw-0" .. "screw-3" and "bearing".
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to (CENTER, CENTER, MIN).
class CBeamEndMount : public build123d::BasePartObject {
public:
    explicit CBeamEndMount(const Rotation& rotation = Rotation(),
                           std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));
};

/// Sketch Object: OpenBuilds C Beam Linear Rail Profile.
///
/// Args:
///     rotation: angle to rotate the profile (degrees). Defaults to 0.
///     align: align min, center, or max of object. Defaults to (CENTER, CENTER).
class CBeamLinearRailProfile : public build123d::BaseSketchObject {
public:
    explicit CBeamLinearRailProfile(double rotation = 0, std::optional<Align2> align = Align2(Align::CENTER, Align::CENTER));
};

/// Part Object: OpenBuilds C-Beam Linear Rail.
///
/// C-Beam Linear Rail aluminum extrusion profile is the ultimate solution combining both linear
/// motion and a modular, structural framing system. It's lightweight yet rigid and provides an
/// ultra smooth track for precise motion. Much like working with lumber, you can cut C-Beam on a
/// chop saw (using a metal blade) or even use a hacksaw; connections are made with a screw
/// driver.
///
/// Product Features: lightweight and strong; tee nut channel; smooth v-groove for linear motion;
/// M5 tap-ready holes; aluminum extrusion; size 80x40.
///
/// Joints: LinearJoint "screw_axis" (along the rail, range (0, length)); RigidJoints
/// "bottom_center" and "top_center".
///
/// Args:
///     length: rail length.
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to (CENTER, CENTER, MIN).
class CBeamLinearRail : public build123d::BasePartObject {
public:
    explicit CBeamLinearRail(double length, const Rotation& rotation = Rotation(),
                             std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));
};

/// Part Object: OpenBuilds C-Beam Gantry Plate.
///
/// Achieve a compact gantry cart footprint with the versatile C-Beam Gantry Plate. This plate
/// accommodates up to 4 Mini V Wheels and can be easily configured to station wheels inside the
/// C-Beam Linear Rail track. The plate features recessed and pre-tapped holes for professional
/// flush mounts.
///
/// Product Features: countersunk holes; pre-tapped holes; center recess; multiple mounting
/// configurations.
///
/// Specifications: 77.5 x 77.5mm, 6mm thick; 6061-T5 aluminum, brushed and anodized black.
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
class CBeamGantryPlate : public build123d::BasePartObject {
public:
    explicit CBeamGantryPlate(const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds C-Beam Gantry Plate X-Large.
///
/// Achieve unparalleled stability in your builds with the XLarge C-Beam Gantry Plate, designed
/// to accommodate up to six wheels for an extremely stable gantry. This versatile plate is
/// essential for both Belt and Pinion and Lead Screw transmissions. With recessed holes for
/// professional flush mounts and tapped holes for a 90° connection, this plate is ideal for XY
/// configurations.
///
/// Product Features: wide stance for enhanced stability; countersunk holes for a flush finish;
/// tapped holes for easy 90° connections; center recess; multiple mounting configurations.
///
/// Specifications: 125 x 125mm, 6mm thick; 6061-T5 aluminum, brushed and anodized black.
///
/// Joints: RigidJoints "a", "b", "c" (the eccentric wheel mounts), "d", "e", "f" (the fixed wheel
/// mounts), "nut_a" .. "nut_d" and "top_center".
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
class CBeamGantryPlateXLarge : public build123d::BasePartObject {
public:
    explicit CBeamGantryPlateXLarge(const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);

    /// The plate thickness: 6 mm.
    double thickness = 6.0;
};

/// Part Object: OpenBuilds C-Beam Riser Plate.
///
/// A key component used to attach C-Beam Gantry Plates to a C-Beam Shield. Pre-threaded holes
/// and center recess allow for flush mounting a top plate.
///
/// Product Features: C-Beam and V-Slot 20x80mm compatible; pre-tapped holes; center recess.
///
/// Specifications: 77.5 x 14.874mm, 8mm thick; 6061-T5 aluminum, brushed and anodized black.
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
class CBeamRiserPlate : public build123d::BasePartObject {
public:
    explicit CBeamRiserPlate(const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds Eccentric Spacer.
///
/// These Eccentric Spacers are the perfect solution for creating a pre-load from the V-Wheels to
/// the V-Slot Linear Rail. The text on the outside shows where the smallest part of the CAM hole
/// is located.
///
/// Product Features: up to 1.5mm off center adjustment; tension wheel to v-slot for a snug,
/// locked on fit; wide stance for stability.
///
/// Specifications: version V7; 6mm or 1/4" CAM height; 5mm bore; rim fits into a 7.12mm hole;
/// aluminum.
///
/// Joints: "a" and "b" are the bottom and top mounting faces; "center" is the eccentric
/// mounting/rotation axis at the top face.
///
/// Args:
///     cam_height: "6mm" or "1/4in". Defaults to "6mm".
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: Invalid cam height.
class EccentricSpacer : public build123d::BasePartObject {
public:
    explicit EccentricSpacer(const std::string& cam_height = "6mm", const Rotation& rotation = Rotation(),
                             std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds Flexible Coupler.
///
/// These Flexible couplings are used to transmit torque from a stepper motor to a lead screw,
/// thus creating the movement in an actuator.
///
/// Specifications: length 25mm (.98"); OD 20mm (.79"); shaft 5mm x 8mm or 1/4"; aluminum; silver.
///
/// Joints: RigidJoints "a" (the motor end) and "b" (the screw end).
///
/// Args:
///     shaft_diameter: load shaft diameter, "8mm" or "1/4in".
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: Invalid shaft diameter.
class FlexibleCoupler : public build123d::BasePartObject {
public:
    explicit FlexibleCoupler(const std::string& shaft_diameter, const Rotation& rotation = Rotation(),
                             std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds LBracket.
///
/// Right-angle mounting bracket for joining OpenBuilds plates, profiles, and other components.
/// The two perpendicular flanges provide elongated mounting slots for positional adjustment
/// during assembly.
///
/// Specifications: 3mm thick brushed aluminum anodize black; 20mm flange width; 14.5mm flange
/// depth; 3.5mm wide slots with 5.5mm center-to-center spacing.
///
/// Joints: RigidJoints "corner", "screw_a", "screw_b", "nut_a" and "nut_b".
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to Align::NONE.
class LBracket : public build123d::BasePartObject {
public:
    explicit LBracket(const Rotation& rotation = Rotation(), std::optional<Align3> align = Align3(Align::NONE));
};

/// Assembly: OpenBuilds LockCollar.
///
/// Lock Collars are used to create mechanical stops at any location along a shaft or to secure
/// components to it. A popular use is to lock the 8mm Lead Screw in place. Includes M5x4 set
/// screw (the children are the "Collar" and the set screw in its threaded hole).
///
/// Product Specifications: 1/4" size ID 0.250 in, OD .500 in, W 7mm; 5mm size ID 5mm, OD 0.5 in,
/// W 7mm; 8mm size ID 8mm, OD 14mm, W 7mm; black oxide finished steel.
///
/// Joints: RigidJoints "a" and "b" (the two faces).
///
/// Args:
///     inside_diameter: inside diameter of the collar, "5mm", "8mm" or "1/4in".
///
/// Raises:
///     std::invalid_argument: Invalid inside diameter.
class LockCollar : public build123d::Compound {
public:
    explicit LockCollar(const std::string& inside_diameter);
};

/// Assembly: OpenBuilds MetricLeadScrew.
///
/// These precision trapezoidal 8mm metric lead screws are a perfect combination of high torque
/// and speed. Although this lead screw is a single object it is created as an Assembly for
/// efficiency reasons (the core cylinder and, unless simple, the thread).
///
/// Product Specifications: Tr8*8-2p (4 starts); pitch 2mm; stainless steel 304; lead screw
/// diameter 7.8mm.
///
/// Joints: RigidJoint "axis" (the bottom of the screw).
///
/// Args:
///     length: screw length.
///     simple: leave out the thread. Defaults to true.
class MetricLeadScrew : public build123d::Compound {
public:
    explicit MetricLeadScrew(double length, bool simple = true);
};

/// RouterSpindleMount.
///
/// Ensure the secure mounting of your router or spindle with this robust and adjustable mount.
/// The thick and hefty design provides a stable base, while the adjustable faceplate
/// accommodates size variations. This mount features pre-tapped holes for easy installation and
/// optional attachments like the OpenBuilds LED Light Ring.
///
/// Product Features: removable faceplate for quick tool changes; adjustable faceplate to
/// accommodate various router sizes; countersunk holes for a flush finish; tapped holes for easy
/// mounting; flexible mounting options.
///
/// Joints: RigidJoints "top_a" .. "top_d", "bottom_a" .. "bottom_d" and "back_mount".
///
/// Args:
///     parts: parts to be created, "base", "faceplate" or "both". Defaults to "both".
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
///
/// Raises:
///     std::runtime_error: bd_warehouse 0.4.0 makes only "both": with "base" it fails with
///         "cannot access local variable 'faceplate' where it is not associated with a value"
///         (Python's UnboundLocalError), with "faceplate" or another value likewise for 'base'.
///     std::invalid_argument: invalid parts parameter (unreachable, as in bd_warehouse).
class RouterSpindleMount : public build123d::BasePartObject {
public:
    explicit RouterSpindleMount(const std::string& parts = "both", const Rotation& rotation = Rotation(),
                                std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds Shim/Washer.
///
/// Product Features: reduce wear and prevents binding; creates a tighter fit among parts.
///
/// Joints: RigidJoints "a" and "b" (the bottom and top faces).
///
/// Args:
///     shim_type: "MiniVWheel", "10x5x1", "12x8x1", "SlotWasher" or "FlatWasher".
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: Invalid shim_type.
class ShimWasher : public build123d::BasePartObject {
public:
    explicit ShimWasher(const std::string& shim_type, const Rotation& rotation = Rotation(),
                        std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds Spacer Block.
///
/// This Spacer Block allows you to raise the V-Slot Gantry Plate up away from the rail 12mm so
/// that you can run transmission components such as a Belt Drive or Threaded Rod underneath the
/// V-Slot Gantry Plate.
///
/// Product Features: adds transmission spacing to any V-Slot Linear Actuators; compatible with
/// multiple OpenBuilds plates.
///
/// Specifications: 86.4 x 20mm, 12mm thick; M5 tapped holes to allow mounting to the V-Slot
/// Gantry Plate; 3 holes that allow for mounting of full size OpenBuilds Wheels; anodized
/// aluminum, black.
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to nullopt.
class SpacerBlock : public build123d::BasePartObject {
public:
    explicit SpacerBlock(const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
};

/// Part Object: OpenBuilds TNut.
///
/// T-nut for attaching components to the OpenBuilds V-Slot linear rail. The nut slides into the
/// rail's tee slot and provides an M5 tapped mounting point.
///
/// Product Features: slides into OpenBuilds V-Slot extrusion; M5 tapped hole for component
/// mounting; chamfered leading edge for easier insertion.
///
/// Specifications: M5 x 0.8 tapped hole; 5.9mm wide; 10mm long; mild steel.
///
/// Joints: RigidJoints "a" and "b".
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to Align::NONE.
class TNut : public build123d::BasePartObject {
public:
    explicit TNut(const Rotation& rotation = Rotation(), std::optional<Align3> align = Align3(Align::NONE));
};

/// Sketch Object: OpenBuilds V Slot Linear Rail Profile.
///
/// Args:
///     rail_size: "20x20", "20x40", "20x60", "20x80" or "40x40". Defaults to "20x20".
///     rotation: angle to rotate the profile (degrees). Defaults to 0.
///     align: align min, center, or max of object. Defaults to (CENTER, CENTER).
///
/// Raises:
///     std::invalid_argument: Invalid rail_size.
class VSlotLinearRailProfile : public build123d::BaseSketchObject {
public:
    explicit VSlotLinearRailProfile(const std::string& rail_size = "20x20", double rotation = 0,
                                    std::optional<Align2> align = Align2(Align::CENTER, Align::CENTER));
};

/// Part Object: OpenBuilds V Slot Linear Rail.
///
/// V-Slot Linear Rail aluminum extrusion profile is the ultimate solution combining both linear
/// motion and a modular, structural framing system. It's lightweight yet rigid and provides an
/// ultra smooth track for precise motion.
///
/// Product Features: lightweight and strong; tee nut channel; smooth v-groove for linear motion;
/// M5 tap-ready holes; aluminum extrusion; sizes 20x20, 20x40, 20x60, 20x80, and 40x40.
///
/// Args:
///     rail_size: size in mm, "20x20", "20x40", "20x60", "20x80" or "40x40".
///     length: rail length.
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to (CENTER, CENTER, MIN).
///
/// Raises:
///     std::invalid_argument: Invalid rail_size.
class VSlotLinearRail : public build123d::BasePartObject {
public:
    VSlotLinearRail(const std::string& rail_size, double length, const Rotation& rotation = Rotation(),
                    std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));
};

/// XLargeCBeamGantry.
///
/// The C-Beam XLarge Gantry Set is ideal for high-performance demands. Tailored to excel in
/// robust Linear Actuator gantries and CNC Machines, it fuses strength, resilience, and
/// unwavering stability. Setting this Gantry kit apart are four robust Xtreme V Wheels, which
/// shine under intense force and weight, delivering heightened accuracy and minimal compression.
///
/// Specifications: plate 125mm x 125mm; compatible with C-Beam Linear Rail; engineered for Lead
/// Screw-driven systems. The children are the plate, the anti-backlash nut assembly and the
/// wheel assemblies.
///
/// Joints: RigidJoints "nut" and "top_center".
///
/// Args:
///     wheel_count: number of wheels, 4 or 6. Defaults to 4.
///     eccentric_angle: angle of the eccentric spacers. Values less than 90 will tighten the
///         wheels to the rail while values greater than 90 will loosen the fit. Defaults to 90.
///
/// Raises:
///     std::invalid_argument: Invalid wheel_count.
class XLargeCBeamGantry : public build123d::Compound {
public:
    explicit XLargeCBeamGantry(int wheel_count = 4, double eccentric_angle = 90);
};

/// Part Object: OpenBuilds Xtreme Solid V Wheel.
///
/// A heavy-duty alternative to Delrin Wheels, suitable where substantial force and weight are
/// introduced into the system: additional accuracy and less compression than Delrin. Flat
/// profile designed to work with a belt on a belt and pinion system as well as lead screw and
/// belt driven systems.
///
/// Product Features: heavy duty; long lasting solid construction; ultra smooth finish for
/// precise motion; flat wheel surface for belt travel; self-centering and self-lubricating; even
/// distribution of weight; easily assembled and maintained.
///
/// Joints: RigidJoints "a" and "b" (the bearing seats).
///
/// Args:
///     rotation: angles to rotate about axes. Defaults to no rotation.
///     align: align min, center, or max of object. Defaults to Align::CENTER.
class XtremeSolidVWheel : public build123d::BasePartObject {
public:
    explicit XtremeSolidVWheel(const Rotation& rotation = Rotation(), std::optional<Align3> align = Align3(Align::CENTER));
};

/// Assembly: OpenBuilds Xtreme Solid V Wheel Assembly.
///
/// All of the components in a Xtreme Solid V Wheel assembly: Xtreme Solid V Wheel x 1, M5-16-5
/// bearing x 2, 10x5x1 Shim x 2, Aluminum Spacer or Eccentric Spacer, Hex Nut (and the screw).
///
/// Note that the RevoluteJoint "mount" should be used to attach this wheel assembly to a gantry
/// plate as it enables the eccentric spacer to precisely align the wheel to the rail by changing
/// the joint angle.
///
/// Args:
///     eccentric: use an eccentric spacer.
class XtremeSolidVWheelAssembly : public build123d::Compound {
public:
    explicit XtremeSolidVWheelAssembly(bool eccentric);
};

/// Assembly: OpenBuilds StepperMotors.
///
/// This custom OpenBuilds NEMA Stepper Motor series offers a range of options, from the NEMA 17
/// for lighter duty, high-precision CNC projects, to the NEMA 23 for larger builds requiring
/// more power, and the NEMA 23 High Torque for enhanced holding power and strength. The children
/// are the motor body and the shaft.
///
/// Common Product Specifications: step angle 1.8; 4 wire bi-polar; RoHS compliant.
/// NEMA 17: shaft 5mm; torque 76 oz*in; peak current 1.68A/phase; M3 mounting holes; 12-24VDC.
/// NEMA 23: shaft 1/4"; torque 175 oz*in; peak current 2.8A/phase; 12-48VDC.
/// NEMA 23 High Torque: 2 phase; shaft 1/4"; torque 345 oz*in; 24-48VDC.
///
/// Joints: RigidJoints "a" .. "d" (the mounting holes) and "shaft".
///
/// Args:
///     motor_type: "Nema17", "Nema23" or "Nema23HighTorque".
///     motor_length: the length of the motor body in the axis of the shaft. If nullopt, uses a
///         standard length for the specified motor type. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: Invalid motor_type; motor_length not greater than 0.
class StepperMotor : public build123d::Compound {
public:
    explicit StepperMotor(const std::string& motor_type, std::optional<double> motor_length = std::nullopt);
};

}  // namespace bd_warehouse
