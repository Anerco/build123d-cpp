// bd_warehouse-cpp: pipe flanges (bd_warehouse's flange.py).
//
// ASME B16.5 class 150 and 300 flanges (the tables bd_warehouse carries): blind, lapped (with its
// stub end), slip-on, socket weld and weld neck flanges with flat, raised or ring joint faces.
// Each flange's axis is Z with the face down on z=0 (the bolt holes through it).
//
//   using namespace bd_warehouse;
//   WeldNeckFlange flange("2", 300);                      // raised face
//   SlipOnFlange slip_on("4", 150, "Flat");
//   // flange.od, flange.thickness; joints "face" and "pipe"
//
// flange.py also defines is_safe and imperial_str_to_float, the same as thread.py's: see
// thread.hpp.
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Align;
using build123d::Align3;
using build123d::Edge;
using build123d::Rotation;
using build123d::Shape;
using build123d::ShapeList;
using build123d::Sketch;

/// An ASME B16.5 table: the values of each nominal pipe size, a number or the table's text (a
/// fraction such as "5/8", or "…" and "Note (8)" where the standard has no value).
using FlangeTable = Dict<std::string, std::vector<Param>>;

/// Nominal pipe size (NPS) to nominal diameter (DN).
const Dict<std::string, int>& nps_to_dn();
/// ASME 16.5 Table 7 Templates for Drilling Class 150 Pipe Flanges and Flanged Fittings, by NPS:
/// O, W (bolt circle), bolt hole diameter, number of bolts, bolt diameter, bolt lengths.
const FlangeTable& drilling_class_150();
/// ASME 16.5 Table 9 Templates for Drilling Class 300 Pipe Flanges and Flanged Fittings.
const FlangeTable& drilling_class_300();
/// ASME 16.5 Table 8 Dimensions of Class 150 Flanges.
const FlangeTable& flange_data_class_150();
/// ASME 16.5 Table 10 Dimensions of Class 300 Flanges.
const FlangeTable& flange_data_class_300();
/// Ring joint facings, class 150: groove number, pitch diameter P, depth E, width F, radius at
/// bottom R, diameter of the raised portion K, distance between flanges.
const FlangeTable& ring_joint_facings_class_150();
/// Ring joint facings, class 300.
const FlangeTable& ring_joint_facings_class_300();

/// The nominal pipe sizes of the flanges (flange.py's Literal Nps): "1/2", "3/4", ... "24".
const std::vector<std::string>& flange_nps();
/// The face types (Literal FaceType): "Flat", "Raised", "Ring", "Tongue", "Groove", "Male",
/// "Female" (bd_warehouse makes Flat, Raised and Ring faces).
const std::vector<std::string>& flange_face_types();
/// The flange classes (Literal FlangeClass): 150, 300, 400, 600, 900, 1500, 2500 (bd_warehouse
/// has the tables of 150 and 300).
const std::vector<int>& flange_classes();

/// Flange: the base class for all the derived flange classes.
///
/// The 2D cross section (on Plane.XZ, its x >= 0 half used) is revolved about Z and the bolt
/// holes are drilled through it on the bolt circle.
///
/// Args:
///     flange_section: 2D cross section (a Face or Sketch on Plane.XZ).
///     bcd: bolt center diameter.
///     bolt_hole_count: number of bolt holes.
///     bolt_hole_diameter: bolt hole size.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
class Flange : public build123d::BasePartObject {
public:
    Flange(const Shape& flange_section, double bcd, int bolt_hole_count, double bolt_hole_diameter,
           const Rotation& rotation = Rotation(),
           std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    /// Validate common flange inputs; true, or std::invalid_argument for an invalid nps,
    /// flange_class or face_type (nullopt is accepted for face_type).
    static bool inputs_are_valid(const std::string& nps, int flange_class, const std::optional<std::string>& face_type);
    /// Check if a flange table dimension is available as a number.
    static bool dimension_is_numeric(const Param& value);
    /// std::invalid_argument "<flange_name> requires unavailable ASME table dimensions: ..." if
    /// any of the columns (name: index) is missing or not a number.
    static void require_numeric_dimensions(const std::vector<Param>& flange_data, const Dict<std::string, int>& columns,
                                           const std::string& flange_name);
    /// Return flange and bolt hole data (the class's flange_data and drilling rows of nps).
    /// std::invalid_argument "Unsupported flange class" for classes without tables;
    /// std::out_of_range for an nps the tables don't have. face_type is accepted for API symmetry.
    static std::pair<std::vector<Param>, std::vector<Param>> get_flange_data(
        const std::string& nps, int flange_class, const std::optional<std::string>& face_type);
    /// Return the data required to create the face cross section: the section (nullopt for no
    /// face, a flat face, or a face type that draws nothing) and its height (the face thickness).
    /// face_thickness overrides the table's depth. std::invalid_argument for an unsupported
    /// class or face_type; std::out_of_range for an nps without ring joint data.
    static std::pair<std::optional<Sketch>, double> get_face_section_data(const std::string& nps, int flange_class,
                                                                          const std::optional<std::string>& face_type,
                                                                          std::optional<double> face_thickness = std::nullopt);

protected:
    Flange() = default;
    /// Python's Flange.__init__ (called by the derived constructors).
    void init_flange(const Shape& flange_section, double bcd, int bolt_hole_count, double bolt_hole_diameter,
                     const Rotation& rotation, std::optional<Align3> align);
};

/// BlindFlange.
///
/// Blind Flange: A blind flange is a solid disk with no bore and is used to close off the end of a
/// pipe or vessel. It is commonly used to isolate or terminate piping systems.
///
/// Args:
///     nps: nominal pipe size.
///     flange_class: class.
///     face_type: face options. Defaults to "Raised".
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
///
/// Joints:
///     "face" (Rigid): attachment point on the flange face for other flanges.
class BlindFlange : public Flange {
public:
    BlindFlange(const std::string& nps, int flange_class, const std::string& face_type = "Raised",
                const Rotation& rotation = Rotation(),
                std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    std::string nps;
    int flange_class = 0;
    std::string face_type;
    /// Outside diameter.
    double od = 0.0;
    /// Overall thickness.
    double thickness = 0.0;
};

/// LappedFlange.
///
/// Lap Joint: The flange faces have a flat surface, and a separate lap joint stub end is used
/// between the flanges. It allows easy alignment and rotation of the flanges during assembly.
///
/// A lapped flange, also known as a lap joint flange, is a type of flange connection that
/// consists of two components: a stub end and a lapped flange. The lapped flange has a flat face,
/// while the stub end is a short piece of pipe with a welded-on collar. The lapped flange
/// connection works by sliding the stub end into the lapped flange, with the flange face of the
/// stub end resting against the back face of the lapped flange. The two components are then
/// bolted together, securing the joint. The sealing is achieved by a separate gasket.
///
/// The lapped flange connection offers easy alignment during installation, rotational
/// flexibility, and the ability to easily disconnect and reconnect the joint. It is commonly used
/// in low-pressure and non-critical applications; it is not suitable for high-pressure or
/// high-temperature applications.
///
/// Args:
///     nps: nominal pipe size.
///     flange_class: class.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
///
/// Joints:
///     "lap" (Rigid): where the stub's lap sits (Plane.XY).
class LappedFlange : public Flange {
public:
    LappedFlange(const std::string& nps, int flange_class, const Rotation& rotation = Rotation(),
                 std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    std::string nps;
    int flange_class = 0;
    /// Outside diameter.
    double od = 0.0;
    /// Inside diameter.
    double id = 0.0;
    /// Overall thickness.
    double thickness = 0.0;
};

/// LappedFlangeStub.
///
/// The stub component of a Lapped Flange. The stub is welded to the pipe allowing the Lapped
/// Flange to rotate freely around the stub. The pipe material must be one the material catalog
/// knows (see Pipe).
///
/// Args:
///     nps: stub pipe size.
///     material: pipe material.
///     pipe_identifier: pipe identifier or schedule.
///     stub_length: end to end length.
///     face_type: face options. Defaults to "Raised".
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
///
/// Joints: "pipe" (at the far end of the stub), "face" (Plane.XY) and "lap" (on the lap).
class LappedFlangeStub : public build123d::BasePartObject {
public:
    /// The radius of the weld between the lap and the pipe: 5 mm.
    static constexpr double weld_radius = 5.0;

    LappedFlangeStub(const std::string& nps, const std::string& material, const std::string& pipe_identifier,
                     double stub_length, const std::string& face_type = "Raised", const Rotation& rotation = Rotation(),
                     std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    std::string nps;
    /// The edges where the pipe joins the lap (the weld, filleted).
    ShapeList<Edge> stub_edges;
};

/// SlipOnFlange.
///
/// Slip-On Flange: A slip-on flange has a flat face and is intended to slide over the pipe and
/// then be welded in place. It is commonly used for low-pressure applications and non-critical
/// systems.
///
/// ASME B16.5 does not provide specific guidelines on the amount of space to be left for welding
/// when inserting a pipe into a slip-on flange. However, industry best practices generally
/// recommend leaving a gap of approximately 1/16 inch (1.6 mm) between the end of the pipe and the
/// inside face of the slip-on flange, for the weld bead. When creating pipe terminated with a
/// slip-on flange, the pipe should be 1/16 inch shorter (for each end) than the desired end to end
/// length.
///
/// Args:
///     nps: nominal pipe size.
///     flange_class: class.
///     face_type: face options. Defaults to "Raised".
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
///
/// Joints:
///     "pipe" (Rigid): attachment point for pipe attached to flange.
///     "face" (Rigid): attachment point on the flange face for other flanges.
class SlipOnFlange : public Flange {
public:
    SlipOnFlange(const std::string& nps, int flange_class, const std::string& face_type = "Raised",
                 const Rotation& rotation = Rotation(),
                 std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    std::string nps;
    int flange_class = 0;
    std::string face_type;
    double od = 0.0;
    double id = 0.0;
    double thickness = 0.0;
};

/// SocketWeldFlange.
///
/// Socket Weld Flange: Socket weld flanges have a socket-like end that is welded to the pipe.
/// They are used for small-diameter and high-pressure piping systems.
///
/// Args:
///     nps: nominal pipe size.
///     flange_class: class.
///     face_type: face options. Defaults to "Raised".
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
///
/// Raises:
///     std::invalid_argument: the table has no socket dimensions for the size.
///
/// Joints:
///     "pipe" (Rigid): attachment point for pipe attached to flange.
///     "face" (Rigid): attachment point on the flange face for other flanges.
class SocketWeldFlange : public Flange {
public:
    /// Flange::get_face_section_data, after checking the table has the socket dimensions.
    static std::pair<std::optional<Sketch>, double> get_face_section_data(const std::string& nps, int flange_class,
                                                                          const std::optional<std::string>& face_type,
                                                                          std::optional<double> face_thickness = std::nullopt);

    SocketWeldFlange(const std::string& nps, int flange_class, const std::string& face_type = "Raised",
                     const Rotation& rotation = Rotation(),
                     std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    std::string nps;
    int flange_class = 0;
    std::string face_type;
    double od = 0.0;
    double id = 0.0;
    double thickness = 0.0;
};

/// Weld Neck Flange.
///
/// Weld Neck Flange: This type of flange is designed with a tapered hub and a neck that provides
/// reinforcement and better structural integrity. It is typically used in high-pressure and
/// high-temperature applications.
///
/// The overall thickness of the flange is stored in `thickness`. When sizing a pipe for a given
/// overall length, reduce the pipe length by this value for each end with this flange type.
///
/// Args:
///     nps: nominal pipe size.
///     flange_class: class.
///     face_type: face options. Defaults to "Raised".
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to (Align::CENTER, Align::CENTER, Align::MIN).
///
/// Raises:
///     std::invalid_argument: the table has no welding neck dimensions for the size.
///
/// Joints:
///     "pipe" (Rigid): attachment point for pipe attached to flange.
///     "face" (Rigid): attachment point on the flange face for other flanges.
class WeldNeckFlange : public Flange {
public:
    /// Flange::get_face_section_data, after checking the table has the welding neck bore.
    static std::pair<std::optional<Sketch>, double> get_face_section_data(const std::string& nps, int flange_class,
                                                                          const std::optional<std::string>& face_type,
                                                                          std::optional<double> face_thickness = std::nullopt);

    WeldNeckFlange(const std::string& nps, int flange_class, const std::string& face_type = "Raised",
                   const Rotation& rotation = Rotation(),
                   std::optional<Align3> align = Align3(Align::CENTER, Align::CENTER, Align::MIN));

    std::string nps;
    int flange_class = 0;
    std::string face_type;
    double od = 0.0;
    double id = 0.0;
    double thickness = 0.0;
};

}  // namespace bd_warehouse
