// bd_warehouse-cpp: parametric fasteners (bd_warehouse's fastener.py).
//
// Nuts, screws and washers of many standards (ISO, DIN, ASME) from bd_warehouse's parameter
// tables, and the holes that fit them: ClearanceHole, TapHole, ThreadedHole and InsertHole.
//
// A fastener is a Part. Screws stand with the underside of the head at z=0 and the shank down
// to z=-length (countersunk screws: their top at z=0); nuts and washers sit on z=0. Threads
// are drawn when simple=false (slow; by default a fastener is simple). Holes are Parts to
// subtract, starting at z=0 and going down:
//
//   using namespace bd_warehouse;
//   SocketHeadCapScrew screw("M6-1", 20);
//   Part plate = Box(50, 50, 10, Rotation(), Align3(Align::CENTER, Align::CENTER, Align::MAX));
//   plate -= ClearanceHole(screw, "Normal", 10);
//   HexNut nut("M6-1");                       // nut.nut_thickness(), nut.nut_diameter()
//   PlainWasher washer("M6", "iso7089");
//
// Sizes are "M<diameter>-<pitch>" (e.g. "M6-1") or imperial "<size>-<TPI>" ("#10-32",
// "1/4-20"); washers take the diameter only ("M6"). Each class lists its standards with
// types() and their sizes with sizes(fastener_type); Nut::select_by_size("M6-1") (and the same
// for Screw and Washer) finds the classes and types of a size.
//
// Fasteners carry a material (mild steel, zinc plated) and RigidJoints: "a" at the base of
// nuts and washers (and under the head of a screw) and "b" at the top of nuts and washers.
#pragma once

#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"
#include "bd_warehouse/thread.hpp"

namespace bd_warehouse {

using build123d::Location;
using build123d::Part;
using build123d::Sketch;
using build123d::Solid;

// ---------------------------------------------------------------------------
// Parameter tables and helpers
// ---------------------------------------------------------------------------

/// Distance across polygon diagonals given the width across flats.
double polygon_diagonal(double width, int num_sides = 6);

/// Parse a csv parameter file of bd_warehouse's data directory (e.g.
/// "socket_head_cap_parameters.csv") into a dictionary of strings: the rows by the first
/// column's value, each a dictionary of the other columns. std::invalid_argument
/// ("No header found in <filename>") for an empty file; std::runtime_error for an unknown file.
StringTable read_fastener_parameters_from_csv(const std::string& filename);

/// Extract the major diameter and pitch (mm) from an imperial size: "#10-32" (numbered size
/// and threads per inch) or "1/4-20". std::out_of_range (KeyError) for an unknown "#" size.
std::pair<double, double> decode_imperial_size(const std::string& size);

/// Convert a metric measurement to a number ("12", "2.5", "1/2"); a string that isn't a safe
/// number (see is_safe) is returned as text.
Param metric_str_to_float(const std::string& measure);

/// Convert the strings in a parameter dictionary into dimensions: with is_metric each value
/// through metric_str_to_float, else through imperial_str_to_float (inches to mm).
ParamDict evaluate_parameter_dict(const Dict<std::string, std::string>& parameters, bool is_metric = true);

/// Convert the string values in a dict of dict structure (see evaluate_parameter_dict).
ParamTable evaluate_parameter_dict_of_dict(const StringTable& parameters, bool is_metric = true);

/// Split the fastener data 'type:value' column names: the rows (sizes) with values for
/// `target_fastener`, each with its dimensions by name (empty values left out).
StringTable isolate_fastener_type(const std::string& target_fastener, const StringTable& fastener_data);

/// The drill sizes table (drill_sizes.csv): letter, number and fraction drill sizes to their
/// diameters in mm.
Dict<std::string, double> read_drill_sizes();

/// A dict of dict of drill size to drill diameter: each drill size is looked up in the drill
/// table, else taken as a number (mm for metric "M" sizes, else inches).
ParamTable lookup_drill_diameters(const StringTable& drill_hole_sizes);

/// The nominal screw lengths (mm) of each screw type (nominal_screw_lengths.csv).
Dict<std::string, std::vector<double>> lookup_nominal_screw_lengths();

/// Type H Cross / Phillips recess for screws: its plan and depth. size must be one of: PH0,
/// PH1, PH2, PH3, or PH4 (std::invalid_argument otherwise). The dimensions are simplified to a
/// single value per drive size.
std::pair<Face, double> cross_recess(const std::string& size);

/// Hexagon recess for screws; size is the size across the flats.
Face hex_recess(double size);

/// Plan of the Hexalobular (Torx) recess for screws and its depth (60% of the maximum
/// diameter). size must be one of: T6, T8, T10, T15, T20, T25, T30, T40, T45, T50, T55, T60,
/// T70, T80, T90, T100 (std::invalid_argument otherwise).
std::pair<Face, double> hexalobular_recess(const std::string& size);

/// Slot recess for screws.
Face slot_recess(double width, double length);

/// Robertson Square recess for screws: its plan and depth. size must be one of: R00, R0, R1,
/// R2, and R3 (std::invalid_argument otherwise). Robertson sizes are also color coded: Orange,
/// Yellow, Green, Red, Black.
std::pair<Face, double> square_recess(const std::string& size);

// ---------------------------------------------------------------------------
// Fastener (C++ only: the common base of Nut, Screw and Washer)
// ---------------------------------------------------------------------------

/// The parts Nut, Screw and Washer share (in Python each of them has these attributes): the
/// thread size and the clearance and tap hole tables. Holes take any Fastener.
class Fastener : public build123d::BasePartObject {
public:
    /// Clearance hole drill sizes by size (without the pitch) and fit ("Close", "Normal",
    /// "Loose") (clearance_hole_sizes.csv).
    static const StringTable& clearance_hole_drill_sizes();
    /// The clearance hole diameters (mm) by size and fit.
    static const ParamTable& clearance_hole_data();
    /// Tap drill sizes by thread size and material ("Soft": aluminum, brass and plastics;
    /// "Hard": steel, stainless and iron) (tap_hole_sizes.csv).
    static const StringTable& tap_hole_drill_sizes();
    /// The tap hole diameters (mm) by thread size and material.
    static const ParamTable& tap_hole_data();

    /// A dictionary of drill sizes for tapped holes (std::invalid_argument "No tap hole data for
    /// size ..." if the size has none; washers have no tap holes).
    virtual Dict<std::string, std::string> tap_drill_sizes() const;
    /// A dictionary of drill diameters (mm) for tapped holes, by material.
    virtual ParamDict tap_hole_diameters() const;
    /// A dictionary of drill sizes for clearance holes (washers have none in bd_warehouse).
    virtual Dict<std::string, std::string> clearance_drill_sizes() const;
    /// A dictionary of drill diameters (mm) for clearance holes, by fit.
    ParamDict clearance_hole_diameters() const;

    /// The profile (on Plane.XZ, x >= 0) of the cutter that countersinks the fastener, revolved
    /// about Z by the holes; fit is "Close", "Normal" or "Loose". nullopt for a set screw.
    virtual std::optional<Face> countersink_profile(const std::string& fit) const = 0;

    /// Python's class name of the fastener (e.g. "HexNut").
    const std::string& class_name() const { return class_name_; }

    /// Custom holes locations (bd_warehouse fills them in builder mode only; always empty here).
    std::vector<Location> hole_locations;
    /// The thread size, e.g. "M6-1" (a washer's: its size, e.g. "M6").
    std::string thread_size;
    /// Whether the size is metric (starts with "M").
    bool is_metric = true;
    /// The thread diameter in mm (NaN for a washer size that isn't a number, e.g. "#10", where
    /// bd_warehouse keeps the size's text).
    double thread_diameter = 0.0;
    /// The standard, e.g. "iso4032".
    std::string fastener_type;

protected:
    Fastener() = default;
    std::string class_name_;
};

// ---------------------------------------------------------------------------
// Nuts
// ---------------------------------------------------------------------------

/// The keyword arguments of the nut classes (Python's order and defaults).
struct NutOptions {
    /// The standard; nullopt for the class's default type.
    std::optional<std::string> fastener_type = std::nullopt;
    /// Thread direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Simplify by not creating the thread. Defaults to true.
    bool simple = true;
    /// Object rotation (as in bd_warehouse, it doesn't apply to a nut with a thread, which is
    /// an assembly of its body and thread). Defaults to no rotation.
    build123d::Rotation rotation = build123d::Rotation();
    /// Object alignment (doesn't apply to a nut with a thread either). Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// Parametric Nut: the base class used to create standard threaded nuts.
///
/// Args:
///     size: standard sizes - e.g. "M6-1" (a heat set nut adds its length: "M5-0.8-Standard").
///     fastener_type: type identifier - e.g. "iso4032".
///     hand: thread direction, "right" or "left". Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///
/// Raises:
///     std::invalid_argument: invalid size, must be formatted as size-pitch or size-TPI;
///         invalid fastener_type; invalid hand, must be one of 'left' or 'right'; invalid size.
///
/// A threaded nut is an assembly whose children are its "body" and "thread".
class Nut : public Fastener {
public:
    /// The nut classes and their types of this size, by class name (e.g.
    /// {"HexNut": ["iso4032", "iso4033"]}); the types sorted (Python lists them in set order).
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    /// The profile of the nut on Plane.XZ, revolved about Z.
    virtual Face nut_profile() const = 0;
    /// The plan of the nut (e.g. a hexagon) on Plane.XY.
    virtual Face nut_plan() const = 0;
    /// The flange of a flanged nut (its profile on Plane.XZ), nullopt for others.
    virtual std::optional<Face> flange_profile() const { return std::nullopt; }
    /// Create the nut from the 2D shapes defined in the derived class (a Solid, or the body
    /// and thread assembly when not simple).
    virtual Shape make_nut() const;

    /// Identifying information, e.g. "HexNut(iso4032): M6-1".
    std::string info() const;
    /// Which derived class created this nut.
    std::string nut_class() const { return class_name_; }
    /// The maximum thickness of the nut (its bounding box's max Z).
    double nut_thickness() const;
    /// The maximum diameter of the nut (of the circle through the first three vertices of its plan).
    virtual double nut_diameter() const;
    /// Screw only parameter (0 for nuts).
    double length_offset() const { return 0; }

    /// Create 2D profile of hex nuts with double chamfers.
    Face default_nut_profile() const;
    /// Create a hexagon plan.
    Face default_nut_plan() const;
    /// A simple rectangle which gets revolved into a cylinder with an extra socket_clearance
    /// (defaults to 6mm across the diameter) for a socket wrench.
    Face default_countersink_profile(const std::string& fit) const;

    /// The size as given (stripped), e.g. "M6-1" or "M5-0.8-Standard".
    std::string nut_size;
    /// The length part of a heat set nut size ("Standard"), if any.
    std::optional<std::string> length_size;
    /// The thread pitch in mm.
    double thread_pitch = 0.0;
    /// "right" or "left".
    std::string hand;
    bool simple = true;
    /// Extra clearance (mm) around the nut for a socket wrench when countersinking: 6 mm.
    double socket_clearance = 6.0;
    /// The dimensions of the nut from its parameter table.
    ParamDict nut_data;

protected:
    Nut() = default;
    /// The table of the derived class (its fastener_data).
    virtual const StringTable& class_fastener_data() const = 0;
    /// Python's Nut.__init__ (called by the derived constructors).
    void init(const std::string& class_name, const std::string& size, const std::string& fastener_type,
              const std::string& hand, bool simple, const build123d::Rotation& rotation, std::optional<Align3> align);
};

/// Domed Cap Nut.
///
/// DIN 1587 domed cap nuts, also known as acorn nuts, feature a high, rounded, closed-end top.
/// These nuts are typically used to provide a finished appearance while also protecting exposed
/// bolt threads from damage or corrosion. The dome prevents the entry of dirt and moisture,
/// making them suitable for applications where hygiene, safety, or aesthetics are important.
///
/// Args:
///     size: size specification, e.g. "M6-1".
///     fastener_type: "din1587" (Hexagon domed cap nuts). Defaults to "din1587".
///     hand: thread direction, "right" or "left". Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to nullopt.
class DomedCapNut : public Nut {
public:
    /// The parameter table (domed_cap_nut_parameters.csv).
    static const StringTable& fastener_data();
    /// The nut types ("din1587").
    static std::set<std::string> types();
    /// The sizes of a type.
    static std::vector<std::string> sizes(const std::string& fastener_type);

    explicit DomedCapNut(const std::string& size, const std::string& fastener_type = "din1587",
                         const std::string& hand = "right", bool simple = true,
                         const build123d::Rotation& rotation = build123d::Rotation(),
                         std::optional<Align3> align = std::nullopt);
    DomedCapNut(const std::string& size, const NutOptions& options);

    /// Create 2D profile of the domed nut.
    Face nut_profile() const override;
    Face nut_plan() const override { return default_nut_plan(); }
    /// A simple rectangle which gets revolved into a cylinder with an extra socket_clearance
    /// for a socket wrench.
    std::optional<Face> countersink_profile(const std::string& fit) const override;

protected:
    const StringTable& class_fastener_data() const override { return fastener_data(); }
};

/// Heat Set Nut.
///
/// Heat set insert nuts are specially designed threaded inserts used in thermoplastics to
/// provide durable, reusable threads in 3D printed or injection-molded parts. These inserts
/// are installed by heating them - typically with a soldering iron - and pressing them into a
/// pre-formed hole in the plastic (see InsertHole).
///
/// Args:
///     size: nut size, e.g. "M5-0.8-Standard".
///     fastener_type: standard or manufacturer that defines the nut, "McMaster-Carr",
///         "Hilitchi", ... (see types()). Defaults to "McMaster-Carr".
///     hand: direction of thread. Defaults to "right".
///     simple: omit the thread from the nut. Defaults to true.
///
/// Attributes:
///     fill_factor(): Fraction of insert hole filled with heatset nut.
class HeatSetNut : public Nut {
public:
    static const StringTable& fastener_data();
    static std::set<std::string> types();
    static std::vector<std::string> sizes(const std::string& fastener_type);

    explicit HeatSetNut(const std::string& size, const std::string& fastener_type = "McMaster-Carr",
                        const std::string& hand = "right", bool simple = true,
                        const build123d::Rotation& rotation = build123d::Rotation(),
                        std::optional<Align3> align = std::nullopt);
    HeatSetNut(const std::string& size, const NutOptions& options);

    /// Faces of a knurled cylinder with a central hole (all but the hole), used to build a
    /// knurled HeatSet insert with maximal performance.
    ///
    /// Args:
    ///     diameter: outside diameter of knurled cylinder.
    ///     bottom_hole_radius: size of bottom hole.
    ///     top_hole_radius: size of top hole.
    ///     height: end-to-end height.
    ///     knurl_depth: size of the knurling.
    ///     pitch: knurling helical pitch.
    ///     tip_count: number of knurled tips.
    ///     hand: direction of knurling, "right" or "left". Defaults to "right".
    static std::vector<Face> knurled_cylinder_faces(double diameter, double bottom_hole_radius,
                                                    double top_hole_radius, double height, double knurl_depth,
                                                    double pitch, int tip_count, const std::string& hand = "right");

    /// The maximum diameter of the nut (its table's s).
    double nut_diameter() const override;
    /// Fraction of insert hole filled with heatset nut.
    double fill_factor() const;
    /// Build the heatset nut: a flanged bottom and two knurled sections with opposite twist
    /// direction, assembled from faces into a shell (with the thread, an assembly).
    Shape make_nut() const override;
    /// Not used (std::logic_error).
    Face nut_profile() const override;
    /// Not used (std::logic_error).
    Face nut_plan() const override;
    /// The profile (a Sketch on Plane.XZ) of the cavity the heatset nut is pressed into.
    /// manufacturing_compensation compensates for over-extrusion of 3D printers: 0.2 mm
    /// increases the radius of the hole by 0.2 mm. Defaults to 0.0.
    Sketch countersink_profile(double manufacturing_compensation = 0.0) const;
    /// A heat set nut has no fit (bd_warehouse raises TypeError): std::invalid_argument.
    std::optional<Face> countersink_profile(const std::string& fit) const override;

protected:
    const StringTable& class_fastener_data() const override { return fastener_data(); }
};

/// Hex Nut.
///
/// Hex nuts are the most commonly used type of fastening nut, featuring a six-sided profile
/// that provides a strong grip and easy installation with standard tools.
///
/// ISO 4032, ISO 4033, and ISO 4035 define different variants of metric hex nuts:
/// - ISO 4032 specifies a regular height hex nut with a standard width across flats.
/// - ISO 4033 defines a heavy series hex nut, typically used for larger or more heavily loaded
///   assemblies.
/// - ISO 4035 specifies a thin (jam) nut, used where space is limited or as a locknut.
///
/// Args:
///     size: size specification, e.g. "M6-1".
///     fastener_type: "iso4032" (Hexagon nuts, Style 1), "iso4033" (Style 2) or "iso4035"
///         (Hexagon thin nuts, chamfered). Defaults to "iso4032".
///     hand: thread direction. Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to nullopt.
class HexNut : public Nut {
public:
    static const StringTable& fastener_data();
    static std::set<std::string> types();
    static std::vector<std::string> sizes(const std::string& fastener_type);

    explicit HexNut(const std::string& size, const std::string& fastener_type = "iso4032",
                    const std::string& hand = "right", bool simple = true,
                    const build123d::Rotation& rotation = build123d::Rotation(),
                    std::optional<Align3> align = std::nullopt);
    HexNut(const std::string& size, const NutOptions& options);

    Face nut_profile() const override { return default_nut_profile(); }
    Face nut_plan() const override { return default_nut_plan(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }

protected:
    const StringTable& class_fastener_data() const override { return fastener_data(); }
};

/// Hex Nut With Flange.
///
/// A hex nut with flange combines a standard six-sided nut with an integrated flange at the
/// base, which acts like a built-in washer. DIN 1665 specifies metric hex flange nuts used in
/// general-purpose and structural applications.
///
/// Args:
///     size: size specification, e.g. "M6-1".
///     fastener_type: "din1665" (Hexagon nuts with flange). Defaults to "din1665".
///     hand: thread direction. Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to nullopt.
class HexNutWithFlange : public Nut {
public:
    static const StringTable& fastener_data();
    static std::set<std::string> types();
    static std::vector<std::string> sizes(const std::string& fastener_type);

    explicit HexNutWithFlange(const std::string& size, const std::string& fastener_type = "din1665",
                              const std::string& hand = "right", bool simple = true,
                              const build123d::Rotation& rotation = build123d::Rotation(),
                              std::optional<Align3> align = std::nullopt);
    HexNutWithFlange(const std::string& size, const NutOptions& options);

    Face nut_profile() const override { return default_nut_profile(); }
    Face nut_plan() const override { return default_nut_plan(); }
    /// Flange for hexagon nuts.
    std::optional<Face> flange_profile() const override;
    /// A simple rectangle which gets revolved into a cylinder with at least socket_clearance
    /// (default 6mm across the diameter) for a socket wrench. std::invalid_argument for an
    /// invalid fit.
    std::optional<Face> countersink_profile(const std::string& fit) const override;

protected:
    const StringTable& class_fastener_data() const override { return fastener_data(); }
};

/// Unchamfered Hexagon Nut.
///
/// ISO 4036 defines a thin, unchamfered hexagon nut, typically used in non-critical
/// applications or where space and weight constraints are a priority (e.g. as a jam nut).
///
/// Args:
///     size: size specification, e.g. "M6-1".
///     fastener_type: "iso4036" (Hexagon thin nuts, unchamfered). Defaults to "iso4036".
///     hand: thread direction. Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to nullopt.
class UnchamferedHexagonNut : public Nut {
public:
    static const StringTable& fastener_data();
    static std::set<std::string> types();
    static std::vector<std::string> sizes(const std::string& fastener_type);

    explicit UnchamferedHexagonNut(const std::string& size, const std::string& fastener_type = "iso4036",
                                   const std::string& hand = "right", bool simple = true,
                                   const build123d::Rotation& rotation = build123d::Rotation(),
                                   std::optional<Align3> align = std::nullopt);
    UnchamferedHexagonNut(const std::string& size, const NutOptions& options);

    /// The 2D profile: a rectangle.
    Face nut_profile() const override;
    Face nut_plan() const override { return default_nut_plan(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }

protected:
    const StringTable& class_fastener_data() const override { return fastener_data(); }
};

/// Square Nut.
///
/// Square nuts, as defined by DIN 557, are four-sided nuts commonly used in older machinery,
/// woodworking, and applications where a flat bearing surface and greater resistance to
/// loosening are beneficial.
///
/// Args:
///     size: size specification, e.g. "M6-1".
///     fastener_type: "din557" (Square Nuts). Defaults to "din557".
///     hand: thread direction. Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to nullopt.
class SquareNut : public Nut {
public:
    static const StringTable& fastener_data();
    static std::set<std::string> types();
    static std::vector<std::string> sizes(const std::string& fastener_type);

    explicit SquareNut(const std::string& size, const std::string& fastener_type = "din557",
                       const std::string& hand = "right", bool simple = true,
                       const build123d::Rotation& rotation = build123d::Rotation(),
                       std::optional<Align3> align = std::nullopt);
    SquareNut(const std::string& size, const NutOptions& options);

    /// 2D profile of square nuts with a chamfer.
    Face nut_profile() const override;
    /// Simple square for the plan.
    Face nut_plan() const override;
    /// A simple rectangle which gets revolved into a cylinder with an extra socket_clearance
    /// for a socket wrench.
    std::optional<Face> countersink_profile(const std::string& fit) const override;

protected:
    const StringTable& class_fastener_data() const override { return fastener_data(); }
};

// ---------------------------------------------------------------------------
// Screws
// ---------------------------------------------------------------------------

/// The keyword arguments of the screw classes (Python's order and defaults).
struct ScrewOptions {
    /// The standard; nullopt for the class's default type.
    std::optional<std::string> fastener_type = std::nullopt;
    /// Thread direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Simplify by not creating the thread. Defaults to true.
    bool simple = true;
    /// Object rotation (it doesn't apply to a screw with a thread, which is an assembly).
    /// Defaults to no rotation.
    build123d::Rotation rotation = build123d::Rotation();
    /// Object alignment (doesn't apply to a screw with a thread). Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// A screw head recess: its plan, depth and taper angle (degrees).
using HeadRecess = std::tuple<Face, double, double>;

/// Parametric Screw: the base class for a set of threaded screws or bolts.
///
/// Args:
///     size: standard sizes - e.g. "M6-1".
///     length: distance from base of head to tip of thread (mm).
///     fastener_type: type identifier - e.g. "iso4014".
///     hand: thread direction, "right" or "left". Defaults to "right".
///     simple: simplify by not creating the thread. Defaults to true.
///     socket_clearance: gap around screw with no recess (e.g. hex head) which allows a socket
///         wrench to be inserted. Defaults to 6mm.
///
/// Raises:
///     std::invalid_argument: invalid size, must be formatted as size-pitch or size-TPI;
///         invalid fastener_type; invalid hand, must be one of 'left' or 'right'; invalid size;
///         screw length <= countersunk screw head; invalid thread or shank length.
///
/// A threaded screw is an assembly whose children are its "body" and "thread".
class Screw : public Fastener {
public:
    /// Nominal screw lengths (mm) keyed by screw type.
    static const Dict<std::string, std::vector<double>>& nominal_length_range();
    /// The screw classes and their types of this size, by class name; the types sorted.
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    /// How much the screw sinks into the part: the head height of countersunk screws, else 0.
    virtual double length_offset() const { return 0; }
    /// The thread length of the standard (by default the whole max_thread_length).
    virtual double calculate_thread_length() const { return default_thread_length(); }
    /// The unthreaded shank length (by default max_thread_length - thread_length).
    virtual double calculate_shank_length() const { return default_shank_length(); }
    /// The distance from the end of the shank to the start of the thread (by default 0).
    virtual double calculate_thread_offset() const { return default_thread_offset(); }
    /// The profile of the head on Plane.XZ (nullopt if the class has none).
    virtual std::optional<Face> head_profile() const { return std::nullopt; }
    /// The plan of the head (e.g. a hexagon), nullopt if the head is round.
    virtual std::optional<Face> head_plan() const { return std::nullopt; }
    /// The drive recess of the head (plan, depth, taper), nullopt if the head has none.
    virtual std::optional<HeadRecess> head_recess() const { return std::nullopt; }
    /// The profile of a flange under the head, nullopt if there is none.
    virtual std::optional<Face> flange_profile() const { return std::nullopt; }
    /// The profile of the unthreaded shank on Plane.XZ (nullopt if there is none).
    virtual std::optional<Face> shank_profile() const { return default_shank_profile(); }
    /// A screw made by the class itself (set screws), nullopt for the others.
    virtual std::optional<Shape> custom_make() const { return std::nullopt; }
    /// The head from head_profile, head_plan, head_recess and flange_profile; nullopt for a
    /// fully custom screw. std::invalid_argument if the class has no head_profile.
    virtual std::optional<Shape> make_head() const;
    /// The cylinder of the hole for the screw's body (down from z=0).
    virtual Solid make_body_hole(double hole_radius, double depth, double head_offset = 0) const;
    /// The minimum depth of a hole for this screw (with the head countersunk or not).
    virtual double min_hole_depth(bool counter_sunk = true) const;

    /// The whole max_thread_length.
    double default_thread_length() const { return max_thread_length; }
    /// ISO 888 thread length: 2d + 6 (length <= 125), 2d + 12 (<= 200), else 2d + 25, at most
    /// max_thread_length.
    double iso888_thread_length() const;
    /// ISO 7380 thread length: 2d + 12 (3d for M16), at most max_thread_length.
    double iso7380_thread_length() const;
    /// Threaded to the head, less the maximum incomplete thread under the head (2P).
    double threaded_to_head_thread_length() const;
    /// The machine-screw standards' thread length: threaded to the head up to the standard's
    /// cutoff length, ISO 888 beyond.
    double machine_screw_thread_length() const;
    /// ASME B18.6.3 thread length.
    double asme_b18_6_3_thread_length() const;
    /// ASME B18.3 thread length (minimum thread lengths of socket head cap screws).
    double asme_b18_3_thread_length() const;
    /// max_thread_length - thread_length.
    double default_shank_length() const { return max_thread_length - thread_length; }
    /// 0.
    double default_thread_offset() const { return 0; }
    /// A rectangle (the shank) under the head, nullopt if the grip length is 0.
    std::optional<Face> default_shank_profile() const;
    /// The default recess: a slot (dk, n, t), a hex socket (s, t) or a cross, hexalobular or
    /// square drive (recess) of the screw's table. std::invalid_argument if there is none.
    HeadRecess default_head_recess() const;
    /// A rectangle as wide as the head plus the fit's clearance and as high as the head.
    Face default_countersink_profile(const std::string& fit) const;

    /// The nominal lengths (mm) of this screw's size, nullopt if unknown.
    std::optional<std::vector<double>> nominal_lengths() const;
    /// Identifying information, e.g. "SocketHeadCapScrew(iso4762): M6-1x20.0".
    std::string info() const;
    /// Which derived class created this screw.
    std::string screw_class() const { return class_name_; }

    /// The size as given, e.g. "M6-1".
    std::string screw_size;
    /// Maximum height of the screw head.
    double head_height = 0.0;
    /// Maximum diameter of the screw head.
    double head_diameter = 0.0;
    /// The thread pitch in mm.
    double thread_pitch = 0.0;
    /// The screw length (mm).
    double length = 0.0;
    std::string hand;
    bool simple = true;
    /// The dimensions of the screw from its parameter table.
    ParamDict screw_data;
    double socket_clearance = 6.0;
    /// length - length_offset().
    double max_thread_length = 0.0;
    double thread_length = 0.0;
    /// The unthreaded shank length.
    double grip_length = 0.0;
    double thread_offset = 0.0;

protected:
    Screw() = default;
    virtual const StringTable& class_fastener_data() const = 0;
    /// Python's Screw.__init__ (called by the derived constructors).
    void init(const std::string& class_name, const std::string& size, double length, const std::string& fastener_type,
              const std::string& hand, bool simple, double socket_clearance, const build123d::Rotation& rotation,
              std::optional<Align3> align);
};

#define BD_WAREHOUSE_SCREW_CLASS_MEMBERS(Class, default_type)                                          \
public:                                                                                               \
    static const StringTable& fastener_data();                                                        \
    static std::set<std::string> types();                                                             \
    static std::vector<std::string> sizes(const std::string& fastener_type);                          \
    Class(const std::string& size, double length, const std::string& fastener_type = default_type,     \
          const std::string& hand = "right", bool simple = true,                                       \
          const build123d::Rotation& rotation = build123d::Rotation(), std::optional<Align3> align = std::nullopt); \
    Class(const std::string& size, double length, const ScrewOptions& options);                        \
                                                                                                      \
protected:                                                                                            \
    const StringTable& class_fastener_data() const override { return fastener_data(); }              \
                                                                                                      \
public:

/// Button Head Screw.
///
/// ISO 7380-1 defines hexagon socket button head screws, characterized by a low-profile,
/// rounded head and an internal hex drive. Common in enclosures, furniture, robotics, and
/// lightweight mechanical assemblies.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso7380_1", the default), hand
/// ("right"), simple (true), rotation, align - or ScrewOptions.
class ButtonHeadScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(ButtonHeadScrew, "iso7380_1")
    double calculate_thread_length() const override { return iso7380_thread_length(); }
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Button Head With Collar Screw.
///
/// ISO 7380-2 defines hexagon socket button head screws with an integrated collar or washer
/// face beneath the head, which increases the bearing surface area.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso7380_2", the default), hand,
/// simple, rotation, align - or ScrewOptions.
class ButtonHeadWithCollarScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(ButtonHeadWithCollarScrew, "iso7380_2")
    double calculate_thread_length() const override { return iso7380_thread_length(); }
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    /// A rectangle as wide as the collar plus the fit's clearance.
    std::optional<Face> countersink_profile(const std::string& fit) const override;
};

/// Cheese Head Screw.
///
/// Cheese head screws are cylindrical-head fasteners with vertical sides and a flat top.
/// - ISO 1207: Slotted cheese head machine screws for general applications.
/// - ISO 7048: Cross-recessed (Phillips) cheese head screws.
/// - ISO 14580: Hex socket (hexalobular) cheese head screws, often used in precision assemblies.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso1207", "iso7048" (the default)
/// or "iso14580"), hand, simple, rotation, align - or ScrewOptions.
class CheeseHeadScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(CheeseHeadScrew, "iso7048")
    double calculate_thread_length() const override { return machine_screw_thread_length(); }
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// CounterSunk Screw.
///
/// Countersunk screws are flat-head fasteners designed to sit flush with or below the surface;
/// the head's top is at z=0.
/// - ISO 2009: Slotted countersunk head screws.
/// - ISO 7046: Cross-recessed (Phillips) countersunk screws for general use.
/// - ISO 10642: Hex socket countersunk screws, often used in machinery and precision equipment.
/// - ISO 14581: Low-profile, hexalobular countersunk screws.
/// - ISO 14582: Hex socket, low-profile countersunk screws.
///
/// Args: size (e.g. "M6-1"), length (mm, overall), fastener_type ("iso2009", "iso7046",
/// "iso10642" (the default), "iso14581" or "iso14582"), hand, simple, rotation, align - or
/// ScrewOptions.
class CounterSunkScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(CounterSunkScrew, "iso10642")
    double calculate_thread_length() const override;
    /// The head height k.
    double length_offset() const override;
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    /// The countersink (a trapezoid as large as the head).
    std::optional<Face> countersink_profile(const std::string& fit) const override;
};

/// Hex Head Screw.
///
/// Hex head screws, also referred to as hex bolts or hex cap screws.
/// - DIN 931: Hexagon head bolts with a partially threaded shank.
/// - ISO 4014: Hex head screws with a partially threaded shank.
/// - ISO 4017: Fully threaded hex head screws.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("din931", "iso4014" (the default) or
/// "iso4017"), hand, simple, rotation, align - or ScrewOptions.
class HexHeadScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(HexHeadScrew, "iso4014")
    double calculate_thread_length() const override;
    std::optional<Face> head_profile() const override;
    std::optional<Face> head_plan() const override;
    /// A rectangle wide enough for a socket wrench (the fit isn't used).
    std::optional<Face> countersink_profile(const std::string& fit) const override;
};

/// Hex Head With Flange Screw.
///
/// Hex head screws with flanges combine a standard hexagonal head with an integrated
/// washer-like flange at the base.
/// - DIN 1662: Hex flange screws with a partially threaded shank.
/// - DIN 1665: Heavy-series hex flange screws with a partially threaded shank.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("din1662" (the default) or
/// "din1665"), hand, simple, rotation, align - or ScrewOptions.
class HexHeadWithFlangeScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(HexHeadWithFlangeScrew, "din1662")
    double calculate_thread_length() const override { return iso888_thread_length(); }
    std::optional<Face> head_profile() const override;
    std::optional<Face> head_plan() const override;
    /// Flange for hexagon bolts.
    std::optional<Face> flange_profile() const override;
    /// A rectangle as wide as the flange plus clearance, or wide enough for a socket wrench.
    std::optional<Face> countersink_profile(const std::string& fit) const override;
};

/// Pan Head Screw.
///
/// Pan head screws feature a broad, low-profile head with gently curved sides.
/// - ISO 1580: Slotted pan head machine screws for general mechanical use.
/// - ISO 14583: Pan head screws with a hexalobular (Torx) drive.
/// - ASME B18.6.3: The U.S. standard for slotted, cross-recessed (Phillips), or
///   combination-drive pan head machine screws.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso1580", "iso14583" (the default)
/// or "asme_b_18.6.3"), hand, simple, rotation, align - or ScrewOptions.
class PanHeadScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(PanHeadScrew, "iso14583")
    double calculate_thread_length() const override;
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Pan Head With Collar Screw.
///
/// DIN 967 defines a pan head screw with an integrated collar or washer-like flange beneath
/// the head.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("din967", the default), hand, simple,
/// rotation, align - or ScrewOptions.
class PanHeadWithCollarScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(PanHeadWithCollarScrew, "din967")
    double calculate_thread_length() const override { return machine_screw_thread_length(); }
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Raised Cheese Head Screw.
///
/// ISO 7045 defines raised cheese head screws with a cylindrical, slightly domed head and a
/// flat underside.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso7045", the default), hand, simple,
/// rotation, align - or ScrewOptions.
class RaisedCheeseHeadScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(RaisedCheeseHeadScrew, "iso7045")
    double calculate_thread_length() const override { return machine_screw_thread_length(); }
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Raised CounterSunk Oval Head Screw.
///
/// Raised countersunk screws - also known as oval head screws - feature a conical bearing
/// surface like standard countersunk screws, but with a gently domed top.
/// - ISO 2010: Slotted raised countersunk head screws.
/// - ISO 7047: Cross-recessed (Phillips) version.
/// - ISO 14584: Hexalobular (Torx) drive version.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso2010", "iso7047" or "iso14584"
/// (the default)), hand, simple, rotation, align - or ScrewOptions.
class RaisedCounterSunkOvalHeadScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(RaisedCounterSunkOvalHeadScrew, "iso14584")
    double calculate_thread_length() const override;
    /// The head height k (without the raised part).
    double length_offset() const override;
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    /// The countersink (the head's cone).
    std::optional<Face> countersink_profile(const std::string& fit) const override;
};

/// Set Screw.
///
/// ISO 4026 defines set screws with a flat point and a hexagon socket drive. These screws are
/// fully threaded and lack a head; they stand from z=0 down to z=-length.
///
/// Args: size (e.g. "M6-1"), length (mm), fastener_type ("iso4026", the default), hand, simple,
/// rotation, align - or ScrewOptions.
class SetScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(SetScrew, "iso4026")
    std::optional<Shape> custom_make() const override { return make_setscrew(); }
    /// The set screw: a cylinder with a hexagon socket.
    Solid make_setscrew() const;
    /// nullopt (no head).
    std::optional<Shape> make_head() const override { return std::nullopt; }
    /// nullopt (nothing to countersink).
    std::optional<Face> countersink_profile(const std::string& fit) const override;
};

/// Shoulder Screw.
///
/// ISO 7379 defines metric hexagon socket head shoulder screws with a precision cylindrical
/// shoulder between the head and the threaded portion, used as pivot pins, guide pins, axles,
/// stops, and bearing or linkage supports.
///
/// Args:
///     size: thread size specification, e.g. "M6-1".
///     length: shoulder length measured from beneath the head to the start of the thread
///         relief (mm); one of the standard's nominal lengths for the size (std::invalid_argument
///         otherwise).
///     fastener_type, hand, simple, rotation, align: as the other screws ("iso7379").
class ShoulderScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(ShoulderScrew, "iso7379")
    double calculate_thread_length() const override;
    double calculate_shank_length() const override;
    double calculate_thread_offset() const override;
    double min_hole_depth(bool counter_sunk = true) const override;
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    /// The shoulder and its thread relief.
    std::optional<Face> shank_profile() const override;
    std::optional<Face> countersink_profile(const std::string& fit) const override;
    /// The thread hole with the shoulder's hole above it.
    Solid make_body_hole(double hole_radius, double depth, double head_offset = 0) const override;

private:
    static std::string validated(const std::string& size, double length, const std::string& fastener_type);
};

/// Socket Head Cap Screw.
///
/// Socket head cap screws are high-strength fasteners with a cylindrical head and an internal
/// hexagonal drive.
/// - ISO 4762 specifies metric socket head cap screws.
/// - ASME B18.3 defines the inch-based counterpart.
///
/// Args: size (e.g. "M6-1" or "#10-32"), length (mm), fastener_type ("iso4762" (the default)
/// or "asme_b18.3"), hand, simple, rotation, align - or ScrewOptions.
class SocketHeadCapScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(SocketHeadCapScrew, "iso4762")
    double calculate_thread_length() const override;
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Low Profile Screw.
///
/// Low profile screws have a wide, flat, and shallow head that minimizes protrusion above the
/// mating surface, as used with OpenBuilds V-slot and C-beam systems. Its material is mild
/// steel, zinc plated and black oxide.
///
/// Args: size (e.g. "M5-0.8"), length (mm), fastener_type ("OpenBuilds", the default), hand,
/// simple, rotation, align - or ScrewOptions.
class LowProfileScrew : public Screw {
    BD_WAREHOUSE_SCREW_CLASS_MEMBERS(LowProfileScrew, "OpenBuilds")
    std::optional<Face> head_profile() const override;
    std::optional<HeadRecess> head_recess() const override { return default_head_recess(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

#undef BD_WAREHOUSE_SCREW_CLASS_MEMBERS

// ---------------------------------------------------------------------------
// Washers
// ---------------------------------------------------------------------------

/// Parametric Washer: the base class used to create standard washers.
///
/// Args:
///     size: standard sizes - e.g. "M6".
///     fastener_type: type identifier - e.g. "iso7089".
///
/// Raises:
///     std::invalid_argument: invalid fastener_type; invalid size.
class Washer : public Fastener {
public:
    /// The washer classes and their types of this size, by class name; the types sorted.
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    /// The profile of the washer on Plane.XZ, revolved about Z.
    virtual Face washer_profile() const = 0;
    /// Create the washer (by default the profile revolved into a Solid).
    virtual Shape make_washer() const;
    /// Washers have no tap holes nor clearance drill sizes (std::invalid_argument, Python's
    /// AttributeError).
    Dict<std::string, std::string> tap_drill_sizes() const override;
    Dict<std::string, std::string> clearance_drill_sizes() const override;
    ParamDict tap_hole_diameters() const override;

    /// Identifying information, e.g. "PlainWasher(iso7089): M6".
    std::string info() const;
    /// Which derived class created this washer.
    std::string washer_class() const { return class_name_; }
    /// The thickness of the washer (its bounding box's height).
    double washer_thickness() const;
    /// The diameter of the washer (twice the farthest vertex from the Z axis).
    double washer_diameter() const;

    /// A rectangle (d1 to d2 by h) at d1 / 2.
    Face default_washer_profile() const;
    /// A rectangle as wide as the washer plus the fit's gap.
    Face default_countersink_profile(const std::string& fit) const;

    /// The size as given, e.g. "M6".
    std::string washer_size;
    /// The dimensions of the washer from its parameter table.
    ParamDict washer_data;

protected:
    Washer() = default;
    virtual const StringTable& class_fastener_data() const = 0;
    void init(const std::string& class_name, const std::string& size, const std::string& fastener_type,
              const build123d::Rotation& rotation, std::optional<Align3> align);
};

#define BD_WAREHOUSE_WASHER_CLASS_MEMBERS(Class)                                     \
public:                                                                             \
    static const StringTable& fastener_data();                                      \
    static std::set<std::string> types();                                           \
    static std::vector<std::string> sizes(const std::string& fastener_type);        \
                                                                                    \
protected:                                                                          \
    const StringTable& class_fastener_data() const override { return fastener_data(); } \
                                                                                    \
public:

/// Plain Washer.
///
/// Plain washers are flat, disc-shaped components used under the head of a screw or nut to
/// distribute the clamping load and protect the mating surface from damage.
/// - ISO 7089: Normal series washers with standard outer diameter and thickness.
/// - ISO 7091: Normal series washers with slightly tighter tolerances.
/// - ISO 7093: Large series washers with greater outer diameter.
/// - ISO 7094: Extra-large series washers.
///
/// Args:
///     size: size specification, e.g. "M6".
///     fastener_type: "iso7089", "iso7091", "iso7093" or "iso7094" (required).
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to nullopt.
class PlainWasher : public Washer {
    BD_WAREHOUSE_WASHER_CLASS_MEMBERS(PlainWasher)
    PlainWasher(const std::string& size, const std::string& fastener_type,
                const build123d::Rotation& rotation = build123d::Rotation(), std::optional<Align3> align = std::nullopt);
    Face washer_profile() const override { return default_washer_profile(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Chamfered Washer.
///
/// ISO 7090 defines chamfered washers, plain washers with a beveled edge.
///
/// Args:
///     size: size specification, e.g. "M6".
///     fastener_type: "iso7090". Defaults to "iso7090".
///     rotation, align: as the other washers.
class ChamferedWasher : public Washer {
    BD_WAREHOUSE_WASHER_CLASS_MEMBERS(ChamferedWasher)
    explicit ChamferedWasher(const std::string& size, const std::string& fastener_type = "iso7090",
                             const build123d::Rotation& rotation = build123d::Rotation(),
                             std::optional<Align3> align = std::nullopt);
    /// The plain washer profile with its top outer corner chamfered.
    Face washer_profile() const override;
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Cheese Head Washer.
///
/// ISO 7092 defines plain washers specifically designed for use with cheese head screws.
///
/// Args:
///     size: size specification, e.g. "M6".
///     fastener_type: "iso7092". Defaults to "iso7092".
///     rotation, align: as the other washers.
class CheeseHeadWasher : public Washer {
    BD_WAREHOUSE_WASHER_CLASS_MEMBERS(CheeseHeadWasher)
    explicit CheeseHeadWasher(const std::string& size, const std::string& fastener_type = "iso7092",
                              const build123d::Rotation& rotation = build123d::Rotation(),
                              std::optional<Align3> align = std::nullopt);
    /// The plain washer profile with its inner corners chamfered.
    Face washer_profile() const override;
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

/// Internal Tooth Lock Washer.
///
/// Internal tooth lock washers are circular washers with multiple sharp, radially
/// inward-facing teeth designed to bite into the surface of the mating part and the underside
/// of the fastener head.
/// - DIN 6797 defines metric internal tooth lock washers.
/// - ASME B18.21.1 provides similar specifications for inch-based applications.
///
/// Args:
///     size: size specification, e.g. "M6" (or "#10", "1/4" for asme_b18.21.1).
///     fastener_type: "din6797" or "asme_b18.21.1". Defaults to "din6797".
///     rotation, align: as the other washers.
class InternalToothLockWasher : public Washer {
    BD_WAREHOUSE_WASHER_CLASS_MEMBERS(InternalToothLockWasher)
    explicit InternalToothLockWasher(const std::string& size, const std::string& fastener_type = "din6797",
                                     const build123d::Rotation& rotation = build123d::Rotation(),
                                     std::optional<Align3> align = std::nullopt);
    /// The washer with its teeth (lofted between projected faces), a Compound.
    Shape make_washer() const override;
    Face washer_profile() const override { return default_washer_profile(); }
    std::optional<Face> countersink_profile(const std::string& fit) const override {
        return default_countersink_profile(fit);
    }
};

#undef BD_WAREHOUSE_WASHER_CLASS_MEMBERS

// ---------------------------------------------------------------------------
// Holes
// ---------------------------------------------------------------------------

/// Part Object: ClearanceHole.
///
/// A clearance hole is a hole that is drilled through one of two (or more) parts to be
/// assembled together with a screw or bolt. The diameter of the clearance hole is larger than
/// the diameter of the screw or bolt so that it can pass through freely without engaging the
/// threads. The hole goes down from z=0, with a countersink for the fastener (its head or the
/// nut) and a drill tip; subtract it from a part: `part -= Pos(x, y, top) * ClearanceHole(screw, "Normal", 10)`.
///
/// Args:
///     fastener: A nut or screw instance (a washer works too).
///     fit: Control hole diameter, "Close", "Normal" or "Loose". Defaults to "Normal".
///     depth: hole depth (mm). Required here (bd_warehouse takes twice the size of the part in
///         builder mode): nullopt throws "No depth provided".
///     counter_sunk: Is the fastener countersunk into the part?. Defaults to true.
///     captive_nut: Is rotation of the nut disabled?. Defaults to false.
///     rotation: object rotation. Defaults to no rotation.
///
/// Raises:
///     std::invalid_argument: Use InsertHole for HeatSetNut; Invalid nut given; No depth
///         provided; invalid fit.
class ClearanceHole : public build123d::BasePartObject {
public:
    explicit ClearanceHole(const Fastener& fastener, const std::string& fit = "Normal",
                           std::optional<double> depth = std::nullopt, bool counter_sunk = true,
                           bool captive_nut = false, const build123d::Rotation& rotation = build123d::Rotation());
    /// The depth of the hole.
    double hole_depth = 0.0;
};

/// Part Object: TapHole.
///
/// A tap hole is precisely drilled through a component, sized optimally for a subsequent
/// operation where a tap is employed to cut threads into the material. The diameter of this
/// pre-drilled hole is critical and is influenced by the specifications of the intended
/// thread and the material.
///
/// Args:
///     fastener: A nut or screw instance.
///     material: Type of material, "Soft" (aluminum, brass and plastics) or "Hard" (steel,
///         stainless and iron). Defaults to "Soft".
///     fit: Control hole diameter of the countersink. Defaults to "Normal".
///     depth: hole depth (mm), required (see ClearanceHole).
///     counter_sunk: Is the fastener countersunk into the part?. Defaults to true.
///
/// Raises:
///     std::invalid_argument: Use InsertHole for HeatSetNut; No depth provided; invalid material.
class TapHole : public build123d::BasePartObject {
public:
    explicit TapHole(const Fastener& fastener, const std::string& material = "Soft",
                     const std::string& fit = "Normal", std::optional<double> depth = std::nullopt,
                     bool counter_sunk = true);
    double hole_depth = 0.0;
};

/// Part Object: ThreadedHole.
///
/// A threaded hole: a tap hole with the thread to cut (an external IsoThread of the
/// fastener's size, as long as the hole below the countersink) when simple is false. The
/// result is an assembly of the cutters (hole, countersink and thread solids).
///
/// Args:
///     fastener: A screw instance (bd_warehouse fails for nuts: their body has no hole).
///     material: Type of material, "Soft" or "Hard". Defaults to "Soft".
///     fit: Control hole diameter of the countersink. Defaults to "Normal".
///     depth: hole depth (mm), required (see ClearanceHole); greater than 0.
///     counter_sunk: Is the fastener countersunk into the part?. Defaults to true.
///     simple: leave out the thread. Defaults to true.
///
/// Raises:
///     std::invalid_argument: Use InsertHole for HeatSetNut; No depth provided; Hole depth must
///         be greater than zero; invalid material; hole too shallow for the countersink.
class ThreadedHole : public build123d::BasePartObject {
public:
    explicit ThreadedHole(const Fastener& fastener, const std::string& material = "Soft",
                          const std::string& fit = "Normal", std::optional<double> depth = std::nullopt,
                          bool counter_sunk = true, bool simple = true);
    double hole_depth = 0.0;
    /// The thread cut into the hole (nullopt when simple).
    std::optional<IsoThread> thread;
    /// The locations the hole was made at (builder mode only: empty here).
    std::vector<Location> thread_locations;
};

/// Part Object: InsertHole.
///
/// The hole a heat set insert is pressed into: a cavity a little smaller than the insert.
///
/// Args:
///     fastener: A heatset nut.
///     fit: Control the hole diameter below the insert. Defaults to "Normal".
///     depth: hole depth (mm), required (see ClearanceHole).
///     manufacturing_compensation: used to compensate for over-extrusion of 3D printers. A
///         value of 0.2mm will increase the radius of the hole by 0.2mm. Defaults to 0.0.
///
/// Raises:
///     std::invalid_argument: No depth provided; invalid fit.
class InsertHole : public build123d::BasePartObject {
public:
    explicit InsertHole(const HeatSetNut& fastener, const std::string& fit = "Normal",
                        std::optional<double> depth = std::nullopt, double manufacturing_compensation = 0.0);
    double hole_depth = 0.0;
};

}  // namespace bd_warehouse
