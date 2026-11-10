// bd_warehouse-cpp: parametric threads (bd_warehouse's thread.py).
//
// Helical threads made by lofting the thread profile along a helix: Thread (any trapezoidal
// profile), IsoThread (ISO 60° metric and UTS), WhitworthThread and BSPPThread (55° rounded,
// ISO 228-1 pipe threads), TrapezoidalThread with AcmeThread and MetricTrapezoidalThread (lead
// screws), and PlasticBottleThread (ASTM D2911 and PCO1881 bottle finishes).
//
// Every thread is a Part along +Z from z=0 to z=length (the thread core - the bolt shank or the
// hole - is not included). External threads point outwards, internal threads point inwards
// and are meant to be fused with the inside of a hole.
//
//   using namespace bd_warehouse;
//   IsoThread thread(10, 1.5, 20);                                    // M10 external, 20 long
//   IsoThread nut_thread(10, 1.5, 8, false, "right", {"fade", "fade"});  // internal
//   Part bolt = Cylinder(thread.min_radius(), 20, Rotation(), Align3(Align::CENTER, Align::CENTER, Align::MIN)) + thread;
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Align3;
using build123d::Face;
using build123d::Rotation;

/// The finish of each end of a thread, (bottom, top), each one of:
///   "raw"     unfinished, which typically results in the thread extended below z=0 or
///             above z=length
///   "fade"    the thread height drops to zero over 90° of arc (or 1/4 pitch)
///   "square"  clipped by the z=0 or z=length plane
///   "chamfer" conical ends which facilitate the alignment of a bolt into a nut
using EndFinishes = std::pair<std::string, std::string>;

/// Whether `value` is a number or fraction that's safe to evaluate: at most 10 characters,
/// all of them digits, '.', '/' or ' ' (bd_warehouse's is_safe).
bool is_safe(const std::string& value);

/// Convert an imperial measurement (possibly a fraction, e.g. "1/4" or "1 1/2") to a float
/// value in mm (inches times build123d::IN). A string that isn't safe (see is_safe) is
/// returned unchanged, as text in the Param (bd_warehouse returns the str).
/// Throws std::invalid_argument for a malformed number (Python's SyntaxError) and
/// std::domain_error for a division by zero.
Param imperial_str_to_float(const std::string& measure);

// ---------------------------------------------------------------------------
// Thread
// ---------------------------------------------------------------------------

/// The keyword arguments of Thread (in Python's order, with Python's defaults).
struct ThreadOptions {
    /// Asymmetric thread apex offset from center. Defaults to 0.0.
    double apex_offset = 0.0;
    /// Amount the thread will overlap with nut or bolt core. Used to help create valid threaded
    /// objects where the thread must fuse with another object. For threaded objects built as
    /// Compounds, this value could be set to 0.0. Defaults to 0.2.
    double interference = 0.2;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Cone angle for tapered thread. Not supported: anything but nullopt throws. Defaults to
    /// nullopt.
    std::optional<double> taper_angle = std::nullopt;
    /// Profile of each end, "raw", "fade", "square" or "chamfer". Defaults to ("raw", "raw").
    EndFinishes end_finishes = {"raw", "raw"};
    /// Stop at the thread calculation, don't create the thread. Defaults to false.
    bool simple = false;
    /// Radial allowance for vertically printed FDM threads. A positive value moves an external
    /// thread inward or an internal thread outward without changing its pitch or profile.
    /// Calibrate the value for the printer, material, and print settings. It does not
    /// compensate horizontal-print stair stepping or sagging. Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    /// Object rotation (as in Python, it doesn't apply: the thread is made of its loops,
    /// see Thread). Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Object alignment (doesn't apply either, see Thread). Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// Helical thread.
///
/// The most general thread class used to build all of the other threads. Creates a right or
/// left hand helical thread with the given root and apex radii: a thread profile (a trapezoid
/// with the root on the core cylinder and the apex at the tip) lofted along a helix, one loop
/// per pitch, with the end finishes applied to the first and last loops.
///
/// The result is an assembly: its children are the loops ("loop", "partial") and the fade tips
/// ("bottom_tip", "top_tip"), and the part is the compound of them. As in bd_warehouse,
/// rotation and align don't apply to a Thread (build123d rebuilds an object with children from
/// them); move the thread instead, e.g. `Pos(0, 0, 5) * thread`.
///
/// Args:
///     apex_radius: Radius at the narrow tip of the thread (mm).
///     apex_width: Width at the narrow tip of the thread (mm).
///     root_radius: Radius at the wide base of the thread (mm).
///     root_width: Thread base width (mm).
///     pitch: Length of 360° of thread rotation (mm).
///     length: End to end length of the thread (mm).
///     apex_offset .. align: see ThreadOptions.
///
/// Raises:
///     std::invalid_argument: if end_finishes are not "raw", "square", "fade" or "chamfer";
///         if taper_angle is given (not currently supported).
class Thread : public build123d::BasePartObject {
public:
    Thread(double apex_radius, double apex_width, double root_radius, double root_width, double pitch,
           double length, double apex_offset = 0.0, double interference = 0.2, const std::string& hand = "right",
           std::optional<double> taper_angle = std::nullopt, const EndFinishes& end_finishes = {"raw", "raw"},
           bool simple = false, double manufacturing_compensation = 0.0, const Rotation& rotation = Rotation(),
           std::optional<Align3> align = std::nullopt);
    Thread(double apex_radius, double apex_width, double root_radius, double root_width, double pitch,
           double length, const ThreadOptions& options);

    /// Whether the thread is external (apex_radius > root_radius).
    bool external = true;
    double apex_radius = 0.0;
    double apex_width = 0.0;
    double root_radius = 0.0;
    double root_width = 0.0;
    double pitch = 0.0;
    double length = 0.0;
    double apex_offset = 0.0;
    double interference = 0.0;
    /// hand == "right".
    bool right_hand = true;
    EndFinishes end_finishes;
    /// |apex_radius - root_radius|.
    double tooth_height = 0.0;
    /// 0 (taper_angle isn't supported).
    double taper = 0.0;
    bool simple = false;
    double manufacturing_compensation = 0.0;
    /// The number of loops of the sweep (length / pitch, adjusted for faded ends); nullopt for a
    /// simple thread.
    std::optional<double> thread_loops;
    /// The thread profile face on Plane.XY (x along the axis, y radial).
    Face thread_profile;
};

// ---------------------------------------------------------------------------
// WhitworthThread, BSPPThread
// ---------------------------------------------------------------------------

/// The keyword arguments of WhitworthThread.
struct WhitworthThreadOptions {
    /// External or internal thread selector. Defaults to true.
    bool external = true;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Profile of the two ends. Defaults to ("fade", "square").
    EndFinishes end_finishes = {"fade", "square"};
    /// Radial overlap with the mating cylindrical core. Defaults to 0.2.
    double interference = 0.2;
    /// Calculate the profile without creating the swept thread. Defaults to false.
    bool simple = false;
    /// Radial depth removed from the theoretical rounded crest before sweeping. Defaults to 0.0.
    double crest_truncation = 0.0;
    /// Radial allowance for vertically printed FDM threads. A positive value moves an external
    /// thread inward or an internal thread outward. Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    /// Object rotation (doesn't apply, see Thread). Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Object alignment (doesn't apply, see Thread). Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// Full- or truncated-form 55° Whitworth thread.
///
/// Creates the rounded crest and root profile used as the basic thread form by standards
/// including ISO 228. The full profile has tangent circular arcs at both the crest and roots. A
/// truncated profile is made by splitting this full profile at the requested radial crest
/// depth before it is swept. Like Thread, the result is made of its loops (rotation and align
/// don't apply).
///
/// Args:
///     major_diameter: Basic major diameter of the thread (mm).
///     pitch: Length of 360° of thread rotation (mm).
///     length: End-to-end length of the thread (mm).
///     external .. align: see WhitworthThreadOptions.
///
/// Raises:
///     std::invalid_argument: hand must be one of "right" or "left"; end_finishes invalid;
///         crest_truncation must be at least 0 and less than h.
///     std::runtime_error: Unable to create the truncated Whitworth profile.
class WhitworthThread : public build123d::BasePartObject {
public:
    /// Included profile angle of 55°.
    static constexpr double thread_angle = 55.0;

    WhitworthThread(double major_diameter, double pitch, double length, bool external = true,
                    const std::string& hand = "right", const EndFinishes& end_finishes = {"fade", "square"},
                    double interference = 0.2, bool simple = false, double crest_truncation = 0.0,
                    double manufacturing_compensation = 0.0, const Rotation& rotation = Rotation(),
                    std::optional<Align3> align = std::nullopt);
    WhitworthThread(double major_diameter, double pitch, double length, const WhitworthThreadOptions& options);

    double major_diameter = 0.0;
    double pitch = 0.0;
    double length = 0.0;
    bool external = true;
    std::string hand;
    EndFinishes end_finishes;
    double interference = 0.0;
    bool simple = false;
    double manufacturing_compensation = 0.0;
    double crest_truncation = 0.0;
    /// Height H of the fundamental triangle (0.960491 pitch).
    double fundamental_triangle_height = 0.0;
    /// Radial height h of the full-form profile (0.640327 pitch).
    double thread_height = 0.0;
    /// Radius r of the rounded crests and roots (0.137329 pitch).
    double rounding_radius = 0.0;
    double minor_diameter = 0.0;
    double apex_radius = 0.0;
    double root_radius = 0.0;
    /// Resulting external major or internal minor diameter.
    double crest_diameter = 0.0;
    double tooth_height = 0.0;
    /// See Thread::thread_loops; nullopt for a simple thread.
    std::optional<double> thread_loops;
    Face thread_profile;

protected:
    WhitworthThread() = default;
    void init(double major_diameter, double pitch, double length, const WhitworthThreadOptions& options);
};

/// The keyword arguments of BSPPThread.
struct BSPPThreadOptions {
    /// External or internal thread selector. Defaults to true.
    bool external = true;
    /// External pitch-diameter tolerance class "A" or "B". Defaults to "A" for external threads
    /// and must be omitted (nullopt) for internal threads.
    std::optional<std::string> tolerance_class = std::nullopt;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Profile of the two ends. Defaults to ("fade", "square").
    EndFinishes end_finishes = {"fade", "square"};
    /// Radial overlap with the mating cylindrical core. Defaults to 0.2.
    double interference = 0.2;
    /// Calculate the profile without creating the swept thread. Defaults to false.
    bool simple = false;
    /// Selected external major or internal minor diameter (mm), within the ISO limits.
    /// Defaults to the basic full-form diameter.
    std::optional<double> crest_diameter = std::nullopt;
    /// Radial allowance for vertically printed FDM threads. Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    Rotation rotation = Rotation();
    std::optional<Align3> align = std::nullopt;
};

/// ISO 228-1 G-series British Standard Pipe Parallel thread.
///
/// By default the generated solid uses the nominal full-form Whitworth profile. Supplying a
/// crest diameter within the applicable ISO limits creates the corresponding truncated profile
/// before the thread is swept. Pitch-diameter limits remain independent metadata.
///
/// Args:
///     size: G-series designation, such as "G1/2". The forms "G 1/2" and "1/2" are also
///         accepted (see sizes()).
///     length: End-to-end length of the thread (mm).
///     external .. align: see BSPPThreadOptions.
///
/// Raises:
///     std::invalid_argument: size invalid; tolerance_class must be one of "A" or "B";
///         internal BSPP threads have one tolerance class; crest_diameter must be within the
///         limits; and those of WhitworthThread.
class BSPPThread : public WhitworthThread {
public:
    /// The canonical ISO 228 G-series designations ("G1/16" ... "G6").
    static std::vector<std::string> sizes();
    /// Normalize common G-series designation spellings to a table key ("g 1/2\"" -> "1/2").
    static std::string _normalize_size(const std::string& size);

    BSPPThread(const std::string& size, double length, bool external = true,
               std::optional<std::string> tolerance_class = std::nullopt, const std::string& hand = "right",
               const EndFinishes& end_finishes = {"fade", "square"}, double interference = 0.2,
               bool simple = false, std::optional<double> crest_diameter = std::nullopt,
               double manufacturing_compensation = 0.0, const Rotation& rotation = Rotation(),
               std::optional<Align3> align = std::nullopt);
    BSPPThread(const std::string& size, double length, const BSPPThreadOptions& options);

    /// The size, e.g. "G1/2".
    std::string thread_size;
    /// Number of threads in 25.4 mm.
    int tpi = 0;
    /// "A" or "B" for external threads, nullopt for internal ones.
    std::optional<std::string> tolerance_class;
    /// "ISO 228-1:1994".
    std::string standard;
    double basic_major_diameter = 0.0;
    double basic_pitch_diameter = 0.0;
    double basic_minor_diameter = 0.0;
    /// ISO designation including the external class and left hand, e.g. "G1/2A LH".
    std::string designation;
    /// Minimum and maximum pitch diameters.
    std::pair<double, double> pitch_diameter_limits;
    /// External major-diameter limits, otherwise nullopt.
    std::optional<std::pair<double, double>> major_diameter_limits;
    /// Internal minor-diameter limits, otherwise nullopt.
    std::optional<std::pair<double, double>> minor_diameter_limits;
};

// ---------------------------------------------------------------------------
// IsoThread
// ---------------------------------------------------------------------------

/// The keyword arguments of IsoThread.
struct IsoThreadOptions {
    /// External or internal thread selector. Defaults to true.
    bool external = true;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Profile of each end, "raw", "fade", "square" or "chamfer". Defaults to
    /// ("fade", "square").
    EndFinishes end_finishes = {"fade", "square"};
    /// Amount the thread will overlap with nut or bolt core. Defaults to 0.2.
    double interference = 0.2;
    /// Stop at the thread calculation, don't create the thread. Defaults to false.
    bool simple = false;
    /// Radial allowance for vertically printed FDM threads. A positive value moves an external
    /// thread inward or an internal thread outward without changing its pitch or profile.
    /// Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    /// Object rotation. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Object alignment. Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// ISO Standard Thread.
///
/// Both external and internal ISO standard 60° threads as shown in the diagram of
/// https://en.wikipedia.org/wiki/ISO_metric_screw_thread. An internal thread with a chamfered
/// end is what a nut takes. The thread is a compound of its solids (rotation and align apply).
///
/// Args:
///     major_diameter: Primary thread diameter (mm).
///     pitch: Length of 360° of thread rotation (mm).
///     length: End to end length of the thread (mm).
///     external .. align: see IsoThreadOptions.
///
/// Raises:
///     std::invalid_argument: if hand not in ["right", "left"]; end_finishes not in
///         ["raw", "square", "fade", "chamfer"].
class IsoThread : public build123d::BasePartObject {
public:
    IsoThread(double major_diameter, double pitch, double length, bool external = true,
              const std::string& hand = "right", const EndFinishes& end_finishes = {"fade", "square"},
              double interference = 0.2, bool simple = false, double manufacturing_compensation = 0.0,
              const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
    IsoThread(double major_diameter, double pitch, double length, const IsoThreadOptions& options);

    /// Value of `h` as shown in the thread diagram: (pitch / 2) / tan(thread_angle / 2).
    double h_parameter() const;
    /// The radius of the root of the thread (the inside radius of the thread diagram).
    double min_radius() const;

    double major_diameter = 0.0;
    double pitch = 0.0;
    double length = 0.0;
    bool external = true;
    /// 60 degrees.
    double thread_angle = 60;
    std::string hand;
    EndFinishes end_finishes;
    double interference = 0.0;
    bool simple = false;
    double manufacturing_compensation = 0.0;
    double apex_radius = 0.0;
    double root_radius = 0.0;
    /// The thread profile (an empty Face for a simple thread, which has none).
    Face thread_profile;
};

// ---------------------------------------------------------------------------
// TrapezoidalThread, AcmeThread, MetricTrapezoidalThread
// ---------------------------------------------------------------------------

/// The keyword arguments of TrapezoidalThread.
struct TrapezoidalThreadOptions {
    /// External or internal thread selector. Defaults to true.
    bool external = true;
    /// The number of thread starts, which indicates how many threads run parallel to each
    /// other along the screw. Defaults to 1.
    int starts = 1;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Profile of each end. Defaults to ("fade", "fade").
    EndFinishes end_finishes = {"fade", "fade"};
    /// Amount the thread will overlap with nut or bolt core. Defaults to 0.2.
    double interference = 0.2;
    /// Radial allowance for vertically printed FDM threads. Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    /// Object rotation (doesn't apply, see Thread). Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Object alignment (doesn't apply, see Thread). Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// Trapezoidal Thread Base Class.
///
/// Trapezoidal thread forms are screw thread profiles with trapezoidal outlines. They are the
/// most common forms used for leadscrews (power screws). They offer high strength and ease of
/// manufacture. They are typically found where large loads are required, as in a vise or the
/// leadscrew of a lathe.
///
/// Trapezoidal Thread is a base class for Metric and Acme derived classes, or can be used to
/// create a trapezoidal thread with arbitrary properties. Its children are the threads of each
/// start (rotation and align don't apply, as in bd_warehouse).
///
/// Args:
///     diameter: thread diameter (mm).
///     pitch: thread pitch - for threads with multiple starts, pitch is the distance between
///         adjacent threads along the axis of the screw (mm).
///     thread_angle: thread angle in degrees.
///     length: thread length (mm).
///     external .. align: see TrapezoidalThreadOptions.
///
/// Raises:
///     std::invalid_argument: hand must be one of "right" or "left"; end_finishes invalid.
class TrapezoidalThread : public build123d::BasePartObject {
public:
    TrapezoidalThread(double diameter, double pitch, double thread_angle, double length, bool external = true,
                      int starts = 1, const std::string& hand = "right",
                      const EndFinishes& end_finishes = {"fade", "fade"}, double interference = 0.2,
                      double manufacturing_compensation = 0.0, const Rotation& rotation = Rotation(),
                      std::optional<Align3> align = std::nullopt);
    TrapezoidalThread(double diameter, double pitch, double thread_angle, double length,
                      const TrapezoidalThreadOptions& options);

    /// The diameter (Python's thread_size of a TrapezoidalThread).
    double thread_size = 0.0;
    bool external = true;
    double length = 0.0;
    double diameter = 0.0;
    /// thread angle in degrees.
    double thread_angle = 0.0;
    double pitch = 0.0;
    /// pitch * starts.
    double lead = 0.0;
    double apex_radius = 0.0;
    double root_radius = 0.0;
    std::string hand;
    EndFinishes end_finishes;
    double interference = 0.0;
    double manufacturing_compensation = 0.0;
    Face thread_profile;

protected:
    TrapezoidalThread() = default;
    void init(double diameter, double pitch, double thread_angle, double length,
              const TrapezoidalThreadOptions& options);
};

/// The keyword arguments of AcmeThread and MetricTrapezoidalThread.
struct AcmeThreadOptions {
    /// External or internal thread selector. Defaults to true.
    bool external = true;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Profile of each end. Defaults to ("fade", "fade").
    EndFinishes end_finishes = {"fade", "fade"};
    /// Amount the thread will overlap with nut or bolt core. Defaults to 0.2.
    double interference = 0.2;
    /// Radial allowance for vertically printed FDM threads. Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    Rotation rotation = Rotation();
    std::optional<Align3> align = std::nullopt;
};
using MetricTrapezoidalThreadOptions = AcmeThreadOptions;

/// ACME Thread.
///
/// The original trapezoidal thread form, and still probably the one most commonly encountered
/// worldwide, with a 29° thread angle, is the Acme thread form.
///
/// Args:
///     size: size as a string (i.e. "3/4" or "1 1/4"), one of sizes().
///     length: thread length (mm).
///     external .. align: see AcmeThreadOptions.
///
/// Raises:
///     std::invalid_argument: Invalid screw size; hand must be one of "right" or "left";
///         end_finishes invalid.
class AcmeThread : public TrapezoidalThread {
public:
    /// Pitch (mm) of each size.
    static const Dict<std::string, double> acme_pitch;
    /// 29 degrees.
    static constexpr double thread_angle = 29.0;
    /// The thread sizes ("1/4" ... "3").
    static std::vector<std::string> sizes();

    AcmeThread(const std::string& size, double length, bool external = true, const std::string& hand = "right",
               const EndFinishes& end_finishes = {"fade", "fade"}, double interference = 0.2,
               double manufacturing_compensation = 0.0, const Rotation& rotation = Rotation(),
               std::optional<Align3> align = std::nullopt);
    AcmeThread(const std::string& size, double length, const AcmeThreadOptions& options);
};

/// Metric Trapezoidal Thread.
///
/// The ISO 2904 standard metric trapezoidal thread with a thread angle of 30°.
///
/// Args:
///     size: specified as a string with diameter x pitch in mm (i.e. "8x1.5"), one of sizes().
///     length: End to end length of the thread (mm).
///     external .. align: see AcmeThreadOptions.
///
/// Raises:
///     std::invalid_argument: size invalid; hand must be one of "right" or "left";
///         end_finishes invalid.
class MetricTrapezoidalThread : public TrapezoidalThread {
public:
    /// The ISO 2904 sizes ("8x1.5", "9x1.5", ... "315x5").
    static const std::vector<std::string> standard_sizes;
    /// 30 degrees.
    static constexpr double thread_angle = 30.0;
    /// The thread sizes (standard_sizes).
    static std::vector<std::string> sizes();

    MetricTrapezoidalThread(const std::string& size, double length, bool external = true,
                            const std::string& hand = "right", const EndFinishes& end_finishes = {"fade", "fade"},
                            double interference = 0.2, double manufacturing_compensation = 0.0,
                            const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
    MetricTrapezoidalThread(const std::string& size, double length, const MetricTrapezoidalThreadOptions& options);
};

// ---------------------------------------------------------------------------
// PlasticBottleThread
// ---------------------------------------------------------------------------

/// A bottle thread standard that fits a set of measurements (PlasticBottleThread::identify).
struct PlasticBottleThreadMatch {
    /// The `size` to give PlasticBottleThread.
    std::string size;
    /// The `bottle_type` to give PlasticBottleThread.
    std::string bottle_type;
    /// Smallest and largest diameter over the thread crests, ASTM D2911's T (mm).
    std::pair<double, double> major_diameter;
    /// Smallest and largest diameter of the neck between the threads, ASTM D2911's E (mm).
    std::pair<double, double> neck_diameter;
    /// Distance from one thread to the next (mm).
    double pitch = 0.0;
    /// Fewest turns of thread the finish allows, nullopt where the standard fixes a thread
    /// length instead.
    std::optional<double> min_turns;
    /// How far the measurements fall outside the standard, in mm.
    double error = 0.0;

    bool operator==(const PlasticBottleThreadMatch& o) const {
        return size == o.size && bottle_type == o.bottle_type && major_diameter == o.major_diameter &&
               neck_diameter == o.neck_diameter && pitch == o.pitch && min_turns == o.min_turns && error == o.error;
    }
};

/// The keyword arguments of PlasticBottleThread.
struct PlasticBottleThreadOptions {
    /// External or internal thread selector. Defaults to true.
    bool external = true;
    /// Twist direction, "right" or "left". Defaults to "right".
    std::string hand = "right";
    /// Amount the thread will overlap with nut or bolt core. Defaults to 0.2.
    double interference = 0.2;
    /// Bottle thread standard, "astm_d2911" or "pco1881". Defaults to "astm_d2911".
    std::string bottle_type = "astm_d2911";
    /// Radial allowance for vertically printed FDM threads. A value of 0.2 mm moves an external
    /// thread inward by 0.2 mm or an internal thread outward by 0.2 mm. Defaults to 0.0.
    double manufacturing_compensation = 0.0;
    /// Object rotation (doesn't apply, see Thread). Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Object alignment (doesn't apply, see Thread). Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// Plastic bottle thread.
///
/// ASTM D2911 is selected by default. PCO1881 is selected with bottle_type="pco1881". The
/// PCO1881 external profile uses the standard bottle-finish diameters, while its internal
/// profile uses a separate empirically fitted closure minor diameter.
///
/// L Style: All-Purpose Thread - trapezoidal shape with 30° shoulders, metal or plastic closures.
/// M Style: Modified Buttress Thread - asymmetric shape with 10° and 40/45/50° shoulders,
/// plastic closures.
///
/// Args:
///     size: as defined by the ASTM is specified as
///         [L|M][diameter(mm)]SP[100|103|110|200|400|410|415|425|444] for ASTM D2911, or "28"
///         for PCO1881 (see sizes()).
///     external .. align: see PlasticBottleThreadOptions.
///
/// Raises:
///     std::invalid_argument: bottle_type must be one of ("astm_d2911", "pco1881"); hand must be
///         one of "right" or "left"; size invalid; finish invalid; diameter invalid.
///
/// Example:
///     PlasticBottleThread thread("M38SP444", false, "right", 0.2, "astm_d2911", 0.2 * MM);
class PlasticBottleThread : public build123d::BasePartObject {
public:
    /// The supported bottle thread standards: ("astm_d2911", "pco1881").
    static std::vector<std::string> bottle_types();
    /// Valid sizes for a bottle thread standard ("astm_d2911" or "pco1881"); std::invalid_argument
    /// for another.
    static std::vector<std::string> sizes(const std::string& bottle_type = "astm_d2911");
    /// The standard threads that fit measurements taken off a bottle.
    ///
    /// A finish is named for the diameter over its thread crests, which is not the diameter of
    /// the neck: a neck of 30 is a 33 finish. Measure either, or both, and as much of the rest
    /// as can be had. The distance between threads separates the coarse sizes from the fine,
    /// and the number of turns separates the finishes of one size: a bottle has at least the
    /// turns its finish requires and usually a little more.
    ///
    /// These are single start threads, so check that only one thread begins at the top of the
    /// neck; on a multiple start thread the distance between neighbouring threads is not the
    /// pitch. Finishes of one size differ mostly in the height of the neck, which is not
    /// tabulated here, so those the turns allow all come back, the finish made in the most
    /// sizes first as the likeliest to be met.
    ///
    /// Args:
    ///     neck_diameter: diameter of the neck between the threads (mm).
    ///     major_diameter: diameter over the thread crests (mm).
    ///     pitch: distance from one thread to the next (mm).
    ///     tpi: threads per inch, in place of `pitch`.
    ///     turns: times the thread wraps the neck.
    ///     style: ASTM D2911 thread style the sizes are given in, "L" or "M". Defaults to "M".
    ///     tolerance: how far outside a standard a measured diameter or pitch may fall and
    ///         still fit (mm). Defaults to 0.5.
    ///
    /// Raises:
    ///     std::invalid_argument: neither diameter is given, both pitch and tpi are, or style
    ///         is invalid.
    ///
    /// Returns: what fits, best first. Example:
    ///     auto best = PlasticBottleThread::identify(30, std::nullopt, 4, std::nullopt, 1.5)[0];
    ///     PlasticBottleThread cap_thread(best.size, false);
    static std::vector<PlasticBottleThreadMatch> identify(std::optional<double> neck_diameter = std::nullopt,
                                                          std::optional<double> major_diameter = std::nullopt,
                                                          std::optional<double> pitch = std::nullopt,
                                                          std::optional<double> tpi = std::nullopt,
                                                          std::optional<double> turns = std::nullopt,
                                                          const std::string& style = "M", double tolerance = 0.5);

    PlasticBottleThread(const std::string& size, bool external = true, const std::string& hand = "right",
                        double interference = 0.2, const std::string& bottle_type = "astm_d2911",
                        double manufacturing_compensation = 0.0, const Rotation& rotation = Rotation(),
                        std::optional<Align3> align = std::nullopt);
    PlasticBottleThread(const std::string& size, const PlasticBottleThreadOptions& options);

    std::string thread_size;
    bool external = true;
    std::string bottle_type;
    std::string hand;
    double interference = 0.0;
    double manufacturing_compensation = 0.0;
    /// "L" or "M" (ASTM D2911), nullopt for PCO1881.
    std::optional<std::string> style;
    /// The nominal diameter (ASTM D2911's size, PCO1881's bottle major diameter).
    double diameter = 0.0;
    /// The ASTM D2911 finish (e.g. 400), nullopt for PCO1881.
    std::optional<int> finish;
    double major_diameter = 0.0;
    double minor_diameter = 0.0;
    /// PCO1881 bottle major, bottle minor and closure minor diameters (nullopt for ASTM D2911).
    std::optional<double> bottle_major_diameter;
    std::optional<double> bottle_minor_diameter;
    std::optional<double> closure_minor_diameter;
    /// Threads per inch.
    double tpi = 0.0;
    double root_width = 0.0;
    double apex_width = 0.0;
    double apex_offset = 0.0;
    double apex_radius = 0.0;
    double root_radius = 0.0;
    double pitch = 0.0;
    int starts = 1;
    double lead = 0.0;
    double length = 0.0;
    Face thread_profile;
};

}  // namespace bd_warehouse
