// bd_warehouse-cpp: O-rings and their glands (bd_warehouse's o_rings.py).
//
// ORing makes the nominal ISO 3601-1 Class A O-ring of a size code (a torus about Z) and the
// cross-section of the gland (groove) that holds it, for static (axial or radial squeeze) and
// dynamic seals. Class functions find sizes by inner diameter or by installed length.
//
//   using namespace bd_warehouse;
//   ORing ring("025");                                   // ring.inner_diameter, ring.width
//   Sketch gland = ring.static_gland_profile();          // opening on the local X axis
//   auto matches = ORing::select_by_length(101.42);       // {"1.78": {"025", "026"}, ...}
#pragma once

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Align3;
using build123d::Rotation;
using build123d::Sketch;

/// Rows of numbers by key, a number missing ("-" in the file) being nullopt: what
/// read_parameters_from_csv returns (Python's dict of dict of float | None).
using NumberTable = Dict<std::string, Dict<std::string, std::optional<double>>>;

/// O-ring size codes near a target, by cross-section width (str of the width, e.g. "1.78"): the
/// nearest codes below and above as a (lower, upper) pair, either nullopt when it isn't within
/// the tolerance, or a one-code tuple for an exact match (Python's dict of tuple of str | None).
using ORingNeighbors = Dict<std::string, std::vector<std::optional<std::string>>>;

/// Parse a metric O-ring gland CSV parameter file of bd_warehouse's data directory: the rows
/// by the first column's value, each a dictionary of the other columns as numbers ("-" gives
/// nullopt). std::invalid_argument "No header found in <filename>" for a file without a header
/// or a value that isn't a number.
NumberTable read_parameters_from_csv(const std::string& filename);

/// The keyword arguments of ORing (Python's order and defaults).
struct ORingOptions {
    /// Dimensional standard used to interpret `size`. Defaults to "iso3601".
    std::string o_ring_type = "iso3601";
    /// Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Align MIN, CENTER, or MAX of the O-ring. Defaults to nullopt.
    std::optional<Align3> align = std::nullopt;
};

/// Parametric O-ring for static and dynamic sealing applications.
///
/// O-rings are toroidal elastomeric seals installed in a gland and compressed between mating
/// components to prevent fluid or gas leakage. This class creates nominal general-industrial
/// Class A geometry from ISO 3601-1 and provides static_gland_profile and dynamic_gland_profile
/// helpers for designing a corresponding gland. The gland dimensions are initial design
/// recommendations from the Apple Rubber Seal Design Guide; they have not been verified against
/// ISO 3601-2.
///
/// The O-ring is a torus about Z centred on the origin, dark grey, with a RigidJoint "a" at its
/// centre.
///
/// Args:
///     size: Three-digit O-ring size code, such as "025". A leading hyphen and omitted leading
///         zeroes are accepted and normalized.
///     o_ring_type: Dimensional standard used to interpret `size`. Defaults to "iso3601".
///     rotation: Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
///     align: Align MIN, CENTER, or MAX of the O-ring. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: `size` or `o_ring_type` is invalid.
class ORing : public build123d::BasePartObject {
public:
    /// The O-ring table as read (o-ring_parameters.csv).
    static const StringTable& o_ring_data();
    /// The evaluated parameters (id, w, id_tol, w_tol in mm) by standard and size code.
    static const Dict<std::string, ParamTable>& o_ring_parameters();
    /// The size codes of each standard grouped by cross-section width: (inner diameter, size
    /// code) pairs sorted by inner diameter.
    static const Dict<std::string, Dict<double, std::vector<std::pair<double, std::string>>>>& o_ring_width_families();
    /// The static gland table by cross-section ("1.78", ...) (o-ring_static_gland_parameters.csv).
    static const NumberTable& o_ring_static_gland_data();
    /// The dynamic gland table by cross-section (o-ring_dynamic_gland_parameters.csv).
    static const NumberTable& o_ring_dynamic_gland_data();

    /// Normalize an O-ring size code ("-25", " 25 " -> "025"); std::invalid_argument
    /// "'...' invalid, size must be a three-digit code" otherwise.
    static std::string _normalize_size(const std::string& size);
    /// Return the available O-ring standards.
    static std::set<std::string> types();
    /// Return the available size codes for an O-ring standard.
    static std::vector<std::string> sizes(const std::string& o_ring_type = "iso3601");
    /// Return the evaluated metric parameters for a size and standard.
    static ParamDict parameters(const std::string& size, const std::string& o_ring_type = "iso3601");
    /// Return the standards that provide the given size code: {"ORing": [standards]} (empty if
    /// none).
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);
    /// Find closest matches to the given inner_diameter within tolerance. The results are by
    /// O-ring width with the O-ring size codes in (lower, upper) pairs; should there be an exact
    /// match, a (match,) tuple is generated.
    ///
    /// Args:
    ///     inner_diameter: inner diameter in mm.
    ///     tolerance: ± fractional deviation from target. Defaults to 0.05.
    ///     o_ring_type: Dimensional standard. Defaults to "iso3601".
    ///
    /// Example: ORing::select_by_inner_diameter(650) gives
    ///     {"5.33": {"394", "395"}, "6.99": {"474", "475"}}.
    static ORingNeighbors select_by_inner_diameter(double inner_diameter, double tolerance = 0.05,
                                                   const std::string& o_ring_type = "iso3601");
    /// Find O-rings for a target installed centre-line length.
    ///
    /// For each cross-section family, the result contains the nearest free O-ring lengths below
    /// and above `length` as a (lower, upper) pair. A lower candidate will be stretched when
    /// installed on the target path, while an upper candidate will be circumferentially
    /// compressed. An exact free-length match is returned as a one-item tuple.
    ///
    /// For typical piston or male-gland applications, use about 2% installed stretch as a design
    /// target. The usual range is 1% to 5%, with 5% treated as an upper limit rather than a
    /// target. Stretch above 5% can accelerate ageing and reduce the O-ring cross-section and
    /// resulting seal compression. Rotary shaft seals generally require no installed stretch.
    /// The installed stretch for a candidate is: stretch = length / candidate.length - 1. Gland
    /// squeeze calculations should account for the reduction in cross-section caused by
    /// stretching.
    ///
    /// Args:
    ///     length: Target installed centre-line length in millimetres.
    ///     tolerance: Maximum fractional difference between the free and installed lengths. The
    ///         default 0.05 searches within 5% of the target.
    ///     o_ring_type: Dimensional standard used for selection. Defaults to "iso3601".
    ///
    /// Example: ORing::select_by_length(101.42).at("1.78") is {"025", "026"};
    ///     101.42 / ORing("025").length - 1 is about 0.02.
    static ORingNeighbors select_by_length(double length, double tolerance = 0.05,
                                           const std::string& o_ring_type = "iso3601");
    /// Return the nominal cross-section widths for an O-ring standard (sorted).
    static std::vector<double> nominal_widths(const std::string& o_ring_type = "iso3601");
    /// Return the recommended gland width for an O-ring cross-section.
    ///
    /// Args:
    ///     o_ring_width: Nominal O-ring cross-section diameter in millimetres.
    ///     application: "static" or "dynamic" gland design. Defaults to "static".
    ///     number_backing_rings: Number of anti-extrusion backing rings accommodated by the
    ///         gland, 0, 1 or 2. Defaults to 0.
    ///
    /// Raises:
    ///     std::invalid_argument: The application, backing-ring count, or O-ring cross-section is
    ///         invalid or unsupported.
    static double gland_width_for(double o_ring_width, const std::string& application = "static",
                                  int number_backing_rings = 0);

    explicit ORing(const std::string& size, const std::string& o_ring_type = "iso3601",
                   const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
    ORing(const std::string& size, const ORingOptions& options);

    /// Return identifying information for this O-ring, e.g. "ORing(iso3601): 025".
    std::string info() const;

    /// Create the recommended profile for a static O-ring gland (and set the gland_*
    /// attributes).
    ///
    /// Args:
    ///     axial: Select axial squeeze when true or radial squeeze when false. Defaults to true.
    ///     number_backing_rings: Number of anti-extrusion backing rings accommodated by the
    ///         gland. Defaults to 0.
    ///
    /// Returns: The gland cross-section with its opening on the local X axis.
    ///
    /// Raises:
    ///     std::invalid_argument: The backing-ring count is invalid or unsupported for the O-ring
    ///         cross-section.
    Sketch static_gland_profile(bool axial = true, int number_backing_rings = 0);
    /// Create the recommended profile for a dynamic radial O-ring gland (and set the gland_*
    /// attributes).
    ///
    /// The dimensions are intended as initial design guidance for reciprocating or oscillating
    /// seals. Dynamic seal performance also depends on material, lubrication, surface finish,
    /// pressure, temperature, stretch, and friction.
    ///
    /// Args:
    ///     number_backing_rings: Number of anti-extrusion backing rings accommodated by the
    ///         gland. Defaults to 0.
    ///
    /// Returns: The gland cross-section with its opening on the local X axis.
    Sketch dynamic_gland_profile(int number_backing_rings = 0);

    /// The normalized size code, e.g. "025".
    std::string o_ring_size;
    /// The standard, e.g. "iso3601".
    std::string o_ring_type;
    /// ISO 3601 size code (o_ring_size).
    std::string dash_number;
    /// inner diameter (mm).
    double inner_diameter = 0.0;
    double inner_diameter_tolerance = 0.0;
    /// cross-section diameter (mm).
    double width = 0.0;
    double width_tolerance = 0.0;
    /// torus major radius: (inner_diameter + width) / 2.
    double major_radius = 0.0;
    /// torus minor radius: width / 2.
    double minor_radius = 0.0;
    /// center line length: 2 pi major_radius.
    double length = 0.0;
    /// The last gland profile's width, depth, radius and their tolerances (nullopt until a gland
    /// profile is made; gland_width_tol is 0.13).
    std::optional<double> gland_width;
    double gland_width_tol = 0.13;
    std::optional<double> gland_depth;
    std::optional<double> gland_depth_tol;
    std::optional<double> gland_radius;
    std::optional<double> gland_radius_tol;
    /// The last gland profile made.
    std::optional<Sketch> gland_profile;

private:
    static ORingNeighbors _find_neighbors(double target, double tolerance, const std::string& o_ring_type,
                                          bool by_diameter = true);
    const Dict<std::string, std::optional<double>>& _gland_data_for_width(const NumberTable& gland_table,
                                                                          const std::string& application) const;
    Sketch _make_gland_profile(const Dict<std::string, std::optional<double>>& gland_data,
                               const std::string& depth_min_key, const std::string& depth_max_key,
                               int number_backing_rings);
};

}  // namespace bd_warehouse
