// bd_warehouse-cpp: retaining rings (bd_warehouse's retaining_ring.py).
//
// DIN 471 external snap rings (for shafts), DIN 472 internal snap rings (for bores) and the
// groove a ring sits in. Sizes are the nominal shaft or bore diameter ("3" ... "300").
//
//   using namespace bd_warehouse;
//   ExternalSnapRing ring("40");                         // on a 40 mm shaft
//   Part shaft = Cylinder(20, 50) - Pos(0, 0, 10) * RetainingRingGroove(ring);
//   // ring.ring_dict.at("s") is its thickness; joints "a" (bottom) and "b" (top)
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Align3;
using build123d::Part;
using build123d::Rotation;

/// Rows of numbers by key (Python's dict of dict of float), e.g. a retaining ring table.
using RingTable = Dict<std::string, Dict<std::string, double>>;

/// Parse a metric retaining ring CSV parameter file of bd_warehouse's data directory: the rows by
/// nominal size (the first column), each a dictionary of its numbers (empty values left out) with
/// the size itself last. std::invalid_argument for a file without a header or a value that isn't
/// a number.
RingTable read_retaining_ring_parameters_from_csv(const std::string& filename);

/// Base class for standardized retaining rings.
///
/// The ring lies on z=0 (its axis is Z) with RigidJoints "a" (bottom) and "b" (top) at the
/// centre of the lug holes' arcs, silver coloured.
///
/// Args:
///     size: Nominal shaft or bore diameter as listed in the applicable standard.
///     rotation: Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
///     align: Align MIN, CENTER, or MAX of object. Defaults to nullopt, which places the minimum
///         Z face on the placement plane while remaining centered in XY.
///
/// Raises:
///     std::invalid_argument: Invalid retaining ring size.
class RetainingRing : public build123d::BasePartObject {
public:
    /// Return retaining ring classes (by name) and standards available in the given size.
    /// (ExternalSnapRing::select_by_size and InternalSnapRing::select_by_size look at their own
    /// class only, as in bd_warehouse.)
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    /// Return identifying information for this retaining ring, e.g. "ExternalSnapRing(DIN471): 40".
    std::string info() const;
    /// Create the retaining ring CAD object.
    virtual Part make_ring() const = 0;

    /// The standard of the ring's class ("DIN471", "DIN472").
    std::string standard;
    /// The nominal size, stripped, e.g. "40".
    std::string ring_size;
    /// The ring's dimensions from the table (d3, b, a, d5, s, d2, m, d4, ... and d1, the size).
    Dict<std::string, double> ring_dict;

protected:
    RetainingRing() = default;
    /// Python's RetainingRing.__init__ (called by the derived constructors).
    void init(const std::string& class_name, const std::string& class_standard, const Dict<std::string, double>& dims,
              const std::string& size, const Rotation& rotation, std::optional<Align3> align);
    std::string class_name_;
};

/// External retaining ring for shafts as defined by DIN 471.
///
/// Args: size (the shaft diameter, "3" ... "300", see sizes()), rotation, align.
class ExternalSnapRing : public RetainingRing {
public:
    /// "DIN471".
    static const std::string standard;
    /// The DIN 471 table (din471.csv).
    static const RingTable& ring_data();
    /// Return the available nominal sizes for this retaining ring class.
    static std::vector<std::string> sizes();
    /// Return the evaluated metric parameters for a nominal size (std::invalid_argument for an
    /// invalid size).
    static Dict<std::string, double> parameters(const std::string& size);
    /// {"ExternalSnapRing": ["DIN471"]} if the size is one of sizes(), else empty.
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    explicit ExternalSnapRing(const std::string& size, const Rotation& rotation = Rotation(),
                              std::optional<Align3> align = std::nullopt);

    /// Create the external retaining ring using the DIN 471 dimensions.
    ///
    /// DIN 471 specifies the critical dimensions but leaves the transitions between the ring body
    /// and lugs illustrative. This construction creates a smooth, symmetric approximation that
    /// honors d3, a, b, d5, and s.
    Part make_ring() const override;
};

/// Internal retaining ring for bores as defined by DIN 472.
///
/// Args: size (the bore diameter, "8" ... "300", see sizes()), rotation, align.
class InternalSnapRing : public RetainingRing {
public:
    /// "DIN472".
    static const std::string standard;
    /// The DIN 472 table (din472.csv).
    static const RingTable& ring_data();
    /// Return the available nominal sizes for this retaining ring class.
    static std::vector<std::string> sizes();
    /// Return the evaluated metric parameters for a nominal size (std::invalid_argument for an
    /// invalid size).
    static Dict<std::string, double> parameters(const std::string& size);
    /// {"InternalSnapRing": ["DIN472"]} if the size is one of sizes(), else empty.
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    explicit InternalSnapRing(const std::string& size, const Rotation& rotation = Rotation(),
                              std::optional<Align3> align = std::nullopt);

    /// Create the canonical DIN 472 internal retaining ring.
    ///
    /// This uses the d1 <= 300 mm Detail X lug profile for every tabulated size. DIN 472's
    /// optional manufacturer-selected lug profiles are not modeled. The resulting symmetric
    /// approximation honors d3, a, b, d5, and s.
    Part make_ring() const override;
};

/// Groove cutter for a DIN 471 or DIN 472 retaining ring.
///
/// The groove uses the nominal diameter d2 and width m specified for the supplied ring. The
/// standard's clearance diameter d4 extends the cutter beyond the nominal shaft or bore surface to
/// provide reliable Boolean overlap. Subtract it from the shaft or the bored part.
///
/// Args:
///     ring: External or internal retaining ring that defines the groove dimensions.
///     rotation: Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
///     align: Align MIN, CENTER, or MAX of the cutter. Defaults to nullopt, which places the
///         minimum Z face on the placement plane while remaining centered in XY.
class RetainingRingGroove : public build123d::BasePartObject {
public:
    explicit RetainingRingGroove(const RetainingRing& ring, const Rotation& rotation = Rotation(),
                                 std::optional<Align3> align = std::nullopt);

    /// The ring the groove is for (a copy).
    std::shared_ptr<const RetainingRing> ring;
    /// The ring's d2.
    double groove_diameter = 0.0;
    /// The ring's m.
    double groove_width = 0.0;
    /// The ring's d4.
    double clearance_diameter = 0.0;
};

}  // namespace bd_warehouse
