// bd_warehouse-cpp: shaft keys and keyways (bd_warehouse's shaft_key.py).
//
// DIN 6885-1 parallel keys (Form A with rounded ends, Form B with square ends) for shaft
// diameters of 6 to 50 mm, and the keyway cutters for the shaft and the hub with their ISO 286
// fits.
//
//   using namespace bd_warehouse;
//   ShaftKey key(20, 25);                                 // a 6 x 6 key for a 20 mm shaft
//   Part shaft = Cylinder(10, 40) - Keyway(key);          // keyseat cut into the shaft
//   Part hub = Cylinder(20, 25) - Cylinder(10, 25) - Keyway(key, "hub", "Sliding");
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Align3;
using build123d::Face;
using build123d::Rotation;
using build123d::Solid;

/// The keyword arguments of ShaftKey (Python's order and defaults).
struct ShaftKeyOptions {
    /// "A" for rounded ends or "B" for square ends. Defaults to "A".
    std::string key_form = "A";
    /// Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Align MIN, CENTER, or MAX of the key. Defaults to nullopt, which keeps the key centered in
    /// XY with its minimum Z face on the placement plane.
    std::optional<Align3> align = std::nullopt;
};

/// DIN 6885-1 parallel key for a shaft-hub connection.
///
/// Shaft keys transmit torque between a shaft and a mounted component such as a gear, pulley,
/// or coupling. Form A has two rounded ends and Form B has two square ends. DIN 6885-1 selects the
/// nominal width and height from the shaft diameter; the key length is supplied by the caller
/// because it depends on hub length and the required torque capacity. This initial dataset covers
/// the nominal shaft diameters from 6 mm through 50 mm listed in the available DIN 6885-1 extract.
///
/// The key lies along X, centered in XY, from z=0 up (grey, RigidJoint "a" at the origin).
///
/// Args:
///     shaft_diameter: Whole-number nominal shaft diameter in millimetres (an integer-valued
///         number).
///     length: Overall key length in millimetres. The supplied custom length is not validated
///         against the DIN preferred-length series.
///     key_form: "A" for rounded ends or "B" for square ends. Defaults to "A".
///     rotation: Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
///     align: Align MIN, CENTER, or MAX of the key. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: The shaft diameter, length, or key form is invalid.
class ShaftKey : public build123d::BasePartObject {
public:
    /// "DIN6885-1".
    static const std::string standard;
    /// The DIN 6885 parameters (b, h, t2, t2_tol+, t4, t4_tol+) by shaft diameter.
    static const Dict<int, ParamDict>& key_data();
    /// Return the supported nominal shaft diameters.
    static std::vector<int> sizes();
    /// Return the nominal key and keyway parameters for a shaft diameter (std::invalid_argument
    /// for an invalid one).
    static ParamDict parameters(double shaft_diameter);
    /// Return the shaft-key class and standard available for a diameter: {"ShaftKey":
    /// ["DIN6885-1"]}, or empty.
    static Dict<std::string, std::vector<std::string>> select_by_size(double shaft_diameter);
    /// The shaft diameter as a whole number; std::invalid_argument "shaft_diameter must be a
    /// whole number of millimetres" or "... must be greater than zero".
    static int _validate_shaft_diameter(double shaft_diameter);

    ShaftKey(double shaft_diameter, double length, const std::string& key_form = "A",
             const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
    ShaftKey(double shaft_diameter, double length, const ShaftKeyOptions& options);

    /// Return identifying information for this shaft key, e.g. "ShaftKey(DIN6885-1, Form A): 12x20".
    std::string info() const;
    /// Return the nominal area of the key plan.
    double plan_area() const;
    /// Create the parallel-key solid with its minimum Z face at zero.
    Solid make_key() const;

    /// "A" or "B".
    std::string key_form;
    /// The nominal shaft diameter (mm).
    int shaft_diameter = 0;
    /// The key's parameters (see key_data).
    ParamDict key_dict;
    /// The key length (mm).
    double length = 0.0;
    /// The key width b (mm).
    double width = 0.0;
    /// The key height h (mm).
    double key_height = 0.0;
};

/// The keyword arguments of Keyway (Python's order and defaults).
struct KeywayOptions {
    /// "shaft" for a shaft keyseat or "hub" for a hub keyway. Defaults to "shaft".
    std::string keyway_type = "shaft";
    /// DIN 6885-1 fit selection: "Tight", "Loose", or "Sliding". Defaults to "Loose".
    std::string fit = "Loose";
    /// Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
    Rotation rotation = Rotation();
    /// Align MIN, CENTER, or MAX of the cutter. Defaults to nullopt, which retains the
    /// shaft-centered placement.
    std::optional<Align3> align = std::nullopt;
};

/// Nominal keyway cutter for a DIN 6885-1 parallel key.
///
/// `fit` selects the DIN 6885-1 tolerance class for the shaft or hub keyway. ISO 286-1 limit
/// deviations determine min_width and max_width; the cutter is modeled at their midpoint,
/// available as width. By default the cutter is positioned for a shaft or bore centered on
/// Axis.Z: its length is centered on Z, its width is centered on Y, and its radial face is located
/// at the nominal shaft radius. Shaft keyways extend inward along negative X; hub keyways extend
/// outward along positive X. Subtract it from the shaft or hub.
///
/// Args:
///     key: Parallel key defining the nominal length, width, and end form.
///     keyway_type: "shaft" for a shaft keyseat or "hub" for a hub keyway. Defaults to "shaft".
///     fit: DIN 6885-1 fit selection: "Tight", "Loose", or "Sliding". Defaults to "Loose".
///     rotation: Sequence of angles about the X, Y, and Z axes. Defaults to no rotation.
///     align: Align MIN, CENTER, or MAX of the cutter. Defaults to nullopt.
///
/// Raises:
///     std::invalid_argument: The keyway type or fit is invalid.
class Keyway : public build123d::BasePartObject {
public:
    /// The ISO 286 tolerance class of each fit, for the hub and the shaft:
    /// Tight: P9/P9, Loose: JS9/N9, Sliding: D10/H9.
    static const Dict<std::string, Dict<std::string, std::string>>& keyway_fits();

    explicit Keyway(const ShaftKey& key, const std::string& keyway_type = "shaft", const std::string& fit = "Loose",
                    const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
    Keyway(const ShaftKey& key, const KeywayOptions& options);

    /// Return the transverse profile of a broached hub keyway.
    ///
    /// The profile lies on the XY plane and extends from the shaft axis at X=0 through the bore to
    /// the bottom of the keyway at the nominal shaft radius plus the keyway depth. Its width is
    /// centered on the X axis. Extrude the profile along the bore axis without applying a radial
    /// offset.
    ///
    /// Raises:
    ///     std::invalid_argument: If this is not a hub keyway.
    Face broach_profile() const;

    /// The key (a copy).
    ShaftKey key;
    /// "shaft" or "hub".
    std::string keyway_type;
    /// "Tight", "Loose" or "Sliding".
    std::string fit;
    /// The key's width.
    double nominal_width = 0.0;
    /// The key's length.
    double length = 0.0;
    /// t4 (shaft) or t2 (hub).
    double keyway_depth = 0.0;
    /// t4_tol+ (shaft) or t2_tol+ (hub).
    double keyway_depth_tolerance = 0.0;
    /// The ISO 286 tolerance class, e.g. "N9".
    std::string width_tolerance;
    /// The limits of the keyway width and the width modeled (their midpoint).
    double min_width = 0.0;
    double max_width = 0.0;
    double width = 0.0;
};

}  // namespace bd_warehouse
