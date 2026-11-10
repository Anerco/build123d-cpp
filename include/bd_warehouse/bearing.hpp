// bd_warehouse-cpp: parametric bearings (bd_warehouse's bearing.py).
//
// Ball and roller bearings from bd_warehouse's parameter tables (SKF catalogue data): deep
// groove ball bearings (open and capped), angular contact ball bearings, cylindrical roller
// bearings and tapered roller bearings, and PressFitHole, the seat and axle hole that hold one.
//
// A bearing is an assembly Part: its children are the "OuterRace", the "InnerRace", the
// "Roller"s (or the two "Cap"s of a capped bearing) and the "Cage" where the bearing has one,
// each with its material. The bearing sits on z=0 (its axis is Z) with RigidJoints "a" at its
// bottom and "b" at its top. Sizes are "M<bore>-<outer diameter>-<width>" (e.g. "M8-22-7").
//
//   using namespace bd_warehouse;
//   SingleRowDeepGrooveBallBearing bearing("M8-22-7");
//   Part plate = Box(40, 40, 10, Rotation(), Align3(Align::CENTER, Align::CENTER, Align::MAX));
//   plate -= PressFitHole(bearing, 0, "Normal", 10);  // the seat and an axle hole through the plate
//   // bearing.bore_diameter(), bearing.outer_diameter(), bearing.thickness()
#pragma once

#include <optional>
#include <set>
#include <string>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Compound;
using build123d::Edge;
using build123d::Face;
using build123d::Location;
using build123d::Solid;

/// Parametric Bearing: the base class used to create standard bearings.
///
/// Args:
///     size: bearing size, e.g. "M8-22-7" (bore, outer diameter and width in mm).
///     bearing_type: type identifier - e.g. "SKT". Defaults to "SKT".
///
/// Raises:
///     std::invalid_argument: bearing_type is invalid; size is invalid.
class Bearing : public build123d::BasePartObject {
public:
    /// Clearance hole drill sizes by size (without the pitch) and fit ("Close", "Normal",
    /// "Loose") (clearance_hole_sizes.csv).
    static const StringTable& clearance_hole_drill_sizes();
    /// The clearance hole diameters (mm) by size and fit.
    static const ParamTable& clearance_hole_data();
    /// Return a dictionary of list of bearing types of this size: the bearing classes (by name)
    /// that make the size, with their types (sorted; Python lists them in set order).
    static Dict<std::string, std::vector<std::string>> select_by_size(const std::string& size);

    /// Did the derived class create this method (e.g. "cap" for a capped bearing, "cage" for
    /// the bearings with a cage). Counts the methods bd_warehouse defines (see method_exists in
    /// fastener.hpp).
    bool method_exists(const std::string& method) const;
    /// A dictionary of drill diameters for clearance holes (of the bore's size), by fit.
    /// std::invalid_argument "No clearance hole data for size ..." if there are none.
    ParamDict clearance_hole_diameters() const;
    /// Diameter of central hole (the table's d).
    double bore_diameter() const;
    /// Bearing outer diameter (the table's D).
    double outer_diameter() const;
    /// Bearing thickness (T for tapered roller bearings, else B).
    double thickness() const;

    /// Each derived class must provide the section of the inner race (a face on Plane.XZ
    /// revolved about Z).
    virtual Face inner_race_section() const = 0;
    /// Each derived class must provide the section of the outer race.
    virtual Face outer_race_section() const = 0;
    /// Each derived class must provide the roller object - a sphere, cylinder or cone.
    virtual Solid roller() const = 0;
    /// Each derived class must provide the profile of a countersink cutter: the bearing's
    /// outline less the radial interference (mm).
    virtual Face countersink_profile(double interference = 0) const = 0;
    /// Each derived class must provide the roller diameter.
    virtual double roller_diameter() const = 0;
    /// Each derived class must provide the roller race center radius.
    virtual double race_center_radius() const = 0;
    /// Number of rollers around the race: int(1.8 pi race_center_radius / roller_diameter).
    virtual int roller_count() const;
    /// The caps of a capped bearing (a thin ring covering the raceway), nullopt for bearings
    /// without (Python: no `cap` method).
    virtual std::optional<Solid> cap() const { return std::nullopt; }
    /// The cage holding the rollers, nullopt for bearings without one (Python: no `cage` method).
    virtual std::optional<Solid> cage() const { return std::nullopt; }

    /// Default roller race center radius: (D1 + d1) / 4.
    double default_race_center_radius() const;
    /// Default roller diameter: 0.625 (D1 - d1).
    double default_roller_diameter() const;
    /// Return identifying information, e.g. "SingleRowDeepGrooveBallBearing(SKT): M8-22-7".
    std::string info() const;
    /// Which derived class created this bearing.
    std::string bearing_class() const { return class_name_; }
    /// Screw only parameter (0).
    double length_offset() const { return 0; }
    /// Create bearing from the shapes defined in the derived class: the races revolved, the
    /// rollers around the race (or the caps) and the cage, as the children of a Compound.
    Compound make_bearing() const;
    /// Create 2D profile inner race: a rounded rectangle from d to d1.
    Face default_inner_race_section() const;
    /// Create 2D profile outer race: a rounded rectangle from D1 to D.
    Face default_outer_race_section() const;
    /// Default press fit seat: the bearing outline less the interference.
    Face default_countersink_profile(double interference = 0) const;
    /// Default roller: a ball.
    Solid default_roller() const;
    /// Default cap: a thin ring covering the raceway.
    Solid default_cap() const;

    /// The size, stripped, e.g. "M8-22-7".
    std::string bearing_size;
    /// The type identifier, e.g. "SKT".
    std::string bearing_type;
    /// Whether the bearing has caps (method_exists("cap")).
    bool capped = false;
    /// Whether the size is metric (starts with "M").
    bool is_metric = true;
    /// Custom holes locations (bd_warehouse fills them in builder mode only; always empty here).
    std::vector<Location> hole_locations;
    /// The dimensions of the bearing from its parameter table (numbers in mm, and text such as
    /// the designation).
    ParamDict bearing_dict;

protected:
    Bearing() = default;
    /// The table of the derived class (its bearing_data).
    virtual const StringTable& class_bearing_data() const = 0;
    /// Python's Bearing.__init__ (called by the derived constructors).
    void init(const std::string& class_name, const std::string& size, const std::string& bearing_type);
    std::string class_name_;
};

#define BD_WAREHOUSE_BEARING_CLASS_MEMBERS(Class)                                        \
public:                                                                                  \
    /** The parameter table of the class's bearings. */                                  \
    static const StringTable& bearing_data();                                            \
    /** Return a set of the bearing types. */                                            \
    static std::set<std::string> types();                                                \
    /** Return a list of the bearing sizes for the given type. */                        \
    static std::vector<std::string> sizes(const std::string& bearing_type);              \
    explicit Class(const std::string& size, const std::string& bearing_type = "SKT");    \
                                                                                         \
protected:                                                                               \
    const StringTable& class_bearing_data() const override { return bearing_data(); }   \
                                                                                         \
public:

/// Single Row Deep Groove Ball Bearing.
///
/// Deep groove ball bearings are particularly versatile. They are simple in design,
/// non-separable, suitable for high and very high speeds and are robust in operation,
/// requiring little maintenance. Because deep groove ball bearings are the most widely used
/// bearing type, they are available in many designs, variants and sizes.
///
/// Args: size (e.g. "M8-22-7", see sizes("SKT")), bearing_type ("SKT", the default).
class SingleRowDeepGrooveBallBearing : public Bearing {
    BD_WAREHOUSE_BEARING_CLASS_MEMBERS(SingleRowDeepGrooveBallBearing)
    double roller_diameter() const override { return default_roller_diameter(); }
    double race_center_radius() const override { return default_race_center_radius(); }
    Face outer_race_section() const override { return default_outer_race_section(); }
    Face inner_race_section() const override { return default_inner_race_section(); }
    Solid roller() const override { return default_roller(); }
    Face countersink_profile(double interference = 0) const override {
        return default_countersink_profile(interference);
    }
};

/// Single Row Capped Deep Groove Ball Bearings.
///
/// Deep groove ball bearings capped with a seal or shield on both sides (the children are the
/// races and the two caps).
///
/// Args: size (e.g. "M8-22-7"), bearing_type ("SKT" (the default) or "OpenBuilds").
class SingleRowCappedDeepGrooveBallBearing : public Bearing {
    BD_WAREHOUSE_BEARING_CLASS_MEMBERS(SingleRowCappedDeepGrooveBallBearing)
    double roller_diameter() const override { return default_roller_diameter(); }
    double race_center_radius() const override { return default_race_center_radius(); }
    Face outer_race_section() const override { return default_outer_race_section(); }
    Face inner_race_section() const override { return default_inner_race_section(); }
    Solid roller() const override { return default_roller(); }
    std::optional<Solid> cap() const override { return default_cap(); }
    Face countersink_profile(double interference = 0) const override {
        return default_countersink_profile(interference);
    }
};

/// Single Row Angular Contact Ball Bearing.
///
/// Angular contact ball bearings have raceways in the inner and outer rings that are displaced
/// relative to each other in the direction of the bearing axis. This means that they are
/// designed to accommodate combined loads, i.e. simultaneously acting radial and axial loads.
/// The axial load carrying capacity of angular contact ball bearings increases with increasing
/// contact angle. The contact angle is defined as the angle between the line joining the points
/// of contact of the ball and the raceways in the radial plane, along which the load is
/// transmitted from one raceway to another, and a line perpendicular to the bearing axis.
///
/// Args: size (e.g. "M10-30-9"), bearing_type ("SKT", the default).
class SingleRowAngularContactBallBearing : public Bearing {
    BD_WAREHOUSE_BEARING_CLASS_MEMBERS(SingleRowAngularContactBallBearing)
    /// The ball diameter: (D2 - d2) / 2 with D2 = D - (d2 - d).
    double roller_diameter() const override;
    /// Contact angle in degrees from the radial plane: atan(a / (D / 2)).
    double contact_angle() const;
    double race_center_radius() const override { return default_race_center_radius(); }
    /// The inner ring with its shoulder and the ball groove.
    Face inner_race_section() const override;
    /// The outer ring with its shoulder and the ball groove.
    Face outer_race_section() const override;
    /// Cage holding the balls apart: a revolved spline surface with ball holes, thickened.
    std::optional<Solid> cage() const override;
    Solid roller() const override { return default_roller(); }
    Face countersink_profile(double interference = 0) const override {
        return default_countersink_profile(interference);
    }
};

/// Single Row Cylindrical Roller Bearings.
///
/// Suitable for very heavy radial loads at moderate speeds, roller bearings use cylindrical
/// rollers instead of spherical ball bearings.
///
/// Args: size (e.g. "M15-35-11"), bearing_type ("SKT", the default).
class SingleRowCylindricalRollerBearing : public Bearing {
    BD_WAREHOUSE_BEARING_CLASS_MEMBERS(SingleRowCylindricalRollerBearing)
    double roller_diameter() const override { return default_roller_diameter(); }
    double race_center_radius() const override { return default_race_center_radius(); }
    Face outer_race_section() const override { return default_outer_race_section(); }
    Face inner_race_section() const override { return default_inner_race_section(); }
    /// A cylinder 0.7 B long, centred on z=0.
    Solid roller() const override;
    Face countersink_profile(double interference = 0) const override {
        return default_countersink_profile(interference);
    }
};

/// Tapered Roller Bearing.
///
/// Tapered roller bearings have tapered inner and outer ring raceways and tapered rollers.
/// They are designed to accommodate combined loads, i.e. simultaneously acting radial and axial
/// loads. The projection lines of the raceways meet at a common point on the bearing axis to
/// provide true rolling and low friction. The axial load carrying capacity of tapered roller
/// bearings increases with increasing contact angle. A single row tapered roller bearing is
/// typically adjusted against a second tapered roller bearing.
///
/// Single row tapered roller bearings are separable, i.e. the inner ring with roller and cage
/// assembly (cone) can be mounted separately from the outer ring (cup).
///
/// The races and the roller are made once and kept (as bd_warehouse caches them).
///
/// Args: size (e.g. "M25-52-19.25"), bearing_type ("SKT", the default).
class SingleRowTaperedRollerBearing : public Bearing {
    BD_WAREHOUSE_BEARING_CLASS_MEMBERS(SingleRowTaperedRollerBearing)
    /// Diameter of the larger end of the roller - increased diameter allows for room for the
    /// cage between the rollers (1.25 times the roller's largest diameter).
    double roller_diameter() const override;
    /// Angle of the outer raceway: atan(e) in degrees.
    double contact_angle() const;
    /// Radius of cone to place the rollers.
    double race_center_radius() const override;
    /// floor(2 pi race_center_radius / roller_diameter).
    int roller_count() const override;
    /// Outer Cup.
    Face outer_race_section() const override;
    /// Central Cone (with a slot around it that captures the rollers).
    Face inner_race_section() const override;
    /// Tapered Roller (along X at the origin, rotated as it lies around the cone).
    Solid roller() const override;
    Face countersink_profile(double interference = 0) const override {
        return default_countersink_profile(interference);
    }
    /// Cage holding the rollers together with the cone.
    std::optional<Solid> cage() const override;

    /// The length of the cone's raceway (set when the inner race section is made).
    mutable double taper_length = 0.0;
    /// The line of the roller's section the cage follows (set when the roller is made).
    mutable Edge cage_edge;

protected:
    /// The outer race section, made (inner_race_section, roller: the same, cached).
    Face _outer_race_section() const;
    Face _inner_race_section() const;
    Solid _roller() const;

private:
    mutable std::optional<Face> outer_race_section_cache_;
    mutable std::optional<Face> inner_race_section_cache_;
    mutable std::optional<Solid> roller_cache_;
    mutable std::vector<double> roller_diameters_;
    mutable double race_center_radius_ = 0.0;
};

#undef BD_WAREHOUSE_BEARING_CLASS_MEMBERS

/// Press Fit Hole.
///
/// A press fit hole for a bearing is a precisely machined cavity designed to hold a bearing
/// securely through interference fit, where the hole's diameter is slightly smaller than the
/// bearing's outer diameter. This fit relies on frictional force to keep the bearing in place
/// without additional fasteners. Press fit holes are widely used in high-precision applications
/// like automotive, aerospace, and industrial machinery.
///
/// The hole is the seat (from z=0 down by the bearing's thickness) and the axle hole below it;
/// subtract it from a part: `part -= Pos(x, y, top) * PressFitHole(bearing)`.
///
/// Args:
///     bearing: A bearing instance.
///     interference: The amount the decrease the hole radius from the bearing outer radius.
///         Defaults to 0.
///     fit: one of "Close", "Normal", "Loose" which determines the axle hole diameter. Defaults
///         to "Normal".
///     depth: axle hole depth. Defaults to the bearing thickness (bd_warehouse's builder mode
///         default, through the part, needs a builder).
///
/// Raises:
///     std::invalid_argument: invalid fit.
class PressFitHole : public build123d::BasePartObject {
public:
    explicit PressFitHole(const Bearing& bearing, double interference = 0, const std::string& fit = "Normal",
                          std::optional<double> depth = std::nullopt);
    /// The depth of the axle hole.
    double hole_depth = 0.0;
};

}  // namespace bd_warehouse
