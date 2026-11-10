// bd_warehouse-cpp: bushings (bd_warehouse's bushing.py).
//
//   using namespace bd_warehouse;
//   HexFlangedEccentricBushing bushing(5, 8, 6, 2.5, 13, 0.75);  // bore 5, body 8 x 6, hex 13
#pragma once

#include <optional>

#include <build123d/build123d.hpp>

namespace bd_warehouse {

using build123d::Align;
using build123d::Align3;
using build123d::Rotation;

/// A cylindrical eccentric bushing with a wrenchable hex flange.
///
/// The cylindrical body and hex flange share the local Z axis. The bore is displaced from that
/// axis by `eccentricity` along +X. The flange stands on z=0 and the body hangs below it; the
/// material is stainless steel.
///
/// This is a generic catalog-style component, not an implementation of an ISO, DIN, or other
/// dimensional standard.
///
/// Args:
///     bore_diameter: Diameter of the through bore.
///     body_diameter: Diameter of the cylindrical mounting body.
///     body_length: Length of the cylindrical body below the flange.
///     flange_height: Axial thickness of the hex flange.
///     hex_size: Width across flats of the hex flange.
///     eccentricity: Distance between the bore and body axes.
///     rotation: object rotation. Defaults to no rotation.
///     align: object alignment. Defaults to Align::NONE (no alignment).
///
/// Attributes:
///     overall_length: Sum of body_length and flange_height.
///     joints: An "outer_center" joint at the center of the underside of the hex flange and an
///         "ecc_top_center" joint at the eccentric bore center on top.
///
/// Raises:
///     std::invalid_argument: If a dimension is invalid or the bore breaks through the cylindrical
///         body.
class HexFlangedEccentricBushing : public build123d::BasePartObject {
public:
    HexFlangedEccentricBushing(double bore_diameter, double body_diameter, double body_length, double flange_height,
                               double hex_size, double eccentricity, const Rotation& rotation = Rotation(),
                               std::optional<Align3> align = Align3(Align::NONE));

    double bore_diameter = 0.0;
    double body_diameter = 0.0;
    double body_length = 0.0;
    double flange_height = 0.0;
    double hex_size = 0.0;
    double eccentricity = 0.0;
    /// body_length + flange_height.
    double overall_length = 0.0;
};

}  // namespace bd_warehouse
