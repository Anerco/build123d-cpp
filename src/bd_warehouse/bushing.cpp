// Port of bd_warehouse's bushing.py.
#include "bd_warehouse/bushing.hpp"

#include <cmath>

#include "builder.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;

HexFlangedEccentricBushing::HexFlangedEccentricBushing(double bore_diameter_, double body_diameter_,
                                                       double body_length_, double flange_height_, double hex_size_,
                                                       double eccentricity_, const Rotation& rotation,
                                                       std::optional<Align3> align) {
    if (eccentricity_ < 0) throw std::invalid_argument("eccentricity must be non-negative");
    if (bore_diameter_ + 2 * eccentricity_ >= body_diameter_) {
        throw std::invalid_argument("bore_diameter plus twice eccentricity must be less than body_diameter");
    }
    if (hex_size_ < body_diameter_) throw std::invalid_argument("hex_size must be at least body_diameter");

    bore_diameter = bore_diameter_;
    body_diameter = body_diameter_;
    body_length = body_length_;
    flange_height = flange_height_;
    hex_size = hex_size_;
    eccentricity = eccentricity_;
    overall_length = body_length + flange_height;

    // Create a hexagonal prism and intersect it with the revolved screw-head profile used by the
    // fastener module to round off its top perimeter.
    const double hex_diagonal = hex_size / std::cos(M_PI / 6);
    Builder bushing(3);
    {
        Builder sketch(2);
        sketch.add(RegularPolygon(hex_size / 2, 6, false));
        bushing.publish(sketch.product());
    }
    detail::extrude(bushing, flange_height);
    {
        Builder head_profile(2);
        head_profile.add(Rectangle(hex_diagonal / 2, flange_height, Align::MIN));
        detail::chamfer(head_profile, {head_profile.vertices().group_by(Axis::Y)[-1].sort_by(Axis::X)[-1]},
                        (hex_diagonal - hex_size) / 2);
        bushing.publish(head_profile.product(), Mode::ADD, {Location()}, {Location(Plane::XZ)});
    }
    detail::revolve(bushing, Axis::Z, 360.0, true, Mode::INTERSECT);
    bushing.add(Cylinder(body_diameter / 2, body_length, Align3(Align::CENTER, Align::CENTER, Align::MAX)));
    detail::hole(bushing, bore_diameter / 2, std::nullopt, {Location(Vector(eccentricity, 0, 0))});

    assign(bushing.part(), rotation, align);
    label = "HexFlangedEccentricBushing";
    material = metals::stainless();
    RigidJoint("outer_center", *this, Location());
    RigidJoint("ecc_top_center", *this, Location(Vector(eccentricity, 0, flange_height)));
}

}  // namespace bd_warehouse
