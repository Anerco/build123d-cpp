// Port of bd_warehouse's gear.py.
#include "bd_warehouse/gear.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepFill.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

#include "builder.hpp"
#include "data.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::degrees;
using detail::Mode;
using detail::radians;

namespace {

/// math.acos, raising Python's "math domain error" outside [-1, 1].
double py_acos(double x) {
    if (!(x >= -1.0 && x <= 1.0)) throw std::invalid_argument("math domain error");
    return std::acos(x);
}

double py_sqrt(double x) {
    if (x < 0) throw std::invalid_argument("math domain error");
    return std::sqrt(x);
}

FinishedMaterial blackened_alloy_steel() {
    return metals::alloy_steel(metals::AlloySteel::G4140_QUENCHED_TEMPERED,
                               MetalOptions{.finish = finishes::black_oxide()});
}

void check_module_system(const std::string& module_system) {
    if (module_system != "normal" && module_system != "transverse") {
        throw std::invalid_argument("module_system must be either 'normal' or 'transverse'");
    }
}

/// RackGearPlan._reference_values / HelicalGear's module conversion: (normal module, transverse
/// module, normal pressure angle, transverse pressure angle).
std::array<double, 4> reference_values(double module, double pressure_angle, double helix_angle,
                                       const std::string& module_system) {
    const double beta = radians(helix_angle);
    if (module_system == "normal") {
        return {module, module / std::cos(beta), pressure_angle,
                degrees(std::atan(std::tan(radians(pressure_angle)) / std::cos(beta)))};
    }
    return {module * std::cos(beta), module, degrees(std::atan(std::tan(radians(pressure_angle)) * std::cos(beta))),
            pressure_angle};
}

}  // namespace

// ---------------------------------------------------------------------------
// InvoluteToothProfile
// ---------------------------------------------------------------------------

namespace {

struct ToothProfile {
    double module, pitch_radius, base_radius, addendum, addendum_radius, dedendum, root_radius;
    int tooth_count;
    Wire wire;
};

ToothProfile make_tooth_profile(double module, int tooth_count, double pressure_angle,
                                const InvoluteToothProfileOptions& o) {
    ToothProfile t{};
    t.module = module;
    t.tooth_count = tooth_count;
    t.pitch_radius = module * tooth_count / 2;
    t.base_radius = t.pitch_radius * std::cos(radians(pressure_angle));
    t.addendum = o.addendum ? *o.addendum : module;
    t.addendum_radius = t.pitch_radius + t.addendum;
    t.dedendum = o.dedendum ? *o.dedendum : 1.25 * module;
    t.root_radius = t.pitch_radius - t.dedendum;
    if (tooth_count == 0) throw std::domain_error("float division by zero");
    const double half_thick_angle = 90.0 / tooth_count;
    const double half_pitch_angle =
        half_thick_angle + degrees(std::tan(radians(pressure_angle)) - radians(pressure_angle));
    // Create the involute curve points
    const double involute_size = t.addendum_radius - t.base_radius;
    std::vector<std::pair<double, double>> pnts;
    for (int i = 0; i < 11; ++i) {
        const double r = t.base_radius + involute_size * i / 10;
        if (r == 0) throw std::domain_error("float division by zero");
        const double alpha = py_acos(t.base_radius / r);  // in radians
        const double involute = std::tan(alpha) - alpha;
        const double rp = r * std::cos(involute);
        if (rp > t.root_radius) pnts.emplace_back(rp, r * std::sin(involute));
    }

    Builder tooth(1);
    std::vector<Vector> rotated_pnts;
    for (const auto& [x, y] : pnts) rotated_pnts.push_back(Vector(x, y).rotate(Axis::Z, -half_pitch_angle));
    if (rotated_pnts.empty()) throw std::out_of_range("list index out of range");
    const Spline l1(rotated_pnts);
    tooth.add(l1);
    const Vector root_flank = Vector(t.root_radius, 0).rotate(Axis::Z, -half_pitch_angle);
    const Line l2(rotated_pnts[0], root_flank);
    tooth.add(l2);
    tooth.add(RadiusArc(l2.position_at(1), Vector(t.root_radius, 0).rotate(Axis::Z, -2 * half_thick_angle),
                        t.root_radius));
    tooth.add(RadiusArc(l1.position_at(1), Vector(t.addendum_radius, 0), -t.addendum_radius));
    if (o.root_fillet) {
        try {
            detail::fillet(tooth, {tooth.vertices().sort_by(Axis::X)[1]}, *o.root_fillet);
        } catch (const Standard_Failure&) {
            throw std::invalid_argument("Invalid root radius, try a smaller value");
        }
    }
    const ShapeList<Edge> edges = tooth.edges();
    detail::mirror(tooth, std::vector<Shape>(edges.begin(), edges.end()), Plane::XZ);

    // (bd_warehouse closes the tooth through Vertex.to_tuple, which this build123d doesn't have)
    if (o.closed) throw std::invalid_argument("'Vertex' object has no attribute 'to_tuple'");
    t.wire = Wire::combine(std::vector<Edge>(tooth.edges())).at(0);
    return t;
}

}  // namespace

InvoluteToothProfile::InvoluteToothProfile(double module_, int tooth_count_, double pressure_angle,
                                           std::optional<double> root_fillet, std::optional<double> addendum_,
                                           std::optional<double> dedendum_, bool closed)
    : InvoluteToothProfile(module_, tooth_count_, pressure_angle,
                           InvoluteToothProfileOptions{root_fillet, addendum_, dedendum_, closed}) {}

InvoluteToothProfile::InvoluteToothProfile(double module_, int tooth_count_, double pressure_angle,
                                           const InvoluteToothProfileOptions& options) {
    const ToothProfile t = make_tooth_profile(module_, tooth_count_, pressure_angle, options);
    static_cast<Wire&>(*this) = BaseLineObject(t.wire);
    module = t.module;
    tooth_count = t.tooth_count;
    pitch_radius = t.pitch_radius;
    base_radius = t.base_radius;
    addendum = t.addendum;
    addendum_radius = t.addendum_radius;
    dedendum = t.dedendum;
    root_radius = t.root_radius;
}

// ---------------------------------------------------------------------------
// RackGearPlan, RackGear
// ---------------------------------------------------------------------------

RackGearPlan::RackGearPlan(double module_, int tooth_count_, double pressure_angle_, double helix_angle_,
                           const std::string& module_system_, std::optional<double> root_fillet_,
                           std::optional<double> addendum_, std::optional<double> dedendum_,
                           std::optional<double> base_height_, double rotation, std::optional<Align2> align)
    : RackGearPlan(module_, tooth_count_, pressure_angle_,
                   RackGearPlanOptions{helix_angle_, module_system_, root_fillet_, addendum_, dedendum_, base_height_,
                                       rotation, align}) {}

RackGearPlan::RackGearPlan(double module_, int tooth_count_, double pressure_angle_, const RackGearPlanOptions& o) {
    const double resolved_base_height = o.base_height ? *o.base_height : 2 * module_;
    // _validate_parameters
    if (module_ <= 0) throw std::invalid_argument("module must be greater than zero");
    if (tooth_count_ <= 0) throw std::invalid_argument("tooth_count must be a positive integer");
    if (!(0 <= pressure_angle_ && pressure_angle_ < 90)) {
        throw std::invalid_argument("pressure_angle must be in the range [0, 90)");
    }
    if (!(-90 < o.helix_angle && o.helix_angle < 90)) {
        throw std::invalid_argument("helix_angle must be in the range (-90, 90)");
    }
    check_module_system(o.module_system);
    if (o.root_fillet && *o.root_fillet < 0) throw std::invalid_argument("root_fillet must be non-negative");
    if (o.addendum && *o.addendum <= 0) throw std::invalid_argument("addendum must be greater than zero");
    if (o.dedendum && *o.dedendum <= 0) throw std::invalid_argument("dedendum must be greater than zero");
    if (resolved_base_height <= 0) throw std::invalid_argument("base_height must be greater than zero");

    const auto values = reference_values(module_, pressure_angle_, o.helix_angle, o.module_system);
    normal_module = values[0];
    transverse_module = values[1];
    normal_pressure_angle = values[2];
    transverse_pressure_angle = values[3];

    module = module_;
    module_system = o.module_system;
    tooth_count = tooth_count_;
    pressure_angle = pressure_angle_;
    helix_angle = o.helix_angle;
    addendum = o.addendum ? *o.addendum : module;
    dedendum = o.dedendum ? *o.dedendum : 1.25 * module;
    root_fillet = o.root_fillet;
    base_height = resolved_base_height;
    pitch = M_PI * transverse_module;
    pitch_length = tooth_count * pitch;
    tooth_height = addendum + dedendum;

    const double alpha_t = radians(transverse_pressure_angle);
    const double half_pitch_thickness = pitch / 4;
    const double half_tip = half_pitch_thickness - addendum * std::tan(alpha_t);
    const double half_root = half_pitch_thickness + dedendum * std::tan(alpha_t);
    const double half_space = pitch / 2 - half_root;
    if (half_tip <= 0) throw std::invalid_argument("addendum and pressure_angle produce zero or negative tip width");
    if (half_space <= 0) {
        throw std::invalid_argument("dedendum and pressure_angle produce zero or negative root space");
    }

    // Constructive form: an array of pressure-angle trapezoidal teeth joined to the rectangular
    // rack body at the root line.
    const Trapezoid tooth_profile(2 * half_root, tooth_height, 90 - transverse_pressure_angle,
                                  90 - transverse_pressure_angle, 0.0, Align2(Align::CENTER, Align::MIN));
    Sketch rack_profile = Pos(0, -dedendum) * (Rectangle(pitch_length, base_height, Align2(Align::CENTER, Align::MAX)) +
                                               GridLocations(pitch, 0, tooth_count, 1) * tooth_profile);
    if (o.root_fillet && *o.root_fillet != 0) {
        try {
            const ShapeList<Vertex> root_vertices =
                rack_profile.vertices().group_by(Axis::Y)[1].sort_by(Axis::X).slice(1, -1);
            rack_profile = Sketch(build123d::fillet(root_vertices, *o.root_fillet));
        } catch (const Standard_Failure&) {
            throw std::invalid_argument("Invalid root fillet radius, try a smaller value");
        } catch (const std::invalid_argument&) {
            throw std::invalid_argument("Invalid root fillet radius, try a smaller value");
        }
    }
    assign(rack_profile, o.rotation, o.align);
}

RackGear::RackGear(double module_, int tooth_count_, double pressure_angle_, double thickness_, double helix_angle_,
                   const std::string& module_system_, std::optional<double> root_fillet_,
                   std::optional<double> addendum_, std::optional<double> dedendum_, std::optional<double> base_height_,
                   const Rotation& rotation, std::optional<Align3> align)
    : RackGear(module_, tooth_count_, pressure_angle_, thickness_,
               RackGearOptions{helix_angle_, module_system_, root_fillet_, addendum_, dedendum_, base_height_, rotation,
                               align}) {}

RackGear::RackGear(double module_, int tooth_count_, double pressure_angle_, double thickness_,
                   const RackGearOptions& o) {
    if (thickness_ <= 0) throw std::invalid_argument("thickness must be greater than zero");
    const RackGearPlan rack_plan(module_, tooth_count_, pressure_angle_,
                                 RackGearPlanOptions{o.helix_angle, o.module_system, o.root_fillet, o.addendum,
                                                     o.dedendum, o.base_height});
    module = rack_plan.module;
    module_system = rack_plan.module_system;
    tooth_count = rack_plan.tooth_count;
    pressure_angle = rack_plan.pressure_angle;
    helix_angle = rack_plan.helix_angle;
    normal_module = rack_plan.normal_module;
    transverse_module = rack_plan.transverse_module;
    normal_pressure_angle = rack_plan.normal_pressure_angle;
    transverse_pressure_angle = rack_plan.transverse_pressure_angle;
    addendum = rack_plan.addendum;
    dedendum = rack_plan.dedendum;
    root_fillet = rack_plan.root_fillet;
    base_height = rack_plan.base_height;
    pitch = rack_plan.pitch;
    pitch_length = rack_plan.pitch_length;
    tooth_height = rack_plan.tooth_height;
    thickness = thickness_;
    lateral_offset = thickness * std::tan(radians(o.helix_angle));

    Solid rack;
    if (o.helix_angle == 0) {
        rack = Solid::extrude(rack_plan.face(), Vector(0, 0, thickness));
    } else {
        // Ruled loft between equivalent planar profiles gives linear tooth traces.
        const Sketch top_plan = Pos(lateral_offset, 0, thickness) * rack_plan;
        rack = Solid::make_loft({rack_plan.face().wire(), top_plan.face().wire()}, true);
    }
    assign(Rot(90, 0, 0) * Pos(0, 0, -thickness / 2) * rack, o.rotation, o.align);
}

// ---------------------------------------------------------------------------
// SpurGearPlan, SpurGear, HelicalGear
// ---------------------------------------------------------------------------

SpurGearPlan::SpurGearPlan(double module, int tooth_count, double pressure_angle, std::optional<double> root_fillet,
                           std::optional<double> addendum, std::optional<double> dedendum, double rotation,
                           std::optional<Align2> align)
    : SpurGearPlan(module, tooth_count, pressure_angle,
                   SpurGearPlanOptions{root_fillet, addendum, dedendum, rotation, align}) {}

SpurGearPlan::SpurGearPlan(double module, int tooth_count, double pressure_angle, const SpurGearPlanOptions& o) {
    const InvoluteToothProfile gear_tooth(module, tooth_count, pressure_angle, o.root_fillet, o.addendum, o.dedendum);
    pitch_radius = gear_tooth.pitch_radius;
    base_radius = gear_tooth.base_radius;
    addendum_radius = gear_tooth.addendum_radius;
    root_radius = gear_tooth.root_radius;
    const auto gear_teeth = PolarLocations(0, tooth_count) * gear_tooth;
    std::vector<Edge> edges;
    for (const auto& tooth : gear_teeth) {
        for (const auto& e : tooth.edges()) edges.push_back(e);
    }
    const Wire gear_wire(edges);
    Face gear_face(gear_wire);
    if (gear_face.normal_at().Z < 0) gear_face = -gear_face;
    assign(gear_face, o.rotation, o.align);
}

SpurGear::SpurGear(double module, int tooth_count, double pressure_angle, double thickness,
                   std::optional<double> root_fillet, std::optional<double> addendum, std::optional<double> dedendum,
                   const Rotation& rotation, std::optional<Align3> align)
    : SpurGear(module, tooth_count, pressure_angle, thickness,
               SpurGearOptions{root_fillet, addendum, dedendum, rotation, align}) {}

SpurGear::SpurGear(double module, int tooth_count, double pressure_angle, double thickness, const SpurGearOptions& o) {
    const SpurGearPlan gear_plan(module, tooth_count, pressure_angle, o.root_fillet, o.addendum, o.dedendum);
    pitch_radius = gear_plan.pitch_radius;
    base_radius = gear_plan.base_radius;
    addendum_radius = gear_plan.addendum_radius;
    root_radius = gear_plan.root_radius;
    assign(build123d::extrude(gear_plan, thickness), o.rotation, o.align);
    material = blackened_alloy_steel();
}

HelicalGear::HelicalGear(double module_, int tooth_count_, double pressure_angle_, double helix_angle_,
                         double thickness_, const std::string& module_system_, std::optional<double> root_fillet,
                         std::optional<double> addendum, std::optional<double> dedendum, const Rotation& rotation,
                         std::optional<Align3> align) {
    init(module_, tooth_count_, pressure_angle_, helix_angle_, thickness_,
         HelicalGearOptions{module_system_, root_fillet, addendum, dedendum, rotation, align});
}

HelicalGear::HelicalGear(double module_, int tooth_count_, double pressure_angle_, double helix_angle_,
                         double thickness_, const HelicalGearOptions& options) {
    init(module_, tooth_count_, pressure_angle_, helix_angle_, thickness_, options);
}

void HelicalGear::init(double module_, int tooth_count_, double pressure_angle_, double helix_angle_,
                       double thickness_, const HelicalGearOptions& o) {
    if (module_ <= 0) throw std::invalid_argument("module must be greater than zero");
    if (tooth_count_ <= 0) throw std::invalid_argument("tooth_count must be greater than zero");
    if (thickness_ <= 0) throw std::invalid_argument("thickness must be greater than zero");
    if (!(0 <= pressure_angle_ && pressure_angle_ < 90)) {
        throw std::invalid_argument("pressure_angle must be in the range [0, 90)");
    }
    if (!(-90 < helix_angle_ && helix_angle_ < 90)) {
        throw std::invalid_argument("helix_angle must be in the range (-90, 90)");
    }
    check_module_system(o.module_system);

    const double beta = radians(helix_angle_);
    const double reference_addendum = o.addendum ? *o.addendum : module_;
    const double reference_dedendum = o.dedendum ? *o.dedendum : 1.25 * module_;
    const auto values = reference_values(module_, pressure_angle_, helix_angle_, o.module_system);
    normal_module = values[0];
    transverse_module = values[1];
    normal_pressure_angle = values[2];
    transverse_pressure_angle = values[3];

    const SpurGearPlan gear_plan(transverse_module, tooth_count_, transverse_pressure_angle, o.root_fillet,
                                 reference_addendum, reference_dedendum);
    module = module_;
    module_system = o.module_system;
    tooth_count = tooth_count_;
    pressure_angle = pressure_angle_;
    helix_angle = helix_angle_;
    thickness = thickness_;
    pitch_radius = gear_plan.pitch_radius;
    base_radius = gear_plan.base_radius;
    addendum_radius = gear_plan.addendum_radius;
    root_radius = gear_plan.root_radius;
    twist_angle = degrees(thickness * std::tan(beta) / pitch_radius);
    lead = helix_angle != 0 ? 2 * M_PI * pitch_radius / std::abs(std::tan(beta))
                            : std::numeric_limits<double>::infinity();

    Shape gear = helix_angle == 0
                     ? Shape(build123d::extrude(gear_plan, thickness))
                     : Shape(Solid::extrude_linear_with_rotation(gear_plan.face(), Vector(0, 0, 0),
                                                                 Vector(0, 0, thickness), twist_angle));
    gear.material = blackened_alloy_steel();
    assign(gear, o.rotation, o.align);
    material = blackened_alloy_steel();
}

// ---------------------------------------------------------------------------
// Worm gears
// ---------------------------------------------------------------------------

namespace {

/// _involute: inv(α) = tan(α) − α, with α in radians.
double involute_of(double angle) { return std::tan(angle) - angle; }

/// _uv_face: the face on `surface` bounded by straight segments between `corners` in its (u, v)
/// parameter space, and its edges in corner order (edge i runs corner i -> i+1).
std::pair<Face, std::vector<Edge>> uv_face(const Handle(Geom_Surface) & surface,
                                           const std::vector<std::pair<double, double>>& corners) {
    BRepBuilderAPI_MakeWire wire_builder;
    std::vector<Edge> edges;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto [u0, v0] = corners[i];
        const auto [u1, v1] = corners[(i + 1) % corners.size()];
        // a line's parameter is distance along it
        const Handle(Geom2d_Line) line = new Geom2d_Line(gp_Pnt2d(u0, v0), gp_Dir2d(u1 - u0, v1 - v0));
        const Handle(Geom2d_TrimmedCurve) segment = new Geom2d_TrimmedCurve(line, 0.0, std::hypot(u1 - u0, v1 - v0));
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(segment, surface).Edge();
        BRepLib::BuildCurves3d(edge);
        wire_builder.Add(edge);
        edges.emplace_back(edge);
    }
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(surface, wire_builder.Wire()).Face();
    return {Face(face), edges};
}

using UAt = std::function<double(double, double)>;

/// _u_parameter: u_at(z, v), the u parameter of a ruled helicoid at height z on the v = const
/// isoline, given the axial offsets of its bounding helices.
UAt u_parameter(double z_start, double root_offset, double tip_offset, double u_scale) {
    return [=](double z, double v) { return (z - z_start - (1 - v) * root_offset - v * tip_offset) * u_scale; };
}

/// _WormGeometry: the derived dimensions of a cylindrical worm (ISO 54 / DIN 3975 nomenclature).
/// The worm axis is Z, the reference tooth (start 0) is centred on the +X axis in the z = 0
/// transverse plane, and the worm is right handed.
class WormGeometry {
public:
    WormGeometry(double module, int starts, double diametral_quotient, double pressure_angle,
                 const std::string& flank_form, std::optional<double> addendum, std::optional<double> dedendum) {
        if (module <= 0) throw std::invalid_argument("module must be greater than zero");
        if (starts <= 0) throw std::invalid_argument("starts must be a positive integer");
        if (diametral_quotient <= 0) throw std::invalid_argument("diametral_quotient must be greater than zero");
        if (!(0 < pressure_angle && pressure_angle < 90)) {
            throw std::invalid_argument("pressure_angle must be in the range (0, 90)");
        }
        if (flank_form != "ZI" && flank_form != "ZA") throw std::invalid_argument("flank_form must be either 'ZI' or 'ZA'");
        if (addendum && *addendum <= 0) throw std::invalid_argument("addendum must be greater than zero");
        if (dedendum && *dedendum <= 0) throw std::invalid_argument("dedendum must be greater than zero");

        d.module = module;
        d.starts = starts;
        d.diametral_quotient = diametral_quotient;
        d.flank_form = flank_form;
        d.normal_pressure_angle = pressure_angle;
        d.addendum = addendum ? *addendum : module;
        d.dedendum = dedendum ? *dedendum : 1.2 * module;

        d.pitch_radius = module * diametral_quotient / 2;
        d.addendum_radius = d.pitch_radius + d.addendum;
        d.root_radius = d.pitch_radius - d.dedendum;
        if (d.root_radius <= 0) throw std::invalid_argument("dedendum must be less than the reference radius");
        d.axial_pitch = M_PI * module;
        d.lead = M_PI * module * starts;
        advance = d.lead / (2 * M_PI);  // axial advance per radian of rotation
        const double gamma = std::atan(starts / diametral_quotient);
        const double alpha_n = radians(pressure_angle);
        d.lead_angle = degrees(gamma);
        d.axial_pressure_angle = degrees(std::atan(std::tan(alpha_n) / std::cos(gamma)));

        if (flank_form == "ZI") {
            // cos γb = cos γ · cos αn relates the base and reference helices of an involute
            // helicoid; the lead is shared so r·tan(γ_r) is constant.
            gamma_b = py_acos(std::cos(gamma) * std::cos(alpha_n));
            base_radius = advance / std::tan(gamma_b);
            alpha_t1 = py_acos(base_radius / d.pitch_radius);
            d.base_lead_angle = degrees(gamma_b);
            d.base_radius = base_radius;
            d.transverse_pressure_angle = degrees(alpha_t1);
        } else {
            gamma_b = base_radius = alpha_t1 = std::numeric_limits<double>::quiet_NaN();
        }

        if (half_thickness_angle(d.addendum_radius) <= 0) {
            throw std::invalid_argument("addendum and pressure_angle produce zero or negative tip width");
        }
        if (half_thickness_angle(d.root_radius) >= M_PI / starts) {
            throw std::invalid_argument("dedendum and pressure_angle produce zero or negative root space");
        }
    }

    WormDimensions d;
    double advance = 0.0;
    double gamma_b = 0.0;
    double base_radius = 0.0;
    double alpha_t1 = 0.0;

    /// Half the angular tooth thickness (radians) in a transverse section.
    double half_thickness_angle(double radius) const {
        const double half_pitch_angle = M_PI / (2 * d.starts);
        if (d.flank_form == "ZA") {
            return half_pitch_angle - (radius - d.pitch_radius) * std::tan(radians(d.axial_pressure_angle)) / advance;
        }
        const double alpha_t = py_acos(base_radius / std::max(radius, base_radius));
        return half_pitch_angle + involute_of(alpha_t1) - involute_of(alpha_t);
    }

    /// Phase and axial offset of a flank helix: the flank of the tooth centred on angle 0 at
    /// z = 0 crosses the cylinder of `radius` along the helix (radius, phase + z/advance,
    /// z + offset). side +1 is the flank facing -Z (lower), -1 the flank facing +Z (upper).
    std::pair<double, double> flank_helix(double radius, int side) const {
        if (d.flank_form == "ZA") {
            return {side * M_PI / (2 * d.starts),
                    side * (radius - d.pitch_radius) * std::tan(radians(d.axial_pressure_angle))};
        }
        // ZI: rulings are tangents of the base helix. Below the base cylinder the flank continues
        // radially (a plain helicoid), as an involute is undefined there.
        const double base_phase = side * (M_PI / (2 * d.starts) + involute_of(alpha_t1));
        if (radius <= base_radius) return {base_phase, 0.0};
        return {base_phase + side * py_acos(base_radius / radius),
                side * py_sqrt(std::pow(radius, 2) - std::pow(base_radius, 2)) * std::tan(gamma_b)};
    }

    /// Angle of a flank of the reference tooth at `radius` and height `z`.
    double flank_angle(double radius, int side, double z) const {
        const auto [phase, offset] = flank_helix(radius, side);
        return phase + (z - offset) / advance;
    }

    /// Radial spans of the ruled surfaces making up one flank, root to tip.
    std::vector<std::pair<double, double>> flank_radii() const {
        if (d.flank_form == "ZI" && d.root_radius < base_radius) {
            return {{d.root_radius, base_radius}, {base_radius, d.addendum_radius}};
        }
        return {{d.root_radius, d.addendum_radius}};
    }

    /// Start height and height of helices covering z ∈ [−length/2, length/2] with margin for the
    /// flank offsets, starting a whole number of leads below z = 0.
    std::pair<double, double> helix_span(double length) const {
        const double start = -d.lead * std::ceil(length / (2 * d.lead) + 1);
        return {start, -2 * start};
    }

    /// Ruled helicoid surfaces of one flank of the tooth centred on `tooth_angle`: (surface, u_at)
    /// pairs from root to tip.
    std::vector<std::pair<Handle(Geom_Surface), UAt>> flank_surfaces(int side, double tooth_angle,
                                                                      double length) const {
        const auto [z_start, height] = helix_span(length);
        const double total_angle = 2 * M_PI * height / d.lead;
        std::vector<std::pair<Handle(Geom_Surface), UAt>> surfaces;
        for (const auto& [r0, r1] : flank_radii()) {
            std::vector<Edge> helices;
            std::vector<double> offsets;
            for (const double radius : {r0, r1}) {
                const auto [phase, offset] = flank_helix(radius, side);
                const Helix helix(d.lead, height, radius, Vector(0, 0, z_start + offset));
                helices.push_back(helix.rotate(Axis::Z, degrees(tooth_angle + phase)));
                offsets.push_back(offset);
            }
            const TopoDS_Face ruled =
                BRepFill::Face(TopoDS::Edge(helices[0].wrapped()), TopoDS::Edge(helices[1].wrapped()));
            const Handle(Geom_Surface) surface = BRep_Tool::Surface(ruled);
            double u_min = 0, u_max = 0, v_min = 0, v_max = 0;
            BRepTools::UVBounds(ruled, u_min, u_max, v_min, v_max);
            // z is linear along the helices (u) and along the rulings (v)
            const double u_scale = u_max / (advance * total_angle);
            surfaces.emplace_back(surface, u_parameter(z_start, offsets[0], offsets[1], u_scale));
        }
        return surfaces;
    }
};

void check_hand(const std::string& hand) {
    if (hand != "right" && hand != "left") throw std::invalid_argument("hand must be either 'right' or 'left'");
}

/// What WormFlank is made of.
struct FlankParts {
    TopoDS_Shape face;
    WormDimensions dims;
    double inner_radius;
};

FlankParts make_flank(double module, int starts, double diametral_quotient, double length, const WormFlankOptions& o) {
    if (length <= 0) throw std::invalid_argument("length must be greater than zero");
    check_hand(o.hand);
    if (o.side != "lower" && o.side != "upper") throw std::invalid_argument("side must be either 'lower' or 'upper'");
    const WormGeometry geometry(module, starts, diametral_quotient, o.pressure_angle, o.flank_form, o.addendum,
                                o.dedendum);
    const auto surfaces = geometry.flank_surfaces(o.side == "lower" ? 1 : -1, 0, length);
    const auto& [surface, u_at] = surfaces.back();
    const double z0 = -length / 2, z1 = length / 2;
    Face face = uv_face(surface, {{u_at(z0, 0), 0}, {u_at(z0, 1), 1}, {u_at(z1, 1), 1}, {u_at(z1, 0), 0}}).first;
    if (o.hand == "left") face = face.mirror(Plane::XZ);
    return {face.wrapped(), geometry.d, geometry.flank_radii().back().first};
}

}  // namespace

WormFlank::WormFlank(double module_, int starts_, double diametral_quotient_, double length, double pressure_angle,
                     const std::string& flank_form_, const std::string& hand_, const std::string& side_,
                     std::optional<double> addendum_, std::optional<double> dedendum_)
    : WormFlank(module_, starts_, diametral_quotient_, length,
                WormFlankOptions{pressure_angle, flank_form_, hand_, side_, addendum_, dedendum_}) {}

WormFlank::WormFlank(double module_, int starts_, double diametral_quotient_, double length,
                     const WormFlankOptions& o) {
    const FlankParts parts = make_flank(module_, starts_, diametral_quotient_, length, o);
    static_cast<Face&>(*this) = Face(parts.face);
    static_cast<WormDimensions&>(*this) = parts.dims;
    axial_length = length;
    hand = o.hand;
    side = o.side;
    inner_radius = parts.inner_radius;
}

Worm::Worm(double module_, int starts_, double diametral_quotient_, double length_, double pressure_angle,
           const std::string& flank_form_, const std::string& hand_, std::optional<double> addendum_,
           std::optional<double> dedendum_, const Rotation& rotation, std::optional<Align3> align)
    : Worm(module_, starts_, diametral_quotient_, length_,
           WormOptions{pressure_angle, flank_form_, hand_, addendum_, dedendum_, rotation, align}) {}

Worm::Worm(double module_, int starts_, double diametral_quotient_, double length_, const WormOptions& o) {
    if (length_ <= 0) throw std::invalid_argument("length must be greater than zero");
    check_hand(o.hand);
    const WormGeometry geometry(module_, starts_, diametral_quotient_, o.pressure_angle, o.flank_form, o.addendum,
                                o.dedendum);
    length = length_;
    hand = o.hand;
    static_cast<WormDimensions&>(*this) = geometry.d;

    // Every face is split into axial bands no longer than one lead so that OCCT's fixed-order
    // quadrature stays accurate on the twisted flanks.
    const int band_count = static_cast<int>(std::ceil(length / geometry.d.lead));
    std::vector<double> z_lines;
    for (int i = 0; i < band_count + 1; ++i) z_lines.push_back(-length / 2 + length * i / band_count);
    const double advance = geometry.advance;
    std::vector<Face> faces;
    std::vector<Edge> cap_edges[2];

    const auto add_bands = [&](const Handle(Geom_Surface) & surface,
                               const std::function<std::vector<std::pair<double, double>>(double, double)>& corners_at) {
        for (int i = 0; i + 1 < static_cast<int>(z_lines.size()); ++i) {
            const auto [face, edges] = uv_face(surface, corners_at(z_lines[i], z_lines[i + 1]));
            faces.push_back(face);
            if (i == 0) cap_edges[0].push_back(edges[0]);
            if (i == band_count - 1) cap_edges[1].push_back(edges[2]);
        }
    };

    for (int start = 0; start < starts_; ++start) {
        const double tooth_angle = 2 * M_PI * start / starts_;
        for (const int side : {1, -1}) {
            for (const auto& [surface, u_at] : geometry.flank_surfaces(side, tooth_angle, length)) {
                const UAt at = u_at;
                add_bands(surface, [at](double z_lo, double z_hi) {
                    return std::vector<std::pair<double, double>>{
                        {at(z_lo, 0), 0}, {at(z_lo, 1), 1}, {at(z_hi, 1), 1}, {at(z_hi, 0), 0}};
                });
            }
        }
        // tip strip between this tooth's flanks and root strip to the next tooth
        const double tip_lo = tooth_angle + geometry.flank_angle(geometry.d.addendum_radius, -1, 0);
        const double tip_hi = tooth_angle + geometry.flank_angle(geometry.d.addendum_radius, 1, 0);
        const double root_lo = tooth_angle + geometry.flank_angle(geometry.d.root_radius, 1, 0);
        const double root_hi =
            tooth_angle + 2 * M_PI / starts_ + geometry.flank_angle(geometry.d.root_radius, -1, 0);
        for (const auto& [radius, angle_lo, angle_hi] :
             {std::tuple<double, double, double>{geometry.d.addendum_radius, tip_lo, tip_hi},
              std::tuple<double, double, double>{geometry.d.root_radius, root_lo, root_hi}}) {
            const Handle(Geom_Surface) cylinder =
                new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), radius);
            const double lo = angle_lo, hi = angle_hi;
            add_bands(cylinder, [lo, hi, advance](double z_lo, double z_hi) {
                return std::vector<std::pair<double, double>>{{lo + z_lo / advance, z_lo},
                                                              {hi + z_lo / advance, z_lo},
                                                              {hi + z_hi / advance, z_hi},
                                                              {lo + z_hi / advance, z_hi}};
            });
        }
    }
    for (const auto& edges : cap_edges) faces.emplace_back(Wire(edges));

    // The band edges of adjacent faces are separately approximated copies of the same helix, so
    // sew with a tolerance above the approximation error.
    BRepBuilderAPI_Sewing sewing(1e-5);
    for (const auto& face : faces) sewing.Add(face.wrapped());
    sewing.Perform();
    if (sewing.NbFreeEdges() != 0) throw std::runtime_error("worm surfaces did not sew into a closed shell");
    TopoDS_Solid solid = BRepBuilderAPI_MakeSolid(TopoDS::Shell(sewing.SewedShape())).Solid();
    BRepLib::OrientClosedSolid(solid);
    Solid worm(solid);
    if (hand == "left") worm = worm.mirror(Plane::XZ);

    assign(worm, o.rotation, o.align);
    material = blackened_alloy_steel();
}

WormWheel::WormWheel(double module, int tooth_count, double thickness, int starts_, double diametral_quotient_,
                     double pressure_angle, const std::string& hand_, std::optional<double> root_fillet,
                     std::optional<double> addendum, std::optional<double> dedendum, const Rotation& rotation,
                     std::optional<Align3> align)
    : WormWheel(module, tooth_count, thickness,
                WormWheelOptions{starts_, diametral_quotient_, pressure_angle, hand_, root_fillet, addendum, dedendum,
                                 rotation, align}) {}

WormWheel::WormWheel(double module_, int tooth_count_, double thickness_, const WormWheelOptions& o) {
    check_hand(o.hand);
    const WormGeometry worm(module_, o.starts, o.diametral_quotient, o.pressure_angle, "ZA", o.addendum, o.dedendum);
    init(module_, tooth_count_, worm.d.axial_pressure_angle, o.hand == "right" ? worm.d.lead_angle : -worm.d.lead_angle,
         thickness_,
         HelicalGearOptions{"transverse", o.root_fillet, worm.d.addendum, worm.d.dedendum, o.rotation, o.align});
    starts = o.starts;
    diametral_quotient = o.diametral_quotient;
    hand = o.hand;
    lead_angle = worm.d.lead_angle;
    axial_pressure_angle = worm.d.axial_pressure_angle;
    center_distance = module_ * (o.diametral_quotient + tooth_count_) / 2;
    material = metals::bronze();
}

WormGear::WormGear(double module, int starts, double diametral_quotient, int tooth_count, double worm_length,
                   double wheel_thickness, double pressure_angle, const std::string& flank_form,
                   const std::string& hand, std::optional<double> root_fillet)
    : WormGear(module, starts, diametral_quotient, tooth_count, worm_length, wheel_thickness,
               WormGearOptions{pressure_angle, flank_form, hand, root_fillet}) {}

WormGear::WormGear(double module, int starts, double diametral_quotient, int tooth_count, double worm_length,
                   double wheel_thickness, const WormGearOptions& o) {
    const Worm worm(module, starts, diametral_quotient, worm_length, o.pressure_angle, o.flank_form, o.hand);
    const WormWheel wheel(module, tooth_count, wheel_thickness, starts, diametral_quotient, o.pressure_angle, o.hand,
                          o.root_fillet);
    center_distance = wheel.center_distance;
    ratio = static_cast<double>(tooth_count) / starts;
    // Thread 0 is centred on the worm's local +X; turn a thread space to face the wheel (local
    // -X), then stand the worm axis along Y beside the wheel.
    Part placed_worm = Pos(center_distance, 0, 0) * Rot(-90, 0, 0) * Rot(0, 0, 180 - 180.0 / starts) * worm;
    // HelicalGear twists from its bottom face, so undo half the twist to centre a wheel tooth on
    // +X in the z = 0 mid-plane.
    Part placed_wheel = Rot(0, 0, -wheel.twist_angle / 2) * wheel;
    placed_worm.label = "worm";
    placed_wheel.label = "wheel";
    children = std::vector<Shape>{placed_worm, placed_wheel};
    RigidJoint("wheel", *this, Location());
    RigidJoint("worm", *this, Pos(center_distance, 0, 0) * Rot(-90, 0, 0));
}

}  // namespace bd_warehouse
