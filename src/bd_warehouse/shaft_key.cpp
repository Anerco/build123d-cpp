// Port of bd_warehouse's shaft_key.py.
#include "bd_warehouse/shaft_key.hpp"

#include <array>
#include <cmath>
#include <cstdio>

#include "bd_warehouse/fastener.hpp"
#include "builder.hpp"
#include "data.hpp"

namespace bd_warehouse {

using namespace build123d;

namespace {

/// Python's f"{x:g}".
std::string format_g(double x) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%g", x);
    return buffer;
}

// ISO 286-1:2010 Tables 1 to 3, limited to the DIN 6885 key widths. Values are upper size limit,
// IT9, IT10, D lower deviation, and P upper deviation, respectively, expressed in micrometres
// except for the size limit.
constexpr std::array<std::array<double, 5>, 4> kIso286HoleRanges = {{
    {3, 25, 40, 20, -6},
    {6, 30, 48, 30, -12},
    {10, 36, 58, 40, -15},
    {18, 43, 70, 50, -18},
}};

/// _iso_286_hole_deviations: the lower and upper hole deviations in millimetres.
std::pair<double, double> iso_286_hole_deviations(double nominal_size, const std::string& tolerance_class) {
    const std::array<double, 5>* range = nullptr;
    for (const auto& values : kIso286HoleRanges) {
        if (nominal_size <= values[0]) {
            range = &values;
            break;
        }
    }
    if (range == nullptr) {
        throw std::invalid_argument("No ISO 286 hole deviations available for " + format_g(nominal_size) + " mm");
    }
    const double it9 = (*range)[1], it10 = (*range)[2], d_lower = (*range)[3], p_upper = (*range)[4];
    std::pair<double, double> deviations;
    if (tolerance_class == "D10") {
        deviations = {d_lower, d_lower + it10};
    } else if (tolerance_class == "H9") {
        deviations = {0, it9};
    } else if (tolerance_class == "JS9") {
        deviations = {-it9 / 2, it9 / 2};
    } else if (tolerance_class == "N9") {
        deviations = {-it9, 0};
    } else if (tolerance_class == "P9") {
        deviations = {p_upper - it9, p_upper};
    } else {
        throw std::invalid_argument("Unsupported ISO 286 hole tolerance " + tolerance_class);
    }
    return {deviations.first / 1000, deviations.second / 1000};
}

/// _key_plan: a private nominal plan for a DIN 6885 parallel key.
Face key_plan(double length, double width, const std::string& key_form) {
    detail::Builder plan(2);
    if (key_form == "A") {
        plan.add(SlotOverall(length, width));
    } else {
        plan.add(Rectangle(length, width));
    }
    return plan.face();
}

/// Python's str() of a validated (whole) shaft diameter as the caller passed it: an int.
std::string diameter_text(double shaft_diameter) {
    if (std::isfinite(shaft_diameter) && std::trunc(shaft_diameter) == shaft_diameter &&
        std::fabs(shaft_diameter) < 1e15) {
        return std::to_string(static_cast<long long>(shaft_diameter));
    }
    return detail::py_str(shaft_diameter);
}

}  // namespace

// ---------------------------------------------------------------------------
// ShaftKey
// ---------------------------------------------------------------------------

const std::string ShaftKey::standard = "DIN6885-1";

const Dict<int, ParamDict>& ShaftKey::key_data() {
    static const Dict<int, ParamDict> table = [] {
        const ParamTable evaluated_parameters = evaluate_parameter_dict_of_dict(
            isolate_fastener_type("din6885", read_fastener_parameters_from_csv("shaft_key_parameters.csv")));
        Dict<int, ParamDict> out;
        for (const auto& [shaft_diameter, dimensions] : evaluated_parameters) {
            out[static_cast<int>(detail::py_int_str(shaft_diameter))] = dimensions;
        }
        return out;
    }();
    return table;
}

std::vector<int> ShaftKey::sizes() { return key_data().keys(); }

int ShaftKey::_validate_shaft_diameter(double shaft_diameter) {
    if (!std::isfinite(shaft_diameter) || std::trunc(shaft_diameter) != shaft_diameter) {
        throw std::invalid_argument("shaft_diameter must be a whole number of millimetres");
    }
    if (shaft_diameter <= 0) throw std::invalid_argument("shaft_diameter must be greater than zero");
    // (a diameter too large for an int is in no table)
    return shaft_diameter > 1e9 ? -1 : static_cast<int>(shaft_diameter);
}

ParamDict ShaftKey::parameters(double shaft_diameter) {
    const int validated_diameter = _validate_shaft_diameter(shaft_diameter);
    const auto found = key_data().get(validated_diameter);
    if (!found) {
        std::string sizes_text = "[";
        const std::vector<int> all = sizes();
        for (std::size_t i = 0; i < all.size(); ++i) sizes_text += (i ? ", " : "") + std::to_string(all[i]);
        throw std::invalid_argument(diameter_text(shaft_diameter) + " invalid, must be one of " + sizes_text + "]");
    }
    return *found;
}

Dict<std::string, std::vector<std::string>> ShaftKey::select_by_size(double shaft_diameter) {
    const int validated_diameter = _validate_shaft_diameter(shaft_diameter);
    Dict<std::string, std::vector<std::string>> out;
    if (key_data().contains(validated_diameter)) out["ShaftKey"] = {standard};
    return out;
}

ShaftKey::ShaftKey(double shaft_diameter_, double length_, const std::string& key_form_, const Rotation& rotation,
                   std::optional<Align3> align)
    : ShaftKey(shaft_diameter_, length_, ShaftKeyOptions{key_form_, rotation, align}) {}

ShaftKey::ShaftKey(double shaft_diameter_, double length_, const ShaftKeyOptions& o) {
    if (o.key_form != "A" && o.key_form != "B") throw std::invalid_argument("key_form must be 'A' or 'B'");
    key_form = o.key_form;
    shaft_diameter = _validate_shaft_diameter(shaft_diameter_);
    key_dict = parameters(shaft_diameter);
    length = length_;
    if (length <= 0) throw std::invalid_argument("length must be greater than zero");

    width = key_dict.at("b").number();
    key_height = key_dict.at("h").number();
    if (length < width) throw std::invalid_argument("length must be greater than or equal to the key width");

    assign(make_key(), o.rotation, o.align);
    label = "ShaftKey-" + standard + "-Form" + key_form + "-" + std::to_string(shaft_diameter) + "x" + format_g(length);
    color = Color(0x909090);
    RigidJoint("a", *this, Location());
}

std::string ShaftKey::info() const {
    return "ShaftKey(" + standard + ", Form " + key_form + "): " + format_g(shaft_diameter) + "x" + format_g(length);
}

double ShaftKey::plan_area() const {
    if (key_form == "A") return width * (length - width) + M_PI * std::pow(width, 2) / 4;
    return length * width;
}

Solid ShaftKey::make_key() const { return Solid::extrude(key_plan(length, width, key_form), Vector(0, 0, key_height)); }

// ---------------------------------------------------------------------------
// Keyway
// ---------------------------------------------------------------------------

const Dict<std::string, Dict<std::string, std::string>>& Keyway::keyway_fits() {
    static const Dict<std::string, Dict<std::string, std::string>> fits = {
        {"Tight", {{"hub", "P9"}, {"shaft", "P9"}}},
        {"Loose", {{"hub", "JS9"}, {"shaft", "N9"}}},
        {"Sliding", {{"hub", "D10"}, {"shaft", "H9"}}},
    };
    return fits;
}

Keyway::Keyway(const ShaftKey& key_, const std::string& keyway_type_, const std::string& fit_,
               const Rotation& rotation, std::optional<Align3> align)
    : Keyway(key_, KeywayOptions{keyway_type_, fit_, rotation, align}) {}

Keyway::Keyway(const ShaftKey& key_, const KeywayOptions& o) : key(key_) {
    if (o.keyway_type != "shaft" && o.keyway_type != "hub") {
        throw std::invalid_argument("keyway_type must be 'shaft' or 'hub'");
    }
    if (!keyway_fits().contains(o.fit)) {
        throw std::invalid_argument("fit must be one of " + detail::py_list_repr(keyway_fits().keys()));
    }
    keyway_type = o.keyway_type;
    fit = o.fit;
    nominal_width = key.width;
    length = key.length;
    const bool shaft = keyway_type == "shaft";
    keyway_depth = key.key_dict.at(shaft ? "t4" : "t2").number();
    keyway_depth_tolerance = key.key_dict.at(shaft ? "t4_tol+" : "t2_tol+").number();
    width_tolerance = keyway_fits().at(fit).at(keyway_type);
    const auto [lower_deviation, upper_deviation] = iso_286_hole_deviations(nominal_width, width_tolerance);
    min_width = nominal_width + lower_deviation;
    max_width = nominal_width + upper_deviation;
    width = (min_width + max_width) / 2;

    const Solid cutter = Solid::extrude(key_plan(length, width, key.key_form), Vector(0, 0, -keyway_depth))
                             .rotate(Axis::Y, shaft ? 90 : -90)
                             .translate(Vector(key.shaft_diameter / 2.0, 0, 0));
    assign(cutter, o.rotation, o.align);
    label = "Keyway-" + keyway_type + "-" + fit + "-" + ShaftKey::standard + "-Form" + key.key_form + "-" +
            std::to_string(key.shaft_diameter) + "x" + format_g(key.length);
}

Face Keyway::broach_profile() const {
    if (keyway_type != "hub") throw std::invalid_argument("broach_profile is only available for hub keyways");
    const double profile_depth = key.shaft_diameter / 2.0 + keyway_depth;
    return Face::make_rect(profile_depth, width, Plane(Vector(profile_depth / 2, 0, 0)));
}

}  // namespace bd_warehouse
