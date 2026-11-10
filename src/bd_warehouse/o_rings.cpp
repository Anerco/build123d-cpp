// Port of bd_warehouse's o_rings.py.
#include "bd_warehouse/o_rings.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "bd_warehouse/fastener.hpp"
#include "data.hpp"
#include "fastener_internal.hpp"

namespace bd_warehouse {

using namespace build123d;

namespace {

/// Python's f"{x:g}".
std::string format_g(double x) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%g", x);
    return buffer;
}

void check_backing_rings(int number_backing_rings) {
    if (number_backing_rings < 0 || number_backing_rings > 2) {
        throw std::invalid_argument("number_backing_rings must be 0, 1, or 2");
    }
}

void check_type(const std::string& o_ring_type) {
    const std::set<std::string> types = ORing::types();
    if (types.count(o_ring_type) == 0) {
        throw std::invalid_argument(o_ring_type + " invalid, must be one of " + detail::py_set_repr(types));
    }
}

using WidthFamilies = Dict<double, std::vector<std::pair<double, std::string>>>;

struct ORingIndexes {
    Dict<std::string, ParamTable> parameters_by_type;
    Dict<std::string, WidthFamilies> width_families_by_type;
};

/// _build_o_ring_indexes: the evaluated parameter and width-family indexes of each standard.
ORingIndexes build_o_ring_indexes(const StringTable& parameter_data) {
    if (parameter_data.empty()) throw std::invalid_argument("O-ring parameter data is empty");
    std::set<std::string> standards;
    for (const auto& [parameter, value] : parameter_data.begin()->second) {
        const std::size_t colon = parameter.find(':');
        if (colon != std::string::npos) standards.insert(parameter.substr(0, colon));
    }
    if (standards.empty()) throw std::invalid_argument("O-ring parameter headers contain no standards");

    ORingIndexes indexes;
    for (const auto& standard : standards) {
        const ParamTable standard_parameters =
            evaluate_parameter_dict_of_dict(isolate_fastener_type(standard, parameter_data));
        if (standard_parameters.empty()) {
            throw std::invalid_argument("O-ring parameter data for " + standard + " is empty");
        }
        WidthFamilies width_families;
        for (const auto& [size_code, dimensions] : standard_parameters) {
            const auto w = dimensions.get("w");
            const auto id = dimensions.get("id");
            if (!w || !id) {
                throw std::invalid_argument("O-ring parameter data for " + standard + ":" + size_code + " is incomplete");
            }
            width_families[w->number()].emplace_back(id->number(), size_code);
        }
        for (auto& [width, entries] : width_families) std::sort(entries.begin(), entries.end());
        indexes.parameters_by_type[standard] = standard_parameters;
        indexes.width_families_by_type[standard] = width_families;
    }
    return indexes;
}

const ORingIndexes& o_ring_indexes() {
    static const ORingIndexes indexes = build_o_ring_indexes(ORing::o_ring_data());
    return indexes;
}

}  // namespace

NumberTable read_parameters_from_csv(const std::string& filename) {
    NumberTable parameters;
    const detail::DictReader reader = detail::dict_reader(detail::data_file("data/" + filename));
    if (!reader.fieldnames || reader.fieldnames->empty()) throw std::invalid_argument("No header found in " + filename);
    const std::string& first = reader.fieldnames->at(0);
    for (const auto& row : reader.rows) {
        if (row.rest) throw std::invalid_argument("'list' object has no attribute 'strip'");
        Dict<std::string, std::optional<double>> values;
        for (const auto& [name, value] : row.fields) {
            if (name == first) continue;
            if (!value) throw std::invalid_argument("'NoneType' object has no attribute 'strip'");
            values[name] = detail::py_strip(*value) == "-" ? std::nullopt : std::optional<double>(detail::py_float(*value));
        }
        parameters[row.str(first)] = values;
    }
    return parameters;
}

// ---------------------------------------------------------------------------
// ORing: tables and class functions
// ---------------------------------------------------------------------------

const StringTable& ORing::o_ring_data() {
    static const StringTable table = read_fastener_parameters_from_csv("o-ring_parameters.csv");
    return table;
}

const Dict<std::string, ParamTable>& ORing::o_ring_parameters() { return o_ring_indexes().parameters_by_type; }

const Dict<std::string, Dict<double, std::vector<std::pair<double, std::string>>>>& ORing::o_ring_width_families() {
    return o_ring_indexes().width_families_by_type;
}

const NumberTable& ORing::o_ring_static_gland_data() {
    static const NumberTable table = read_parameters_from_csv("o-ring_static_gland_parameters.csv");
    return table;
}

const NumberTable& ORing::o_ring_dynamic_gland_data() {
    static const NumberTable table = read_parameters_from_csv("o-ring_dynamic_gland_parameters.csv");
    return table;
}

std::string ORing::_normalize_size(const std::string& size) {
    std::string normalized = detail::py_strip(size);
    if (!normalized.empty() && normalized[0] == '-') normalized = normalized.substr(1);
    const bool digits = !normalized.empty() && std::all_of(normalized.begin(), normalized.end(), [](char c) {
        return c >= '0' && c <= '9';
    });
    if (!digits || normalized.size() > 3) {
        throw std::invalid_argument(detail::py_repr(size) + " invalid, size must be a three-digit code");
    }
    return std::string(3 - normalized.size(), '0') + normalized;
}

std::set<std::string> ORing::types() {
    std::set<std::string> out;
    for (const auto& [standard, parameters] : o_ring_parameters()) out.insert(standard);
    return out;
}

std::vector<std::string> ORing::sizes(const std::string& o_ring_type) {
    check_type(o_ring_type);
    return o_ring_parameters().at(o_ring_type).keys();
}

ParamDict ORing::parameters(const std::string& size, const std::string& o_ring_type) {
    const std::string normalized_size = _normalize_size(size);
    check_type(o_ring_type);
    const auto found = o_ring_parameters().at(o_ring_type).get(normalized_size);
    if (!found) {
        throw std::invalid_argument(detail::py_repr(size) + " invalid, must be one of " +
                                    detail::py_list_repr(sizes(o_ring_type)));
    }
    return *found;
}

Dict<std::string, std::vector<std::string>> ORing::select_by_size(const std::string& size) {
    const std::string normalized_size = _normalize_size(size);
    std::vector<std::string> matching_types;
    for (const auto& o_ring_type : types()) {
        const auto type_sizes = sizes(o_ring_type);
        if (std::find(type_sizes.begin(), type_sizes.end(), normalized_size) != type_sizes.end()) {
            matching_types.push_back(o_ring_type);
        }
    }
    Dict<std::string, std::vector<std::string>> out;
    if (!matching_types.empty()) out["ORing"] = matching_types;
    return out;
}

ORingNeighbors ORing::_find_neighbors(double target, double tolerance, const std::string& o_ring_type,
                                      bool by_diameter) {
    if (target <= 0) throw std::invalid_argument("target must be greater than zero");
    if (tolerance < 0) throw std::invalid_argument("tolerance must be greater than or equal to zero");
    check_type(o_ring_type);

    ORingNeighbors results;
    for (const auto& [width, diameter_entries] : o_ring_width_families().at(o_ring_type)) {
        std::vector<std::pair<double, std::string>> entries;
        if (by_diameter) {
            entries = diameter_entries;
        } else {
            for (const auto& [inner_diameter, size_code] : diameter_entries) {
                entries.emplace_back(M_PI * (inner_diameter + width), size_code);
            }
        }
        std::vector<double> values;
        for (const auto& entry : entries) values.push_back(entry.first);
        const std::size_t index =
            static_cast<std::size_t>(std::lower_bound(values.begin(), values.end(), target) - values.begin());

        if (index < entries.size() && values[index] == target) {
            results[detail::py_str(width)] = {entries[index].second};
            continue;
        }
        std::optional<std::string> lower, upper;
        if (index > 0 && std::abs(values[index - 1] - target) / target <= tolerance) lower = entries[index - 1].second;
        if (index < entries.size() && std::abs(values[index] - target) / target <= tolerance) {
            upper = entries[index].second;
        }
        if (lower || upper) results[detail::py_str(width)] = {lower, upper};
    }
    return results;
}

ORingNeighbors ORing::select_by_inner_diameter(double inner_diameter, double tolerance,
                                               const std::string& o_ring_type) {
    return _find_neighbors(inner_diameter, tolerance, o_ring_type);
}

ORingNeighbors ORing::select_by_length(double length, double tolerance, const std::string& o_ring_type) {
    return _find_neighbors(length, tolerance, o_ring_type, false);
}

std::vector<double> ORing::nominal_widths(const std::string& o_ring_type) {
    check_type(o_ring_type);
    std::vector<double> widths = o_ring_width_families().at(o_ring_type).keys();
    std::sort(widths.begin(), widths.end());
    return widths;
}

double ORing::gland_width_for(double o_ring_width, const std::string& application, int number_backing_rings) {
    if (application != "static" && application != "dynamic") {
        throw std::invalid_argument("application must be 'static' or 'dynamic'");
    }
    check_backing_rings(number_backing_rings);
    if (!std::isfinite(o_ring_width) || o_ring_width <= 0) {
        throw std::invalid_argument("o_ring_width must be greater than zero");
    }
    const NumberTable& gland_table = application == "static" ? o_ring_static_gland_data() : o_ring_dynamic_gland_data();
    const auto gland_data = gland_table.get(format_g(o_ring_width));
    if (!gland_data) {
        throw std::invalid_argument("No " + application + " gland data for an O-ring with a " +
                                    detail::py_str(o_ring_width) + " mm cross-section");
    }
    const std::optional<double> gland_width = gland_data->at("g_w_" + std::to_string(number_backing_rings));
    if (!gland_width) {
        throw std::invalid_argument(std::to_string(number_backing_rings) +
                                    " backing rings are not supported for an O-ring with a " +
                                    detail::py_str(o_ring_width) + " mm cross-section");
    }
    return *gland_width;
}

// ---------------------------------------------------------------------------
// ORing
// ---------------------------------------------------------------------------

ORing::ORing(const std::string& size, const std::string& o_ring_type_, const Rotation& rotation,
             std::optional<Align3> align)
    : ORing(size, ORingOptions{o_ring_type_, rotation, align}) {}

ORing::ORing(const std::string& size, const ORingOptions& o) {
    o_ring_size = _normalize_size(size);
    o_ring_type = o.o_ring_type;
    const ParamDict dimensions = parameters(o_ring_size, o_ring_type);

    dash_number = o_ring_size;
    inner_diameter = dimensions.at("id").number();
    inner_diameter_tolerance = dimensions.at("id_tol").number();
    width = dimensions.at("w").number();
    width_tolerance = dimensions.at("w_tol").number();
    major_radius = (inner_diameter + width) / 2;
    minor_radius = width / 2;
    length = 2 * M_PI * major_radius;

    const Solid o_ring = Solid::make_torus(major_radius, minor_radius);
    gland_width = std::nullopt;
    gland_width_tol = 0.13;
    gland_depth = std::nullopt;
    gland_depth_tol = std::nullopt;
    gland_radius = std::nullopt;
    gland_radius_tol = std::nullopt;
    gland_profile = std::nullopt;

    assign(o_ring, o.rotation, o.align);
    label = "ORing-" + o_ring_type + "-" + o_ring_size;
    color = Color(0x202020);
    RigidJoint("a", *this, Location());
}

std::string ORing::info() const { return "ORing(" + o_ring_type + "): " + o_ring_size; }

const Dict<std::string, std::optional<double>>& ORing::_gland_data_for_width(const NumberTable& gland_table,
                                                                             const std::string& application) const {
    const std::string key = format_g(width);
    if (!gland_table.contains(key)) {
        throw std::invalid_argument("No " + application + " gland data for an O-ring with a " + detail::py_str(width) +
                                    " mm cross-section");
    }
    return gland_table.at(key);
}

Sketch ORing::_make_gland_profile(const Dict<std::string, std::optional<double>>& gland_data,
                                  const std::string& depth_min_key, const std::string& depth_max_key,
                                  int number_backing_rings) {
    check_backing_rings(number_backing_rings);
    const std::optional<double> gland_w = gland_data.at("g_w_" + std::to_string(number_backing_rings));
    if (!gland_w) {
        throw std::invalid_argument(std::to_string(number_backing_rings) +
                                    " backing rings are not supported for an O-ring with a " + detail::py_str(width) +
                                    " mm cross-section");
    }
    const std::optional<double> depth_min = gland_data.at(depth_min_key);
    const std::optional<double> depth_max = gland_data.at(depth_max_key);
    if (!depth_min || !depth_max) throw std::invalid_argument("Gland depth data is incomplete");
    const double depth_avg = (*depth_max + *depth_min) / 2;
    const std::optional<double> groove_r_min = gland_data.at("g_r_min");
    const std::optional<double> groove_r_max = gland_data.at("g_r_max");
    if (!groove_r_min || !groove_r_max) throw std::invalid_argument("Gland radius data is incomplete");
    const double groove_r_avg = (*groove_r_max + *groove_r_min) / 2;

    const Rectangle rectangle(*gland_w, depth_avg, Align2(Align::CENTER, Align::MAX));
    const Sketch profile(build123d::fillet(rectangle.vertices().group_by(Axis::Y)[0], groove_r_avg));
    gland_width = *gland_w;
    gland_depth = depth_avg;
    gland_depth_tol = *depth_max - depth_avg;
    gland_radius = groove_r_avg;
    gland_radius_tol = *groove_r_max - groove_r_avg;
    gland_profile = profile;
    return profile;
}

Sketch ORing::static_gland_profile(bool axial, int number_backing_rings) {
    const auto& gland_data = _gland_data_for_width(o_ring_static_gland_data(), "static");
    const std::string axial_radial = axial ? "a" : "r";
    return _make_gland_profile(gland_data, "d_" + axial_radial + "_min", "d_" + axial_radial + "_max",
                               number_backing_rings);
}

Sketch ORing::dynamic_gland_profile(int number_backing_rings) {
    const auto& gland_data = _gland_data_for_width(o_ring_dynamic_gland_data(), "dynamic");
    return _make_gland_profile(gland_data, "d_min", "d_max", number_backing_rings);
}

}  // namespace bd_warehouse
