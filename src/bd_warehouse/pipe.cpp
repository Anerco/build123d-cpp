// Port of bd_warehouse's pipe.py.
#include "bd_warehouse/pipe.hpp"

#include <algorithm>

#include "builder.hpp"
#include "data.hpp"

namespace bd_warehouse {

using namespace build123d;
using detail::Builder;
using detail::Mode;

const std::vector<std::string>& pipe_nps() {
    static const std::vector<std::string> values = {
        "1/8", "1/4", "3/8", "1/2", "3/4", "1",  "1 1/4", "1 1/2", "2",  "2 1/2", "3",  "4",  "5",  "6",
        "8",   "10",  "12",  "14",  "16",  "18", "20",    "22",    "24", "30",    "32", "34", "36", "42"};
    return values;
}

const std::vector<std::string>& pipe_identifiers() {
    static const std::vector<std::string> values = {"K",  "L",  "M",   "STD", "XS", "XXS", "5S",  "10",  "10S", "20",
                                                    "30", "40", "40S", "60",  "80", "80S", "100", "120", "140", "160"};
    return values;
}

const std::vector<std::string>& pipe_materials() {
    static const std::vector<std::string> values = {"abs", "copper", "iron", "pvc", "stainless", "steel"};
    return values;
}

namespace {

bool contains(const std::vector<std::string>& values, const std::string& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

const Dict<std::string, std::pair<double, double>>& PipeSection::pipe_data() {
    // _read_pipe_data: {nps + material + identifier: (od, thickness)} in inches
    static const Dict<std::string, std::pair<double, double>> table = [] {
        Dict<std::string, std::pair<double, double>> pipe_data;
        const auto rows = detail::csv_reader(detail::data_file("pipe.csv"));
        for (std::size_t i = 1; i < rows.size(); ++i) {  // skip the header row
            const auto& row = rows[i];
            if (row.empty()) continue;  // skip blank rows
            if (row.size() != 5) {
                throw std::invalid_argument("too many values to unpack (expected 5, got " + std::to_string(row.size()) +
                                            ")");
            }
            pipe_data[row[0] + row[1] + row[2]] = {detail::py_float(row[3]), detail::py_float(row[4])};
        }
        return pipe_data;
    }();
    return table;
}

PipeSection::PipeSection(const std::string& nps_, const std::string& material, const std::string& identifier,
                         std::optional<Align2> align) {
    nps = nps_;
    sch = identifier;
    // bd_warehouse resolves the material first (an unknown name throws), then the object's
    // initialization leaves the section without a material
    (void)FinishedMaterial(material);

    if (!contains(pipe_nps(), nps)) {
        throw std::invalid_argument("Invalid nps value - the valid values are: " + detail::py_tuple_repr(pipe_nps()));
    }
    if (!contains(pipe_identifiers(), identifier)) {
        throw std::invalid_argument("Invalid identifier value - the valid values are: " +
                                    detail::py_tuple_repr(pipe_identifiers()));
    }
    if (!contains(pipe_materials(), material)) {
        throw std::invalid_argument("Invalid material value - the valid values are: " +
                                    detail::py_tuple_repr(pipe_materials()));
    }
    const auto found = pipe_data().get(nps + material + identifier);
    if (!found) throw std::invalid_argument("No pipe data for " + nps + ", " + material + ", " + identifier);

    od = found->first * IN;
    thickness = found->second * IN;
    id = od - 2 * thickness;

    Builder cross_section(2);
    cross_section.add(Circle(od / 2));
    cross_section.add(Circle(id / 2), Mode::SUBTRACT);
    assign(cross_section.sketch(), 0.0, align);
}

Pipe::Pipe(const std::string& nps, const std::string& material, const std::string& identifier, const Edge& path,
           const Rotation& rotation, std::optional<Align3> align) {
    init(nps, material, identifier, {path}, rotation, align);
}

Pipe::Pipe(const std::string& nps, const std::string& material, const std::string& identifier, const Wire& path,
           const Rotation& rotation, std::optional<Align3> align) {
    init(nps, material, identifier, std::vector<Edge>(path.edges()), rotation, align);
}

void Pipe::init(const std::string& nps, const std::string& material_name, const std::string& identifier,
                const std::vector<Edge>& path_edges, const Rotation& rotation, std::optional<Align3> align) {
    const PipeSection section(nps, material_name, identifier);
    od = section.od;
    id = section.id;
    thickness = section.thickness;
    length = 0;
    for (const auto& p : path_edges) length += p.length();

    Builder pipe(3);
    for (const auto& p : path_edges) {
        detail::insert(pipe, {p});
        {
            Builder sketch(2);
            detail::insert(sketch, {section});
            pipe.publish(sketch.product(), Mode::ADD, {Location()},
                         {Location(Plane(p.position_at(0), std::nullopt, p.tangent_at(0)))});
        }
        detail::sweep(pipe);
    }

    assign(pipe.part(), rotation, align);
    material = FinishedMaterial(material_name);

    // Add the joints
    if (path_edges.empty()) throw std::out_of_range("list index out of range");
    RigidJoint("inlet", *this,
               Location(Plane(path_edges.front().position_at(0), std::nullopt, -path_edges.front().tangent_at(0))));
    RigidJoint("outlet", *this, path_edges.back().location_at(1));
}

}  // namespace bd_warehouse
