// Viewers: Mesher._mesh_shape, the arrays of vtk_tools.to_vtk_poly_data, to_vtkpoly_string and
// jupyter_tools.shape_to_html, without VTK. The normals follow VTK 9's vtkPolyDataNormals with
// a feature angle of 360 (vtkTriangle::ComputeNormal per cell; per point, the normalized float
// sum of the normals of its cells, in cell order).
#include "build123d/viewers.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <stdexcept>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include "build123d/json.hpp"

namespace build123d {

namespace {

const char* const kTemplateJs =
#include "template_render.inc"
    ;

// --- VTK's normal computations ------------------------------------------------------------

// vtkTriangle::ComputeNormal: (v3 - v2) x (v1 - v2), normalized unless zero.
std::array<float, 3> triangle_normal(const std::array<float, 3>& p1, const std::array<float, 3>& p2,
                                     const std::array<float, 3>& p3) {
    const double v1[3] = {p1[0], p1[1], p1[2]};
    const double v2[3] = {p2[0], p2[1], p2[2]};
    const double v3[3] = {p3[0], p3[1], p3[2]};
    const double ax = v3[0] - v2[0], ay = v3[1] - v2[1], az = v3[2] - v2[2];
    const double bx = v1[0] - v2[0], by = v1[1] - v2[1], bz = v1[2] - v2[2];
    double n[3] = {ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx};
    const double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (length != 0.0) {
        n[0] /= length;
        n[1] /= length;
        n[2] /= length;
    }
    return {static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2])};
}

// --- VTK XML ------------------------------------------------------------------------------

std::string base64(const std::string& bytes) {
    static const char* const table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < bytes.size(); i += 3) {
        const auto b0 = static_cast<unsigned char>(bytes[i]), b1 = static_cast<unsigned char>(bytes[i + 1]),
                   b2 = static_cast<unsigned char>(bytes[i + 2]);
        out += table[b0 >> 2];
        out += table[((b0 & 0x03) << 4) | (b1 >> 4)];
        out += table[((b1 & 0x0f) << 2) | (b2 >> 6)];
        out += table[b2 & 0x3f];
    }
    if (i < bytes.size()) {
        const auto b0 = static_cast<unsigned char>(bytes[i]);
        const auto b1 = i + 1 < bytes.size() ? static_cast<unsigned char>(bytes[i + 1]) : 0;
        out += table[b0 >> 2];
        out += table[((b0 & 0x03) << 4) | (b1 >> 4)];
        out += i + 1 < bytes.size() ? table[(b1 & 0x0f) << 2] : '=';
        out += '=';
    }
    return out;
}

bool little_endian() {
    const std::uint16_t one = 1;
    unsigned char first = 0;
    std::memcpy(&first, &one, 1);
    return first == 1;
}

// A binary (base64, uncompressed) DataArray: a UInt32 byte count followed by the values.
template <class T>
std::string binary_payload(const std::vector<T>& values) {
    std::string bytes(sizeof(std::uint32_t) + values.size() * sizeof(T), '\0');
    const auto count = static_cast<std::uint32_t>(values.size() * sizeof(T));
    std::memcpy(bytes.data(), &count, sizeof(count));
    if (!values.empty()) std::memcpy(bytes.data() + sizeof(count), values.data(), values.size() * sizeof(T));
    return base64(bytes);
}

std::string range_text(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.11g", v);
    return buf;
}

// RangeMin/RangeMax of a 3-component array: the range of the vector magnitudes, as VTK writes.
std::string float3_array(const char* name, const std::vector<std::array<float, 3>>& values, const std::string& indent) {
    std::vector<float> flat;
    flat.reserve(values.size() * 3);
    double lo = 0.0, hi = 0.0;
    bool first = true;
    for (const auto& v : values) {
        flat.insert(flat.end(), v.begin(), v.end());
        const double magnitude = std::sqrt(static_cast<double>(v[0]) * v[0] + static_cast<double>(v[1]) * v[1] +
                                           static_cast<double>(v[2]) * v[2]);
        lo = first ? magnitude : std::min(lo, magnitude);
        hi = first ? magnitude : std::max(hi, magnitude);
        first = false;
    }
    std::string range;
    if (!values.empty()) range = " RangeMin=\"" + range_text(lo) + "\" RangeMax=\"" + range_text(hi) + "\"";
    return indent + "<DataArray type=\"Float32\" Name=\"" + name + "\" NumberOfComponents=\"3\" format=\"binary\"" +
           range + ">\n" + indent + "  " + binary_payload(flat) + "\n" + indent + "</DataArray>\n";
}

std::string cell_arrays(const char* element, const std::vector<std::int64_t>& connectivity,
                        const std::vector<std::int64_t>& offsets) {
    const std::string indent = "        ";
    std::string out = "      <" + std::string(element) + ">\n";
    out += indent + "<DataArray type=\"Int64\" Name=\"connectivity\" format=\"binary\">\n" + indent + "  " +
           binary_payload(connectivity) + "\n" + indent + "</DataArray>\n";
    out += indent + "<DataArray type=\"Int64\" Name=\"offsets\" format=\"binary\">\n" + indent + "  " +
           binary_payload(offsets) + "\n" + indent + "</DataArray>\n";
    return out + "      </" + std::string(element) + ">\n";
}

// --- string.Template ----------------------------------------------------------------------

bool identifier_start(char c) { return c == '_' || std::isalpha(static_cast<unsigned char>(c)); }
bool identifier_char(char c) { return c == '_' || std::isalnum(static_cast<unsigned char>(c)); }

// Python's string.Template(template).substitute(mapping).
std::string substitute(const std::string& text, const std::map<std::string, std::string>& mapping) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] != '$') {
            out += text[i];
            continue;
        }
        if (i + 1 < text.size() && text[i + 1] == '$') {
            out += '$';
            ++i;
            continue;
        }
        std::string name;
        std::size_t end = i + 1;
        if (end < text.size() && text[end] == '{') {
            std::size_t close = end + 1;
            while (close < text.size() && identifier_char(text[close])) ++close;
            if (close < text.size() && text[close] == '}' && close > end + 1 && identifier_start(text[end + 1])) {
                name = text.substr(end + 1, close - end - 1);
                end = close + 1;
            }
        } else if (end < text.size() && identifier_start(text[end])) {
            while (end < text.size() && identifier_char(text[end])) ++end;
            name = text.substr(i + 1, end - i - 1);
        }
        if (name.empty()) throw std::invalid_argument("Invalid placeholder in string");
        const auto it = mapping.find(name);
        if (it == mapping.end()) throw std::out_of_range(name);
        out += it->second;
        i = end - 1;
    }
    return out;
}

std::string random_hex(int digits) {
    std::random_device device;
    std::mt19937_64 generator(device());
    std::uniform_int_distribution<int> nibble(0, 15);
    std::string out;
    for (int i = 0; i < digits; ++i) out += "0123456789abcdef"[nibble(generator)];
    return out;
}

}  // namespace

ShapeMesh mesh_shape(const Shape& shape, double linear_deflection, double angular_deflection) {
    ShapeMesh mesh;
    if (shape.is_null()) return mesh;
    BRepMesh_IncrementalMesh(shape.wrapped(), linear_deflection, true, angular_deflection, true);
    std::int64_t offset = 0;
    for (const auto& facet : shape.faces()) {
        TopLoc_Location loc;
        const TopoDS_Face& face = TopoDS::Face(facet.wrapped());
        const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
        if (triangulation.IsNull()) continue;
        const gp_Trsf trsf = loc.Transformation();
        const int node_count = triangulation->NbNodes();
        for (int i = 1; i <= node_count; ++i) mesh.vertices.emplace_back(triangulation->Node(i).Transformed(trsf));
        const bool facet_reversed = face.Orientation() == TopAbs_REVERSED;
        for (int t = 1; t <= triangulation->NbTriangles(); ++t) {
            int n1 = 0, n2 = 0, n3 = 0;
            triangulation->Triangle(t).Get(n1, n2, n3);
            if (facet_reversed) std::swap(n2, n3);
            mesh.triangles.push_back({n1 + offset - 1, n2 + offset - 1, n3 + offset - 1});
        }
        offset += node_count;
    }
    return mesh;
}

VtkMesh vtk_mesh(const Shape& obj, std::optional<double> tolerance, std::optional<double> angular_tolerance,
                 bool normals) {
    if (!obj) throw std::invalid_argument("Cannot convert an empty shape");
    // build123d: `tolerance if tolerance else 1e-3`
    const double deviation_coefficient = tolerance && *tolerance != 0.0 ? *tolerance : 1e-3;
    const double deviation_angle = angular_tolerance && *angular_tolerance != 0.0 ? *angular_tolerance : 0.1;
    const ShapeMesh mesh = mesh_shape(obj, deviation_coefficient, deviation_angle);

    VtkMesh out;
    out.points.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices) {
        out.points.push_back({static_cast<float>(v.X), static_cast<float>(v.Y), static_cast<float>(v.Z)});
    }
    out.polys = mesh.triangles;
    if (!normals || out.polys.empty()) return out;

    out.cell_normals.reserve(out.polys.size());
    for (const auto& tri : out.polys) {
        out.cell_normals.push_back(triangle_normal(out.points[static_cast<std::size_t>(tri[0])],
                                                   out.points[static_cast<std::size_t>(tri[1])],
                                                   out.points[static_cast<std::size_t>(tri[2])]));
    }
    out.point_normals.assign(out.points.size(), {0.0F, 0.0F, 0.0F});
    for (std::size_t c = 0; c < out.polys.size(); ++c) {
        for (const auto idx : out.polys[c]) {
            auto& pn = out.point_normals[static_cast<std::size_t>(idx)];
            for (int k = 0; k < 3; ++k) pn[k] = pn[k] + out.cell_normals[c][k];
        }
    }
    for (auto& pn : out.point_normals) {
        const float length = std::sqrt(pn[0] * pn[0] + pn[1] * pn[1] + pn[2] * pn[2]);
        if (length != 0.0F) {
            const auto scale = static_cast<float>(1.0 / static_cast<double>(length));
            for (auto& value : pn) value *= scale;
        }
    }
    return out;
}

std::string to_vtkpoly_string(const Shape& shape, double tolerance, double angular_tolerance) {
    const VtkMesh mesh = vtk_mesh(shape, tolerance, angular_tolerance, true);
    std::vector<std::int64_t> connectivity, offsets;
    connectivity.reserve(mesh.polys.size() * 3);
    offsets.reserve(mesh.polys.size());
    for (const auto& tri : mesh.polys) {
        connectivity.insert(connectivity.end(), tri.begin(), tri.end());
        offsets.push_back(static_cast<std::int64_t>(connectivity.size()));
    }
    const std::string ind = "        ";
    std::string xml = "<?xml version=\"1.0\"?>\n";
    xml += std::string("<VTKFile type=\"PolyData\" version=\"0.1\" byte_order=\"") +
           (little_endian() ? "LittleEndian" : "BigEndian") + "\" header_type=\"UInt32\">\n";
    xml += "  <PolyData>\n";
    xml += "    <Piece NumberOfPoints=\"" + std::to_string(mesh.points.size()) +
           "\" NumberOfVerts=\"0\" NumberOfLines=\"0\" NumberOfStrips=\"0\" NumberOfPolys=\"" +
           std::to_string(mesh.polys.size()) + "\">\n";
    if (!mesh.point_normals.empty()) {
        xml += "      <PointData Normals=\"Normals\">\n" + float3_array("Normals", mesh.point_normals, ind) +
               "      </PointData>\n";
        xml += "      <CellData Normals=\"Normals\">\n" + float3_array("Normals", mesh.cell_normals, ind) +
               "      </CellData>\n";
    } else {
        xml += "      <PointData>\n      </PointData>\n      <CellData>\n      </CellData>\n";
    }
    xml += "      <Points>\n" + float3_array("Points", mesh.points, ind) + "      </Points>\n";
    xml += cell_arrays("Verts", {}, {});
    xml += cell_arrays("Lines", {}, {});
    xml += cell_arrays("Strips", {}, {});
    xml += cell_arrays("Polys", connectivity, offsets);
    xml += "    </Piece>\n  </PolyData>\n</VTKFile>\n";
    return xml;
}

std::string shape_to_html(const Shape& shape) {
    const Json payload = Json::array({Json::object({
        {"shape", to_vtkpoly_string(shape)},
        {"color", Json::array({1, 0.8, 0, 1})},
        {"position", Json::array({0, 0, 0})},
        {"orientation", Json::array({0, 0, 0})},
    })});
    // A new div with a unique id, plus the JS code templated with the id
    const std::string div_id = "shape-" + random_hex(8);
    const std::string code =
        substitute(kTemplateJs, {{"data", payload.dump()}, {"div_id", div_id}, {"ratio", "0.5"}});
    return "<div id=" + div_id + "></div><script>" + code + "</script>";
}

}  // namespace build123d
