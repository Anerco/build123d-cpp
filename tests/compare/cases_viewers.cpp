// Viewers (vtk_tools.py, jupyter_tools.py), compared with cases_viewers_py.py.
// Meshes from different OCCT versions list their triangles in different orders, so the arrays
// are compared as sorted rows. The .vtp files written here are read back by VTK in Python.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

std::filesystem::path out_dir() {
    const auto dir = std::filesystem::temp_directory_path() / "build123d_compare_viewers";
    std::filesystem::create_directories(dir);
    return dir;
}

double r5(double v) { return std::round(v * 1e5) / 1e5; }

// Points with their normals, and triangles as their sorted vertices plus the cell normal.
V summary(const VtkMesh& m) {
    std::vector<std::vector<double>> point_rows, tri_rows;
    for (std::size_t i = 0; i < m.points.size(); ++i) {
        std::vector<double> row;
        for (const float c : m.points[i]) row.push_back(r5(c));
        if (!m.point_normals.empty()) {
            for (const float c : m.point_normals[i]) row.push_back(r5(c));
        }
        point_rows.push_back(row);
    }
    for (std::size_t t = 0; t < m.polys.size(); ++t) {
        std::vector<std::vector<double>> verts;
        for (const auto idx : m.polys[t]) {
            const auto& p = m.points[static_cast<std::size_t>(idx)];
            verts.push_back({r5(p[0]), r5(p[1]), r5(p[2])});
        }
        std::sort(verts.begin(), verts.end());
        std::vector<double> row;
        for (const auto& v : verts) row.insert(row.end(), v.begin(), v.end());
        if (!m.cell_normals.empty()) {
            for (const float c : m.cell_normals[t]) row.push_back(r5(c));
        }
        tri_rows.push_back(row);
    }
    std::sort(point_rows.begin(), point_rows.end());
    std::sort(tri_rows.begin(), tri_rows.end());
    auto rows = [](const std::vector<std::vector<double>>& rs) {
        std::vector<V> out;
        for (const auto& r : rs) out.push_back(V::list_of(r));
        return V::list(out);
    };
    return V::obj({{"points", m.points.size()},
                   {"polys", m.polys.size()},
                   {"point_rows", rows(point_rows)},
                   {"tri_rows", rows(tri_rows)}});
}

Part holed() { return Box(4, 4, 2) - Cylinder(1, 2); }

}  // namespace

COMPARE_CASES {
    compare::add_value("viewers_vtk_mesh_box", [] { return summary(vtk_mesh(Box(1, 2, 3), 1e-3, 0.1, true)); });
    compare::add_value("viewers_vtk_mesh_holed", [] { return summary(vtk_mesh(holed(), 0.01, 0.3, true)); });
    compare::add_value("viewers_vtk_mesh_sphere", [] { return summary(vtk_mesh(Sphere(1), 0.05, 0.5, true)); });
    compare::add_value("viewers_vtk_mesh_no_normals", [] { return summary(vtk_mesh(Cylinder(1, 2), 0.05, 0.5)); });
    compare::add_value("viewers_vtk_mesh_empty", [] { return summary(vtk_mesh(Part())); });
    compare::add_value("viewers_vtkpoly_string", [] {
        // Written here, read back with VTK by the Python case.
        std::vector<V> out;
        for (const auto& [name, shape] : std::vector<std::pair<std::string, Shape>>{
                 {"box", Box(1, 2, 3)}, {"holed", holed()}, {"face", Rectangle(2, 3).face()}}) {
            std::ofstream(out_dir() / (name + ".vtp")) << to_vtkpoly_string(shape);
            out.push_back(summary(vtk_mesh(shape, 1e-3, 0.1, true)));
        }
        return V::list(out);
    });
    compare::add_value("viewers_shape_to_html", [] {
        const std::string html = shape_to_html(Box(1, 2, 3));
        std::ofstream(out_dir() / "box.html") << html;
        // The div id is random; the payload holds the VTK text, compared by the vtk cases.
        const auto id_start = html.find("shape-");
        const std::string div_id = html.substr(id_start, 14);
        std::string skeleton = html;
        for (auto pos = skeleton.find(div_id); pos != std::string::npos; pos = skeleton.find(div_id)) {
            skeleton.replace(pos, div_id.size(), "DIV");
        }
        const auto data_start = skeleton.find("const data = ") + 13;
        const auto data_end = skeleton.find(";\n", data_start);
        const Json payload = Json::parse(skeleton.substr(data_start, data_end - data_start));
        skeleton.replace(data_start, data_end - data_start, "DATA");
        const Json& item = payload.as_array().at(0);
        std::vector<std::string> keys;
        for (const auto& [key, value] : item.as_object()) keys.push_back(key);
        return V::obj({{"skeleton", skeleton},
                       {"id_length", div_id.size()},
                       {"keys", V::list_of(keys)},
                       {"color", item["color"].dump()},
                       {"position", item["position"].dump()},
                       {"orientation", item["orientation"].dump()},
                       {"shape_starts", item["shape"].as_string().substr(0, 21)}});
    });
}
