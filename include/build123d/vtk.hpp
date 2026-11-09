// build123d's to_vtk_poly_data for programs that use VTK (vtk_tools.py). This header needs the
// VTK headers and libraries (CommonCore, CommonDataModel, FiltersCore); the build123d library
// itself doesn't depend on VTK.
#pragma once

#include <optional>
#include <stdexcept>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "build123d/viewers.hpp"

namespace build123d {

/// The shape's mesh as vtkPolyData; with `normals`, point and cell normals are added by
/// vtkPolyDataNormals with a feature angle of 360. `tolerance` and `angular_tolerance`
/// default to 1e-3 and 0.1.
inline vtkSmartPointer<vtkPolyData> to_vtk_poly_data(const Shape& obj, std::optional<double> tolerance = std::nullopt,
                                                     std::optional<double> angular_tolerance = std::nullopt,
                                                     bool normals = false) {
    if (!obj) throw std::invalid_argument("Cannot convert an empty shape");
    // build123d: `tolerance if tolerance else 1e-3`
    const double deviation_coefficient = tolerance && *tolerance != 0.0 ? *tolerance : 1e-3;
    const double deviation_angle = angular_tolerance && *angular_tolerance != 0.0 ? *angular_tolerance : 0.1;
    const ShapeMesh mesh = mesh_shape(obj, deviation_coefficient, deviation_angle);

    auto points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& v : mesh.vertices) points->InsertNextPoint(v.X, v.Y, v.Z);
    auto triangle_cells = vtkSmartPointer<vtkCellArray>::New();
    for (const auto& tri : mesh.triangles) {
        triangle_cells->InsertNextCell(3);
        for (const auto idx : tri) triangle_cells->InsertCellPoint(idx);
    }
    auto poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPoints(points);
    poly_data->SetPolys(triangle_cells);

    auto t_filter = vtkSmartPointer<vtkTriangleFilter>::New();
    t_filter->SetInputData(poly_data);
    t_filter->Update();
    vtkSmartPointer<vtkPolyData> result = t_filter->GetOutput();
    if (normals) {
        auto n_filter = vtkSmartPointer<vtkPolyDataNormals>::New();
        n_filter->SetComputePointNormals(true);
        n_filter->SetComputeCellNormals(true);
        n_filter->SetFeatureAngle(360);
        n_filter->SetInputData(result);
        n_filter->Update();
        result = n_filter->GetOutput();
    }
    return result;
}

}  // namespace build123d
