// bd_warehouse-cpp: pipes (bd_warehouse's pipe.py).
//
// Standard pipes (ASME B36.10/B36.19 steel and stainless, iron, copper types K/L/M, ABS and PVC)
// by nominal pipe size (NPS) and schedule, as a cross section or swept along a path.
//
//   using namespace bd_warehouse;
//   Pipe pipe("1", "copper", "K", Edge::make_line(Vector(0, 0, 0), Vector(0, 0, 100)));
//   // pipe.od, pipe.id, pipe.thickness; joints "inlet" and "outlet"
//
// The material resolves through the material catalog (bd_materials): as in bd_warehouse 0.4.0
// with bd_materials, "iron", "pvc" and "steel" are not catalog names, so those pipes throw
// std::invalid_argument ("unknown material 'steel' -- did you mean ...").
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <build123d/build123d.hpp>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse {

using build123d::Align;
using build123d::Align2;
using build123d::Align3;
using build123d::Edge;
using build123d::Rotation;
using build123d::Wire;

/// The nominal pipe sizes (Python's Literal Nps of pipe.py): "1/8", "1/4", ... "42".
const std::vector<std::string>& pipe_nps();
/// The pipe identifiers, i.e. schedules or types (Python's Literal Identifier): "K", "L", "M",
/// "STD", "XS", "XXS", "5S", "10", "10S", "20", "30", "40", "40S", "60", "80", "80S", "100",
/// "120", "140", "160".
const std::vector<std::string>& pipe_identifiers();
/// The pipe materials (Python's Literal Material): "abs", "copper", "iron", "pvc", "stainless",
/// "steel".
const std::vector<std::string>& pipe_materials();

/// Pipe Section: the cross section of a pipe with the given parameters (an annulus on XY).
///
/// Args:
///     nps: nominal pipe size (see pipe_nps()).
///     material: material type (see pipe_materials()).
///     identifier: pipe identifier, i.e. schedule or type (see pipe_identifiers()).
///     align: alignment. Defaults to Align::CENTER.
///
/// Raises:
///     std::invalid_argument: Invalid nps; Invalid identifier; Invalid material; No pipe data for
///         the combination; a material the catalog doesn't know.
class PipeSection : public build123d::BaseSketchObject {
public:
    /// The pipe table (pipe.csv): (outside diameter, wall thickness) in inches by
    /// nps + material + identifier (e.g. "1copperK").
    static const Dict<std::string, std::pair<double, double>>& pipe_data();

    PipeSection(const std::string& nps, const std::string& material, const std::string& identifier,
                std::optional<Align2> align = Align2(Align::CENTER));

    /// The nominal pipe size.
    std::string nps;
    /// The identifier (schedule or type).
    std::string sch;
    /// Outside diameter (mm).
    double od = 0.0;
    /// Wall thickness (mm).
    double thickness = 0.0;
    /// Inside diameter (mm): od - 2 thickness.
    double id = 0.0;
};

/// Pipe: parametric pipes of standard sizes and materials with its center following the provided
/// path parameter (each edge of the path swept with the pipe's section).
///
/// Args:
///     nps: nominal pipe size.
///     material: material type.
///     identifier: pipe identifier, i.e. schedule or type.
///     path: center line path of the pipe, an Edge or a Wire (bd_warehouse's builder-mode
///         default, the builder's pending edges, needs a builder).
///     rotation: rotations about axes. Defaults to no rotation.
///     align: alignment. Defaults to nullopt.
///
/// Joints:
///     "inlet" (Rigid): inlet end of the pipe.
///     "outlet" (Rigid): outlet end of the pipe.
class Pipe : public build123d::BasePartObject {
public:
    Pipe(const std::string& nps, const std::string& material, const std::string& identifier, const Edge& path,
         const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);
    Pipe(const std::string& nps, const std::string& material, const std::string& identifier, const Wire& path,
         const Rotation& rotation = Rotation(), std::optional<Align3> align = std::nullopt);

    /// Outside diameter, inside diameter and wall thickness (mm).
    double od = 0.0;
    double id = 0.0;
    double thickness = 0.0;
    /// The length of the path.
    double length = 0.0;

private:
    void init(const std::string& nps, const std::string& material, const std::string& identifier,
              const std::vector<Edge>& path_edges, const Rotation& rotation, std::optional<Align3> align);
};

}  // namespace bd_warehouse
