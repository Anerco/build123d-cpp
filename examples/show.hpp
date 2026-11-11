// Stand-in for the ocp_vscode viewer the Python examples use: show() prints the main
// properties of an object as one JSON line. tests/compare/compare_examples.py compares
// these lines with the same examples run in Python build123d.
#pragma once

#include <build123d/build123d.hpp>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <cstdio>
#include <iostream>
#include <string>

namespace example {

inline std::string number(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.12g", v);
    return buf;
}

/// Volume, area, total edge length, topology counts, bounding box and center of mass.
inline void show(const build123d::Shape& shape, const std::string& name) {
    using namespace build123d;
    const auto edges = shape.edges();
    double length = 0;
    for (const auto& e : edges) length += e.length();
    const auto solids = shape.solids().size();
    const auto faces = shape.faces().size();
    GProp_GProps props;
    if (solids > 0) {
        BRepGProp::VolumeProperties(shape.wrapped(), props);
    } else if (faces > 0) {
        BRepGProp::SurfaceProperties(shape.wrapped(), props);
    } else {
        BRepGProp::LinearProperties(shape.wrapped(), props);
    }
    const gp_Pnt c = props.CentreOfMass();
    const BoundBox bb = shape.bounding_box();
    std::cout << "{\"name\": \"" << name << "\", \"volume\": " << number(shape.volume())
              << ", \"area\": " << number(shape.area()) << ", \"length\": " << number(length)
              << ", \"solids\": " << solids << ", \"faces\": " << faces << ", \"edges\": " << edges.size()
              << ", \"vertices\": " << shape.vertices().size() << ", \"bbox\": [" << number(bb.min.X) << ", "
              << number(bb.min.Y) << ", " << number(bb.min.Z) << ", " << number(bb.max.X) << ", "
              << number(bb.max.Y) << ", " << number(bb.max.Z) << "], \"center\": [" << number(c.X()) << ", "
              << number(c.Y()) << ", " << number(c.Z()) << "]}" << std::endl;
}

}  // namespace example
