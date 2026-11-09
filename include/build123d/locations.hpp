// build123d-cpp: lists of locations (Locations, GridLocations, PolarLocations,
// HexLocations). In algebra mode they multiply shapes: GridLocations(5, 5, 2, 2) * Circle(1).
#pragma once

#include <initializer_list>
#include <vector>

#include "build123d/topology.hpp"

namespace build123d {

class LocationList : public std::vector<Location> {
public:
    using std::vector<Location>::vector;
    LocationList() = default;
    explicit LocationList(std::vector<Location> locations)
        : std::vector<Location>(std::move(locations)) {}
    const std::vector<Location>& locations() const { return *this; }
};

/// Explicit locations from points, Locations, Planes, Vertices or Faces.
class Locations : public LocationList {
public:
    Locations(std::initializer_list<Vector> points);
    Locations(std::initializer_list<Location> locations);
    explicit Locations(const std::vector<Vector>& points);
    explicit Locations(const std::vector<Location>& locations);
    explicit Locations(const std::vector<Plane>& planes);
    explicit Locations(const std::vector<Vertex>& vertices);
    explicit Locations(const std::vector<Face>& faces);
};

/// Rectangular array of locations.
class GridLocations : public LocationList {
public:
    GridLocations(double x_spacing, double y_spacing, int x_count, int y_count,
                  Align2 align = Align::CENTER);
    double x_spacing;
    double y_spacing;
    int x_count;
    int y_count;
    Vector size;
    Vector min;
    Vector max;
};

/// Locations on a circle, optionally rotated to follow it.
class PolarLocations : public LocationList {
public:
    PolarLocations(double radius, int count, double start_angle = 0.0,
                   double angular_range = 360.0, bool rotate = true, bool endpoint = false);
};

/// Hexagonal array of locations.
class HexLocations : public LocationList {
public:
    HexLocations(double radius, int x_count, int y_count, bool major_radius = false,
                 Align2 align = Align::CENTER);
    double radius;
    double apothem;
    double diagonal;
    int x_count;
    int y_count;
};

}  // namespace build123d
