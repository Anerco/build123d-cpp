// build123d-cpp: lists of locations (Locations, GridLocations, PolarLocations,
// HexLocations). In algebra mode they multiply shapes: GridLocations(5, 5, 2, 2) * Circle(1).
#pragma once

#include <initializer_list>
#include <type_traits>
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
    /// The locations in local construction coordinates (the same as locations() without a
    /// builder).
    const std::vector<Location>& local_locations() const { return *this; }
};

namespace detail {
template <class T>
inline constexpr bool is_locations_item_v =
    std::is_same_v<T, Vector> || std::is_base_of_v<Location, T> || std::is_same_v<T, Vertex> ||
    std::is_same_v<T, Face> || std::is_same_v<T, Plane> || std::is_same_v<T, Axis>;
template <class T, class = void>
struct is_locations_arg : std::bool_constant<is_locations_item_v<T>> {};
template <class T>
struct is_locations_arg<T, std::void_t<typename T::value_type>>
    : std::bool_constant<std::is_base_of_v<std::vector<typename T::value_type>, T> &&
                         is_locations_arg<typename T::value_type>::value> {};
template <class T>
inline constexpr bool is_locations_arg_v = is_locations_arg<std::decay_t<T>>::value;
}  // namespace detail

/// Explicit locations from points, Locations, Planes, Axes, Vertices or Faces, or any mix
/// of them and lists of them: Locations(Vector(1, 2), Pos(3, 4), Axis::X, faces).
class Locations : public LocationList {
public:
    Locations(std::initializer_list<Vector> points);
    Locations(std::initializer_list<Location> locations);
    explicit Locations(const std::vector<Vector>& points);
    explicit Locations(const std::vector<Location>& locations);
    explicit Locations(const std::vector<Plane>& planes);
    explicit Locations(const std::vector<Vertex>& vertices);
    explicit Locations(const std::vector<Face>& faces);
    explicit Locations(const std::vector<Axis>& axes);
    template <class... Ts,
              std::enable_if_t<(sizeof...(Ts) >= 1) && (detail::is_locations_arg_v<Ts> && ...), int> = 0>
    explicit Locations(const Ts&... pts) {
        (append(pts), ...);
    }

private:
    void append(const Vector& point);
    void append(const Location& location);
    void append(const Vertex& vertex);
    void append(const Face& face);
    void append(const Plane& plane);
    void append(const Axis& axis);
    template <class T>
    void append(const std::vector<T>& items) {
        for (const auto& item : items) append(item);
    }
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
    Align2 align;
    Vector size;
    Vector min;
    Vector max;
    std::vector<Plane> planes;
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
    bool major_radius;
    Align2 align;
};

}  // namespace build123d
