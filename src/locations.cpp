#include <cmath>

#include "build123d/locations.hpp"
#include "internal.hpp"

namespace build123d {

Locations::Locations(std::initializer_list<Vector> points) : Locations(std::vector<Vector>(points)) {}

Locations::Locations(std::initializer_list<Location> locations)
    : LocationList(std::vector<Location>(locations)) {}

Locations::Locations(const std::vector<Vector>& points) {
    for (const auto& p : points) push_back(Location(p));
}

Locations::Locations(const std::vector<Location>& locations) : LocationList(locations) {}

Locations::Locations(const std::vector<Plane>& planes) {
    for (const auto& p : planes) push_back(Location(p));
}

Locations::Locations(const std::vector<Vertex>& vertices) {
    for (const auto& v : vertices) push_back(Location(Vector(v)));
}

Locations::Locations(const std::vector<Face>& faces) {
    for (const auto& f : faces) push_back(Location(Plane(f)));
}

Locations::Locations(const std::vector<Axis>& axes) {
    for (const auto& a : axes) push_back(a.location());
}

void Locations::append(const Vector& point) { push_back(Location(point)); }
void Locations::append(const Location& location) { push_back(location); }
void Locations::append(const Vertex& vertex) { push_back(Location(Vector(vertex))); }
void Locations::append(const Face& face) { push_back(Location(Plane(face))); }
void Locations::append(const Plane& plane) { push_back(Location(plane)); }
void Locations::append(const Axis& axis) { push_back(axis.location()); }

GridLocations::GridLocations(double xs, double ys, int xc, int yc, Align2 align_in)
    : x_spacing(xs), y_spacing(ys), x_count(xc), y_count(yc), align(align_in) {
    if (xc < 1 || yc < 1) {
        throw std::invalid_argument("At least 1 elements required, requested " + std::to_string(xc) + ", " +
                                    std::to_string(yc));
    }
    size = Vector(xs * (xc - 1), ys * (yc - 1));
    const Vector offset = to_align_offset(Vector(0, 0), size, Align3(align_in, Align::NONE));
    min = offset;
    max = min + size;
    for (int i = 0; i < xc; ++i) {
        for (int j = 0; j < yc; ++j) push_back(Location(offset + Vector(i * xs, j * ys)));
    }
}

PolarLocations::PolarLocations(double radius, int count, double start_angle, double angular_range, bool rotate,
                               bool endpoint) {
    if (count < 1) throw std::invalid_argument("At least 1 elements required, requested " + std::to_string(count));
    const double step = count == 1 ? 0.0 : angular_range / (count - (endpoint ? 1 : 0));
    for (int i = 0; i < count; ++i) {
        const Vector position = Vector(radius, 0).rotate(Axis::Z, start_angle + step * i);
        push_back(Location(position, Vector(0, 0, 1), rotate ? step * i + start_angle : 0.0));
    }
}

HexLocations::HexLocations(double r, int xc, int yc, bool major, Align2 align_in)
    : radius(r), x_count(xc), y_count(yc), major_radius(major), align(align_in) {
    if (major) {
        diagonal = 2 * r;
        apothem = r * std::cos(PI / 6);
    } else {
        diagonal = 4 * r / std::sqrt(3.0);
        apothem = r;
    }
    const double x_spacing = 3 * diagonal / 4;
    const double y_spacing = diagonal * std::sqrt(3.0) / 2;
    if (x_spacing <= 0 || y_spacing <= 0 || xc < 1 || yc < 1) throw std::invalid_argument("Spacing and count must be > 0 ");
    std::vector<Vector> points;
    for (int x = 0; x < xc; x += 2) {
        for (int y = 0; y < yc; ++y) points.emplace_back(x_spacing * x, y_spacing * y + y_spacing / 2);
    }
    for (int x = 1; x < xc; x += 2) {
        for (int y = 0; y < yc; ++y) points.emplace_back(x_spacing * x, y_spacing * y + y_spacing);
    }
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    for (const auto& p : points) {
        xmin = std::min(xmin, p.X);
        xmax = std::max(xmax, p.X);
        ymin = std::min(ymin, p.Y);
        ymax = std::max(ymax, p.Y);
    }
    const Vector size(xmax - xmin, ymax - ymin);
    const Vector min_corner(xmin, ymin);
    const Vector offset = to_align_offset(Vector(0, 0), size, Align3(align_in, Align::NONE));
    for (const auto& p : points) push_back(Location(p + offset - min_corner));
}

}  // namespace build123d
