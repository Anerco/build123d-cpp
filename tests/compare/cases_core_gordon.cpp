// Gordon surface cases (Face::make_gordon_surface), compared with cases_core_gordon_py.py.
#include <cmath>

#include <BRep_Tool.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TopoDS.hxx>

#include "case_registry.hpp"

using namespace build123d;
using compare::V;

namespace {

using CurveSpec = std::variant<Vector, Edge>;

V xyz(const Vector& v) { return V::list({v.X, v.Y, v.Z}); }

/// The surface's area, box, B-spline structure, and points and normals at (u, v) samples.
V summary(const Face& face) {
    const Handle(Geom_BSplineSurface) s = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(TopoDS::Face(face.wrapped())));
    const BoundBox bb = face.bounding_box();
    std::vector<V> points, normals;
    for (double u : {0.0, 0.25, 0.5, 0.8, 1.0}) {
        for (double v : {0.0, 0.3, 0.5, 1.0}) points.push_back(xyz(face.position_at(u, v)));
    }
    for (const auto& [u, v] : std::vector<std::pair<double, double>>{{0.5, 0.5}, {0.25, 0.75}, {0.9, 0.1}}) {
        normals.push_back(xyz(face.normal_at(u, v)));
    }
    std::vector<V> poles;
    for (int i : {1, (s->NbUPoles() + 1) / 2, s->NbUPoles()}) {
        for (int j : {1, (s->NbVPoles() + 1) / 2, s->NbVPoles()}) poles.push_back(xyz(Vector(s->Pole(i, j))));
    }
    return V::list({V(face.area()), xyz(bb.min), xyz(bb.max), V(s->UDegree()), V(s->VDegree()), V(s->NbUPoles()),
                    V(s->NbVPoles()), V(s->NbUKnots()), V(s->NbVKnots()), V(s->IsUClosed()), V(s->IsVClosed()),
                    V::list(points), V::list(normals), V::list(poles)});
}

/// The test network of build123d's test_make_gordon_surface: splines through
/// z = 0.2 sin(pi u) cos(pi v).
std::pair<std::vector<Edge>, std::vector<Edge>> create_test_curves(int num_profiles = 3, int num_guides = 4,
                                                                   double u_range = 1.0, double v_range = 1.0) {
    std::vector<std::vector<Vector>> intersection_points(static_cast<std::size_t>(num_profiles),
                                                         std::vector<Vector>(static_cast<std::size_t>(num_guides)));
    for (int i = 0; i < num_profiles; ++i) {
        for (int j = 0; j < num_guides; ++j) {
            const double u = i * u_range / (num_profiles - 1);
            const double v = j * v_range / (num_guides - 1);
            const double z = 0.2 * std::sin(u * PI) * std::cos(v * PI);
            intersection_points[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = Vector(u, v, z);
        }
    }
    std::vector<Edge> profiles, guides;
    for (int i = 0; i < num_profiles; ++i) profiles.push_back(Spline(intersection_points[static_cast<std::size_t>(i)]));
    for (int j = 0; j < num_guides; ++j) {
        std::vector<Vector> points;
        for (int i = 0; i < num_profiles; ++i) points.push_back(intersection_points[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        guides.push_back(Spline(points));
    }
    return {profiles, guides};
}

const std::vector<Vector> kPoints{Vector(0, 0, 0), Vector(10, 0, 0), Vector(12, 20, 1), Vector(4, 22, -1)};

}  // namespace

COMPARE_CASES {
    using compare::add_value;

    add_value("core_gordon_splines", [] {
        const auto [profiles, guides] = create_test_curves();
        const Face face = Face::make_gordon_surface(profiles, guides, 3e-4);
        return V::list({summary(face), xyz(guides[0].position_at(0.0)), xyz(profiles[0].position_at(1.0)),
                        xyz(guides[0].position_at(1.0)), xyz(profiles.back().position_at(1.0))});
    });
    add_value("core_gordon_splines_large", [] {
        const auto [profiles, guides] = create_test_curves(5, 6, 3.0, 2.0);
        return summary(Face::make_gordon_surface(profiles, guides));
    });
    add_value("core_gordon_splines_shuffled", [] {
        // the curves in no particular order, some running backwards
        auto [profiles, guides] = create_test_curves(4, 3, 2.0, 1.5);
        std::swap(profiles[0], profiles[2]);
        std::swap(guides[0], guides[1]);
        const auto reversed = [](const Edge& e) {
            std::vector<Vector> points;
            for (int i = 0; i <= 10; ++i) points.push_back(e.position_at(1.0 - i / 10.0));
            return Edge(Spline(points));
        };
        profiles[1] = reversed(profiles[1]);
        guides[2] = reversed(guides[2]);
        return summary(Face::make_gordon_surface(profiles, guides));
    });
    add_value("core_gordon_lines", [] {
        const std::vector<CurveSpec> profiles{Line(kPoints[0], kPoints[1]), Line(kPoints[3], kPoints[2])};
        const std::vector<CurveSpec> guides{Line(kPoints[0], kPoints[3]), Line(kPoints[1], kPoints[2])};
        const Face face = Face::make_gordon_surface(profiles, guides, 3e-4);
        return V::list({summary(face), xyz((kPoints[0] + kPoints[1] + kPoints[2] + kPoints[3]) / 4)});
    });
    add_value("core_gordon_arcs", [] {
        const std::vector<Edge> profiles{
            ThreePointArc(kPoints[0], (kPoints[0] + kPoints[1]) / 2 + Vector(0, 0, 2), kPoints[1]),
            ThreePointArc(kPoints[3], (kPoints[3] + kPoints[2]) / 2 + Vector(0, 0, 3), kPoints[2])};
        const std::vector<Edge> guides{Line(profiles[0].position_at(0), profiles[1].position_at(0)),
                                       Line(profiles[0].position_at(1), profiles[1].position_at(1))};
        const Face face = Face::make_gordon_surface(profiles, guides);
        return V::list({summary(face), xyz(guides[0].position_at(0.5)), xyz(guides[1].position_at(0.5))});
    });
    add_value("core_gordon_bezier", [] {
        const std::vector<Edge> profiles{
            Edge::make_bezier({kPoints[0], kPoints[0] + Vector(1, 0, 1), kPoints[1] - Vector(1, 0, 1), kPoints[1]}),
            Edge::make_bezier({kPoints[3], kPoints[3] + Vector(1, 0, 1), kPoints[2] - Vector(1, 0, 1), kPoints[2]})};
        const std::vector<Edge> guides{Line(profiles[0].position_at(0), profiles[1].position_at(0)),
                                       Line(profiles[0].position_at(1), profiles[1].position_at(1))};
        const Face face = Face::make_gordon_surface(profiles, guides);
        return V::list({summary(face), xyz(guides[0].position_at(0.5)), xyz(guides[1].position_at(0.5))});
    });
    add_value("core_gordon_closed_profiles", [] {
        const std::vector<Edge> profiles{Edge::make_ellipse(10, 6), Edge::make_ellipse(8, 7).translate(Vector(1, 2, 10))};
        const std::vector<Edge> guides{Line(profiles[0].position_at(0), profiles[1].position_at(0)),
                                       Line(profiles[0].position_at(0.5), profiles[1].position_at(0.5))};
        const Face face = Face::make_gordon_surface(profiles, guides);
        return V::list({summary(face), xyz(guides[0].position_at(0.5))});
    });
    add_value("core_gordon_closed_guides", [] {
        // the roles swapped: lines along z between two circles
        const std::vector<Edge> guides{Edge::make_circle(5), Edge::make_circle(3, Plane::XY.offset(8))};
        std::vector<Edge> profiles;
        for (double t : {0.0, 0.25, 0.5, 0.75}) profiles.push_back(Line(guides[0].position_at(t), guides[1].position_at(t)));
        return summary(Face::make_gordon_surface(profiles, guides));
    });
    add_value("core_gordon_points", [] {
        const Edge middle = ThreePointArc(kPoints[1], (kPoints[1] + kPoints[3]) / 2 + Vector(0, 0, 2), kPoints[3]);
        const std::vector<CurveSpec> profiles{kPoints[0], middle, kPoints[2]};
        const std::vector<CurveSpec> guides{Spline({kPoints[0], middle.position_at(0), kPoints[2]}),
                                        Spline({kPoints[0], middle.position_at(1), kPoints[2]})};
        const Face face = Face::make_gordon_surface(profiles, guides, 3e-4);
        return V::list({summary(face), xyz(std::get<Edge>(guides[0]).position_at(1)),
                        xyz(std::get<Edge>(guides[1]).position_at(1))});
    });
    add_value("core_gordon_point_guide", [] {
        // a guide that is a point: a cone-like patch
        const Vector apex(0, 0, 6);
        const std::vector<Edge> profiles{Line(Vector(-4, 0, 0), apex), Line(Vector(0, -1, 0), apex),
                                         Line(Vector(4, 0, 0), apex)};
        const std::vector<CurveSpec> guides{Edge(Spline({Vector(-4, 0, 0), Vector(0, -1, 0), Vector(4, 0, 0)})), apex};
        std::vector<CurveSpec> profile_curves(profiles.begin(), profiles.end());
        return summary(Face::make_gordon_surface(profile_curves, guides));
    });
    add_value("core_gordon_tolerance", [] {
        // the curves meet only within 1e-3
        const auto [profiles, guides] = create_test_curves(3, 3);
        std::vector<Edge> moved_guides;
        for (const auto& g : guides) moved_guides.push_back(g.translate(Vector(0, 0, 5e-4)));
        return summary(Face::make_gordon_surface(profiles, moved_guides, 2e-3));
    });
    add_value("core_gordon_errors", [] {
        const auto raises = [](auto f) -> V {
            try {
                f();
            } catch (const std::invalid_argument&) {
                return V("ValueError");
            } catch (const std::exception&) {
                return V("Exception");
            }
            return V();
        };
        const auto [profiles, guides] = create_test_curves();
        auto empty_profile = profiles;
        empty_profile[0] = Edge();
        auto empty_guide = guides;
        empty_guide[0] = Edge();
        const std::vector<CurveSpec> point_middle{Line(kPoints[0], kPoints[1]), (kPoints[0] + kPoints[2]) / 2,
                                              Line(kPoints[3], kPoints[2])};
        const std::vector<CurveSpec> point_guides{
            Spline({kPoints[0], (kPoints[0] + kPoints[2]) / 2, kPoints[3]}),
            Spline({kPoints[1], (kPoints[0] + kPoints[2]) / 2, kPoints[2]})};
        const std::vector<Edge> apart_profiles{Line(Vector(0, 0, 0), Vector(1, 0, 0)), Line(Vector(0, 1, 0), Vector(1, 1, 0))};
        const std::vector<Edge> apart_guides{Line(Vector(0, 0, 1), Vector(0, 1, 1)), Line(Vector(1, 0, 1), Vector(1, 1, 1))};
        const std::vector<Edge> one_profile{profiles[0]};
        const std::vector<Edge> twice{profiles[0], profiles[0]};
        return V::list({raises([&] { (void)Face::make_gordon_surface(empty_profile, guides); }),
                        raises([&] { (void)Face::make_gordon_surface(profiles, empty_guide); }),
                        raises([&] { (void)Face::make_gordon_surface(point_middle, point_guides); }),
                        raises([&] { (void)Face::make_gordon_surface(apart_profiles, apart_guides); }),
                        raises([&] { (void)Face::make_gordon_surface(one_profile, guides); }),
                        raises([&] { (void)Face::make_gordon_surface(twice, guides); })});
    });
}
