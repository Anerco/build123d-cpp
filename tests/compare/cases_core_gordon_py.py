"""Python twins of cases_core_gordon.cpp (Face.make_gordon_surface)."""

import math

from build123d import *  # noqa: F403
from OCP.BRep import BRep_Tool

from values import V


def xyz(v):
    return [v.X, v.Y, v.Z]


def summary(face):
    s = BRep_Tool.Surface_s(face.wrapped)
    bb = face.bounding_box()
    points = [xyz(face.position_at(u, v)) for u in (0.0, 0.25, 0.5, 0.8, 1.0) for v in (0.0, 0.3, 0.5, 1.0)]
    normals = [xyz(face.normal_at(u, v)) for u, v in ((0.5, 0.5), (0.25, 0.75), (0.9, 0.1))]
    poles = []
    for i in (1, (s.NbUPoles() + 1) // 2, s.NbUPoles()):
        for j in (1, (s.NbVPoles() + 1) // 2, s.NbVPoles()):
            poles.append(xyz(Vector(s.Pole(i, j))))
    return [face.area, xyz(bb.min), xyz(bb.max), s.UDegree(), s.VDegree(), s.NbUPoles(), s.NbVPoles(), s.NbUKnots(),
            s.NbVKnots(), s.IsUClosed(), s.IsVClosed(), points, normals, poles]


def create_test_curves(num_profiles=3, num_guides=4, u_range=1.0, v_range=1.0):
    intersection_points = [[(0.0, 0.0, 0.0) for _ in range(num_guides)] for _ in range(num_profiles)]
    for i in range(num_profiles):
        for j in range(num_guides):
            u = i * u_range / (num_profiles - 1)
            v = j * v_range / (num_guides - 1)
            z = 0.2 * math.sin(u * math.pi) * math.cos(v * math.pi)
            intersection_points[i][j] = (u, v, z)
    profiles = [Spline([intersection_points[i][j] for j in range(num_guides)]) for i in range(num_profiles)]
    guides = [Spline([intersection_points[i][j] for i in range(num_profiles)]) for j in range(num_guides)]
    return profiles, guides


POINTS = [Vector(0, 0, 0), Vector(10, 0, 0), Vector(12, 20, 1), Vector(4, 22, -1)]


def gordon_splines():
    profiles, guides = create_test_curves()
    face = Face.make_gordon_surface(profiles, guides, tolerance=3e-4)
    return V([summary(face), xyz(guides[0].position_at(0.0)), xyz(profiles[0].position_at(1.0)),
              xyz(guides[0].position_at(1.0)), xyz(profiles[-1].position_at(1.0))])


def gordon_splines_large():
    profiles, guides = create_test_curves(5, 6, 3.0, 2.0)
    return V(summary(Face.make_gordon_surface(profiles, guides)))


def gordon_splines_shuffled():
    profiles, guides = create_test_curves(4, 3, 2.0, 1.5)
    profiles[0], profiles[2] = profiles[2], profiles[0]
    guides[0], guides[1] = guides[1], guides[0]

    def reversed_(e):
        return Spline([e.position_at(1.0 - i / 10.0) for i in range(11)])

    profiles[1] = reversed_(profiles[1])
    guides[2] = reversed_(guides[2])
    return V(summary(Face.make_gordon_surface(profiles, guides)))


def gordon_lines():
    p = POINTS
    profiles = [Line(p[0], p[1]), Line(p[3], p[2])]
    guides = [Line(p[0], p[3]), Line(p[1], p[2])]
    face = Face.make_gordon_surface(profiles, guides, tolerance=3e-4)
    return V([summary(face), xyz((p[0] + p[1] + p[2] + p[3]) / 4)])


def gordon_arcs():
    p = POINTS
    profiles = [ThreePointArc(p[0], (p[0] + p[1]) / 2 + Vector(0, 0, 2), p[1]),
                ThreePointArc(p[3], (p[3] + p[2]) / 2 + Vector(0, 0, 3), p[2])]
    guides = [Line(profiles[0] @ 0, profiles[1] @ 0), Line(profiles[0] @ 1, profiles[1] @ 1)]
    face = Face.make_gordon_surface(profiles, guides)
    return V([summary(face), xyz(guides[0] @ 0.5), xyz(guides[1] @ 0.5)])


def gordon_bezier():
    p = POINTS
    profiles = [Edge.make_bezier(p[0], p[0] + Vector(1, 0, 1), p[1] - Vector(1, 0, 1), p[1]),
                Edge.make_bezier(p[3], p[3] + Vector(1, 0, 1), p[2] - Vector(1, 0, 1), p[2])]
    guides = [Line(profiles[0] @ 0, profiles[1] @ 0), Line(profiles[0] @ 1, profiles[1] @ 1)]
    face = Face.make_gordon_surface(profiles, guides)
    return V([summary(face), xyz(guides[0] @ 0.5), xyz(guides[1] @ 0.5)])


def gordon_closed_profiles():
    profiles = [Edge.make_ellipse(10, 6), Edge.make_ellipse(8, 7).translate((1, 2, 10))]
    guides = [Line(profiles[0] @ 0, profiles[1] @ 0), Line(profiles[0] @ 0.5, profiles[1] @ 0.5)]
    face = Face.make_gordon_surface(profiles, guides)
    return V([summary(face), xyz(guides[0] @ 0.5)])


def gordon_closed_guides():
    guides = [Edge.make_circle(5), Edge.make_circle(3, Plane.XY.offset(8))]
    profiles = [Line(guides[0] @ t, guides[1] @ t) for t in (0.0, 0.25, 0.5, 0.75)]
    return V(summary(Face.make_gordon_surface(profiles, guides)))


def gordon_points():
    p = POINTS
    middle = ThreePointArc(p[1], (p[1] + p[3]) / 2 + Vector(0, 0, 2), p[3])
    profiles = [p[0], middle, p[2]]
    guides = [Spline(p[0], middle @ 0, p[2]), Spline(p[0], middle @ 1, p[2])]
    face = Face.make_gordon_surface(profiles, guides, tolerance=3e-4)
    return V([summary(face), xyz(guides[0] @ 1), xyz(guides[1] @ 1)])


def gordon_point_guide():
    apex = Vector(0, 0, 6)
    profiles = [Line((-4, 0, 0), apex), Line((0, -1, 0), apex), Line((4, 0, 0), apex)]
    guides = [Spline((-4, 0, 0), (0, -1, 0), (4, 0, 0)), apex]
    return V(summary(Face.make_gordon_surface(profiles, guides)))


def gordon_tolerance():
    profiles, guides = create_test_curves(3, 3)
    moved_guides = [g.translate((0, 0, 5e-4)) for g in guides]
    return V(summary(Face.make_gordon_surface(profiles, moved_guides, 2e-3)))


def gordon_errors():
    def raises(f):
        try:
            f()
        except ValueError:
            return "ValueError"
        except Exception:  # noqa: BLE001
            return "Exception"
        return None

    p = POINTS
    profiles, guides = create_test_curves()
    empty_profile = list(profiles)
    empty_profile[0] = Edge()
    empty_guide = list(guides)
    empty_guide[0] = Edge()
    point_middle = [Line(p[0], p[1]), (p[0] + p[2]) / 2, Line(p[3], p[2])]
    point_guides = [Spline(p[0], (p[0] + p[2]) / 2, p[3]), Spline(p[1], (p[0] + p[2]) / 2, p[2])]
    apart_profiles = [Line((0, 0, 0), (1, 0, 0)), Line((0, 1, 0), (1, 1, 0))]
    apart_guides = [Line((0, 0, 1), (0, 1, 1)), Line((1, 0, 1), (1, 1, 1))]
    return V([raises(lambda: Face.make_gordon_surface(empty_profile, guides)),
              raises(lambda: Face.make_gordon_surface(profiles, empty_guide)),
              raises(lambda: Face.make_gordon_surface(point_middle, point_guides)),
              raises(lambda: Face.make_gordon_surface(apart_profiles, apart_guides)),
              raises(lambda: Face.make_gordon_surface([profiles[0]], guides)),
              raises(lambda: Face.make_gordon_surface([profiles[0], profiles[0]], guides))])


CASES = {
    "core_gordon_splines": gordon_splines,
    "core_gordon_splines_large": gordon_splines_large,
    "core_gordon_splines_shuffled": gordon_splines_shuffled,
    "core_gordon_lines": gordon_lines,
    "core_gordon_arcs": gordon_arcs,
    "core_gordon_bezier": gordon_bezier,
    "core_gordon_closed_profiles": gordon_closed_profiles,
    "core_gordon_closed_guides": gordon_closed_guides,
    "core_gordon_points": gordon_points,
    "core_gordon_point_guide": gordon_point_guide,
    "core_gordon_tolerance": gordon_tolerance,
    "core_gordon_errors": gordon_errors,
}
