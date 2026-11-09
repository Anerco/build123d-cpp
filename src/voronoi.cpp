// Voronoi vertices for full_round, following what scipy.spatial.Voronoi gets from Qhull
// ("Qbb Qc Qz"): the Delaunay triangulation (Bowyer-Watson with exact predicates),
// adjacent facets merged when their points are cocircular within Qhull's roundoff
// (DISTround in the Qbb-scaled lifted space), and each Voronoi vertex computed with
// Qhull's qh_voronoi_center formula.
#include "voronoi.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <numeric>
#include <stdexcept>

namespace build123d::detail {

namespace {

// ---------------------------------------------------------------------------
// Exact arithmetic on floating-point expansions (J. R. Shewchuk, "Adaptive Precision
// Floating-Point Arithmetic and Fast Robust Geometric Predicates", 1997).
// ---------------------------------------------------------------------------

using Expansion = std::vector<double>;  // nonoverlapping components, increasing magnitude

inline void two_sum(double a, double b, double& x, double& y) {
    x = a + b;
    const double bv = x - a;
    const double av = x - bv;
    y = (a - av) + (b - bv);
}

inline void two_diff(double a, double b, double& x, double& y) {
    x = a - b;
    const double bv = a - x;
    const double av = x + bv;
    y = (a - av) + (bv - b);
}

inline void two_product(double a, double b, double& x, double& y) {
    x = a * b;
    y = std::fma(a, b, -x);
}

Expansion grow(const Expansion& e, double b) {
    Expansion h;
    double q = b;
    for (double ei : e) {
        double s, err;
        two_sum(q, ei, s, err);
        q = s;
        if (err != 0.0) h.push_back(err);
    }
    if (q != 0.0 || h.empty()) h.push_back(q);
    return h;
}

Expansion sum(const Expansion& e, const Expansion& f) {
    Expansion h = e;
    for (double fi : f) h = grow(h, fi);
    return h;
}

Expansion negate(Expansion e) {
    for (double& x : e) x = -x;
    return e;
}

Expansion scale(const Expansion& e, double b) {
    Expansion h;
    if (e.empty()) return {0.0};
    double q, err;
    two_product(e[0], b, q, err);
    if (err != 0.0) h.push_back(err);
    for (std::size_t i = 1; i < e.size(); ++i) {
        double p1, p0, s;
        two_product(e[i], b, p1, p0);
        two_sum(q, p0, s, err);
        if (err != 0.0) h.push_back(err);
        two_sum(p1, s, q, err);
        if (err != 0.0) h.push_back(err);
    }
    if (q != 0.0 || h.empty()) h.push_back(q);
    return h;
}

Expansion product(const Expansion& e, const Expansion& f) {
    Expansion h{0.0};
    for (double fi : f) h = sum(h, scale(e, fi));
    return h;
}

int sign(const Expansion& e) {
    for (auto it = e.rbegin(); it != e.rend(); ++it) {
        if (*it > 0) return 1;
        if (*it < 0) return -1;
    }
    return 0;
}

Expansion diff(double a, double b) {
    double x, y;
    two_diff(a, b, x, y);
    if (y == 0.0) return {x};
    return {y, x};
}

constexpr double kEpsilon = 1.1102230246251565e-16;  // 2^-53
constexpr double kCcwErrBoundA = (3.0 + 16.0 * kEpsilon) * kEpsilon;
constexpr double kIccErrBoundA = (10.0 + 96.0 * kEpsilon) * kEpsilon;

int sgn(double v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

}  // namespace

int orient2d_sign(const Point2& a, const Point2& b, const Point2& c) {
    const double detleft = (a[0] - c[0]) * (b[1] - c[1]);
    const double detright = (a[1] - c[1]) * (b[0] - c[0]);
    const double det = detleft - detright;
    double detsum;
    if (detleft > 0.0) {
        if (detright <= 0.0) return sgn(det);
        detsum = detleft + detright;
    } else if (detleft < 0.0) {
        if (detright >= 0.0) return sgn(det);
        detsum = -detleft - detright;
    } else {
        return sgn(det);
    }
    const double errbound = kCcwErrBoundA * detsum;
    if (det >= errbound || -det >= errbound) return sgn(det);
    const Expansion acx = diff(a[0], c[0]), bcx = diff(b[0], c[0]);
    const Expansion acy = diff(a[1], c[1]), bcy = diff(b[1], c[1]);
    return sign(sum(product(acx, bcy), negate(product(acy, bcx))));
}

int incircle_sign(const Point2& a, const Point2& b, const Point2& c, const Point2& d) {
    const double adx = a[0] - d[0], bdx = b[0] - d[0], cdx = c[0] - d[0];
    const double ady = a[1] - d[1], bdy = b[1] - d[1], cdy = c[1] - d[1];
    const double bdxcdy = bdx * cdy, cdxbdy = cdx * bdy;
    const double alift = adx * adx + ady * ady;
    const double cdxady = cdx * ady, adxcdy = adx * cdy;
    const double blift = bdx * bdx + bdy * bdy;
    const double adxbdy = adx * bdy, bdxady = bdx * ady;
    const double clift = cdx * cdx + cdy * cdy;
    const double det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
    const double permanent = (std::abs(bdxcdy) + std::abs(cdxbdy)) * alift +
                             (std::abs(cdxady) + std::abs(adxcdy)) * blift +
                             (std::abs(adxbdy) + std::abs(bdxady)) * clift;
    const double errbound = kIccErrBoundA * permanent;
    if (det > errbound || -det > errbound) return sgn(det);

    const Expansion eadx = diff(a[0], d[0]), ebdx = diff(b[0], d[0]), ecdx = diff(c[0], d[0]);
    const Expansion eady = diff(a[1], d[1]), ebdy = diff(b[1], d[1]), ecdy = diff(c[1], d[1]);
    const Expansion ealift = sum(product(eadx, eadx), product(eady, eady));
    const Expansion eblift = sum(product(ebdx, ebdx), product(ebdy, ebdy));
    const Expansion eclift = sum(product(ecdx, ecdx), product(ecdy, ecdy));
    const Expansion bc = sum(product(ebdx, ecdy), negate(product(ecdx, ebdy)));
    const Expansion ca = sum(product(ecdx, eady), negate(product(eadx, ecdy)));
    const Expansion ab = sum(product(eadx, ebdy), negate(product(ebdx, eady)));
    return sign(sum(sum(product(ealift, bc), product(eblift, ca)), product(eclift, ab)));
}

namespace {

struct Triangle {
    std::array<int, 3> v;  // counter-clockwise
    bool alive = true;
};

std::pair<int, int> edge_key(int a, int b) { return a < b ? std::make_pair(a, b) : std::make_pair(b, a); }

/// qh_voronoi_center for a 2D simplex: the circumcenter computed relative to the first point
/// with Qhull's determinants.
Point2 voronoi_center(const Point2& p0, const Point2& p1, const Point2& p2) {
    const double m00 = p1[0] - p0[0], m01 = p2[0] - p0[0];
    const double m10 = p1[1] - p0[1], m11 = p2[1] - p0[1];
    const double det = m00 * m11 - m01 * m10;
    const double factor = 0.5 / det;
    auto sum2 = [&](const Point2& p) {
        double s = 0.0;
        for (int k = 0; k < 2; ++k) {
            const double diffp = p[static_cast<std::size_t>(k)] - p0[static_cast<std::size_t>(k)];
            s += diffp * diffp;
        }
        return s;
    };
    const double s1 = sum2(p1), s2 = sum2(p2);
    const double det_x = s1 * m11 - s2 * m10;  // row 0 replaced by the squared lengths
    const double det_y = m00 * s2 - m01 * s1;  // row 1 replaced
    return {det_x * factor + p0[0], det_y * factor + p0[1]};
}

}  // namespace

std::vector<Point2> voronoi_vertices(const std::vector<Point2>& input) {
    // Coincident points are coplanar points for Qhull ("Qc"), not vertices.
    double maxabs = 0.0;
    for (const auto& p : input) maxabs = std::max({maxabs, std::abs(p[0]), std::abs(p[1])});
    const double dup_tol = 1e-13 * std::max(1.0, maxabs);
    std::vector<Point2> pts;
    for (const auto& p : input) {
        const bool duplicate = std::any_of(pts.begin(), pts.end(), [&](const Point2& q) {
            return std::hypot(p[0] - q[0], p[1] - q[1]) <= dup_tol;
        });
        if (!duplicate) pts.push_back(p);
    }
    const int n = static_cast<int>(pts.size());
    if (n < 4) {
        throw std::runtime_error("QH6214 qhull input error: not enough points(" + std::to_string(n) +
                                 ") to construct initial simplex (need 4)");
    }
    // The points must span the plane
    bool flat = true;
    for (int i = 2; i < n && flat; ++i) flat = orient2d_sign(pts[0], pts[1], pts[static_cast<std::size_t>(i)]) == 0;
    if (flat) throw std::runtime_error("QH6154 Qhull precision error: initial simplex is flat");

    // Super triangle far away
    double minx = pts[0][0], maxx = minx, miny = pts[0][1], maxy = miny;
    for (const auto& p : pts) {
        minx = std::min(minx, p[0]);
        maxx = std::max(maxx, p[0]);
        miny = std::min(miny, p[1]);
        maxy = std::max(maxy, p[1]);
    }
    const double cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
    const double span = std::max({maxx - minx, maxy - miny, 1.0}) * 1e5;
    std::vector<Point2> all = pts;
    all.push_back({cx - 2 * span, cy - span});
    all.push_back({cx + 2 * span, cy - span});
    all.push_back({cx, cy + 2 * span});

    std::vector<Triangle> tris;
    tris.push_back({{n, n + 1, n + 2}, true});

    for (int pi = 0; pi < n; ++pi) {
        const Point2& p = all[static_cast<std::size_t>(pi)];
        std::map<std::pair<int, int>, std::pair<int, std::pair<int, int>>> edges;  // key -> (count, directed edge)
        for (auto& t : tris) {
            if (!t.alive) continue;
            if (incircle_sign(all[static_cast<std::size_t>(t.v[0])], all[static_cast<std::size_t>(t.v[1])],
                              all[static_cast<std::size_t>(t.v[2])], p) > 0) {
                t.alive = false;
                for (int k = 0; k < 3; ++k) {
                    const int a = t.v[static_cast<std::size_t>(k)], b = t.v[static_cast<std::size_t>((k + 1) % 3)];
                    auto& entry = edges[edge_key(a, b)];
                    entry.first += 1;
                    entry.second = {a, b};
                }
            }
        }
        for (const auto& [key, entry] : edges) {
            if (entry.first == 1) tris.push_back({{entry.second.first, entry.second.second, pi}, true});
        }
    }

    // The Delaunay triangles of the input points
    std::vector<std::array<int, 3>> delaunay;
    for (const auto& t : tris) {
        if (t.alive && t.v[0] < n && t.v[1] < n && t.v[2] < n) delaunay.push_back(t.v);
    }

    // Lift onto the paraboloid, scaled like "Qbb" (last coordinate to [0, maxabs])
    double zmin = 1e300, zmax = -1e300;
    std::vector<double> lift(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const Point2& p = pts[static_cast<std::size_t>(i)];
        lift[static_cast<std::size_t>(i)] = p[0] * p[0] + p[1] * p[1];
        zmin = std::min(zmin, lift[static_cast<std::size_t>(i)]);
        zmax = std::max(zmax, lift[static_cast<std::size_t>(i)]);
    }
    const double newhigh = maxabs > 0 ? maxabs : 1.0;
    const double zscale = zmax > zmin ? newhigh / (zmax - zmin) : 1.0;
    for (auto& z : lift) z = (z - zmin) * zscale;
    // Qhull's DISTround for the lifted points
    double lmaxabs = 0.0, lmaxsumabs = 0.0;
    for (int i = 0; i < n; ++i) {
        const Point2& p = pts[static_cast<std::size_t>(i)];
        const double z = lift[static_cast<std::size_t>(i)];
        lmaxabs = std::max({lmaxabs, std::abs(p[0]), std::abs(p[1]), std::abs(z)});
        lmaxsumabs = std::max(lmaxsumabs, std::abs(p[0]) + std::abs(p[1]) + std::abs(z));
    }
    const double distround = 2.220446049250313e-16 * (3 * lmaxsumabs * 1.01 + lmaxabs);

    auto lifted_distance = [&](const std::array<int, 3>& t, int q) {
        const Point2& a = pts[static_cast<std::size_t>(t[0])];
        const Point2& b = pts[static_cast<std::size_t>(t[1])];
        const Point2& c = pts[static_cast<std::size_t>(t[2])];
        const Point2& d = pts[static_cast<std::size_t>(q)];
        const double az = lift[static_cast<std::size_t>(t[0])], bz = lift[static_cast<std::size_t>(t[1])];
        const double cz = lift[static_cast<std::size_t>(t[2])], dz = lift[static_cast<std::size_t>(q)];
        const double ux = b[0] - a[0], uy = b[1] - a[1], uz = bz - az;
        const double vx = c[0] - a[0], vy = c[1] - a[1], vz = cz - az;
        const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        return (nx * (d[0] - a[0]) + ny * (d[1] - a[1]) + nz * (dz - az)) / norm;
    };

    // Merge adjacent facets whose points are cocircular within the roundoff
    std::vector<int> parent(delaunay.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int i) {
        while (parent[static_cast<std::size_t>(i)] != i) {
            parent[static_cast<std::size_t>(i)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(i)])];
            i = parent[static_cast<std::size_t>(i)];
        }
        return i;
    };
    std::map<std::pair<int, int>, std::vector<int>> edge_tris;
    for (std::size_t ti = 0; ti < delaunay.size(); ++ti) {
        for (int k = 0; k < 3; ++k) {
            edge_tris[edge_key(delaunay[ti][static_cast<std::size_t>(k)], delaunay[ti][static_cast<std::size_t>((k + 1) % 3)])]
                .push_back(static_cast<int>(ti));
        }
    }
    for (const auto& [key, ts] : edge_tris) {
        if (ts.size() != 2) continue;
        const auto& t1 = delaunay[static_cast<std::size_t>(ts[0])];
        const auto& t2 = delaunay[static_cast<std::size_t>(ts[1])];
        int other = -1;
        for (int v : t2) {
            if (v != key.first && v != key.second) other = v;
        }
        if (std::abs(lifted_distance(t1, other)) <= distround) {
            parent[static_cast<std::size_t>(find(ts[0]))] = find(ts[1]);
        }
    }

    // One Voronoi vertex per (merged) facet, in the order the facets were made
    std::vector<int> cell_order;
    std::map<int, std::vector<int>> cell_points;
    for (std::size_t ti = 0; ti < delaunay.size(); ++ti) {
        const int root = find(static_cast<int>(ti));
        if (!cell_points.count(root)) cell_order.push_back(root);
        auto& cp = cell_points[root];
        for (int v : delaunay[ti]) {
            if (std::find(cp.begin(), cp.end(), v) == cp.end()) cp.push_back(v);
        }
    }
    std::vector<Point2> out;
    for (int root : cell_order) {
        std::vector<int> cp = cell_points[root];
        // Qhull lists a facet's vertices newest first
        std::sort(cp.begin(), cp.end(), std::greater<int>());
        if (cp.size() == 3) {
            out.push_back(voronoi_center(pts[static_cast<std::size_t>(cp[0])], pts[static_cast<std::size_t>(cp[1])],
                                         pts[static_cast<std::size_t>(cp[2])]));
            continue;
        }
        // qh_maxsimplex: the points of minimum and maximum x, then the largest triangle
        int imin = cp[0], imax = cp[0];
        for (int v : cp) {
            if (pts[static_cast<std::size_t>(v)][0] < pts[static_cast<std::size_t>(imin)][0]) imin = v;
            if (pts[static_cast<std::size_t>(v)][0] > pts[static_cast<std::size_t>(imax)][0]) imax = v;
        }
        int best = -1;
        double best_det = -1.0;
        for (int v : cp) {
            if (v == imin || v == imax) continue;
            const Point2& a = pts[static_cast<std::size_t>(imin)];
            const Point2& b = pts[static_cast<std::size_t>(imax)];
            const Point2& c = pts[static_cast<std::size_t>(v)];
            const double det = std::abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]));
            if (det > best_det) {
                best_det = det;
                best = v;
            }
        }
        out.push_back(voronoi_center(pts[static_cast<std::size_t>(imin)], pts[static_cast<std::size_t>(imax)],
                                     pts[static_cast<std::size_t>(best)]));
    }
    return out;
}

}  // namespace build123d::detail
