// detect_primitives (brep_from_stl.py), ported step by step. Python's containers are kept
// where their order shows in the results: sets of face indices are CPython sets (pyset.hpp),
// dicts keep their insertion order, sums use Python's sum() and numpy's pairwise summation.
// The parts of scikit-learn (DBSCAN) and numpy (SVD, least squares) the module uses are
// ported too.
#include "build123d/brep_from_stl.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include <BRepBuilderAPI_Sewing.hxx>
#include <BRep_Tool.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>

#include "build123d/objects.hpp"
#include "internal.hpp"
#include "pyformat.hpp"
#include "pyset.hpp"

namespace build123d {

namespace {

using pyset::Set;
using Index = long long;

constexpr double EPS = 1e-9;
/// build123d's geometry TOLERANCE and TOL_DIGITS (brep_from_stl.py has its own TOLERANCE).
constexpr double kGeometryTolerance = 1e-6;
constexpr int kTolDigits = 6;
/// brep_from_stl.py's TOLERANCE for the generated code.
constexpr double kCodeTolerance = 1e-4;
constexpr double kInf = std::numeric_limits<double>::infinity();

using Key3 = std::array<double, 3>;
using EdgeKey = std::pair<Key3, Key3>;
using FaceKey = std::vector<Key3>;

// ---------------------------------------------------------------------------
// Python numerics
// ---------------------------------------------------------------------------

/// C's pow, called so that the compiler can't replace it (pow(x, 2.0) by x * x, ...).
double libm_pow(double x, double y) {
    static double (*volatile fn)(double, double) = static_cast<double (*)(double, double)>(&std::pow);
    return fn(x, y);
}

/// Python's float ** float (float_pow) for the finite values used here.
double py_pow(double iv, double iw) {
    if (iw == 0) return 1.0;
    if (iv == 0.0) {
        if (iw < 0.0) throw std::domain_error("0.0 cannot be raised to a negative power");
        const bool odd = std::fmod(std::fabs(iw), 2.0) == 1.0;
        return odd ? iv : 0.0;
    }
    bool negate = false;
    if (iv < 0.0) {
        if (iw != std::floor(iw)) throw std::domain_error("negative number raised to a fractional power");
        iv = -iv;
        negate = std::fmod(std::fabs(iw), 2.0) == 1.0;
    }
    if (iv == 1.0) return negate ? -1.0 : 1.0;
    const double ix = libm_pow(iv, iw);
    return negate ? -ix : ix;
}

/// Python's sum(values): the start 0, then Neumaier's compensated summation of floats.
double py_sum(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double f_result = 0.0 + values[0];
    double c = 0.0;
    for (std::size_t i = 1; i < values.size(); ++i) {
        const double x = values[i];
        const double t = f_result + x;
        if (std::fabs(f_result) >= std::fabs(x)) {
            c += (f_result - t) + x;
        } else {
            c += (x - t) + f_result;
        }
        f_result = t;
    }
    if (c != 0.0 && std::isfinite(c)) f_result += c;
    return f_result;
}

[[noreturn]] void zero_division() { throw std::domain_error("division by zero"); }

/// numpy's pairwise summation (pairwise_sum_DOUBLE) of values[lo, lo + n).
double numpy_pairwise_sum(const std::vector<double>& a, std::size_t lo, std::size_t n) {
    if (n < 8) {
        double res = 0.;
        for (std::size_t i = 0; i < n; ++i) res += a[lo + i];
        return res;
    }
    if (n <= 128) {
        double r[8];
        for (std::size_t j = 0; j < 8; ++j) r[j] = a[lo + j];
        std::size_t i = 8;
        for (; i < n - (n % 8); i += 8) {
            for (std::size_t j = 0; j < 8; ++j) r[j] += a[lo + i + j];
        }
        double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; ++i) res += a[lo + i];
        return res;
    }
    std::size_t n2 = n / 2;
    n2 -= n2 % 8;
    return numpy_pairwise_sum(a, lo, n2) + numpy_pairwise_sum(a, lo + n2, n - n2);
}

/// float(np.std(values)): the population standard deviation as numpy computes it.
double numpy_std(const std::vector<double>& values) {
    const std::size_t n = values.size();
    const auto count = static_cast<double>(n);
    const double mean = numpy_pairwise_sum(values, 0, n) / count;
    std::vector<double> squares(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double x = values[i] - mean;
        squares[i] = x * x;
    }
    return std::sqrt(numpy_pairwise_sum(squares, 0, n) / count);
}

/// Vector / denominator as build123d computes it: multiply by 1.0 / denominator.
Vector divided(const Vector& v, double denominator) {
    if (denominator == 0) zero_division();
    return v * (1.0 / denominator);
}

/// format(value, "0.6g")
std::string g6(double value) { return pyfmt::format_g(value, 6); }

/// format(vector, "0.6g") (Vector.__format__: values within TOLERANCE of 0 shown as 0)
std::string g6(const Vector& v) {
    auto trim = [](double x) { return std::fabs(x) > kGeometryTolerance ? pyfmt::round(x, 6) : 0.0; };
    return "(" + g6(trim(v.X)) + ", " + g6(trim(v.Y)) + ", " + g6(trim(v.Z)) + ")";
}

/// format(location, "0.6g")
std::string g6(const Location& location) {
    return "(" + g6(location.position()) + ", " + g6(location.orientation()) + ")";
}

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------

struct FaceSample {
    Index index;
    Face face;
    Vector center;
    Vector normal;
};

struct PlanePatch {
    Set face_indices;
    Vector origin;
    Vector normal;
    double u_min, u_max, v_min, v_max;
    double residual;
};

struct CylinderPatch {
    Set face_indices;
    Vector axis_point;
    Vector axis_direction;
    double radius;
    int normal_sign;
    double residual;
};

struct SpherePatch {
    Set face_indices;
    Vector center;
    double radius;
    double residual;
};

using DetectedPatch = std::variant<PlanePatch, CylinderPatch, SpherePatch>;

/// A Python dict keeping its insertion order.
template <class K, class V>
class OrderedDict {
public:
    V& operator[](const K& key) {
        const auto it = index_.find(key);
        if (it != index_.end()) return items_[it->second].second;
        index_.emplace(key, items_.size());
        items_.emplace_back(key, V());
        return items_.back().second;
    }
    const V* find(const K& key) const {
        const auto it = index_.find(key);
        return it == index_.end() ? nullptr : &items_[it->second].second;
    }
    const std::vector<std::pair<K, V>>& items() const { return items_; }

private:
    std::map<K, std::size_t> index_;
    std::vector<std::pair<K, V>> items_;
};

Key3 rounded_vertex_key(const Vector& vector, int digits = 9) {
    return {pyfmt::round(vector.X, digits), pyfmt::round(vector.Y, digits), pyfmt::round(vector.Z, digits)};
}

/// _edge_key: the edge's rounded end points, sorted.
EdgeKey edge_key(const Edge& edge) {
    std::vector<Key3> ends;
    for (const Vertex& vertex : edge.vertices()) ends.push_back(rounded_vertex_key(vertex.center()));
    std::stable_sort(ends.begin(), ends.end());
    if (ends.size() < 2) throw std::out_of_range("list index out of range (an edge with one vertex)");
    return {ends[0], ends[1]};
}

/// _face_key: the face's rounded vertices, sorted.
FaceKey face_key(const Face& face) {
    FaceKey key;
    for (const Vertex& vertex : face.vertices()) key.push_back(rounded_vertex_key(vertex.center()));
    std::stable_sort(key.begin(), key.end());
    return key;
}

/// _normalized
Vector normalized(const Vector& vector) {
    if (vector.length() <= EPS) throw std::invalid_argument("Cannot normalize near-zero vector");
    return vector.normalized();
}

struct MeshIndex {
    std::vector<Face> faces;
    std::vector<FaceSample> face_samples;
    std::map<FaceKey, Index> face_key_lookup;
    std::optional<std::vector<Set>> adjacent_face_indices;

    static MeshIndex from_shape(const Shape& shape) {
        MeshIndex m;
        for (const Face& face : shape.faces()) m.faces.push_back(face);
        for (std::size_t index = 0; index < m.faces.size(); ++index) {
            const Face& face = m.faces[index];
            m.face_samples.push_back(
                FaceSample{static_cast<Index>(index), face, face.center(), normalized(face.normal_at())});
        }
        for (std::size_t index = 0; index < m.faces.size(); ++index) {
            m.face_key_lookup[face_key(m.faces[index])] = static_cast<Index>(index);
        }
        return m;
    }

    /// ensure_adjacency
    const std::vector<Set>& adjacency() {
        if (adjacent_face_indices) return *adjacent_face_indices;
        OrderedDict<EdgeKey, Set> edge_to_face_indices;
        for (std::size_t index = 0; index < faces.size(); ++index) {
            for (const Edge& edge : faces[index].edges()) edge_to_face_indices[edge_key(edge)].add(static_cast<Index>(index));
        }
        std::vector<Set> adjacency(faces.size());
        for (const auto& [key, face_indices] : edge_to_face_indices.items()) {
            for (const Index face_index : face_indices.items()) {
                Set single;
                single.add(face_index);
                adjacency[static_cast<std::size_t>(face_index)].update(face_indices.difference(single));
            }
        }
        adjacent_face_indices = std::move(adjacency);
        return *adjacent_face_indices;
    }

    std::vector<Face> face_set(const std::vector<Index>& face_indices) const {
        std::vector<Face> out;
        out.reserve(face_indices.size());
        for (const Index index : face_indices) out.push_back(faces[static_cast<std::size_t>(index)]);
        return out;
    }

    const FaceSample& sample(Index index) const { return face_samples[static_cast<std::size_t>(index)]; }
};

// ---------------------------------------------------------------------------
// Basic numeric and vector helpers
// ---------------------------------------------------------------------------

double mean_scalar(const std::vector<double>& values) {
    if (values.empty()) zero_division();
    return py_sum(values) / static_cast<double>(values.size());
}

double median_scalar(const std::vector<double>& values) {
    if (values.empty()) throw std::out_of_range("list index out of range");
    std::vector<double> ordered = values;
    std::stable_sort(ordered.begin(), ordered.end());
    const std::size_t middle = ordered.size() / 2;
    if (ordered.size() % 2) return ordered[middle];
    return (ordered[middle - 1] + ordered[middle]) / 2.0;
}

double std_scalar(const std::vector<double>& values) {
    const double mean = mean_scalar(values);
    std::vector<double> squares;
    squares.reserve(values.size());
    for (const double value : values) squares.push_back(py_pow(value - mean, 2));
    return std::sqrt(py_sum(squares) / static_cast<double>(values.size()));
}

Vector mean_vector(const std::vector<Vector>& vectors) {
    Vector total;
    for (const Vector& vector : vectors) total = total + vector;
    return divided(total, static_cast<double>(vectors.size()));
}

template <class T>
std::vector<T> evenly_spaced_subset(const std::vector<T>& values, long long max_count) {
    if (max_count <= 0) return {};
    if (static_cast<long long>(values.size()) <= max_count) return values;
    if (max_count == 1) return {values[0]};
    const auto last_index = static_cast<long long>(values.size()) - 1;
    std::vector<T> out;
    for (long long i = 0; i < max_count; ++i) {
        const double position = static_cast<double>(i * last_index) / static_cast<double>(max_count - 1);
        out.push_back(values[static_cast<std::size_t>(std::nearbyint(position))]);
    }
    return out;
}

// ---------------------------------------------------------------------------
// DBSCAN (scikit-learn's DBSCAN.fit(...).labels_) and the clusters build123d takes from it
// ---------------------------------------------------------------------------

/// dbscan_inner: labels from the neighborhoods (each including the point itself).
std::vector<long long> dbscan_labels(const std::vector<std::vector<std::size_t>>& neighborhoods,
                                     std::size_t min_samples) {
    const std::size_t n = neighborhoods.size();
    std::vector<long long> labels(n, -1);
    std::vector<char> is_core(n);
    for (std::size_t i = 0; i < n; ++i) is_core[i] = neighborhoods[i].size() >= min_samples;
    long long label_num = 0;
    std::vector<std::size_t> stack;
    for (std::size_t start = 0; start < n; ++start) {
        if (labels[start] != -1 || !is_core[start]) continue;
        std::size_t i = start;
        while (true) {
            if (labels[i] == -1) {
                labels[i] = label_num;
                if (is_core[i]) {
                    for (const std::size_t v : neighborhoods[i]) {
                        if (labels[v] == -1) stack.push_back(v);
                    }
                }
            }
            if (stack.empty()) break;
            i = stack.back();
            stack.pop_back();
        }
        ++label_num;
    }
    return labels;
}

/// [labels == label for label in sorted(set(labels)) if label != -1]
std::vector<std::vector<bool>> cluster_masks(const std::vector<long long>& labels) {
    long long max_label = -1;
    for (const long long label : labels) max_label = std::max(max_label, label);
    std::vector<std::vector<bool>> masks;
    for (long long label = 0; label <= max_label; ++label) {
        std::vector<bool> mask(labels.size());
        for (std::size_t i = 0; i < labels.size(); ++i) mask[i] = labels[i] == label;
        masks.push_back(std::move(mask));
    }
    return masks;
}

std::size_t count_nonzero(const std::vector<bool>& mask) {
    return static_cast<std::size_t>(std::count(mask.begin(), mask.end(), true));
}

/// max(masks, key=count_nonzero): the first of the largest.
const std::vector<bool>& largest_mask(const std::vector<std::vector<bool>>& masks) {
    std::size_t best = 0;
    for (std::size_t i = 1; i < masks.size(); ++i) {
        if (count_nonzero(masks[i]) > count_nonzero(masks[best])) best = i;
    }
    return masks[best];
}

/// _cluster_points: DBSCAN with the euclidean metric (radius check on squared distances).
std::vector<std::vector<bool>> cluster_points(const std::vector<std::array<double, 2>>& points, double eps,
                                              std::size_t min_samples) {
    if (points.size() < min_samples) return {};
    const double reduced_eps = eps * eps;
    std::vector<std::vector<std::size_t>> neighborhoods(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        for (std::size_t j = 0; j < points.size(); ++j) {
            double d = 0;
            for (std::size_t k = 0; k < 2; ++k) {
                const double tmp = points[i][k] - points[j][k];
                d += tmp * tmp;
            }
            if (d <= reduced_eps) neighborhoods[i].push_back(j);
        }
    }
    return cluster_masks(dbscan_labels(neighborhoods, min_samples));
}

/// _cluster_unit_vectors: DBSCAN with the cosine metric (1 - cosine similarity of the
/// normalized rows, clipped to [0, 2]).
std::vector<std::vector<bool>> cluster_unit_vectors(const std::vector<Vector>& vectors, double eps,
                                                    std::size_t min_samples) {
    if (vectors.size() < min_samples) return {};
    std::vector<std::array<double, 3>> rows;
    rows.reserve(vectors.size());
    for (const Vector& v : vectors) {
        const double norm = std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        const double scale = norm == 0.0 ? 1.0 : norm;  // _handle_zeros_in_scale
        rows.push_back({v.X / scale, v.Y / scale, v.Z / scale});
    }
    std::vector<std::vector<std::size_t>> neighborhoods(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
        for (std::size_t j = 0; j < rows.size(); ++j) {
            const double similarity = rows[i][0] * rows[j][0] + rows[i][1] * rows[j][1] + rows[i][2] * rows[j][2];
            double distance = -similarity + 1;
            distance = std::min(std::max(distance, 0.0), 2.0);
            if (distance <= eps) neighborhoods[i].push_back(j);
        }
    }
    return cluster_masks(dbscan_labels(neighborhoods, min_samples));
}

// ---------------------------------------------------------------------------
// numpy's SVD and least squares (one-sided Jacobi SVD)
// ---------------------------------------------------------------------------

/// The singular values and right singular vectors (columns of v) of the m x n matrix a,
/// sorted by decreasing singular value. u holds the left singular vectors times the
/// singular values (a v).
struct Svd {
    std::vector<double> s;
    std::vector<std::vector<double>> v;   // v[row][col]
    std::vector<std::vector<double>> av;  // av[row][col]
};

Svd jacobi_svd(std::vector<std::vector<double>> a, std::size_t n) {
    const std::size_t m = a.size();
    std::vector<std::vector<double>> v(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) v[i][i] = 1.0;
    const double eps = std::numeric_limits<double>::epsilon();
    for (int sweep = 0; sweep < 100; ++sweep) {
        bool rotated = false;
        for (std::size_t p = 0; p + 1 < n; ++p) {
            for (std::size_t q = p + 1; q < n; ++q) {
                double alpha = 0, beta = 0, gamma = 0;
                for (std::size_t k = 0; k < m; ++k) {
                    alpha += a[k][p] * a[k][p];
                    beta += a[k][q] * a[k][q];
                    gamma += a[k][p] * a[k][q];
                }
                if (gamma == 0.0 || std::fabs(gamma) <= eps * std::sqrt(alpha * beta)) continue;
                rotated = true;
                const double zeta = (beta - alpha) / (2.0 * gamma);
                const double t = (zeta >= 0 ? 1.0 : -1.0) / (std::fabs(zeta) + std::sqrt(1.0 + zeta * zeta));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = c * t;
                for (std::size_t k = 0; k < m; ++k) {
                    const double akp = a[k][p];
                    const double akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const double vkp = v[k][p];
                    const double vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
        if (!rotated) break;
    }
    std::vector<double> s(n);
    for (std::size_t j = 0; j < n; ++j) {
        double norm = 0;
        for (std::size_t k = 0; k < m; ++k) norm += a[k][j] * a[k][j];
        s[j] = std::sqrt(norm);
    }
    std::vector<std::size_t> order(n);
    for (std::size_t j = 0; j < n; ++j) order[j] = j;
    std::stable_sort(order.begin(), order.end(), [&s](std::size_t i, std::size_t j) { return s[i] > s[j]; });
    Svd out;
    out.v.assign(n, std::vector<double>(n));
    out.av.assign(m, std::vector<double>(n));
    for (std::size_t c = 0; c < n; ++c) {
        out.s.push_back(s[order[c]]);
        for (std::size_t r = 0; r < n; ++r) out.v[r][c] = v[r][order[c]];
        for (std::size_t r = 0; r < m; ++r) out.av[r][c] = a[r][order[c]];
    }
    return out;
}

/// np.linalg.lstsq(a, b, rcond=None): the solution and the rank.
std::pair<std::vector<double>, int> lstsq(const std::vector<std::vector<double>>& a, const std::vector<double>& b,
                                          std::size_t n) {
    const std::size_t m = a.size();
    const Svd svd = jacobi_svd(a, n);
    const double rcond = std::numeric_limits<double>::epsilon() * static_cast<double>(std::max(m, n));
    const double threshold = svd.s.empty() ? 0.0 : rcond * svd.s[0];
    std::vector<double> x(n, 0.0);
    int rank = 0;
    for (std::size_t c = 0; c < n; ++c) {
        const double sigma = svd.s[c];
        if (!(sigma > threshold)) continue;
        ++rank;
        // u_c . b / sigma with u_c = av_c / sigma
        double ub = 0;
        for (std::size_t r = 0; r < m; ++r) ub += svd.av[r][c] * b[r];
        const double coefficient = ub / (sigma * sigma);
        for (std::size_t r = 0; r < n; ++r) x[r] += coefficient * svd.v[r][c];
    }
    return {x, rank};
}

// ---------------------------------------------------------------------------
// Clustering and low-level geometry helpers
// ---------------------------------------------------------------------------

/// _as_face for a Sketch or Part: its face().
template <class T>
Face as_face(const T& value, const char* context) {
    if (value.faces().empty()) throw std::runtime_error(std::string("Expected Face while building ") + context);
    return value.face();
}

std::pair<Vector, Vector> plane_basis(const Vector& normal) {
    Vector helper(1.0, 0.0, 0.0);
    if (std::fabs(helper.dot(normal)) > 0.9) helper = Vector(0.0, 1.0, 0.0);
    const Vector u = normalized(normal.cross(helper));
    const Vector v = normalized(normal.cross(u));
    return {u, v};
}

std::vector<double> plane_point_distances(const std::vector<Vector>& points, const Vector& plane_origin,
                                          const Vector& plane_normal) {
    std::vector<double> out;
    out.reserve(points.size());
    for (const Vector& point : points) out.push_back(std::fabs((point - plane_origin).dot(plane_normal)));
    return out;
}

std::optional<std::array<Vector, 3>> pick_non_collinear_triplet(const std::vector<Vector>& points) {
    const std::size_t n = points.size();
    if (n < 3) return std::nullopt;
    for (std::size_t a = 0; a < n; ++a) {
        for (std::size_t b = a + 1; b < n; ++b) {
            for (std::size_t c = b + 1; c < n; ++c) {
                if ((points[b] - points[a]).cross(points[c] - points[a]).length() > EPS) {
                    return std::array<Vector, 3>{points[a], points[b], points[c]};
                }
            }
        }
    }
    return std::nullopt;
}

std::optional<double> circumradius_from_points(const Vector& point_a, const Vector& point_b, const Vector& point_c) {
    const double side_a = (point_b - point_c).length();
    const double side_b = (point_a - point_c).length();
    const double side_c = (point_a - point_b).length();
    const double area_twice = (point_b - point_a).cross(point_c - point_a).length();
    if (area_twice <= EPS) return std::nullopt;
    return (side_a * side_b * side_c) / (2.0 * area_twice);
}

std::vector<Vector> unique_face_vertices(const std::vector<Face>& faces) {
    std::map<Key3, std::size_t> seen;
    std::vector<Vector> vertices;
    for (const Face& face : faces) {
        for (const Vertex& vertex : face.vertices()) {
            const Vector center = vertex.center();
            if (seen.emplace(rounded_vertex_key(center), vertices.size()).second) vertices.push_back(center);
        }
    }
    return vertices;
}

Vector canonicalize_direction(const Vector& direction) {
    const Vector unit = normalized(direction);
    const std::array<double, 3> values{unit.X, unit.Y, unit.Z};
    std::size_t max_index = 0;
    for (std::size_t i = 1; i < 3; ++i) {
        if (std::fabs(values[i]) > std::fabs(values[max_index])) max_index = i;
    }
    return values[max_index] >= 0 ? unit : unit * -1.0;
}

/// _fit_plane_to_points: the centroid and the normal from the SVD of the centered points.
std::pair<Vector, Vector> fit_plane_to_points(const std::vector<Vector>& points) {
    if (points.size() < 3) throw std::invalid_argument("Need at least three points to fit a plane");
    const Vector centroid = mean_vector(points);
    std::vector<std::vector<double>> rows;
    rows.reserve(points.size());
    for (const Vector& point : points) {
        const Vector d = point - centroid;
        rows.push_back({d.X, d.Y, d.Z});
    }
    const Svd svd = jacobi_svd(rows, 3);
    const Vector normal = canonicalize_direction(Vector(svd.v[0][2], svd.v[1][2], svd.v[2][2]));
    return {centroid, normal};
}

// ---------------------------------------------------------------------------
// Adjacency and grouping helpers
// ---------------------------------------------------------------------------

std::vector<Index> bfs_patch(MeshIndex& mesh_index, Index seed_index, const Set& allowed_indices, int max_depth) {
    const std::vector<Set>& adjacency = mesh_index.adjacency();
    Set visited;
    visited.add(seed_index);
    std::vector<std::pair<Index, int>> frontier{{seed_index, 0}};
    std::size_t head = 0;
    std::vector<Index> patch;
    while (head < frontier.size()) {
        const auto [face_index, depth] = frontier[head++];
        patch.push_back(face_index);
        if (depth >= max_depth) continue;
        for (const Index neighbor_index : adjacency[static_cast<std::size_t>(face_index)].items()) {
            if (visited.contains(neighbor_index) || !allowed_indices.contains(neighbor_index)) continue;
            visited.add(neighbor_index);
            frontier.emplace_back(neighbor_index, depth + 1);
        }
    }
    return patch;
}

/// sorted(lists, key=len, reverse=True)
template <class T>
void sort_by_length_descending(std::vector<std::vector<T>>& lists) {
    std::stable_sort(lists.begin(), lists.end(),
                     [](const std::vector<T>& a, const std::vector<T>& b) { return a.size() > b.size(); });
}

std::vector<std::vector<Index>> group_indices_by_area(const MeshIndex& mesh_index, const Set& allowed_indices,
                                                      int tol_digits = 5) {
    OrderedDict<double, std::vector<Index>> by_area;
    for (const Index face_index : allowed_indices.items()) {
        by_area[pyfmt::round(mesh_index.faces[static_cast<std::size_t>(face_index)].area(), tol_digits)].push_back(
            face_index);
    }
    std::vector<std::vector<Index>> groups;
    for (const auto& [area, indices] : by_area.items()) groups.push_back(indices);
    sort_by_length_descending(groups);
    return groups;
}

/// Face.sew_faces: the faces sewn together, grouped by the sewn shells.
std::vector<std::vector<Face>> sew_faces(const std::vector<Face>& faces) {
    BRepBuilderAPI_Sewing shell_builder;
    for (const Face& face : faces) shell_builder.Add(face.wrapped());
    shell_builder.Perform();
    const TopoDS_Shape sewed_shape = shell_builder.SewedShape();
    // get_top_level_topods_shapes
    std::vector<TopoDS_Shape> top_level_shapes;
    std::vector<TopoDS_Shape> stack{sewed_shape};
    while (!stack.empty()) {
        const TopoDS_Shape current = stack.back();
        stack.pop_back();
        if (!current.IsNull() && current.ShapeType() == TopAbs_COMPOUND) {
            for (TopoDS_Iterator it(current); it.More(); it.Next()) {
                if (it.Value().ShapeType() == TopAbs_COMPOUND) {
                    stack.push_back(it.Value());
                } else {
                    top_level_shapes.push_back(it.Value());
                }
            }
        } else {
            top_level_shapes.push_back(current);
        }
    }
    std::vector<std::vector<Face>> sewn_faces;
    for (const TopoDS_Shape& top_level_shape : top_level_shapes) {
        if (top_level_shape.IsNull()) throw std::runtime_error("SewedShape returned an empty shape");
        switch (top_level_shape.ShapeType()) {
            case TopAbs_FACE: sewn_faces.push_back({Face(top_level_shape)}); break;
            case TopAbs_SHELL: {
                const ShapeList<Face> shell_faces = Shell(top_level_shape).faces();
                sewn_faces.emplace_back(shell_faces.begin(), shell_faces.end());
                break;
            }
            case TopAbs_SOLID: {
                std::vector<Face> solid_faces;
                for (const TopoDS_Shape& f : detail::entities(top_level_shape, TopAbs_FACE)) solid_faces.emplace_back(f);
                sewn_faces.push_back(std::move(solid_faces));
                break;
            }
            default: throw std::runtime_error("SewedShape returned an unexpected shape type");
        }
    }
    return sewn_faces;
}

std::vector<Index> indices_from_sewn_component(const MeshIndex& mesh_index, const std::vector<Face>& component) {
    std::vector<Index> indices;
    for (const Face& component_face : component) {
        for (const Face& face : component_face.faces()) {
            const auto it = mesh_index.face_key_lookup.find(face_key(face));
            if (it != mesh_index.face_key_lookup.end()) indices.push_back(it->second);
        }
    }
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    return indices;
}

using EdgeAdjacency = std::vector<std::vector<std::pair<Index, Vector>>>;

EdgeAdjacency build_face_edge_midpoint_adjacency(const MeshIndex& mesh_index) {
    OrderedDict<EdgeKey, std::vector<Index>> edge_to_faces;
    std::map<EdgeKey, Vector> edge_midpoints;
    for (std::size_t index = 0; index < mesh_index.faces.size(); ++index) {
        for (const Edge& edge : mesh_index.faces[index].edges()) {
            const EdgeKey key = edge_key(edge);
            edge_to_faces[key].push_back(static_cast<Index>(index));
            std::vector<Vector> vertices;
            for (const Vertex& vertex : edge.vertices()) vertices.push_back(vertex.center());
            edge_midpoints[key] = Vector((vertices[0].X + vertices[1].X) / 2.0, (vertices[0].Y + vertices[1].Y) / 2.0,
                                         (vertices[0].Z + vertices[1].Z) / 2.0);
        }
    }
    EdgeAdjacency adjacency(mesh_index.faces.size());
    for (const auto& [key, face_indices] : edge_to_faces.items()) {
        if (face_indices.size() != 2) continue;
        const Index first = face_indices[0];
        const Index second = face_indices[1];
        const Vector& midpoint = edge_midpoints.at(key);
        adjacency[static_cast<std::size_t>(first)].emplace_back(second, midpoint);
        adjacency[static_cast<std::size_t>(second)].emplace_back(first, midpoint);
    }
    for (auto& neighbors : adjacency) {
        std::stable_sort(neighbors.begin(), neighbors.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });
    }
    return adjacency;
}

// ---------------------------------------------------------------------------
// Primitive face builders
// ---------------------------------------------------------------------------

Face build_plane_face(const PlanePatch& patch) {
    const double u_size = patch.u_max - patch.u_min;
    const double v_size = patch.v_max - patch.v_min;
    const double u_center = (patch.u_min + patch.u_max) / 2.0;
    const double v_center = (patch.v_min + patch.v_max) / 2.0;
    const Vector u_vec = plane_basis(patch.normal).first;
    const Plane plane(patch.origin, u_vec, patch.normal);
    return as_face((plane * Pos(u_center, v_center, 0)) * Rectangle(u_size, v_size), "plane primitive");
}

Face build_cylinder_face(const CylinderPatch& patch, const std::vector<Face>& support_faces) {
    const std::vector<Vector> vertices = unique_face_vertices(support_faces);
    std::vector<double> axis_values;
    std::vector<double> radial_distances;
    for (const Vector& vertex : vertices) axis_values.push_back((vertex - patch.axis_point).dot(patch.axis_direction));
    for (const Vector& vertex : vertices) {
        const Vector offset = vertex - patch.axis_point;
        const Vector radial = offset - patch.axis_direction * offset.dot(patch.axis_direction);
        if (radial.length() <= EPS) continue;
        radial_distances.push_back(radial.length());
    }
    if (axis_values.empty()) throw std::invalid_argument("min() arg is an empty sequence");
    const double axis_min = *std::min_element(axis_values.begin(), axis_values.end());
    const double axis_max = *std::max_element(axis_values.begin(), axis_values.end());
    const double radius = median_scalar(radial_distances);
    const Part cylinder_shape = Plane(patch.axis_point + patch.axis_direction * axis_min, std::nullopt,
                                      patch.axis_direction) *
                               Cylinder(radius, axis_max - axis_min, Align::NONE);
    const ShapeList<Face> filtered_faces = cylinder_shape.faces().filter_by(GeomType::CYLINDER);
    if (filtered_faces.empty()) {
        throw std::runtime_error("Expected cylindrical face while building cylinder primitive");
    }
    return patch.normal_sign > 0 ? filtered_faces[0] : -filtered_faces[0];
}

Face build_sphere_face(const SpherePatch& patch, const std::vector<Face>& support_faces) {
    std::vector<double> distances;
    for (const Vector& vertex : unique_face_vertices(support_faces)) distances.push_back((vertex - patch.center).length());
    const double radius = median_scalar(distances);
    return as_face(Pos(patch.center) * Sphere(radius), "sphere primitive");
}

// ---------------------------------------------------------------------------
// Local signature and patch-growth helpers
// ---------------------------------------------------------------------------

double relative_radius_spread(const std::vector<double>& signature) {
    std::vector<double> finite;
    for (const double value : signature) {
        if (std::isfinite(value)) finite.push_back(value);
    }
    if (finite.size() < 3) return kInf;
    const double low = finite[0];
    const double mid = finite[finite.size() / 2];
    const double high = finite.back();
    if (mid <= EPS) return kInf;
    return (high - low) / mid;
}

std::vector<double> face_radius_signature(const EdgeAdjacency& edge_adjacency, const MeshIndex& mesh_index,
                                          Index face_index, const Set& allowed_indices) {
    const FaceSample& sample = mesh_index.sample(face_index);
    std::vector<double> estimates;
    for (const auto& [neighbor_index, edge_midpoint] : edge_adjacency[static_cast<std::size_t>(face_index)]) {
        if (!allowed_indices.contains(neighbor_index)) continue;
        const FaceSample& neighbor = mesh_index.sample(neighbor_index);
        const auto radius = circumradius_from_points(sample.center, neighbor.center, edge_midpoint);
        estimates.push_back(radius ? *radius : kInf);
    }
    std::stable_sort(estimates.begin(), estimates.end());
    return estimates;
}

std::vector<std::vector<Index>> connected_face_components(MeshIndex& mesh_index, const Set& face_indices) {
    const std::vector<Set>& adjacency = mesh_index.adjacency();
    Set remaining = face_indices.copy();
    std::vector<std::vector<Index>> components;
    while (!remaining.empty()) {
        const Index seed_index = remaining.pop();
        std::vector<Index> component{seed_index};
        std::vector<Index> frontier{seed_index};
        while (!frontier.empty()) {
            const Index face_index = frontier.back();
            frontier.pop_back();
            for (const Index neighbor_index : adjacency[static_cast<std::size_t>(face_index)].items()) {
                if (!remaining.contains(neighbor_index)) continue;
                remaining.remove(neighbor_index);
                component.push_back(neighbor_index);
                frontier.push_back(neighbor_index);
            }
        }
        std::sort(component.begin(), component.end());
        components.push_back(std::move(component));
    }
    sort_by_length_descending(components);
    return components;
}

std::vector<std::vector<Index>> sphere_like_face_components(MeshIndex& mesh_index, const Set& allowed_indices,
                                                            double similarity_tolerance = 1.0) {
    const EdgeAdjacency edge_adjacency = build_face_edge_midpoint_adjacency(mesh_index);
    Set sphere_like_indices;
    for (const Index face_index : allowed_indices.items()) {
        const std::vector<double> signature =
            face_radius_signature(edge_adjacency, mesh_index, face_index, allowed_indices);
        if (signature.empty()) continue;
        if (relative_radius_spread(signature) <= similarity_tolerance) sphere_like_indices.add(face_index);
    }
    return connected_face_components(mesh_index, sphere_like_indices);
}

std::optional<double> cylinder_face_error(const FaceSample& sample, const CylinderPatch& patch, double shape_scale) {
    const Vector offset = sample.center - patch.axis_point;
    const Vector radial = offset - patch.axis_direction * offset.dot(patch.axis_direction);
    if (radial.length() <= EPS) return std::nullopt;
    const double radius_error = std::fabs(radial.length() - patch.radius);
    const double normal_error = 1.0 - std::fabs(radial.normalized().dot(sample.normal));
    const double radius_tolerance = std::max(shape_scale * 0.01, patch.radius * 0.02);
    if (radius_error > radius_tolerance || normal_error > 0.03) return std::nullopt;
    return radius_error / std::max(radius_tolerance, EPS) + normal_error / 0.03;
}

std::optional<double> sphere_face_error(const FaceSample& sample, const SpherePatch& patch, double shape_scale) {
    const Vector radial = patch.center - sample.center;
    if (radial.length() <= EPS) return std::nullopt;
    const double radius_error = std::fabs(radial.length() - patch.radius);
    const double normal_error = 1.0 - std::fabs(radial.normalized().dot(sample.normal));
    const double radius_tolerance = std::max(shape_scale * 0.01, patch.radius * 0.02);
    if (radius_error > radius_tolerance || normal_error > 0.03) return std::nullopt;
    return radius_error / std::max(radius_tolerance, EPS) + normal_error / 0.03;
}

bool bounding_boxes_overlap(const BoundBox& box1, const BoundBox& box2, double tolerance = 0.0) {
    return !(box1.max.X < box2.min.X - tolerance || box2.max.X < box1.min.X - tolerance ||
             box1.max.Y < box2.min.Y - tolerance || box2.max.Y < box1.min.Y - tolerance ||
             box1.max.Z < box2.min.Z - tolerance || box2.max.Z < box1.min.Z - tolerance);
}

/// grow_curved_patch: the faces claimed from the patch's faces over compatible neighbors.
template <class Patch, class Score>
Set grow_claimed(MeshIndex& mesh_index, const Patch& patch, const Set& allowed_indices, Score score) {
    const std::vector<Set>& adjacency = mesh_index.adjacency();
    Set claimed = patch.face_indices.copy().intersection(allowed_indices);
    std::vector<Index> frontier = claimed.items();
    while (!frontier.empty()) {
        const Index face_index = frontier.back();
        frontier.pop_back();
        for (const Index neighbor_index : adjacency[static_cast<std::size_t>(face_index)].items()) {
            if (claimed.contains(neighbor_index) || !allowed_indices.contains(neighbor_index)) continue;
            if (!score(mesh_index.sample(neighbor_index))) continue;
            claimed.add(neighbor_index);
            frontier.push_back(neighbor_index);
        }
    }
    return claimed;
}

CylinderPatch grow_curved_patch(MeshIndex& mesh_index, const CylinderPatch& patch, const Set& allowed_indices,
                                double shape_scale) {
    const Set claimed = grow_claimed(mesh_index, patch, allowed_indices, [&](const FaceSample& sample) {
        return cylinder_face_error(sample, patch, shape_scale).has_value();
    });
    std::vector<double> radii;
    std::vector<double> residuals;
    for (const Index index : claimed.sorted()) {
        const FaceSample& sample = mesh_index.sample(index);
        const Vector offset = sample.center - patch.axis_point;
        const Vector radial = offset - patch.axis_direction * offset.dot(patch.axis_direction);
        if (radial.length() <= EPS) continue;
        radii.push_back(radial.length());
        residuals.push_back(1.0 - std::fabs(radial.normalized().dot(sample.normal)));
    }
    return CylinderPatch{claimed.copy(),
                         patch.axis_point,
                         patch.axis_direction,
                         radii.empty() ? patch.radius : mean_scalar(radii),
                         patch.normal_sign,
                         residuals.empty() ? patch.residual : mean_scalar(residuals)};
}

SpherePatch grow_curved_patch(MeshIndex& mesh_index, const SpherePatch& patch, const Set& allowed_indices,
                              double shape_scale) {
    const Set claimed = grow_claimed(mesh_index, patch, allowed_indices, [&](const FaceSample& sample) {
        return sphere_face_error(sample, patch, shape_scale).has_value();
    });
    std::vector<double> radii;
    std::vector<double> residuals;
    for (const Index index : claimed.sorted()) {
        const FaceSample& sample = mesh_index.sample(index);
        const Vector radial = patch.center - sample.center;
        if (radial.length() <= EPS) continue;
        radii.push_back(radial.length());
        residuals.push_back(1.0 - std::fabs(radial.normalized().dot(sample.normal)));
    }
    return SpherePatch{claimed.copy(), patch.center, radii.empty() ? patch.radius : mean_scalar(radii),
                       residuals.empty() ? patch.residual : mean_scalar(residuals)};
}

// ---------------------------------------------------------------------------
// Plane detection
// ---------------------------------------------------------------------------

double shape_diagonal(const Shape& shape) {
    const BoundBox box = shape.bounding_box();
    if (box.wrapped().IsVoid()) return 0.0;
    return py_pow(box.wrapped().SquareExtent(), 0.5);
}

std::vector<std::vector<Index>> plane_like_face_components(MeshIndex& mesh_index, const Set& allowed_indices,
                                                           int normal_digits = 3) {
    OrderedDict<Key3, Set> normal_groups;
    for (const Index face_index : allowed_indices.items()) {
        const Vector normal = canonicalize_direction(mesh_index.sample(face_index).normal);
        normal_groups[rounded_vertex_key(normal, normal_digits)].add(face_index);
    }
    std::vector<std::vector<Index>> components;
    for (const auto& [key, face_indices] : normal_groups.items()) {
        for (auto& component : connected_face_components(mesh_index, face_indices)) {
            components.push_back(std::move(component));
        }
    }
    sort_by_length_descending(components);
    return components;
}

std::vector<PlanePatch> detect_planes_from_clean_proxy(const Shape& shape, MeshIndex& mesh_index,
                                                       double normal_tolerance = 1e-3,
                                                       double plane_tolerance_factor = 0.002,
                                                       double bbox_tolerance_factor = 0.002,
                                                       double inside_tolerance_factor = 0.002,
                                                       std::size_t min_proxy_edges = 4,
                                                       double min_proxy_area_ratio = 0.5) {
    // Python fails here on an empty shape (AssertionError: the shape has no wrapped OCCT shape)
    if (shape.is_null()) throw std::invalid_argument("detect_primitives: the mesh is an empty shape");
    const double shape_scale = shape_diagonal(shape);
    const double plane_tolerance = shape_scale * plane_tolerance_factor;
    const double bbox_tolerance = shape_scale * bbox_tolerance_factor;
    const double inside_tolerance = shape_scale * inside_tolerance_factor;

    const Shape cleaned_shape = shape.clean();
    std::vector<Face> proxy_faces;
    for (const Face& face : cleaned_shape.faces()) {
        if (face.edges().size() >= min_proxy_edges) proxy_faces.push_back(face);
    }
    if (proxy_faces.empty()) return {};
    double max_area = proxy_faces[0].area();
    for (const Face& face : proxy_faces) max_area = std::max(max_area, face.area());
    std::vector<Face> large_faces;
    for (const Face& face : proxy_faces) {
        if (face.area() >= max_area * min_proxy_area_ratio) large_faces.push_back(face);
    }

    std::vector<std::optional<BoundBox>> sample_boxes(mesh_index.face_samples.size());
    std::vector<PlanePatch> patches;
    for (const Face& proxy_face : large_faces) {
        const Vector proxy_normal = canonicalize_direction(proxy_face.normal_at());
        const Vector proxy_center = proxy_face.center();
        const BoundBox proxy_bbox = proxy_face.bounding_box();

        std::vector<Index> matched_indices;
        std::vector<double> distances;
        for (const FaceSample& sample : mesh_index.face_samples) {
            if (1.0 - std::fabs(sample.normal.dot(proxy_normal)) > normal_tolerance) continue;
            const double distance = std::fabs((sample.center - proxy_center).dot(proxy_normal));
            if (distance > plane_tolerance) continue;
            auto& box = sample_boxes[static_cast<std::size_t>(sample.index)];
            if (!box) box = sample.face.bounding_box();
            if (!bounding_boxes_overlap(*box, proxy_bbox, bbox_tolerance)) continue;
            if (!proxy_face.is_inside(sample.center, inside_tolerance)) continue;
            matched_indices.push_back(sample.index);
            distances.push_back(distance);
        }
        if (matched_indices.size() < 2) continue;

        const std::vector<Vector> vertices = unique_face_vertices(mesh_index.face_set(matched_indices));
        const auto [u_vec, v_vec] = plane_basis(proxy_normal);
        std::vector<double> u_values;
        std::vector<double> v_values;
        for (const Vector& vertex : vertices) {
            u_values.push_back((vertex - proxy_center).dot(u_vec));
            v_values.push_back((vertex - proxy_center).dot(v_vec));
        }
        patches.push_back(PlanePatch{Set::from_iterable(matched_indices), proxy_center, proxy_normal,
                                     *std::min_element(u_values.begin(), u_values.end()),
                                     *std::max_element(u_values.begin(), u_values.end()),
                                     *std::min_element(v_values.begin(), v_values.end()),
                                     *std::max_element(v_values.begin(), v_values.end()), mean_scalar(distances)});
    }
    return patches;
}

std::optional<PlanePatch> build_plane_patch(const MeshIndex& mesh_index, const std::vector<Index>& face_indices,
                                            double shape_scale, double plane_tolerance_factor = 0.003,
                                            double normal_tolerance = 0.01) {
    if (face_indices.size() < 2) return std::nullopt;
    const std::vector<Vector> vertices = unique_face_vertices(mesh_index.face_set(face_indices));
    if (vertices.size() < 3) return std::nullopt;

    auto [origin, normal] = fit_plane_to_points(vertices);
    normal = canonicalize_direction(normal);
    const std::vector<double> distances = plane_point_distances(vertices, origin, normal);
    const double plane_tolerance = shape_scale * plane_tolerance_factor;
    if (*std::max_element(distances.begin(), distances.end()) > plane_tolerance) return std::nullopt;

    double max_normal_error = -kInf;
    for (const Index index : face_indices) {
        max_normal_error = std::max(max_normal_error, 1.0 - std::fabs(mesh_index.sample(index).normal.dot(normal)));
    }
    if (!face_indices.empty() && max_normal_error > normal_tolerance) return std::nullopt;

    const auto [u_vec, v_vec] = plane_basis(normal);
    std::vector<double> u_values;
    std::vector<double> v_values;
    for (const Vector& vertex : vertices) {
        u_values.push_back((vertex - origin).dot(u_vec));
        v_values.push_back((vertex - origin).dot(v_vec));
    }
    return PlanePatch{Set::from_iterable(face_indices),
                      origin,
                      normal,
                      *std::min_element(u_values.begin(), u_values.end()),
                      *std::max_element(u_values.begin(), u_values.end()),
                      *std::min_element(v_values.begin(), v_values.end()),
                      *std::max_element(v_values.begin(), v_values.end()),
                      distances.empty() ? 0.0 : mean_scalar(distances)};
}

double direction_angle_delta(const Vector& direction_a, const Vector& direction_b) {
    const double dot = std::fabs(canonicalize_direction(direction_a).dot(canonicalize_direction(direction_b)));
    return std::acos(std::min(1.0, std::max(-1.0, dot)));
}

double perpendicular_axis_shift(const Vector& point_a, const Vector& direction_a, const Vector& point_b,
                                const Vector& direction_b) {
    const Vector average_direction =
        canonicalize_direction(mean_vector({canonicalize_direction(direction_a), canonicalize_direction(direction_b)}));
    const Vector delta = point_b - point_a;
    const Vector perpendicular = delta - average_direction * delta.dot(average_direction);
    return perpendicular.length();
}

/// max(a, b, c) of floats
double max3(double a, double b, double c) { return std::max(std::max(a, b), c); }

std::vector<CylinderPatch> merge_equivalent_cylinders(const MeshIndex& mesh_index,
                                                      const std::vector<CylinderPatch>& patches, double shape_scale,
                                                      double axis_angle_tolerance = 0.04,
                                                      double axis_shift_factor = 0.015,
                                                      double radius_ratio_tolerance = 0.12) {
    std::vector<std::vector<const CylinderPatch*>> groups;
    for (const CylinderPatch& patch : patches) {
        bool placed = false;
        for (auto& group : groups) {
            const CylinderPatch& representative = *group[0];
            if (direction_angle_delta(representative.axis_direction, patch.axis_direction) <= axis_angle_tolerance &&
                perpendicular_axis_shift(representative.axis_point, representative.axis_direction, patch.axis_point,
                                         patch.axis_direction) <= shape_scale * axis_shift_factor &&
                std::fabs(representative.radius - patch.radius) / max3(representative.radius, patch.radius, EPS) <=
                    radius_ratio_tolerance) {
                group.push_back(&patch);
                placed = true;
                break;
            }
        }
        if (!placed) groups.push_back({&patch});
    }

    std::vector<CylinderPatch> merged;
    for (const auto& group : groups) {
        if (group.size() == 1) {
            merged.push_back(*group[0]);
            continue;
        }
        std::vector<const Set*> members;
        for (const CylinderPatch* patch : group) members.push_back(&patch->face_indices);
        const Set face_indices = Set().union_with(members);
        std::vector<Vector> directions;
        std::vector<Vector> points;
        for (const CylinderPatch* patch : group) {
            directions.push_back(patch->axis_direction);
            points.push_back(patch->axis_point);
        }
        const Vector axis_direction = canonicalize_direction(mean_vector(directions));
        const Vector axis_point = mean_vector(points);
        std::vector<double> radii;
        std::vector<double> residuals;
        for (const Index index : face_indices.sorted()) {
            const FaceSample& sample = mesh_index.sample(index);
            const Vector offset = sample.center - axis_point;
            const Vector radial = offset - axis_direction * offset.dot(axis_direction);
            if (radial.length() <= EPS) continue;
            radii.push_back(radial.length());
            residuals.push_back(1.0 - std::fabs(radial.normalized().dot(sample.normal)));
        }
        if (radii.empty()) continue;
        long long sign_sum = 0;
        std::vector<double> group_residuals;
        for (const CylinderPatch* patch : group) {
            sign_sum += patch->normal_sign * static_cast<long long>(patch->face_indices.size());
            group_residuals.push_back(patch->residual);
        }
        merged.push_back(CylinderPatch{face_indices, axis_point, axis_direction, mean_scalar(radii),
                                       sign_sum >= 0 ? 1 : -1,
                                       residuals.empty() ? mean_scalar(group_residuals) : mean_scalar(residuals)});
    }
    return merged;
}

bool validate_bounded_cylinder(const CylinderPatch& patch, const std::vector<Face>& support_faces,
                               double shape_scale, double plane_tolerance_factor = 0.004,
                               double radius_tolerance_factor = 0.01, double min_bin_fraction = 0.35,
                               double max_radius_std_ratio = 0.08, double plane_parallel_tolerance = 0.02,
                               double end_radius_ratio_tolerance = 0.12) {
    const std::vector<Vector> vertices = unique_face_vertices(support_faces);
    if (vertices.size() < 6) return false;

    std::vector<double> radial_distances;
    for (const Vector& vertex : vertices) {
        const Vector offset = vertex - patch.axis_point;
        const Vector radial = offset - patch.axis_direction * offset.dot(patch.axis_direction);
        if (radial.length() <= EPS) continue;
        radial_distances.push_back(radial.length());
    }
    if (radial_distances.empty()) return false;
    const double radius_tolerance = std::max(shape_scale * radius_tolerance_factor, patch.radius * 0.02);
    const double radius_mean = mean_scalar(radial_distances);
    const double radius_std = std_scalar(radial_distances);
    if (std::fabs(radius_mean - patch.radius) > radius_tolerance) return false;
    if (radius_std / std::max(radius_mean, EPS) > max_radius_std_ratio) return false;

    const Vector center = mean_vector(vertices);
    std::vector<Vector> negative_bin;
    std::vector<Vector> positive_bin;
    for (const Vector& vertex : vertices) {
        const double value = (vertex - center).dot(patch.axis_direction);
        if (value < 0) negative_bin.push_back(vertex);
        if (value >= 0) positive_bin.push_back(vertex);
    }
    if (negative_bin.size() < 3 || positive_bin.size() < 3) return false;
    const auto count = static_cast<double>(vertices.size());
    if (static_cast<double>(negative_bin.size()) / count < min_bin_fraction ||
        static_cast<double>(positive_bin.size()) / count < min_bin_fraction) {
        return false;
    }

    const double plane_tolerance = shape_scale * plane_tolerance_factor;
    std::vector<std::pair<Vector, Vector>> end_planes;
    std::vector<double> end_radii;
    for (const auto* point_bin : {&negative_bin, &positive_bin}) {
        const auto [plane_origin, plane_normal] = fit_plane_to_points(*point_bin);
        const std::vector<double> distances = plane_point_distances(*point_bin, plane_origin, plane_normal);
        if (*std::max_element(distances.begin(), distances.end()) > plane_tolerance ||
            mean_scalar(distances) > plane_tolerance / 2.0) {
            return false;
        }
        end_planes.emplace_back(plane_origin, plane_normal);

        const auto triplet = pick_non_collinear_triplet(*point_bin);
        if (!triplet) return false;
        const auto end_radius = circumradius_from_points((*triplet)[0], (*triplet)[1], (*triplet)[2]);
        if (!end_radius || *end_radius <= EPS) return false;
        end_radii.push_back(*end_radius);
    }

    if (std::fabs(std::fabs(end_planes[0].second.dot(end_planes[1].second)) - 1.0) > plane_parallel_tolerance) {
        return false;
    }

    const double mean_end_radius = mean_scalar(end_radii);
    if (mean_end_radius <= EPS) return false;
    double max_end_deviation = -kInf;
    for (const double radius : end_radii) max_end_deviation = std::max(max_end_deviation, std::fabs(radius - mean_end_radius));
    if (max_end_deviation / mean_end_radius > end_radius_ratio_tolerance) return false;

    double max_radial_deviation = -kInf;
    for (const double radius : radial_distances) {
        max_radial_deviation = std::max(max_radial_deviation, std::fabs(radius - mean_end_radius));
    }
    return !(max_radial_deviation / mean_end_radius > 0.2);
}

std::optional<std::array<double, 2>> intersect_2d_lines(const std::array<double, 2>& point_a,
                                                        const std::array<double, 2>& direction_a,
                                                        const std::array<double, 2>& point_b,
                                                        const std::array<double, 2>& direction_b) {
    const double determinant = direction_b[0] * direction_a[1] - direction_a[0] * direction_b[1];
    if (std::fabs(determinant) <= EPS) return std::nullopt;
    const double delta_x = point_b[0] - point_a[0];
    const double delta_y = point_b[1] - point_a[1];
    const double scale_a = (direction_b[0] * delta_y - direction_b[1] * delta_x) / determinant;
    return std::array<double, 2>{point_a[0] + scale_a * direction_a[0], point_a[1] + scale_a * direction_a[1]};
}

std::optional<CylinderPatch> fit_local_cylinder(const std::vector<FaceSample>& samples, double shape_scale,
                                                long long max_pair_face_count = 64,
                                                long long max_intersection_face_count = 64) {
    const std::vector<FaceSample> axis_samples = evenly_spaced_subset(samples, max_pair_face_count);
    std::vector<std::pair<std::pair<Index, Index>, Vector>> records;
    for (std::size_t a = 0; a < axis_samples.size(); ++a) {
        for (std::size_t b = a + 1; b < axis_samples.size(); ++b) {
            const Vector cross = axis_samples[a].normal.cross(axis_samples[b].normal);
            if (cross.length() <= 1e-3) continue;
            records.push_back({{axis_samples[a].index, axis_samples[b].index}, canonicalize_direction(cross)});
        }
    }
    if (records.empty()) return std::nullopt;

    std::vector<Vector> directions;
    for (const auto& record : records) directions.push_back(record.second);
    const auto masks = cluster_unit_vectors(directions, 0.03, 4);
    if (masks.empty()) return std::nullopt;

    const std::vector<bool>& best_mask = largest_mask(masks);
    std::vector<Vector> cluster_directions;
    std::vector<Index> face_indices;
    for (std::size_t i = 0; i < records.size(); ++i) {
        if (!best_mask[i]) continue;
        cluster_directions.push_back(records[i].second);
        face_indices.push_back(records[i].first.first);
        face_indices.push_back(records[i].first.second);
    }
    std::sort(face_indices.begin(), face_indices.end());
    face_indices.erase(std::unique(face_indices.begin(), face_indices.end()), face_indices.end());
    if (face_indices.size() < 4) return std::nullopt;

    std::vector<FaceSample> face_group;
    for (const FaceSample& sample : samples) {
        if (std::binary_search(face_indices.begin(), face_indices.end(), sample.index)) face_group.push_back(sample);
    }
    const Vector axis_direction = canonicalize_direction(mean_vector(cluster_directions));
    const auto [u_vec, v_vec] = plane_basis(axis_direction);

    const std::vector<FaceSample> center_samples = evenly_spaced_subset(face_group, max_intersection_face_count);
    std::vector<std::array<double, 2>> points_2d;
    std::vector<std::array<double, 2>> normals_2d;
    std::vector<double> axis_coords;
    for (const FaceSample& sample : center_samples) {
        const std::array<double, 2> projected_normal{sample.normal.dot(u_vec), sample.normal.dot(v_vec)};
        const double projected_length = std::sqrt(py_pow(projected_normal[0], 2) + py_pow(projected_normal[1], 2));
        if (projected_length <= 1e-3) continue;
        points_2d.push_back({sample.center.dot(u_vec), sample.center.dot(v_vec)});
        normals_2d.push_back({projected_normal[0] / projected_length, projected_normal[1] / projected_length});
        axis_coords.push_back(sample.center.dot(axis_direction));
    }
    if (points_2d.size() < 4) return std::nullopt;

    std::vector<std::array<double, 2>> intersections_2d;
    for (std::size_t a = 0; a < points_2d.size(); ++a) {
        for (std::size_t b = a + 1; b < points_2d.size(); ++b) {
            if (const auto intersection = intersect_2d_lines(points_2d[a], normals_2d[a], points_2d[b], normals_2d[b])) {
                intersections_2d.push_back(*intersection);
            }
        }
    }
    if (intersections_2d.size() < 4) return std::nullopt;

    const auto point_masks = cluster_points(intersections_2d, shape_scale * 0.03, 4);
    if (point_masks.empty()) return std::nullopt;
    const std::vector<bool>& best_point_mask = largest_mask(point_masks);
    std::vector<double> best_x;
    std::vector<double> best_y;
    for (std::size_t i = 0; i < intersections_2d.size(); ++i) {
        if (!best_point_mask[i]) continue;
        best_x.push_back(intersections_2d[i][0]);
        best_y.push_back(intersections_2d[i][1]);
    }
    const std::array<double, 2> center_2d{mean_scalar(best_x), mean_scalar(best_y)};
    const Vector axis_point = u_vec * center_2d[0] + v_vec * center_2d[1] + axis_direction * mean_scalar(axis_coords);

    std::vector<double> radii;
    std::vector<double> residuals;
    std::vector<double> signed_alignments;
    for (const FaceSample& sample : face_group) {
        const Vector offset = sample.center - axis_point;
        const Vector radial = offset - axis_direction * offset.dot(axis_direction);
        if (radial.length() <= EPS) continue;
        const Vector radial_direction = radial.normalized();
        radii.push_back(radial.length());
        const double signed_alignment = radial_direction.dot(sample.normal);
        signed_alignments.push_back(signed_alignment);
        residuals.push_back(1.0 - std::fabs(signed_alignment));
    }
    if (radii.empty()) return std::nullopt;
    const double radius = mean_scalar(radii);
    if (radius <= EPS || radius > shape_scale) return std::nullopt;
    const double radius_std = std_scalar(radii);
    const double residual = residuals.empty() ? 0.0 : mean_scalar(residuals);
    if (radius_std / std::max(radius, EPS) > 0.15 || residual > 0.05) return std::nullopt;
    return CylinderPatch{Set::from_iterable(face_indices),
                         axis_point,
                         axis_direction,
                         radius,
                         mean_scalar(signed_alignments) >= 0 ? 1 : -1,
                         residual};
}

std::vector<PlanePatch> detect_planes_from_normals(const Shape& mesh, MeshIndex& mesh_index,
                                                   const Set& blocked_indices, int normal_digits = 3,
                                                   std::size_t min_component_size = 2,
                                                   double min_two_face_area_factor = 0.05) {
    const double shape_scale = shape_diagonal(mesh);
    std::vector<PlanePatch> plane_patches;
    Set remaining = Set::from_range(static_cast<long long>(mesh_index.faces.size())).difference(blocked_indices);

    for (const std::vector<Index>& component : plane_like_face_components(mesh_index, remaining, normal_digits)) {
        std::vector<Index> component_indices;
        for (const Index face_index : component) {
            if (remaining.contains(face_index)) component_indices.push_back(face_index);
        }
        if (component_indices.size() < min_component_size) continue;
        if (component_indices.size() == 2) {
            std::vector<double> areas;
            for (const Index face_index : component_indices) {
                areas.push_back(mesh_index.faces[static_cast<std::size_t>(face_index)].area());
            }
            if (py_sum(areas) < py_pow(shape_scale, 2) * min_two_face_area_factor) continue;
        }
        auto patch = build_plane_patch(mesh_index, component_indices, shape_scale);
        if (!patch) continue;
        plane_patches.push_back(*patch);
        remaining.difference_update(plane_patches.back().face_indices);
    }
    return plane_patches;
}

// ---------------------------------------------------------------------------
// Cylinder detection
// ---------------------------------------------------------------------------

std::vector<Index> cylinder_like_face_indices(MeshIndex& mesh_index, const Set& allowed_indices,
                                              double pair_similarity_tolerance = 0.35,
                                              double anisotropy_ratio_threshold = 1.5) {
    const EdgeAdjacency edge_adjacency = build_face_edge_midpoint_adjacency(mesh_index);
    std::vector<Index> cylinder_like_indices;
    for (const Index face_index : allowed_indices.items()) {
        const std::vector<double> signature =
            face_radius_signature(edge_adjacency, mesh_index, face_index, allowed_indices);
        std::vector<double> finite;
        for (const double value : signature) {
            if (std::isfinite(value)) finite.push_back(value);
        }
        if (finite.size() < 2) continue;
        if (finite.size() == 2) {
            const double low = finite[0];
            const double high = finite[1];
            if (low <= EPS) continue;
            const double anisotropy = high > EPS ? high / low : 0.0;
            if (anisotropy >= anisotropy_ratio_threshold) cylinder_like_indices.push_back(face_index);
            continue;
        }
        const double low = finite[0];
        const double mid = finite[1];
        const double high = finite[2];
        if (low <= EPS || mid <= EPS) continue;
        const double pair_similarity = std::fabs(mid - low) / mid;
        const double anisotropy = high > EPS ? high / mid : 0.0;
        if (pair_similarity <= pair_similarity_tolerance && anisotropy >= anisotropy_ratio_threshold) {
            cylinder_like_indices.push_back(face_index);
        }
    }
    std::sort(cylinder_like_indices.begin(), cylinder_like_indices.end());
    return cylinder_like_indices;
}

std::optional<SpherePatch> fit_local_sphere(const std::vector<FaceSample>& samples, double shape_scale,
                                            double radius_std_ratio_limit = 0.2,
                                            double normal_error_limit = 0.08) {
    if (samples.size() < 4) return std::nullopt;

    std::vector<std::vector<double>> rows;
    std::vector<double> rhs;
    for (const FaceSample& sample : samples) {
        const double x = sample.center.X;
        const double y = sample.center.Y;
        const double z = sample.center.Z;
        rows.push_back({x, y, z, 1.0});
        rhs.push_back(-(x * x + y * y + z * z));
    }
    const auto [coeffs, rank] = lstsq(rows, rhs, 4);
    if (rank < 4) return std::nullopt;

    const double a = coeffs[0];
    const double b = coeffs[1];
    const double c = coeffs[2];
    const double d = coeffs[3];
    const Vector center(-a / 2.0, -b / 2.0, -c / 2.0);
    const double radius_sq = center.dot(center) - d;
    if (radius_sq <= EPS) return std::nullopt;

    std::vector<double> radii;
    std::vector<double> normal_errors;
    for (const FaceSample& sample : samples) {
        const Vector radial = center - sample.center;
        const double radial_length = radial.length();
        if (radial_length <= EPS) continue;
        radii.push_back(radial_length);
        normal_errors.push_back(1.0 - std::fabs(radial.normalized().dot(sample.normal)));
    }
    if (radii.size() < 4) return std::nullopt;

    const double radius = py_sum(radii) / static_cast<double>(radii.size());
    if (radius <= EPS || radius > shape_scale) return std::nullopt;

    const double radius_std = numpy_std(radii);
    const double normal_error =
        normal_errors.empty() ? 0.0 : py_sum(normal_errors) / static_cast<double>(normal_errors.size());
    std::vector<double> deviations;
    for (const double value : radii) deviations.push_back(std::fabs(value - radius));
    const double residual = py_sum(deviations) / static_cast<double>(radii.size());

    if (radius_std / std::max(radius, EPS) > radius_std_ratio_limit) return std::nullopt;
    if (normal_error > normal_error_limit) return std::nullopt;

    std::vector<Index> indices;
    for (const FaceSample& sample : samples) indices.push_back(sample.index);
    return SpherePatch{Set::from_iterable(indices), center, radius, residual};
}

bool cylinder_patch_looks_spherical(const std::vector<FaceSample>& samples, const CylinderPatch& cylinder_patch,
                                    double shape_scale, double residual_factor = 0.35) {
    const auto sphere_patch = fit_local_sphere(samples, shape_scale);
    if (!sphere_patch) return false;
    return sphere_patch->residual <= cylinder_patch.residual * residual_factor;
}

std::vector<FaceSample> samples_of(const MeshIndex& mesh_index, const std::vector<Index>& indices) {
    std::vector<FaceSample> out;
    out.reserve(indices.size());
    for (const Index index : indices) out.push_back(mesh_index.sample(index));
    return out;
}

std::optional<CylinderPatch> finalize_cylinder_patch(MeshIndex& mesh_index, const CylinderPatch& patch,
                                                     const Set& remaining, double shape_scale,
                                                     std::size_t min_component_size, bool require_bounded_validation) {
    CylinderPatch grown_patch = grow_curved_patch(mesh_index, patch, remaining, shape_scale);
    if (grown_patch.face_indices.size() < min_component_size) return std::nullopt;

    std::vector<FaceSample> grown_samples = samples_of(mesh_index, grown_patch.face_indices.sorted());
    if (const auto refit_patch = fit_local_cylinder(grown_samples, shape_scale)) {
        grown_patch = grow_curved_patch(mesh_index, *refit_patch, remaining, shape_scale);
        if (grown_patch.face_indices.size() < min_component_size) return std::nullopt;
        grown_samples = samples_of(mesh_index, grown_patch.face_indices.sorted());
    }

    if (require_bounded_validation) {
        const std::vector<Face> support_faces = mesh_index.face_set(grown_patch.face_indices.sorted());
        if (!validate_bounded_cylinder(grown_patch, support_faces, shape_scale)) return std::nullopt;
    } else if (cylinder_patch_looks_spherical(grown_samples, grown_patch, shape_scale)) {
        return std::nullopt;
    }
    return grown_patch;
}

std::vector<CylinderPatch> detect_cylinders(const Shape& mesh, MeshIndex& mesh_index, const Set& blocked_indices,
                                            int area_tol_digits = 5, double pair_similarity_tolerance = 0.35,
                                            double anisotropy_ratio_threshold = 1.5, int local_seed_depth = 2,
                                            std::size_t min_component_size = 4) {
    Set remaining = Set::from_range(static_cast<long long>(mesh_index.faces.size())).difference(blocked_indices);
    const double shape_scale = shape_diagonal(mesh);
    std::vector<CylinderPatch> patches;

    for (const std::vector<Index>& area_group : group_indices_by_area(mesh_index, remaining, area_tol_digits)) {
        for (const std::vector<Face>& component : sew_faces(mesh_index.face_set(area_group))) {
            std::vector<Index> component_indices;
            for (const Index face_index : indices_from_sewn_component(mesh_index, component)) {
                if (remaining.contains(face_index)) component_indices.push_back(face_index);
            }
            if (component_indices.size() < min_component_size) continue;
            const auto patch = fit_local_cylinder(samples_of(mesh_index, component_indices), shape_scale);
            if (!patch) continue;
            auto finalized_patch =
                finalize_cylinder_patch(mesh_index, *patch, remaining, shape_scale, min_component_size, true);
            if (!finalized_patch) continue;
            patches.push_back(std::move(*finalized_patch));
            remaining.difference_update(patches.back().face_indices);
        }
    }

    Set tried_seed_indices;
    const std::vector<Index> clfi =
        cylinder_like_face_indices(mesh_index, remaining, pair_similarity_tolerance, anisotropy_ratio_threshold);
    for (const Index seed_index : clfi) {
        if (!remaining.contains(seed_index) || tried_seed_indices.contains(seed_index)) continue;
        tried_seed_indices.add(seed_index);
        const std::vector<Index> local_indices = bfs_patch(mesh_index, seed_index, remaining, local_seed_depth);
        if (local_indices.size() < min_component_size) continue;
        const auto patch = fit_local_cylinder(samples_of(mesh_index, local_indices), shape_scale);
        if (!patch) continue;
        auto finalized_patch =
            finalize_cylinder_patch(mesh_index, *patch, remaining, shape_scale, min_component_size, false);
        if (!finalized_patch) continue;
        patches.push_back(std::move(*finalized_patch));
        remaining.difference_update(patches.back().face_indices);
    }

    return merge_equivalent_cylinders(mesh_index, patches, shape_scale);
}

// ---------------------------------------------------------------------------
// Sphere detection
// ---------------------------------------------------------------------------

std::vector<SpherePatch> suppress_duplicate_spheres(const std::vector<SpherePatch>& patches, double center_tolerance,
                                                    double radius_ratio_tolerance = 0.1,
                                                    double overlap_ratio_tolerance = 0.5) {
    std::vector<const SpherePatch*> ordered;
    for (const SpherePatch& patch : patches) ordered.push_back(&patch);
    std::stable_sort(ordered.begin(), ordered.end(), [](const SpherePatch* a, const SpherePatch* b) {
        if (a->residual != b->residual) return a->residual < b->residual;
        return -static_cast<long long>(a->face_indices.size()) < -static_cast<long long>(b->face_indices.size());
    });
    std::vector<SpherePatch> kept;
    for (const SpherePatch* patch : ordered) {
        bool duplicate = false;
        for (const SpherePatch& kept_patch : kept) {
            const double center_shift = (patch->center - kept_patch.center).length();
            const double radius_delta =
                std::fabs(patch->radius - kept_patch.radius) / max3(patch->radius, kept_patch.radius, EPS);
            const auto shared = static_cast<double>(patch->face_indices.intersection(kept_patch.face_indices).size());
            const auto smaller = std::min(patch->face_indices.size(), kept_patch.face_indices.size());
            const double overlap = shared / static_cast<double>(std::max<std::size_t>(smaller, 1));
            if (center_shift <= center_tolerance && radius_delta <= radius_ratio_tolerance &&
                overlap >= overlap_ratio_tolerance) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) kept.push_back(*patch);
    }
    return kept;
}

std::vector<SpherePatch> detect_spheres(const Shape& mesh, MeshIndex& mesh_index, const Set& blocked_indices,
                                        double similarity_tolerance = 0.2, std::size_t min_component_size = 6) {
    Set remaining = Set::from_range(static_cast<long long>(mesh_index.faces.size())).difference(blocked_indices);
    const double shape_scale = shape_diagonal(mesh);
    std::vector<SpherePatch> patches;

    for (const std::vector<Index>& radius_group :
         sphere_like_face_components(mesh_index, remaining, similarity_tolerance)) {
        for (const std::vector<Face>& component : sew_faces(mesh_index.face_set(radius_group))) {
            std::vector<Index> component_indices;
            for (const Index face_index : indices_from_sewn_component(mesh_index, component)) {
                if (remaining.contains(face_index)) component_indices.push_back(face_index);
            }
            if (component_indices.size() < min_component_size) continue;
            const auto patch = fit_local_sphere(samples_of(mesh_index, component_indices), shape_scale);
            if (!patch) continue;
            SpherePatch grown_patch = grow_curved_patch(mesh_index, *patch, remaining, shape_scale);
            if (grown_patch.face_indices.size() < min_component_size) continue;
            patches.push_back(std::move(grown_patch));
            remaining.difference_update(patches.back().face_indices);
        }
    }
    return suppress_duplicate_spheres(patches, shape_scale * 0.03);
}

// ---------------------------------------------------------------------------
// Shape to build123d code conversion
// ---------------------------------------------------------------------------

std::string offset_code(const std::string& base, double val) {
    return std::fabs(val) < kCodeTolerance ? base + " * " : base + ".offset(" + g6(val) + ") * ";
}

std::string pos_code(double a, double b) {
    return (std::fabs(a) < kCodeTolerance && std::fabs(b) < kCodeTolerance) ? ""
                                                                         : "Pos(" + g6(a) + ", " + g6(b) + ") * ";
}

/// An axis-aligned plane: (z_dir, x_dir, name, offset sign, offset coordinate, position coordinates).
struct PlaneConfig {
    Vector z_dir;
    Vector x_dir;
    const char* name;
    int sign;
    int offset_coord;              // 0 = x, 1 = y, 2 = z
    std::array<int, 2> pos_coords;
};

const PlaneConfig kPlaneConfigs[] = {
    {Vector(0, 0, 1), Vector(1, 0, 0), "Plane.XY", 1, 2, {0, 1}},
    {Vector(0, 0, -1), Vector(0, 1, 0), "Plane.YX", -1, 2, {1, 0}},
    {Vector(0, 1, 0), Vector(0, 0, 1), "Plane.ZX", 1, 1, {2, 0}},
    {Vector(0, -1, 0), Vector(1, 0, 0), "Plane.XZ", -1, 1, {0, 2}},
    {Vector(1, 0, 0), Vector(0, 1, 0), "Plane.YZ", 1, 0, {1, 2}},
    {Vector(-1, 0, 0), Vector(0, 0, 1), "Plane.ZY", -1, 0, {2, 1}},
};

const PlaneConfig* find_plane_config(const Vector& z_dir) {
    for (const PlaneConfig& config : kPlaneConfigs) {
        if (z_dir.dot(config.z_dir) > 1 - kCodeTolerance) return &config;
    }
    return nullptr;
}

/// plane_sort_key
std::pair<std::string, double> plane_sort_key(const std::string& s) {
    std::string name = "ZZZ";
    auto is_word = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; };
    if (s.compare(0, 6, "Plane.") == 0 && s.size() > 6 && is_word(s[6])) {
        std::size_t e = 6;
        while (e < s.size() && is_word(s[e])) ++e;
        name = s.substr(0, e);
    } else if (s.compare(0, 8, "Location") == 0) {
        name = "Location";
    }
    double offset = 0.0;
    std::size_t from = 0;
    while (true) {
        const std::size_t p = s.find(".offset(", from);
        if (p == std::string::npos) break;
        const std::size_t start = p + 8;
        const std::size_t close = s.find(')', start);
        if (close != std::string::npos && close > start) {
            offset = std::strtod(s.substr(start, close - start).c_str(), nullptr);
            break;
        }
        from = p + 1;
    }
    return {name, offset};
}

/// Face.radius of a cylindrical or spherical face.
double face_radius(const Face& face) {
    const GeomType type = face.geom_type();
    if (type == GeomType::CYLINDER || type == GeomType::SPHERE) {
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(face.wrapped()));
        if (!surface->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface))) {
            if (const auto cylinder = Handle(Geom_CylindricalSurface)::DownCast(surface)) return cylinder->Radius();
            if (const auto sphere = Handle(Geom_SphericalSurface)::DownCast(surface)) return sphere->Radius();
        }
    }
    throw std::invalid_argument("unsupported format string passed to NoneType.__format__");
}

/// rect.intersect(primitive)[0] when it is a Face (the coplanar overlap of the two faces).
std::optional<Face> common_face(const Face& rect, const Face& primitive) {
    const TopoDS_Shape common =
        detail::bool_op(detail::BoolOp::COMMON, {rect.wrapped()}, {primitive.wrapped()});
    if (common.IsNull()) return std::nullopt;
    for (const TopoDS_Shape& face : detail::entities(common, TopAbs_FACE)) return Face(face);
    return std::nullopt;
}

std::vector<std::string> shapes_to_code(const std::vector<Face>& primitives) {
    std::vector<std::string> code_lines;
    for (const Face& primitive : primitives) {
        switch (primitive.geom_type()) {
            case GeomType::PLANE: {
                Plane pln(primitive);
                const PlaneConfig* cfg = find_plane_config(pln.z_dir);
                if (cfg != nullptr) pln = Plane(pln.origin, cfg->x_dir, cfg->z_dir);

                const Face center_oriented_rect = pln.to_local_coords(primitive);
                const Vertex local_corner =
                    center_oriented_rect.vertices().group_by(Axis::X)[0].sort_by(Axis::Y)[0];
                const Vertex local_origin(local_corner.X(), local_corner.Y(), 0);
                const Vertex global_origin = pln.from_local_coords(local_origin);
                pln = pln.shift_origin(Vector(global_origin));
                const BoundBox bbox = center_oriented_rect.bounding_box();

                double w = bbox.size.X;
                double height = bbox.size.Y;
                const Face rect = as_face(pln * Rectangle(w, height, Align::MIN), "planar rectangle");
                const auto common = common_face(rect, primitive);
                if (!common) {
                    code_lines.emplace_back("Error in generating planar rectangle");
                    continue;
                }
                if (std::fabs(common->area() - primitive.area()) > kCodeTolerance) std::swap(w, height);

                const std::array<double, 3> xyz{pyfmt::round(pln.origin.X, kTolDigits),
                                                pyfmt::round(pln.origin.Y, kTolDigits),
                                                pyfmt::round(pln.origin.Z, kTolDigits)};
                std::string pln_str;
                std::string pos_str;
                if (cfg != nullptr) {
                    pln_str = offset_code(cfg->name, cfg->sign * xyz[static_cast<std::size_t>(cfg->offset_coord)]);
                    pos_str = pos_code(xyz[static_cast<std::size_t>(cfg->pos_coords[0])],
                                       xyz[static_cast<std::size_t>(cfg->pos_coords[1])]);
                } else {
                    pos_str = "Location" + g6(Location(pln)) + " * ";
                }
                code_lines.push_back(pln_str + pos_str + "Rectangle(" + g6(w) + ", " + g6(height) +
                                     ", align=Align.MIN)");
                break;
            }
            case GeomType::CYLINDER: {
                const auto axis = primitive.axis_of_rotation();
                if (!axis) throw std::invalid_argument("the cylinder has no axis of rotation");
                Plane pln(*axis);
                const PlaneConfig* cfg = find_plane_config(pln.z_dir);
                if (cfg != nullptr) pln = Plane(pln.origin, cfg->x_dir, cfg->z_dir);

                const Face local_cyl = pln.to_local_coords(primitive);
                const double height = local_cyl.bounding_box().size.Z;
                const std::array<double, 3> xyz{pyfmt::round(pln.origin.X, kTolDigits),
                                                pyfmt::round(pln.origin.Y, kTolDigits),
                                                pyfmt::round(pln.origin.Z, kTolDigits)};
                std::string pln_str;
                std::string pos_str;
                if (cfg != nullptr) {
                    pln_str = offset_code(cfg->name, cfg->sign * xyz[static_cast<std::size_t>(cfg->offset_coord)]);
                    pos_str = pos_code(xyz[static_cast<std::size_t>(cfg->pos_coords[0])],
                                       xyz[static_cast<std::size_t>(cfg->pos_coords[1])]);
                } else {
                    pln_str = "Location" + g6(primitive.location()) + " * ";
                }
                code_lines.push_back(pln_str + pos_str + "Face.extrude(Circle(" + g6(face_radius(primitive)) +
                                     ").edge(), (0, 0, " + g6(height) + "))");
                break;
            }
            case GeomType::SPHERE:
                code_lines.push_back("Pos(" + g6(primitive.position()) + ") * Sphere(" + g6(face_radius(primitive)) +
                                     ").faces().filter_by(GeomType.SPHERE)[0]");
                break;
            default: break;
        }
    }
    return code_lines;
}

/// set().union(*(patch.face_indices for patch in patches)) if patches else set()
template <class Patch>
Set union_of_indices(const std::vector<Patch>& patches) {
    std::vector<const Set*> sets;
    for (const Patch& patch : patches) sets.push_back(&patch.face_indices);
    return Set().union_with(sets);
}

}  // namespace

std::tuple<ShapeList<Face>, ShapeList<Face>, std::vector<std::string>> detect_primitives(const Shape& mesh) {
    return detail::occt("detect_primitives", [&] {
        MeshIndex mesh_index = MeshIndex::from_shape(mesh);

        const std::vector<PlanePatch> clean_plane_patches = detect_planes_from_clean_proxy(mesh, mesh_index);
        const Set clean_plane_indices = union_of_indices(clean_plane_patches);

        const std::vector<SpherePatch> sphere_patches = detect_spheres(mesh, mesh_index, clean_plane_indices);
        const Set sphere_indices = union_of_indices(sphere_patches);

        const std::vector<CylinderPatch> cylinder_patches =
            detect_cylinders(mesh, mesh_index, clean_plane_indices | sphere_indices);
        const Set cylinder_indices = union_of_indices(cylinder_patches);

        const std::vector<PlanePatch> normal_plane_patches =
            detect_planes_from_normals(mesh, mesh_index, (clean_plane_indices | sphere_indices) | cylinder_indices);

        std::vector<DetectedPatch> patches;
        for (const auto& p : clean_plane_patches) patches.emplace_back(p);
        for (const auto& p : cylinder_patches) patches.emplace_back(p);
        for (const auto& p : sphere_patches) patches.emplace_back(p);
        for (const auto& p : normal_plane_patches) patches.emplace_back(p);

        std::vector<Face> primitives;
        Set claimed;
        for (const DetectedPatch& patch : patches) {
            std::visit(
                [&](const auto& p) {
                    const std::vector<Face> support_faces = mesh_index.face_set(p.face_indices.sorted());
                    claimed.update(p.face_indices);
                    using P = std::decay_t<decltype(p)>;
                    if constexpr (std::is_same_v<P, PlanePatch>) {
                        primitives.push_back(build_plane_face(p));
                    } else if constexpr (std::is_same_v<P, CylinderPatch>) {
                        primitives.push_back(build_cylinder_face(p, support_faces));
                    } else {
                        primitives.push_back(build_sphere_face(p, support_faces));
                    }
                },
                patch);
        }

        const Set all_faces = Set::from_range(static_cast<long long>(mesh_index.faces.size()));
        const std::vector<Face> leftovers = mesh_index.face_set(all_faces.difference(claimed).sorted());

        std::vector<std::string> code_lines = shapes_to_code(primitives);
        const std::size_t pair_count = std::min(primitives.size(), code_lines.size());
        std::vector<std::size_t> order(pair_count);
        std::vector<std::pair<std::string, double>> keys(pair_count);
        for (std::size_t i = 0; i < pair_count; ++i) {
            order[i] = i;
            keys[i] = plane_sort_key(code_lines[i]);
        }
        std::stable_sort(order.begin(), order.end(),
                         [&keys](std::size_t a, std::size_t b) { return keys[a] < keys[b]; });
        ShapeList<Face> sorted_primitives;
        std::vector<std::string> sorted_code;
        for (const std::size_t i : order) {
            sorted_primitives.push_back(primitives[i]);
            sorted_code.push_back(code_lines[i]);
        }

        // Add instance variables to the generated code
        if (!sorted_code.empty()) {
            const std::size_t num_lines = sorted_code.size();
            const auto num_digits = static_cast<int>(std::ceil(std::log10(static_cast<double>(num_lines))));
            static const std::pair<const char*, const char*> shape_keys[] = {
                {"Rectangle", "r"}, {"Circle", "c"}, {"Sphere", "s"}};
            for (std::size_t i = 0; i < num_lines; ++i) {
                std::string code_type = "None";
                for (const auto& [label, key] : shape_keys) {
                    if (sorted_code[i].find(label) != std::string::npos) {
                        code_type = key;
                        break;
                    }
                }
                std::string number = std::to_string(i);
                if (static_cast<int>(number.size()) < num_digits) {
                    number.insert(0, static_cast<std::size_t>(num_digits) - number.size(), '0');
                }
                sorted_code[i] = code_type + number + " = " + sorted_code[i];
            }
        }
        ShapeList<Face> leftover_faces;
        for (const Face& face : leftovers) leftover_faces.push_back(face);
        return std::tuple<ShapeList<Face>, ShapeList<Face>, std::vector<std::string>>{sorted_primitives,
                                                                                    leftover_faces, sorted_code};
    });
}

}  // namespace build123d
