// Matrix and OrientedBoundBox (geometry.py), Shape.transform_geometry,
// oriented_bounding_box and Solid.from_bounding_box(OrientedBoundBox).
#include <sstream>

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_GTransform.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

// ---------------------------------------------------------------------------
// Matrix
// ---------------------------------------------------------------------------

Matrix::Matrix(const std::vector<std::vector<double>>& matrix) {
    bool valid_sizes = matrix.size() == 3 || matrix.size() == 4;
    for (const auto& row : matrix) valid_sizes = valid_sizes && row.size() == 4;
    if (!valid_sizes) throw std::invalid_argument("Matrix constructor requires 2d list of 4x3 or 4x4");
    if (matrix.size() == 4 &&
        !(matrix[3][0] == 0 && matrix[3][1] == 0 && matrix[3][2] == 0 && matrix[3][3] == 1)) {
        throw std::invalid_argument("Expected the last row to be [0,0,0,1]");
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) trsf_.SetValue(i + 1, j + 1, matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
    }
}

void Matrix::rotate(const Axis& axis, double angle) {
    gp_Trsf rotation;
    rotation.SetRotation(axis.wrapped(), angle * DEG2RAD);
    trsf_ = trsf_ * gp_GTrsf(rotation);
}

Matrix Matrix::inverse() const { return Matrix(trsf_.Inverted()); }

Matrix Matrix::multiply(const Matrix& other) const { return Matrix(trsf_.Multiplied(other.trsf_)); }

Vector Matrix::multiply(const Vector& other) const { return other.transform(*this); }

std::vector<double> Matrix::transposed_list() const {
    double data[4][4] = {{0}};
    for (int i = 1; i <= 3; ++i) {
        for (int j = 1; j <= 4; ++j) data[i - 1][j - 1] = trsf_.Value(i, j);
    }
    data[3][3] = 1.0;
    std::vector<double> out;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) out.push_back(data[j][i]);
    }
    return out;
}

double Matrix::operator()(int row, int col) const {
    if (!(0 <= row && row <= 3 && 0 <= col && col <= 3)) {
        throw std::out_of_range("Out of bounds access into 4x4 matrix: (" + std::to_string(row) + ", " +
                                std::to_string(col) + ")");
    }
    if (row < 3) return trsf_.Value(row + 1, col + 1);
    return col == 3 ? 1.0 : 0.0;
}

std::string Matrix::to_string() const {
    const auto t = transposed_list();
    std::ostringstream os;
    os << "Matrix([";
    for (int i = 0; i < 4; ++i) {
        os << (i ? ",\n        " : "") << "[";
        for (int j = 0; j < 4; ++j) os << (j ? ", " : "") << t[static_cast<std::size_t>(i + 4 * j)];
        os << "]";
    }
    os << "])";
    return os.str();
}

std::ostream& operator<<(std::ostream& os, const Matrix& m) { return os << m.to_string(); }

Vector Vector::transform(const Matrix& affine_transform, bool is_direction) const {
    const gp_Trsf trsf = detail::occt("Vector.transform", [&] { return affine_transform.wrapped().Trsf(); });
    return transform(trsf, is_direction);
}

Vector Vector::round(int ndigits) const {
    return {py_round(X, ndigits), py_round(Y, ndigits), py_round(Z, ndigits)};
}

// ---------------------------------------------------------------------------
// OrientedBoundBox
// ---------------------------------------------------------------------------

OrientedBoundBox::OrientedBoundBox(const Shape& shape) {
    if (shape.is_null()) throw std::invalid_argument("Expected Bnd_OBB or Shape, got an empty shape");
    occt("OrientedBoundBox", [&] {
        BRepBndLib::AddOBB(shape.wrapped(), obb_, true);
        return 0;
    });
}

std::vector<Vector> OrientedBoundBox::corners() const {
    const Vector hs = size() * 0.5;
    const bool dx = hs.X < TOLERANCE, dy = hs.Y < TOLERANCE, dz = hs.Z < TOLERANCE;
    std::vector<std::array<int, 3>> order;
    if (dx && dy && !dz) {
        order = {{1, 1, 1}, {1, 1, -1}};
    } else if (dx && !dy && dz) {
        order = {{1, 1, 1}, {1, -1, 1}};
    } else if (!dx && dy && dz) {
        order = {{1, 1, 1}, {-1, 1, 1}};
    } else if (dx && !dy && !dz) {
        order = {{1, 1, 1}, {1, 1, -1}, {1, -1, -1}, {1, -1, 1}};
    } else if (!dx && dy && !dz) {
        order = {{1, 1, 1}, {1, 1, -1}, {-1, 1, -1}, {-1, 1, 1}};
    } else if (!dx && !dy && dz) {
        order = {{1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, 1}};
    } else if (!dx && !dy && !dz) {
        for (int sx : {-1, 1}) {
            for (int sy : {-1, 1}) {
                for (int sz : {-1, 1}) order.push_back({sx, sy, sz});
            }
        }
    } else {
        throw std::invalid_argument("The oriented bounding box is a point");  // KeyError in build123d
    }
    const Plane p = plane();
    std::vector<Vector> out;
    for (const auto& [sx, sy, sz] : order) out.push_back(p.from_local_coords(Vector(sx * hs.X, sy * hs.Y, sz * hs.Z)));
    return out;
}

double OrientedBoundBox::diagonal() const { return std::pow(obb_.SquareExtent(), 0.5); }
Location OrientedBoundBox::location() const { return Location(plane()); }
Plane OrientedBoundBox::plane() const { return Plane(center(), x_direction(), z_direction()); }
Vector OrientedBoundBox::size() const { return Vector(obb_.XHSize(), obb_.YHSize(), obb_.ZHSize()) * 2.0; }
Vector OrientedBoundBox::x_direction() const { return Vector(obb_.XDirection()); }
Vector OrientedBoundBox::y_direction() const { return Vector(obb_.YDirection()); }
Vector OrientedBoundBox::z_direction() const { return Vector(obb_.ZDirection()); }
Vector OrientedBoundBox::center() const { return Vector(obb_.Center()); }
bool OrientedBoundBox::is_completely_inside(const OrientedBoundBox& other) const {
    return obb_.IsCompletelyInside(other.obb_);
}
bool OrientedBoundBox::is_outside(const Vector& point) const { return obb_.IsOut(point.to_pnt()); }

std::string OrientedBoundBox::to_string() const {
    return "OrientedBoundBox(center=" + center().to_string() + ", size=" + size().to_string() +
           ", plane=" + plane().to_string() + ")";
}

std::ostream& operator<<(std::ostream& os, const OrientedBoundBox& b) { return os << b.to_string(); }

// ---------------------------------------------------------------------------
// Shape / Solid
// ---------------------------------------------------------------------------

OrientedBoundBox Shape::oriented_bounding_box() const {
    if (obj_.IsNull()) return OrientedBoundBox(Bnd_OBB());
    return OrientedBoundBox(*this);
}

TopoDS_Shape Shape::gtransform_topods(const Matrix& t_matrix) const {
    if (obj_.IsNull()) return obj_;
    return occt("transform_geometry", [&] { return BRepBuilderAPI_GTransform(obj_, t_matrix.wrapped(), true).Shape(); });
}

Solid Solid::from_bounding_box(const OrientedBoundBox& bbox) {
    const Plane moved_plane = Plane(Location(bbox.size() * -0.5)).moved(bbox.location());
    return make_box(bbox.size().X, bbox.size().Y, bbox.size().Z, moved_plane);
}

}  // namespace build123d
