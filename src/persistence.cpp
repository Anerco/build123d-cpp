// Shapes and locations to and from bytes (persistence.py).
#include "build123d/persistence.hpp"

#include <cstring>
#include <sstream>
#include <string>

#include <BinTools.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "internal.hpp"

namespace build123d {

using namespace detail;

Bytes serialize_shape(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return {};
    return occt("serialize_shape", [&] {
        std::ostringstream stream(std::ios::out | std::ios::binary);
        BinTools::Write(shape, stream);
        const std::string s = stream.str();
        return Bytes(s.begin(), s.end());
    });
}

TopoDS_Shape deserialize_shape(const Bytes& buffer) {
    if (buffer.empty()) return TopoDS_Shape();
    return occt("deserialize_shape", [&] {
        std::istringstream stream(std::string(buffer.begin(), buffer.end()), std::ios::in | std::ios::binary);
        TopoDS_Shape shape;
        BinTools::Read(shape, stream);
        if (shape.IsNull()) throw std::runtime_error("deserialize_shape: no shape in the data");
        return shape;
    });
}

Bytes serialize_location(const TopLoc_Location& location) {
    const gp_Trsf transform = location.Transformation();
    const gp_XYZ translation = transform.TranslationPart();
    const gp_Quaternion rotation = transform.GetRotation();
    const float values[7] = {static_cast<float>(translation.Coord(1)), static_cast<float>(translation.Coord(2)),
                             static_cast<float>(translation.Coord(3)), static_cast<float>(rotation.X()),
                             static_cast<float>(rotation.Y()),        static_cast<float>(rotation.Z()),
                             static_cast<float>(rotation.W())};
    Bytes buffer(sizeof(values));
    std::memcpy(buffer.data(), values, sizeof(values));
    return buffer;
}

TopLoc_Location deserialize_location(const Bytes& buffer) {
    if (buffer.empty()) return TopLoc_Location();
    float values[7];
    if (buffer.size() < sizeof(values)) throw std::invalid_argument("deserialize_location: needs 28 bytes");
    std::memcpy(values, buffer.data(), sizeof(values));
    const gp_Vec translation(values[0], values[1], values[2]);
    const gp_Quaternion rotation(values[3], values[4], values[5], values[6]);
    gp_Trsf transform;
    transform.SetTransformation(rotation, translation);
    return TopLoc_Location(transform);
}

// ---------------------------------------------------------------------------
// Whole objects
// ---------------------------------------------------------------------------

namespace {

constexpr char kMagic[] = "b123dcpp";
constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kShape = 'S';
constexpr std::uint8_t kLocation = 'L';

class Writer {
public:
    Bytes out;
    void u8(std::uint8_t v) { out.push_back(v); }
    void u32(std::uint32_t v) {
        for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    void f64(double v) {
        std::uint64_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
    }
    void bytes(const Bytes& b) {
        u32(static_cast<std::uint32_t>(b.size()));
        out.insert(out.end(), b.begin(), b.end());
    }
    void str(const std::string& s) { bytes(Bytes(s.begin(), s.end())); }
    void header(std::uint8_t kind) {
        out.insert(out.end(), kMagic, kMagic + 8);
        u8(kVersion);
        u8(kind);
    }
    void shape(const Shape& s) {
        const auto parent = s.topo_parent();
        std::uint8_t flags = 0;
        if (s.color) flags |= 1;
        if (parent && !parent->is_null()) flags |= 2;
        u8(flags);
        str(s.label);
        if (s.color) {
            f64(s.color->red);
            f64(s.color->green);
            f64(s.color->blue);
            f64(s.color->alpha);
        }
        bytes(serialize_shape(s.wrapped()));
        if (flags & 2) bytes(serialize_shape(parent->wrapped()));
        u32(static_cast<std::uint32_t>(s.children.size()));
        for (const auto& child : s.children) shape(child);
    }
};

class Reader {
public:
    explicit Reader(const Bytes& data) : data_(data) {}
    std::uint8_t u8() {
        need(1);
        return data_[pos_++];
    }
    std::uint32_t u32() {
        need(4);
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(data_[pos_++]) << (8 * i);
        return v;
    }
    double f64() {
        need(8);
        std::uint64_t bits = 0;
        for (int i = 0; i < 8; ++i) bits |= static_cast<std::uint64_t>(data_[pos_++]) << (8 * i);
        double v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    }
    Bytes bytes() {
        const std::uint32_t n = u32();
        need(n);
        Bytes b(data_.begin() + static_cast<std::ptrdiff_t>(pos_), data_.begin() + static_cast<std::ptrdiff_t>(pos_ + n));
        pos_ += n;
        return b;
    }
    std::string str() {
        const Bytes b = bytes();
        return std::string(b.begin(), b.end());
    }
    void header(std::uint8_t kind) {
        need(10);
        if (std::memcmp(data_.data(), kMagic, 8) != 0) throw std::invalid_argument("Not build123d-cpp serialized data");
        pos_ = 8;
        if (u8() != kVersion) throw std::invalid_argument("Unsupported build123d-cpp serialization version");
        if (u8() != kind) {
            throw std::invalid_argument(kind == kShape ? "The data holds no shape" : "The data holds no location");
        }
    }
    Shape shape() {
        const std::uint8_t flags = u8();
        const std::string label = str();
        std::optional<Color> color;
        if (flags & 1) {
            const double r = f64(), g = f64(), b = f64(), a = f64();
            color = Color(r, g, b, a);
        }
        Shape s(deserialize_shape(bytes()));
        if (flags & 2) s.set_topo_parent(Shape(deserialize_shape(bytes())));
        s.label = label;
        s.color = color;
        const std::uint32_t n = u32();
        for (std::uint32_t i = 0; i < n; ++i) s.children.push_back(shape());
        return s;
    }
    void finish() const {
        if (pos_ != data_.size()) throw std::invalid_argument("Unexpected data after the serialized object");
    }

private:
    const Bytes& data_;
    std::size_t pos_ = 0;
    void need(std::size_t n) const {
        if (pos_ + n > data_.size()) throw std::invalid_argument("Truncated build123d-cpp serialized data");
    }
};

}  // namespace

Bytes to_bytes(const Shape& shape) {
    Writer w;
    w.header(kShape);
    w.shape(shape);
    return w.out;
}

Bytes to_bytes(const Location& location) {
    Writer w;
    w.header(kLocation);
    w.bytes(serialize_location(location.wrapped()));
    return w.out;
}

Shape shape_from_bytes(const Bytes& data) {
    Reader r(data);
    r.header(kShape);
    Shape s = r.shape();
    r.finish();
    return s;
}

Location location_from_bytes(const Bytes& data) {
    Reader r(data);
    r.header(kLocation);
    const Location loc(deserialize_location(r.bytes()));
    r.finish();
    return loc;
}

}  // namespace build123d
