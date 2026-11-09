// build123d-cpp: a small JSON value type with Python's json.dumps/json.loads behaviour, and
// GeomEncoder (geometry.py), which encodes Vector, Axis, Color, Location and Plane.
#pragma once

#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "build123d/geometry.hpp"

namespace build123d {

/// A JSON value. Integers and floats are kept apart, as in Python (1 and 1.0), and objects
/// keep their insertion order, like Python dicts.
class Json {
public:
    enum class Type { NUL, BOOL, INT, FLOAT, STRING, ARRAY, OBJECT };
    using Array = std::vector<Json>;
    using Object = std::vector<std::pair<std::string, Json>>;

    Json() = default;
    Json(std::nullptr_t) {}  // NOLINT
    Json(bool b) : type_(Type::BOOL), bool_(b) {}  // NOLINT
    template <class T, std::enable_if_t<std::is_integral_v<T> && !std::is_same_v<T, bool>, int> = 0>
    Json(T i) : type_(Type::INT), int_(static_cast<std::int64_t>(i)) {}  // NOLINT
    template <class T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
    Json(T x) : type_(Type::FLOAT), float_(static_cast<double>(x)) {}  // NOLINT
    Json(const char* s) : type_(Type::STRING), string_(s) {}  // NOLINT
    Json(std::string s) : type_(Type::STRING), string_(std::move(s)) {}  // NOLINT
    Json(Array a) : type_(Type::ARRAY), array_(std::move(a)) {}  // NOLINT
    Json(Object o) : type_(Type::OBJECT), object_(std::move(o)) {}  // NOLINT

    static Json array(std::initializer_list<Json> items = {}) { return Json(Array(items)); }
    static Json object(std::initializer_list<std::pair<std::string, Json>> items = {}) {
        return Json(Object(items));
    }

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::NUL; }
    bool is_bool() const { return type_ == Type::BOOL; }
    bool is_number() const { return type_ == Type::INT || type_ == Type::FLOAT; }
    bool is_string() const { return type_ == Type::STRING; }
    bool is_array() const { return type_ == Type::ARRAY; }
    bool is_object() const { return type_ == Type::OBJECT; }

    bool as_bool() const;
    std::int64_t as_int() const;
    /// The value of an INT or FLOAT.
    double as_double() const;
    const std::string& as_string() const;
    const Array& as_array() const;
    Array& as_array();
    const Object& as_object() const;
    Object& as_object();

    /// Number of items of an array or object.
    std::size_t size() const;
    bool contains(const std::string& key) const;
    /// Object member (std::out_of_range if missing).
    const Json& operator[](const std::string& key) const;
    const Json& operator[](const char* key) const { return (*this)[std::string(key)]; }
    /// Object member, added (as null) if missing; a null value becomes an object.
    Json& operator[](const std::string& key);
    Json& operator[](const char* key) { return (*this)[std::string(key)]; }
    /// Array item (std::out_of_range if out of range).
    const Json& at(std::size_t index) const;
    Json& at(std::size_t index);
    void push_back(Json value);

    /// json.dumps(value, indent=indent): Python's separators, float repr, and ASCII-only
    /// output (other characters as \uXXXX escapes).
    std::string dump(std::optional<int> indent = std::nullopt) const;
    /// json.loads(text); NaN, Infinity and -Infinity are accepted as in Python.
    /// Raises std::invalid_argument on malformed input.
    static Json parse(const std::string& text);

    bool operator==(const Json& other) const;
    bool operator!=(const Json& other) const { return !(*this == other); }

private:
    Type type_ = Type::NUL;
    bool bool_ = false;
    std::int64_t int_ = 0;
    double float_ = 0.0;
    std::string string_;
    Array array_;
    Object object_;
};

/// The geometry types GeomEncoder knows (its CLASS_REGISTRY).
using Geometry = std::variant<Axis, Color, Location, Plane, Vector>;

/// JSON encoding of geometry objects (geometry.py GeomEncoder). Each object becomes a
/// one-key object naming its type, e.g. {"Vector": [1.0, 2.0, 3.0]}:
///
///     Json data = Json::object({{"my_axis", GeomEncoder::encode(axis)}});
///     std::string text = data.dump(4);                       // json.dumps(..., indent=4)
///     Axis axis2 = std::get<Axis>(GeomEncoder::geometry_hook(Json::parse(text)["my_axis"]));
class GeomEncoder {
public:
    /// GeomEncoder.default(o): the JSON form of a geometry object.
    static Json encode(const Axis& o);
    static Json encode(const Color& o);
    static Json encode(const Location& o);
    static Json encode(const Plane& o);
    static Json encode(const Vector& o);
    static Json encode(const Geometry& o);
    /// json.dumps(o, cls=GeomEncoder, indent=indent) for a single geometry object.
    template <class G>
    static std::string dumps(const G& o, std::optional<int> indent = std::nullopt) {
        return encode(o).dump(indent);
    }
    /// GeomEncoder.geometry_hook: the object a one-key JSON object describes.
    /// std::invalid_argument if it doesn't have exactly one key; std::out_of_range if the
    /// key isn't a geometry type (Python's KeyError).
    static Geometry geometry_hook(const Json& json_dict);
    /// json.loads(text, object_hook=GeomEncoder.geometry_hook) for a single geometry object.
    static Geometry loads(const std::string& text) { return geometry_hook(Json::parse(text)); }
};

}  // namespace build123d
