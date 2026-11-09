// Json (Python's json module behaviour) and GeomEncoder (geometry.py).
#include "build123d/json.hpp"

#include <cmath>
#include <cstdio>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "pyformat.hpp"

namespace build123d {

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

namespace {

[[noreturn]] void type_error(const char* expected) {
    throw std::invalid_argument(std::string("Json value is not ") + expected);
}

}  // namespace

bool Json::as_bool() const {
    if (type_ != Type::BOOL) type_error("a bool");
    return bool_;
}

std::int64_t Json::as_int() const {
    if (type_ != Type::INT) type_error("an int");
    return int_;
}

double Json::as_double() const {
    if (type_ == Type::INT) return static_cast<double>(int_);
    if (type_ != Type::FLOAT) type_error("a number");
    return float_;
}

const std::string& Json::as_string() const {
    if (type_ != Type::STRING) type_error("a string");
    return string_;
}

const Json::Array& Json::as_array() const {
    if (type_ != Type::ARRAY) type_error("an array");
    return array_;
}

Json::Array& Json::as_array() {
    if (type_ != Type::ARRAY) type_error("an array");
    return array_;
}

const Json::Object& Json::as_object() const {
    if (type_ != Type::OBJECT) type_error("an object");
    return object_;
}

Json::Object& Json::as_object() {
    if (type_ != Type::OBJECT) type_error("an object");
    return object_;
}

std::size_t Json::size() const {
    if (type_ == Type::ARRAY) return array_.size();
    if (type_ == Type::OBJECT) return object_.size();
    type_error("an array or object");
}

bool Json::contains(const std::string& key) const {
    if (type_ != Type::OBJECT) return false;
    for (const auto& kv : object_) {
        if (kv.first == key) return true;
    }
    return false;
}

const Json& Json::operator[](const std::string& key) const {
    for (const auto& kv : as_object()) {
        if (kv.first == key) return kv.second;
    }
    throw std::out_of_range("Json object has no key '" + key + "'");
}

Json& Json::operator[](const std::string& key) {
    if (type_ == Type::NUL) type_ = Type::OBJECT;
    for (auto& kv : as_object()) {
        if (kv.first == key) return kv.second;
    }
    object_.emplace_back(key, Json());
    return object_.back().second;
}

const Json& Json::at(std::size_t index) const {
    const Array& a = as_array();
    if (index >= a.size()) throw std::out_of_range("Json array index out of range");
    return a[index];
}

Json& Json::at(std::size_t index) {
    Array& a = as_array();
    if (index >= a.size()) throw std::out_of_range("Json array index out of range");
    return a[index];
}

void Json::push_back(Json value) {
    if (type_ == Type::NUL) type_ = Type::ARRAY;
    as_array().push_back(std::move(value));
}

bool Json::operator==(const Json& o) const {
    if (is_number() && o.is_number()) {
        if (type_ == Type::INT && o.type_ == Type::INT) return int_ == o.int_;
        return as_double() == o.as_double();
    }
    if (type_ != o.type_) return false;
    switch (type_) {
        case Type::NUL: return true;
        case Type::BOOL: return bool_ == o.bool_;
        case Type::STRING: return string_ == o.string_;
        case Type::ARRAY: return array_ == o.array_;
        case Type::OBJECT: {
            // Python compares dicts without regard to order
            if (object_.size() != o.object_.size()) return false;
            for (const auto& kv : object_) {
                if (!o.contains(kv.first) || !(o[kv.first] == kv.second)) return false;
            }
            return true;
        }
        default: return false;
    }
}

// ---------------------------------------------------------------------------
// json.dumps
// ---------------------------------------------------------------------------

namespace {

void append_hex4(std::string& out, unsigned v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "\\u%04x", v & 0xFFFFu);
    out += buf;
}

/// Python's encode_basestring_ascii: the string quoted, with everything outside ' '..'~'
/// escaped. The input is UTF-8.
std::string quote_ascii(const std::string& s) {
    std::string out = "\"";
    const auto* p = reinterpret_cast<const unsigned char*>(s.data());
    const auto* end = p + s.size();
    while (p < end) {
        unsigned cp = *p;
        int extra = 0;
        if (cp >= 0xF0 && cp < 0xF8) {
            cp &= 0x07;
            extra = 3;
        } else if (cp >= 0xE0) {
            cp &= 0x0F;
            extra = 2;
        } else if (cp >= 0xC0) {
            cp &= 0x1F;
            extra = 1;
        }
        ++p;
        for (int i = 0; i < extra && p < end && (*p & 0xC0) == 0x80; ++i, ++p) cp = (cp << 6) | (*p & 0x3F);
        switch (cp) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (cp >= 0x20 && cp <= 0x7E) {
                    out += static_cast<char>(cp);
                } else if (cp < 0x10000) {
                    append_hex4(out, cp);
                } else {
                    const unsigned n = cp - 0x10000;
                    append_hex4(out, 0xD800 | ((n >> 10) & 0x3FF));
                    append_hex4(out, 0xDC00 | (n & 0x3FF));
                }
        }
    }
    return out + "\"";
}

std::string float_text(double x) {
    if (std::isnan(x)) return "NaN";
    if (std::isinf(x)) return x > 0 ? "Infinity" : "-Infinity";
    return pyfmt::repr(x);
}

void dump_value(const Json& v, std::optional<int> indent, int level, std::string& out, bool compact) {
    const std::string item_separator = indent || compact ? "," : ", ";
    const char* const key_separator = compact ? ":" : ": ";
    switch (v.type()) {
        case Json::Type::NUL: out += "null"; return;
        case Json::Type::BOOL: out += v.as_bool() ? "true" : "false"; return;
        case Json::Type::INT: out += std::to_string(v.as_int()); return;
        case Json::Type::FLOAT: out += float_text(v.as_double()); return;
        case Json::Type::STRING: out += quote_ascii(v.as_string()); return;
        case Json::Type::ARRAY: {
            const auto& a = v.as_array();
            if (a.empty()) {
                out += "[]";
                return;
            }
            out += "[";
            std::string newline_indent;
            if (indent) {
                newline_indent = "\n" + std::string(static_cast<std::size_t>(*indent * (level + 1)), ' ');
                out += newline_indent;
            }
            for (std::size_t i = 0; i < a.size(); ++i) {
                if (i) out += item_separator + newline_indent;
                dump_value(a[i], indent, level + 1, out, compact);
            }
            if (indent) out += "\n" + std::string(static_cast<std::size_t>(*indent * level), ' ');
            out += "]";
            return;
        }
        case Json::Type::OBJECT: {
            const auto& o = v.as_object();
            if (o.empty()) {
                out += "{}";
                return;
            }
            out += "{";
            std::string newline_indent;
            if (indent) {
                newline_indent = "\n" + std::string(static_cast<std::size_t>(*indent * (level + 1)), ' ');
                out += newline_indent;
            }
            for (std::size_t i = 0; i < o.size(); ++i) {
                if (i) out += item_separator + newline_indent;
                out += quote_ascii(o[i].first);
                out += key_separator;
                dump_value(o[i].second, indent, level + 1, out, compact);
            }
            if (indent) out += "\n" + std::string(static_cast<std::size_t>(*indent * level), ' ');
            out += "}";
            return;
        }
    }
}

}  // namespace

std::string Json::dump(std::optional<int> indent) const {
    if (indent && *indent < 0) indent = 0;
    std::string out;
    dump_value(*this, indent, 0, out, false);
    return out;
}

std::string Json::dump_compact() const {
    std::string out;
    dump_value(*this, std::nullopt, 0, out, true);
    return out;
}

// ---------------------------------------------------------------------------
// json.loads
// ---------------------------------------------------------------------------

namespace {

class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    Json parse_document() {
        skip_ws();
        Json v = parse_value();
        skip_ws();
        if (i_ != s_.size()) fail("Extra data");
        return v;
    }

private:
    const std::string& s_;
    std::size_t i_ = 0;

    [[noreturn]] void fail(const std::string& what) const {
        throw std::invalid_argument(what + ": char " + std::to_string(i_));
    }
    void skip_ws() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) ++i_;
    }
    bool starts_with(const char* word) const { return s_.compare(i_, std::char_traits<char>::length(word), word) == 0; }

    Json parse_value() {
        if (i_ >= s_.size()) fail("Expecting value");
        const char c = s_[i_];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return Json(parse_string());
        if (starts_with("null")) {
            i_ += 4;
            return Json();
        }
        if (starts_with("true")) {
            i_ += 4;
            return Json(true);
        }
        if (starts_with("false")) {
            i_ += 5;
            return Json(false);
        }
        if (starts_with("NaN")) {
            i_ += 3;
            return Json(std::nan(""));
        }
        if (starts_with("Infinity")) {
            i_ += 8;
            return Json(HUGE_VAL);
        }
        if (starts_with("-Infinity")) {
            i_ += 9;
            return Json(-HUGE_VAL);
        }
        if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
        fail("Expecting value");
    }

    Json parse_number() {
        const std::size_t start = i_;
        if (s_[i_] == '-') ++i_;
        if (i_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[i_]))) fail("Expecting value");
        if (s_[i_] == '0') {
            ++i_;
        } else {
            while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
        }
        bool is_float = false;
        if (i_ + 1 < s_.size() && s_[i_] == '.' && std::isdigit(static_cast<unsigned char>(s_[i_ + 1]))) {
            is_float = true;
            ++i_;
            while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
        }
        if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            std::size_t j = i_ + 1;
            if (j < s_.size() && (s_[j] == '+' || s_[j] == '-')) ++j;
            if (j < s_.size() && std::isdigit(static_cast<unsigned char>(s_[j]))) {
                is_float = true;
                i_ = j;
                while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
            }
        }
        const std::string text = s_.substr(start, i_ - start);
        if (!is_float) {
            errno = 0;
            const long long v = std::strtoll(text.c_str(), nullptr, 10);
            if (errno != ERANGE) return Json(static_cast<std::int64_t>(v));
        }
        return Json(std::strtod(text.c_str(), nullptr));
    }

    static void append_utf8(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    unsigned hex4() {
        if (i_ + 4 > s_.size()) fail("Invalid \\uXXXX escape");
        unsigned v = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = s_[i_++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<unsigned>(c - 'A' + 10);
            else fail("Invalid \\uXXXX escape");
        }
        return v;
    }

    std::string parse_string() {
        ++i_;  // opening quote
        std::string out;
        while (true) {
            if (i_ >= s_.size()) fail("Unterminated string starting at");
            const char c = s_[i_++];
            if (c == '"') break;
            if (static_cast<unsigned char>(c) < 0x20) fail("Invalid control character at");
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i_ >= s_.size()) fail("Unterminated string starting at");
            const char e = s_[i_++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp = hex4();
                    if (cp >= 0xD800 && cp <= 0xDBFF && i_ + 6 <= s_.size() && s_[i_] == '\\' && s_[i_ + 1] == 'u') {
                        const std::size_t save = i_;
                        i_ += 2;
                        const unsigned low = hex4();
                        if (low >= 0xDC00 && low <= 0xDFFF) {
                            cp = 0x10000 + (((cp - 0xD800) << 10) | (low - 0xDC00));
                        } else {
                            i_ = save;
                        }
                    }
                    append_utf8(out, cp);
                    break;
                }
                default: fail("Invalid \\escape");
            }
        }
        return out;
    }

    Json parse_array() {
        ++i_;
        Json::Array items;
        skip_ws();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return Json(std::move(items));
        }
        while (true) {
            skip_ws();
            items.push_back(parse_value());
            skip_ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == ']') {
                ++i_;
                return Json(std::move(items));
            }
            fail("Expecting ',' delimiter");
        }
    }

    Json parse_object() {
        ++i_;
        Json::Object members;
        skip_ws();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return Json(std::move(members));
        }
        while (true) {
            skip_ws();
            if (i_ >= s_.size() || s_[i_] != '"') fail("Expecting property name enclosed in double quotes");
            std::string key = parse_string();
            skip_ws();
            if (i_ >= s_.size() || s_[i_] != ':') fail("Expecting ':' delimiter");
            ++i_;
            skip_ws();
            Json value = parse_value();
            // dict(pairs): a repeated key keeps its first position and its last value
            bool replaced = false;
            for (auto& kv : members) {
                if (kv.first == key) {
                    kv.second = std::move(value);
                    replaced = true;
                    break;
                }
            }
            if (!replaced) members.emplace_back(std::move(key), std::move(value));
            skip_ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == '}') {
                ++i_;
                return Json(std::move(members));
            }
            fail("Expecting ',' delimiter");
        }
    }
};

}  // namespace

Json Json::parse(const std::string& text) { return Parser(text).parse_document(); }

// ---------------------------------------------------------------------------
// GeomEncoder
// ---------------------------------------------------------------------------

namespace {

Json vec(const Vector& v) { return Json::array({v.X, v.Y, v.Z}); }

Vector to_vector(const Json& j) {
    const auto& a = j.as_array();
    if (a.size() == 2) return Vector(a[0].as_double(), a[1].as_double());
    if (a.size() != 3) throw std::invalid_argument("A vector needs 2 or 3 values");
    return Vector(a[0].as_double(), a[1].as_double(), a[2].as_double());
}

}  // namespace

Json GeomEncoder::encode(const Axis& o) { return Json::object({{"Axis", Json::array({vec(o.position), vec(o.direction)})}}); }

Json GeomEncoder::encode(const Color& o) {
    return Json::object({{"Color", Json::array({o.red, o.green, o.blue, o.alpha})}});
}

Json GeomEncoder::encode(const Location& o) {
    return Json::object({{"Location", Json::array({vec(o.position()), vec(o.orientation())})}});
}

Json GeomEncoder::encode(const Plane& o) {
    return Json::object({{"Plane", Json::array({vec(o.origin), vec(o.x_dir), vec(o.z_dir)})}});
}

Json GeomEncoder::encode(const Vector& o) { return Json::object({{"Vector", vec(o)}}); }

Json GeomEncoder::encode(const Geometry& o) {
    return std::visit([](const auto& g) { return encode(g); }, o);
}

Geometry GeomEncoder::geometry_hook(const Json& json_dict) {
    if (!json_dict.is_object() || json_dict.size() != 1) {
        throw std::invalid_argument("Invalid geometry json object " + json_dict.dump());
    }
    const auto& [key, value] = json_dict.as_object().front();
    const auto& args = value.as_array();
    if (key == "Axis") {
        if (args.size() != 2) throw std::invalid_argument("Axis needs a position and a direction");
        return Axis(to_vector(args[0]), to_vector(args[1]));
    }
    if (key == "Color") {
        if (args.size() < 3 || args.size() > 4) throw std::invalid_argument("Color needs 3 or 4 values");
        return Color(args[0].as_double(), args[1].as_double(), args[2].as_double(),
                     args.size() == 4 ? args[3].as_double() : 1.0);
    }
    if (key == "Location") {
        if (args.size() != 2) throw std::invalid_argument("Location needs a position and an orientation");
        return Location(to_vector(args[0]), to_vector(args[1]));
    }
    if (key == "Plane") {
        if (args.size() != 3) throw std::invalid_argument("Plane needs an origin, x_dir and z_dir");
        return Plane(to_vector(args[0]), to_vector(args[1]), to_vector(args[2]));
    }
    if (key == "Vector") return to_vector(value);
    throw std::out_of_range("'" + key + "'");
}

}  // namespace build123d
