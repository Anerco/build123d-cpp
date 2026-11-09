// Registry for the comparison cases. cases.cpp holds the original cases; every other
// tests/compare/cases_<area>.cpp file registers more with COMPARE_CASES, and the matching
// cases_<area>_py.py builds the same cases in Python (see compare.py).
//
//   COMPARE_CASES {
//       add("my_box", [] { return Box(1, 2, 3); });              // compared by shape metrics
//       add_value("my_point", [] { return V(Vector(1, 2, 3)); });  // compared value by value
//   }
#pragma once

#include <build123d/build123d.hpp>

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace compare {

/// A JSON value for cases whose result is not a shape. compare.py compares numbers with a
/// tolerance and everything else exactly. A Vector or Vertex becomes [X, Y, Z].
class V {
public:
    enum class Kind { NUL, BOOL, NUMBER, STRING, LIST, OBJECT };

    V() = default;
    V(std::nullptr_t) {}
    V(bool b) : kind_(Kind::BOOL), bool_(b) {}
    template <class T, std::enable_if_t<std::is_arithmetic_v<T> && !std::is_same_v<T, bool>, int> = 0>
    V(T x) : kind_(Kind::NUMBER), number_(static_cast<double>(x)) {}
    V(const char* s) : kind_(Kind::STRING), string_(s) {}
    V(std::string s) : kind_(Kind::STRING), string_(std::move(s)) {}
    V(const build123d::Vector& v) : V(list({v.X, v.Y, v.Z})) {}
    V(const build123d::Vertex& v) : V(list({v.X(), v.Y(), v.Z()})) {}

    static V list(std::vector<V> items) {
        V v;
        v.kind_ = Kind::LIST;
        v.items_ = std::move(items);
        return v;
    }
    static V obj(std::vector<std::pair<std::string, V>> fields) {
        V v;
        v.kind_ = Kind::OBJECT;
        v.fields_ = std::move(fields);
        return v;
    }
    /// A list from any container of values convertible to V.
    template <class Container>
    static V list_of(const Container& c) {
        std::vector<V> items;
        for (const auto& x : c) items.emplace_back(x);
        return list(std::move(items));
    }

    std::string dump() const {
        switch (kind_) {
            case Kind::NUL: return "null";
            case Kind::BOOL: return bool_ ? "true" : "false";
            case Kind::NUMBER: {
                if (std::isnan(number_)) return "NaN";
                if (std::isinf(number_)) return number_ > 0 ? "Infinity" : "-Infinity";
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.17g", number_);
                return buf;
            }
            case Kind::STRING: return quote(string_);
            case Kind::LIST: {
                std::string out = "[";
                for (std::size_t i = 0; i < items_.size(); ++i) out += (i ? ", " : "") + items_[i].dump();
                return out + "]";
            }
            case Kind::OBJECT: {
                std::string out = "{";
                for (std::size_t i = 0; i < fields_.size(); ++i) {
                    out += (i ? ", " : "") + quote(fields_[i].first) + ": " + fields_[i].second.dump();
                }
                return out + "}";
            }
        }
        return "null";
    }

    static std::string quote(const std::string& s) {
        std::string out = "\"";
        for (const unsigned char c : s) {
            if (c == '"' || c == '\\') {
                out += '\\';
                out += static_cast<char>(c);
            } else if (c == '\n') {
                out += "\\n";
            } else if (c == '\t') {
                out += "\\t";
            } else if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += static_cast<char>(c);
            }
        }
        return out + "\"";
    }

private:
    Kind kind_ = Kind::NUL;
    bool bool_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<V> items_;
    std::vector<std::pair<std::string, V>> fields_;
};

struct Entry {
    std::string name;
    std::function<build123d::Shape()> shape;  // set for shape cases
    std::function<V()> value;                 // set for value cases
};

inline std::vector<Entry>& registry() {
    static std::vector<Entry> entries;
    return entries;
}

/// A case compared by the metrics of the shape it returns.
inline void add(std::string name, std::function<build123d::Shape()> build) {
    registry().push_back({std::move(name), std::move(build), {}});
}

/// A case compared value by value.
inline void add_value(std::string name, std::function<V()> build) {
    registry().push_back({std::move(name), {}, std::move(build)});
}

}  // namespace compare

#define COMPARE_CASES_CAT2(a, b) a##b
#define COMPARE_CASES_CAT(a, b) COMPARE_CASES_CAT2(a, b)
/// Registers the cases of one file when the program starts.
#define COMPARE_CASES                                                                         \
    static void COMPARE_CASES_CAT(compare_cases_, __LINE__)();                                \
    static const bool COMPARE_CASES_CAT(compare_cases_registered_, __LINE__) =                \
        (COMPARE_CASES_CAT(compare_cases_, __LINE__)(), true);                                \
    static void COMPARE_CASES_CAT(compare_cases_, __LINE__)()
