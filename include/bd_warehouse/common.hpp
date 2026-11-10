// bd_warehouse-cpp: types shared by the bd_warehouse modules.
//
// bd_warehouse is a parametric part library for build123d (threads, fasteners, bearings,
// flanges, pipes, gears, sprockets, o-rings, retaining rings, shaft keys, bushings and
// OpenBuilds parts). This is its C++ port on build123d-cpp: namespace bd_warehouse, one header
// per Python module (include/bd_warehouse/<module>.hpp) and bd_warehouse.hpp for all of them.
//
// Conventions of the port:
//   - Parameters keep Python's names, order and defaults. Python's string choices
//     (Literal["right", "left"], ...) stay strings with the same accepted values, checked as
//     Python checks them. Python's `mode` (a builder-only argument) is left out: combine parts
//     with the algebra (`part -= Pos(0, 0, 10) * ClearanceHole(screw)`).
//   - Python's ValueError is std::invalid_argument, KeyError and IndexError std::out_of_range,
//     RuntimeError std::runtime_error, ZeroDivisionError std::domain_error, and warnings go
//     through build123d::warn.
//   - Python's dicts are bd_warehouse::Dict (insertion ordered), table values bd_warehouse::Param.
#pragma once

#include <cstddef>
#include <initializer_list>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace bd_warehouse {

namespace detail {
std::string key_repr(const std::string& key);
std::string key_repr(int key);
std::string key_repr(long long key);
std::string key_repr(double key);
template <class A, class B>
std::string key_repr(const std::pair<A, B>& key) {
    return "(" + key_repr(key.first) + ", " + key_repr(key.second) + ")";
}
}  // namespace detail

/// An insertion-ordered dictionary, the C++ form of the Python dicts bd_warehouse returns
/// (size tables, parameter tables, ...). Iterating gives (key, value) pairs in insertion order;
/// `at` of a missing key throws std::out_of_range with Python's KeyError message (the key's
/// repr, e.g. "'M6-1'").
template <class K, class V>
class Dict {
public:
    using value_type = std::pair<K, V>;
    using const_iterator = typename std::vector<value_type>::const_iterator;
    using iterator = typename std::vector<value_type>::iterator;

    Dict() = default;
    Dict(std::initializer_list<value_type> items) {
        for (const auto& item : items) (*this)[item.first] = item.second;
    }

    /// d[key]: the value, inserted (default constructed) at the end when the key is new.
    V& operator[](const K& key) {
        const auto it = index_.find(key);
        if (it != index_.end()) return items_[it->second].second;
        index_.emplace(key, items_.size());
        items_.emplace_back(key, V{});
        return items_.back().second;
    }
    /// The value of `key`; std::out_of_range (Python's KeyError) if it is missing.
    const V& at(const K& key) const {
        const auto it = index_.find(key);
        if (it == index_.end()) throw std::out_of_range(detail::key_repr(key));
        return items_[it->second].second;
    }
    V& at(const K& key) {
        const auto it = index_.find(key);
        if (it == index_.end()) throw std::out_of_range(detail::key_repr(key));
        return items_[it->second].second;
    }
    /// The value of `key`, or nullopt (Python's dict.get).
    std::optional<V> get(const K& key) const {
        const auto it = index_.find(key);
        if (it == index_.end()) return std::nullopt;
        return items_[it->second].second;
    }
    /// `key in d`.
    bool contains(const K& key) const { return index_.count(key) > 0; }
    /// Remove `key` and return its value (Python's dict.pop); std::out_of_range if missing.
    V pop(const K& key) {
        const auto it = index_.find(key);
        if (it == index_.end()) throw std::out_of_range(detail::key_repr(key));
        const std::size_t position = it->second;
        V value = std::move(items_[position].second);
        items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(position));
        index_.erase(it);
        for (auto& entry : index_) {
            if (entry.second > position) --entry.second;
        }
        return value;
    }

    std::size_t size() const { return items_.size(); }
    bool empty() const { return items_.empty(); }
    const_iterator begin() const { return items_.begin(); }
    const_iterator end() const { return items_.end(); }
    iterator begin() { return items_.begin(); }
    iterator end() { return items_.end(); }
    /// The keys in insertion order (Python's list(d)).
    std::vector<K> keys() const {
        std::vector<K> out;
        out.reserve(items_.size());
        for (const auto& item : items_) out.push_back(item.first);
        return out;
    }
    /// The values in insertion order.
    std::vector<V> values() const {
        std::vector<V> out;
        out.reserve(items_.size());
        for (const auto& item : items_) out.push_back(item.second);
        return out;
    }
    /// Equal keys with equal values (order doesn't matter, as in Python).
    bool operator==(const Dict& other) const {
        if (size() != other.size()) return false;
        for (const auto& [key, value] : items_) {
            const auto it = other.index_.find(key);
            if (it == other.index_.end() || !(other.items_[it->second].second == value)) return false;
        }
        return true;
    }
    bool operator!=(const Dict& other) const { return !(*this == other); }

private:
    std::vector<value_type> items_;
    std::map<K, std::size_t> index_;
};

/// A value of a parameter table read by bd_warehouse: a number (a dimension in mm, or the
/// value the table gives) or, for an entry that isn't a number or a fraction, its text
/// (Python's `float | str`, as produced by metric_str_to_float and imperial_str_to_float).
/// Converts to double implicitly; doing so with text throws std::invalid_argument (Python
/// raises TypeError when it calculates with such a string).
class Param {
public:
    Param() = default;
    Param(double number) : value_(number) {}  // NOLINT: a table number
    Param(int number) : value_(static_cast<double>(number)) {}  // NOLINT
    Param(std::string text) : value_(std::move(text)) {}  // NOLINT: a table text
    Param(const char* text) : value_(std::string(text)) {}  // NOLINT

    /// Whether the value is a number.
    bool is_number() const { return std::holds_alternative<double>(value_); }
    /// The number; std::invalid_argument if the value is text.
    double number() const {
        if (const double* n = std::get_if<double>(&value_)) return *n;
        throw std::invalid_argument("the table value '" + std::get<std::string>(value_) + "' is not a number");
    }
    /// The text; std::invalid_argument if the value is a number.
    const std::string& text() const {
        if (const std::string* s = std::get_if<std::string>(&value_)) return *s;
        throw std::invalid_argument("the table value is a number, not text");
    }
    operator double() const { return number(); }  // NOLINT: calculate with dimensions directly
    bool operator==(const Param& other) const { return value_ == other.value_; }
    bool operator!=(const Param& other) const { return !(*this == other); }

private:
    std::variant<double, std::string> value_ = 0.0;
};

/// A table of string values by row and column (Python's dict of dict of str, e.g. what
/// read_fastener_parameters_from_csv returns).
using StringTable = Dict<std::string, Dict<std::string, std::string>>;
/// A row of evaluated parameters by name (Python's dict of float | str).
using ParamDict = Dict<std::string, Param>;
/// Rows of evaluated parameters by size (Python's dict of dict of float | str).
using ParamTable = Dict<std::string, ParamDict>;

}  // namespace bd_warehouse
