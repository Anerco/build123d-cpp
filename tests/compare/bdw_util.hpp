// Helpers shared by the bd_warehouse comparison cases (cases_bdw_*.cpp), the twins of
// bdw_util.py.
#pragma once

#include <bd_warehouse/bd_warehouse.hpp>

#include "case_registry.hpp"

namespace bdw_cases {

using compare::V;

/// The error a call throws as [kind, message] (null if it throws nothing).
template <class F>
V error_of(F build) {
    try {
        build();
    } catch (const std::domain_error& e) {
        return V::list({"domain_error", e.what()});
    } catch (const std::out_of_range& e) {
        return V::list({"out_of_range", e.what()});
    } catch (const std::invalid_argument& e) {
        return V::list({"invalid_argument", e.what()});
    } catch (const std::runtime_error& e) {
        return V::list({"runtime_error", e.what()});
    }
    return V();
}

inline V loc_values(const build123d::Location& loc) { return V::list({loc.position(), loc.orientation()}); }

inline V num(const bd_warehouse::Param& p) { return p.is_number() ? V(p.number()) : V(p.text()); }

/// A dict of dict of values, keys as strings, in order.
template <class Table>
V table_values(const Table& table) {
    std::vector<V> rows;
    for (const auto& [key, row] : table) {
        std::vector<V> fields;
        for (const auto& [name, value] : row) fields.push_back(V::list({name, num(value)}));
        rows.push_back(V::list({key, V::list(fields)}));
    }
    return V::list(rows);
}

/// The labels and volumes of the children of an assembly, in order.
inline V children_values(const build123d::Shape& shape) {
    std::vector<V> out;
    for (const auto& c : shape.children) out.push_back(V::list({c.label, c.volume()}));
    return V::list(out);
}

/// [name, value] pairs, like bdw_util.attrs.
inline V attrs(std::vector<std::pair<std::string, V>> fields) {
    std::vector<V> out;
    for (auto& [name, value] : fields) out.push_back(V::list({name, value}));
    return V::list(out);
}

inline V pair_values(const std::pair<double, double>& p) { return V::list({p.first, p.second}); }

inline V pair_values(const std::pair<std::string, std::string>& p) { return V::list({p.first, p.second}); }

template <class T>
V opt(const std::optional<T>& value) {
    return value ? V(*value) : V();
}

inline V opt(const std::optional<std::pair<double, double>>& value) { return value ? pair_values(*value) : V(); }

}  // namespace bdw_cases
