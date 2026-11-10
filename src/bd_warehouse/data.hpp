// bd_warehouse's data files (embedded, see scripts/gen_bdw_data.py) and the bits of Python the
// ports of its readers need: the csv module's reader and DictReader, eval of the numbers and
// fractions bd_warehouse accepts as safe, and str helpers.
#pragma once

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "bd_warehouse/common.hpp"

namespace bd_warehouse::detail {

/// The text of one of bd_warehouse's data files, by the path the Python code opens it with
/// ("data/iso4762.csv", "pipe.csv"). std::runtime_error (Python's FileNotFoundError) if the
/// package has no such file.
const std::string& data_file(const std::string& path);

/// csv.reader over a file's text (the excel dialect: commas, double quotes, doubled quotes in
/// quoted fields, not strict). An empty line gives an empty row, as in Python.
std::vector<std::vector<std::string>> csv_reader(const std::string& text);

/// A row of csv.DictReader: the fields by column name, in the header's order. A row shorter
/// than the header lacks the last fields (Python gives them the value None); a longer one keeps
/// the extra values in `rest` (Python's restkey None).
struct DictRow {
    std::vector<std::pair<std::string, std::optional<std::string>>> fields;
    std::optional<std::vector<std::string>> rest;

    /// row[key]; std::out_of_range (KeyError) for a column the header doesn't have.
    const std::optional<std::string>& at(const std::string& key) const;
    /// row[key] of a field that has a value (std::invalid_argument for None).
    const std::string& str(const std::string& key) const;
    bool contains(const std::string& key) const;
};

/// csv.DictReader: the header's field names (nullopt for an empty file) and the rows after it
/// (empty lines skipped).
struct DictReader {
    std::optional<std::vector<std::string>> fieldnames;
    std::vector<DictRow> rows;
};
DictReader dict_reader(const std::string& text);

/// The result of eval() of a string bd_warehouse checked as safe (digits, '.', '/', ' ', and
/// the '+' imperial_str_to_float puts between a whole number and a fraction): its value and
/// whether Python gives an int. Throws std::invalid_argument for a SyntaxError and
/// std::domain_error for a ZeroDivisionError, as Python's eval raises them.
struct PyNumber {
    double value = 0.0;
    bool is_int = false;
};
PyNumber py_eval_number(const std::string& expression);

/// The methods bd_warehouse's class `cls` (by name, e.g. "HexNut") defines, inherited ones
/// included (scripts/gen_bdw_methods.py); nullptr for a name that isn't one of its classes.
const std::set<std::string>* class_methods(const std::string& cls);

/// Python's float(str) (std::invalid_argument "could not convert string to float: '...'").
double py_float(const std::string& text);
/// Python's int(str) (std::invalid_argument "invalid literal for int() with base 10: '...'").
long long py_int_str(const std::string& text);
/// Python's str.split() (whitespace) and str.split(sep).
std::vector<std::string> py_split(const std::string& text);
std::vector<std::string> py_split(const std::string& text, const std::string& sep);
/// Python's str.strip(), str.replace(old, new).
std::string py_strip(const std::string& text);
std::string py_replace(const std::string& text, const std::string& old, const std::string& replacement);
/// Python's repr(str).
std::string py_repr(const std::string& text);
/// Python's str(float) / repr(float).
std::string py_str(double value);
/// Python's str(int).
std::string py_str(int value);
/// Python's repr of a list of str, e.g. ['M6-1', 'M8-1.25'], and of a list or tuple of
/// numbers, e.g. [100, 103] or (20.741, 20.955).
std::string py_list_repr(const std::vector<std::string>& items);
std::string py_list_repr(const std::vector<int>& items);
std::string py_tuple_repr(const std::vector<std::string>& items);
std::string py_tuple_repr(const std::vector<double>& items);
/// Python's math.radians (x * (pi / 180)) and math.degrees (x * (180 / pi)), which round
/// differently from (x * pi) / 180.
inline double radians(double x) { return x * (3.141592653589793 / 180.0); }
inline double degrees(double x) { return x * (180.0 / 3.141592653589793); }
/// Python's round(x, ndigits) (correctly rounded, ties to even).
double py_round(double x, int ndigits);

}  // namespace bd_warehouse::detail
