// bd_warehouse's embedded data files, Python's csv reader, and the Python helpers the readers use.
#include "data.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>

#include "../py_compat.hpp"

namespace bd_warehouse {

namespace detail {

std::string key_repr(const std::string& key) { return py_repr(key); }
std::string key_repr(int key) { return std::to_string(key); }
std::string key_repr(long long key) { return std::to_string(key); }
std::string key_repr(double key) { return py_str(key); }

namespace {

struct EmbeddedFile {
    const char* path;
    const char* const* chunks;
    std::size_t chunk_count;
    std::size_t size;
};

#include "data.inc"

}  // namespace

const std::string& data_file(const std::string& path) {
    static std::mutex mutex;
    static std::map<std::string, std::string> files;
    const std::lock_guard<std::mutex> lock(mutex);
    const auto found = files.find(path);
    if (found != files.end()) return found->second;
    for (const auto& file : kEmbeddedFiles) {
        if (path != file.path) continue;
        std::string text;
        text.reserve(file.size);
        for (std::size_t i = 0; i < file.chunk_count; ++i) text += file.chunks[i];
        return files.emplace(path, std::move(text)).first->second;
    }
    throw std::runtime_error("[Errno 2] No such file or directory: '" + path + "'");
}

// ---------------------------------------------------------------------------
// csv (a port of the state machine of CPython's _csv.c for the excel dialect)
// ---------------------------------------------------------------------------

namespace {

enum class CsvState { START_RECORD, START_FIELD, IN_FIELD, IN_QUOTED_FIELD, QUOTE_IN_QUOTED_FIELD, EAT_CRNL };

constexpr int kEol = -2;

struct CsvParser {
    CsvState state = CsvState::START_RECORD;
    std::string field;
    std::vector<std::string> fields;

    void save_field() {
        fields.push_back(field);
        field.clear();
    }

    void process(int c) {
        switch (state) {
            case CsvState::START_RECORD:
                if (c == kEol) break;  // empty line: []
                if (c == '\n' || c == '\r') {
                    state = CsvState::EAT_CRNL;
                    break;
                }
                state = CsvState::START_FIELD;
                [[fallthrough]];
            case CsvState::START_FIELD:
                if (c == '\n' || c == '\r' || c == kEol) {
                    save_field();
                    state = c == kEol ? CsvState::START_RECORD : CsvState::EAT_CRNL;
                } else if (c == '"') {
                    state = CsvState::IN_QUOTED_FIELD;
                } else if (c == ',') {
                    save_field();
                } else {
                    field.push_back(static_cast<char>(c));
                    state = CsvState::IN_FIELD;
                }
                break;
            case CsvState::IN_FIELD:
                if (c == '\n' || c == '\r' || c == kEol) {
                    save_field();
                    state = c == kEol ? CsvState::START_RECORD : CsvState::EAT_CRNL;
                } else if (c == ',') {
                    save_field();
                    state = CsvState::START_FIELD;
                } else {
                    field.push_back(static_cast<char>(c));
                }
                break;
            case CsvState::IN_QUOTED_FIELD:
                if (c == kEol) {
                } else if (c == '"') {
                    state = CsvState::QUOTE_IN_QUOTED_FIELD;
                } else {
                    field.push_back(static_cast<char>(c));
                }
                break;
            case CsvState::QUOTE_IN_QUOTED_FIELD:
                if (c == '"') {
                    field.push_back('"');
                    state = CsvState::IN_QUOTED_FIELD;
                } else if (c == ',') {
                    save_field();
                    state = CsvState::START_FIELD;
                } else if (c == '\n' || c == '\r' || c == kEol) {
                    save_field();
                    state = c == kEol ? CsvState::START_RECORD : CsvState::EAT_CRNL;
                } else {
                    field.push_back(static_cast<char>(c));  // not strict
                    state = CsvState::IN_FIELD;
                }
                break;
            case CsvState::EAT_CRNL:
                if (c == '\n' || c == '\r') {
                } else if (c == kEol) {
                    state = CsvState::START_RECORD;
                } else {
                    throw std::runtime_error(
                        "new-line character seen in unquoted field - do you need to open the file with "
                        "newline=''?");
                }
                break;
        }
    }
};

/// The lines of a text as a file iterator gives them (ending with their '\n', '\r' or "\r\n").
std::vector<std::string> lines_of(const std::string& text) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\n' || text[i] == '\r') {
            if (text[i] == '\r' && i + 1 < text.size() && text[i + 1] == '\n') ++i;
            lines.push_back(text.substr(start, i + 1 - start));
            start = i + 1;
        }
    }
    if (start < text.size()) lines.push_back(text.substr(start));
    return lines;
}

}  // namespace

std::vector<std::vector<std::string>> csv_reader(const std::string& text) {
    std::vector<std::vector<std::string>> rows;
    CsvParser parser;
    const std::vector<std::string> lines = lines_of(text);
    std::size_t next = 0;
    while (true) {
        parser.fields.clear();
        bool got_line = false;
        do {
            if (next == lines.size()) {
                // End of input: a partial field (or an open quote) ends the last row
                if (!parser.field.empty() || parser.state == CsvState::IN_QUOTED_FIELD) {
                    parser.save_field();
                    rows.push_back(parser.fields);
                }
                return rows;
            }
            got_line = true;
            for (const char c : lines[next]) parser.process(static_cast<unsigned char>(c));
            ++next;
            parser.process(kEol);
        } while (parser.state != CsvState::START_RECORD);
        if (got_line) rows.push_back(parser.fields);
    }
}

const std::optional<std::string>& DictRow::at(const std::string& key) const {
    for (const auto& [name, value] : fields) {
        if (name == key) return value;
    }
    throw std::out_of_range(py_repr(key));
}

const std::string& DictRow::str(const std::string& key) const {
    const auto& value = at(key);
    if (!value) throw std::invalid_argument("the column " + py_repr(key) + " has no value in this row");
    return *value;
}

bool DictRow::contains(const std::string& key) const {
    for (const auto& field : fields) {
        if (field.first == key) return true;
    }
    return false;
}

DictReader dict_reader(const std::string& text) {
    DictReader reader;
    const auto rows = csv_reader(text);
    std::size_t i = 0;
    // fieldnames: the first row (DictReader skips no blank line for it)
    if (i < rows.size()) reader.fieldnames = rows[i++];
    if (!reader.fieldnames) return reader;
    const auto& names = *reader.fieldnames;
    for (; i < rows.size(); ++i) {
        const auto& row = rows[i];
        if (row.empty()) continue;  // DictReader skips empty rows
        DictRow out;
        // dict(zip(fieldnames, row)): a repeated name keeps its first place and the last value
        const std::size_t n = std::min(names.size(), row.size());
        const auto put = [&](const std::string& name, std::optional<std::string> value) {
            for (auto& field : out.fields) {
                if (field.first == name) {
                    field.second = std::move(value);
                    return;
                }
            }
            out.fields.emplace_back(name, std::move(value));
        };
        for (std::size_t k = 0; k < n; ++k) put(names[k], row[k]);
        if (names.size() < row.size()) {
            out.rest = std::vector<std::string>(row.begin() + static_cast<std::ptrdiff_t>(names.size()), row.end());
        } else {
            for (std::size_t k = row.size(); k < names.size(); ++k) put(names[k], std::nullopt);
        }
        reader.rows.push_back(std::move(out));
    }
    return reader;
}

// ---------------------------------------------------------------------------
// eval of the expressions bd_warehouse considers safe
// ---------------------------------------------------------------------------

namespace {

[[noreturn]] void syntax_error() { throw std::invalid_argument("invalid syntax"); }

struct EvalParser {
    const std::string& s;
    std::size_t pos = 0;

    void skip_spaces() {
        while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
    }
    bool peek(char c) {
        skip_spaces();
        return pos < s.size() && s[pos] == c;
    }

    PyNumber number() {
        skip_spaces();
        const std::size_t start = pos;
        while (pos < s.size() && std::isdigit(static_cast<unsigned char>(s[pos]))) ++pos;
        bool is_float = false;
        if (pos < s.size() && s[pos] == '.') {
            is_float = true;
            ++pos;
            while (pos < s.size() && std::isdigit(static_cast<unsigned char>(s[pos]))) ++pos;
        }
        const std::string token = s.substr(start, pos - start);
        if (token.empty() || token == ".") syntax_error();
        if (!is_float && token.size() > 1 && token[0] == '0' && token.find_first_not_of('0') != std::string::npos) {
            // "leading zeros in decimal integer literals are not permitted"
            throw std::invalid_argument(
                "leading zeros in decimal integer literals are not permitted; use an 0o prefix for octal integers");
        }
        return {std::strtod(token.c_str(), nullptr), !is_float};
    }

    PyNumber factor() {
        if (peek('+')) {
            ++pos;
            return factor();
        }
        if (peek('-')) {
            ++pos;
            PyNumber value = factor();
            value.value = -value.value;
            return value;
        }
        return number();
    }

    PyNumber term() {
        PyNumber left = factor();
        while (peek('/')) {
            ++pos;
            const bool floor_division = pos < s.size() && s[pos] == '/';
            if (floor_division) ++pos;
            const PyNumber right = factor();
            if (floor_division) {
                if (right.value == 0.0) {
                    throw std::domain_error(left.is_int && right.is_int ? "integer division or modulo by zero"
                                                                        : "float floor division by zero");
                }
                left.value = std::floor(left.value / right.value);
                left.is_int = left.is_int && right.is_int;
            } else {
                if (right.value == 0.0) throw std::domain_error("division by zero");
                left.value = left.value / right.value;
                left.is_int = false;
            }
        }
        return left;
    }

    PyNumber expression() {
        PyNumber left = term();
        while (peek('+') || peek('-')) {
            const char op = s[pos++];
            const PyNumber right = term();
            left.value = op == '+' ? left.value + right.value : left.value - right.value;
            left.is_int = left.is_int && right.is_int;
        }
        return left;
    }
};

}  // namespace

PyNumber py_eval_number(const std::string& expression) {
    EvalParser parser{expression};
    const PyNumber value = parser.expression();
    parser.skip_spaces();
    if (parser.pos != expression.size()) syntax_error();
    return value;
}

// ---------------------------------------------------------------------------
// str helpers
// ---------------------------------------------------------------------------

namespace {
bool is_py_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f'; }
}  // namespace

double py_float(const std::string& text) {
    const std::string stripped = py_strip(text);
    std::string digits;
    bool ok = !stripped.empty() && stripped.find_first_of("xX") == std::string::npos;
    for (std::size_t i = 0; ok && i < stripped.size(); ++i) {
        if (stripped[i] == '_') {
            // single underscores between digits
            ok = i > 0 && i + 1 < stripped.size() && std::isdigit(static_cast<unsigned char>(stripped[i - 1])) &&
                 std::isdigit(static_cast<unsigned char>(stripped[i + 1]));
        } else {
            digits.push_back(stripped[i]);
        }
    }
    if (ok) {
        char* end = nullptr;
        errno = 0;
        const double value = std::strtod(digits.c_str(), &end);
        if (end != nullptr && *end == '\0') return value;
    }
    throw std::invalid_argument("could not convert string to float: " + py_repr(text));
}

long long py_int_str(const std::string& text) { return build123d::detail::py_int(text); }

std::vector<std::string> py_split(const std::string& text) {
    std::vector<std::string> out;
    std::string current;
    bool in_word = false;
    for (const char c : text) {
        if (is_py_space(c)) {
            if (in_word) out.push_back(current);
            current.clear();
            in_word = false;
        } else {
            current.push_back(c);
            in_word = true;
        }
    }
    if (in_word) out.push_back(current);
    return out;
}

std::vector<std::string> py_split(const std::string& text, const std::string& sep) {
    if (sep.empty()) throw std::invalid_argument("empty separator");
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
        const std::size_t found = text.find(sep, start);
        if (found == std::string::npos) {
            out.push_back(text.substr(start));
            return out;
        }
        out.push_back(text.substr(start, found - start));
        start = found + sep.size();
    }
}

std::string py_strip(const std::string& text) {
    std::size_t begin = 0, end = text.size();
    while (begin < end && is_py_space(text[begin])) ++begin;
    while (end > begin && is_py_space(text[end - 1])) --end;
    return text.substr(begin, end - begin);
}

std::string py_replace(const std::string& text, const std::string& old, const std::string& replacement) {
    if (old.empty()) {
        std::string out = replacement;
        for (const char c : text) {
            out.push_back(c);
            out += replacement;
        }
        return out;
    }
    std::string out;
    std::size_t start = 0;
    while (true) {
        const std::size_t found = text.find(old, start);
        if (found == std::string::npos) break;
        out += text.substr(start, found - start) + replacement;
        start = found + old.size();
    }
    return out + text.substr(start);
}

std::string py_repr(const std::string& text) {
    const bool has_single = text.find('\'') != std::string::npos;
    const bool has_double = text.find('"') != std::string::npos;
    const char quote = has_single && !has_double ? '"' : '\'';
    std::string out(1, quote);
    for (const char ch : text) {
        const auto c = static_cast<unsigned char>(ch);
        if (c == '\\') {
            out += "\\\\";
        } else if (c == static_cast<unsigned char>(quote)) {
            out += '\\';
            out += ch;
        } else if (c == '\t') {
            out += "\\t";
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c < 0x20 || c == 0x7F) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\x%02x", c);
            out += buf;
        } else {
            out += ch;
        }
    }
    return out + quote;
}

std::string py_str(double value) { return build123d::detail::py_float_repr(value); }

}  // namespace detail

}  // namespace bd_warehouse
