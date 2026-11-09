// DXF loading as ezdxf does it for import_dxf (see dxf_reader.hpp).
#include "dxf_reader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace build123d::dxf {

namespace {

struct CodePage {
    const char* name;
    std::uint32_t table[128];
};

const CodePage kCodePages[] = {
#include "dxf_codepages.inc"
};

std::string upper(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }

std::string strip(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && is_space(s[b])) ++b;
    while (e > b && is_space(s[e - 1])) --e;
    return s.substr(b, e - b);
}

bool in(int code, int a, int b) { return code >= a && code <= b; }
bool is_point_code(int c) { return in(c, 10, 18) || in(c, 110, 112) || in(c, 210, 213) || in(c, 1010, 1013); }
bool is_double(int c) { return in(c, 10, 59) || in(c, 110, 149) || in(c, 210, 239) || in(c, 460, 469) || in(c, 1010, 1059); }
bool is_byte(int c) { return in(c, 290, 299); }
bool is_int16(int c) {
    return in(c, 60, 79) || in(c, 170, 179) || in(c, 270, 289) || in(c, 370, 389) || in(c, 400, 409) ||
           in(c, 1060, 1070);
}
bool is_int32(int c) { return in(c, 90, 99) || in(c, 420, 429) || in(c, 440, 459) || c == 1071; }
bool is_int64(int c) { return in(c, 160, 169); }
bool is_int(int c) { return is_byte(c) || is_int16(c) || is_int32(c) || is_int64(c); }
bool is_binary(int c) { return in(c, 310, 319) || c == 1004; }

/// Python's int(text) (whitespace and underscores between digits allowed).
std::optional<long long> py_int(const std::string& text) {
    const std::string s = strip(text);
    std::size_t i = 0;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    if (i == s.size()) return std::nullopt;
    std::string digits = s.substr(0, i);
    for (std::size_t k = i; k < s.size(); ++k) {
        if (std::isdigit(static_cast<unsigned char>(s[k]))) {
            digits += s[k];
        } else if (s[k] == '_' && k > i && k + 1 < s.size() && std::isdigit(static_cast<unsigned char>(s[k - 1])) &&
                   std::isdigit(static_cast<unsigned char>(s[k + 1]))) {
            continue;
        } else {
            return std::nullopt;
        }
    }
    return std::strtoll(digits.c_str(), nullptr, 10);
}

/// Python's float(text).
std::optional<double> py_float(const std::string& text) {
    std::string s = strip(text);
    if (s.empty()) return std::nullopt;
    std::string clean;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '_' && i > 0 && i + 1 < s.size() && std::isdigit(static_cast<unsigned char>(s[i - 1])) &&
            std::isdigit(static_cast<unsigned char>(s[i + 1]))) {
            continue;
        }
        clean += s[i];
    }
    std::string l = clean;
    for (auto& c : l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (l.find("0x") != std::string::npos || l.find('_') != std::string::npos || l.find("nan(") != std::string::npos) {
        return std::nullopt;
    }
    char* end = nullptr;
    const double v = std::strtod(clean.c_str(), &end);
    if (end != clean.c_str() + clean.size()) return std::nullopt;
    return v;
}

/// Bytes in a Windows code page (or UTF-8) to UTF-8.
std::string decode(const std::string& bytes, const std::string& encoding) {
    if (encoding == "utf8") return bytes;
    const CodePage* page = nullptr;
    for (const auto& p : kCodePages) {
        if (encoding == std::string("cp") + p.name) page = &p;
    }
    if (page == nullptr) return bytes;  // multi-byte code pages: kept as they are
    std::u32string out;
    for (const unsigned char c : bytes) {
        if (c < 0x80) {
            out += static_cast<char32_t>(c);
        } else {
            const std::uint32_t cp = page->table[c - 0x80];
            out += cp ? static_cast<char32_t>(cp) : U'\uFFFD';
        }
    }
    return utf8_encode(out);
}

/// ezdxf's toencoding($DWGCODEPAGE)
std::string to_encoding(const std::string& dxfcodepage) {
    static const std::pair<const char*, const char*> table[] = {
        {"874", "cp874"},   {"932", "cp932"},   {"936", "gbk"},     {"949", "cp949"},   {"950", "cp950"},
        {"1250", "cp1250"}, {"1251", "cp1251"}, {"1252", "cp1252"}, {"1253", "cp1253"}, {"1254", "cp1254"},
        {"1255", "cp1255"}, {"1256", "cp1256"}, {"1257", "cp1257"}, {"1258", "cp1258"}};
    for (const auto& [cp, enc] : table) {
        const std::size_t n = std::strlen(cp);
        if (dxfcodepage.size() >= n && dxfcodepage.compare(dxfcodepage.size() - n, n, cp) == 0) return enc;
    }
    return "cp1252";
}

/// A raw tag: the group code and the value as text, or typed for binary DXF.
struct RawTag {
    int code = 0;
    std::string value;
    bool typed = false;
    Tag tag;
    RawTag(int c, std::string v) : code(c), value(std::move(v)) {}
};

/// Universal newlines: the lines of the text.
std::vector<std::string> split_lines(const std::string& text) {
    std::vector<std::string> lines;
    std::string current;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (c == '\n' || c == '\r') {
            lines.push_back(current);
            current.clear();
            if (c == '\r' && i + 1 < text.size() && text[i + 1] == '\n') ++i;
        } else {
            current += c;
        }
    }
    if (!current.empty()) lines.push_back(current);
    return lines;
}

/// ascii_tags_loader over decoded lines (comments skipped, nothing after EOF).
std::vector<RawTag> ascii_tags(const std::vector<std::string>& lines) {
    std::vector<RawTag> tags;
    for (std::size_t i = 0; i + 1 < lines.size(); i += 2) {
        const auto code = py_int(lines[i]);
        if (!code) throw StructureError("Invalid group code \"" + lines[i] + "\" at line " + std::to_string(i + 1) + ".");
        const std::string& value = lines[i + 1];
        if (*code != 999) tags.push_back(RawTag{static_cast<int>(*code), value});
        if (*code == 0 && value == "EOF") break;
    }
    return tags;
}

/// The header variables $ACADVER and $DWGCODEPAGE of an ASCII DXF (ezdxf's dxf_info).
std::pair<std::string, std::string> ascii_info(const std::vector<RawTag>& tags) {
    std::string version = "AC1009", codepage = "ANSI_1252";
    if (tags.size() < 2 || tags[0].code != 0 || tags[0].value != "SECTION") return {version, codepage};
    if (tags[1].code != 2 || tags[1].value != "HEADER") return {version, codepage};
    for (std::size_t i = 2; i + 1 < tags.size(); ++i) {
        if (tags[i].code == 0 && strip(tags[i].value) == "ENDSEC") break;
        if (tags[i].code != 9) continue;
        if (tags[i].value == "$ACADVER") version = tags[i + 1].value;
        if (tags[i].value == "$DWGCODEPAGE") codepage = tags[i + 1].value;
    }
    return {version, codepage};
}

std::vector<RawTag> binary_tags(const std::string& data) {
    // scan_params
    std::string dxfversion = "AC1009";
    std::string encoding = "cp1252";
    const std::string head = data.substr(0, std::min<std::size_t>(data.size(), 1024));
    auto find = [&head](const char* key) {
        const auto p = head.find(key, 22);
        return p == std::string::npos ? p : p;
    };
    if (auto p = find("$ACADVER"); p != std::string::npos) {
        std::size_t start = p + 10;
        if (start < data.size() && data[start] != 'A') ++start;
        dxfversion = data.substr(start, 6);
    }
    if (dxfversion >= "AC1021") {
        encoding = "utf8";
    } else if (auto p = find("$DWGCODEPAGE"); p != std::string::npos) {
        std::size_t start = p + 14;
        if (start < data.size() && data[start] != 'A') ++start;
        std::size_t end = start + 5;
        while (end < data.size() && data[end] != 0) ++end;
        encoding = to_encoding(data.substr(start, end - start));
    }
    const bool r12 = dxfversion <= "AC1009";
    std::vector<RawTag> tags;
    std::size_t index = 22;
    auto need = [&data](std::size_t at, std::size_t n) {
        if (at + n > data.size()) throw StructureError("Unexpected end of binary DXF data.");
    };
    auto le = [&data](std::size_t at, int n) {
        std::uint64_t v = 0;
        for (int k = n - 1; k >= 0; --k) v = (v << 8) | static_cast<unsigned char>(data[at + static_cast<std::size_t>(k)]);
        return v;
    };
    while (index < data.size()) {
        int code = static_cast<unsigned char>(data[index]);
        if (r12) {
            if (code == 255) {
                need(index, 3);
                code = static_cast<int>(le(index + 1, 2));
                index += 3;
            } else {
                index += 1;
            }
        } else {
            need(index, 2);
            code = static_cast<int>(le(index, 2));
            index += 2;
        }
        RawTag t{code, ""};
        t.typed = true;
        t.tag.code = code;
        if (is_binary(code)) {
            need(index, 1);
            const std::size_t length = static_cast<unsigned char>(data[index]);
            index += 1;
            need(index, length);
            t.tag.kind = Tag::Kind::BINARY;
            t.tag.text = data.substr(index, length);
            index += length;
        } else if (is_int16(code)) {
            need(index, 2);
            t.tag.kind = Tag::Kind::INT;
            t.tag.i = static_cast<std::int16_t>(le(index, 2));
            index += 2;
        } else if (is_double(code)) {
            need(index, 8);
            const std::uint64_t bits = le(index, 8);
            double v;
            std::memcpy(&v, &bits, sizeof(v));
            t.tag.kind = Tag::Kind::FLOAT;
            t.tag.x = v;
            index += 8;
        } else if (is_int32(code)) {
            need(index, 4);
            t.tag.kind = Tag::Kind::INT;
            t.tag.i = static_cast<std::int32_t>(le(index, 4));
            index += 4;
        } else if (is_int64(code)) {
            need(index, 8);
            t.tag.kind = Tag::Kind::INT;
            t.tag.i = static_cast<std::int64_t>(le(index, 8));
            index += 8;
        } else if (is_byte(code)) {
            need(index, 1);
            t.tag.kind = Tag::Kind::INT;
            t.tag.i = static_cast<unsigned char>(data[index]);
            index += 1;
        } else {
            const auto end = data.find('\0', index);
            if (end == std::string::npos) throw StructureError("Unterminated string in binary DXF data.");
            t.tag.kind = Tag::Kind::STRING;
            t.tag.text = decode(data.substr(index, end - index), encoding);
            t.value = t.tag.text;
            index = end + 1;
        }
        tags.push_back(std::move(t));
    }
    return tags;
}

/// tag_compiler: typed tags, points from x, y[, z] tags.
std::vector<Tag> compile(const std::vector<RawTag>& raw) {
    std::vector<Tag> out;
    auto number = [](const RawTag& t) -> double {
        if (t.typed) {
            if (t.tag.kind == Tag::Kind::FLOAT) return t.tag.x;
            if (t.tag.kind == Tag::Kind::INT) return static_cast<double>(t.tag.i);
        }
        const auto v = py_float(t.value);
        if (!v) throw StructureError("Invalid floating point values.");
        return *v;
    };
    std::size_t i = 0;
    while (i < raw.size()) {
        const RawTag& x = raw[i];
        const int code = x.code;
        if (is_point_code(code)) {
            if (i + 1 >= raw.size()) return out;  // StopIteration: the tagger ends
            if (raw[i + 1].code != code + 10) throw StructureError("Missing required y coordinate.");
            Tag t;
            t.code = code;
            t.kind = Tag::Kind::POINT;
            t.x = number(x);
            t.y = number(raw[i + 1]);
            if (i + 2 >= raw.size()) return out;  // StopIteration without a z tag: the point is lost
            if (raw[i + 2].code == code + 20) {
                t.z = number(raw[i + 2]);
                t.dims = 3;
                i += 3;
            } else {
                t.dims = 2;
                i += 2;
            }
            out.push_back(t);
            continue;
        }
        if (x.typed) {
            out.push_back(x.tag);
            if (code == 0 && x.tag.kind == Tag::Kind::STRING) out.back().text = strip(out.back().text);
            ++i;
            continue;
        }
        Tag t;
        t.code = code;
        if (is_binary(code)) {
            t.kind = Tag::Kind::BINARY;
            std::string bytes;
            for (std::size_t k = 0; k + 1 < x.value.size(); k += 2) {
                bytes += static_cast<char>(std::stoi(x.value.substr(k, 2), nullptr, 16));
            }
            t.text = bytes;
        } else if (is_double(code)) {
            const auto v = py_float(x.value);
            if (!v) throw StructureError("Invalid tag (code=" + std::to_string(code) + ", value=\"" + x.value + "\").");
            t.kind = Tag::Kind::FLOAT;
            t.x = *v;
        } else if (is_int(code)) {
            auto v = py_int(x.value);
            if (!v) {
                const auto f = py_float(x.value);  // ProE stores ints as floats
                if (!f || !std::isfinite(*f)) {
                    throw StructureError("Invalid tag (code=" + std::to_string(code) + ", value=\"" + x.value + "\").");
                }
                v = static_cast<long long>(std::trunc(*f));
            }
            t.kind = Tag::Kind::INT;
            t.i = *v;
        } else {
            t.kind = Tag::Kind::STRING;
            t.text = code == 0 ? strip(x.value) : x.value;
        }
        out.push_back(t);
        ++i;
    }
    return out;
}

bool is(const Tag& t, int code, const char* value) {
    return t.code == code && t.kind == Tag::Kind::STRING && t.text == value;
}

using Group = std::vector<Tag>;

/// load_dxf_structure: the sections by name, each a list of entities (tag groups).
std::map<std::string, std::vector<Group>> load_structure(const std::vector<Tag>& tags) {
    // group_tags(splitcode=0), tags before the first 0-tag ignored
    std::vector<Group> groups;
    for (const Tag& t : tags) {
        if (t.code == 0) {
            groups.push_back(Group{t});
        } else if (!groups.empty()) {
            groups.back().push_back(t);
        }
    }
    std::map<std::string, std::vector<Group>> sections;
    std::vector<Group> section;
    bool eof = false;
    auto inside = [&section] { return !section.empty() && is(section[0][0], 0, "SECTION"); };
    for (auto& entity : groups) {
        const Tag& tag = entity[0];
        if (is(tag, 0, "SECTION")) {
            if (inside()) throw StructureError("DXFStructureError: missing ENDSEC tag.");
            section = {entity};
        } else if (is(tag, 0, "ENDSEC")) {
            if (!inside()) throw StructureError("DXFStructureError: found ENDSEC tag without previous SECTION tag.");
            const Group& header = section[0];
            if (header.size() < 2 || header[1].code != 2) {
                throw StructureError("DXFStructureError: missing required section NAME tag (2, name) at start of section.");
            }
            sections[header[1].text] = section;
            section.clear();
        } else if (is(tag, 0, "EOF")) {
            eof = true;
        } else {
            section.push_back(entity);
        }
    }
    if (inside()) throw StructureError("DXFStructureError: missing ENDSEC tag.");
    if (!eof) throw StructureError("DXFStructureError: missing EOF tag.");
    return sections;
}

Entity make_entity(const Group& g) {
    Entity e;
    e.type = g[0].text;
    e.tags = g;
    return e;
}

/// The checks ezdxf's entity loaders make (load_dxf_attribs): a missing subclass is a structure
/// error, as is a HATCH without its boundary path count.
void validate(const Entity& e) {
    static const std::pair<const char*, const char*> required[] = {
        {"LWPOLYLINE", "AcDbPolyline"}, {"SPLINE", "AcDbSpline"}, {"MTEXT", "AcDbMText"}};
    for (const auto& [type, subclass] : required) {
        if (e.type == type && !e.subclass(2)) {
            throw StructureError(std::string("missing '") + subclass + "' subclass in " + type);
        }
    }
    if (e.type == "HATCH") {
        const auto data = e.subclass(2);
        if (!data) throw std::out_of_range("list index out of range (HATCH without its AcDbHatch subclass)");
        const bool has_count = std::any_of(data->begin(), data->end(), [](const Tag& t) { return t.code == 91; });
        if (!has_count) {
            throw StructureError(
                "HATCH: Missing required DXF tag 'Number of boundary paths (loops)' (code=91).");
        }
    }
}

/// entity_linker over a section's entities: VERTEX/ATTRIB and SEQEND are linked to the
/// preceding POLYLINE/INSERT (with attribs_follow) and don't appear on their own.
std::vector<Entity> link_entities(const std::vector<Group>& groups, std::size_t first, std::size_t last) {
    std::vector<Entity> out;
    bool open = false;
    std::string expected;
    for (std::size_t k = first; k < last; ++k) {
        Entity e = make_entity(groups[k]);
        if (open) {
            if (e.type == "SEQEND") {
                open = false;
            } else if (e.type == expected) {
                out.back().linked.push_back(std::move(e));
            } else {
                throw StructureError("Expected DXF entity " + e.type + " or SEQEND");
            }
            continue;
        }
        validate(e);
        const bool polyline = e.type == "POLYLINE";
        bool insert_with_attribs = false;
        if (e.type == "INSERT") {
            const Tag* attribs = e.get(66);
            insert_with_attribs = attribs != nullptr && attribs->i != 0;
        }
        out.push_back(std::move(e));
        if (polyline || insert_with_attribs) {
            open = true;
            expected = polyline ? "VERTEX" : "ATTRIB";
        }
    }
    return out;
}

Document load(const std::vector<Tag>& tags) {
    const auto sections = load_structure(tags);
    Document doc;
    // HEADER
    if (const auto it = sections.find("HEADER"); it != sections.end() && !it->second.empty()) {
        const Group& header = it->second[0];
        for (std::size_t k = 0; k + 1 < header.size(); ++k) {
            if (header[k].code == 9 && header[k].text == "$ACADVER") doc.dxfversion = header[k + 1].text;
        }
    }
    // BLOCK_RECORD handles of the model and paper space
    std::string msp_handle, psp_handle;
    if (const auto it = sections.find("TABLES"); it != sections.end()) {
        for (const Group& g : it->second) {
            if (g[0].text != "BLOCK_RECORD") continue;
            const Entity e = make_entity(g);
            const Tag* name = e.get(2);
            const Tag* handle = e.get(5);
            if (!name || !handle) continue;
            const std::string n = upper(name->text);
            if (n == "*MODEL_SPACE") msp_handle = handle->text;
            if (n == "*PAPER_SPACE") psp_handle = handle->text;
        }
    }
    // BLOCKS (BlocksSection.load): a BLOCK starts a block definition, ENDBLK stores it
    std::vector<Entity> msp_block_entities;
    if (const auto it = sections.find("BLOCKS"); it != sections.end()) {
        const auto& groups = it->second;
        std::optional<Entity> block;
        std::vector<Entity> content;
        for (Entity& e : link_entities(groups, 1, groups.size())) {
            if (e.type == "BLOCK") {
                block = std::move(e);  // a previous block without ENDBLK is dropped
                content.clear();
            } else if (e.type == "ENDBLK") {
                if (block) {
                    Block b;
                    b.block = std::move(*block);
                    const Tag* name = b.block.get(2);
                    b.name = name ? name->text : std::string();
                    b.entities = std::move(content);
                    const std::string key = upper(b.name);
                    if (key == "*MODEL_SPACE" || key == "$MODEL_SPACE") {
                        msp_block_entities.insert(msp_block_entities.end(), b.entities.begin(), b.entities.end());
                    }
                    doc.blocks[key] = std::move(b);
                }
                block.reset();
                content.clear();
            } else {
                content.push_back(std::move(e));
            }
        }
    }
    doc.modelspace = msp_block_entities;
    // ENTITIES
    if (const auto it = sections.find("ENTITIES"); it != sections.end()) {
        const auto& groups = it->second;
        for (Entity& e : link_entities(groups, 1, groups.size())) {
            int paperspace = 0;
            const Tag* owner = nullptr;
            // the owner handle: the first 330 outside application data groups
            bool app_data = false;
            for (const Tag& t : e.tags) {
                if (t.code == 102) {
                    app_data = !t.text.empty() && t.text[0] == '{';
                    continue;
                }
                if (app_data) continue;
                if (t.code == 100) break;
                if (t.code == 330) {
                    owner = &t;
                    break;
                }
            }
            if (owner && !msp_handle.empty() && owner->text == msp_handle) {
                paperspace = 0;
            } else if (owner && !psp_handle.empty() && owner->text == psp_handle) {
                paperspace = 1;
            } else if (const Tag* flag = e.get(67)) {
                paperspace = static_cast<int>(flag->i);
            }
            if (!paperspace) doc.modelspace.push_back(std::move(e));
        }
    }
    return doc;
}

}  // namespace

const Tag* Entity::get(int code) const {
    bool app_data = false;
    for (std::size_t k = 1; k < tags.size(); ++k) {
        const Tag& t = tags[k];
        if (t.code == 102) {
            app_data = !t.text.empty() && t.text[0] == '{';
            continue;
        }
        if (app_data) continue;
        if (t.code == 101 || t.code >= 1000) break;  // embedded objects, extended data
        if (t.code == code) return &t;
    }
    return nullptr;
}

std::optional<std::vector<Tag>> Entity::subclass(std::size_t index) const {
    std::vector<std::vector<Tag>> parts(1);
    for (std::size_t k = 1; k < tags.size(); ++k) {
        const Tag& t = tags[k];
        if (t.code == 101 || t.code >= 1000) break;
        if (t.code == 100) {
            parts.emplace_back();
            continue;
        }
        parts.back().push_back(t);
    }
    if (index >= parts.size()) return std::nullopt;
    return parts[index];
}

const Block* Document::block(const std::string& name) const {
    const auto it = blocks.find(upper(name));
    return it == blocks.end() ? nullptr : &it->second;
}

Document read_data(const std::string& data) {
    if (data.compare(0, 22, std::string("AutoCAD Binary DXF\r\n\x1a\x00", 22)) == 0) {
        return load(compile(binary_tags(data)));
    }
    // is_dxf_file: a (0, SECTION) tag before any tag with a code above 999
    {
        const auto lines = split_lines(data);
        bool dxf = false;
        try {
            for (const RawTag& t : ascii_tags(lines)) {
                if (t.code == 0 && t.value == "SECTION") {
                    dxf = true;
                    break;
                }
                if (t.code > 999) break;
            }
        } catch (const StructureError&) {
        }
        if (!dxf) throw std::runtime_error("The data is not a DXF document.");
        // dxf_file_info: the encoding
        const auto [version, codepage] = ascii_info(ascii_tags(lines));
        const std::string encoding = version >= "AC1021" ? "utf8" : to_encoding(codepage);
        return load(compile(ascii_tags(split_lines(decode(data, encoding)))));
    }
}

Document read_text(const std::string& text) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    while (start < text.size()) {
        const auto p = text.find('\n', start);
        if (p == std::string::npos) {
            lines.push_back(text.substr(start));
            break;
        }
        lines.push_back(text.substr(start, p - start));
        start = p + 1;
    }
    return load(compile(ascii_tags(lines)));
}

std::string latin1_to_utf8(const std::string& bytes) {
    std::u32string out;
    for (const unsigned char c : bytes) out += static_cast<char32_t>(c);
    return utf8_encode(out);
}

Document read_file(const std::string& filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) throw std::runtime_error("No such file or directory: '" + filename + "'");
    std::ostringstream content;
    content << in.rdbuf();
    try {
        return read_data(content.str());
    } catch (const StructureError&) {
        throw;
    } catch (const std::runtime_error&) {
        throw std::runtime_error("File '" + filename + "' is not a DXF file.");
    }
}

std::string fast_plain_mtext(const std::string& content) {
    std::u32string text = utf8_decode(content);
    // caret_decode: ^X is a control character
    {
        std::u32string decoded;
        for (std::size_t i = 0; i < text.size(); ++i) {
            if (text[i] == U'^' && i + 1 < text.size() && text[i + 1] != U'\n') {
                const long long c = static_cast<long long>(text[i + 1]);
                decoded += static_cast<char32_t>(((c - 64) % 126 + 126) % 126);
                ++i;
            } else {
                decoded += text[i];
            }
        }
        text = decoded;
    }
    std::u32string chars;
    std::u32string raw(text.rbegin(), text.rend());  // reversed for pop_back()
    const std::u32string one_char_commands = U"PNLlOoKkX";
    while (!raw.empty()) {
        char32_t c = raw.back();
        raw.pop_back();
        if (c == U'\\') {
            if (raw.empty()) break;  // premature end of text
            c = raw.back();
            raw.pop_back();
            if (c == U'\\' || c == U'{' || c == U'}') {
                chars += c;
            } else if (one_char_commands.find(c) != std::u32string::npos) {
                if (c == U'P') chars += U'\n';
                if (c == U'N') chars += U' ';
            } else if (c == U';') {
                // an empty command: nothing to remove
            } else {
                // commands with arguments end with ';'
                const bool stacking = c == U'S';
                const char32_t first_char = c;
                std::u32string search = raw;
                bool ended = false;
                std::u32string stacked;
                while (!search.empty()) {
                    c = search.back();
                    search.pop_back();
                    if (c == U';') {
                        ended = true;
                        break;
                    }
                    if (stacking) stacked += c;
                }
                if (ended) {
                    chars += stacked;
                    raw = search;
                } else {
                    chars += stacked;  // appended while searching
                    chars += U'\\';
                    chars += first_char;
                }
            }
        } else if (c == U'{' || c == U'}') {
            // grouping
        } else if (c == U'%') {
            if (!raw.empty() && raw.back() == U'%') {
                raw.pop_back();
                if (!raw.empty()) {
                    const char32_t code = raw.back();
                    raw.pop_back();
                    const char32_t lower = code < 128 ? static_cast<char32_t>(std::tolower(static_cast<int>(code))) : code;
                    if (lower == U'c') {
                        chars += U'\u00D8';
                    } else if (lower == U'd') {
                        chars += U'\u00B0';
                    } else if (lower == U'p') {
                        chars += U'\u00B1';
                    } else if (lower == U'%') {
                        chars += U'%';
                    } else {
                        chars += U"%%";
                        chars += code;
                    }
                }
            } else {
                chars += c;
            }
        } else {
            chars += c;
        }
    }
    return utf8_encode(chars);
}

std::u32string utf8_decode(const std::string& s) {
    std::u32string out;
    const auto* p = reinterpret_cast<const unsigned char*>(s.data());
    const auto* end = p + s.size();
    while (p < end) {
        char32_t cp = *p;
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
        for (int i = 0; i < extra && p < end; ++i, ++p) cp = (cp << 6) | (*p & 0x3F);
        out += cp;
    }
    return out;
}

std::string utf8_encode(const std::u32string& s) {
    std::string out;
    for (const char32_t cp : s) {
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
    return out;
}

}  // namespace build123d::dxf
