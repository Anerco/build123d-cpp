// Minimal namespace-aware XML reader (see xml_reader.hpp).
#include "xml_reader.hpp"

#include <cctype>
#include <cstdint>
#include <map>
#include <stdexcept>

namespace build123d::xml {

namespace {

void append_utf8(std::string& out, std::uint32_t cp) {
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

class Reader {
public:
    explicit Reader(const std::string& text) : s_(text) {
        // expat normalizes line endings: \r\n and \r become \n
        std::string t;
        t.reserve(text.size());
        for (std::size_t i = 0; i < text.size(); ++i) {
            if (text[i] == '\r') {
                t += '\n';
                if (i + 1 < text.size() && text[i + 1] == '\n') ++i;
            } else {
                t += text[i];
            }
        }
        s_ = t;
        // a UTF-8 byte order mark
        if (s_.compare(0, 3, "\xEF\xBB\xBF") == 0) i_ = 3;
    }

    std::unique_ptr<Node> parse_document() {
        skip_misc();
        if (i_ >= s_.size() || s_[i_] != '<') fail("no element found");
        auto root = parse_element({{"xml", "http://www.w3.org/XML/1998/namespace"}});
        skip_misc();
        if (i_ < s_.size()) fail("junk after document element");
        return root;
    }

private:
    std::string s_;
    std::size_t i_ = 0;
    std::map<std::string, std::string> entities_;

    [[noreturn]] void fail(const std::string& what) const {
        throw std::invalid_argument("XML syntax error: " + what + " (at offset " + std::to_string(i_) + ")");
    }
    bool starts(const char* p) const { return s_.compare(i_, std::char_traits<char>::length(p), p) == 0; }
    static bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
    void skip_space() {
        while (i_ < s_.size() && is_space(s_[i_])) ++i_;
    }
    void skip_until(const char* end) {
        const auto p = s_.find(end, i_);
        if (p == std::string::npos) fail(std::string("unterminated construct, expected ") + end);
        i_ = p + std::char_traits<char>::length(end);
    }

    /// Prolog, comments, processing instructions and the DOCTYPE (internal entities are read).
    void skip_misc() {
        while (true) {
            skip_space();
            if (starts("<?")) {
                skip_until("?>");
            } else if (starts("<!--")) {
                skip_until("-->");
            } else if (starts("<!DOCTYPE")) {
                parse_doctype();
            } else {
                return;
            }
        }
    }

    void parse_doctype() {
        i_ += 9;
        while (i_ < s_.size() && s_[i_] != '[' && s_[i_] != '>') ++i_;
        if (i_ < s_.size() && s_[i_] == '[') {
            ++i_;
            while (i_ < s_.size() && s_[i_] != ']') {
                if (starts("<!ENTITY")) {
                    i_ += 8;
                    skip_space();
                    bool parameter = false;
                    if (i_ < s_.size() && s_[i_] == '%') {
                        parameter = true;
                        ++i_;
                        skip_space();
                    }
                    std::string name = read_name();
                    skip_space();
                    std::string value;
                    if (i_ < s_.size() && (s_[i_] == '"' || s_[i_] == '\'')) {
                        const char q = s_[i_++];
                        const auto e = s_.find(q, i_);
                        if (e == std::string::npos) fail("unterminated entity value");
                        value = s_.substr(i_, e - i_);
                        i_ = e + 1;
                    }
                    skip_until(">");
                    if (!parameter) entities_[name] = expand(value, false);
                } else if (starts("<!--")) {
                    skip_until("-->");
                } else if (s_[i_] == '<') {
                    skip_until(">");
                } else {
                    ++i_;
                }
            }
            if (i_ < s_.size()) ++i_;
        }
        skip_until(">");
    }

    std::string read_name() {
        const std::size_t start = i_;
        while (i_ < s_.size()) {
            const unsigned char c = static_cast<unsigned char>(s_[i_]);
            if (std::isalnum(c) || c == '_' || c == ':' || c == '-' || c == '.' || c >= 0x80) {
                ++i_;
            } else {
                break;
            }
        }
        if (i_ == start) fail("not well-formed (invalid token)");
        return s_.substr(start, i_ - start);
    }

    /// Expand entity and character references in `raw`.
    std::string expand(const std::string& raw, bool attribute) {
        std::string out;
        for (std::size_t k = 0; k < raw.size(); ++k) {
            const char c = raw[k];
            if (c == '&') {
                const auto semi = raw.find(';', k);
                if (semi == std::string::npos) fail("not well-formed (invalid token)");
                const std::string ref = raw.substr(k + 1, semi - k - 1);
                if (!ref.empty() && ref[0] == '#') {
                    std::uint32_t cp = 0;
                    if (ref.size() > 1 && (ref[1] == 'x' || ref[1] == 'X')) {
                        cp = static_cast<std::uint32_t>(std::stoul(ref.substr(2), nullptr, 16));
                    } else {
                        cp = static_cast<std::uint32_t>(std::stoul(ref.substr(1), nullptr, 10));
                    }
                    append_utf8(out, cp);
                } else if (ref == "lt") {
                    out += '<';
                } else if (ref == "gt") {
                    out += '>';
                } else if (ref == "amp") {
                    out += '&';
                } else if (ref == "quot") {
                    out += '"';
                } else if (ref == "apos") {
                    out += '\'';
                } else {
                    const auto it = entities_.find(ref);
                    if (it == entities_.end()) fail("undefined entity &" + ref + ";");
                    out += attribute ? normalize(it->second) : it->second;
                }
                k = semi;
            } else if (attribute && (c == '\n' || c == '\t' || c == '\r')) {
                out += ' ';
            } else {
                out += c;
            }
        }
        return out;
    }

    static std::string normalize(const std::string& v) {
        std::string out = v;
        for (auto& c : out) {
            if (c == '\n' || c == '\t' || c == '\r') c = ' ';
        }
        return out;
    }

    static std::pair<std::string, std::string> split_qname(const std::string& q) {
        const auto colon = q.find(':');
        if (colon == std::string::npos) return {"", q};
        return {q.substr(0, colon), q.substr(colon + 1)};
    }

    std::unique_ptr<Node> parse_element(std::map<std::string, std::string> ns) {
        ++i_;  // '<'
        auto node = std::make_unique<Node>();
        node->qname = read_name();
        std::vector<std::pair<std::string, std::string>> raw;
        while (true) {
            skip_space();
            if (i_ >= s_.size()) fail("unclosed token");
            if (s_[i_] == '/' || s_[i_] == '>') break;
            std::string name = read_name();
            skip_space();
            if (i_ >= s_.size() || s_[i_] != '=') fail("not well-formed (invalid token)");
            ++i_;
            skip_space();
            if (i_ >= s_.size() || (s_[i_] != '"' && s_[i_] != '\'')) fail("not well-formed (invalid token)");
            const char q = s_[i_++];
            const auto e = s_.find(q, i_);
            if (e == std::string::npos) fail("unclosed token");
            std::string value = expand(s_.substr(i_, e - i_), true);
            i_ = e + 1;
            for (const auto& kv : raw) {
                if (kv.first == name) fail("duplicate attribute");
            }
            raw.emplace_back(std::move(name), std::move(value));
        }
        node->raw_attrib = raw;
        for (const auto& [name, value] : raw) {
            if (name == "xmlns") {
                node->namespaces.emplace_back("", value);
                ns[""] = value;
            } else if (name.compare(0, 6, "xmlns:") == 0) {
                node->namespaces.emplace_back(name.substr(6), value);
                ns[name.substr(6)] = value;
            }
        }
        for (const auto& [name, value] : raw) {
            if (name == "xmlns" || name.compare(0, 6, "xmlns:") == 0) continue;
            const auto [prefix, local] = split_qname(name);
            if (prefix.empty()) {
                node->attrib.emplace_back(name, value);
            } else {
                const auto it = ns.find(prefix);
                if (it == ns.end()) fail("unbound prefix");
                node->attrib.emplace_back("{" + it->second + "}" + local, value);
            }
        }
        {
            const auto [prefix, local] = split_qname(node->qname);
            const auto it = ns.find(prefix);
            if (!prefix.empty() && it == ns.end()) fail("unbound prefix");
            node->tag = (it != ns.end() && !it->second.empty()) ? "{" + it->second + "}" + local : local;
        }
        if (s_[i_] == '/') {
            i_ += 2;  // "/>"
            return node;
        }
        ++i_;  // '>'
        bool before_first_child = true;
        while (true) {
            if (i_ >= s_.size()) fail("no element found");
            if (starts("</")) {
                i_ += 2;
                const std::string end = read_name();
                if (end != node->qname) fail("mismatched tag");
                skip_space();
                if (i_ >= s_.size() || s_[i_] != '>') fail("not well-formed (invalid token)");
                ++i_;
                return node;
            }
            if (starts("<!--")) {
                skip_until("-->");
            } else if (starts("<![CDATA[")) {
                i_ += 9;
                const auto e = s_.find("]]>", i_);
                if (e == std::string::npos) fail("unclosed CDATA section");
                if (before_first_child) node->text += s_.substr(i_, e - i_);
                i_ = e + 3;
            } else if (starts("<?")) {
                skip_until("?>");
            } else if (s_[i_] == '<') {
                node->children.push_back(parse_element(ns));
                before_first_child = false;
            } else {
                const auto e = s_.find('<', i_);
                const std::string raw_text = s_.substr(i_, (e == std::string::npos ? s_.size() : e) - i_);
                if (before_first_child) node->text += expand(raw_text, false);
                i_ = e == std::string::npos ? s_.size() : e;
            }
        }
    }
};

}  // namespace

std::unique_ptr<Node> parse(const std::string& text) { return Reader(text).parse_document(); }

}  // namespace build123d::xml
