// A small XML reader with the behaviour the SVG importers rely on from Python's
// xml.etree.ElementTree / xml.dom.minidom (expat): namespace-qualified names as
// "{uri}local", namespace declarations reported separately, attribute order kept,
// attribute values normalized, entities and character references expanded.
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace build123d::xml {

struct Node {
    /// "{uri}local" for a namespaced element, else "local".
    std::string tag;
    /// The name as written ("svg:path"), for minidom's getElementsByTagName.
    std::string qname;
    /// Attributes in document order; namespaced ones as "{uri}local". No xmlns attributes.
    std::vector<std::pair<std::string, std::string>> attrib;
    /// Attributes with their names as written (minidom), including xmlns declarations.
    std::vector<std::pair<std::string, std::string>> raw_attrib;
    /// Namespace declarations of this element: (prefix, uri), "" for the default namespace.
    std::vector<std::pair<std::string, std::string>> namespaces;
    std::vector<std::unique_ptr<Node>> children;
    /// Text before the first child (ElementTree's text).
    std::string text;

    const std::string* get(const std::string& name) const {
        for (const auto& kv : attrib) {
            if (kv.first == name) return &kv.second;
        }
        return nullptr;
    }
};

/// Parse a document; raises std::invalid_argument (Python's ParseError) when it isn't
/// well-formed.
std::unique_ptr<Node> parse(const std::string& text);

}  // namespace build123d::xml
