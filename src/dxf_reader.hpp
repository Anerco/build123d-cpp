// The parts of ezdxf's DXF loading that import_dxf relies on: ASCII and binary tag loading with
// the document's text encoding, typed tags with points (tag_compiler), the section structure,
// linked entities (POLYLINE/VERTEX, INSERT/ATTRIB), the blocks and the model space entities.
#pragma once

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace build123d::dxf {

/// A DXF tag with its typed value.
struct Tag {
    enum class Kind { STRING, FLOAT, INT, POINT, BINARY };
    int code = 0;
    Kind kind = Kind::STRING;
    std::string text;  // STRING (UTF-8) and BINARY (raw bytes)
    double x = 0, y = 0, z = 0;  // FLOAT (x), POINT
    int dims = 0;      // POINT: 2 or 3
    long long i = 0;   // INT
};

/// One DXF entity: its tags (the first is (0, type)) and its linked entities.
struct Entity {
    std::string type;
    std::vector<Tag> tags;
    std::vector<Entity> linked;  // VERTEX of a POLYLINE, ATTRIB of an INSERT

    /// The first tag with this group code in the entity's data (not in application data
    /// groups, embedded objects or extended data); nullptr if there is none.
    const Tag* get(int code) const;
    /// The tags of the subclass with this index (0 is the part before the first subclass
    /// marker), without the marker; nullopt if the entity has no such subclass.
    std::optional<std::vector<Tag>> subclass(std::size_t index) const;
};

struct Block {
    std::string name;
    Entity block;  // the BLOCK entity
    std::vector<Entity> entities;
};

struct Document {
    std::string dxfversion = "AC1009";
    std::vector<Entity> modelspace;
    /// Blocks by upper-case name.
    std::map<std::string, Block> blocks;
    const Block* block(const std::string& name) const;
};

/// ezdxf's DXFStructureError (build123d turns it into a ValueError).
struct StructureError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

/// ezdxf.readfile: an ASCII or binary DXF file. Raises std::runtime_error if the file can't be
/// read or isn't a DXF file, StructureError for invalid DXF structures.
Document read_file(const std::string& filename);
/// The same for the content of a file.
Document read_data(const std::string& data);
/// ezdxf.read(StringIO(text)) for UTF-8 text: ASCII DXF only, lines split at \n only (a \r
/// stays in the values, as with Python's StringIO), no encoding detection.
Document read_text(const std::string& text);
/// bytes.decode("latin1") as UTF-8.
std::string latin1_to_utf8(const std::string& bytes);

/// ezdxf.tools.text.fast_plain_mtext: MTEXT content without inline formatting codes.
std::string fast_plain_mtext(const std::string& content);

/// Unicode code points of UTF-8 text, and back.
std::u32string utf8_decode(const std::string& s);
std::string utf8_encode(const std::u32string& s);

}  // namespace build123d::dxf
