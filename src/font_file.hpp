// The part of fontTools that build123d's FontManager uses to name the faces of a font file:
// TTFont/TTCollection reading (ttLib/sfnt.py, ttCollection.py), the 'name' table with
// NameRecord.toUnicode (tables/_n_a_m_e.py, misc/encodingTools.py) and the named instances of
// the 'fvar' table (tables/_f_v_a_r.py).
#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace build123d::detail {

/// fontTools' TTLibError: the file can't be read as a font.
class TTLibError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// fontTools' TTLibFileIsCollectionError: TTFont() of a font collection.
class TTLibFileIsCollectionError : public TTLibError {
public:
    using TTLibError::TTLibError;
};

/// A record of the 'name' table; `string` holds the raw bytes.
struct NameRecord {
    std::uint16_t platformID = 0;
    std::uint16_t platEncID = 0;
    std::uint16_t langID = 0;
    std::uint16_t nameID = 0;
    std::string string;
};

/// One font of a font file, as far as build123d reads it.
struct SfntFont {
    /// The 'name' records in file order (fontTools skips the malformed ones).
    std::vector<NameRecord> names;
    /// Whether the font has an 'fvar' table, and the subfamilyNameID of each named instance.
    bool has_fvar = false;
    std::vector<std::uint16_t> fvar_subfamily_ids;
};

/// NameRecord.toUnicode(): the decoded name as code points; nullopt where Python raises
/// UnicodeDecodeError.
std::optional<std::u32string> name_to_unicode(const NameRecord& record);

/// TTFont(path) followed by font["name"] and the 'fvar' instances. Throws TTLibError (and
/// TTLibFileIsCollectionError for a collection), std::runtime_error for a missing file or
/// where Python raises other errors.
SfntFont read_font(const std::string& path);
/// TTCollection(path): every font of a collection.
std::vector<SfntFont> read_font_collection(const std::string& path);

/// Python's str.split() (Unicode whitespace) and str.strip().
std::vector<std::u32string> py_split(const std::u32string& text);
std::u32string py_strip(const std::u32string& text);
std::string to_utf8(const std::u32string& text);

}  // namespace build123d::detail
