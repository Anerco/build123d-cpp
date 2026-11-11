// build123d-cpp: fonts (build123d's text.py): the font manager, the fonts it knows and the
// bundled single-line font. Text itself is in objects.hpp (Text) and topology.hpp
// (Compound::make_text, Compound::resolve_font).
#pragma once

#include <string>
#include <vector>

#include <Font_FontMgr.hxx>
#include <Font_SystemFont.hxx>

#include "build123d/enums.hpp"

namespace build123d {

/// A registered font: its name and the styles (OCCT font aspects) it has (Python's FontInfo).
struct FontInfo {
    std::string name;
    std::vector<FontStyle> styles;

    /// Python's repr: Font(name='Arial', styles=('REGULAR', 'BOLD')).
    std::string repr() const;
    bool operator==(const FontInfo& other) const { return name == other.name && styles == other.styles; }
    bool operator!=(const FontInfo& other) const { return !(*this == other); }
};

/// A font file that comes with build123d: its font name, its file in the bundled font
/// directory and whether it is a single-line (single stroke) font.
struct BundledFont {
    std::string name;
    std::string file;
    bool single_stroke = false;
};

/// build123d's FontManager, a wrapper of OCCT's global Font_FontMgr. The first one created
/// registers the bundled fonts and the alias "singleline" of the bundled single-line font
/// "Relief SingleLine CAD".
class FontManager {
public:
    /// Relief SingleLine CAD (reliefsingleline/ReliefSingleLineCAD-Regular.ttf, single stroke).
    static const std::vector<BundledFont> bundled_fonts;
    /// The directory of the bundled fonts (Python's package data/fonts), found at run time:
    /// $BUILD123D_FONT_DIR, else share/build123d/fonts next to the program or library
    /// (../share/build123d/fonts), the install prefix or the source tree. Empty if none has the fonts.
    static std::string bundled_path();

    FontManager();

    /// OCCT's font manager.
    Handle(Font_FontMgr) manager;

    /// The available fonts with their styles, sorted by name.
    std::vector<FontInfo> available_fonts() const;
    /// The font in the file at `path`, or a null handle if there is no font there.
    Handle(Font_SystemFont) check_font(const std::string& path) const;
    /// The font of that name and style (OCCT substitutes another font if there is none).
    Handle(Font_SystemFont) find_font(const std::string& name, FontStyle style) const;
    /// Register every face of a font file (TrueType, OpenType or a collection) and return the
    /// face names. A file that can't be read as a font gives none (Python logs a warning); a
    /// missing file throws std::runtime_error (Python's FileNotFoundError).
    std::vector<std::string> register_font(const std::string& path, bool override = false,
                                           bool single_stroke = false);
    /// Register the fonts of a folder (files matching *ttf, *otf and *ttc) and return the face
    /// names, each once.
    std::vector<std::string> register_folder(const std::string& path, bool override = false,
                                             bool single_stroke = false);
    /// Register the fonts of the system's font folders (and their subfolders).
    void register_system_fonts();

private:
    // The same, for a caller that holds the font registry's lock (text.cpp).
    std::vector<std::string> register_font_locked(const std::string& path, bool override, bool single_stroke);
    std::vector<std::string> register_folder_locked(const std::string& path, bool override, bool single_stroke);
    void register_system_fonts_locked();
};

/// The available fonts: FontManager().available_fonts().
std::vector<FontInfo> available_fonts();

}  // namespace build123d
