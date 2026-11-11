// FontManager, FontInfo and available_fonts (build123d's text.py).
#include "build123d/text.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <shared_mutex>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <TCollection_AsciiString.hxx>
#include <TCollection_HAsciiString.hxx>

#include "build123d/warnings.hpp"
#include "font_file.hpp"
#include "internal.hpp"

namespace build123d {

namespace fs = std::filesystem;

namespace {

Font_FontAspect font_aspect(FontStyle style) {  // FONT_ASPECT
    switch (style) {
        case FontStyle::REGULAR: return Font_FA_Regular;
        case FontStyle::BOLD: return Font_FA_Bold;
        case FontStyle::ITALIC: return Font_FA_Italic;
        case FontStyle::BOLDITALIC: return Font_FA_BoldItalic;
    }
    return Font_FA_Regular;
}

const char* style_name(FontStyle style) {
    switch (style) {
        case FontStyle::REGULAR: return "REGULAR";
        case FontStyle::BOLD: return "BOLD";
        case FontStyle::ITALIC: return "ITALIC";
        case FontStyle::BOLDITALIC: return "BOLDITALIC";
    }
    return "REGULAR";
}

/// Python's repr(str) (UTF-8 in and out).
std::string py_str_repr(const std::string& s) {
    const bool has_single = s.find('\'') != std::string::npos;
    const bool has_double = s.find('"') != std::string::npos;
    const char quote = has_single && !has_double ? '"' : '\'';
    std::string out(1, quote);
    for (std::size_t i = 0; i < s.size(); ++i) {
        const auto c = static_cast<unsigned char>(s[i]);
        char buf[16];
        if (c == '\\') {
            out += "\\\\";
        } else if (c == static_cast<unsigned char>(quote)) {
            out += '\\';
            out += static_cast<char>(c);
        } else if (c == '\t') {
            out += "\\t";
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c < 0x20 || c == 0x7F) {
            std::snprintf(buf, sizeof(buf), "\\x%02x", c);
            out += buf;
        } else if (c < 0x80) {
            out += static_cast<char>(c);
        } else {
            // A UTF-8 sequence: C1 controls and the soft hyphen are escaped, as Python does
            std::size_t len = c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : 2;
            len = std::min(len, s.size() - i);
            char32_t cp = c & (len == 2 ? 0x1F : len == 3 ? 0x0F : 0x07);
            for (std::size_t k = 1; k < len; ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3F);
            if (cp <= 0xA0 || cp == 0xAD) {
                std::snprintf(buf, sizeof(buf), "\\x%02x", static_cast<unsigned>(cp));
                out += buf;
            } else if ((cp >= 0x200B && cp <= 0x200F) || (cp >= 0x2028 && cp <= 0x202E) ||
                       (cp >= 0x2060 && cp <= 0x2064) || cp == 0xFEFF) {
                std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(cp));
                out += buf;
            } else {
                out += s.substr(i, len);
            }
            i += len - 1;
        }
    }
    out += quote;
    return out;
}

/// os.path.normpath.
std::string normpath(const std::string& path) {
    if (path.empty()) return ".";
    return fs::path(path).lexically_normal().string();
}

/// os.path.join(a, b) of a directory and a name.
std::string path_join(const std::string& a, const std::string& b) {
    if (a.empty() || a.back() == '/') return a + b;
    return a + "/" + b;
}

/// The extension os.path.splitext finds, without its dot and lower case.
std::string extension(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    const std::string base = slash == std::string::npos ? path : path.substr(slash + 1);
    const std::size_t dot = base.find_last_of('.');
    if (dot == std::string::npos) return {};
    // leading dots belong to the name
    if (base.find_first_not_of('.') == std::string::npos || dot < base.find_first_not_of('.')) return {};
    std::string ext = base.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

/// The directory of the file holding this code (the library, or the program it is linked into).
std::string module_dir() {
#if defined(_WIN32)
    HMODULE module = nullptr;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCSTR>(&module_dir), &module) != 0) {
        char buffer[MAX_PATH];
        const DWORD n = GetModuleFileNameA(module, buffer, MAX_PATH);
        if (n > 0) return fs::path(std::string(buffer, n)).parent_path().string();
    }
    return {};
#else
    // A shared library has its absolute path; for a program (a static build) it may be the
    // relative path it was started with
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&module_dir), &info) != 0 && info.dli_fname != nullptr &&
        fs::path(info.dli_fname).is_absolute()) {
        return fs::path(info.dli_fname).parent_path().string();
    }
#if defined(__linux__)
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) return exe.parent_path().string();
#endif
    if (info.dli_fname != nullptr && *info.dli_fname) {
        std::error_code canonical_ec;
        const fs::path file = fs::weakly_canonical(info.dli_fname, canonical_ec);
        if (!canonical_ec) return file.parent_path().string();
    }
    return {};
#endif
}

std::string find_font_dir() {
    const std::string probe = FontManager::bundled_fonts.front().file;
    std::vector<fs::path> candidates;
    if (const char* env = std::getenv("BUILD123D_FONT_DIR"); env != nullptr && *env != '\0') candidates.emplace_back(env);
    const std::string dir = module_dir();
    if (!dir.empty()) candidates.push_back(fs::path(dir) / ".." / "share" / "build123d" / "fonts");
#ifdef BUILD123D_INSTALL_FONT_DIR
    candidates.emplace_back(BUILD123D_INSTALL_FONT_DIR);
#endif
#ifdef BUILD123D_SOURCE_FONT_DIR
    candidates.emplace_back(BUILD123D_SOURCE_FONT_DIR);
#endif
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(candidate / probe, ec)) return normpath(candidate.string());
    }
    return {};
}

/// FontManager._get_font_faces: the faces of a font, named and given an aspect from the
/// 'name' table (and the named instances of a variable font).
std::vector<Handle(Font_SystemFont)> get_font_faces(const detail::SfntFont& ft_font, const std::string& path) {
    std::u32string family, sub, preferred;
    for (const auto& record : ft_font.names) {
        const auto value = detail::name_to_unicode(record);
        if (!value) continue;
        if (record.nameID == 1 && family.empty()) {
            family = *value;
        } else if (record.nameID == 2 && sub.empty()) {
            sub = *value;
        } else if (record.nameID == 16 && preferred.empty()) {
            preferred = *value;
        }
    }
    if (!preferred.empty()) family = preferred;

    std::vector<std::u32string> subfamilies;
    if (ft_font.has_fvar) {
        const auto& sub_ids = ft_font.fvar_subfamily_ids;
        for (const auto& record : ft_font.names) {
            if (std::find(sub_ids.begin(), sub_ids.end(), record.nameID) == sub_ids.end()) continue;
            if (const auto value = detail::name_to_unicode(record)) subfamilies.push_back(*value);
        }
    } else {
        subfamilies.push_back(sub);
    }

    // build123d's corrections of OCCT's aspect rules: "Oblique" is "Italic", "Book" isn't "Regular"
    const std::u32string kRegular = U"Regular", kBold = U"Bold", kItalic = U"Italic", kOblique = U"Oblique";
    std::vector<Handle(Font_SystemFont)> fonts;
    for (std::size_t i = 0; i < subfamilies.size(); ++i) {
        std::vector<std::u32string> labels = detail::py_split(subfamilies[i]);
        const auto has = [&](const std::u32string& a) { return std::find(labels.begin(), labels.end(), a) != labels.end(); };
        const bool regular = has(kRegular), bold = has(kBold), italic = has(kItalic) || has(kOblique);
        const auto without = [&](std::initializer_list<std::u32string> drop) {
            std::vector<std::u32string> kept;
            for (const auto& l : labels) {
                if (std::find(drop.begin(), drop.end(), l) == drop.end()) kept.push_back(l);
            }
            return kept;
        };
        Font_FontAspect aspect;
        if (bold) {
            labels = without({kBold, kItalic, kOblique});
            aspect = italic ? Font_FA_BoldItalic : Font_FA_Bold;
        } else if (italic) {
            labels = without({kItalic, kOblique});
            aspect = Font_FA_Italic;
        } else {
            if (regular) labels.clear();
            aspect = Font_FA_Regular;
        }
        std::u32string subfamily;
        for (std::size_t k = 0; k < labels.size(); ++k) subfamily += (k ? U" " : U"") + labels[k];
        std::u32string font_name = subfamily.empty() ? family : family + U" " + subfamily;
        font_name = detail::py_strip(font_name);

        Handle(Font_SystemFont) ocp_font = new Font_SystemFont(TCollection_AsciiString(detail::to_utf8(font_name).c_str()));
        ocp_font->SetFontPath(aspect, TCollection_AsciiString(path.c_str()), static_cast<int>(i << 16));
        ocp_font->SetSingleStrokeFont(std::string(ocp_font->FontKey().ToCString()).rfind("olf ", 0) == 0);
        fonts.push_back(ocp_font);
    }
    return fonts;
}

/// os.walk(top): every directory below `top` (top first, depth first, not into symbolic links).
void walk(const fs::path& top, std::vector<std::string>& roots) {
    std::error_code ec;
    fs::directory_iterator it(top, ec);
    if (ec) return;
    roots.push_back(top.string());
    std::vector<std::string> dirs;
    for (; it != fs::directory_iterator(); it.increment(ec)) {
        if (ec) break;
        std::error_code dir_ec;
        if (it->is_directory(dir_ec)) dirs.push_back(it->path().filename().string());
    }
    for (const auto& name : dirs) {
        const fs::path new_path = top / name;
        std::error_code link_ec;
        if (!fs::is_symlink(new_path, link_ec)) walk(new_path, roots);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// FontInfo
// ---------------------------------------------------------------------------

std::string FontInfo::repr() const {
    std::string out = "Font(name=" + py_str_repr(name) + ", styles=(";
    for (std::size_t i = 0; i < styles.size(); ++i) {
        out += (i ? ", '" : "'") + std::string(style_name(styles[i])) + "'";
    }
    if (styles.size() == 1) out += ",";
    return out + "))";
}

// ---------------------------------------------------------------------------
// FontManager
// ---------------------------------------------------------------------------

const std::vector<BundledFont> FontManager::bundled_fonts = {
    {"Relief SingleLine CAD", "reliefsingleline/ReliefSingleLineCAD-Regular.ttf", true}};

std::string FontManager::bundled_path() {
    static const std::string dir = find_font_dir();
    return dir;
}

namespace detail {

std::shared_mutex& font_registry_mutex() {
    static std::shared_mutex mutex;
    return mutex;
}

}  // namespace detail

namespace {

/// Is the "singleline" alias, which tells whether the bundled fonts are registered, there?
bool bundled_registered(const Handle(Font_FontMgr)& manager) {
    NCollection_Sequence<Handle(TCollection_HAsciiString)> aliases;
    manager->GetAllAliases(aliases);
    for (const auto& alias : aliases) {
        if (!alias.IsNull() && std::string(alias->ToCString()) == "singleline") return true;
    }
    return false;
}

}  // namespace

FontManager::FontManager() {
#if defined(__linux__)
    // (as build123d does on Linux) Once per process: setenv races other threads reading the
    // environment, and the values are the same every time.
    static std::once_flag environment;
    std::call_once(environment, [] {
        setenv("FONTCONFIG_FILE", "/etc/fonts/fonts.conf", 1);
        setenv("FONTCONFIG_PATH", "/etc/fonts/", 1);
    });
#endif
    manager = Font_FontMgr::GetInstance();
    {
        const std::shared_lock<std::shared_mutex> reading(detail::font_registry_mutex());
        if (bundled_registered(manager)) return;
    }
    // Checked again and registered under one exclusive lock, so two threads don't both register
    const std::unique_lock<std::shared_mutex> registering(detail::font_registry_mutex());
    if (bundled_registered(manager)) return;
#if defined(_WIN32)
    register_system_fonts_locked();  // OCCT doesn't add user fonts on Windows
#endif
    const std::string dir = bundled_path();
    for (const auto& font : bundled_fonts) {
        if (dir.empty()) {
            warn("build123d's bundled font '" + font.file +
                 "' was not found; install build123d-cpp or set BUILD123D_FONT_DIR to its data/fonts directory");
            continue;
        }
        register_font_locked(normpath(path_join(dir, font.file)), false, font.single_stroke);
    }
    manager->AddFontAlias(TCollection_AsciiString("singleline"), TCollection_AsciiString("Relief SingleLine CAD"));
}

std::vector<FontInfo> FontManager::available_fonts() const {
    // (the order of build123d's font_aspects dict)
    const std::pair<FontStyle, Font_FontAspect> font_aspects[] = {{FontStyle::REGULAR, Font_FA_Regular},
                                                                  {FontStyle::BOLD, Font_FA_Bold},
                                                                  {FontStyle::BOLDITALIC, Font_FA_BoldItalic},
                                                                  {FontStyle::ITALIC, Font_FA_Italic}};
    std::vector<FontInfo> font_list;
    const std::shared_lock<std::shared_mutex> reading(detail::font_registry_mutex());
    for (const auto& f : manager->GetAvailableFonts()) {
        FontInfo info{f->FontName().ToCString(), {}};
        for (const auto& [style, aspect] : font_aspects) {
            if (f->HasFontAspect(aspect)) info.styles.push_back(style);
        }
        font_list.push_back(std::move(info));
    }
    std::stable_sort(font_list.begin(), font_list.end(), [](const FontInfo& a, const FontInfo& b) { return a.name < b.name; });
    return font_list;
}

Handle(Font_SystemFont) FontManager::check_font(const std::string& path) const {
    const std::shared_lock<std::shared_mutex> reading(detail::font_registry_mutex());
    return manager->CheckFont(path.c_str());
}

Handle(Font_SystemFont) FontManager::find_font(const std::string& name, FontStyle style) const {
    Font_FontAspect aspect = font_aspect(style);  // (OCCT sets it to the aspect it found)
    const std::shared_lock<std::shared_mutex> reading(detail::font_registry_mutex());
    return manager->FindFont(TCollection_AsciiString(name.c_str()), aspect);
}

std::vector<std::string> FontManager::register_font(const std::string& path, bool override, bool single_stroke) {
    const std::unique_lock<std::shared_mutex> registering(detail::font_registry_mutex());
    return register_font_locked(path, override, single_stroke);
}

std::vector<std::string> FontManager::register_font_locked(const std::string& path, bool override, bool single_stroke) {
    std::vector<Handle(Font_SystemFont)> system_fonts;
    try {
        std::vector<detail::SfntFont> fonts;
        if (extension(path) == "ttc") {
            fonts = detail::read_font_collection(path);
        } else {
            try {
                fonts = {detail::read_font(path)};
            } catch (const detail::TTLibFileIsCollectionError&) {
                // Some files with a .ttf extension are collections
                fonts = detail::read_font_collection(path);
            }
        }
        for (const auto& font : fonts) {
            for (const auto& face : get_font_faces(font, path)) system_fonts.push_back(face);
        }
    } catch (const detail::TTLibError&) {
        // build123d logs "Failed to load font file" (its logger has only a NullHandler)
        return {};
    }
    std::vector<std::string> font_faces;
    for (const auto& font : system_fonts) {
        font_faces.emplace_back(font->FontName().ToCString());
        font->SetSingleStrokeFont(single_stroke);
        manager->RegisterFont(font, override);
    }
    return font_faces;
}

std::vector<std::string> FontManager::register_folder(const std::string& path, bool override, bool single_stroke) {
    const std::unique_lock<std::shared_mutex> registering(detail::font_registry_mutex());
    return register_folder_locked(path, override, single_stroke);
}

std::vector<std::string> FontManager::register_folder_locked(const std::string& path, bool override, bool single_stroke) {
    std::vector<std::string> font_faces;
    const std::string folder = normpath(path);
    for (const char* ext : {"ttf", "otf", "ttc"}) {
        // glob(folder/*ext): the names ending with ext, not hidden ones, in directory order
        std::error_code ec;
        fs::directory_iterator it(folder, ec);
        if (ec) continue;
        const std::string suffix(ext);
        for (; it != fs::directory_iterator(); it.increment(ec)) {
            if (ec) break;
            const std::string name = it->path().filename().string();
            if (name.empty() || name[0] == '.' || name.size() < suffix.size() ||
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
                continue;
            }
            for (auto& face : register_font_locked(path_join(folder, name), override, single_stroke)) {
                font_faces.push_back(std::move(face));
            }
        }
    }
    // build123d returns list(set(font_faces)): each name once (in an arbitrary order there)
    std::vector<std::string> unique;
    for (const auto& face : font_faces) {
        if (std::find(unique.begin(), unique.end(), face) == unique.end()) unique.push_back(face);
    }
    return unique;
}

void FontManager::register_system_fonts() {
    const std::unique_lock<std::shared_mutex> registering(detail::font_registry_mutex());
    register_system_fonts_locked();
}

void FontManager::register_system_fonts_locked() {
    std::vector<std::string> paths;
#if defined(_WIN32)
    const char* user = std::getenv("USERNAME");
    paths = {"C:/Windows/Fonts", std::string("C:/Users/") + (user ? user : "") + "/AppData/Local/Microsoft/Windows/Fonts"};
#elif defined(__APPLE__)
    const char* home = std::getenv("HOME");
    paths = {"/System/Library/Fonts", "/System/Library/Fonts/Supplemental", "/Library/Fonts",
             "/Library/Fonts/Supplemental", std::string(home ? home : "~") + "/Library/Fonts"};
#else
    const char* home = std::getenv("HOME");
    const std::string user_home = home ? home : "~";
    const std::vector<std::string> base_paths = {"/system/fonts", "/usr/share/fonts", "/usr/local/share/fonts",
                                                 user_home + "/.fonts", user_home + "/.local/share/fonts"};
    for (const auto& base : base_paths) {
        std::error_code ec;
        if (fs::exists(base, ec)) walk(base, paths);
    }
#endif
    for (const auto& path : paths) {
        std::error_code ec;
        if (fs::exists(path, ec)) register_folder_locked(path, false, false);
    }
}

std::vector<FontInfo> available_fonts() { return FontManager().available_fonts(); }

}  // namespace build123d
