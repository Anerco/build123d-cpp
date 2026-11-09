// Port of the fontTools code build123d's FontManager relies on (see font_file.hpp).
#include "font_file.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>

#if __has_include(<iconv.h>)
#include <iconv.h>
#define BUILD123D_HAVE_ICONV 1
#endif

namespace build123d::detail {

namespace {

// ---------------------------------------------------------------------------
// Bytes
// ---------------------------------------------------------------------------

/// The whole file, as Python's open(path, "rb").read() (OSError messages included).
std::string read_file(const std::string& path) {
    std::error_code ec;
    if (std::filesystem::is_directory(path, ec)) {
        throw std::runtime_error("[Errno " + std::to_string(EISDIR) + "] " + std::strerror(EISDIR) + ": '" + path + "'");
    }
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (file == nullptr) {
        const int err = errno;
        throw std::runtime_error("[Errno " + std::to_string(err) + "] " + std::strerror(err) + ": '" + path + "'");
    }
    std::string data;
    char buffer[65536];
    std::size_t n = 0;
    while ((n = std::fread(buffer, 1, sizeof(buffer), file)) > 0) data.append(buffer, n);
    std::fclose(file);
    return data;
}

/// file.read(size) at `offset`: fewer bytes at the end of the data.
std::string read_at(const std::string& data, std::size_t offset, std::size_t size) {
    if (offset >= data.size()) return {};
    return data.substr(offset, size);
}

std::uint16_t u16(const std::string& d, std::size_t pos) {
    return static_cast<std::uint16_t>((static_cast<unsigned char>(d[pos]) << 8) | static_cast<unsigned char>(d[pos + 1]));
}

std::uint32_t u32(const std::string& d, std::size_t pos) {
    return (static_cast<std::uint32_t>(u16(d, pos)) << 16) | u16(d, pos + 2);
}

/// struct.error of struct.unpack on too little data.
[[noreturn]] void struct_error(std::size_t size) {
    throw std::runtime_error("unpack requires a buffer of " + std::to_string(size) + " bytes");
}

// ---------------------------------------------------------------------------
// sfnt directories (SFNTReader, readTTCHeader)
// ---------------------------------------------------------------------------

struct TableEntry {
    std::uint32_t offset = 0;
    std::uint32_t length = 0;
};

using TableDirectory = std::map<std::string, TableEntry>;

constexpr std::uint32_t TTC_V1 = 0x00010000;
constexpr std::uint32_t TTC_V1_1 = 0x00010001;
constexpr std::uint32_t TTC_V2 = 0x00020000;
constexpr std::uint32_t TTC_V2_1 = 0x00020001;

struct TTCHeader {
    std::vector<std::uint32_t> offsets;   // offsetTable
    std::vector<std::uint32_t> offsets2;  // offsetTable2 (versions 1.1 and 2.1)
    bool has_table2 = false;
};

TTCHeader read_ttc_header(const std::string& data) {
    const std::string head = read_at(data, 0, 12);
    if (head.size() != 12) throw TTLibError("Not a Font Collection (not enough data)");
    if (head.substr(0, 4) != "ttcf") throw TTLibError("Not a Font Collection");
    const std::uint32_t version = u32(head, 4);
    const std::uint32_t num_fonts = u32(head, 8);
    if (version != TTC_V1 && version != TTC_V1_1 && version != TTC_V2 && version != TTC_V2_1) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "unrecognized TTC version 0x%08x", version);
        throw std::runtime_error(buf);  // Python asserts
    }
    TTCHeader header;
    std::size_t pos = 12;
    const std::string table = read_at(data, pos, static_cast<std::size_t>(num_fonts) * 4);
    if (table.size() != static_cast<std::size_t>(num_fonts) * 4) struct_error(static_cast<std::size_t>(num_fonts) * 4);
    for (std::uint32_t i = 0; i < num_fonts; ++i) header.offsets.push_back(u32(table, 4 * i));
    pos += table.size();
    if (version == TTC_V2 || version == TTC_V2_1) {
        if (read_at(data, pos, 12).size() != 12) throw TTLibError("Not a Font Collection (not enough data)");
        pos += 12;
    }
    if (version == TTC_V1_1 || version == TTC_V2_1) {
        const std::string count = read_at(data, pos, 4);
        if (count.size() != 4) throw TTLibError("Not a Font Collection (not enough data)");
        pos += 4;
        const std::uint32_t num_fonts2 = u32(count, 0);
        const std::string table2 = read_at(data, pos, static_cast<std::size_t>(num_fonts2) * 4);
        if (table2.size() != static_cast<std::size_t>(num_fonts2) * 4) {
            throw TTLibError("Not a Font Collection (not enough data)");
        }
        for (std::uint32_t i = 0; i < num_fonts2; ++i) header.offsets2.push_back(u32(table2, 4 * i));
        header.has_table2 = true;
    }
    return header;
}

/// SFNTReader(file, fontNumber): the table directory of one font (font_number -1 for a
/// single font file).
TableDirectory read_directory(const std::string& data, long font_number) {
    const std::string sfnt_version = read_at(data, 0, 4);
    std::string directory;
    std::size_t pos = 0;
    if (sfnt_version == "wOF2" || sfnt_version == "wOFF") {
        // fontTools reads WOFF/WOFF2 too (zlib/brotli compressed tables); not ported
        throw TTLibError("WOFF and WOFF2 fonts are not supported");
    }
    if (sfnt_version == "ttcf") {
        const TTCHeader header = read_ttc_header(data);
        const std::vector<std::uint32_t>& offsets = header.has_table2 ? header.offsets2 : header.offsets;
        const long num_fonts = static_cast<long>(offsets.size());
        if (!(0 <= font_number && font_number < num_fonts)) {
            throw TTLibFileIsCollectionError("specify a font number between 0 and " + std::to_string(num_fonts - 1) +
                                             " (inclusive)");
        }
        pos = offsets[static_cast<std::size_t>(font_number)];
        directory = read_at(data, pos, 12);
        if (directory.size() != 12) throw TTLibError("Not a Font Collection (not enough data)");
    } else {
        directory = read_at(data, 0, 12);
        if (directory.size() != 12) throw TTLibError("Not a TrueType or OpenType font (not enough data)");
    }
    const std::string version = directory.substr(0, 4);
    if (version != std::string("\x00\x01\x00\x00", 4) && version != "OTTO" && version != "true") {
        throw TTLibError("Not a TrueType or OpenType font (bad sfntVersion)");
    }
    const std::uint16_t num_tables = u16(directory, 4);
    pos += 12;
    TableDirectory tables;
    for (std::uint16_t i = 0; i < num_tables; ++i) {
        const std::string entry = read_at(data, pos, 16);
        if (entry.size() != 16) {
            throw TTLibError("unexpected end of table directory: expected 16 bytes but got " +
                             std::to_string(entry.size()));
        }
        tables[entry.substr(0, 4)] = TableEntry{u32(entry, 8), u32(entry, 12)};
        pos += 16;
    }
    return tables;
}

/// DirectoryEntry.loadData.
std::string table_data(const std::string& data, const TableDirectory& tables, const std::string& tag) {
    const auto it = tables.find(tag);
    if (it == tables.end()) throw std::out_of_range("'" + tag + "'");  // Python's KeyError
    const std::string bytes = read_at(data, it->second.offset, it->second.length);
    if (bytes.size() != it->second.length) {
        throw TTLibError("unexpected end of '" + tag + "' table data: expected " + std::to_string(it->second.length) +
                         " bytes but got " + std::to_string(bytes.size()) + " at offset " +
                         std::to_string(it->second.offset));
    }
    return bytes;
}

/// table__n_a_m_e.decompile (records that don't fit are skipped, as fontTools does).
std::vector<NameRecord> decompile_name(const std::string& data) {
    if (data.size() < 6) struct_error(6);
    const std::uint16_t n = u16(data, 2);
    const std::uint16_t string_offset = u16(data, 4);
    const std::string string_data = string_offset < data.size() ? data.substr(string_offset) : std::string();
    std::vector<NameRecord> names;
    std::size_t pos = 6;
    for (std::uint16_t i = 0; i < n; ++i) {
        if (data.size() < pos + 12) continue;  // "skipping malformed name record"
        NameRecord record;
        record.platformID = u16(data, pos);
        record.platEncID = u16(data, pos + 2);
        record.langID = u16(data, pos + 4);
        record.nameID = u16(data, pos + 6);
        const std::size_t length = u16(data, pos + 8);
        const std::size_t offset = u16(data, pos + 10);
        pos += 12;
        if (offset + length > string_data.size()) continue;  // "skipping malformed name record"
        record.string = string_data.substr(offset, length);
        names.push_back(std::move(record));
    }
    return names;
}

/// table__f_v_a_r.decompile: the subfamilyNameID of each named instance.
std::vector<std::uint16_t> decompile_fvar(const std::string& data) {
    if (data.size() < 16) struct_error(16);
    const std::uint32_t version = u32(data, 0);
    if (version != 0x00010000) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "unsupported 'fvar' version %04x", version);
        throw TTLibError(buf);
    }
    std::size_t pos = u16(data, 4);
    const std::uint16_t axis_count = u16(data, 8);
    const std::uint16_t axis_size = u16(data, 10);
    const std::uint16_t instance_count = u16(data, 12);
    const std::uint16_t instance_size = u16(data, 14);
    const auto slice_size = [&](std::size_t from, std::size_t size) {
        return from >= data.size() ? std::size_t{0} : std::min(size, data.size() - from);
    };
    for (std::uint16_t i = 0; i < axis_count; ++i) {
        if (slice_size(pos, axis_size) < 20) struct_error(20);
        pos += axis_size;
    }
    std::vector<std::uint16_t> ids;
    for (std::uint16_t i = 0; i < instance_count; ++i) {
        const std::size_t available = slice_size(pos, instance_size);
        if (available < 4) struct_error(4);
        if (available < 4 + 4 * static_cast<std::size_t>(axis_count)) struct_error(4);  // a coordinate is missing
        ids.push_back(u16(data, pos));
        pos += instance_size;
    }
    return ids;
}

/// The tables build123d reads, in its order: font["name"].names, "fvar" in font,
/// font["fvar"].instances.
SfntFont read_tables(const std::string& data, const TableDirectory& tables) {
    SfntFont font;
    font.names = decompile_name(table_data(data, tables, "name"));
    font.has_fvar = tables.count("fvar") > 0;
    if (font.has_fvar) font.fvar_subfamily_ids = decompile_fvar(table_data(data, tables, "fvar"));
    return font;
}

// ---------------------------------------------------------------------------
// Decoding (encodingTools.getEncoding and the codecs it names)
// ---------------------------------------------------------------------------

/// The upper halves (0x80-0xFF) of Python's Mac single-byte codecs (generated with Python).
const char32_t kMacRoman[128] = {  // mac_roman
        0x00C4, 0x00C5, 0x00C7, 0x00C9, 0x00D1, 0x00D6, 0x00DC, 0x00E1,
        0x00E0, 0x00E2, 0x00E4, 0x00E3, 0x00E5, 0x00E7, 0x00E9, 0x00E8,
        0x00EA, 0x00EB, 0x00ED, 0x00EC, 0x00EE, 0x00EF, 0x00F1, 0x00F3,
        0x00F2, 0x00F4, 0x00F6, 0x00F5, 0x00FA, 0x00F9, 0x00FB, 0x00FC,
        0x2020, 0x00B0, 0x00A2, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x00DF,
        0x00AE, 0x00A9, 0x2122, 0x00B4, 0x00A8, 0x2260, 0x00C6, 0x00D8,
        0x221E, 0x00B1, 0x2264, 0x2265, 0x00A5, 0x00B5, 0x2202, 0x2211,
        0x220F, 0x03C0, 0x222B, 0x00AA, 0x00BA, 0x03A9, 0x00E6, 0x00F8,
        0x00BF, 0x00A1, 0x00AC, 0x221A, 0x0192, 0x2248, 0x2206, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x00C0, 0x00C3, 0x00D5, 0x0152, 0x0153,
        0x2013, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x25CA,
        0x00FF, 0x0178, 0x2044, 0x20AC, 0x2039, 0x203A, 0xFB01, 0xFB02,
        0x2021, 0x00B7, 0x201A, 0x201E, 0x2030, 0x00C2, 0x00CA, 0x00C1,
        0x00CB, 0x00C8, 0x00CD, 0x00CE, 0x00CF, 0x00CC, 0x00D3, 0x00D4,
        0xF8FF, 0x00D2, 0x00DA, 0x00DB, 0x00D9, 0x0131, 0x02C6, 0x02DC,
        0x00AF, 0x02D8, 0x02D9, 0x02DA, 0x00B8, 0x02DD, 0x02DB, 0x02C7};
const char32_t kMacIceland[128] = {  // mac_iceland
        0x00C4, 0x00C5, 0x00C7, 0x00C9, 0x00D1, 0x00D6, 0x00DC, 0x00E1,
        0x00E0, 0x00E2, 0x00E4, 0x00E3, 0x00E5, 0x00E7, 0x00E9, 0x00E8,
        0x00EA, 0x00EB, 0x00ED, 0x00EC, 0x00EE, 0x00EF, 0x00F1, 0x00F3,
        0x00F2, 0x00F4, 0x00F6, 0x00F5, 0x00FA, 0x00F9, 0x00FB, 0x00FC,
        0x00DD, 0x00B0, 0x00A2, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x00DF,
        0x00AE, 0x00A9, 0x2122, 0x00B4, 0x00A8, 0x2260, 0x00C6, 0x00D8,
        0x221E, 0x00B1, 0x2264, 0x2265, 0x00A5, 0x00B5, 0x2202, 0x2211,
        0x220F, 0x03C0, 0x222B, 0x00AA, 0x00BA, 0x03A9, 0x00E6, 0x00F8,
        0x00BF, 0x00A1, 0x00AC, 0x221A, 0x0192, 0x2248, 0x2206, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x00C0, 0x00C3, 0x00D5, 0x0152, 0x0153,
        0x2013, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x25CA,
        0x00FF, 0x0178, 0x2044, 0x20AC, 0x00D0, 0x00F0, 0x00DE, 0x00FE,
        0x00FD, 0x00B7, 0x201A, 0x201E, 0x2030, 0x00C2, 0x00CA, 0x00C1,
        0x00CB, 0x00C8, 0x00CD, 0x00CE, 0x00CF, 0x00CC, 0x00D3, 0x00D4,
        0xF8FF, 0x00D2, 0x00DA, 0x00DB, 0x00D9, 0x0131, 0x02C6, 0x02DC,
        0x00AF, 0x02D8, 0x02D9, 0x02DA, 0x00B8, 0x02DD, 0x02DB, 0x02C7};
const char32_t kMacTurkish[128] = {  // mac_turkish
        0x00C4, 0x00C5, 0x00C7, 0x00C9, 0x00D1, 0x00D6, 0x00DC, 0x00E1,
        0x00E0, 0x00E2, 0x00E4, 0x00E3, 0x00E5, 0x00E7, 0x00E9, 0x00E8,
        0x00EA, 0x00EB, 0x00ED, 0x00EC, 0x00EE, 0x00EF, 0x00F1, 0x00F3,
        0x00F2, 0x00F4, 0x00F6, 0x00F5, 0x00FA, 0x00F9, 0x00FB, 0x00FC,
        0x2020, 0x00B0, 0x00A2, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x00DF,
        0x00AE, 0x00A9, 0x2122, 0x00B4, 0x00A8, 0x2260, 0x00C6, 0x00D8,
        0x221E, 0x00B1, 0x2264, 0x2265, 0x00A5, 0x00B5, 0x2202, 0x2211,
        0x220F, 0x03C0, 0x222B, 0x00AA, 0x00BA, 0x03A9, 0x00E6, 0x00F8,
        0x00BF, 0x00A1, 0x00AC, 0x221A, 0x0192, 0x2248, 0x2206, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x00C0, 0x00C3, 0x00D5, 0x0152, 0x0153,
        0x2013, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x25CA,
        0x00FF, 0x0178, 0x011E, 0x011F, 0x0130, 0x0131, 0x015E, 0x015F,
        0x2021, 0x00B7, 0x201A, 0x201E, 0x2030, 0x00C2, 0x00CA, 0x00C1,
        0x00CB, 0x00C8, 0x00CD, 0x00CE, 0x00CF, 0x00CC, 0x00D3, 0x00D4,
        0xF8FF, 0x00D2, 0x00DA, 0x00DB, 0x00D9, 0xF8A0, 0x02C6, 0x02DC,
        0x00AF, 0x02D8, 0x02D9, 0x02DA, 0x00B8, 0x02DD, 0x02DB, 0x02C7};
const char32_t kMacCroatian[128] = {  // mac_croatian
        0x00C4, 0x00C5, 0x00C7, 0x00C9, 0x00D1, 0x00D6, 0x00DC, 0x00E1,
        0x00E0, 0x00E2, 0x00E4, 0x00E3, 0x00E5, 0x00E7, 0x00E9, 0x00E8,
        0x00EA, 0x00EB, 0x00ED, 0x00EC, 0x00EE, 0x00EF, 0x00F1, 0x00F3,
        0x00F2, 0x00F4, 0x00F6, 0x00F5, 0x00FA, 0x00F9, 0x00FB, 0x00FC,
        0x2020, 0x00B0, 0x00A2, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x00DF,
        0x00AE, 0x0160, 0x2122, 0x00B4, 0x00A8, 0x2260, 0x017D, 0x00D8,
        0x221E, 0x00B1, 0x2264, 0x2265, 0x2206, 0x00B5, 0x2202, 0x2211,
        0x220F, 0x0161, 0x222B, 0x00AA, 0x00BA, 0x03A9, 0x017E, 0x00F8,
        0x00BF, 0x00A1, 0x00AC, 0x221A, 0x0192, 0x2248, 0x0106, 0x00AB,
        0x010C, 0x2026, 0x00A0, 0x00C0, 0x00C3, 0x00D5, 0x0152, 0x0153,
        0x0110, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x25CA,
        0xF8FF, 0x00A9, 0x2044, 0x20AC, 0x2039, 0x203A, 0x00C6, 0x00BB,
        0x2013, 0x00B7, 0x201A, 0x201E, 0x2030, 0x00C2, 0x0107, 0x00C1,
        0x010D, 0x00C8, 0x00CD, 0x00CE, 0x00CF, 0x00CC, 0x00D3, 0x00D4,
        0x0111, 0x00D2, 0x00DA, 0x00DB, 0x00D9, 0x0131, 0x02C6, 0x02DC,
        0x00AF, 0x03C0, 0x00CB, 0x02DA, 0x00B8, 0x00CA, 0x00E6, 0x02C7};
const char32_t kMacLatin2[128] = {  // mac_latin2
        0x00C4, 0x0100, 0x0101, 0x00C9, 0x0104, 0x00D6, 0x00DC, 0x00E1,
        0x0105, 0x010C, 0x00E4, 0x010D, 0x0106, 0x0107, 0x00E9, 0x0179,
        0x017A, 0x010E, 0x00ED, 0x010F, 0x0112, 0x0113, 0x0116, 0x00F3,
        0x0117, 0x00F4, 0x00F6, 0x00F5, 0x00FA, 0x011A, 0x011B, 0x00FC,
        0x2020, 0x00B0, 0x0118, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x00DF,
        0x00AE, 0x00A9, 0x2122, 0x0119, 0x00A8, 0x2260, 0x0123, 0x012E,
        0x012F, 0x012A, 0x2264, 0x2265, 0x012B, 0x0136, 0x2202, 0x2211,
        0x0142, 0x013B, 0x013C, 0x013D, 0x013E, 0x0139, 0x013A, 0x0145,
        0x0146, 0x0143, 0x00AC, 0x221A, 0x0144, 0x0147, 0x2206, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x0148, 0x0150, 0x00D5, 0x0151, 0x014C,
        0x2013, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x25CA,
        0x014D, 0x0154, 0x0155, 0x0158, 0x2039, 0x203A, 0x0159, 0x0156,
        0x0157, 0x0160, 0x201A, 0x201E, 0x0161, 0x015A, 0x015B, 0x00C1,
        0x0164, 0x0165, 0x00CD, 0x017D, 0x017E, 0x016A, 0x00D3, 0x00D4,
        0x016B, 0x016E, 0x00DA, 0x016F, 0x0170, 0x0171, 0x0172, 0x0173,
        0x00DD, 0x00FD, 0x0137, 0x017B, 0x0141, 0x017C, 0x0122, 0x02C7};
const char32_t kMacRomanian[128] = {  // mac_romanian
        0x00C4, 0x00C5, 0x00C7, 0x00C9, 0x00D1, 0x00D6, 0x00DC, 0x00E1,
        0x00E0, 0x00E2, 0x00E4, 0x00E3, 0x00E5, 0x00E7, 0x00E9, 0x00E8,
        0x00EA, 0x00EB, 0x00ED, 0x00EC, 0x00EE, 0x00EF, 0x00F1, 0x00F3,
        0x00F2, 0x00F4, 0x00F6, 0x00F5, 0x00FA, 0x00F9, 0x00FB, 0x00FC,
        0x2020, 0x00B0, 0x00A2, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x00DF,
        0x00AE, 0x00A9, 0x2122, 0x00B4, 0x00A8, 0x2260, 0x0102, 0x0218,
        0x221E, 0x00B1, 0x2264, 0x2265, 0x00A5, 0x00B5, 0x2202, 0x2211,
        0x220F, 0x03C0, 0x222B, 0x00AA, 0x00BA, 0x03A9, 0x0103, 0x0219,
        0x00BF, 0x00A1, 0x00AC, 0x221A, 0x0192, 0x2248, 0x2206, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x00C0, 0x00C3, 0x00D5, 0x0152, 0x0153,
        0x2013, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x25CA,
        0x00FF, 0x0178, 0x2044, 0x20AC, 0x2039, 0x203A, 0x021A, 0x021B,
        0x2021, 0x00B7, 0x201A, 0x201E, 0x2030, 0x00C2, 0x00CA, 0x00C1,
        0x00CB, 0x00C8, 0x00CD, 0x00CE, 0x00CF, 0x00CC, 0x00D3, 0x00D4,
        0xF8FF, 0x00D2, 0x00DA, 0x00DB, 0x00D9, 0x0131, 0x02C6, 0x02DC,
        0x00AF, 0x02D8, 0x02D9, 0x02DA, 0x00B8, 0x02DD, 0x02DB, 0x02C7};
const char32_t kMacGreek[128] = {  // mac_greek
        0x00C4, 0x00B9, 0x00B2, 0x00C9, 0x00B3, 0x00D6, 0x00DC, 0x0385,
        0x00E0, 0x00E2, 0x00E4, 0x0384, 0x00A8, 0x00E7, 0x00E9, 0x00E8,
        0x00EA, 0x00EB, 0x00A3, 0x2122, 0x00EE, 0x00EF, 0x2022, 0x00BD,
        0x2030, 0x00F4, 0x00F6, 0x00A6, 0x20AC, 0x00F9, 0x00FB, 0x00FC,
        0x2020, 0x0393, 0x0394, 0x0398, 0x039B, 0x039E, 0x03A0, 0x00DF,
        0x00AE, 0x00A9, 0x03A3, 0x03AA, 0x00A7, 0x2260, 0x00B0, 0x00B7,
        0x0391, 0x00B1, 0x2264, 0x2265, 0x00A5, 0x0392, 0x0395, 0x0396,
        0x0397, 0x0399, 0x039A, 0x039C, 0x03A6, 0x03AB, 0x03A8, 0x03A9,
        0x03AC, 0x039D, 0x00AC, 0x039F, 0x03A1, 0x2248, 0x03A4, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x03A5, 0x03A7, 0x0386, 0x0388, 0x0153,
        0x2013, 0x2015, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x0389,
        0x038A, 0x038C, 0x038E, 0x03AD, 0x03AE, 0x03AF, 0x03CC, 0x038F,
        0x03CD, 0x03B1, 0x03B2, 0x03C8, 0x03B4, 0x03B5, 0x03C6, 0x03B3,
        0x03B7, 0x03B9, 0x03BE, 0x03BA, 0x03BB, 0x03BC, 0x03BD, 0x03BF,
        0x03C0, 0x03CE, 0x03C1, 0x03C3, 0x03C4, 0x03B8, 0x03C9, 0x03C2,
        0x03C7, 0x03C5, 0x03B6, 0x03CA, 0x03CB, 0x0390, 0x03B0, 0x00AD};
const char32_t kMacCyrillic[128] = {  // mac_cyrillic
        0x0410, 0x0411, 0x0412, 0x0413, 0x0414, 0x0415, 0x0416, 0x0417,
        0x0418, 0x0419, 0x041A, 0x041B, 0x041C, 0x041D, 0x041E, 0x041F,
        0x0420, 0x0421, 0x0422, 0x0423, 0x0424, 0x0425, 0x0426, 0x0427,
        0x0428, 0x0429, 0x042A, 0x042B, 0x042C, 0x042D, 0x042E, 0x042F,
        0x2020, 0x00B0, 0x0490, 0x00A3, 0x00A7, 0x2022, 0x00B6, 0x0406,
        0x00AE, 0x00A9, 0x2122, 0x0402, 0x0452, 0x2260, 0x0403, 0x0453,
        0x221E, 0x00B1, 0x2264, 0x2265, 0x0456, 0x00B5, 0x0491, 0x0408,
        0x0404, 0x0454, 0x0407, 0x0457, 0x0409, 0x0459, 0x040A, 0x045A,
        0x0458, 0x0405, 0x00AC, 0x221A, 0x0192, 0x2248, 0x2206, 0x00AB,
        0x00BB, 0x2026, 0x00A0, 0x040B, 0x045B, 0x040C, 0x045C, 0x0455,
        0x2013, 0x2014, 0x201C, 0x201D, 0x2018, 0x2019, 0x00F7, 0x201E,
        0x040E, 0x045E, 0x040F, 0x045F, 0x2116, 0x0401, 0x0451, 0x044F,
        0x0430, 0x0431, 0x0432, 0x0433, 0x0434, 0x0435, 0x0436, 0x0437,
        0x0438, 0x0439, 0x043A, 0x043B, 0x043C, 0x043D, 0x043E, 0x043F,
        0x0440, 0x0441, 0x0442, 0x0443, 0x0444, 0x0445, 0x0446, 0x0447,
        0x0448, 0x0449, 0x044A, 0x044B, 0x044C, 0x044D, 0x044E, 0x20AC};

struct Codec {
    enum Kind { UTF16BE, ASCII, LATIN1, MAC, ICONV } kind = ASCII;
    const char32_t* mac_table = nullptr;
    const char* iconv_name = nullptr;
    /// fontTools' ExtendCodec mapping for the x_mac_*_ttx codecs: bytes the base codec
    /// rejects (a 0 entry is not mapped).
    const char32_t* extension = nullptr;
};

// ExtendCodec mappings of fontTools/encodings/codecs.py, by byte
const char32_t* japanese_extension() {
    static const auto table = [] {
        static char32_t t[256] = {};
        t[0xFC] = 0x007C; t[0x7E] = 0x007E; t[0x80] = 0x005C; t[0xA0] = 0x00A0;
        t[0xFD] = 0x00A9; t[0xFE] = 0x2122; t[0xFF] = 0x2026;
        return t;
    }();
    return table;
}
const char32_t* trad_chinese_extension() {
    static const auto table = [] {
        static char32_t t[256] = {};
        t[0x80] = 0x005C; t[0xA0] = 0x00A0; t[0xFD] = 0x00A9; t[0xFE] = 0x2122; t[0xFF] = 0x2026;
        return t;
    }();
    return table;
}
const char32_t* korean_extension() {
    static const auto table = [] {
        static char32_t t[256] = {};
        t[0x80] = 0x00A0; t[0x81] = 0x20A9; t[0x82] = 0x2014; t[0x83] = 0x00A9; t[0xFE] = 0x2122; t[0xFF] = 0x2026;
        return t;
    }();
    return table;
}
const char32_t* simp_chinese_extension() {
    static const auto table = [] {
        static char32_t t[256] = {};
        t[0x80] = 0x00FC; t[0xA0] = 0x00A0; t[0xFD] = 0x00A9; t[0xFE] = 0x2122; t[0xFF] = 0x2026;
        return t;
    }();
    return table;
}

/// NameRecord.getEncoding() (encodingTools.getEncoding with the default "ascii").
Codec get_encoding(std::uint16_t platform_id, std::uint16_t plat_enc_id, std::uint16_t lang_id) {
    const auto utf16 = [] { return Codec{Codec::UTF16BE}; };
    const auto mac = [](const char32_t* table) { return Codec{Codec::MAC, table}; };
    const auto conv = [](const char* name, const char32_t* extension = nullptr) {
        return Codec{Codec::ICONV, nullptr, name, extension};
    };
    switch (platform_id) {
        case 0:  // Unicode
            if (plat_enc_id <= 6) return utf16();
            break;
        case 1:  // Macintosh
            switch (plat_enc_id) {
                case 0:
                    switch (lang_id) {
                        case 15: return mac(kMacIceland);
                        case 17: return mac(kMacTurkish);
                        case 18: return mac(kMacCroatian);
                        case 24: case 25: case 26: case 27: case 28: case 36: case 38: case 39: case 40:
                            return mac(kMacLatin2);
                        case 37: return mac(kMacRomanian);
                        default: return mac(kMacRoman);
                    }
                case 1: return conv("SHIFT_JIS", japanese_extension());
                case 2: return conv("BIG5", trad_chinese_extension());
                case 3: return conv("EUC-KR", korean_extension());
                case 6: return mac(kMacGreek);
                case 7: return mac(kMacCyrillic);
                case 25: return conv("GB2312", simp_chinese_extension());
                case 29: return mac(kMacLatin2);
                case 35: return mac(kMacTurkish);
                case 37: return mac(kMacIceland);
                default: break;
            }
            break;
        case 2:  // ISO
            if (plat_enc_id == 0) return Codec{Codec::ASCII};
            if (plat_enc_id == 1) return utf16();
            if (plat_enc_id == 2) return Codec{Codec::LATIN1};
            break;
        case 3:  // Microsoft
            switch (plat_enc_id) {
                case 0: case 1: case 10: return utf16();
                case 2: return conv("SHIFT_JIS");
                case 3: return conv("GB2312");
                case 4: return conv("BIG5");
                case 5: return conv("EUC-KR");
                case 6: return conv("JOHAB");
                default: break;
            }
            break;
        default: break;
    }
    return Codec{Codec::ASCII};
}

/// Python's bytes.decode("utf_16_be") (strict).
std::optional<std::u32string> decode_utf16be(const std::string& bytes) {
    if (bytes.size() % 2 != 0) return std::nullopt;  // "truncated data"
    std::u32string out;
    for (std::size_t i = 0; i < bytes.size(); i += 2) {
        const char32_t unit = u16(bytes, i);
        if (unit >= 0xD800 && unit <= 0xDBFF) {
            if (i + 2 >= bytes.size()) return std::nullopt;
            const char32_t low = u16(bytes, i + 2);
            if (low < 0xDC00 || low > 0xDFFF) return std::nullopt;
            out.push_back(0x10000 + ((unit - 0xD800) << 10) + (low - 0xDC00));
            i += 2;
        } else if (unit >= 0xDC00 && unit <= 0xDFFF) {
            return std::nullopt;  // "illegal encoding"
        } else {
            out.push_back(unit);
        }
    }
    return out;
}

std::optional<std::u32string> decode_iconv(const std::string& bytes, const Codec& codec) {
#ifdef BUILD123D_HAVE_ICONV
    const std::uint16_t probe = 1;
    const bool little_endian = *reinterpret_cast<const unsigned char*>(&probe) == 1;
    iconv_t cd = iconv_open(little_endian ? "UTF-32LE" : "UTF-32BE", codec.iconv_name);
    if (cd == reinterpret_cast<iconv_t>(-1)) return std::nullopt;
    std::u32string out;
    std::string input = bytes;
    char* in = input.data();
    std::size_t in_left = input.size();
    bool ok = true;
    while (in_left > 0) {
        char32_t buffer[256];
        char* outp = reinterpret_cast<char*>(buffer);
        std::size_t out_left = sizeof(buffer);
        const std::size_t r = iconv(cd, &in, &in_left, &outp, &out_left);
        out.append(buffer, (sizeof(buffer) - out_left) / sizeof(char32_t));
        if (r == static_cast<std::size_t>(-1)) {
            if (errno == E2BIG) continue;
            // ExtendCodec: a byte the base codec rejects may be one of the extension's
            const unsigned char byte = static_cast<unsigned char>(*in);
            if (codec.extension != nullptr && codec.extension[byte] != 0) {
                out.push_back(codec.extension[byte]);
                ++in;
                --in_left;
                iconv(cd, nullptr, nullptr, nullptr, nullptr);
                continue;
            }
            ok = false;
            break;
        }
    }
    iconv_close(cd);
    if (!ok) return std::nullopt;
    return out;
#else
    return std::nullopt;
#endif
}

std::optional<std::u32string> decode(const std::string& bytes, const Codec& codec) {
    switch (codec.kind) {
        case Codec::UTF16BE: return decode_utf16be(bytes);
        case Codec::ASCII: {
            std::u32string out;
            for (const char c : bytes) {
                const auto b = static_cast<unsigned char>(c);
                if (b >= 0x80) return std::nullopt;
                out.push_back(b);
            }
            return out;
        }
        case Codec::LATIN1: {
            std::u32string out;
            for (const char c : bytes) out.push_back(static_cast<unsigned char>(c));
            return out;
        }
        case Codec::MAC: {
            std::u32string out;
            for (const char c : bytes) {
                const auto b = static_cast<unsigned char>(c);
                out.push_back(b < 0x80 ? char32_t{b} : codec.mac_table[b - 0x80]);
            }
            return out;
        }
        case Codec::ICONV: return decode_iconv(bytes, codec);
    }
    return std::nullopt;
}

bool is_py_space(char32_t c) {
    return (c >= 0x09 && c <= 0x0D) || (c >= 0x1C && c <= 0x20) || c == 0x85 || c == 0xA0 || c == 0x1680 ||
           (c >= 0x2000 && c <= 0x200A) || c == 0x2028 || c == 0x2029 || c == 0x202F || c == 0x205F || c == 0x3000;
}

}  // namespace

std::optional<std::u32string> name_to_unicode(const NameRecord& record) {
    const auto isascii = [](char32_t b) { return (b >= 0x20 && b <= 0x7E) || b == 0x09 || b == 0x0A || b == 0x0D; };
    const Codec codec = get_encoding(record.platformID, record.platEncID, record.langID);
    std::string string = record.string;
    if (codec.kind == Codec::UTF16BE && string.size() % 2 == 1) {
        // Recover badly encoded UTF-16 strings that have an odd number of bytes
        bool odd_zero_even_ascii = true;
        for (std::size_t i = 0; i < string.size(); ++i) {
            const auto b = static_cast<unsigned char>(string[i]);
            if (i % 2 ? b != 0 : !isascii(b)) {
                odd_zero_even_ascii = false;
                break;
            }
        }
        bool first_zero_rest_ascii = string[0] == 0;
        for (std::size_t i = 1; first_zero_rest_ascii && i < string.size(); ++i) {
            if (!isascii(static_cast<unsigned char>(string[i]))) first_zero_rest_ascii = false;
        }
        if (string.back() == 0) {
            string.pop_back();
        } else if (odd_zero_even_ascii) {
            string.insert(string.begin(), '\0');
        } else if (first_zero_rest_ascii) {
            std::string fixed;
            for (std::size_t i = 1; i < string.size(); ++i) {
                fixed.push_back('\0');
                fixed.push_back(string[i]);
            }
            string = fixed;
        }
    }
    std::optional<std::u32string> decoded = decode(string, codec);
    if (!decoded) return std::nullopt;
    // A decoded string that still looks like UTF-16BE was encoded twice
    bool looks_utf16 = true;
    for (std::size_t i = 0; i < decoded->size(); ++i) {
        const char32_t c = (*decoded)[i];
        if (i % 2 == 0 ? c != 0 : !isascii(c)) {
            looks_utf16 = false;
            break;
        }
    }
    if (looks_utf16) {
        std::u32string narrowed;
        for (std::size_t i = 1; i < decoded->size(); i += 2) narrowed.push_back((*decoded)[i]);
        decoded = narrowed;
    }
    return decoded;
}

SfntFont read_font(const std::string& path) {
    const std::string data = read_file(path);
    const TableDirectory tables = read_directory(data, -1);
    return read_tables(data, tables);
}

std::vector<SfntFont> read_font_collection(const std::string& path) {
    const std::string data = read_file(path);
    const TTCHeader header = read_ttc_header(data);
    const std::size_t num_fonts = header.has_table2 ? header.offsets2.size() : header.offsets.size();
    // TTCollection reads every font's directory, then build123d reads their tables
    std::vector<TableDirectory> directories;
    for (std::size_t i = 0; i < num_fonts; ++i) directories.push_back(read_directory(data, static_cast<long>(i)));
    std::vector<SfntFont> fonts;
    for (const auto& tables : directories) fonts.push_back(read_tables(data, tables));
    return fonts;
}

std::vector<std::u32string> py_split(const std::u32string& text) {
    std::vector<std::u32string> out;
    std::u32string current;
    for (const char32_t c : text) {
        if (is_py_space(c)) {
            if (!current.empty()) out.push_back(current);
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    if (!current.empty()) out.push_back(current);
    return out;
}

std::u32string py_strip(const std::u32string& text) {
    std::size_t begin = 0, end = text.size();
    while (begin < end && is_py_space(text[begin])) ++begin;
    while (end > begin && is_py_space(text[end - 1])) --end;
    return text.substr(begin, end - begin);
}

std::string to_utf8(const std::u32string& text) {
    std::string out;
    for (const char32_t c : text) {
        if (c < 0x80) {
            out.push_back(static_cast<char>(c));
        } else if (c < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (c >> 6)));
            out.push_back(static_cast<char>(0x80 | (c & 0x3F)));
        } else if (c < 0x10000) {
            out.push_back(static_cast<char>(0xE0 | (c >> 12)));
            out.push_back(static_cast<char>(0x80 | ((c >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (c & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xF0 | (c >> 18)));
            out.push_back(static_cast<char>(0x80 | ((c >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((c >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (c & 0x3F)));
        }
    }
    return out;
}

}  // namespace build123d::detail
