// The texture directory lookup (see textures.hpp), like the bundled fonts' (text.cpp).
#include "textures.hpp"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace build123d::detail {

namespace fs = std::filesystem;

namespace {

/// The directory of the library (or program) this code is in.
std::string library_dir() {
#if defined(_WIN32)
    HMODULE module = nullptr;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCSTR>(&library_dir), &module) != 0) {
        char buffer[MAX_PATH];
        const DWORD n = GetModuleFileNameA(module, buffer, MAX_PATH);
        if (n > 0) return fs::path(std::string(buffer, n)).parent_path().string();
    }
    return {};
#else
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&library_dir), &info) != 0 && info.dli_fname != nullptr &&
        fs::path(info.dli_fname).is_absolute()) {
        return fs::path(info.dli_fname).parent_path().string();
    }
#if defined(__linux__)
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) return exe.parent_path().string();
#endif
    return {};
#endif
}

}  // namespace

std::string find_texture_dir() {
    std::vector<fs::path> candidates;
    if (const char* env = std::getenv("BUILD123D_TEXTURE_DIR"); env != nullptr && *env != '\0') {
        candidates.emplace_back(env);
    }
    const std::string dir = library_dir();
    if (!dir.empty()) candidates.push_back(fs::path(dir) / ".." / "share" / "build123d" / "textures");
#ifdef BUILD123D_INSTALL_TEXTURE_DIR
    candidates.emplace_back(BUILD123D_INSTALL_TEXTURE_DIR);
#endif
#ifdef BUILD123D_SOURCE_TEXTURE_DIR
    candidates.emplace_back(BUILD123D_SOURCE_TEXTURE_DIR);
#endif
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(candidate / "_brush" / "normal.png", ec)) {
            return fs::weakly_canonical(candidate, ec).string();
        }
    }
    return {};
}

void throw_missing_textures(const std::string& file) {
    throw std::runtime_error("the texture '" + file +
                             "' is missing: the PBR textures aren't installed. Run scripts/fetch-textures.sh "
                             "(it downloads them into data/textures), or set BUILD123D_TEXTURE_DIR to a "
                             "directory with them (threejs_materials' pbr_properties/_assets)");
}

}  // namespace build123d::detail
