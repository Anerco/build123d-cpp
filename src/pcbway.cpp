// export_to_pcbway (build123d/exporters3d.py): STEP to a temporary file, a multipart upload
// like requests.post(url, files={"file": (name, f, "application/step")}, timeout=(10, 120)),
// and the same checks of PCBWay's JSON answer.
#include "build123d/pcbway.hpp"

#include <cstdio>
#include <filesystem>
#include <mutex>
#include <random>
#include <stdexcept>

#include "build123d/io.hpp"
#include "build123d/json.hpp"
#include "build123d/warnings.hpp"

#if defined(BUILD123D_HAVE_CURL)
#include <curl/curl.h>
#endif

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#else
#include <spawn.h>
#include <sys/wait.h>
extern char** environ;
#endif

namespace build123d {

namespace {

// tempfile.NamedTemporaryFile(suffix=".step", delete=False): a new, empty "tmpXXXXXXXX.step".
std::filesystem::path make_temp_step() {
    static const char* const chars = "abcdefghijklmnopqrstuvwxyz0123456789_";
    std::random_device device;
    std::mt19937 generator(device());
    std::uniform_int_distribution<int> pick(0, 36);
    const auto dir = std::filesystem::temp_directory_path();
    for (int attempt = 0; attempt < 100; ++attempt) {
        std::string name = "tmp";
        for (int i = 0; i < 8; ++i) name += chars[pick(generator)];
        const auto path = dir / (name + ".step");
        if (std::FILE* file = std::fopen(path.string().c_str(), "wbx")) {
            std::fclose(file);
            return path;
        }
    }
    throw std::runtime_error("No usable temporary file name found");
}

// Python's repr() of a JSON value, for the error message that shows PCBWay's answer.
std::string python_repr(const Json& value) {
    switch (value.type()) {
        case Json::Type::NUL: return "None";
        case Json::Type::BOOL: return value.as_bool() ? "True" : "False";
        case Json::Type::INT:
        case Json::Type::FLOAT: return value.dump();
        case Json::Type::STRING: {
            std::string out = "'";
            for (const char c : value.as_string()) {
                if (c == '\\' || c == '\'') out += '\\';
                out += c;
            }
            return out + "'";
        }
        case Json::Type::ARRAY: {
            std::string out = "[";
            bool first = true;
            for (const auto& item : value.as_array()) {
                out += (first ? "" : ", ") + python_repr(item);
                first = false;
            }
            return out + "]";
        }
        case Json::Type::OBJECT: {
            std::string out = "{";
            bool first = true;
            for (const auto& [key, item] : value.as_object()) {
                out += (first ? "'" : ", '") + key + "': " + python_repr(item);
                first = false;
            }
            return out + "}";
        }
    }
    return value.dump();
}

struct Response {
    long status = 0;
    std::string reason;
    std::string body;
};

#if defined(BUILD123D_HAVE_CURL)
std::size_t collect_body(char* data, std::size_t size, std::size_t count, void* user) {
    static_cast<std::string*>(user)->append(data, size * count);
    return size * count;
}

// Keep the reason phrase of the last status line (after redirects).
std::size_t collect_status(char* data, std::size_t size, std::size_t count, void* user) {
    const std::string line(data, size * count);
    if (line.rfind("HTTP/", 0) == 0) {
        auto& reason = *static_cast<std::string*>(user);
        const auto code_start = line.find(' ');
        const auto reason_start = code_start == std::string::npos ? std::string::npos : line.find(' ', code_start + 1);
        reason = reason_start == std::string::npos ? "" : line.substr(reason_start + 1);
        while (!reason.empty() && (reason.back() == '\r' || reason.back() == '\n')) reason.pop_back();
    }
    return size * count;
}
#endif

Response post_step_file(const std::filesystem::path& file, const PcbwayUpload& upload) {
#if defined(BUILD123D_HAVE_CURL)
    static std::once_flag init;
    std::call_once(init, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
    CURL* curl = curl_easy_init();
    if (curl == nullptr) throw std::runtime_error("Couldn't start an HTTP request");
    Response response;
    curl_mime* mime = curl_mime_init(curl);
    curl_mimepart* part = curl_mime_addpart(mime);
    curl_mime_name(part, "file");
    curl_mime_filedata(part, file.string().c_str());
    curl_mime_filename(part, file.filename().string().c_str());
    curl_mime_type(part, "application/step");
    // No "Expect: 100-continue" round trip (requests doesn't use one either)
    curl_slist* headers = curl_slist_append(nullptr, "Expect:");
    curl_easy_setopt(curl, CURLOPT_URL, upload.upload_url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "build123d-cpp");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, upload.connect_timeout);
    // requests' read timeout: give up when no data arrives for read_timeout seconds
    curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
    curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, upload.read_timeout);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, collect_body);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, collect_status);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &response.reason);
    const CURLcode code = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status);
    curl_slist_free_all(headers);
    curl_mime_free(mime);
    curl_easy_cleanup(curl);
    if (code != CURLE_OK) {
        throw std::runtime_error("Upload to " + upload.upload_url + " failed: " + curl_easy_strerror(code));
    }
    return response;
#else
    (void)file;
    (void)upload;
    throw std::runtime_error("export_to_pcbway needs build123d built with libcurl");
#endif
}

}  // namespace

bool open_in_browser(const std::string& url) {
    // Only web pages; the URL is passed as one argument, never through a shell.
    if (url.rfind("http://", 0) != 0 && url.rfind("https://", 0) != 0) return false;
#if defined(_WIN32)
    const int wide_size = MultiByteToWideChar(CP_UTF8, 0, url.c_str(), -1, nullptr, 0);
    std::wstring wide(static_cast<std::size_t>(wide_size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, url.c_str(), -1, wide.data(), wide_size);
    return reinterpret_cast<INT_PTR>(ShellExecuteW(nullptr, L"open", wide.c_str(), nullptr, nullptr, SW_SHOWNORMAL)) > 32;
#else
#if defined(__APPLE__)
    const char* program = "open";
#else
    const char* program = "xdg-open";
#endif
    std::string arg0 = program, arg1 = url;
    char* argv[] = {arg0.data(), arg1.data(), nullptr};
    pid_t pid = 0;
    if (posix_spawnp(&pid, program, nullptr, nullptr, argv, environ) != 0) return false;
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return false;
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
#endif
}

std::string export_to_pcbway(const Shape& to_export, Unit unit, bool write_pcurves, PrecisionMode precision_mode) {
    return export_to_pcbway(to_export, PcbwayUpload{}, unit, write_pcurves, precision_mode);
}

std::string export_to_pcbway(const Shape& to_export, const PcbwayUpload& upload, Unit unit, bool write_pcurves,
                             PrecisionMode precision_mode) {
    std::string redirect_url;
    // Export the part to a temp file
    std::filesystem::path step_file;
    try {
        step_file = make_temp_step();
        export_step(to_export, step_file.string(), unit, write_pcurves, precision_mode);
        // Transfer the step file
        const Response response = post_step_file(step_file, upload);
        // response.raise_for_status()
        if (response.status >= 400) {
            const char* kind = response.status < 500 ? "Client" : "Server";
            throw std::runtime_error(std::to_string(response.status) + " " + kind + " Error: " + response.reason +
                                     " for url: " + upload.upload_url);
        }
        Json result;
        try {
            result = Json::parse(response.body);
        } catch (const std::exception&) {
            throw std::runtime_error("PCBWay returned a non-JSON response: " + response.body.substr(0, 500));
        }
        const bool is_object = result.is_object();
        const bool success = is_object && result.contains("state") && result["state"].is_string() &&
                             result["state"].as_string() == "SUCCESS";
        if (!success || !result.contains("redirect") || !result["redirect"].is_string()) {
            throw std::runtime_error("PCBWay upload failed or returned no redirect: " + python_repr(result));
        }
        redirect_url = result["redirect"].as_string();
    } catch (...) {
        // Always remove the tmp file
        std::error_code ignored;
        if (!step_file.empty()) std::filesystem::remove(step_file, ignored);
        throw;
    }
    std::error_code ignored;
    std::filesystem::remove(step_file, ignored);

    const bool opened = upload.open_browser ? upload.open_browser(redirect_url) : open_in_browser(redirect_url);
    if (!opened) warn("webbrowser failed to open on pricing page " + redirect_url);
    return redirect_url;
}

}  // namespace build123d
