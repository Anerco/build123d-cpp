// The operation cache's folder store (docs/operation-cache.md, "The store"): an entry per
// file, written whole under a temporary name and linked into place, so the first writer
// wins and a reader sees no file or a whole one; the least recently used removed past the
// budget. POSIX only.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#define XXH_INLINE_ALL
#include "xxhash.h"

#include "build123d/operation_cache.hpp"

namespace build123d {

namespace {

constexpr char kMagic[8] = {'B', '1', '2', '3', 'D', 'O', 'C', '\0'};
constexpr std::uint32_t kFileFormat = 1;
constexpr std::size_t kKeySize = 32;
constexpr std::size_t kHeaderSize = sizeof kMagic + sizeof(std::uint32_t) + kKeySize + sizeof(std::uint64_t) + 16;
/// A hit refreshes an entry's time (what "least recently used" reads) when it is older than this.
constexpr long kTouchAfterSeconds = 60;
/// Temporaries a crashed writer left are removed once this old.
constexpr long kStaleTemporarySeconds = 3600;

bool make_folders(const std::string& path) {
    std::string at;
    for (std::size_t i = 0; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') {
            if (!at.empty() && mkdir(at.c_str(), 0700) != 0 && errno != EEXIST) return false;
        }
        if (i < path.size()) at.push_back(path[i]);
    }
    return true;
}

bool write_all(int fd, const char* data, std::size_t size) {
    while (size > 0) {
        const ssize_t n = write(fd, data, size);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        data += n;
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

bool hex_key(const std::string& key) {
    return key.size() == kKeySize &&
           std::all_of(key.begin(), key.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

class FolderStore : public OperationStore {
public:
    FolderStore(std::string folder, std::uint64_t budget) : folder_(std::move(folder)), budget_(budget) {
        while (folder_.size() > 1 && folder_.back() == '/') folder_.pop_back();
        if (!make_folders(folder_)) throw std::runtime_error("operation cache: the folder " + folder_ + " can't be made");
        prune();
    }

    std::optional<std::vector<std::uint8_t>> get(const std::string& key) override {
        if (!hex_key(key)) return std::nullopt;
        const std::string path = path_of(key);
        const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) return std::nullopt;
        struct stat st {};
        std::vector<char> bytes;
        bool ok = fstat(fd, &st) == 0 && st.st_size >= static_cast<off_t>(kHeaderSize);
        if (ok) {
            bytes.resize(static_cast<std::size_t>(st.st_size));
            std::size_t got = 0;
            while (ok && got < bytes.size()) {
                const ssize_t n = read(fd, bytes.data() + got, bytes.size() - got);
                if (n < 0 && errno == EINTR) continue;
                if (n <= 0) ok = false;
                else got += static_cast<std::size_t>(n);
            }
        }
        std::optional<std::vector<std::uint8_t>> payload;
        if (ok) payload = check(key, bytes);
        if (payload) {
            const auto now = std::chrono::system_clock::now().time_since_epoch();
            if (std::chrono::duration_cast<std::chrono::seconds>(now).count() - st.st_mtim.tv_sec > kTouchAfterSeconds) {
                futimens(fd, nullptr);
            }
        }
        close(fd);
        if (!payload) unlink(path.c_str());  // torn or not ours: a miss, and gone
        return payload;
    }

    bool put(const std::string& key, const std::vector<std::uint8_t>& entry) override {
        if (!hex_key(key)) return false;
        const std::string path = path_of(key);
        if (access(path.c_str(), F_OK) == 0) return false;
        const std::string dir = folder_ + "/" + key.substr(0, 2);
        if (!make_folders(dir)) return false;
        // a name of this process's and thread's own: two threads writing one key each write
        // their own temporary, and the first linked stays
        static std::atomic<std::uint64_t> temporaries{0};
        const std::string temporary = dir + "/." + key + "." + std::to_string(getpid()) + "." +
                                      std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())) + "." +
                                      std::to_string(temporaries.fetch_add(1)) + ".tmp";
        const int fd = open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
        if (fd < 0) return false;
        std::string header(kMagic, sizeof kMagic);
        const std::uint32_t format = kFileFormat;
        header.append(reinterpret_cast<const char*>(&format), sizeof format);
        header += key;
        const std::uint64_t size = entry.size();
        header.append(reinterpret_cast<const char*>(&size), sizeof size);
        const XXH128_hash_t sum = XXH3_128bits(entry.data(), entry.size());
        header.append(reinterpret_cast<const char*>(&sum.low64), sizeof sum.low64);
        header.append(reinterpret_cast<const char*>(&sum.high64), sizeof sum.high64);
        const bool written = write_all(fd, header.data(), header.size()) &&
                             write_all(fd, reinterpret_cast<const char*>(entry.data()), entry.size());
        close(fd);
        // link() never replaces a name: the first entry kept for a key stays.
        const bool linked = written && link(temporary.c_str(), path.c_str()) == 0;
        unlink(temporary.c_str());
        if (linked) after_writing(header.size() + entry.size());
        return linked;
    }

private:
    std::string folder_;
    std::uint64_t budget_;

    std::string path_of(const std::string& key) const { return folder_ + "/" + key.substr(0, 2) + "/" + key; }

    static std::optional<std::vector<std::uint8_t>> check(const std::string& key, const std::vector<char>& bytes) {
        const char* p = bytes.data();
        if (std::memcmp(p, kMagic, sizeof kMagic) != 0) return std::nullopt;
        p += sizeof kMagic;
        std::uint32_t format;
        std::memcpy(&format, p, sizeof format);
        p += sizeof format;
        if (format != kFileFormat || std::string(p, kKeySize) != key) return std::nullopt;
        p += kKeySize;
        std::uint64_t size;
        std::memcpy(&size, p, sizeof size);
        p += sizeof size;
        XXH128_hash_t sum;
        std::memcpy(&sum.low64, p, sizeof sum.low64);
        std::memcpy(&sum.high64, p + 8, sizeof sum.high64);
        p += 16;
        if (size != bytes.size() - kHeaderSize) return std::nullopt;
        const XXH128_hash_t got = XXH3_128bits(p, size);
        if (got.low64 != sum.low64 || got.high64 != sum.high64) return std::nullopt;
        return std::vector<std::uint8_t>(p, p + size);
    }

    /// Prune about once per sixteenth of the budget written, counted across every process
    /// that writes the folder: each write prunes with the chance its share gives.
    void after_writing(std::size_t bytes) {
        thread_local std::mt19937_64 random_source;
        thread_local pid_t seeded_by = 0;
        if (seeded_by != getpid()) {
            seeded_by = getpid();
            random_source.seed(static_cast<std::uint64_t>(seeded_by) * 0x9e3779b97f4a7c15ull ^
                               static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count()));
        }
        const double share = static_cast<double>(bytes) / (static_cast<double>(budget_) / 16.0);
        if (std::uniform_real_distribution<double>(0.0, 1.0)(random_source) < share) prune();
    }

    void prune() const {
        const std::string lock_path = folder_ + "/.prune.lock";
        const int lock = open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
        if (lock < 0) return;
        if (flock(lock, LOCK_EX | LOCK_NB) != 0) {  // another process is pruning
            close(lock);
            return;
        }
        struct File {
            std::string path;
            std::uint64_t size;
            long long time;
        };
        std::vector<File> files;
        std::uint64_t total = 0;
        const long long now = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
        if (DIR* top = opendir(folder_.c_str())) {
            while (dirent* d = readdir(top)) {
                if (std::strlen(d->d_name) != 2 || d->d_name[0] == '.') continue;
                const std::string dir = folder_ + "/" + d->d_name;
                DIR* sub = opendir(dir.c_str());
                if (!sub) continue;
                while (dirent* f = readdir(sub)) {
                    const std::string name = f->d_name;
                    if (name == "." || name == "..") continue;
                    const std::string path = dir + "/" + name;
                    struct stat st {};
                    if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
                    if (name[0] == '.') {
                        if (now - st.st_mtim.tv_sec > kStaleTemporarySeconds) unlink(path.c_str());
                        continue;
                    }
                    files.push_back({path, static_cast<std::uint64_t>(st.st_size), static_cast<long long>(st.st_mtim.tv_sec)});
                    total += static_cast<std::uint64_t>(st.st_size);
                }
                closedir(sub);
            }
            closedir(top);
        }
        if (total > budget_) {
            std::sort(files.begin(), files.end(), [](const File& a, const File& b) { return a.time < b.time; });
            const std::uint64_t target = budget_ / 10 * 9;
            for (const File& f : files) {
                if (total <= target) break;
                if (unlink(f.path.c_str()) == 0) total -= f.size;
            }
        }
        flock(lock, LOCK_UN);
        close(lock);
    }
};

}  // namespace

std::shared_ptr<OperationStore> folder_operation_store(const std::string& folder, std::uint64_t budget_bytes) {
    return std::make_shared<FolderStore>(folder, budget_bytes);
}

}  // namespace build123d
