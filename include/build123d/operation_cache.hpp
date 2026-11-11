// build123d-cpp: the operation cache. Each expensive operation (booleans, fillets, sweeps,
// lofts, offsets, meshing, ...) is looked up by a key of what it reads (its arguments and
// its inputs' content) and, on a hit, read back instead of computed: the same shape,
// sharing the same sub-shapes and geometry with the inputs, the inputs changed in place as
// OpenCascade changes them, the same warnings and history. Off unless turned on:
//
//   build123d::enable_operation_cache({.folder = "/var/cache/build123d", .budget_bytes = 2ull << 30});
//
// docs/operation-cache.md has the design: the keys, what an entry holds, how a hit is held
// to what computing gives, the store.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace build123d {

/// Where entries are kept. A store is shared by every thread of the process (and by the
/// children it forks), so its methods must be thread-safe.
class OperationStore {
public:
    virtual ~OperationStore() = default;
    /// The entry kept under `key` (32 hex digits), if there is one and it is whole.
    virtual std::optional<std::vector<std::uint8_t>> get(const std::string& key) = 0;
    /// Keep `entry` under `key`, unless there is one already: the first kept stays.
    /// Returns whether it was written.
    virtual bool put(const std::string& key, const std::vector<std::uint8_t>& entry) = 0;
};

/// The folder store: one file per entry under `folder`, written whole under a temporary
/// name and linked into place (the first writer wins), safe for many processes at once;
/// the least recently used entries are removed once the folder holds more than
/// `budget_bytes`. The folder is created if missing.
std::shared_ptr<OperationStore> folder_operation_store(const std::string& folder, std::uint64_t budget_bytes);

/// What one lookup did (OperationCacheOptions::on_lookup).
struct OperationCacheEvent {
    std::string operation;   ///< e.g. "bool", "fillet", "sweep", "mesh"
    std::string key;         ///< 32 hex digits
    bool hit = false;
    bool stored = false;     ///< a miss whose result was kept
    std::string rejected;    ///< why a miss's result was not kept ("" if kept or under the threshold)
    double ms = 0;           ///< the lookup's whole time: reading on a hit, computing (and keeping) on a miss
    double computed_ms = 0;  ///< what computing took (on a hit, what it took when it was kept)
};

struct OperationCacheOptions {
    /// The folder of a folder store (see folder_operation_store).
    std::string folder;
    std::uint64_t budget_bytes = 1ull << 30;
    /// Keep the results of operations that took at least this long to compute.
    double min_ms = 5.0;
    /// Called after each lookup, on the thread that made it.
    std::function<void(const OperationCacheEvent&)> on_lookup;
};

/// Turn the cache on with a folder store; it stays on for the process and the children it forks.
void enable_operation_cache(const OperationCacheOptions& options);
/// Turn it on with a store of one's own (nullptr turns it off).
void enable_operation_cache(std::shared_ptr<OperationStore> store, const OperationCacheOptions& options = {});
void disable_operation_cache();
bool operation_cache_enabled();

/// Counts and times of this process's lookups since the cache was turned on (or reset).
struct OperationCacheStats {
    struct Row {
        std::string operation;
        std::uint64_t lookups = 0, hits = 0, misses = 0, stored = 0, rejected = 0;
        double computed_ms = 0;  ///< time computing misses
        double hit_ms = 0;       ///< time reading hits
        double saved_ms = 0;     ///< on hits, what computing took when kept, less what reading took
    };
    std::uint64_t lookups = 0, hits = 0, misses = 0, stored = 0, rejected = 0;
    /// Operations computed without a lookup: their inputs hold something the cache can't
    /// rebuild (a representation type it doesn't know).
    std::uint64_t bypassed = 0;
    double computed_ms = 0, hit_ms = 0, saved_ms = 0;
    double key_ms = 0;    ///< writing and hashing the inputs, on every lookup
    double store_ms = 0;  ///< encoding, checking and writing entries
    std::vector<Row> by_operation;
    /// Why results were not kept, with counts ("not reproduced", "history", ...).
    std::vector<std::pair<std::string, std::uint64_t>> rejections;
};
OperationCacheStats operation_cache_stats();
void reset_operation_cache_stats();

/// BRepMesh_IncrementalMesh(shape, linear_deflection, relative, angular_deflection, parallel),
/// through the cache: a host that meshes a build's shapes itself keeps their triangles across
/// builds as the library's own meshing does (Shape::mesh, tessellate, the exporters).
void incremental_mesh(const TopoDS_Shape& shape, double linear_deflection, bool relative, double angular_deflection,
                      bool parallel);

}  // namespace build123d
