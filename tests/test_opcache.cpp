// Unit tests for the operation cache (operation_cache.hpp, docs/operation-cache.md): a hit
// gives what computing gave, down to the bytes of the results written with their inputs.
#include <build123d/build123d.hpp>

#include <BinTools_ShapeSet.hxx>
#include <TopLoc_Datum3D.hxx>
#include <TopoDS_Iterator.hxx>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/opcache.hpp"
#include "gate.hpp"
#include "test_framework.hpp"

using namespace build123d;
namespace fs = std::filesystem;

namespace {

/// A cache in a folder of its own for one test, keeping everything (min_ms 0). The cache
/// the gate turned on (tests/gate) is turned on again after.
struct TestCache {
    fs::path folder;
    explicit TestCache(const std::string& name, std::uint64_t budget = 1ull << 30) {
        folder = fs::temp_directory_path() / ("build123d_test_opcache_" + name + "_" + std::to_string(::getpid()));
        fs::remove_all(folder);
        enable_operation_cache({.folder = folder.string(), .budget_bytes = budget, .min_ms = 0});
        reset_operation_cache_stats();
    }
    ~TestCache() {
        disable_operation_cache();
        fs::remove_all(folder);
        gate::enable_from_environment();
    }
};

/// The shapes written together, as one byte string: equal bytes mean the same geometry,
/// tolerances, flags, triangulations, sub-shape order, and the same sharing among them.
std::string together(const std::vector<TopoDS_Shape>& shapes) {
    BinTools_ShapeSet set;
    set.SetWithTriangles(true);
    set.SetWithNormals(true);
    for (const auto& s : shapes) set.Add(s);
    std::ostringstream os;
    set.Write(os);
    for (const auto& s : shapes) set.Write(s, os);
    return os.str();
}

std::size_t shared_tshapes(const TopoDS_Shape& result, const std::vector<TopoDS_Shape>& inputs) {
    std::set<const void*> in;
    const std::function<void(const TopoDS_Shape&, std::set<const void*>&)> collect = [&](const TopoDS_Shape& s,
                                                                                         std::set<const void*>& out) {
        if (!out.insert(s.TShape().get()).second) return;
        for (TopoDS_Iterator it(s, false, false); it.More(); it.Next()) collect(it.Value(), out);
    };
    for (const auto& s : inputs) collect(s, in);
    std::set<const void*> mine;
    collect(result, mine);
    std::size_t n = 0;
    for (const void* p : mine) n += in.count(p);
    return n;
}

struct Made {
    std::vector<TopoDS_Shape> inputs;
    Part result;
};

/// A box with a hole: built anew each call, so a second call is a hit with other live inputs.
Made box_minus_cylinder(double radius = 3) {
    Part a = Box(10, 10, 10);
    Part b = Pos(2, 1, 0) * Cylinder(radius, 20);
    Made m{{a.wrapped(), b.wrapped()}, a - b};
    return m;
}

}  // namespace

TEST(opcache_hit_gives_what_computing_gave) {
    const TestCache cache("hit");
    const Made cold = box_minus_cylinder();
    CHECK(operation_cache_stats().hits == 0);
    CHECK(operation_cache_stats().stored >= 1);
    const Made warm = box_minus_cylinder();
    CHECK(operation_cache_stats().hits >= 1);
    // the results with their inputs: the same bytes, so the same sharing with the inputs
    CHECK(together(cold.inputs) + together({cold.result.wrapped()}) == together(warm.inputs) + together({warm.result.wrapped()}));
    std::vector<TopoDS_Shape> a = cold.inputs, b = warm.inputs;
    a.push_back(cold.result.wrapped());
    b.push_back(warm.result.wrapped());
    CHECK(together(a) == together(b));
    CHECK(shared_tshapes(cold.result.wrapped(), cold.inputs) > 0);
    CHECK(shared_tshapes(warm.result.wrapped(), warm.inputs) == shared_tshapes(cold.result.wrapped(), cold.inputs));
    CHECK_NEAR(warm.result.volume(), cold.result.volume(), 0);
    CHECK_NEAR(warm.result.area(), cold.result.area(), 0);
    // the sub-shapes in the same order: a selection by index selects the same face
    const auto cold_faces = cold.result.faces(), warm_faces = warm.result.faces();
    CHECK(cold_faces.size() == warm_faces.size());
    for (std::size_t i = 0; i < cold_faces.size(); ++i) {
        CHECK(cold_faces[i].center() == warm_faces[i].center());
        CHECK_NEAR(cold_faces[i].area(), warm_faces[i].area(), 0);
    }
    CHECK(cold.result.edges().size() == warm.result.edges().size());
    CHECK(cold.result.vertices().size() == warm.result.vertices().size());
}

TEST(opcache_hit_is_a_copy) {
    // Each hit decodes a copy of its own: changing what one hit gave (meshing it, a destructive
    // boolean on it, as OCCT's are) changes neither the entry nor what the next hit gives.
    const TestCache cache("copy");
    const std::string computed = together({box_minus_cylinder().result.wrapped()});
    Part first = box_minus_cylinder().result;
    CHECK(operation_cache_stats().hits >= 1);
    first.mesh(0.01);
    (void)(first + Pos(5, 0, 0) * Box(10, 10, 10));
    CHECK(together({first.wrapped()}) != computed);
    const Part second = box_minus_cylinder().result;
    CHECK(together({second.wrapped()}) == computed);
}

TEST(opcache_keeps_the_history) {
    const TestCache cache("history");
    const Made cold = box_minus_cylinder();
    const Made warm = box_minus_cylinder();
    CHECK(operation_cache_stats().hits >= 1);
    // the hole's face was brought in by the cylinder (LAST); the edges where it meets the
    // box exist in neither input (NEW)
    for (Select select : {Select::NEW, Select::LAST}) {
        const auto c = cold.result.edges(select), w = warm.result.edges(select);
        CHECK(!c.empty());
        CHECK(c.size() == w.size());
        for (std::size_t i = 0; i < c.size(); ++i) CHECK(c[i].center() == w[i].center());
        CHECK(cold.result.faces(select).size() == warm.result.faces(select).size());
    }
    CHECK(!warm.result.faces(Select::LAST).empty());
    // what the history says of the inputs' own sub-shapes, too
    const auto& history = warm.result.history();
    CHECK(history != nullptr);
    std::size_t modified = 0;
    for (const auto& f : Shape(warm.inputs[0]).faces()) modified += history->modified(f.wrapped()).size();
    std::size_t cold_modified = 0;
    for (const auto& f : Shape(cold.inputs[0]).faces()) cold_modified += cold.result.history()->modified(f.wrapped()).size();
    CHECK(modified == cold_modified);
}

TEST(opcache_changes_inputs_as_computing_does) {
    // A fuse of boxes sharing a face adds p-curves to the boxes' edges in place.
    const TestCache cache("mutation");
    const auto make = [] {
        Part a = Box(10, 10, 10);
        Part b = Pos(5, 0, 0) * Box(10, 10, 10);
        const std::string before = together({a.wrapped(), b.wrapped()});
        Part c = a + b;
        return std::make_tuple(before, together({a.wrapped(), b.wrapped()}), together({a.wrapped(), b.wrapped(), c.wrapped()}));
    };
    const auto [before, cold_after, cold_all] = make();
    CHECK(before != cold_after);
    const auto [before2, warm_after, warm_all] = make();
    CHECK(operation_cache_stats().hits >= 1);
    CHECK(warm_after == cold_after);
    CHECK(warm_all == cold_all);
}

TEST(opcache_replays_warnings) {
    const TestCache cache("warnings");
    std::vector<std::string> said;
    const WarningHandler previous = set_warning_handler([&](const std::string& m) { said.push_back(m); });
    const auto op = [] {
        const Part box = Box(1, 2, 3);
        return detail::opcache::run(detail::opcache::Args("test_warns"), {box.wrapped()}, [&] {
            warn("first");
            warn("second");
            return detail::opcache::Outcome{{box.wrapped()}, nullptr};
        });
    };
    op();
    op();
    set_warning_handler(previous);
    CHECK(operation_cache_stats().hits == 1);
    CHECK((said == std::vector<std::string>{"first", "second", "first", "second"}));
}

TEST(opcache_misses_when_an_argument_changes) {
    const TestCache cache("arguments");
    box_minus_cylinder(3);
    const auto stored = operation_cache_stats().stored;
    box_minus_cylinder(3.5);
    CHECK(operation_cache_stats().hits == 0);
    CHECK(operation_cache_stats().stored > stored);
    box_minus_cylinder(3);
    box_minus_cylinder(3.5);
    CHECK(operation_cache_stats().hits >= 2);
    // the same box moved is another input
    Part a = Box(10, 10, 10), b = Cylinder(3, 20);
    reset_operation_cache_stats();
    (void)(a - b);
    (void)(Pos(0, 0, 1e-9) * a - b);
    CHECK(operation_cache_stats().hits == 0);
}

TEST(opcache_keys_skip_padding) {
    // A gp_Trsf has padding between its form and its matrix, whatever was in memory: two equal
    // locations whose padding differs are one key.
    const TestCache cache("padding");
    const auto located = [](unsigned char fill) {
        gp_Trsf t;
        t.SetTranslation(gp_Vec(1, 2, 3));
        const TopLoc_Location loc(t);
        // the padding of the transformation the location holds, after its scale and form
        const gp_Trsf& held = loc.FirstDatum()->Transformation();
        char* at = const_cast<char*>(reinterpret_cast<const char*>(&held));
        const std::size_t from = sizeof(double) + sizeof(gp_TrsfForm);
        const std::size_t to = static_cast<std::size_t>(reinterpret_cast<const char*>(&held.HVectorialPart()) - at);
        if (to > from) std::memset(at + from, fill, to - from);
        Part box = Box(10, 10, 10);
        box.set_wrapped(box.wrapped().Located(loc));
        return box;
    };
    const Part tool = Cylinder(3, 20);
    (void)(located(0xAB) - tool);
    (void)(located(0x00) - tool);
    CHECK(operation_cache_stats().hits == 1);
}

TEST(opcache_fillet_selects_the_same_edges) {
    const TestCache cache("fillet");
    const auto make = [](int which) {
        Part box = Box(10, 10, 10);
        const auto edges = box.edges().filter_by(Axis::Z);
        return fillet(edges.slice(which, which + 2), 1);
    };
    const Part cold = make(0);
    const Part warm = make(0);
    CHECK(operation_cache_stats().hits >= 1);
    CHECK(together({cold.wrapped()}) == together({warm.wrapped()}));
    const auto hits = operation_cache_stats().hits;
    const Part other = make(1);  // other edges of the same box: another key
    CHECK(operation_cache_stats().hits == hits);
    CHECK(together({other.wrapped()}) != together({cold.wrapped()}));
}

TEST(opcache_mesh_is_kept) {
    const TestCache cache("mesh");
    const auto make = [] {
        Part p = Sphere(5) - Box(4, 4, 20);
        return p.tessellate(0.01, 0.1);
    };
    const auto cold = make();
    const auto warm = make();
    CHECK(operation_cache_stats().hits >= 2);
    CHECK(cold.first.size() == warm.first.size());
    CHECK(cold.second == warm.second);
    for (std::size_t i = 0; i < cold.first.size(); ++i) {
        CHECK(cold.first[i].X == warm.first[i].X && cold.first[i].Y == warm.first[i].Y && cold.first[i].Z == warm.first[i].Z);
    }
}

TEST(opcache_folder_store) {
    const fs::path folder = fs::temp_directory_path() / ("build123d_test_opcache_store_" + std::to_string(::getpid()));
    fs::remove_all(folder);
    {
        const auto store = folder_operation_store(folder.string(), 1ull << 30);
        const std::string key(32, 'a');
        CHECK(!store->get(key));
        CHECK(store->put(key, {1, 2, 3}));
        CHECK(!store->put(key, {4, 5}));  // the first kept stays
        CHECK((*store->get(key) == std::vector<std::uint8_t>{1, 2, 3}));
        CHECK(!store->put("not a key", {1}));
        // a torn entry is a miss, and is removed
        const fs::path file = folder / "aa" / key;
        fs::resize_file(file, fs::file_size(file) - 1);
        CHECK(!store->get(key));
        CHECK(!fs::exists(file));
    }
    fs::remove_all(folder);
    {
        // past the budget the oldest go: 64 entries of 1 KB in a budget of 16 KB
        const auto store = folder_operation_store(folder.string(), 16 * 1024);
        for (int i = 0; i < 64; ++i) {
            char key[33];
            std::snprintf(key, sizeof key, "%032x", i);
            store->put(key, std::vector<std::uint8_t>(1024, static_cast<std::uint8_t>(i)));
        }
        std::uintmax_t total = 0;
        for (const auto& e : fs::recursive_directory_iterator(folder)) {
            if (e.is_regular_file() && e.path().filename().string()[0] != '.') total += e.file_size();
        }
        CHECK(total <= 16 * 1024 + 2 * 1100);
        CHECK(total > 0);
    }
    fs::remove_all(folder);
}

TEST(opcache_threads) {
    // Builds on threads of their own at once, as a program's parts built concurrently: the
    // same keys looked up, computed and stored by eight threads together, each on its own
    // shapes. Each gives what computing gives, cold and warm.
    const auto model = [] {
        Part a = Box(10, 10, 10);
        Part b = Pos(2, 1, 0) * Cylinder(3, 20);
        Part c = a - b;
        c = fillet(c.edges().filter_by(Axis::Z), 0.5);
        Part d = c + Pos(5, 0, 0) * Box(10, 4, 4);
        return together({d.wrapped()}) + together({a.wrapped(), b.wrapped(), c.wrapped()});
    };
    disable_operation_cache();
    const std::string computed = model();
    const TestCache cache("threads");
    for (int round = 0; round < 2; ++round) {
        std::vector<std::string> got(8), failed(8);
        std::vector<std::thread> threads;
        for (std::size_t i = 0; i < got.size(); ++i) {
            threads.emplace_back([&, i] {
                try {
                    got[i] = model();
                } catch (const std::exception& e) {
                    failed[i] = e.what();
                }
            });
        }
        for (auto& t : threads) t.join();
        for (std::size_t i = 0; i < got.size(); ++i) {
            CHECK(failed[i].empty());
            CHECK(got[i] == computed);
        }
    }
    const auto stats = operation_cache_stats();
    CHECK(stats.hits >= 8 * 3);
    CHECK(stats.rejected == 0);
    std::size_t temporaries = 0;
    for (const auto& e : fs::recursive_directory_iterator(cache.folder)) {
        temporaries += e.path().filename().string().rfind(".", 0) == 0 && e.path().extension() == ".tmp";
    }
    CHECK(temporaries == 0);
}
