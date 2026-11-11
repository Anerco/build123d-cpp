// The operation cache: keys, entries, hits and the check that keeps entries exact.
// docs/operation-cache.md has the design; this file follows its sections.
#include "opcache.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#if defined(__linux__) || defined(__FreeBSD__)
#include <dlfcn.h>
#include <link.h>
#include <sys/stat.h>
#endif

#include <BRep_Curve3D.hxx>
#include <BRep_GCurve.hxx>
#include <BRep_CurveOn2Surfaces.hxx>
#include <BRep_CurveOnClosedSurface.hxx>
#include <BRep_CurveOnSurface.hxx>
#include <BRep_PointOnCurve.hxx>
#include <BRep_PointOnCurveOnSurface.hxx>
#include <BRep_PointOnSurface.hxx>
#include <BRep_Polygon3D.hxx>
#include <BRep_PolygonOnClosedSurface.hxx>
#include <BRep_PolygonOnClosedTriangulation.hxx>
#include <BRep_PolygonOnSurface.hxx>
#include <BRep_PolygonOnTriangulation.hxx>
#include <BRep_Builder.hxx>
#include <BRep_TEdge.hxx>
#include <TopoDS_Vertex.hxx>
#include <Geom2d_Conic.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_Conic.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_TFace.hxx>
#include <BRep_TVertex.hxx>
#include <BinTools_ShapeSet.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopExp.hxx>
#include <TopLoc_Datum3D.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_TCompSolid.hxx>
#include <TopoDS_TCompound.hxx>
#include <TopoDS_TShell.hxx>
#include <TopoDS_TSolid.hxx>
#include <TopoDS_TWire.hxx>

#define XXH_INLINE_ALL
#include "xxhash.h"

#include "build123d/history.hpp"
#include "build123d/operation_cache.hpp"
#include "build123d/topology.hpp"
#include "build123d/version.hpp"
#include "build123d/warnings.hpp"
#include "internal.hpp"

namespace build123d {

namespace detail::opcache {

namespace {

/// The entry format: entries of another format are never read (the format is in the key).
constexpr std::uint32_t kFormat = 1;

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

// ---------------------------------------------------------------------------
// The state: the store, the options, and the statistics
// ---------------------------------------------------------------------------

struct State {
    std::shared_ptr<OperationStore> store;
    OperationCacheOptions options;
    /// What every key starts with: the format, the library, OpenCascade and their builds.
    std::string prefix;
};

std::atomic<bool> on{false};
std::mutex state_mutex;
std::shared_ptr<const State> state;

std::shared_ptr<const State> current() {
    const std::lock_guard<std::mutex> lock(state_mutex);
    return state;
}

std::mutex stats_mutex;
OperationCacheStats stats;
std::map<std::string, std::uint64_t> rejection_counts;

OperationCacheStats::Row& row_of(const std::string& operation) {
    for (auto& row : stats.by_operation) {
        if (row.operation == operation) return row;
    }
    stats.by_operation.push_back({});
    stats.by_operation.back().operation = operation;
    return stats.by_operation.back();
}

// ---------------------------------------------------------------------------
// Bytes
// ---------------------------------------------------------------------------

struct Writer {
    std::string b;
    void u8(std::uint8_t v) { b.push_back(static_cast<char>(v)); }
    void u32(std::uint32_t v) { b.append(reinterpret_cast<const char*>(&v), sizeof v); }
    void u64(std::uint64_t v) { b.append(reinterpret_cast<const char*>(&v), sizeof v); }
    void f64(double v) { b.append(reinterpret_cast<const char*>(&v), sizeof v); }
    void str(const std::string& s) {
        u64(s.size());
        b.append(s);
    }
};

struct Reader {
    const char* p;
    const char* end;
    explicit Reader(const std::string& s) : p(s.data()), end(s.data() + s.size()) {}
    void need(std::size_t n) const {
        if (static_cast<std::size_t>(end - p) < n) throw std::runtime_error("operation cache: a short entry");
    }
    template <class T>
    T get() {
        need(sizeof(T));
        T v;
        std::memcpy(&v, p, sizeof v);
        p += sizeof v;
        return v;
    }
    std::uint8_t u8() { return get<std::uint8_t>(); }
    std::uint32_t u32() { return get<std::uint32_t>(); }
    std::uint64_t u64() { return get<std::uint64_t>(); }
    double f64() { return get<double>(); }
    std::string str() {
        const std::uint64_t n = u64();
        need(n);
        std::string s(p, n);
        p += n;
        return s;
    }
};

std::string hex(const XXH128_hash_t& h) {
    static const char* digits = "0123456789abcdef";
    std::string out(32, '0');
    for (int i = 0; i < 8; ++i) {
        const auto hi = static_cast<unsigned>((h.high64 >> (56 - 8 * i)) & 0xff);
        const auto lo = static_cast<unsigned>((h.low64 >> (56 - 8 * i)) & 0xff);
        out[2 * i] = digits[hi >> 4];
        out[2 * i + 1] = digits[hi & 15];
        out[16 + 2 * i] = digits[lo >> 4];
        out[16 + 2 * i + 1] = digits[lo & 15];
    }
    return out;
}

// ---------------------------------------------------------------------------
// Which build of the library and of OpenCascade is computing
// ---------------------------------------------------------------------------

/// The library's own object and every OpenCascade toolkit loaded, by name, size and
/// modification time: another build of either starts a namespace of its own.
std::string build_identity() {
    std::string out;
#if defined(__linux__) || defined(__FreeBSD__)
    const auto add_file = [&out](const std::string& path) {
        struct stat st {};
        if (stat(path.c_str(), &st) != 0) return;
        const std::string name = path.substr(path.rfind('/') + 1);
        out += name;
        out.push_back('\0');
        out += std::to_string(static_cast<long long>(st.st_size)) + ":" + std::to_string(static_cast<long long>(st.st_mtim.tv_sec)) +
               "." + std::to_string(static_cast<long long>(st.st_mtim.tv_nsec));
        out.push_back('\0');
    };
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&build_identity), &info) != 0 && info.dli_fname != nullptr) {
        std::string self = info.dli_fname;
        if (self.empty() || self.find('/') == std::string::npos) self = "/proc/self/exe";
        add_file(self);
    }
    std::vector<std::string> toolkits;
    dl_iterate_phdr(
        [](dl_phdr_info* phdr, std::size_t, void* data) {
            if (phdr->dlpi_name != nullptr && std::strstr(phdr->dlpi_name, "/libTK") != nullptr) {
                static_cast<std::vector<std::string>*>(data)->emplace_back(phdr->dlpi_name);
            }
            return 0;
        },
        &toolkits);
    std::sort(toolkits.begin(), toolkits.end());
    for (const auto& t : toolkits) add_file(t);
#endif
    return out;
}

std::string key_prefix() {
    Writer w;
    w.u32(kFormat);
    w.str(cpp_version);
    w.str(OCC_VERSION_COMPLETE);
    w.str(build_identity());
    return w.b;
}

// ---------------------------------------------------------------------------
// The canonical walk
// ---------------------------------------------------------------------------

/// Representation kinds, by exact class.
enum : std::uint8_t {
    kCurve3D = 1,
    kCurveOnSurface,
    kCurveOnClosedSurface,
    kCurveOn2Surfaces,
    kPolygon3D,
    kPolygonOnTriangulation,
    kPolygonOnClosedTriangulation,
    kPolygonOnSurface,
    kPolygonOnClosedSurface,
};
enum : std::uint8_t { kPointOnCurve = 1, kPointOnCurveOnSurface, kPointOnSurface };

std::uint8_t curve_kind(const Handle(BRep_CurveRepresentation)& cr) {
    const Handle(Standard_Type)& t = cr->DynamicType();
    if (t == STANDARD_TYPE(BRep_Curve3D)) return kCurve3D;
    if (t == STANDARD_TYPE(BRep_CurveOnSurface)) return kCurveOnSurface;
    if (t == STANDARD_TYPE(BRep_CurveOnClosedSurface)) return kCurveOnClosedSurface;
    if (t == STANDARD_TYPE(BRep_CurveOn2Surfaces)) return kCurveOn2Surfaces;
    if (t == STANDARD_TYPE(BRep_Polygon3D)) return kPolygon3D;
    if (t == STANDARD_TYPE(BRep_PolygonOnTriangulation)) return kPolygonOnTriangulation;
    if (t == STANDARD_TYPE(BRep_PolygonOnClosedTriangulation)) return kPolygonOnClosedTriangulation;
    if (t == STANDARD_TYPE(BRep_PolygonOnSurface)) return kPolygonOnSurface;
    if (t == STANDARD_TYPE(BRep_PolygonOnClosedSurface)) return kPolygonOnClosedSurface;
    return 0;
}

std::uint8_t point_kind(const Handle(BRep_PointRepresentation)& pr) {
    const Handle(Standard_Type)& t = pr->DynamicType();
    if (t == STANDARD_TYPE(BRep_PointOnCurve)) return kPointOnCurve;
    if (t == STANDARD_TYPE(BRep_PointOnCurveOnSurface)) return kPointOnCurveOnSurface;
    if (t == STANDARD_TYPE(BRep_PointOnSurface)) return kPointOnSurface;
    return 0;
}

TopoDS_Shape holder(const Handle(TopoDS_TShape)& t) {
    TopoDS_Shape s;
    s.TShape(t);
    s.Orientation(TopAbs_FORWARD);
    return s;
}

template <class T>
void put_raw(std::string& out, const T& v) {
    static_assert(std::is_trivially_copyable_v<T>);
    out.append(reinterpret_cast<const char*>(&v), sizeof v);
}

/// What BinTools reads back otherwise of a geometry object (and of its basis, written
/// inline): the axes it is placed by, as they are held. BinTools reads an axis system back
/// from its main and X directions, so its Y direction comes back recomputed (with the
/// signs of its zeros changed), and a direction back normalized again.
void geometry_state(const Handle(Standard_Transient)& g, std::string& out) {
    if (g.IsNull()) return;
    if (const Handle(Geom_Line) c = Handle(Geom_Line)::DownCast(g)) {
        put_raw(out, c->Position());
    } else if (const Handle(Geom_Conic) c = Handle(Geom_Conic)::DownCast(g)) {
        put_raw(out, c->Position());
    } else if (const Handle(Geom_TrimmedCurve) c = Handle(Geom_TrimmedCurve)::DownCast(g)) {
        geometry_state(c->BasisCurve(), out);
    } else if (const Handle(Geom_OffsetCurve) c = Handle(Geom_OffsetCurve)::DownCast(g)) {
        put_raw(out, c->Direction());
        geometry_state(c->BasisCurve(), out);
    } else if (const Handle(Geom_ElementarySurface) e = Handle(Geom_ElementarySurface)::DownCast(g)) {
        put_raw(out, e->Position());
    } else if (const Handle(Geom_SurfaceOfLinearExtrusion) e = Handle(Geom_SurfaceOfLinearExtrusion)::DownCast(g)) {
        put_raw(out, e->Direction());
        geometry_state(e->BasisCurve(), out);
    } else if (const Handle(Geom_SurfaceOfRevolution) e = Handle(Geom_SurfaceOfRevolution)::DownCast(g)) {
        put_raw(out, e->Axis());
        geometry_state(e->BasisCurve(), out);
    } else if (const Handle(Geom_RectangularTrimmedSurface) e = Handle(Geom_RectangularTrimmedSurface)::DownCast(g)) {
        geometry_state(e->BasisSurface(), out);
    } else if (const Handle(Geom_OffsetSurface) e = Handle(Geom_OffsetSurface)::DownCast(g)) {
        geometry_state(e->BasisSurface(), out);
    } else if (const Handle(Geom2d_Line) c = Handle(Geom2d_Line)::DownCast(g)) {
        put_raw(out, c->Position());
    } else if (const Handle(Geom2d_Conic) c = Handle(Geom2d_Conic)::DownCast(g)) {
        put_raw(out, c->Position());
    } else if (const Handle(Geom2d_TrimmedCurve) c = Handle(Geom2d_TrimmedCurve)::DownCast(g)) {
        geometry_state(c->BasisCurve(), out);
    } else if (const Handle(Geom2d_OffsetCurve) c = Handle(Geom2d_OffsetCurve)::DownCast(g)) {
        geometry_state(c->BasisCurve(), out);
    } else if (const Handle(Poly_Triangulation) t = Handle(Poly_Triangulation)::DownCast(g)) {
        put_raw(out, static_cast<std::int32_t>(t->MeshPurpose()));
    }
}

/// Every object a set of shapes is made of that can be shared (TShapes, surfaces, curves,
/// p-curves, polygons, triangulations, location datums), each once, in an order that
/// depends only on the shapes' structure; and a signature of that structure: each
/// object's kind and where it is met again, each representation's kind, each face's
/// triangulations, each TShape's lock.
class Walk {
public:
    std::vector<Handle(Standard_Transient)> objects;
    std::unordered_map<const Standard_Transient*, std::uint32_t> index;
    /// Every curve representation, in walk order (what BinTools reads back otherwise is
    /// restored on them: see State).
    std::vector<Handle(BRep_CurveRepresentation)> curves;
    /// Every location datum, in walk order.
    std::vector<Handle(TopLoc_Datum3D)> datums;
    /// Every geometry object and triangulation, in walk order.
    std::vector<Handle(Standard_Transient)> geometry;
    std::string signature;
    /// An object of a class the cache can't rebuild was met.
    bool unsupported = false;

    void root(const TopoDS_Shape& s) {
        if (s.IsNull()) {
            sig('0');
            return;
        }
        sig('R');
        sig(static_cast<std::uint32_t>(s.Orientation()));
        location(s.Location());
        tshape(s.TShape());
    }

    std::uint64_t signature_hash() const { return XXH3_64bits(signature.data(), signature.size()); }

private:
    void sig(char c) { signature.push_back(c); }
    void sig(std::uint32_t v) { signature.append(reinterpret_cast<const char*>(&v), sizeof v); }
    void sig(double v) { signature.append(reinterpret_cast<const char*>(&v), sizeof v); }
    void sig(const gp_Pnt2d& p) {
        sig(p.X());
        sig(p.Y());
    }

    /// Record the object; false if it was met before.
    bool add(const Handle(Standard_Transient)& h) {
        const auto [it, fresh] = index.emplace(h.get(), static_cast<std::uint32_t>(objects.size()));
        if (!fresh) {
            sig('#');
            sig(it->second);
            return false;
        }
        objects.push_back(h);
        sig('N');
        return true;
    }

    void object(const Handle(Standard_Transient)& h) {
        if (h.IsNull()) {
            sig('n');
            return;
        }
        if (add(h)) {
            geometry.push_back(h);
            geometry_state(h, signature);
        }
    }

    void location(const TopLoc_Location& l) {
        sig('L');
        for (TopLoc_Location it = l; !it.IsIdentity(); it = it.NextLocation()) {
            const Handle(TopLoc_Datum3D)& datum = it.FirstDatum();
            if (add(datum)) {
                // the transformation as it is held, its form too, which BinTools reads back as a
                // compound transformation (and OCCT computes differently by form); field by
                // field, as a gp_Trsf has padding between its form and its matrix
                datums.push_back(datum);
                const gp_Trsf& t = datum->Transformation();
                sig(t.ScaleFactor());
                sig(static_cast<std::uint32_t>(t.Form()));
                for (int r = 1; r <= 3; ++r) {
                    for (int c = 1; c <= 3; ++c) sig(t.HVectorialPart().Value(r, c));
                }
                for (int i = 1; i <= 3; ++i) sig(t.TranslationPart().Coord(i));
            }
            sig(static_cast<std::uint32_t>(it.FirstPower()));
        }
        sig('.');
    }

    void tshape(const Handle(TopoDS_TShape)& t) {
        if (!add(t)) return;
        sig(static_cast<std::uint32_t>(t->ShapeType()));
        sig(t->Locked() ? 'k' : 'u');
        if (const Handle(BRep_TVertex) tv = Handle(BRep_TVertex)::DownCast(t)) {
            sig('v');
            for (const auto& pr : tv->Points()) {
                const std::uint8_t kind = point_kind(pr);
                sig(static_cast<char>('0' + kind));
                switch (kind) {
                    case kPointOnCurve: object(pr->Curve()); break;
                    case kPointOnCurveOnSurface:
                        object(pr->PCurve());
                        object(pr->Surface());
                        break;
                    case kPointOnSurface: object(pr->Surface()); break;
                    default: unsupported = true;
                }
                location(pr->Location());
            }
        } else if (const Handle(BRep_TEdge) te = Handle(BRep_TEdge)::DownCast(t)) {
            sig('e');
            for (const auto& cr : te->Curves()) {
                const std::uint8_t kind = curve_kind(cr);
                sig(static_cast<char>('0' + kind));
                curves.push_back(cr);
                // the range of every curve, also a null one (BinTools leaves those out), and the
                // ends of each p-curve (which BinTools reads back recomputed)
                if (const Handle(BRep_GCurve) gc = Handle(BRep_GCurve)::DownCast(cr)) {
                    sig(gc->First());
                    sig(gc->Last());
                }
                if (const Handle(BRep_CurveOnSurface) cos = Handle(BRep_CurveOnSurface)::DownCast(cr)) {
                    gp_Pnt2d a, b;
                    cos->UVPoints(a, b);
                    sig(a);
                    sig(b);
                }
                if (const Handle(BRep_CurveOnClosedSurface) cocs = Handle(BRep_CurveOnClosedSurface)::DownCast(cr)) {
                    gp_Pnt2d a, b;
                    cocs->UVPoints2(a, b);
                    sig(a);
                    sig(b);
                }
                switch (kind) {
                    case kCurve3D: object(cr->Curve3D()); break;
                    case kCurveOnSurface:
                        object(cr->PCurve());
                        object(cr->Surface());
                        break;
                    case kCurveOnClosedSurface:
                        object(cr->PCurve());
                        object(cr->PCurve2());
                        object(cr->Surface());
                        break;
                    case kCurveOn2Surfaces:
                        object(cr->Surface());
                        object(cr->Surface2());
                        location(cr->Location2());
                        break;
                    case kPolygon3D: object(cr->Polygon3D()); break;
                    case kPolygonOnTriangulation:
                        object(cr->PolygonOnTriangulation());
                        object(cr->Triangulation());
                        break;
                    case kPolygonOnClosedTriangulation:
                        object(cr->PolygonOnTriangulation());
                        object(cr->PolygonOnTriangulation2());
                        object(cr->Triangulation());
                        break;
                    case kPolygonOnSurface:
                        object(cr->Polygon());
                        object(cr->Surface());
                        break;
                    case kPolygonOnClosedSurface:
                        object(cr->Polygon());
                        object(cr->Polygon2());
                        object(cr->Surface());
                        break;
                    default: unsupported = true;
                }
                location(cr->Location());
            }
        } else if (const Handle(BRep_TFace) tf = Handle(BRep_TFace)::DownCast(t)) {
            sig('f');
            object(tf->Surface());
            location(tf->Location());
            sig(static_cast<std::uint32_t>(tf->Triangulations().Extent()));
            for (const auto& tri : tf->Triangulations()) object(tri);
            object(tf->ActiveTriangulation());
        } else if (!t->IsKind(STANDARD_TYPE(TopoDS_TWire)) && !t->IsKind(STANDARD_TYPE(TopoDS_TShell)) &&
                   !t->IsKind(STANDARD_TYPE(TopoDS_TSolid)) && !t->IsKind(STANDARD_TYPE(TopoDS_TCompSolid)) &&
                   !t->IsKind(STANDARD_TYPE(TopoDS_TCompound))) {
            unsupported = true;
        }
        sig('(');
        for (TopoDS_Iterator it(holder(t), false, false); it.More(); it.Next()) {
            const TopoDS_Shape& c = it.Value();
            sig(static_cast<std::uint32_t>(c.Orientation()));
            location(c.Location());
            tshape(c.TShape());
        }
        sig(')');
    }
};

Walk walk_of(const std::vector<TopoDS_Shape>& roots) {
    Walk w;
    for (const auto& r : roots) w.root(r);
    return w;
}

/// What BinTools doesn't read back as it was, in walk order: each curve representation's
/// range and p-curve ends, and each location datum's transformation as it is held.
static_assert(std::is_trivially_copyable_v<gp_Trsf>);

std::string state_of(const Walk& w) {
    Writer out;
    for (const auto& cr : w.curves) {
        if (const Handle(BRep_GCurve) gc = Handle(BRep_GCurve)::DownCast(cr)) {
            out.f64(gc->First());
            out.f64(gc->Last());
        }
        if (const Handle(BRep_CurveOnSurface) cos = Handle(BRep_CurveOnSurface)::DownCast(cr)) {
            gp_Pnt2d a, b;
            cos->UVPoints(a, b);
            for (double v : {a.X(), a.Y(), b.X(), b.Y()}) out.f64(v);
        }
        if (const Handle(BRep_CurveOnClosedSurface) cocs = Handle(BRep_CurveOnClosedSurface)::DownCast(cr)) {
            gp_Pnt2d a, b;
            cocs->UVPoints2(a, b);
            for (double v : {a.X(), a.Y(), b.X(), b.Y()}) out.f64(v);
        }
    }
    for (const auto& d : w.datums) {
        const gp_Trsf& t = d->Transformation();
        out.b.append(reinterpret_cast<const char*>(&t), sizeof t);
    }
    for (const auto& g : w.geometry) geometry_state(g, out.b);
    return out.b;
}

template <class T>
T get_raw(Reader& in) {
    static_assert(std::is_trivially_copyable_v<T>);
    in.need(sizeof(T));
    T v;
    std::memcpy(static_cast<void*>(&v), in.p, sizeof v);
    in.p += sizeof v;
    return v;
}

/// geometry_state's counterpart: the state put back on a geometry object just read.
void restore_geometry(const Handle(Standard_Transient)& g, Reader& in) {
    if (g.IsNull()) return;
    if (const Handle(Geom_Line) c = Handle(Geom_Line)::DownCast(g)) {
        c->SetPosition(get_raw<gp_Ax1>(in));
    } else if (const Handle(Geom_Conic) c = Handle(Geom_Conic)::DownCast(g)) {
        c->SetPosition(get_raw<gp_Ax2>(in));
    } else if (const Handle(Geom_TrimmedCurve) c = Handle(Geom_TrimmedCurve)::DownCast(g)) {
        restore_geometry(c->BasisCurve(), in);
    } else if (const Handle(Geom_OffsetCurve) c = Handle(Geom_OffsetCurve)::DownCast(g)) {
        c->SetDirection(get_raw<gp_Dir>(in));
        restore_geometry(c->BasisCurve(), in);
    } else if (const Handle(Geom_ElementarySurface) e = Handle(Geom_ElementarySurface)::DownCast(g)) {
        e->SetPosition(get_raw<gp_Ax3>(in));
    } else if (const Handle(Geom_SurfaceOfLinearExtrusion) e = Handle(Geom_SurfaceOfLinearExtrusion)::DownCast(g)) {
        const gp_Dir d = get_raw<gp_Dir>(in);
        restore_geometry(e->BasisCurve(), in);
        e->SetBasisCurve(e->BasisCurve());
        e->SetDirection(d);
    } else if (const Handle(Geom_SurfaceOfRevolution) e = Handle(Geom_SurfaceOfRevolution)::DownCast(g)) {
        const gp_Ax1 a = get_raw<gp_Ax1>(in);
        restore_geometry(e->BasisCurve(), in);
        e->SetBasisCurve(e->BasisCurve());
        e->SetAxis(a);
    } else if (const Handle(Geom_RectangularTrimmedSurface) e = Handle(Geom_RectangularTrimmedSurface)::DownCast(g)) {
        restore_geometry(e->BasisSurface(), in);
    } else if (const Handle(Geom_OffsetSurface) e = Handle(Geom_OffsetSurface)::DownCast(g)) {
        restore_geometry(e->BasisSurface(), in);
    } else if (const Handle(Geom2d_Line) c = Handle(Geom2d_Line)::DownCast(g)) {
        c->SetPosition(get_raw<gp_Ax2d>(in));
    } else if (const Handle(Geom2d_Conic) c = Handle(Geom2d_Conic)::DownCast(g)) {
        c->SetAxis(get_raw<gp_Ax22d>(in));
    } else if (const Handle(Geom2d_TrimmedCurve) c = Handle(Geom2d_TrimmedCurve)::DownCast(g)) {
        restore_geometry(c->BasisCurve(), in);
    } else if (const Handle(Geom2d_OffsetCurve) c = Handle(Geom2d_OffsetCurve)::DownCast(g)) {
        restore_geometry(c->BasisCurve(), in);
    } else if (const Handle(Poly_Triangulation) t = Handle(Poly_Triangulation)::DownCast(g)) {
        t->SetMeshPurpose(static_cast<Poly_MeshPurpose>(get_raw<std::int32_t>(in)));
    }
}

/// Puts the state back on the representations of a walk of shapes just read; gives the
/// datums as they were held, by the datum read.
std::unordered_map<const Standard_Transient*, Handle(Standard_Transient)> restore_state(const Walk& w,
                                                                                        const std::string& state) {
    Reader in(state);
    for (const auto& cr : w.curves) {
        if (const Handle(BRep_GCurve) gc = Handle(BRep_GCurve)::DownCast(cr)) {
            const double first = in.f64();
            const double last = in.f64();
            gc->SetRange(first, last);
        }
        if (const Handle(BRep_CurveOnSurface) cos = Handle(BRep_CurveOnSurface)::DownCast(cr)) {
            const double ax = in.f64(), ay = in.f64(), bx = in.f64(), by = in.f64();
            cos->SetUVPoints(gp_Pnt2d(ax, ay), gp_Pnt2d(bx, by));
        }
        if (const Handle(BRep_CurveOnClosedSurface) cocs = Handle(BRep_CurveOnClosedSurface)::DownCast(cr)) {
            const double ax = in.f64(), ay = in.f64(), bx = in.f64(), by = in.f64();
            cocs->SetUVPoints2(gp_Pnt2d(ax, ay), gp_Pnt2d(bx, by));
        }
    }
    std::unordered_map<const Standard_Transient*, Handle(Standard_Transient)> held;
    for (const auto& d : w.datums) {
        in.need(sizeof(gp_Trsf));
        gp_Trsf t;
        std::memcpy(static_cast<void*>(&t), in.p, sizeof t);
        in.p += sizeof t;
        held.emplace(d.get(), new TopLoc_Datum3D(t));
    }
    for (const auto& g : w.geometry) restore_geometry(g, in);
    if (in.p != in.end) throw std::runtime_error("operation cache: a bad state");
    return held;
}

// ---------------------------------------------------------------------------
// Shapes to bytes and back, written together
// ---------------------------------------------------------------------------

std::string write_together(const std::vector<TopoDS_Shape>& roots) {
    BinTools_ShapeSet set;
    set.SetWithTriangles(true);
    set.SetWithNormals(true);
    for (const auto& r : roots) {
        if (!r.IsNull()) set.Add(r);
    }
    std::ostringstream os(std::ios::binary);
    set.Write(os);
    for (const auto& r : roots) set.Write(r, os);
    return os.str();
}

std::vector<TopoDS_Shape> read_together(const std::string& bytes, std::size_t count) {
    BinTools_ShapeSet set;
    std::istringstream is(bytes, std::ios::binary);
    set.Read(is);
    std::vector<TopoDS_Shape> roots(count);
    for (auto& r : roots) set.ReadSubs(r, is, set.NbShapes());
    if (is.fail()) throw std::runtime_error("operation cache: shapes could not be read");
    return roots;
}

// ---------------------------------------------------------------------------
// History, by indices
// ---------------------------------------------------------------------------

using ShapeIndex = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

ShapeIndex tracked_index(const std::vector<TopoDS_Shape>& shapes) {
    ShapeIndex out;
    for (const auto& s : tracked_subshapes(shapes)) out.Add(s);
    return out;
}

struct NotEncodable : std::runtime_error {
    using std::runtime_error::runtime_error;
};

/// The history as answers for every tracked sub-shape of the inputs and the results, each
/// shape by its side ('i' or 'o'), its index there and its orientation.
std::string encode_history(const ShapeHistory& history, const std::vector<TopoDS_Shape>& inputs,
                           const std::vector<TopoDS_Shape>& results) {
    Writer w;
    const auto root = [&](const TopoDS_Shape& s) {
        for (std::size_t k = 0; k < inputs.size(); ++k) {
            if (inputs[k].IsEqual(s)) {
                w.u8('i');
                w.u32(static_cast<std::uint32_t>(k));
                return;
            }
        }
        for (std::size_t k = 0; k < results.size(); ++k) {
            if (results[k].IsEqual(s)) {
                w.u8('o');
                w.u32(static_cast<std::uint32_t>(k));
                return;
            }
        }
        throw NotEncodable("history: an input that is not one of the operation's");
    };
    w.u32(static_cast<std::uint32_t>(history.before.size()));
    for (const auto& s : history.before) root(s);
    w.u32(static_cast<std::uint32_t>(history.brought.size()));
    for (const auto& s : history.brought) root(s);

    const ShapeIndex in = tracked_index(inputs);
    const ShapeIndex out = tracked_index(results);
    Writer records;
    const auto value = [&](const TopoDS_Shape& s) {
        if (const int k = out.FindIndex(s)) {
            records.u8('o');
            records.u32(static_cast<std::uint32_t>(k));
        } else if (const int j = in.FindIndex(s)) {
            records.u8('i');
            records.u32(static_cast<std::uint32_t>(j));
        } else {
            throw NotEncodable("history: a shape that is neither an input's nor a result's");
        }
        records.u8(static_cast<std::uint8_t>(s.Orientation()));
    };
    std::uint32_t count = 0;
    for (const ShapeIndex* side : {&in, &out}) {
        for (int k = 1; k <= side->Extent(); ++k) {
            const TopoDS_Shape& s = (*side)(k);
            if (side == &out && in.Contains(s)) continue;  // an input's sub-shape in the result: one key, said once
            const auto modified = history.modified(s);
            const auto generated = history.generated(s);
            const bool removed = history.is_removed(s);
            if (modified.empty() && generated.empty() && !removed) continue;
            records.u8(side == &in ? 'i' : 'o');
            records.u32(static_cast<std::uint32_t>(k));
            records.u8(removed ? 1 : 0);
            records.u32(static_cast<std::uint32_t>(modified.size()));
            for (const auto& m : modified) value(m);
            records.u32(static_cast<std::uint32_t>(generated.size()));
            for (const auto& g : generated) value(g);
            ++count;
        }
    }
    w.u32(count);
    w.b += records.b;
    return w.b;
}

std::shared_ptr<ShapeHistory> decode_history(const std::string& bytes, const std::vector<TopoDS_Shape>& inputs,
                                             const std::vector<TopoDS_Shape>& results) {
    Reader r(bytes);
    const auto root = [&]() -> TopoDS_Shape {
        const std::uint8_t side = r.u8();
        const std::uint32_t k = r.u32();
        const auto& from = side == 'i' ? inputs : results;
        if (k >= from.size()) throw std::runtime_error("operation cache: a bad history");
        return from[k];
    };
    std::vector<TopoDS_Shape> before, brought;
    for (std::uint32_t n = r.u32(); n > 0; --n) before.push_back(root());
    for (std::uint32_t n = r.u32(); n > 0; --n) brought.push_back(root());
    auto history = std::make_shared<ShapeHistory>(nullptr, std::move(before), std::move(brought));

    const ShapeIndex in = tracked_index(inputs);
    const ShapeIndex out = tracked_index(results);
    const auto shape = [&](std::uint8_t side, std::uint32_t k) -> TopoDS_Shape {
        const ShapeIndex& from = side == 'i' ? in : out;
        if (k < 1 || static_cast<int>(k) > from.Extent()) throw std::runtime_error("operation cache: a bad history");
        return from(static_cast<int>(k));
    };
    const auto value = [&]() {
        const std::uint8_t side = r.u8();
        const std::uint32_t k = r.u32();
        return shape(side, k).Oriented(static_cast<TopAbs_Orientation>(r.u8()));
    };
    // BRepTools_History's own adds undo each other (a modification un-removes, a removal
    // drops the modifications), but its Merge leaves a shape removed and modified when its
    // generations run out: such a shape is given a placeholder generation, which a merge
    // then removes, as Merge removed the real ones.
    struct Record {
        TopoDS_Shape key;
        bool removed;
        std::vector<TopoDS_Shape> modified, generated;
    };
    const Handle(BRepTools_History)& h = history->wrapped();
    std::vector<const Record*> removed_and_modified;
    std::vector<Record> records;
    for (std::uint32_t n = r.u32(); n > 0; --n) {
        Record rec;
        const std::uint8_t side = r.u8();
        rec.key = shape(side, r.u32());
        rec.removed = r.u8() != 0;
        for (std::uint32_t m = r.u32(); m > 0; --m) rec.modified.push_back(value());
        for (std::uint32_t g = r.u32(); g > 0; --g) rec.generated.push_back(value());
        records.push_back(std::move(rec));
    }
    TopoDS_Vertex placeholder;
    BRep_Builder().MakeVertex(placeholder, gp_Pnt(), 0);
    for (const Record& rec : records) {
        if (rec.removed && !rec.modified.empty()) {
            for (const auto& m : rec.modified) h->AddModified(rec.key, m);
            h->AddGenerated(rec.key, placeholder);
            removed_and_modified.push_back(&rec);
            continue;
        }
        for (const auto& g : rec.generated) h->AddGenerated(rec.key, g);
        for (const auto& m : rec.modified) h->AddModified(rec.key, m);
        if (rec.removed) h->Remove(rec.key);
    }
    if (!removed_and_modified.empty()) {
        Handle(BRepTools_History) without = new BRepTools_History();
        without->Remove(placeholder);
        h->Merge(without);
        for (const Record* rec : removed_and_modified) {
            for (const auto& g : rec->generated) h->AddGenerated(rec->key, g);
        }
    }
    return history;
}

// ---------------------------------------------------------------------------
// Rebinding a read entry to the live inputs
// ---------------------------------------------------------------------------

using CurveList = NCollection_List<Handle(BRep_CurveRepresentation)>;

struct Flags {
    bool free, modified, checked, orientable, closed, infinite, convex, locked;
};

Flags flags_of(const Handle(TopoDS_TShape)& t) {
    return {t->Free(), t->Modified(), t->Checked(), t->Orientable(), t->Closed(), t->Infinite(), t->Convex(), t->Locked()};
}

void set_flags(const Handle(TopoDS_TShape)& t, const Flags& f) {
    t->Modified(f.modified);  // (setting it clears Checked)
    t->Checked(f.checked);
    t->Orientable(f.orientable);
    t->Closed(f.closed);
    t->Infinite(f.infinite);
    t->Convex(f.convex);
    t->Locked(f.locked);
    t->Free(f.free);
}

/// Points the objects of a read entry that stand for live ones at the live ones.
class Rebinder {
public:
    std::unordered_map<const Standard_Transient*, Handle(Standard_Transient)> live_of;

    template <class T>
    Handle(T) map(const Handle(T)& h) const {
        if (h.IsNull()) return h;
        const auto it = live_of.find(h.get());
        if (it == live_of.end()) return h;
        return Handle(T)::DownCast(it->second);
    }

    TopLoc_Location location(const TopLoc_Location& l) const {
        if (l.IsIdentity()) return l;
        const TopLoc_Location next = location(l.NextLocation());
        const Handle(TopLoc_Datum3D) datum = map(l.FirstDatum());
        if (datum == l.FirstDatum() && next.IsEqual(l.NextLocation())) return l;
        return next * TopLoc_Location(datum).Powered(l.FirstPower());
    }

    TopoDS_Shape shape(const TopoDS_Shape& s) const {
        if (s.IsNull()) return s;
        TopoDS_Shape out = s;
        const Handle(TopoDS_TShape) t = map(s.TShape());
        if (t != s.TShape()) out.TShape(t);
        const TopLoc_Location l = location(s.Location());
        if (!l.IsEqual(s.Location())) out.Location(l, false);
        return out;
    }

    /// The representation with its objects mapped: itself, changed where it has setters, or
    /// a new one with the same parameters.
    Handle(BRep_CurveRepresentation) curve(const Handle(BRep_CurveRepresentation)& cr) const {
        const TopLoc_Location l = location(cr->Location());
        const bool same_l = l.IsEqual(cr->Location());
        switch (curve_kind(cr)) {
            case kCurve3D: {
                const Handle(Geom_Curve) c = map(cr->Curve3D());
                if (c != cr->Curve3D()) cr->Curve3D(c);
                if (!same_l) cr->Location(l);
                return cr;
            }
            case kCurveOnSurface: {
                const Handle(Geom2d_Curve) pc = map(cr->PCurve());
                const Handle(Geom_Surface) s = map(cr->Surface());
                if (pc == cr->PCurve() && s == cr->Surface() && same_l) return cr;
                const Handle(BRep_CurveOnSurface) old = Handle(BRep_CurveOnSurface)::DownCast(cr);
                double f, last;
                old->Range(f, last);
                gp_Pnt2d a, b;
                old->UVPoints(a, b);
                Handle(BRep_CurveOnSurface) n = new BRep_CurveOnSurface(pc, s, l);
                n->SetRange(f, last);
                n->SetUVPoints(a, b);
                return n;
            }
            case kCurveOnClosedSurface: {
                const Handle(Geom2d_Curve) pc = map(cr->PCurve());
                const Handle(Geom2d_Curve) pc2 = map(cr->PCurve2());
                const Handle(Geom_Surface) s = map(cr->Surface());
                if (pc == cr->PCurve() && pc2 == cr->PCurve2() && s == cr->Surface() && same_l) return cr;
                const Handle(BRep_CurveOnClosedSurface) old = Handle(BRep_CurveOnClosedSurface)::DownCast(cr);
                double f, last;
                old->Range(f, last);
                gp_Pnt2d a, b, a2, b2;
                old->UVPoints(a, b);
                old->UVPoints2(a2, b2);
                Handle(BRep_CurveOnClosedSurface) n = new BRep_CurveOnClosedSurface(pc, pc2, s, l, old->Continuity());
                n->SetRange(f, last);
                n->SetUVPoints(a, b);
                n->SetUVPoints2(a2, b2);
                return n;
            }
            case kCurveOn2Surfaces: {
                const Handle(Geom_Surface) s = map(cr->Surface());
                const Handle(Geom_Surface) s2 = map(cr->Surface2());
                const TopLoc_Location l2 = location(cr->Location2());
                if (s == cr->Surface() && s2 == cr->Surface2() && same_l && l2.IsEqual(cr->Location2())) return cr;
                return new BRep_CurveOn2Surfaces(s, s2, l, l2, cr->Continuity());
            }
            case kPolygon3D: {
                const Handle(Poly_Polygon3D) p = map(cr->Polygon3D());
                if (p != cr->Polygon3D()) cr->Polygon3D(p);
                if (!same_l) cr->Location(l);
                return cr;
            }
            case kPolygonOnTriangulation: {
                const Handle(Poly_PolygonOnTriangulation) p = map(cr->PolygonOnTriangulation());
                const Handle(Poly_Triangulation) t = map(cr->Triangulation());
                if (p == cr->PolygonOnTriangulation() && t == cr->Triangulation() && same_l) return cr;
                return new BRep_PolygonOnTriangulation(p, t, l);
            }
            case kPolygonOnClosedTriangulation: {
                const Handle(Poly_PolygonOnTriangulation) p = map(cr->PolygonOnTriangulation());
                const Handle(Poly_PolygonOnTriangulation) p2 = map(cr->PolygonOnTriangulation2());
                const Handle(Poly_Triangulation) t = map(cr->Triangulation());
                if (p == cr->PolygonOnTriangulation() && p2 == cr->PolygonOnTriangulation2() && t == cr->Triangulation() &&
                    same_l) {
                    return cr;
                }
                return new BRep_PolygonOnClosedTriangulation(p, p2, t, l);
            }
            case kPolygonOnSurface: {
                const Handle(Poly_Polygon2D) p = map(cr->Polygon());
                const Handle(Geom_Surface) s = map(cr->Surface());
                if (p == cr->Polygon() && s == cr->Surface() && same_l) return cr;
                return new BRep_PolygonOnSurface(p, s, l);
            }
            case kPolygonOnClosedSurface: {
                const Handle(Poly_Polygon2D) p = map(cr->Polygon());
                const Handle(Poly_Polygon2D) p2 = map(cr->Polygon2());
                const Handle(Geom_Surface) s = map(cr->Surface());
                if (p == cr->Polygon() && p2 == cr->Polygon2() && s == cr->Surface() && same_l) return cr;
                return new BRep_PolygonOnClosedSurface(p, p2, s, l);
            }
            default: throw std::runtime_error("operation cache: an unknown curve representation");
        }
    }

    void point(const Handle(BRep_PointRepresentation)& pr) const {
        switch (point_kind(pr)) {
            case kPointOnCurve: {
                const Handle(Geom_Curve) c = map(pr->Curve());
                if (c != pr->Curve()) pr->Curve(c);
                break;
            }
            case kPointOnCurveOnSurface: {
                const Handle(Geom2d_Curve) pc = map(pr->PCurve());
                if (pc != pr->PCurve()) pr->PCurve(pc);
                const Handle(Geom_Surface) s = map(pr->Surface());
                if (s != pr->Surface()) pr->Surface(s);
                break;
            }
            case kPointOnSurface: {
                const Handle(Geom_Surface) s = map(pr->Surface());
                if (s != pr->Surface()) pr->Surface(s);
                break;
            }
            default: throw std::runtime_error("operation cache: an unknown point representation");
        }
        const TopLoc_Location l = location(pr->Location());
        if (!l.IsEqual(pr->Location())) pr->Location(l);
    }

    /// A TShape of the entry's own: its geometry and children pointed at the live objects.
    void rebind(const Handle(TopoDS_TShape)& t) const {
        if (const Handle(BRep_TVertex) tv = Handle(BRep_TVertex)::DownCast(t)) {
            for (const auto& pr : tv->Points()) point(pr);
        } else if (const Handle(BRep_TEdge) te = Handle(BRep_TEdge)::DownCast(t)) {
            for (CurveList::Iterator it(te->ChangeCurves()); it.More(); it.Next()) it.ChangeValue() = curve(it.Value());
        } else if (const Handle(BRep_TFace) tf = Handle(BRep_TFace)::DownCast(t)) {
            const Handle(Geom_Surface) s = map(tf->Surface());
            if (s != tf->Surface()) tf->Surface(s);
            const TopLoc_Location l = location(tf->Location());
            if (!l.IsEqual(tf->Location())) tf->Location(l);
            triangulations(tf, tf);
        }
        // the children
        TopoDS_Shape self = holder(t);
        std::vector<TopoDS_Shape> before, after;
        bool changed = false;
        for (TopoDS_Iterator it(self, false, false); it.More(); it.Next()) {
            before.push_back(it.Value());
            after.push_back(shape(it.Value()));
            changed = changed || !after.back().IsEqual(before.back());
        }
        if (!changed) return;
        const Flags f = flags_of(t);
        t->Free(true);
        TopoDS_Builder builder;
        for (const auto& c : before) builder.Remove(self, c);
        for (const auto& c : after) builder.Add(self, c);
        set_flags(t, f);
    }

    /// A live input TShape changed as the operation changed it: it takes the state of its
    /// copy in the entry, the copy's objects pointed at the live ones.
    void patch(const Handle(TopoDS_TShape)& live, const Handle(TopoDS_TShape)& copy) const {
        if (live->DynamicType() != copy->DynamicType()) throw std::runtime_error("operation cache: inputs differ");
        if (const Handle(BRep_TVertex) cv = Handle(BRep_TVertex)::DownCast(copy)) {
            const Handle(BRep_TVertex) lv = Handle(BRep_TVertex)::DownCast(live);
            for (const auto& pr : cv->Points()) point(pr);
            lv->Pnt(cv->Pnt());
            lv->Tolerance(cv->Tolerance());
            lv->ChangePoints() = cv->Points();
        } else if (const Handle(BRep_TEdge) ce = Handle(BRep_TEdge)::DownCast(copy)) {
            const Handle(BRep_TEdge) le = Handle(BRep_TEdge)::DownCast(live);
            for (CurveList::Iterator it(ce->ChangeCurves()); it.More(); it.Next()) it.ChangeValue() = curve(it.Value());
            le->Tolerance(ce->Tolerance());
            le->SameParameter(ce->SameParameter());
            le->SameRange(ce->SameRange());
            le->Degenerated(ce->Degenerated());
            le->ChangeCurves() = ce->Curves();
        } else if (const Handle(BRep_TFace) cf = Handle(BRep_TFace)::DownCast(copy)) {
            const Handle(BRep_TFace) lf = Handle(BRep_TFace)::DownCast(live);
            lf->Surface(map(cf->Surface()));
            lf->Location(location(cf->Location()));
            lf->Tolerance(cf->Tolerance());
            lf->NaturalRestriction(cf->NaturalRestriction());
            triangulations(cf, lf);
        }
        set_flags(live, flags_of(copy));
    }

private:
    /// `to` takes `from`'s triangulations, mapped.
    void triangulations(const Handle(BRep_TFace)& from, const Handle(BRep_TFace)& to) const {
        NCollection_List<Handle(Poly_Triangulation)> list;
        bool changed = from != to;
        for (const auto& tri : from->Triangulations()) {
            list.Append(map(tri));
            changed = changed || list.Last() != tri;
        }
        const Handle(Poly_Triangulation) active = map(from->ActiveTriangulation());
        changed = changed || active != from->ActiveTriangulation();
        if (changed) to->Triangulations(list, active);
    }
};

/// Shapes written together read back as they were: their state restored, their datums as
/// they were held.
std::vector<TopoDS_Shape> read_exact(const std::string& shapes, const std::string& state, std::size_t count) {
    const std::vector<TopoDS_Shape> roots = read_together(shapes, count);
    const Walk w = walk_of(roots);
    Rebinder rb;
    rb.live_of = restore_state(w, state);
    for (const auto& object : w.objects) {
        if (const Handle(TopoDS_TShape) t = Handle(TopoDS_TShape)::DownCast(object)) rb.rebind(t);
    }
    std::vector<TopoDS_Shape> out;
    for (const auto& r : roots) out.push_back(rb.shape(r));
    return out;
}

// ---------------------------------------------------------------------------
// Entries
// ---------------------------------------------------------------------------

struct Entry {
    /// Not an entry but the record that the operation's result can't be kept, and why: it is
    /// computed then, without trying again.
    std::string rejected;
    bool mutated = false;
    std::uint32_t inputs = 0, results = 0;
    std::uint32_t walk_count = 0;
    std::uint64_t walk_signature = 0;
    std::string shapes;  ///< the inputs after the operation and its results, written together
    std::string state;   ///< what BinTools doesn't read back as it was (state_of)
    std::vector<std::pair<std::uint32_t, std::uint32_t>> links;  ///< (position in the entry's walk, in the inputs')
    bool has_history = false;
    std::string history;
    std::vector<std::string> warnings;
    double compute_ms = 0;

    std::string encode() const {
        Writer w;
        w.u32(kFormat);
        w.str(rejected);
        if (!rejected.empty()) return w.b;
        w.u8(mutated ? 1 : 0);
        w.u32(inputs);
        w.u32(results);
        w.u32(walk_count);
        w.u64(walk_signature);
        w.str(shapes);
        w.str(state);
        w.u32(static_cast<std::uint32_t>(links.size()));
        for (const auto& [p, k] : links) {
            w.u32(p);
            w.u32(k);
        }
        w.u8(has_history ? 1 : 0);
        if (has_history) w.str(history);
        w.u32(static_cast<std::uint32_t>(warnings.size()));
        for (const auto& m : warnings) w.str(m);
        w.f64(compute_ms);
        return w.b;
    }

    static Entry decode(const std::string& bytes) {
        Reader r(bytes);
        if (r.u32() != kFormat) throw std::runtime_error("operation cache: an entry of another format");
        Entry e;
        e.rejected = r.str();
        if (!e.rejected.empty()) return e;
        e.mutated = r.u8() != 0;
        e.inputs = r.u32();
        e.results = r.u32();
        e.walk_count = r.u32();
        e.walk_signature = r.u64();
        e.shapes = r.str();
        e.state = r.str();
        for (std::uint32_t n = r.u32(); n > 0; --n) {
            const std::uint32_t p = r.u32();
            e.links.emplace_back(p, r.u32());
        }
        e.has_history = r.u8() != 0;
        if (e.has_history) e.history = r.str();
        for (std::uint32_t n = r.u32(); n > 0; --n) e.warnings.push_back(r.str());
        e.compute_ms = r.f64();
        return e;
    }
};

struct Rejected {};

/// An entry that failed after it changed the live inputs (apply).
struct Damaged : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct Hit {
    Outcome outcome;
    std::vector<std::string> warnings;
    double compute_ms = 0;
};

/// What the entry gives for these live inputs (whose walk is `inputs_walk`): the results
/// sharing the live objects, the live inputs changed as the operation changed them.
Hit apply(const Entry& e, const std::vector<TopoDS_Shape>& inputs, const Walk& inputs_walk) {
    if (e.inputs != inputs.size()) throw std::runtime_error("operation cache: inputs differ");
    const std::vector<TopoDS_Shape> roots = read_together(e.shapes, e.inputs + e.results);
    // The walk of what was read has the order of the walk the entry was written from: the
    // same shapes, but for the state BinTools reads back otherwise, put back here.
    const Walk w = walk_of(roots);
    if (w.objects.size() != e.walk_count || w.unsupported) {
        throw std::runtime_error("operation cache: an entry read back otherwise");
    }
    Rebinder rb;
    rb.live_of = restore_state(w, e.state);
    for (const auto& [p, k] : e.links) {
        if (p >= w.objects.size() || k >= inputs_walk.objects.size()) throw std::runtime_error("operation cache: a bad link");
        const Handle(Standard_Transient)& copy = w.objects[p];
        const Handle(Standard_Transient)& live = inputs_walk.objects[k];
        if (copy->DynamicType() != live->DynamicType()) throw std::runtime_error("operation cache: inputs differ");
        rb.live_of[copy.get()] = live;
    }
    // What can fail without touching the live inputs first: the results' own TShapes
    // pointed at the live objects, and the history.
    std::vector<std::pair<Handle(TopoDS_TShape), Handle(TopoDS_TShape)>> changed;  // (live, copy)
    for (const auto& object : w.objects) {
        const Handle(TopoDS_TShape) t = Handle(TopoDS_TShape)::DownCast(object);
        if (t.IsNull()) continue;
        const auto it = rb.live_of.find(t.get());
        if (it == rb.live_of.end()) {
            rb.rebind(t);
        } else if (e.mutated) {
            changed.emplace_back(Handle(TopoDS_TShape)::DownCast(it->second), t);
        }
    }
    Hit hit;
    for (std::uint32_t k = 0; k < e.results; ++k) hit.outcome.shapes.push_back(rb.shape(roots[e.inputs + k]));
    if (e.has_history) hit.outcome.history = decode_history(e.history, inputs, hit.outcome.shapes);
    // Then the live inputs changed as the operation changed them. Past here a failure would
    // leave them changed halfway, so it is the program's error, never a quiet miss: the
    // entry was held to the computation on inputs of this content, so it means a key
    // collision or a fault of the cache.
    try {
        for (const auto& [live, copy] : changed) rb.patch(live, copy);
        // the inputs and results as the entry was written from them, state and sharing included
        std::vector<TopoDS_Shape> all = inputs;
        all.insert(all.end(), hit.outcome.shapes.begin(), hit.outcome.shapes.end());
        if (walk_of(all).signature_hash() != e.walk_signature) throw std::runtime_error("the walk differs");
    } catch (const Standard_Failure& f) {
        throw Damaged(std::string("operation cache: an entry could not be applied to its inputs (") + occt_message(f) +
                      "): turn the cache off, or empty its folder");
    } catch (const std::exception& f) {
        throw Damaged(std::string("operation cache: an entry could not be applied to its inputs (") + f.what() +
                      "): turn the cache off, or empty its folder");
    }
    hit.warnings = e.warnings;
    hit.compute_ms = e.compute_ms;
    return hit;
}

// ---------------------------------------------------------------------------
// Lookups
// ---------------------------------------------------------------------------

void tally(const std::string& operation, bool hit, bool stored, const std::string& rejected, double computed_ms,
           double hit_ms, double saved_ms, double key_ms, double store_ms) {
    const std::lock_guard<std::mutex> lock(stats_mutex);
    auto& row = row_of(operation);
    ++stats.lookups;
    ++row.lookups;
    if (hit) {
        ++stats.hits;
        ++row.hits;
    } else {
        ++stats.misses;
        ++row.misses;
    }
    if (stored) {
        ++stats.stored;
        ++row.stored;
    }
    if (!rejected.empty()) {
        ++stats.rejected;
        ++row.rejected;
        ++rejection_counts[rejected];
    }
    stats.computed_ms += computed_ms;
    row.computed_ms += computed_ms;
    stats.hit_ms += hit_ms;
    row.hit_ms += hit_ms;
    stats.saved_ms += saved_ms;
    row.saved_ms += saved_ms;
    stats.key_ms += key_ms;
    stats.store_ms += store_ms;
}

/// The entry for a computed operation, if it reproduces the computation (else why not).
std::optional<Entry> make_entry(const std::vector<TopoDS_Shape>& inputs, const std::string& inputs_before,
                                const std::string& state_before, const Walk& walk_before, const Outcome& outcome,
                                std::vector<std::string> warnings, double compute_ms, std::string& rejected) {
    std::vector<TopoDS_Shape> roots = inputs;
    roots.insert(roots.end(), outcome.shapes.begin(), outcome.shapes.end());
    const Walk w = walk_of(roots);
    if (w.unsupported) {
        rejected = "unsupported result";
        return std::nullopt;
    }
    Entry e;
    e.inputs = static_cast<std::uint32_t>(inputs.size());
    e.results = static_cast<std::uint32_t>(outcome.shapes.size());
    e.walk_count = static_cast<std::uint32_t>(w.objects.size());
    e.walk_signature = w.signature_hash();
    e.shapes = write_together(roots);
    e.state = state_of(w);
    e.mutated = write_together(inputs) != inputs_before || walk_of(inputs).signature != walk_before.signature;
    for (std::uint32_t p = 0; p < w.objects.size(); ++p) {
        const auto it = walk_before.index.find(w.objects[p].get());
        if (it != walk_before.index.end()) e.links.emplace_back(p, it->second);
    }
    if (outcome.history) {
        try {
            e.history = encode_history(*outcome.history, inputs, outcome.shapes);
            e.has_history = true;
        } catch (const NotEncodable&) {
            rejected = "history";
            return std::nullopt;
        }
    }
    e.warnings = std::move(warnings);
    e.compute_ms = compute_ms;

    // Held to the computation: the inputs as they were, read back from the key's bytes,
    // given the entry as a hit gives it, must come out as the live inputs and results are.
    try {
        const std::vector<TopoDS_Shape> before = read_exact(inputs_before, state_before, inputs.size());
        const Walk before_walk = walk_of(before);
        if (before_walk.signature != walk_before.signature) {
            rejected = "inputs read back otherwise";
            return std::nullopt;
        }
        const Hit hit = apply(Entry::decode(e.encode()), before, before_walk);
        std::vector<TopoDS_Shape> replayed = before;
        replayed.insert(replayed.end(), hit.outcome.shapes.begin(), hit.outcome.shapes.end());
        if (write_together(replayed) != e.shapes) {
            rejected = "not reproduced";
            return std::nullopt;
        }
        if (walk_of(replayed).signature != w.signature) {
            rejected = "structure not reproduced";
            return std::nullopt;
        }
        if (e.has_history && encode_history(*hit.outcome.history, before, hit.outcome.shapes) != e.history) {
            rejected = "history not reproduced";
            return std::nullopt;
        }
    } catch (const NotEncodable&) {
        rejected = "history not reproduced";
        return std::nullopt;
    } catch (const Standard_Failure&) {
        rejected = "not read back";
        return std::nullopt;
    } catch (const std::exception&) {
        rejected = "not read back";
        return std::nullopt;
    }
    return e;
}

}  // namespace

bool enabled() { return on.load(std::memory_order_acquire); }

Args& Args::raw(char tag, const void* data, std::size_t size) {
    bytes_.push_back(tag);
    bytes_.append(static_cast<const char*>(data), size);
    return *this;
}

Args& Args::add(const std::string& v) {
    add(static_cast<std::int64_t>(v.size()));
    bytes_.push_back('s');
    bytes_ += v;
    return *this;
}

Args& Args::add(const Vector& v) { return add(v.X).add(v.Y).add(v.Z); }

Args& Args::add(const gp_Trsf& t) {
    add(static_cast<std::int64_t>(t.Form()));
    add(t.ScaleFactor());
    for (int r = 1; r <= 3; ++r) {
        for (int c = 1; c <= 4; ++c) add(t.Value(r, c));
    }
    return *this;
}


Outcome run(const Args& args, const std::vector<TopoDS_Shape>& inputs, const std::function<Outcome()>& compute) {
    if (!enabled()) return compute();
    const std::shared_ptr<const State> st = current();
    if (!st || !st->store) return compute();

    // The key: the build, the operation and its arguments, and the inputs written together.
    const auto t0 = Clock::now();
    Walk walk;
    std::string written;
    try {
        walk = walk_of(inputs);
        if (!walk.unsupported) written = write_together(inputs);
    } catch (const Standard_Failure&) {
        walk.unsupported = true;
    }
    if (walk.unsupported) {
        {
            const std::lock_guard<std::mutex> lock(stats_mutex);
            ++stats.bypassed;
        }
        return compute();
    }
    XXH3_state_t* hs = XXH3_createState();
    XXH3_128bits_reset(hs);
    XXH3_128bits_update(hs, st->prefix.data(), st->prefix.size());
    const std::string op = args.operation();
    const std::uint64_t op_size = op.size();
    XXH3_128bits_update(hs, &op_size, sizeof op_size);
    XXH3_128bits_update(hs, op.data(), op.size());
    XXH3_128bits_update(hs, args.bytes().data(), args.bytes().size());
    const char context = clean_enabled() ? 'c' : 's';
    XXH3_128bits_update(hs, &context, 1);
    const std::uint64_t sizes[3] = {inputs.size(), walk.signature.size(), written.size()};
    XXH3_128bits_update(hs, sizes, sizeof sizes);
    XXH3_128bits_update(hs, walk.signature.data(), walk.signature.size());
    XXH3_128bits_update(hs, written.data(), written.size());
    const std::string key = hex(XXH3_128bits_digest(hs));
    XXH3_freeState(hs);
    const double key_ms = ms_since(t0);

    OperationCacheEvent event;
    event.operation = op;
    event.key = key;

    // A hit
    bool known_rejected = false;
    if (std::optional<std::vector<std::uint8_t>> bytes = st->store->get(key)) {
        try {
            const Entry e = Entry::decode(std::string(bytes->begin(), bytes->end()));
            if (!e.rejected.empty()) throw Rejected();
            Hit hit = apply(e, inputs, walk);
            for (const auto& m : hit.warnings) warn(m);
            const double ms = ms_since(t0);
            tally(op, true, false, "", 0, ms, hit.compute_ms - ms, key_ms, 0);
            event.hit = true;
            event.ms = ms;
            event.computed_ms = hit.compute_ms;
            if (st->options.on_lookup) st->options.on_lookup(event);
            return std::move(hit.outcome);
        } catch (const Rejected&) {
            known_rejected = true;
        } catch (const Damaged&) {
            throw;
        } catch (const Standard_Failure&) {
        } catch (const std::exception&) {
        }
        // An entry that can't be given for these inputs: computed as a miss.
    }

    // A miss: what BinTools doesn't read back as it was is taken before the operation can
    // change it, for the check of the entry.
    const std::string state_before = state_of(walk);
    Outcome outcome;
    std::vector<std::string> warnings;
    const auto c0 = Clock::now();
    {
        WarningCapture capture;
        outcome = compute();
        warnings = std::move(capture.messages);
    }
    const double compute_ms = ms_since(c0);
    bool stored = false;
    std::string rejected;
    double store_ms = 0;
    if (compute_ms >= st->options.min_ms && !known_rejected) {
        const auto s0 = Clock::now();
        std::optional<Entry> e = make_entry(inputs, written, state_before, walk, outcome, warnings, compute_ms, rejected);
        // Worth keeping only if reading it back is much cheaper than computing it: a hit
        // costs about what the check above took, which reads it back once. (A threshold
        // of 0 keeps everything, for tests.)
        if (e && st->options.min_ms > 0 && ms_since(s0) * 0.5 >= compute_ms) rejected = "not worth reading back";
        if (!rejected.empty()) {
            e = Entry{};
            e->rejected = rejected;
        }
        const std::string payload = e->encode();
        stored = st->store->put(key, std::vector<std::uint8_t>(payload.begin(), payload.end())) && rejected.empty();
        store_ms = ms_since(s0);
    }
    const double ms = ms_since(t0);
    tally(op, false, stored, rejected, compute_ms, 0, 0, key_ms, store_ms);
    event.stored = stored;
    event.rejected = rejected;
    event.ms = ms;
    event.computed_ms = compute_ms;
    if (st->options.on_lookup) st->options.on_lookup(event);
    return outcome;
}

TopoDS_Shape run_shape(const Args& args, const std::vector<TopoDS_Shape>& inputs,
                       const std::function<TopoDS_Shape()>& compute) {
    if (!enabled()) return compute();
    Outcome o = run(args, inputs, [&] { return Outcome{{compute()}, nullptr}; });
    return o.shapes.at(0);
}

}  // namespace detail::opcache

// ---------------------------------------------------------------------------
// The public side
// ---------------------------------------------------------------------------

void enable_operation_cache(std::shared_ptr<OperationStore> store, const OperationCacheOptions& options) {
    using namespace detail::opcache;
    if (!store) {
        disable_operation_cache();
        return;
    }
    auto s = std::make_shared<State>();
    s->store = std::move(store);
    s->options = options;
    s->prefix = key_prefix();
    {
        const std::lock_guard<std::mutex> lock(state_mutex);
        state = std::move(s);
    }
    on.store(true, std::memory_order_release);
}

void enable_operation_cache(const OperationCacheOptions& options) {
    if (options.folder.empty()) throw std::invalid_argument("enable_operation_cache: no folder");
    enable_operation_cache(folder_operation_store(options.folder, options.budget_bytes), options);
}

void disable_operation_cache() {
    using namespace detail::opcache;
    on.store(false, std::memory_order_release);
    const std::lock_guard<std::mutex> lock(state_mutex);
    state.reset();
}

bool operation_cache_enabled() { return detail::opcache::enabled(); }

OperationCacheStats operation_cache_stats() {
    using namespace detail::opcache;
    const std::lock_guard<std::mutex> lock(stats_mutex);
    OperationCacheStats out = stats;
    out.rejections.assign(rejection_counts.begin(), rejection_counts.end());
    return out;
}

void reset_operation_cache_stats() {
    using namespace detail::opcache;
    const std::lock_guard<std::mutex> lock(stats_mutex);
    stats = OperationCacheStats{};
    rejection_counts.clear();
}

}  // namespace build123d
