// The operation cache's side the library's operations call (operation_cache.hpp is the
// public side, docs/operation-cache.md the design). An operation that may be kept runs its
// work through run():
//
//   Outcome r = opcache::run(opcache::Args("fillet").add(radius), {obj_, edge1, edge2},
//                            [&] { ...compute...; return Outcome{{result}, history}; });
//
// `Args` names the operation and holds every argument it reads besides its input shapes,
// hashed by their exact bits; the inputs are written into the key by their content. The
// work must read nothing else that changes what it gives (SkipClean is added to every
// key), and its history (if any) may refer only to the inputs and the results.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace build123d {
class ShapeHistory;
class Vector;
}

namespace build123d::detail::opcache {

/// Is the cache on? (One atomic load: operations check it before building their key.)
bool enabled();

/// An operation's name and arguments, as bytes for its key.
class Args {
public:
    explicit Args(const char* operation) : operation_(operation) {}
    Args& add(double v) { return raw('d', &v, sizeof v); }
    Args& add(int v) { return add(static_cast<std::int64_t>(v)); }
    Args& add(std::int64_t v) { return raw('i', &v, sizeof v); }
    Args& add(bool v) { return add(static_cast<std::int64_t>(v ? 1 : 0)); }
    Args& add(const std::string& v);
    Args& add(const char* v) { return add(std::string(v)); }
    Args& add(const gp_Pnt& p) { return add(p.X()).add(p.Y()).add(p.Z()); }
    Args& add(const gp_Vec& v) { return add(v.X()).add(v.Y()).add(v.Z()); }
    Args& add(const gp_Dir& d) { return add(d.X()).add(d.Y()).add(d.Z()); }
    Args& add(const gp_Ax1& a) { return add(a.Location()).add(a.Direction()); }
    Args& add(const gp_Ax2& a) { return add(a.Location()).add(a.Direction()).add(a.XDirection()); }
    Args& add(const gp_Trsf& t);
    Args& add(const Vector& v);
    template <class T>
    Args& add(const std::optional<T>& v) {
        if (!v) return add(std::int64_t{-1});
        add(std::int64_t{1});
        return add(*v);
    }
    /// An enum, by its value.
    template <class E, std::enable_if_t<std::is_enum_v<E>, int> = 0>
    Args& add(E e) {
        return add(static_cast<std::int64_t>(e));
    }

    const std::string& operation() const { return operation_; }
    const std::string& bytes() const { return bytes_; }

private:
    Args& raw(char tag, const void* data, std::size_t size);
    std::string operation_;
    std::string bytes_;
};

/// What an operation gives: its result shapes (null ones too) and, if it keeps one, the
/// record of what it did to its inputs' sub-shapes.
struct Outcome {
    std::vector<TopoDS_Shape> shapes;
    std::shared_ptr<ShapeHistory> history;
};

/// Look the operation up and on a hit give what computing gave; otherwise compute it and,
/// if it took long enough, keep it. `inputs` are every shape the work reads (sub-shapes
/// of one another too: a fillet's edges with their solid).
Outcome run(const Args& args, const std::vector<TopoDS_Shape>& inputs, const std::function<Outcome()>& compute);

/// run() for an operation with one result and no history.
TopoDS_Shape run_shape(const Args& args, const std::vector<TopoDS_Shape>& inputs,
                       const std::function<TopoDS_Shape()>& compute);

/// The warnings raised on this thread while it lives, also passed on to the handler as
/// usual (warnings.cpp).
struct WarningCapture {
    WarningCapture();
    ~WarningCapture();
    WarningCapture(const WarningCapture&) = delete;
    WarningCapture& operator=(const WarningCapture&) = delete;
    std::vector<std::string> messages;
};

}  // namespace build123d::detail::opcache
