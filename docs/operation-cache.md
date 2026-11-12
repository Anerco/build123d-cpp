# The operation cache

A program built again with a parameter changed computes most of its operations
again with the same inputs. The operation cache keeps the result of each
operation that takes long enough. When the same operation comes with the same
inputs again, possibly in another process, it reads the result back instead of
computing it. A hit returns what computing returned: the same shape, sharing
the same sub-shapes and geometry with the operation's inputs. It also makes
the same in-place changes OpenCascade made to those inputs, raises the same
warnings and keeps the same history.

It is off unless a program or its host turns it on:

```cpp
build123d::enable_operation_cache({.folder = "/path/to/cache", .budget_bytes = 2ull << 30});
```

`include/build123d/operation_cache.hpp` is the API, `src/opcache.cpp` the
keys, entries and hits, `src/opcache_store.cpp` the folder store.

## Why the key is the inputs' content, not their provenance

The first idea was a provenance hash: every shape carries a hash of what
made it. A constructor would hash its arguments, `a - b` the operator and
both inputs' hashes, `Pos(...) * a` the move and `a`'s. Measured on this
library with OCCT 8.0.1, three things rule it out:

1. **OpenCascade changes its inputs in place.** A fuse of two boxes that
   share a face adds p-curves to the boxes' edges (9,160 → 9,714 bytes
   serialized). So do a fuse of two cylinders, an extrusion (it changes the
   sketch it extrudes), a loft (its sections double in size) and a 2D offset.
   These are BOP's default "destructive" mode and BRepLib's in-place updates.
   After `c = a + b`, `a` is no longer what its constructor made. A
   provenance hash can't see that. The next operation on `a` would then hit an
   entry computed from a different `a`.
2. **Results share sub-shapes and geometry with their inputs.** `box -
   cylinder` keeps 17 of its 42 TShapes from its inputs, and 44 of its 55
   surfaces and curves. A fillet keeps 30 of its 83 surfaces and curves. Later
   operations see that sharing. A boolean treats a face that both arguments
   share differently from two equal faces. `BRep_Tool::CurveOnSurface` finds an
   edge's p-curve by its surface's handle. `ShapeList` set operations and
   `Select::NEW` compare sub-shapes by TShape. A shape read back from bytes
   shares nothing, so a hit must reconnect it to the inputs. To do that it
   needs the inputs' structure anyway.
3. **Provenance has to be right in every function that makes a shape.**
   The library has hundreds of them, and one that forgets to change the hash
   gives a wrong hit. A content key can only miss.

So the key is a hash of what the operation reads:

```
key = XXH3-128( the build, the operation's name, each argument's exact bits,
                SkipClean, the inputs' walk, the inputs written together )
```

- **The build:** the entry format, the library's version, OCCT's version
  string, and the name, size and modification time of the loaded build123d
  object and every loaded `libTK*`. A rebuild of either gives other keys. Its
  old entries are never read again and age out under the budget.
- **The inputs written together** into one OCCT `BinTools_ShapeSet`, with
  triangulations and normals, then each input's root (orientation, index,
  location). Writing them together records which inputs share sub-shapes:
  `fuse(a, a.moved(...))` and two separate boxes give different keys. A
  sub-shape selected from an input, such as the edges of
  `fillet(part.edges()...)`, is written as a reference into its parent: the
  key says *which* edges, not only what they look like.
- **The inputs' walk** (`Walk` in `opcache.cpp`): each TShape, surface,
  curve, p-curve, polygon, triangulation and location datum, in the order a
  depth-first walk meets them, each marked new or met before. It adds what
  `BinTools` writes nothing of or reads back otherwise:
  - each representation's class, and the lock flags;
  - each face's list of triangulations;
  - the range of every curve representation, also one with no curve;
  - each p-curve's end points, which `BinTools` reads back recomputed;
  - each location datum's `gp_Trsf` as it is held, with its form, field by
    field (a `gp_Trsf` has 4 bytes of padding after its form, whatever was
    in memory: hashed whole, they made equal inputs miss);
  - each elementary curve's and surface's axis system as it is held:
    `BinTools` reads one back from its main and X directions, so its Y
    direction comes back recomputed;
  - each triangulation's purpose.

  Without these a fillet's input read back from bytes gives another fillet
  (the null curves' ranges), and a boolean on it another face orientation
  (the recomputed Y directions).
- **Doubles by their bits**, so formatting can't cause a miss and
  `-0.0 != 0.0`.

Costs: `BinTools` writes about 250 MB/s (a plate with 72 holes, 107 KB, in
0.4 ms), and the walk takes about a third of that: its index is an
open-addressing table of addresses, each class is resolved once per type,
and the state is gathered in the same pass. An operation on small shapes
costs microseconds. A build of the Faraday box's 76 lookups spends 14 ms on
keys, MagSafe's 156 about 38 ms.

What makes keys miss:

- **A part built where it stands.** `stand_in(back_t)` writes `back_t` into
  every coordinate, so every operation in it misses when `back_t` changes. The
  same part built at the origin and then moved (`Pos(0, 0, back_t) *
  stand_in()`) hits: the move is not an operation the cache keeps, and every
  operation under it sees the same inputs.
- **Anything that changes an input's content**, including its triangulation.
  A shape that was meshed (`tessellate`, an export) is a different input from
  the same shape unmeshed, because OCCT's algorithms can read the mesh
  (`BRepBndLib::Add` uses it for bounding boxes).
- **Another build of the library or of OCCT**, as above.

What makes keys collide: only a 128-bit hash collision, about 2⁻⁸⁸ at a
billion entries. That only holds if no program can write the folder: see
*Trust*.

## What is kept

Only the operations listed below are looked up. Each one runs through
`detail::opcache::run(args, inputs, compute)`. A result is kept only if:

- computing it took at least `min_ms` (5 ms by default);
- checking it took less than twice as long as computing it (a hit costs
  about what the check costs);
- it succeeded (an exception is never kept: the next build fails the same
  way, by computing);
- **the entry reproduces the computation**, checked before it is written
  (see *Exactness*).

A result that took long enough but can't be kept leaves a marker entry with
the reason, so later builds compute it without checking it again.

| operation | key arguments | inputs | history |
|---|---|---|---|
| booleans with history (`+ - &`, `fuse`, `cut`, `intersect`) | op, fuzzy value, glue, how many are arguments | arguments, tools | yes |
| booleans without history (`bool_op`: extrude both ways, thicken, section, offset with holes, sweeps with holes, ...) | op, fuzzy value, clean, glue, how many are arguments | arguments, tools | — |
| `clean()` (UnifySameDomain) | whether the history is asked for | the shape | yes |
| `fillet`, `chamfer` (3D) | radius / both lengths, whether a reference face is given | the shape, its edges, the face | yes |
| `extrude` (the prism; a tapered one is a loft and a cut), `revolve` | vector / angle, axis | the profile | — |
| `sweep` (one section, multisection), `Shell.sweep` | solid, frenet, binormal, transition | sections, path, binormal wire | — |
| `loft` | filled, ruled | sections | — |
| `offset_3d` (and `offset` of solids), `offset_2d` (and `offset` of faces and curves), `thicken` | amount, tolerance, kind | the shape, openings | — |
| `split` (the splitter; the keep is applied after it) | — | the shape, the tool | — |
| meshing: `Shape::mesh` (so `tessellate`, glTF, OBJ, 3MF), the viewers, and `incremental_mesh` for hosts | deflections, relative, parallel | the shape | — |

Not kept, because they cost less than a lookup: constructors of primitives
and sketches, moves and placements (`Location * shape`, `Plane * shape`,
`moved`, `located`), selections and filters (`edges()`, `filter_by`,
`sort_by`, ...), copies, and `make_face`. Labels, colors, materials, joints and
children are never part of an entry. The library's wrappers set them around
the operation as before, on a hit too. `export_stl` meshes twice in a row, as
Python's does, and is left as it is.

Nested operations (a sweep with holes cuts; a boolean cleans) are each
looked up too. The outer entry includes the warnings its inner operations
raised.

## What an entry holds

An entry is one file:

```
"B123DOC\0", u32 format, 32-hex-digit key, u64 payload size, XXH3-128 of the payload, payload
```

The payload (`Entry` in `opcache.cpp`):

- **The inputs after the operation and its results**, written together in
  one `BinTools_ShapeSet` (with triangulations), then each root. Writing them
  together records what the results share with the inputs and how the
  operation changed the inputs.
- **Their state**: what `BinTools` reads back otherwise, listed above,
  in walk order.
- **A link table.** It names each object of the entry's walk that existed in
  the inputs before the operation, by its position in the inputs' walk. It is
  recorded by pointer identity, comparing the walks before and after the
  operation.
- **Whether the operation changed its inputs.**
- **The history**, if the operation keeps one. It lists the before and
  brought inputs by number. For each tracked sub-shape of the inputs and
  results (vertex, edge, face, solid, in `TopExp` map order), it holds what
  the sub-shape was modified into, what it generated (each by side, map
  index and orientation), and whether it was removed.
- **The warnings** the operation raised, in order.
- **The time computing took**, for the statistics.

A marker entry holds only the reason the result was not kept.

## A hit

1. Read the set back: fresh copies of the inputs as the operation left them,
   and of the results. Put back their state.
2. Walk the copies, and map each object that the link table names to the
   live input object at that position of the inputs' walk.
3. Rebind every result TShape that is new: its children, surfaces, curves,
   p-curves, triangulations, polygons and locations point to the live
   objects where the link table says so. Representations without a setter
   (a curve on a surface, a regularity, a polygon on a triangulation) are
   rebuilt with the same parameters.
4. If the operation changed its inputs, change the live inputs the same way.
   Each live input TShape takes its copy's tolerance, flags, point,
   representations and triangulations, remapped to the live objects.
5. Walk the live inputs and the results together: the walk must be the one
   the entry was written from.
6. Rebuild the history from indices against the live inputs and the
   results, and raise the warnings again through `build123d::warn`, so the
   host's handler sees them from the same call.

An entry that can't be applied (it fails a check, or is torn) is a miss: the
operation is computed.

**The history's one quirk.** `BRepTools_History`'s own `AddModified` and
`Remove` undo each other, but its `Merge` can leave a shape both modified and
removed (when its generations were all removed). Such a shape is rebuilt with
a placeholder generation, which a merge then removes, as `Merge` removed the
real ones. What the history says of shapes that are neither inputs' nor
results' sub-shapes (intermediate shapes OCCT records keys for) is not kept.
Nothing in the library asks it about those.

## Exactness

The cache's promise: **a hit is what computing gave.** It holds by
construction, and each entry is checked before it is written:

1. Before the operation runs, the inputs are written to bytes (the key's
   bytes) and their state is taken.
2. After it runs, those bytes and that state are read back into a separate
   copy of the inputs as they were. The entry is applied to that copy, as a
   hit would apply it.
3. The copy (now changed as the operation changed the live inputs) and the
   results applied to it are written together. They must give **the same
   bytes** as the live inputs and the computed results written together.
   That compares geometry, tolerances, flags, triangulations, the sharing of
   every TShape, surface, curve, polygon and location, and every
   sub-shape's order. `TopExp` maps are made by the same walk of children
   that `BinTools` writes, so the same bytes mean the same map order: a
   selection by index after a hit selects the same faces.
4. Their walks must be the same: the structure and state `BinTools` doesn't
   write.
5. The history must answer the same for every tracked sub-shape.

An entry that fails is not written: a marker says why
(`operation_cache_stats().rejections`). Because the walk and the
substitutions depend only on the inputs' content, which the key fixes, a hit
later with other live inputs of the same content gives the same results.

What it does not cover:

- **Sharing with objects other than the inputs.** A result that shares a
  TShape or geometry with a global the operation read, such as a font
  cache, would come back unshared. None of the operations in the table
  does that.
- **Sharing inside composite geometry.** A trimmed curve's basis, or an
  offset surface's: `BinTools` writes them inline, so they come back as
  copies. Step 3 rejects an entry where that would show.
- **Representation classes the walk doesn't know.** The walk lists every
  class modern OCCT makes. An input with another is computed without a
  lookup (`bypassed` in the statistics).

**Determinism.** Some of OCCT's results depend on where the heap puts
things: maps keyed by address. Measured with the cache off, at least 19 of
the 3,288 comparison cases and 6 of the 47 example objects give other bytes
from run to run (ASLR) or under other malloc settings (`GLIBC_TUNABLES`):
texts, drawings, a DXF import, the brake-formed parts, three snap rings, the
logos, two of the joints example's parts, the vase. Their measures differ at
1e-15 or not at all. Turning the cache on moves the heap too, so
these give other bytes with it, as they would with any other change to the
program. A kept result is one of the results computing gives, and the first
entry written for a key stays (`link(2)` never replaces a name). Every build
after it gets that result, so builds become more repeatable, not less.

## The store

```cpp
class OperationStore {                 // operation_cache.hpp
public:
    virtual std::optional<std::vector<std::uint8_t>> get(const std::string& key) = 0;
    virtual bool put(const std::string& key, const std::vector<std::uint8_t>& entry) = 0;
};
```

The folder backend, `folder_operation_store(folder, budget_bytes)`:

- An entry is `<folder>/<k0k1>/<key>`. It is written whole to a temporary
  file beside it, then `link(2)`ed to its name and the temporary unlinked.
  `link` fails if the name exists, so the first writer wins. A reader sees
  no file or a whole one. Many processes can share the folder.
- A read checks the header, the key and the payload's hash. A bad file is
  removed and counts as a miss.
- A hit sets the file's modification time if it is older than a minute, so
  "oldest" means least recently used.
- **The budget.** When the store opens, and after about every `budget/16`
  bytes written by all the processes together (each write prunes with the
  chance its share of that gives), a process prunes. It takes an `flock` on
  `<folder>/.prune.lock`, skipping if another process holds it, and adds up
  every entry. Over budget, it removes the oldest until 90% of the budget is
  left. It also removes temporaries over an hour old. A reader that opened an
  entry before it was removed still reads it whole.
- Writes are not `fsync`ed: an entry torn by a crash fails its hash and is
  removed.

**Trust.** An entry is trusted as the operation's result. A program that
can write the folder can plant any result for any key, and every later
build that reads the folder would take it. On one person's machine, every
program is that person's. A host that builds other people's programs needs
one folder per trust domain. The host's sandbox should bind the folder
read-write only for builds of that domain.

## Turning it on

```cpp
struct OperationCacheOptions {
    std::string folder;                       // created if missing
    std::uint64_t budget_bytes = 1ull << 30;
    double min_ms = 5.0;                      // keep operations that took at least this
    std::function<void(const OperationCacheEvent&)> on_lookup;  // each lookup, as it ends
};
void enable_operation_cache(const OperationCacheOptions&);
void enable_operation_cache(std::shared_ptr<OperationStore>, const OperationCacheOptions& = {});
void disable_operation_cache();
OperationCacheStats operation_cache_stats();   // per process
void incremental_mesh(const TopoDS_Shape&, double linear, bool relative, double angular, bool parallel);
```

The statistics: lookups, hits, misses, entries stored, results rejected (by
reason), lookups bypassed, the milliseconds computing took, hits took and hits
saved (what computing took when kept, less what reading took), and the time
spent on keys and on checking and writing entries, each also by operation.

**A host** turns it on once, in the process its builds are forked from, with
a folder of its own and a budget. Each build's child inherits it. A host
that meshes a build's shapes itself calls `incremental_mesh` in place of
`BRepMesh_IncrementalMesh`, so the triangles of a kept shape are kept too.

## Concurrency

- Between processes: the folder store above.
- Between threads in one process (a program's parts built concurrently):
  the state is read under a mutex and the statistics kept under one, the
  store keeps no index in memory, each thread writes its temporaries under
  names of its own, and the warning capture is per thread. A hit raises its
  operation's warnings with `warn()` on the thread that asked, from inside
  the same call, so a host that places a warning by the thread's stack
  places it where computing would have. Two threads that compute the same
  key at once both compute it, each on its own inputs; the first entry
  linked stays and the other's `put` returns false. Each operation's key and
  walk read only its own inputs. Two threads must not run operations on the
  same shapes at once: OCCT itself doesn't allow that, since operations
  change their inputs (and a hit changes them as computing would).
- **A hit is a copy.** An entry is bytes, written once and never rewritten,
  and every hit decodes new TShapes and geometry from them, sharing only with
  that caller's own live inputs, as computing would. OCCT changes shapes it
  works with (booleans are destructive, meshing writes triangles onto the
  faces), so a later operation may change what a hit gave: it changes that
  build's or that branch's copy, never the entry nor another hit
  (`opcache_hit_is_a_copy`).
- **Booleans stay destructive.** Making them non-destructive
  (`SetNonDestructive(true)` on both of the library's boolean paths) was
  measured on the 3,288 comparison cases, alternating with today's, twice
  each: about as fast (237 and 237 s against 249 and 230 s), but 775 cases
  give other bytes and 56 other measures (centroids at 1e-15 to 1e-10, two
  assemblies' values), and Python build123d's booleans are destructive
  (OCP's default). The cache needs neither: it replays what computing does
  to the inputs, and hands out copies.
- **Settings kept per thread.** The library keeps one, `SkipClean` (which a
  host carries into a program's branches); the key reads it on the asking
  thread. The cache adds none: whether it is on, its store and its options
  are the process's.

## Testing it

- `tests/test_opcache.cpp`:
  - a hit gives the same bytes as computing, the results and inputs written
    together (so the same sharing), the same volume and area, and the same
    faces at the same indices;
  - `Select::NEW` and `LAST` select the same edges and faces;
  - an input changed in place is changed again on a hit;
  - warnings are replayed;
  - a changed argument, or an input moved by 1e-9, misses;
  - a fillet of other edges of the same box misses;
  - meshing is kept;
  - a hit is a copy: changing what one hit gave doesn't change the next;
  - two equal locations whose `gp_Trsf` padding differs are one key;
  - eight threads computing the same keys at once, cold and then warm, each
    give what computing alone gives;
  - the folder store keeps the first entry, removes a torn one, and stays
    within its budget.
- **The gate**, `python3 -I scripts/opcache_gate.py <build folder>`: the unit
  tests, the examples (their objects' lines and the files they write, by
  their bytes) and the comparison cases (each shape's bytes hashed too,
  `BUILD123D_SHOW_BREP`), each run with the cache off under five heap
  layouts, then with the cache cold, then warm, keeping every operation
  (`min_ms` 0). `tests/gate/gate.cpp`, linked into each, turns the cache on
  from `BUILD123D_OPCACHE_DIR`; `examples/` stays as it is, since vibe123d's
  setup trains OpenCascade's profile on those programs and builds it again
  when they change. Every output that is
  the same bytes in all five off runs must be the same bytes cold and warm,
  unless a second look finds what the cache gave with the cache off: the case
  or example alone under 16 more heap layouts, then its whole program under 4
  more. Each run has a temporary folder of its own.
