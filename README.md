# build123d-cpp

[![CI](https://github.com/Anerco/build123d-cpp/actions/workflows/ci.yml/badge.svg)](https://github.com/Anerco/build123d-cpp/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

A C++ port of [build123d](https://github.com/gumyr/build123d), the Python CAD library, and of
[bd_warehouse](https://github.com/gumyr/bd_warehouse), its library of standard parts, built
directly on [OpenCASCADE](https://dev.opencascade.org/) (OCCT).

- **All of build123d 0.13's algebra API**: the topology classes, every object and operation,
  joints and assemblies, materials, drafting, and every importer and exporter. Builder mode
  (`BuildPart`, `BuildSketch`, `BuildLine`) is the one thing not ported.
- **All of bd_warehouse 0.4**: threads, fasteners and their holes, bearings, gears, sprockets,
  flanges, pipes, O-rings, retaining rings, shaft keys and the OpenBuilds parts.
- **Checked against the Python originals**, not just compiled: 3,288 models and values are built in
  both and compared, and all of them match.

This is an independent port, not part of the build123d project. Its API follows build123d's as
closely as C++ allows, so build123d's documentation applies;
[From Python to C++](#from-python-to-c) lists the differences.

```cpp
#include <build123d/build123d.hpp>
using namespace build123d;

int main() {
    Part plate = Box(80, 60, 10) - GridLocations(60, 40, 2, 2) * Cylinder(3, 10);
    plate = fillet(plate.edges().filter_by(Axis::Z), 5);
    plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10);
    export_step(plate, "plate.step");
}
```

The same model in Python build123d:

```python
plate = Box(80, 60, 10) - GridLocations(60, 40, 2, 2) * Cylinder(3, 10)
plate = fillet(plate.edges().filter_by(Axis.Z), 5)
plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10)
export_step(plate, "plate.step")
```

![Models built by the C++ examples](docs/examples.png)

*Some of the ported examples, as written by the C++ programs in `examples/`
(rendered with `scripts/render_examples.py`).*

## Why C++

- **One language.** OCCT, the kernel underneath, is a C++ library. Python build123d reaches it
  through Python bindings (OCP), so a model runs as Python on top of C++. This port removes the
  Python layer: the modeling API sits directly on the kernel, in its own language.
- **No Python.** Use build123d's modeling from a C++ program, library or server, or ship a CAD
  feature without bundling an interpreter.
- **It starts at once.** A C++ program starts in 29 ms with 44 MB, where `import build123d`
  takes 1.25 s and 391 MB, so a program that builds one model and writes its STEP finishes about
  10× sooner. Once a model is building, the geometry is OCCT's in both, and it takes about as
  long ([Performance](#performance)).
- **Mistakes are caught by the compiler.** The algebra tracks dimensions in its types: `Box +
  Sphere` is a `Part`, `Circle - Rectangle` is a `Sketch`, and adding a circle to a box doesn't
  compile ([Static types](#static-types)).
- **The same results.** A model ported from Python builds the same solid, to the volume, area,
  topology and bounding box.

## What it covers

- **Geometry**: `Vector`, `Axis`, `Location`/`Pos`/`Rot`, `Plane`, `BoundBox`,
  `OrientedBoundBox`, `Matrix`, `Color`, length and mass units, and `GeomEncoder` (JSON).
  Intersections between them return what Python returns (a point, an axis, a plane, …).
- **Topology**: `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` and the algebra
  types `Curve`, `Sketch`, `Part`, with all of build123d's methods. That includes:
  - intersection and `touch` in every dimension and with geometry;
  - history, so `part.edges(Select::NEW)` gives the edges the last operation created;
  - convexity, mass/material/inertia, hashing, Gordon surfaces, wrapping faces onto surfaces;
  - the topology helpers `topo_explore_connected_edges`, `topo_explore_common_vertex`,
    `topo_distance_to`, `new_edges`, `delta`, `polar` and `edges_to_wires`.
- **Materials**: `FinishedMaterial` with bd_materials' catalog of 95 materials, its finishes,
  processes and colors, and their physically based looks (`PbrProperties`, with threejs_materials'
  presets). Assigning a material colors the shape, `mass()` uses its density, and `export_gltf`
  writes the looks with their textures.
- **Assemblies**: `children`, with `parent()`, `root()`, `path()`, `descendants()`, `leaves()`,
  `global_location()`, … (build123d's anytree tree).
- **Joints**: `RigidJoint`, `RevoluteJoint`, `LinearJoint`, `CylindricalJoint`, `BallJoint`,
  `connect_to`, `relative_to`, and their symbols.
- **Algebra**: `+`, `-`, `&` (and `+=`, `-=`, `&=`) with build123d's semantics, also with lists;
  placement with `Location * shape`, `Plane * shape` and `locations * shape`; `%` (tangent) and
  `^` (location) on curves.
- **Selectors**: `ShapeList` (set semantics like Python's), with `sort_by`, `group_by`,
  `filter_by`, `filter_by_position`, the `> < >> << |` operators and Python-style slicing.
- **Objects**:
  - 1D: `Line`, `Polyline`, `Spline`, `Bezier`, `BSpline`, `BlendCurve`, `CenterArc`,
    `ThreePointArc`, `RadiusArc`, `SagittaArc`, `TangentArc`, `JernArc`, `DoubleTangentArc`,
    `EllipticalCenterArc`, `EllipticalStartArc`, `ParabolicCenterArc`, `HyperbolicCenterArc`,
    `Helix`, `PolarLine`, `FilletPolyline`, `IntersectingLine`, `ConstrainedArcs`,
    `ConstrainedLines`, `Airfoil`. Arcs and `PolarLine` can end at a shape, axis, plane or point.
  - 2D: `Circle`, `Ellipse`, `Rectangle`, `RectangleRounded`, `Polygon`, `RegularPolygon`,
    `Trapezoid`, `Triangle`, `SlotArc`, `SlotCenterPoint`, `SlotCenterToCenter`, `SlotOverall`,
    `Superellipse`, `Text` (any font, including the bundled single-line font, with
    `FontManager` and `available_fonts`).
  - 3D: `Box`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `Wedge`, `Hole`, `CounterBoreHole`,
    `CounterSinkHole`, `ConvexPolyhedron`.
  - Drafting: `Draft`, `DimensionLine`, `ExtensionLine`, `Arrow`, `ArrowHead`,
    `TechnicalDrawing`.
  - Custom objects derive from `BaseCurveObject`, `BaseLineObject`, `BaseEdgeObject`,
    `BaseSketchObject` or `BasePartObject`, as in Python.
- **Locations**: `Locations`, `GridLocations`, `PolarLocations`, `HexLocations`.
- **Operations**: `extrude`, `revolve`, `loft`, `sweep`, `section`, `thicken`, `draft`, `fillet`,
  `chamfer`, `full_round`, `offset`, `mirror`, `scale`, `split`, `project`, `project_workplane`,
  `make_face`, `make_hull`, `make_brake_formed`, `trace`, `bounding_box`, `pack`.
- **Import and export**:
  - `export_step` (assemblies with names and colors), `export_brep`, `export_stl`,
    `export_gltf` (glTF and GLB, with the materials' looks) and `export_obj`; 3MF and STL meshes
    with `Mesher`;
  - 2D drawings with `ExportSVG` and `ExportDXF`, from hidden-line views (`project_to_viewport`);
  - `import_svg`, `import_svg_as_buildline_code`, `import_dxf`, `import_step`, `import_brep`,
    `import_stl`, and `detect_primitives` (faces from a mesh);
  - `serialize_shape`/`deserialize_shape`, which write and read the same bytes as Python's
    `persistence` module, and `to_bytes`/`from_bytes`, which save a shape with its label, color
    and children (Python's pickling);
  - `export_to_pcbway` (3D printing quotes);
  - `shape_to_html` and VTK output for viewers (Jupyter's display of a shape).

### bd_warehouse

`#include <bd_warehouse/bd_warehouse.hpp>`, namespace `bd_warehouse`. Every part is a `Part`,
so it takes the algebra like any other:

- **Threads**: `IsoThread`, `TrapezoidalThread`, `AcmeThread`, `MetricTrapezoidalThread`,
  `WhitworthThread`, `BSPPThread`, `PlasticBottleThread` and the general `Thread`.
- **Fasteners**: screws, nuts and washers of every standard bd_warehouse has (`SocketHeadCapScrew`,
  `HexNut`, `PlainWasher`, …, selected by size), and their `ClearanceHole`, `TapHole`,
  `ThreadedHole` and `InsertHole`.
- **Bearings**: deep groove (open and capped), angular contact, cylindrical and tapered roller,
  with `PressFitHole`.
- **Gears**: `SpurGear`, `HelicalGear`, `RackGear`, `Worm`, `WormWheel` and `WormGear`, with
  their plans and `InvoluteToothProfile`; and `Sprocket`.
- **Pipes and flanges**: `Pipe` and `PipeSection` by NPS and schedule; `Flange`, `BlindFlange`,
  `LappedFlange`, `SlipOnFlange`, `SocketWeldFlange` and `WeldNeckFlange` from the ASME tables.
- **Seals and keys**: `ORing` with its gland profiles, `ExternalSnapRing`, `InternalSnapRing` and
  their grooves, `ShaftKey` and `Keyway`, and `HexFlangedEccentricBushing`.
- **OpenBuilds**: rails, plates, wheels, spacers, couplers, lead screws, stepper motors and the
  C-Beam assemblies, with their joints.

Python's keyword arguments are an options struct:

```cpp
#include <build123d/build123d.hpp>
#include <bd_warehouse/bd_warehouse.hpp>
using namespace build123d;
namespace bdw = bd_warehouse;

int main() {
    Part gear = bdw::SpurGear(2, 24, 20, 8, {.root_fillet = 0.5});       // module 2, 24 teeth, 20°, 8 mm thick
    Part screw = bdw::SocketHeadCapScrew("M6-1", 20);                    // ISO 4762, M6 × 20
    Part thread = bdw::IsoThread(20, 1.5, 10, {.external = false});      // an internal M20 × 1.5 thread
    export_step(Compound::make_assembly({gear, Pos(40, 0, 0) * screw, Pos(-40, 0, 0) * thread}), "parts.step");
}
```

## How it was checked

Everything above was checked against Python build123d 0.13.0 and bd_warehouse 0.4.0
themselves, not just compiled:

- `tests/compare` builds the same 3,288 models and values in C++ and in Python and compares
  them: 884 of build123d's, and 2,404 of bd_warehouse's, with every size of every standard part
  and the errors it raises. Shapes are compared by volume, area, total edge length, topology
  counts, bounding box and center of mass (to 1e-6 relative); other results value by value
  (points, axes, mass properties, file contents, …). All of them match.
- All 24 of build123d's official algebra examples are ported in `examples/`. The 47 objects they
  show match their Python originals the same way.
- `tests/test_*.cpp` has 88 unit tests for the C++ API itself (static result types, joints and
  tree semantics, error handling, file round trips, PCBWay uploads against a local server, the
  operation cache).

With OCCT 8.0.1 all of them pass: 88/88 unit tests, 3,288/3,288 cases and 47/47 example
objects, against Python build123d 0.13 on the same OCCT (its OCP wheel is 8.0.1). They pass
with 8.0.0 too: there `export_step` gives the STEP writer the defaults 8.0.1 gives it (OCCT
#1334), without which 8.0.0 drops cones that narrow along their axis, such as a chamfer on a
round edge.

The build123d part also builds and runs with OCCT 7.6 to 7.9. Measured before bd_warehouse was
ported, against its 64 unit tests, 884 cases and 47 example objects, compared with Python on
OCCT 8.0:

| OCCT | unit tests | comparison cases | example objects |
|---|---|---|---|
| 7.9.3 | 64/64 | 862/884 | 47/47 |
| 7.7.2 | 64/64 | 843/884 | 47/47 |
| 7.6.3 | 64/64 | 838/884 | 44/47 |

With the older versions, results differ only where OCCT's own results differ:

- the OCCT version written into STEP and glTF headers;
- meshes (STL, OBJ, UVs) and hidden-line views;
- in 7.6 and 7.7, also 2D offsets (slots, offset curves), convexity, wrapping, sewing and the
  outlines of single-line text;
- a torus with both a partial minor and a partial major angle: OCCT builds end faces it
  flags as invalid in every version, and before 8.0 its algorithms (volume, bounding box)
  throw on them.

## Performance

The same models built with this library and with Python build123d 0.13 / bd_warehouse 0.4,
checked to give the same solids (volume, area, faces):

| | C++ | Python | Python ÷ C++ |
|---|---:|---:|---:|
| Start: a program vs `import build123d` | 29 ms, 44 MB | 1.25 s, 391 MB | 43 |
| Whole run: build one model, write a STEP (median of 45) | 0.14 s, 66 MB | 1.39 s, 419 MB | 10 (1.5–34) |
| Build in a running process: the 24 examples | 8.65 s | 8.71 s | 1.01 (each: 0.94–2.65, median 1.12) |
| 14 bd_warehouse parts | 1.88 s | 2.28 s | 1.21 (each: 1.03–2.05) |
| The 1,473 bd_warehouse comparison models | 87 s | 110 s | 1.27 |
| STEP, STL and glTF export, STEP import | 4.05 s | 4.05 s | 1.00 |

- **The start is the difference.** Python spends 1.2 s and 390 MB importing build123d (OCP, and
  scipy and scikit-learn for STL import) before building anything, so every whole run takes
  1.2–1.6 s longer, and small models finish 10–30× sooner in C++. Of the C++ program's 29 ms,
  25 ms is loading OCCT's shared libraries.
- **Once running, the geometry costs the same:** it is OCCT's in both. Booleans, fillets, gears,
  sprockets and file I/O are within 7%. Python adds about 1 µs to each call into OCCT, so models
  made of many small calls gain most: threads and threaded fasteners 1.5–2×, the build123d logo
  1.9×.
- **Python is faster on some OCCT-bound models:** the heat exchanger (0.95), loft (0.94) and key
  cap (0.98), and a boolean and a fillet called on OCCT directly (0.98). Both make the same OCCT
  calls; the OCCT measured here (conda-forge 8.0.0) runs them a little slower than the one in
  Python's wheel (8.0.1, a different build).
- `Text` is 4× faster because the port skips a bounding box and a copy that build123d computes
  even when no alignment is asked for: a shortcut in the port, not C++.

Ryzen 7 9800X3D (16 threads), Ubuntu 24.04 on WSL2, GCC 15.3 (library `-O3`, programs `-O2`),
Python 3.12.3. Builds: median of at least 10 after 2 warm-ups, spread within ±4% but for two
models that take microseconds; whole runs: median of 5; memory: peak RSS. Run serially on an
idle machine; a second pass moved the ratios by 1% (median), at most 6%. Every model's numbers
are in [`bench/results/report.md`](bench/results/report.md). To reproduce, with the Release build
and a Python with build123d 0.13 and bd_warehouse 0.4: `PY=/path/to/python bench/run.sh` (about
50 minutes).

## The operation cache

A program built again with one parameter changed recomputes most of its operations with the
same inputs. `enable_operation_cache({.folder = ..., .budget_bytes = ...})` keeps the result of
each expensive operation (booleans, fillets, sweeps, lofts, offsets, meshing, ...) in a folder,
keyed by its arguments and its inputs' content, and reads it back when the same operation
comes again, in this process or another. A hit gives what computing gave, down to the bytes:
the same shape, sharing the same sub-shapes with its inputs, the inputs changed in place as
OCCT changes them, the same warnings and history. Each entry is checked against the
computation before it is written. It is off unless turned on.
[`docs/operation-cache.md`](docs/operation-cache.md) has the design;
`scripts/opcache_gate.py` runs the unit tests, examples and comparison cases with it off,
cold and warm.

## Building

Requirements:
- CMake ≥ 3.16.
- A C++17 compiler. The examples and tests need C++20, because they use designated
  initializers.
- OCCT 7.6 or newer with its CMake config files.
- lib3mf (for `Mesher`).
- libcurl (optional, for `export_to_pcbway`; without it that function throws).

Builds were tested on Linux x86-64 with GCC 15. Programs that use the library also compile
with clang. CI (`.github/workflows/ci.yml`) builds with the toolchain below on GitHub's Ubuntu
24.04 runners and runs the unit tests and the examples on every push and pull request; on
`main`, weekly and when started by hand, it also runs the comparison with Python build123d.

```sh
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/occt
cmake --build build
./build/examples/tea_cup        # prints its properties and writes tea_cup.step
./build/build123d_tests
```

If you don't have the dependencies, `scripts/setup-toolchain.sh` creates a self-contained
toolchain (OCCT 8.0 without VTK, lib3mf, libcurl, CMake, Ninja and GCC from conda-forge) in
`./.toolchain` without root access, and prints the commands to build with it:

```sh
scripts/setup-toolchain.sh
.toolchain-bin/micromamba run -p ./.toolchain cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=$PWD/.toolchain
.toolchain-bin/micromamba run -p ./.toolchain cmake --build build
```

To use the library from another CMake project, either `add_subdirectory()` this directory or
install it (`cmake --install build --prefix …`) and use `find_package`:

```cmake
find_package(build123d CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE build123d::build123d)
```

The bundled single-line font is installed to `<prefix>/share/build123d/fonts`. The library looks
for it in `$BUILD123D_FONT_DIR`, next to the program or library, in the install prefix and in the
source tree.

The textures of the materials' looks (wood, brushed metal, 3D-printed layers, fabrics, …:
67 PNG files, 87 MB) aren't part of the sources. `scripts/fetch-textures.sh` downloads them into
`data/textures`; it fetches the pinned threejs_materials wheel from PyPI, checks its SHA-256 and
extracts only the textures. `scripts/setup-toolchain.sh` offers to run it, and `cmake --install`
installs them to `<prefix>/share/build123d/textures`. The library looks for them in
`$BUILD123D_TEXTURE_DIR`, next to the library, in the install prefix and in the source tree.
Looks without textures (plain metals, plastics, glass) don't need them. Without them,
`export_gltf` of a textured look throws `std::runtime_error`, naming the script. Assigning a
textured catalog material still gives the shape its color; a textured material with choices of
its own leaves the color unchanged, with a warning.

Programs that use VTK can `#include <build123d/vtk.hpp>` for `to_vtk_poly_data`, which returns a
`vtkPolyData`; link VTK yourself. The rest of the library doesn't depend on VTK.

### Comparing with Python build123d

With a Python environment that has `build123d==0.13.0` and `bd_warehouse==0.4.0` (and `vtk`,
for the viewer cases), and the fonts that the text cases read by their paths (on Ubuntu 24.04,
`fonts-dejavu-core`, `fonts-dejavu-mono`, `fonts-ubuntu` and `fonts-urw-base35`):

```sh
python tests/compare/compare.py build/compare_cases           # the models and values
python tests/compare/compare_examples.py build/examples       # the examples
```

Unless `BUILD123D_TEXTURE_DIR` is set, `compare.py` gives the C++ side the textures of the
Python environment's threejs_materials.

## From Python to C++

| Python | C++ |
|---|---|
| `from build123d import *` | `#include <build123d/build123d.hpp>` and `using namespace build123d;` |
| `Box(1, 2, 3, align=Align.MIN)` | `Box(1, 2, 3, Align::MIN)` |
| `Box(1, 2, 3, align=(Align.MIN, Align.CENTER, Align.MAX))` | `Box(1, 2, 3, {Align::MIN, Align::CENTER, Align::MAX})` |
| `extrude(s, 5, both=True, taper=10)` | `extrude(s, 5, {.both = true, .taper = 10})` |
| `extrude(s, until=Until.NEXT, target=part)` | `extrude(s, Until::NEXT, part)` |
| `offset(part, -1, openings=faces)` | `offset(part, -1, {.openings = faces})` |
| `sweep(sections, path, multisection=True)` | `sweep(sections, path, {.multisection = true})` |
| `Text("hi", 10, font_style=FontStyle.BOLD)` | `Text("hi", 10, {.font_style = FontStyle::BOLD})` |
| `Triangle(a=3, b=4, c=5)` | `Triangle({.a = 3, .b = 4, .c = 5})` |
| `CenterArc((0, 0), 1, 0, arc_size=line)` (a limit) | `CenterArc({0, 0}, 1, 0, line)` |
| `curve @ 0.5` | `curve.position_at(0.5)` |
| `curve % 0.5`, `curve ^ 0.5` | `curve % 0.5`, `curve ^ 0.5` |
| `part.volume`, `edge.length`, `vertex.X` | `part.volume()`, `edge.length()`, `vertex.X()` |
| `faces()[-1]`, `faces()[1:3]`, `faces().last` | `faces()[-1]`, `faces().slice(1, 3)`, `faces().last()` |
| `edges().filter_by(lambda e: e.length > 1)` | `edges().filter_by([](const Edge& e) { return e.length() > 1; })` |
| `edges().group_by(Edge.radius)` | `edges().group_by([](const Edge& e) { return e.radius(); })` |
| `edges() > Axis.Z`, `>>`, `<<`, `\|` | the same operators |
| `part.edges(Select.NEW)` | `part.edges(Select::NEW)` |
| `GridLocations(5, 5, 2, 2) * Circle(1)` | `GridLocations(5, 5, 2, 2) * Circle(1)` |
| `Plane(origin=(0, 0, 5), z_dir=(1, 0, 0))` | `Plane({0, 0, 5}, std::nullopt, {1, 0, 0})` |
| `Plane(origin, x_dir=x, y_dir=y)` | `Plane::from_x_y(origin, x, y)` |
| `Rot(Z=45)` | `Rot(0, 0, 45)` |
| `Pos(*point)` | `Pos(point)` |
| `axis.intersect(plane)` (a Vector, an Axis or None) | `axis.intersect(plane)`: `std::optional<std::variant<Vector, Axis>>` |
| `part.intersect(other)` (a ShapeList or None) | `part.intersect(other)`: a `ShapeList<Shape>`, empty for None |
| `top, bottom = part.split(plane, keep=Keep.BOTH)` | `auto [top, bottom] = part.split(plane, Keep::BOTH);` |
| `Compound(children=[a, b])` | `Compound::make_assembly({a, b})`, or `Compound c; c.children = {a, b};` |
| `child.parent = assembly` | `assembly.children.push_back(child)` (the tree stores a copy) |
| `node.global_location` | `node.global_location()` |
| `part.color = "red"`, `part.material = "aluminum"` | the same |
| `part._color` | `part.color.raw()` |
| `part.position = (5, 0, 0)`, `part.orientation = (0, 0, 90)` | `part.set_position({5, 0, 0})`, `part.set_orientation({0, 0, 90})` |
| `part.location = loc`, `part.wrapped = obj` | `part.set_location(loc)`, `part.set_wrapped(obj)` |
| `axis.direction = d`, `plane.x_dir = d` | `axis.set_direction(d)`, `plane.set_x_dir(d)` (assigning the fields directly skips normalizing) |
| `RigidJoint("j", part, location)` | the same |
| `a.joints["j"].connect_to(b.joints["k"], angle=90)` | `a.joints["j"].connect_to(b.joints["k"], {.angle = 90})` |
| `with SkipClean(): ...` | `{ SkipClean skip; ... }` |
| `show(part)` (a viewer) | `export_step(part, "part.step")`, or `shape_to_html(part)` |
| `ValueError`, `TypeError` | `std::invalid_argument` |
| `IndexError`, `KeyError` | `std::out_of_range` |
| `ZeroDivisionError` | `std::domain_error` |
| `RuntimeError`, OCCT failures | `std::runtime_error` |
| `warnings.warn(...)` | `build123d::warn(...)`; redirect or silence with `set_warning_handler` |

Option structs use C++20 designated initializers, which must follow the order the fields
are declared in (see `operations.hpp` and `objects.hpp`). Untyped results (`import_step`,
`import_brep`, the elements of `intersect`, …) are `Shape`s; get a specific type with
`cast_shape<Solid>(shape)` or `std::visit(…, cast_shape(shape))` (Python's `Shape.cast`).

### Static types

The algebra keeps track of dimensions at compile time:

- `Box + Sphere` is a `Part`, `Circle - Rectangle` is a `Sketch` and `Line + Arc` is a
  `Curve`. Python returns a `Wire` or an `Edge` when curves join; in C++ it's always a `Curve`.
- `a & b` is a `Part` for two 3D shapes, a `Sketch` for 3D & 2D and a `Curve` for 3D & 1D.
  Every other combination is a `Compound`, because those results can mix dimensions (two
  crossing lines meet in a vertex). A `Compound` converts to `Sketch`, `Curve` or `Part`,
  checked when it runs.
- Adding shapes of different dimensions (`Box(1, 1, 1) + Circle(1)`) does not compile.
- Placement keeps the type: `Pos(1, 0) * Box(…)` is a `Part`, and `Rot(…) * edge` is an
  `Edge`.
- `Part`, `Sketch` and `Curve` don't convert into each other.
- `fillet`/`chamfer` of vertices return a `Compound`, which converts to `Sketch` or `Curve`.
- `Edge::close` and `offset_2d` always return a `Wire`; Python returns an `Edge` when the result
  is a single edge. `Mixin1D::project` returns a `Mixin1D` (an edge or a wire).

Python code that changes a variable's type needs a different declaration in C++:

```cpp
Curve s = Spline({{30, 10}, {69, 105}}, {{1, 0.5}, {0.7, 1}});  // not `Spline s`
s += Polyline({...});                                          // Spline + Polyline is a Curve
Part cup = revolve(Plane::XZ * make_face(s), Axis::Z);          // a new variable, not `s =`
```

Inside a class derived from a shape type, the shape's own member functions (`mirror`,
`scale`, `fillet`, …) hide the free operations of the same name. Call them as
`build123d::mirror(…)`, or build the shape in a free function (see
`examples/custom_sketch_objects.cpp`).

### Values, joints and assemblies

Shapes are values: copying one copies it, where Python shares the object.

- An assembly owns its children: `children.push_back(shape)` stores a copy, and every child
  knows its parent (`parent()`) through copies and moves. As in Python, changing the children of
  a compound rebuilds its geometry from them.
- Joint objects are handles to the joint in their part's `joints`: `j.connect_to(...)` and
  `part.joints["j"].connect_to(...)` act on the same joint. Copying a part gives the copy its
  own joints (Python's deepcopy does the same). A joint whose part was destroyed throws.
- `color` and `material` behave like build123d's properties. A shape without its own takes its
  nearest ancestor's when read, and keeps it. Assigning a material (a catalog name or a
  `FinishedMaterial`) also colors the shape with the material's look. Read them like a
  `std::optional`. `raw()` and `set_raw()` give and set the shape's own value without either
  effect, like Python's `_color` and `_material`.

### Differences from Python build123d

Where Python build123d has a bug, the C++ is the corrected version:

- `Edge::param_at_point` returns the parameter along the edge's orientation, so
  `position_at(param_at_point(p)) == p` also for reversed edges.
- `Wire::location_at` with `FrameMethod::CORRECTED` in parameter mode normalizes the position
  once (Python does it twice).
- `IntersectingLine` ignores intersections behind its start point.
- `Location(plane, offset)` applies the offset (Python ignores it).
- `Color(0)` is black (Python makes it white).

Smaller differences:

- `Face::wrap_faces` leaves the faces it is given unchanged (Python moves them).
- Where Python returns something in an arbitrary order (from a `set`), the C++ keeps the order
  found: `ShapeList` set operations, `Face::axes_of_symmetry`, `FontManager::register_folder`.
- Numbers that Python formats differently as int and float (e.g. a drawing scale of `2`) are
  formatted as floats.
- `shape_to_html`, `to_vtkpoly_string` and `vtk_mesh` don't need VTK. The VTK text has the same
  content as Python's but isn't compressed.
- Materials: images that Python re-encodes for glTF (packed metallic/roughness, palette
  textures) are compressed by lodepng rather than zlib, so their bytes differ but their pixels
  don't. Textures must be PNG files, as all of threejs_materials' are. Numbers that Python keeps
  as ints are floats: in the catalog's `repr()` (`density=2700.0`) and in `PbrProperties::from_dict`
  data written to glTF (`[1.0, 1.0]`).
- `to_bytes` uses its own format, so it can't read Python pickles (the shape data inside them,
  `serialize_shape`'s bytes, is the same). Unlike Python, it can save a shape that has a color.
- `export_to_pcbway` only opens http(s) pages, and never through a shell.
- Before OCCT 8, OCCT's exceptions don't derive from `std::exception`: an OCCT error that the
  library doesn't translate arrives as a `Standard_Failure`.

### Not ported

Builder mode: `BuildPart`, `BuildSketch`, `BuildLine`, `Mode`, the builder-context selector
functions, `add`/`insert`, locations used as context managers and the `mode=` arguments.
bd_warehouse's parts that build themselves in builder mode are built the same way inside the
port. Its holes (`ClearanceHole`, `TapHole`, …) need their depth given, which Python reads from
the builder's part.

## Layout

```
include/build123d/   public headers (build123d.hpp includes them all; vtk.hpp needs VTK)
include/bd_warehouse/ bd_warehouse's headers (bd_warehouse.hpp includes them all)
src/                 implementation, bd_warehouse's in src/bd_warehouse/
data/fonts/          the bundled single-line font (SIL Open Font License)
data/textures/       the materials' textures, once scripts/fetch-textures.sh has fetched them
third_party/         lodepng (PNG files), xxHash (the operation cache's keys)
examples/            ports of build123d's algebra examples; show.hpp prints results
tests/test_*.cpp     unit tests
tests/compare/       comparison with Python build123d
bench/               the benchmark against Python build123d (Performance); results/report.md
                     has every model's numbers
.github/workflows/   CI: the build, unit tests and examples, and the comparison with Python
scripts/             toolchain setup, fetch-textures.sh, the gen_*.py generators of the tables
                     in src/ (from the Python packages), render_examples.py (docs/examples.png),
                     opcache_gate.py (the operation cache's gate)
docs/                operation-cache.md (the operation cache's design), examples.png
cmake/               the package config for find_package(build123d)
```

## License

Licensed under the Apache License 2.0 (`LICENSE`); `NOTICE` credits the projects it is ported
from. This is a port of build123d and bd_warehouse, both licensed under the Apache License 2.0;
the files in `examples/` are ports of build123d's examples, and `src/template_render.inc` is
build123d's `template_render.js`. The material system is a port of bd_materials and
threejs_materials (both Apache License 2.0), with their catalog and presets generated from their
sources. Keep these projects' licenses and notices when distributing this code.

Other files have their own licenses:
- the bundled Relief SingleLine CAD font: SIL Open Font License
  (`data/fonts/reliefsingleline/OFL.txt`);
- lodepng, used for PNG files: zlib license (`third_party/lodepng/LICENSE`);
- xxHash, the operation cache's hash: BSD 2-Clause license (`third_party/xxhash/LICENSE`);
- the textures that `scripts/fetch-textures.sh` downloads: CC0 (ambientCG), MIT (AMD GPUOpen)
  and Apache License 2.0 (threejs_materials).
