# build123d-cpp

A C++ port of the **algebra mode** of [build123d](https://github.com/gumyr/build123d), the
Python CAD library, built directly on [OpenCASCADE](https://dev.opencascade.org/) (OCCT).

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
plate = Box(80, 60, 10) - [loc * Cylinder(3, 10) for loc in GridLocations(60, 40, 2, 2)]
plate = fillet(plate.edges().filter_by(Axis.Z), 5)
plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10)
```

![Models built by the C++ examples](docs/examples.png)

*Some of the ported examples, as written by the C++ programs in `examples/`
(rendered with `scripts/render_examples.py`).*

## What it covers

- **Geometry**: `Vector`, `Axis`, `Location`/`Pos`/`Rot`, `Plane` (all named planes),
  `BoundBox`, `Color`, unit constants (`MM`, `CM`, `M`, `IN`, `FT`, `THOU`).
- **Topology**: `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` and the
  algebra types `Curve`, `Sketch`, `Part`, with build123d's methods (positions, tangents and
  locations along curves, normals, intersections, projection, splitting, offsets, …).
- **Algebra**: `+`, `-`, `&` (and `+=`, `-=`, `&=`) with build123d's fuse/cut/clean
  semantics; placement with `Location * shape`, `Plane * shape` and
  `locations * shape`; `%` (tangent) and `^` (location) on curves.
- **Selectors**: `ShapeList` with `sort_by`, `group_by`, `filter_by`,
  `filter_by_position`, the `> < >> << |` operators, set operations and Python-style
  negative indices.
- **Objects**: 1D: `Line`, `Polyline`, `Spline`, `Bezier`, `BSpline`, `CenterArc`,
  `ThreePointArc`, `RadiusArc`, `SagittaArc`, `TangentArc`, `JernArc`,
  `EllipticalCenterArc`, `EllipticalStartArc`, `ParabolicCenterArc`, `HyperbolicCenterArc`,
  `Helix`, `PolarLine`, `FilletPolyline`, `IntersectingLine`. 2D: `Circle`, `Ellipse`,
  `Rectangle`, `RectangleRounded`, `Polygon`, `RegularPolygon`, `Trapezoid`, `Triangle`,
  `SlotArc`, `SlotCenterPoint`, `SlotCenterToCenter`, `SlotOverall`, `Superellipse`,
  `Text`. 3D: `Box`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `Wedge`, `Hole`,
  `CounterBoreHole`, `CounterSinkHole`, `ConvexPolyhedron`.
- **Locations**: `Locations`, `GridLocations`, `PolarLocations`, `HexLocations`.
- **Operations**: `extrude` (amount, both, taper, direction, until next/last/…),
  `revolve`, `loft`, `sweep` (multisection, Frenet, binormal), `section`, `thicken`,
  `draft`, `fillet`/`chamfer` (3D edges, 2D and 1D vertices), `offset` (1D, 2D, 3D
  hollowing), `mirror`, `scale`, `split`, `bounding_box`, `make_face`, `make_hull`,
  `trace`, `project_workplane`.
- **Import/export**: STEP (assemblies with labels and colors), STL, BREP, glTF/GLB;
  import of STEP, STL and BREP.

## How it was checked

The port was checked against Python build123d 0.13.0 itself:

- `tests/compare` builds the same 155 models in C++ and in Python and compares volume,
  area, total edge length (to 1e-6 relative), the number of solids, faces, edges and
  vertices, and the bounding box and center of mass. All 155 match.
- 22 of build123d's official algebra examples are ported in `examples/`. All 39 objects
  they show match their Python originals the same way. The heat exchanger example also
  passes its own built-in volume check.
- `tests/test_main.cpp` has unit tests for the C++ API (math, selectors, errors, static
  result types, import/export round trips including STEP assembly labels and colors).

Results with different OCCT versions (Python build123d itself runs on OCCT 8.0.1):

| OCCT | unit tests | 155 models | 39 example objects |
|---|---|---|---|
| 8.0.0 | 16/16 | 155 | 39 |
| 7.9.3 | 16/16 | 155 | 39 |
| 7.7.2 | 16/16 | 148 | 39 |
| 7.6.3 | 16/16 | 149 | 36 |

The differences with 7.6 and 7.7 come from OCCT: those versions split some edges and
faces differently. For example, 2D offsets keep their arcs in two pieces and some fused
faces aren't merged. 7.6 also places one glyph differently when wrapping text onto a
sphere. Volumes and areas still agree.

## Building

Requirements:
- CMake ≥ 3.16.
- A C++17 compiler. The examples and tests need C++20, because they use designated
  initializers.
- OCCT 7.6 or newer with its CMake config files.

Builds were tested on Linux x86-64 with GCC.

```sh
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/occt
cmake --build build
./build/examples/tea_cup        # prints its properties and writes tea_cup.step
./build/build123d_tests
```

If you don't have OCCT, `scripts/setup-toolchain.sh` creates a self-contained toolchain
(OCCT 8.0 without VTK, CMake, Ninja and GCC from conda-forge) in `./.toolchain` without
root access, and prints the commands to build with it:

```sh
scripts/setup-toolchain.sh
.toolchain-bin/micromamba run -p ./.toolchain cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=$PWD/.toolchain
.toolchain-bin/micromamba run -p ./.toolchain cmake --build build
```

To use the library from another CMake project, either `add_subdirectory()` this
directory or install it (`cmake --install build --prefix …`), then:

```cmake
target_link_libraries(my_app PRIVATE build123d::build123d)
```

### Comparing with Python build123d

With a Python environment that has `build123d==0.13.0` installed:

```sh
python tests/compare/compare.py build/compare_cases           # 155 models
python tests/compare/compare_examples.py build/examples       # 22 examples
```

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
| `curve @ 0.5` | `curve.position_at(0.5)` |
| `curve % 0.5`, `curve ^ 0.5` | `curve % 0.5`, `curve ^ 0.5` |
| `part.volume`, `edge.length`, `vertex.X` | `part.volume()`, `edge.length()`, `vertex.X()` |
| `faces()[-1]`, `faces().last` | `faces()[-1]`, `faces().last()` |
| `edges().filter_by(lambda e: e.length > 1)` | `edges().filter_by([](const Edge& e) { return e.length() > 1; })` |
| `edges().group_by(Edge.radius)` | `edges().group_by([](const Edge& e) { return e.radius(); })` |
| `edges() > Axis.Z`, `>>`, `<<`, `\|` | the same operators |
| `[loc * Circle(1) for loc in GridLocations(5, 5, 2, 2)]` | `GridLocations(5, 5, 2, 2) * Circle(1)` |
| `Plane(origin=(0, 0, 5), z_dir=(1, 0, 0))` | `Plane({0, 0, 5}, std::nullopt, {1, 0, 0})` |
| `Plane(origin, x_dir=x, y_dir=y)` | `Plane::from_x_y(origin, x, y)` |
| `Rot(Z=45)` | `Rot(0, 0, 45)` |
| `Pos(*point)` | `Pos(point)` |
| `Compound(children=[a, b])` | `Compound::make_assembly({a, b})` |
| `with SkipClean(): ...` | `{ SkipClean skip; ... }` |
| `show(part)` | `export_step(part, "part.step")` (or STL/glTF) |
| `ValueError` | `std::invalid_argument`; OCCT failures become `std::runtime_error` |

Option structs use C++20 designated initializers, which must follow the order the fields
are declared in (see `operations.hpp` and `objects.hpp`).

### Static types

The algebra keeps track of dimensions at compile time:

- `Box + Sphere` is a `Part`, `Circle - Rectangle` is a `Sketch` and `Line + Arc` is a
  `Curve`.
- Adding shapes of different dimensions (`Box(1, 1, 1) + Circle(1)`) does not compile.
- Placement keeps the type: `Pos(1, 0) * Box(…)` is a `Part`, and `Rot(…) * edge` is an
  `Edge`.
- `Part`, `Sketch` and `Curve` don't convert into each other.
- `fillet`/`chamfer` of vertices return a `Compound`, which converts to `Sketch` or `Curve`
  (checked when it runs).

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

### Differences from Python build123d

These are deliberate:

- `Edge::param_at_point` returns the parameter along the edge's orientation.
- `Wire::location_at` with `FrameMethod::CORRECTED` normalizes positions correctly.
- 2D fillets of wires and faces use OCCT's 2D fillet algorithms.
- `Edge::find_intersection_points` uses a distance query, which also works in 3D.
- `Plane::intersect(Plane)` returns no axis for coplanar planes.
- `IntersectingLine` ignores intersections behind its start point.

### Not ported

- Builder mode (`BuildPart`, `BuildSketch`, `BuildLine`, `Mode`).
- Joints.
- Drafting and dimensions (`DimensionLine`, `TechnicalDrawing`, …).
- SVG/DXF import and export, OBJ/3MF export.
- These objects: `BlendCurve`, `ConstrainedArcs`, `ConstrainedLines`, `DoubleTangentArc`,
  `Airfoil`.
- Arc limits given as shapes. The center arcs take only a numeric `arc_size`.
- These operations: `full_round`, `make_brake_formed`, `project`, `pack`.
- Mass properties with materials and units, `OrientedBoundBox`, `Matrix`.

## Layout

```
include/build123d/   public headers (build123d.hpp includes them all)
src/                 implementation
examples/            ports of build123d's algebra examples; show.hpp prints results
tests/test_main.cpp  unit tests
tests/compare/       comparison with Python build123d
scripts/             toolchain setup; render_examples.py draws docs/examples.png
```

## License

This is a port of build123d, which is licensed under the Apache License 2.0; the files in
`examples/` are ports of build123d's examples. Keep build123d's license and notices when
distributing this code.
