# build123d-cpp vs Python build123d: benchmark

Measures the C++ port against Python build123d 0.13 / bd_warehouse 0.4 on the same models.
It changes nothing in the repo: it compiles its own programs against the repo's Release build
(`build/libbuild123d.a`, the `.toolchain`) and copies of `examples/*.cpp`.

    PY=/path/to/venv/bin/python REPO=/path/to/build123d-cpp ./run.sh            # everything, ~50 min
    PY=... ./run.sh startup inprocess                                            # some blocks
    ONLY=tea_cup,gear_spur PY=... ./run.sh inprocess                             # some models (→ *.partial.json)

`REPO` defaults to the directory above this one if that is the repo (so the folder can live in
the repo as `bench/`), else it must be given. Run it on an idle machine: before every
model the driver waits until under 8% of the CPUs have been busy for 2 s, and logs
`/proc/loadavg` (`results/idle_log_*.json`). Tables: `results/report.md`; per-model numbers:
`results/summary.csv`; raw timings: `results/*.json`.

## What is measured

| block | what | how |
|---|---|---|
| startup | a C++ program's start vs `python -I -c "import build123d"` (and each bd_warehouse module) | 15 fresh processes after one untimed; wall time and peak RSS (`/usr/bin/time -f %M`) |
| inprocess | each model's build inside a running process | 2 untimed builds, then ≥10 builds and ≥2 s (≤100); median and IQR; the previous result is destroyed outside the timed region |
| io | STEP, STL, glTF export and STEP import of one part | same as inprocess; the mesh is removed before every export, outside the timed region |
| wholerun | a fresh process that builds the model and writes one STEP file | 5 runs each, C++ and Python alternated, after one untimed run of each; wall and peak RSS |
| suite | every bd_warehouse comparison case (2,404), in-process | `cpp/suite_main.cpp` linked with `tests/compare/cases_bdw_*.cpp`, `py/suite.py` with `cases_bdw_*_py.py`: 1 untimed + 1 timed build per case |
| flow | four bd_warehouse modules run as `compare.py` runs them | `build/compare_cases <names>` vs `python cases_py.py <names>`, whole processes |
| share | how many OCCT calls a Python build makes, and their time | `py/occt_share.py` (sys.monitoring) |
| explain | the checks behind the outliers | OCCT calls counted in C++ (`cpp/count_calls.cpp`, `cpp/count_ops.cpp`, LD_PRELOAD) and in Python (`py/ocp_calls.py`) → `results/explain.txt` |

Models (each with a C++ and a Python twin that build the same geometry; volume, area and face
count are compared in every in-process run, column "same geometry"):

- the 24 build123d examples: `examples/*.cpp` compiled with `cpp/examples_shim/` (show() keeps
  the shapes, the example's own `export_step` is dropped, `main` becomes a function), and
  `tests/compare/examples_py.py` (copied by `py/gen_examples.py` without its metrics);
- 14 bd_warehouse parts: `cpp/models/`, `py/models/` (written by `py/gen_bdw_models.py`);
- single operations: `cpp/ops/`, `py/ops/` — the README's plate, a box, 225 holes in one boolean,
  a fillet of 120 edges, a line of text, export/import (`op_io`), and two controls that call
  OCCT directly without either library (`occt_holes`, `occt_fillet`), to compare the two OCCT
  builds.

Equal work: export is counted in both (whole run) or in neither (build time); the warm-up is
the same; Python's import and C++'s process start are outside the build time. The suite's value
cases are *not* equal work: several Python twins build the part once per value they read
(bearing `*_default_parts` 7 times, worm gear `*_values` 4 times) where the C++ twin builds it
once, so the report gives shape cases and value cases separately.

## Files

- `run.sh`: builds, runs the driver, writes the report. `build.sh`: compiles into `bin/`.
- `driver.py`: the measurements; `report.py`: `results/report.md` and `results/summary.csv`.
- `cpp/bench.hpp` and `py/pybench.py`: the two harnesses (same rules).
- `py/perf_by_function.py`: `perf` samples of one OCCT library by function (used by hand:
  `perf record -F 1000 -o x.data <cmd>`, then this script), to compare where the two OCCT
  builds spend their time.
- `results/repeat/`: a second in-process and io pass, for run-to-run variation.
- `results/superseded/`: whole runs measured before a fix (the C++ harness also computed the
  shapes' volume and area after exporting; Python's didn't). Not used in the report.
