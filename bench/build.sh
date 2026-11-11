#!/usr/bin/env bash
# Compiles the C++ benchmark programs into bin/ against the repo's Release build of
# libbuild123d.a, and writes the Python copy of the examples. Changes nothing in the repo.
#   REPO=/path/to/build123d-cpp PY=/path/to/python ./build.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# the repo: $REPO, else the directory above this one if it is the repo (bench/ inside it)
if [[ -z "${REPO:-}" && -f "$HERE/../include/build123d/build123d.hpp" ]]; then REPO="$(cd "$HERE/.." && pwd)"; fi
REPO="${REPO:-$(cd "$(dirname "$0")/.." && pwd)}"
TC="$REPO/.toolchain"
CXX="$TC/bin/x86_64-conda-linux-gnu-c++"
PY="${PY:?set PY to a Python with build123d 0.13 and bd_warehouse 0.4}"
CXXFLAGS=(-std=c++20 -O2 -DNDEBUG -I"$REPO/include" -isystem "$TC/include/opencascade")
LIBS=(-L"$REPO/build" -lbuild123d $(ls "$TC"/lib/libTK*.so) "$TC/lib/lib3mf.so" "$TC/lib/libcurl.so" -ldl
      -Wl,-rpath,"$REPO/build:$TC/lib")
mkdir -p "$HERE/bin" "$HERE/gen/examples" "$HERE/py/gen" "$HERE/out"

compile() {  # out sources...
    local out="$1"; shift
    "$CXX" "${CXXFLAGS[@]}" "$@" -o "$HERE/bin/$out" "${LIBS[@]}"
}

JOBS="${JOBS:-8}"
throttle() { while (( $(jobs -rp | wc -l) >= JOBS )); do wait -n; done; }
# the 24 examples (examples/01_quickstart.cpp is the README's plate, measured as "plate")
for src in "$REPO"/examples/*.cpp; do
    name="$(basename "$src" .cpp)"
    [[ "$name" == 01_quickstart ]] && continue
    cp "$src" "$HERE/gen/examples/$name.cpp"
    cp "$HERE/cpp/examples_shim/show.hpp" "$HERE/gen/examples/show.hpp"
    compile "ex_$name" -I"$HERE/cpp/examples_shim" "$HERE/gen/examples/$name.cpp" \
        "$HERE/cpp/examples_shim/example_driver.cpp" &
    throttle
done
for src in "$HERE"/cpp/models/*.cpp "$HERE"/cpp/ops/*.cpp; do
    compile "$(basename "$src" .cpp)" "$src" &
    throttle
done

# an empty program linked to OCCT's libraries alone (what loading them costs at start)
"$CXX" -O2 "$HERE/cpp/start_occt.cpp" -o "$HERE/bin/start_occt" -Wl,--no-as-needed $(ls "$TC"/lib/libTK*.so) \
    -Wl,-rpath,"$TC/lib" &
# LD_PRELOAD shims that count OCCT calls (driver.py's explain block)
for shim in count_calls count_ops; do
    "$CXX" -std=c++20 -O2 -shared -fPIC "$HERE/cpp/$shim.cpp" -o "$HERE/bin/$shim.so" -ldl &
done
# the bd_warehouse comparison cases with a timing main instead of compare_cases' main
compile suite_bdw -I"$REPO/tests/compare" -I"$REPO/examples" "$REPO"/tests/compare/cases_bdw_*.cpp \
    "$HERE/cpp/suite_main.cpp" &
wait

"$PY" -I "$HERE/py/gen_examples.py" "$REPO" "$HERE/py/gen/examples_bench.py"
echo "built $(ls "$HERE/bin" | wc -l) programs"
