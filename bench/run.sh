#!/usr/bin/env bash
# Reproduces every number of the C++/Python benchmark.
#
#   REPO=/path/to/build123d-cpp PY=/path/to/venv/bin/python ./run.sh [block ...]
#
# Needs: the repo's Release build (build/libbuild123d.a) and its .toolchain; a Python with
# build123d 0.13 and bd_warehouse 0.4. Blocks: env startup inprocess io wholerun suite flow
# share explain (default: all). Results go to results/, tables to results/report.md.
# Run it on an idle machine: the driver waits until under 8% of the CPUs are busy before each
# block. Takes about 50 minutes; the suite block alone about 25.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if [[ -z "${REPO:-}" && -f "$HERE/../include/build123d/build123d.hpp" ]]; then REPO="$(cd "$HERE/.." && pwd)"; fi
export REPO="${REPO:?set REPO to the build123d-cpp checkout, or run this from its bench/}"
export PY="${PY:?set PY to a Python with build123d 0.13 and bd_warehouse 0.4}"
"$HERE/build.sh"
"$PY" -I -B "$HERE/driver.py" "$@"
"$PY" -I -B "$HERE/report.py" > "$HERE/results/report.md"
echo "tables: $HERE/results/report.md"
