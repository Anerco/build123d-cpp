"""Times the bd_warehouse comparison cases (tests/compare/cases_bdw_*_py.py) one by one, the
twin of cpp/suite_main.cpp.

    python -I -B suite.py <repo> <prefix> <warmup> <runs>

Only the case modules whose cases start with <prefix> are imported. What is timed is the
case's function alone (the shape or the values it returns), not the metrics compare.py
compares. The previous result is released before the timed region.
"""

import glob
import importlib
import json
import os
import sys
import time

sys.dont_write_bytecode = True


def main():
    repo, prefix, warmup, runs = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    here = os.path.join(repo, "tests", "compare")
    sys.path.insert(0, here)
    cases = {}
    for path in sorted(glob.glob(os.path.join(here, "cases_bdw_*_py.py"))):
        module = os.path.splitext(os.path.basename(path))[0]
        area = "bdw_" + module[len("cases_bdw_") : -len("_py")]
        if not (area.startswith(prefix) or prefix.startswith(area)):
            continue
        cases.update(importlib.import_module(module).CASES)
    for name, build in cases.items():
        if not name.startswith(prefix):
            continue
        times, error, volume, result = [], None, 0.0, None
        for i in range(warmup + runs):
            result = None
            t0 = time.perf_counter()
            try:
                result = build()
            except Exception as exc:  # pylint: disable=broad-exception-caught
                error = f"{type(exc).__name__}: {exc}"
            t1 = time.perf_counter()
            if i >= warmup:
                times.append(1000 * (t1 - t0))
        kind = "value" if type(result).__name__ == "V" else "shape"
        if kind == "shape" and error is None and result is not None:
            shape = result
            if isinstance(shape, (list, tuple)):
                from build123d import Compound

                shape = Compound(list(shape))
            volume = shape.volume
        print(json.dumps({"name": name, "kind": kind, "times_ms": times, "volume": volume, "error": error}), flush=True)


if __name__ == "__main__":
    main()
