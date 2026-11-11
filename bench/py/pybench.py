"""Timing harness for the Python side, the twin of cpp/bench.hpp.

    python -I -B pybench.py time <model.py> <func> <warmup> <min_runs> <max_runs> <target_s>
    python -I -B pybench.py run  <model.py> <func> <out.step>

<func> returns a Shape or a dict of named shapes. The model file is loaded by path (with -I,
neither its directory nor the current one is on sys.path). The previous result is released
before the timed region, so its destruction isn't timed, as in C++.
"""

import importlib.util
import json
import os
import sys
import time

sys.dont_write_bytecode = True


def load(path):
    sys.path.insert(0, os.path.dirname(os.path.abspath(path)))
    spec = importlib.util.spec_from_file_location("bench_model", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def named(result, name):
    if isinstance(result, dict):
        return list(result.items())
    return [(name, result)]


def metrics(shapes):
    return [
        {"name": n, "volume": s.volume, "area": s.area, "faces": len(s.faces())} for n, s in shapes
    ]


def time_mode(build, name, prepare, warmup, min_runs, max_runs, target_s):
    warm = []
    last = None
    for _ in range(warmup):
        if prepare:
            prepare()
        last = None
        t0 = time.perf_counter()
        last = build()
        warm.append(1000 * (time.perf_counter() - t0))
    times = []
    total = 0.0
    while len(times) < max_runs and (len(times) < min_runs or total < 1000 * target_s):
        if prepare:
            prepare()
        last = None  # released here, outside the timed region
        t0 = time.perf_counter()
        last = build()
        t1 = time.perf_counter()
        times.append(1000 * (t1 - t0))
        total += times[-1]
    shapes = named(last, name) if last is not None else []
    print(json.dumps({"lang": "py", "warmup_ms": warm, "times_ms": times, "shapes": metrics(shapes)}), flush=True)


def run_mode(build, name, out):
    from build123d import Compound, export_step

    shapes = named(build(), name)
    to_export = shapes[0][1] if len(shapes) == 1 else Compound([s for _, s in shapes])
    export_step(to_export, out)
    print(json.dumps({"lang": "py"}), flush=True)


def main():
    mode, path, func = sys.argv[1], sys.argv[2], sys.argv[3]
    module = load(path)
    # the examples module maps example names to functions (loft -> loft_example, ...)
    build = getattr(module, "EXAMPLES", {}).get(func) or getattr(module, func)
    prepare = getattr(module, "prepare", None)
    name = os.path.splitext(os.path.basename(path))[0] if func == "model" else func
    if mode == "time":
        warmup, min_runs, max_runs = (int(x) for x in sys.argv[4:7])
        time_mode(build, name, prepare, warmup, min_runs, max_runs, float(sys.argv[7]))
    elif mode == "run":
        run_mode(build, name, sys.argv[4])
    else:
        raise SystemExit(f"unknown mode {mode}")


if __name__ == "__main__":
    main()
