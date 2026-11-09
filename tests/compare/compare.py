"""Compare build123d-cpp against Python build123d on the same models.

Usage: python compare.py <path to compare_cases executable> [case-name]
(run with a Python environment where build123d is installed)
"""

import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

REL = 1e-6
ABS_GEOM = 1e-4


def close(a, b, rel=REL, abs_tol=1e-9):
    return abs(a - b) <= max(abs_tol, rel * max(abs(a), abs(b)))


def run(cmd, cwd=None):
    out = subprocess.run(cmd, capture_output=True, text=True, check=False, cwd=cwd)
    rows = {}
    for line in out.stdout.splitlines():
        line = line.strip()
        if line.startswith("{"):
            row = json.loads(line)
            rows[row["name"]] = row
    if out.returncode != 0:
        print(out.stderr[-2000:], file=sys.stderr)
    return rows


def diff_values(a, b, path="values"):
    """Differences between two JSON values: numbers within tolerance, the rest exactly."""
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None or isinstance(a, str) or isinstance(b, str):
        return [] if a == b and type(a) is type(b) else [f"{path}: {a!r} != {b!r}"]
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if a != a and b != b:  # both NaN
            return []
        return [] if close(a, b, REL, ABS_GEOM) else [f"{path}: {a:.9g} != {b:.9g}"]
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [f"{path}: length {len(a)} != {len(b)}"]
        problems = []
        for i, (x, y) in enumerate(zip(a, b)):
            problems += diff_values(x, y, f"{path}[{i}]")
        return problems
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return [f"{path}: keys {sorted(a)} != {sorted(b)}"]
        problems = []
        for key in a:
            problems += diff_values(a[key], b[key], f"{path}.{key}")
        return problems
    return [f"{path}: {type(a).__name__} {a!r} != {type(b).__name__} {b!r}"]


def diff(cpp, py):
    problems = []
    if "error" in cpp and "error" in py:
        return []  # both reject the model
    if "error" in cpp or "error" in py:
        return [f"cpp error: {cpp.get('error')}" if "error" in cpp else "", f"py error: {py.get('error')}" if "error" in py else ""]
    if "values" in cpp or "values" in py:
        if "values" not in cpp or "values" not in py:
            return ["one side returned values, the other a shape"]
        return diff_values(cpp["values"], py["values"])[:10]
    for key in ("solids", "faces", "edges", "vertices"):
        if cpp[key] != py[key]:
            problems.append(f"{key} {cpp[key]} != {py[key]}")
    for key in ("volume", "area", "length"):
        if not close(cpp[key], py[key]):
            problems.append(f"{key} {cpp[key]:.9g} != {py[key]:.9g}")
    for key in ("bbox", "center"):
        if any(not close(a, b, 1e-6, ABS_GEOM) for a, b in zip(cpp[key], py[key])):
            problems.append(f"{key} {[round(v, 5) for v in cpp[key]]} != {[round(v, 5) for v in py[key]]}")
    return [p for p in problems if p]


def main():
    exe = sys.argv[1]
    only = sys.argv[2:]
    # the C++ side reads the PBR textures of this environment's threejs_materials, unless told
    if not os.environ.get("BUILD123D_TEXTURE_DIR"):
        import threejs_materials

        assets = os.path.join(os.path.dirname(threejs_materials.__file__), "pbr_properties", "_assets")
        os.environ["BUILD123D_TEXTURE_DIR"] = assets
    cpp = run([exe, *only])
    py = run([sys.executable, os.path.join(HERE, "cases_py.py"), *only])
    names = list(py.keys()) + [n for n in cpp if n not in py]
    failures = 0
    for name in names:
        if name not in cpp or name not in py:
            print(f"MISSING  {name}: {'cpp' if name not in cpp else 'python'} side")
            failures += 1
            continue
        problems = diff(cpp[name], py[name])
        if problems:
            failures += 1
            print(f"DIFF     {name}: " + "; ".join(problems))
        else:
            print(f"ok       {name}")
    print(f"\n{len(names) - failures}/{len(names)} cases match")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
