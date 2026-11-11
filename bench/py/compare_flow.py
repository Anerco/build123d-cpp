"""Times a module's comparison cases the way tests/compare/compare.py runs them: the C++
side is build/compare_cases <names>, the Python side is `python cases_py.py <names>` (which
imports every case module, then builds each case once and computes its metrics).

    python -I -B compare_flow.py <repo> <module> [...]    e.g. bdw_fastener bdw_thread

Prints one JSON line per module and side: wall time, peak RSS, cases printed. Case names come
from results/suite.json (driver.py's suite block). cases_py.py runs without -I (it needs its directory on
sys.path, as compare.py runs it) but with -B, so nothing is written into the repo.
"""

import json
import os
import subprocess
import sys
import tempfile
import time

PY = sys.executable


def timed(cmd, cwd):
    with tempfile.NamedTemporaryFile("r", suffix=".time") as tf:
        t0 = time.perf_counter()
        p = subprocess.run(["/usr/bin/time", "-f", "%M", "-o", tf.name, *cmd], capture_output=True, text=True, cwd=cwd)
        wall = time.perf_counter() - t0
        rss = int(open(tf.name).read().split()[-1])
    rows = [json.loads(l) for l in p.stdout.splitlines() if l.startswith("{")]
    return wall, rss, rows


def main():
    repo, modules = sys.argv[1], sys.argv[2:]
    exe = os.path.join(repo, "build", "compare_cases")
    cases_py = os.path.join(repo, "tests", "compare", "cases_py.py")
    with tempfile.TemporaryDirectory() as cwd:
        suite = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "suite.json")))
        for module in modules:
            names = [c["name"] for r in suite if r["module"] == module and r["lang"] == "cpp" for c in r["cases"]]
            for side, cmd in (("cpp", [exe, *names]), ("py", [PY, "-B", cases_py, *names])):
                wall, rss, rows = timed(cmd, cwd)
                print(json.dumps({"module": module, "side": side, "cases": len(names), "printed": len(rows),
                                  "wall_s": wall, "rss_kb": rss}), flush=True)


if __name__ == "__main__":
    main()
