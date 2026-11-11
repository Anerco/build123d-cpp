"""Runs every measurement of the C++/Python benchmark, serially, and writes results/*.json[l].

    python -I -B driver.py [block ...]     blocks: env startup inprocess io wholerun suite flow
                                           share explain (default: all, in that order)

Before each block the machine must be idle: under IDLE_BUSY of the CPUs busy over 2 s
(/proc/stat), or the driver waits and checks again. /proc/loadavg is recorded with every block.
"""

import json
import os
import platform
import statistics
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get("REPO") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = os.environ["PY"]
BIN = os.path.join(HERE, "bin")
RES = os.path.join(HERE, "results")
OUT = os.path.join(HERE, "out")
IDLE_BUSY = float(os.environ.get("IDLE_BUSY", "0.08"))  # fraction of all CPUs
WARMUP, MIN_RUNS, MAX_RUNS, TARGET_S = 2, 10, 100, 2.0
WHOLE_REPS = int(os.environ.get("WHOLE_REPS", "5"))
START_REPS = int(os.environ.get("START_REPS", "15"))

EXAMPLES = [
    "tea_cup", "vase", "heat_exchanger", "handle", "lego", "key_cap", "pillow_block", "din_rail",
    "pegboard_j_hook", "circuit_board", "intersecting_chamfers", "holes", "boxes_on_faces",
    "multiple_workplanes", "loft", "extrude", "clock", "roller_coaster", "canadian_flag",
    "custom_sketch_objects", "projection", "build123d_logo", "build123d_customizable_logo", "joints",
]
BDW = [
    "gear_spur", "gear_helical", "gear_worm", "thread_iso_external", "thread_iso_internal",
    "screw_shcs_simple", "screw_shcs_threaded", "nut_hex_simple", "nut_hex_threaded",
    "bearing_deep_groove", "flange_weld_neck", "sprocket", "o_ring", "openbuilds_cbeam_assembly",
]
OPS = ["box", "plate", "op_holes", "op_fillet", "op_text", "occt_holes", "occt_fillet"]
IO = ["export_step", "export_stl", "export_gltf", "import_step"]
SUITE = ["bdw_bearing", "bdw_bushing", "bdw_fastener", "bdw_flange", "bdw_gear", "bdw_o_rings",
         "bdw_open_builds", "bdw_pipe", "bdw_retaining_ring", "bdw_shaft_key", "bdw_sprocket", "bdw_thread"]


def models():
    """(group, name, C++ command prefix, Python model file, Python function)"""
    out = []
    for e in EXAMPLES:
        out.append(("example", e, [f"{BIN}/ex_{e}"], f"{HERE}/py/gen/examples_bench.py", e))
    for m in BDW:
        out.append(("bd_warehouse", m, [f"{BIN}/{m}"], f"{HERE}/py/models/{m}.py", "model"))
    for m in OPS:
        out.append(("operation", m, [f"{BIN}/{m}"], f"{HERE}/py/ops/{m}.py", "model"))
    return out


def cpu_busy(seconds=2.0):
    def read():
        with open("/proc/stat") as f:
            v = [int(x) for x in f.readline().split()[1:]]
        idle = v[3] + v[4]
        return idle, sum(v)

    i0, t0 = read()
    time.sleep(seconds)
    i1, t1 = read()
    return 1 - (i1 - i0) / max(1, t1 - t0)


def loadavg():
    with open("/proc/loadavg") as f:
        return f.read().split()[:3]


def wait_idle(block, log):
    waited = 0
    while True:
        busy = cpu_busy()
        if busy < IDLE_BUSY:
            entry = {"block": block, "loadavg": loadavg(), "cpu_busy": round(busy, 3), "waited_s": waited,
                     "time": time.strftime("%Y-%m-%d %H:%M:%S")}
            log.append(entry)
            print(f"[{block}] idle: busy {busy:.1%}, loadavg {' '.join(entry['loadavg'])}, waited {waited} s", flush=True)
            return entry
        print(f"[{block}] busy {busy:.1%}, loadavg {' '.join(loadavg())}: waiting", flush=True)
        time.sleep(15)
        waited += 17
        if waited > 1800:
            raise SystemExit("machine not idle for 30 min")


def run_json(cmd, env=None, timeout=1800):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env, cwd=OUT)
    rows = [json.loads(l) for l in p.stdout.splitlines() if l.startswith("{")]
    if p.returncode != 0 or not rows:
        return {"error": f"exit {p.returncode}: {p.stderr[-800:]}"}
    return rows[-1]


def timed_process(cmd, env=None):
    """Wall time (s) and peak RSS (KB, /usr/bin/time %M) of a fresh process."""
    with tempfile.NamedTemporaryFile("r", dir=OUT, suffix=".time") as tf:
        t0 = time.perf_counter()
        p = subprocess.run(["/usr/bin/time", "-f", "%M", "-o", tf.name, *cmd], capture_output=True, text=True,
                           env=env, cwd=OUT)
        wall = time.perf_counter() - t0
        rss = int(open(tf.name).read().split()[-1])
    if p.returncode != 0:
        raise RuntimeError(f"{cmd}: exit {p.returncode}: {p.stderr[-800:]}")
    return wall, rss


def summarize(times):
    s = sorted(times)
    q = statistics.quantiles(s, n=4) if len(s) >= 2 else [s[0]] * 3
    return {"n": len(s), "median": statistics.median(s), "p25": q[0], "p75": q[2], "min": s[0], "max": s[-1]}


def env_block():
    def out(cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, shell=isinstance(cmd, str)).stdout.strip()
        except OSError as e:
            return str(e)

    cpu = out("grep -m1 'model name' /proc/cpuinfo").split(":")[-1].strip()
    pkgs = out([PY, "-I", "-c", "import importlib.metadata as m, sys, json; print(json.dumps({'python': sys.version.split()[0], **{p: m.version(p) for p in ['build123d','bd_warehouse','cadquery-ocp-novtk','numpy','ocpsvg','ocp_gordon']}}))"])
    occt_cpp = out(f"grep -m1 '#define OCC_VERSION_COMPLETE' {REPO}/.toolchain/include/opencascade/Standard_Version.hxx")
    return {
        "cpu": cpu, "threads": os.cpu_count(), "kernel": platform.release(),
        "os": out("grep PRETTY_NAME /etc/os-release").split("=")[-1].strip('"'),
        "mem_total": out("grep MemTotal /proc/meminfo"),
        "cxx": out([f"{REPO}/.toolchain/bin/x86_64-conda-linux-gnu-c++", "--version"]).splitlines()[0],
        "library_flags": "-O3 -DNDEBUG (CMake Release, libbuild123d.a)",
        "program_flags": "-std=c++20 -O2 -DNDEBUG",
        "occt_cpp": occt_cpp.split()[-1].strip('"'),
        "occt_cpp_build": out(f"readelf -p .comment {REPO}/.toolchain/lib/libTKBO.so | grep GCC").split("]")[-1].strip(),
        "python_packages": json.loads(pkgs) if pkgs.startswith("{") else pkgs,
        "repo_commit": out(["git", "-C", REPO, "rev-parse", "--short", "HEAD"]),
        "repo_dirty": out(["git", "-C", REPO, "status", "--short"]) != "",
        "date": time.strftime("%Y-%m-%d %H:%M"),
    }


def startup_block(log):
    wait_idle("startup", log)
    py_imports = {
        "python -I -c pass": [PY, "-I", "-c", "pass"],
        "import build123d": [PY, "-I", "-c", "import build123d"],
        "import bd_warehouse": [PY, "-I", "-c", "import bd_warehouse"],
    }
    for mod in ["thread", "fastener", "gear", "bearing", "flange", "sprocket", "o_rings", "open_builds"]:
        py_imports[f"import bd_warehouse.{mod}"] = [PY, "-I", "-c", f"import bd_warehouse.{mod}"]
    cpp = {
        "empty C++ program linked to OCCT's libraries": [f"{BIN}/start_occt"],
        "C++ program start (no model)": [f"{BIN}/box", "start"],
        "C++ program start, bd_warehouse linked": [f"{BIN}/openbuilds_cbeam_assembly", "start"],
    }
    rows = {}
    for label, cmd in {**cpp, **py_imports}.items():
        timed_process(cmd)  # one untimed run: file cache
        walls, rsss = [], []
        for _ in range(START_REPS):
            w, r = timed_process(cmd)
            walls.append(w)
            rsss.append(r)
        rows[label] = {"wall_s": summarize(walls), "rss_kb": summarize(rsss)}
        print(f"  {label}: {statistics.median(walls)*1000:.1f} ms, {statistics.median(rsss)/1024:.0f} MB", flush=True)
    # the import alone, timed inside the process, and its biggest parts (-X importtime)
    inproc = []
    for _ in range(START_REPS):
        r = subprocess.run([PY, "-I", "-c", "import time; t=time.perf_counter(); import build123d; print(time.perf_counter()-t)"],
                           capture_output=True, text=True)
        inproc.append(float(r.stdout.strip()))
    rows["import build123d, timed in-process"] = {"wall_s": summarize(inproc)}
    r = subprocess.run([PY, "-I", "-X", "importtime", "-c", "import build123d"], capture_output=True, text=True)
    top = {}
    for line in r.stderr.splitlines():
        if not line.startswith("import time:") or "|" not in line:
            continue
        parts = [p.strip() for p in line[len("import time:"):].split("|")]
        try:
            cum = int(parts[1])
        except ValueError:
            continue
        name = parts[2]
        depth = (len(parts[2]) - len(parts[2].lstrip())) // 2
        root = name.strip().split(".")[0]
        if depth == 0 or "." not in name.strip():
            top[root] = max(top.get(root, 0), cum)
    rows["importtime_top_us"] = dict(sorted(top.items(), key=lambda kv: -kv[1])[:15])
    return rows


def inprocess_block(log, selected=None):
    rows = []
    for group, name, cpp_cmd, py_file, py_func in models():
        if selected and name not in selected:
            continue
        wait_idle(f"inprocess {name}", log)
        args = [str(WARMUP), str(MIN_RUNS), str(MAX_RUNS), str(TARGET_S)]
        c = run_json([*cpp_cmd, "time", *args])
        p = run_json([PY, "-I", "-B", f"{HERE}/py/pybench.py", "time", py_file, py_func, *args])
        row = {"group": group, "name": name, "cpp": c, "py": p, "loadavg": loadavg()}
        rows.append(row)
        cm = statistics.median(c["times_ms"]) if "times_ms" in c else float("nan")
        pm = statistics.median(p["times_ms"]) if "times_ms" in p else float("nan")
        print(f"  {group:12s} {name:28s} C++ {cm:9.2f} ms  Py {pm:9.2f} ms  Py/C++ {pm/cm:5.2f}", flush=True)
    return rows


def wholerun_block(log, selected=None):
    rows = []
    for group, name, cpp_cmd, py_file, py_func in models():
        if selected and name not in selected:
            continue
        wait_idle(f"wholerun {name}", log)
        c_cmd = [*cpp_cmd, "run", f"{OUT}/{name}_cpp.step"]
        p_cmd = [PY, "-I", "-B", f"{HERE}/py/pybench.py", "run", py_file, py_func, f"{OUT}/{name}_py.step"]
        timed_process(c_cmd)  # untimed: file cache, in both
        timed_process(p_cmd)
        res = {"cpp": {"wall": [], "rss": []}, "py": {"wall": [], "rss": []}}
        for _ in range(WHOLE_REPS):  # alternated, so that drift hits both
            for lang, cmd in (("cpp", c_cmd), ("py", p_cmd)):
                w, r = timed_process(cmd)
                res[lang]["wall"].append(w)
                res[lang]["rss"].append(r)
        sizes = {lang: os.path.getsize(f"{OUT}/{name}_{lang}.step") for lang in ("cpp", "py")}
        rows.append({"group": group, "name": name, **res, "step_bytes": sizes, "loadavg": loadavg()})
        cw, pw = statistics.median(res["cpp"]["wall"]), statistics.median(res["py"]["wall"])
        print(f"  {name:28s} C++ {cw:6.3f} s {statistics.median(res['cpp']['rss'])/1024:5.0f} MB   "
              f"Py {pw:6.3f} s {statistics.median(res['py']['rss'])/1024:5.0f} MB", flush=True)
    return rows


def io_block(log):
    rows = []
    env = dict(os.environ, BENCH_OUT=OUT, BENCH_STEP_IN=f"{OUT}/import_input.step")
    # one input file for both: op_holes' plate, written by the C++ side
    subprocess.run([f"{BIN}/op_holes", "run", f"{OUT}/import_input.step"], check=True, capture_output=True, cwd=OUT)
    for what in IO:
        wait_idle(f"io {what}", log)
        args = [str(WARMUP), str(MIN_RUNS), str(MAX_RUNS), str(TARGET_S)]
        c = run_json([f"{BIN}/op_io", what, "time", *args], env=env)
        p = run_json([PY, "-I", "-B", f"{HERE}/py/pybench.py", "time", f"{HERE}/py/ops/op_io.py", what, *args], env=env)
        rows.append({"group": "io", "name": what, "cpp": c, "py": p, "loadavg": loadavg()})
        cm, pm = statistics.median(c["times_ms"]), statistics.median(p["times_ms"])
        print(f"  {what:28s} C++ {cm:9.2f} ms  Py {pm:9.2f} ms  Py/C++ {pm/cm:5.2f}", flush=True)
    return rows


def suite_block(log, warmup=1, runs=1):
    rows = []
    for prefix in SUITE:
        wait_idle(f"suite {prefix}", log)
        for lang, cmd in (("cpp", [f"{BIN}/suite_bdw", prefix + "_", str(warmup), str(runs)]),
                          ("py", [PY, "-I", "-B", f"{HERE}/py/suite.py", REPO, prefix + "_", str(warmup), str(runs)])):
            t0 = time.perf_counter()
            p = subprocess.run(["/usr/bin/time", "-f", "%M", "-o", f"{OUT}/suite.time", *cmd], capture_output=True,
                               text=True, cwd=OUT)
            wall = time.perf_counter() - t0
            cases = [json.loads(l) for l in p.stdout.splitlines() if l.startswith("{")]
            rss = int(open(f"{OUT}/suite.time").read().split()[-1])
            rows.append({"module": prefix, "lang": lang, "process_wall_s": wall, "rss_kb": rss, "cases": cases,
                         "exit": p.returncode, "loadavg": loadavg()})
            total = sum(sum(c["times_ms"]) / max(1, len(c["times_ms"])) for c in cases) / 1000
            print(f"  {prefix:20s} {lang:3s} {len(cases):4d} cases, cases {total:7.2f} s, process {wall:7.2f} s", flush=True)
    return rows


SHARE = ["box", "plate", "op_holes", "op_text", "roller_coaster", "build123d_logo", "handle", "din_rail",
         "tea_cup", "heat_exchanger", "gear_spur", "thread_iso_external", "screw_shcs_threaded",
         "nut_hex_threaded", "bearing_deep_groove", "openbuilds_cbeam_assembly"]


def share_block(log):
    """Python builds split into time inside OCP (OCCT) and the rest (py/occt_share.py)."""
    rows = []
    for group, name, cpp_cmd, py_file, py_func in models():
        if name not in SHARE:
            continue
        wait_idle(f"share {name}", log)
        r = run_json([PY, "-I", "-B", f"{HERE}/py/occt_share.py", py_file, py_func, "5"])
        rows.append({"group": group, "name": name, **r})
        print(f"  {name:28s} monitored {r.get('monitored_wall_ms', float('nan')):9.2f} ms, in OCP "
              f"{r.get('ocp_ms', float('nan')):9.2f} ms, {r.get('ocp_calls')} OCP calls", flush=True)
    return rows


FLOW = ["bdw_fastener", "bdw_thread", "bdw_bearing", "bdw_gear"]


def flow_block(log):
    """Modules of the suite run as compare.py runs them (py/compare_flow.py); needs suite.json."""
    wait_idle("flow", log)
    p = subprocess.run([PY, "-I", "-B", f"{HERE}/py/compare_flow.py", REPO, *FLOW], capture_output=True, text=True)
    with open(os.path.join(RES, "compare_flow.jsonl"), "w") as f:
        f.write(p.stdout)
    print(p.stdout, p.stderr[-500:], flush=True)


EXPLAIN = {
    # model: (C++ program, Python file, Python function, shim)
    "thread_iso_external": ("thread_iso_external", "py/models/thread_iso_external.py", "model", "count_calls"),
    "nut_hex_threaded": ("nut_hex_threaded", "py/models/nut_hex_threaded.py", "model", "count_calls"),
    "bearing_deep_groove": ("bearing_deep_groove", "py/models/bearing_deep_groove.py", "model", "count_calls"),
    "op_text": ("op_text", "py/ops/op_text.py", "model", "count_calls"),
    "heat_exchanger": ("ex_heat_exchanger", "py/gen/examples_bench.py", "heat_exchanger", "count_ops"),
    "loft": ("ex_loft", "py/gen/examples_bench.py", "loft", "count_ops"),
    "key_cap": ("ex_key_cap", "py/gen/examples_bench.py", "key_cap", "count_ops"),
}


def explain_block(log):
    """The checks behind the outliers, as text: OCCT calls counted in C++ (LD_PRELOAD shims, 5
    builds) and in Python (py/ocp_calls.py, per build)."""
    out = []
    for name, (cpp, py_file, func, shim) in EXPLAIN.items():
        wait_idle(f"explain {name}", log)
        env = dict(os.environ, LD_PRELOAD=f"{BIN}/{shim}.so")
        c = subprocess.run([f"{BIN}/{cpp}", "time", "0", "5", "5", "0"], capture_output=True, text=True, env=env, cwd=OUT)
        p = subprocess.run([PY, "-I", "-B", f"{HERE}/py/ocp_calls.py", f"{HERE}/{py_file}", func, "3", "400"],
                           capture_output=True, text=True, cwd=OUT)
        out += [f"== {name}", "C++, totals over 5 builds:", *[l for l in c.stderr.splitlines() if l.startswith("count_")],
                "Python, per build (all OCP calls, by time):", *p.stdout.splitlines()[:16]]
        if shim == "count_ops":
            ops = ("BRepAlgoAPI_Fuse", "BRepAlgoAPI_Cut", "BRepAlgoAPI_Common", "ShapeUpgrade_UnifySameDomain",
                   "ShapeFix_Shape", "BRepFilletAPI_MakeFillet")
            out += ["Python, the operations the C++ shim counts:",
                    *[l for l in p.stdout.splitlines() if l.strip().endswith(ops)]]
        out.append("")
        print(f"  {name}", flush=True)
    with open(os.path.join(RES, "explain.txt"), "w") as f:
        f.write("\n".join(out))


def save(name, data):
    os.makedirs(RES, exist_ok=True)
    if os.environ.get("TAG"):
        name = name.replace(".json", f".{os.environ['TAG']}.json")
    elif os.environ.get("ONLY"):
        name = name.replace(".json", ".partial.json")
    with open(os.path.join(RES, name), "w") as f:
        json.dump(data, f, indent=1)


def main():
    os.makedirs(OUT, exist_ok=True)
    blocks = sys.argv[1:] or ["env", "startup", "inprocess", "io", "wholerun", "suite", "flow", "share", "explain"]
    only = os.environ.get("ONLY", "").split(",") if os.environ.get("ONLY") else None
    log = []
    for b in blocks:
        print(f"== {b}", flush=True)
        if b == "env":
            save("env.json", env_block())
        elif b == "startup":
            save("startup.json", startup_block(log))
        elif b == "inprocess":
            save("inprocess.json", inprocess_block(log, only))
        elif b == "wholerun":
            save("wholerun.json", wholerun_block(log, only))
        elif b == "io":
            save("io.json", io_block(log))
        elif b == "suite":
            save("suite.json", suite_block(log))
        elif b == "share":
            save("share.json", share_block(log))
        elif b == "flow":
            flow_block(log)
        elif b == "explain":
            explain_block(log)
        save(f"idle_log_{b}.json", log)
        log = []


if __name__ == "__main__":
    main()
