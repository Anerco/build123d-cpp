#!/usr/bin/env python3
"""The operation cache's gate (docs/operation-cache.md, "Testing it").

The unit tests, the examples and the comparison cases, run with the cache off, then cold (an
empty folder), then warm (the folder the cold run filled), keeping every operation (min_ms 0):
tests/gate, linked into each of them, turns the cache on from BUILD123D_OPCACHE_DIR. Then cold
and warm again in a folder of their own with the stores deferred (BUILD123D_OPCACHE_DEFERRED),
as a host keeps them, for the unit tests and the cases, which flush as they end; the examples,
whose sources stay as they are, keep theirs as each operation ends there too. Each
comparison case's shape gets a line with a hash of its bytes (BUILD123D_SHOW_BREP), and each
file an example writes is hashed (a STEP without its FILE_NAME line).

Some of OCCT's results depend on where the heap puts things: maps keyed by address, so a
run differs from the next one under ASLR, or under other malloc settings. Turning the cache
on moves the heap too. So the gate first runs everything with the cache off under several
heap layouts, and calls a line *stable* if every one of those runs gives the same bytes.
Every stable line must be the same bytes cold and warm. One that is not gets a second look:
its case or example run alone with the cache off under 16 more heap layouts, then, if none
gives what the cache gave, the whole program (all the cases, or the example) under 4 more,
since a case alone meets another heap than among the others; if one of those gives what the
cache gave, the line was not stable after all. Each run has a temporary folder of its own
(TMPDIR): some unit tests write fixed names there, and the off runs run at once; the run's
folder is said <work> in what is compared. An unstable line is reported
with whether cold and warm gave one of the results the off runs gave, and whether its
measures (all but the bytes' hash) are the same.

    python3 -I scripts/opcache_gate.py <build folder> [<out folder>]

Exits 1 if a stable line differs, or a line is missing.
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

BUILD = os.path.abspath(sys.argv[1])
OUT = os.path.abspath(sys.argv[2] if len(sys.argv) > 2 else tempfile.mkdtemp(prefix="opcache-gate-"))
CACHE = os.path.join(OUT, "cache")
# Each cached run's folder, and whether the unit tests and the cases defer their stores in it.
CACHED = {"cold": (CACHE, False), "warm": (CACHE, False),
          "cold-deferred": (CACHE + "-deferred", True), "warm-deferred": (CACHE + "-deferred", True)}
# The heap layouts of the off runs: two with the default (ASLR differs), three otherwise.
LAYOUTS = {
    "off": "",
    "off-again": "",
    "off-mmap": "glibc.malloc.mmap_threshold=65536",
    "off-tcache": "glibc.malloc.tcache_max=64",
    "off-notcache": "glibc.malloc.tcache_count=0",
}
# The second look's.
FULL_LAYOUTS = ["", "", "glibc.malloc.mmap_threshold=65536", "glibc.malloc.tcache_count=0"]
MORE_LAYOUTS = ["", "", "", "glibc.malloc.mmap_threshold=32768", "glibc.malloc.mmap_threshold=131072",
                "glibc.malloc.mmap_threshold=262144", "glibc.malloc.top_pad=65536", "glibc.malloc.top_pad=4194304",
                "glibc.malloc.tcache_max=16", "glibc.malloc.tcache_max=256", "glibc.malloc.tcache_count=1",
                "glibc.malloc.tcache_count=3", "glibc.malloc.arena_max=1", "glibc.malloc.trim_threshold=65536",
                "glibc.malloc.mxfast=0", "glibc.malloc.mxfast=32"]


def lines_of(text, prefix):
    """{key: line}: a JSON line by its name, any other line by its program and number."""
    out = {}
    for n, line in enumerate(text.splitlines()):
        key = None
        if line.startswith("{"):
            try:
                key = "%s %s" % (prefix, json.loads(line)["name"])
            except (ValueError, KeyError, TypeError):
                pass
        if key is None or key in out:
            key = "%s #%d" % (prefix, n)
        out[key] = line
    return out


def file_hash(path):
    with open(path, "rb") as f:
        data = f.read()
    if path.endswith((".step", ".stp")):
        data = re.sub(rb"FILE_NAME\([^;]*;", b"FILE_NAME(...);", data)
    return hashlib.sha256(data).hexdigest()[:16]


def run_program(args, cwd, env, name):
    r = subprocess.run(args, cwd=cwd, env=env, capture_output=True)
    # what a program says of its folder (the temporary one's paths in errors) is the same in every run
    said = lambda b: b.decode("utf-8", "replace").replace(cwd, "<work>")
    out = lines_of(said(r.stdout), name)
    out["%s stderr" % name] = said(r.stderr)
    out["%s exit" % name] = str(r.returncode)
    return out


def environment(tunables, cache, work, deferred=False):
    tmp = os.path.join(work, "tmp")
    os.makedirs(tmp, exist_ok=True)
    env = dict(os.environ, BUILD123D_SHOW_BREP="1", TMPDIR=tmp)
    for name in ("BUILD123D_OPCACHE_DIR", "BUILD123D_OPCACHE_DEFERRED", "GLIBC_TUNABLES"):
        env.pop(name, None)
    if tunables:
        env["GLIBC_TUNABLES"] = tunables
    if cache:
        env["BUILD123D_OPCACHE_DIR"] = cache
    if deferred:
        env["BUILD123D_OPCACHE_DEFERRED"] = "1"
    return env


def run_example(e, work, env):
    """An example's lines, and the files it wrote by their hashes."""
    before = {f: os.path.getmtime(os.path.join(work, f)) for f in os.listdir(work)}
    out = run_program([os.path.join(BUILD, "examples", e)], work, env, "example " + e)
    for f in sorted(os.listdir(work)):
        path = os.path.join(work, f)
        if f != "tmp" and os.path.isfile(path) and before.get(f) != os.path.getmtime(path):
            out["file %s %s" % (e, f)] = file_hash(path)
    return out


def run(mode, tunables=""):
    """Every output of one run, as {key: text}."""
    work = os.path.join(OUT, "work-" + mode)
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    cache, deferred = CACHED.get(mode, (None, False))
    env = environment(tunables, cache, work)
    flushing = environment(tunables, cache, work, deferred)
    t0 = time.monotonic()
    out = {}
    out.update(run_program([os.path.join(BUILD, "build123d_tests")], work, flushing, "tests"))
    for e in sorted(os.listdir(os.path.join(BUILD, "examples"))):
        path = os.path.join(BUILD, "examples", e)
        if os.access(path, os.X_OK) and not os.path.isdir(path):
            out.update(run_example(e, work, env))
    out.update(run_program([os.path.join(BUILD, "compare_cases")], work, flushing, "cases"))
    with open(os.path.join(OUT, mode + ".json"), "w") as f:
        json.dump(out, f, indent=0, sort_keys=True)
    print("%-13s %4.0f s, %d outputs" % (mode, time.monotonic() - t0, len(out)), flush=True)
    return out


def second_look(key, wanted):
    """What the case or example of `key` gives with the cache off: alone under MORE_LAYOUTS,
    then, unless `wanted` was among those, in its whole program under FULL_LAYOUTS (None for
    an output that has no case or example of its own)."""
    words = key.split(" ")
    # a case's line, or the hash of its shape's bytes the case says on a line of its own
    if words[0] == "cases" and (len(words) == 2 or words[2:] == ["brep"]) and not words[1].startswith("#"):
        def one(tunables, work, whole):
            args = [os.path.join(BUILD, "compare_cases")] + ([] if whole else [words[1]])
            r = subprocess.run(args, cwd=work, capture_output=True, env=environment(tunables, None, work))
            return lines_of(r.stdout.decode("utf-8", "replace"), "cases").get(key)
    elif words[0] in ("example", "file") and len(words) >= 3:
        def one(tunables, work, whole):
            return run_example(words[1], work, environment(tunables, None, work)).get(key)
    else:
        return None

    def at(i, tunables, whole):
        work = os.path.join(OUT, "work-look-%d" % i)
        shutil.rmtree(work, ignore_errors=True)
        os.makedirs(work)
        return one(tunables, work, whole)

    seen = {at(0, t, False) for t in MORE_LAYOUTS}
    if wanted not in seen:
        with ThreadPoolExecutor(len(FULL_LAYOUTS)) as pool:
            seen |= set(pool.map(lambda it: at(it[0] + 1, it[1], True), enumerate(FULL_LAYOUTS)))
    return seen


def measures(line):
    try:
        d = json.loads(line)
    except (ValueError, TypeError):
        return None
    if isinstance(d, dict):
        d.pop("brep", None)
    return d


os.makedirs(OUT, exist_ok=True)
for cache, _ in CACHED.values():
    shutil.rmtree(cache, ignore_errors=True)
with ThreadPoolExecutor(len(LAYOUTS)) as pool:
    offs = dict(zip(LAYOUTS, pool.map(lambda m: run(m, LAYOUTS[m]), LAYOUTS)))
cached = {mode: run(mode) for mode in CACHED}
for cache in sorted({c for c, _ in CACHED.values()}):
    entries = sum(len(fs) for _, _, fs in os.walk(cache))
    size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(cache) for f in fs)
    print("%s: %d files, %.0f MB" % (os.path.basename(cache), entries, size / 1e6))

base = offs["off"]
keys = sorted(set().union(*[set(o) for o in offs.values()], *cached.values()))
stable, unstable, failures = [], [], []
for k in keys:
    variants = {o.get(k) for o in offs.values()}
    if len(variants) == 1:
        stable.append(k)
        for mode, got in cached.items():
            if got.get(k) != base.get(k):
                failures.append((k, mode))
    else:
        unstable.append((k, variants))

looked = []
seen_of = {}
for k, mode in list(failures):
    got = cached[mode].get(k)
    if k not in seen_of or (seen_of[k] is not None and got not in seen_of[k]):
        more = second_look(k, got)
        seen_of[k] = None if more is None else (seen_of.get(k) or set()) | more
    seen = seen_of[k]
    if seen is not None and got in seen:
        failures.remove((k, mode))
        looked.append((k, mode))
        if k in stable:
            stable.remove(k)
            unstable.append((k, {o.get(k) for o in offs.values()} | seen))
for k, mode in looked:
    print("second look: %s %s gave what the cache off gives under another heap layout" % (mode, k))

print("%d outputs stable across the %d heap layouts, %d not" % (len(stable), len(offs), len(unstable)))
for k, variants in unstable:
    said = []
    for mode, got in cached.items():
        line = got.get(k)
        if line in variants:
            said.append("%s gave an off result" % mode)
        elif line is not None and measures(line) is not None and measures(line) == measures(base.get(k)):
            said.append("%s: other bytes, the same measures" % mode)
        else:
            said.append("%s: OTHER" % mode)
    print("  unstable  %-50s %d variants off; %s" % (k[:50], len(variants), "; ".join(said)))
for k, mode in failures:
    print("DIFFERS   %s %s" % (mode, k))
    print("   off:  %s" % (base.get(k) or "")[:300])
    print("   %s: %s" % (mode, (cached[mode].get(k) or "")[:300]))
print("%s: %d stable outputs, %d differ cached" % ("FAILED" if failures else "passed", len(stable), len(failures)))
sys.exit(1 if failures else 0)
