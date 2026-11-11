"""Markdown tables from results/*.json (written by driver.py), and results/summary.csv.

    python -I -B report.py > results/report.md
"""

import csv
import json
import math
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")


def load(name):
    path = os.path.join(RES, name)
    return json.load(open(path)) if os.path.exists(path) else None


def load_all(stem):
    """results/<stem>.json, then its tagged supplements <stem>.<tag>.json (not .partial), in
    name order; a later row replaces an earlier one of the same model, in place."""
    names = sorted(n for n in os.listdir(RES) if n.startswith(stem + ".") and n.endswith(".json")
                   and ".partial." not in n and n != stem + ".json")
    if os.path.exists(os.path.join(RES, stem + ".json")):
        names.insert(0, stem + ".json")
    rows, index = [], {}
    for name in names:
        for row in json.load(open(os.path.join(RES, name))):
            key = (row.get("group"), row.get("name"))
            if key in index:
                rows[index[key]] = row
            else:
                index[key] = len(rows)
                rows.append(row)
    return rows


def med(xs):
    return statistics.median(xs)


def q(xs):
    s = sorted(xs)
    if len(s) < 2:
        return s[0], s[0]
    qs = statistics.quantiles(s, n=4)
    return qs[0], qs[2]


def fmt_ms(x):
    if x >= 1000:
        return f"{x / 1000:.2f} s"
    if x >= 100:
        return f"{x:.0f} ms"
    if x >= 10:
        return f"{x:.1f} ms"
    if x >= 1:
        return f"{x:.2f} ms"
    return f"{x * 1000:.0f} µs"


def spread(xs):
    lo, hi = q(xs)
    m = med(xs)
    return f"±{100 * (hi - lo) / 2 / m:.0f}%" if m > 0 else ""


def geomean(xs):
    xs = [x for x in xs if x > 0 and math.isfinite(x)]
    return math.exp(sum(math.log(x) for x in xs) / len(xs)) if xs else float("nan")


def same(c, p):
    """Do the two sides' shapes match (volume, area to 1e-6; face count)?"""
    cs = {s["name"]: s for s in c.get("shapes", [])}
    ps = {s["name"]: s for s in p.get("shapes", [])}
    if set(cs) != set(ps) or not cs:
        return False
    for n in cs:
        a, b = cs[n], ps[n]
        for k in ("volume", "area"):
            if abs(a[k] - b[k]) > max(1e-6, 1e-6 * max(abs(a[k]), abs(b[k]))):
                return False
        if a["faces"] != b["faces"]:
            return False
    return True


def inprocess_table(rows, group, title, csv_rows):
    rows = [r for r in rows if r["group"] == group]
    if not rows:
        return []
    out = [f"### {title}", "",
           "| model | objects | C++ | Python | Python / C++ | runs C++/Py | same geometry |",
           "|---|---:|---:|---:|---:|---:|:---:|"]
    ratios = []
    tc = tp = 0
    for r in rows:
        c, p = r["cpp"], r["py"]
        if "times_ms" not in c or "times_ms" not in p:
            out.append(f"| {r['name']} | | {c.get('error', '')[:60]} | {p.get('error', '')[:60]} | | | |")
            continue
        cm, pm = med(c["times_ms"]), med(p["times_ms"])
        ratios.append(pm / cm)
        tc += cm
        tp += pm
        out.append(f"| {r['name']} | {len(c['shapes'])} | {fmt_ms(cm)} {spread(c['times_ms'])} | "
                   f"{fmt_ms(pm)} {spread(p['times_ms'])} | {pm / cm:.2f} | {len(c['times_ms'])}/{len(p['times_ms'])} | "
                   f"{'yes' if same(c, p) else '**no**'} |")
        csv_rows.append({"block": "inprocess", "group": group, "name": r["name"], "cpp_median_ms": cm,
                         "cpp_p25_ms": q(c["times_ms"])[0], "cpp_p75_ms": q(c["times_ms"])[1],
                         "py_median_ms": pm, "py_p25_ms": q(p["times_ms"])[0], "py_p75_ms": q(p["times_ms"])[1],
                         "ratio_py_cpp": pm / cm, "n_cpp": len(c["times_ms"]), "n_py": len(p["times_ms"]),
                         "same_geometry": same(c, p)})
    if len(ratios) > 1:
        out.append(f"| **sum; median ratio** | | **{fmt_ms(tc)}** | **{fmt_ms(tp)}** | "
                   f"**{tp / tc:.2f}** (sum), **{med(ratios):.2f}** (median) | | |")
    out.append("")
    return out


def main():
    env = load("env.json") or {}
    start = load("startup.json")
    inproc = load_all("inprocess")
    io = load("io.json") or []
    whole = load_all("wholerun")
    suite = load("suite.json") or []
    csv_rows = []
    out = ["# C++ build123d-cpp vs Python build123d: results", ""]
    if env:
        pk = env.get("python_packages", {})
        out += [f"Machine: {env['cpu']} ({env['threads']} threads), {env['os']} on WSL2 (kernel {env['kernel']}), "
                f"{env['mem_total'].split(':')[-1].strip()} RAM.",
                f"C++: {env['cxx']}; library {env['library_flags']}, programs {env['program_flags']}; "
                f"OCCT {env['occt_cpp']} (conda-forge, {env['occt_cpp_build']}). Repo at {env['repo_commit']}"
                f"{' (dirty)' if env.get('repo_dirty') else ''}.",
                f"Python {pk.get('python')}: build123d {pk.get('build123d')}, bd_warehouse {pk.get('bd_warehouse')}, "
                f"OCP {pk.get('cadquery-ocp-novtk')} (OCCT 8.0.1), numpy {pk.get('numpy')}. Measured {env['date']}.", ""]
    if inproc and whole:
        wh = {(r["group"], r["name"]): r for r in whole}
        titles = {"example": "The 24 build123d examples", "bd_warehouse": "bd_warehouse parts",
                  "operation": "Single operations"}
        out += ["## Per model, combined", "",
                "Build: in a running process, median (details below). Whole run: a fresh process that builds "
                "the model and writes one STEP file, median of 5; memory: its peak RSS.", ""]
        for group, title in titles.items():
            rows = [r for r in inproc if r["group"] == group and "times_ms" in r["cpp"] and "times_ms" in r["py"]]
            if not rows:
                continue
            out += [f"**{title}**", "",
                    "| model | build C++ | build Python | Py / C++ | whole run C++ | whole run Python | C++ memory | Python memory |",
                    "|---|---:|---:|---:|---:|---:|---:|---:|"]
            for r in rows:
                cm, pm = med(r["cpp"]["times_ms"]), med(r["py"]["times_ms"])
                w = wh.get((group, r["name"]))
                wcells = (f"{med(w['cpp']['wall']):.3f} s | {med(w['py']['wall']):.2f} s | "
                          f"{med(w['cpp']['rss']) / 1024:.0f} MB | {med(w['py']['rss']) / 1024:.0f} MB") if w else " | | | "
                out.append(f"| {r['name']} | {fmt_ms(cm)} | {fmt_ms(pm)} | {pm / cm:.2f} | {wcells} |")
            out.append("")
    if start:
        out += ["## Startup (fresh process, median of the runs, peak RSS)", "",
                "| what | wall | peak memory |", "|---|---:|---:|"]
        for k, v in start.items():
            if k == "importtime_top_us":
                continue
            rss = f"{v['rss_kb']['median'] / 1024:.0f} MB" if "rss_kb" in v else ""
            out.append(f"| {k} | {fmt_ms(1000 * v['wall_s']['median'])} (n={v['wall_s']['n']}, "
                       f"{fmt_ms(1000 * v['wall_s']['min'])}–{fmt_ms(1000 * v['wall_s']['max'])}) | {rss} |")
            csv_rows.append({"block": "startup", "name": k, "wall_median_s": v["wall_s"]["median"],
                             "rss_median_kb": v.get("rss_kb", {}).get("median")})
        if "importtime_top_us" in start:
            top = ", ".join(f"{k} {v / 1000:.0f} ms" for k, v in list(start["importtime_top_us"].items())[:8])
            out += ["", f"`python -X importtime -c 'import build123d'`, cumulative, largest top-level imports: {top}."]
        out.append("")
    if inproc or io:
        out += ["## Build time inside a running process",
                "",
                "Warmed up (2 untimed builds), then at least 10 builds and 2 s, at most 100; median, "
                "with ± half the interquartile range as a share of the median. The result is destroyed "
                "outside the timed region. 'same geometry': volume and area within 1e-6, same face count.", ""]
        out += inprocess_table(inproc, "example", "The 24 build123d examples", csv_rows)
        out += inprocess_table(inproc, "bd_warehouse", "bd_warehouse parts", csv_rows)
        out += inprocess_table(inproc, "operation", "Single operations", csv_rows)
        if io:
            out += inprocess_table(io, "io", "Export and import (op_holes' plate: 1 solid, 231 faces)", csv_rows)
    if whole:
        out += ["## Whole run: a fresh process that builds the model and writes one STEP file", "",
                "C++: the program; Python: `python -I` running the same model. Median of the runs "
                "(alternated C++/Python, after one untimed run of each), peak RSS from `/usr/bin/time -f %M`.", "",
                "| model | C++ | C++ memory | Python | Python memory | Python / C++ | Python − C++ |",
                "|---|---:|---:|---:|---:|---:|---:|"]
        wr = []
        for r in whole:
            cw, pw = med(r["cpp"]["wall"]), med(r["py"]["wall"])
            cr, pr = med(r["cpp"]["rss"]) / 1024, med(r["py"]["rss"]) / 1024
            wr.append((cw, pw, cr, pr))
            out.append(f"| {r['name']} | {cw:.3f} s {spread(r['cpp']['wall'])} | {cr:.0f} MB | {pw:.3f} s "
                       f"{spread(r['py']['wall'])} | {pr:.0f} MB | {pw / cw:.1f} | {pw - cw:+.2f} s |")
            csv_rows.append({"block": "wholerun", "group": r["group"], "name": r["name"], "cpp_median_s": cw,
                             "py_median_s": pw, "cpp_rss_mb": cr, "py_rss_mb": pr, "ratio_py_cpp": pw / cw,
                             "n_cpp": len(r["cpp"]["wall"]), "n_py": len(r["py"]["wall"])})
        if wr:
            out.append(f"| **median of the {len(wr)} models** | **{med([w[0] for w in wr]):.3f} s** | "
                       f"**{med([w[2] for w in wr]):.0f} MB** | **{med([w[1] for w in wr]):.3f} s** | "
                       f"**{med([w[3] for w in wr]):.0f} MB** | **{med([w[1] / w[0] for w in wr]):.1f}** | "
                       f"**{med([w[1] - w[0] for w in wr]):+.2f} s** |")
        out.append("")
    if suite:
        out += ["## bd_warehouse comparison suite, per module", "",
                "Every case of tests/compare/cases_bdw_*, timed in-process by cpp/suite_main.cpp and "
                "py/suite.py: one untimed build, then one timed; sums of the timed builds. A shape case "
                "builds one object on each side (equal work). A value case returns values; several Python "
                "twins build the part once per value they read (e.g. bearing `*_default_parts` builds it 7 "
                "times, worm gear `*_values` 4 times) where the C++ twin builds it once, so value cases are "
                "not equal work. Every case gives the same result on both sides (volumes checked here too).", "",
                "| module | shape cases | C++ | Python | Py / C++ | value cases | C++ | Python | Py / C++ |",
                "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
        by = {}
        for r in suite:
            by.setdefault(r["module"], {})[r["lang"]] = r
        tot = {"shape": [0, 0, 0], "value": [0, 0, 0]}
        for mod, d in by.items():
            if "cpp" not in d or "py" not in d:
                continue
            cases_c = {x["name"]: x for x in d["cpp"]["cases"]}
            cases_p = {x["name"]: x for x in d["py"]["cases"]}
            cells = [mod[4:]]
            row = {"block": "suite", "name": mod}
            for kind in ("shape", "value"):
                names = [n for n, x in cases_c.items() if x["kind"] == kind and n in cases_p]
                cs = sum(med(cases_c[n]["times_ms"]) for n in names) / 1000
                ps = sum(med(cases_p[n]["times_ms"]) for n in names) / 1000
                for i, v in enumerate((len(names), cs, ps)):
                    tot[kind][i] += v
                ratio = f"{ps / cs:.2f}" if cs > 0 else ""
                cells += [str(len(names)), f"{cs:.2f} s", f"{ps:.2f} s", ratio]
                row.update({f"{kind}_cases": len(names), f"cpp_{kind}_s": cs, f"py_{kind}_s": ps})
            out.append("| " + " | ".join(cells) + " |")
            csv_rows.append(row)
        sh, va = tot["shape"], tot["value"]
        out.append(f"| **all** | **{sh[0]}** | **{sh[1]:.1f} s** | **{sh[2]:.1f} s** | **{sh[2] / sh[1]:.2f}** | "
                   f"**{va[0]}** | **{va[1]:.1f} s** | **{va[2]:.1f} s** | **{va[2] / va[1]:.2f}** |")
        out.append("")
        procs = ", ".join(f"{m[4:]} {d['cpp']['process_wall_s']:.0f}/{d['py']['process_wall_s']:.0f} s"
                          for m, d in by.items() if "cpp" in d and "py" in d)
        out += [f"Whole timing programs (start, imports, every case built twice), C++/Python: {procs}.", ""]
    flow = os.path.join(RES, "compare_flow.jsonl")
    if os.path.exists(flow):
        rows = [json.loads(l) for l in open(flow) if l.strip()]
        out += ["### The suite as compare.py runs it", "",
                "`build/compare_cases <names>` against `python cases_py.py <names>` (imports every case "
                "module, builds each case once, computes its metrics), whole processes:", "",
                "| module | cases | C++ | Python | Py / C++ | C++ memory | Python memory |", "|---|---:|---:|---:|---:|---:|---:|"]
        by = {}
        for r in rows:
            by.setdefault(r["module"], {})[r["side"]] = r
        for mod, d in by.items():
            c, p = d["cpp"], d["py"]
            out.append(f"| {mod[4:]} | {c['cases']} | {c['wall_s']:.1f} s | {p['wall_s']:.1f} s | {p['wall_s'] / c['wall_s']:.2f} | "
                       f"{c['rss_kb'] / 1024:.0f} MB | {p['rss_kb'] / 1024:.0f} MB |")
        out.append("")
    share = load("share.json")
    if share:
        ip = {(r["group"], r["name"]): r for r in inproc}
        out += ["## Where a Python build's time goes", "",
                "py/occt_share.py counts the calls into OCP (OCCT) in a Python build and times them "
                "(sys.monitoring, 5 builds, median). 'in OCP' is measured while monitored, which adds about "
                "1 µs per call, so it overstates OCCT's time; the per-call column uses only the call count "
                "and the unmonitored times above. results/explain.txt has the per-function breakdowns and "
                "the C++ call counts behind the notes.", "",
                "| model | C++ build | Python build | Py − C++ | OCP calls per build | (Py − C++) per OCP call | in OCP (monitored) |",
                "|---|---:|---:|---:|---:|---:|---:|"]
        for r in share:
            base = ip.get((r["group"], r["name"]))
            if not base or "times_ms" not in base["py"] or "ocp_ms" not in r:
                continue
            cm, pm = med(base["cpp"]["times_ms"]), med(base["py"]["times_ms"])
            per = 1000 * (pm - cm) / r["ocp_calls"] if r["ocp_calls"] else float("nan")
            diff = fmt_ms(pm - cm) if pm >= cm else "−" + fmt_ms(cm - pm)
            out.append(f"| {r['name']} | {fmt_ms(cm)} | {fmt_ms(pm)} | {diff} | {r['ocp_calls']} | {per:+.1f} µs | "
                       f"{fmt_ms(r['ocp_ms'])} |")
            csv_rows.append({"block": "share", "name": r["name"], "cpp_median_ms": cm, "py_median_ms": pm,
                             "py_in_ocp_ms_monitored": r["ocp_ms"], "ocp_calls": r["ocp_calls"]})
        out.append("")
    rep = os.path.join(RES, "repeat", "inprocess.json")
    if os.path.exists(rep):
        again = {(r["group"], r["name"]): r for r in json.load(open(rep))}
        dc, dp, dr = [], [], []
        for r in inproc:
            a = again.get((r["group"], r["name"]))
            if not a or "times_ms" not in a["cpp"] or "times_ms" not in r["cpp"] or "times_ms" not in r["py"] or "times_ms" not in a["py"]:
                continue
            c1, c2 = med(r["cpp"]["times_ms"]), med(a["cpp"]["times_ms"])
            p1, p2 = med(r["py"]["times_ms"]), med(a["py"]["times_ms"])
            dc.append(abs(c2 / c1 - 1))
            dp.append(abs(p2 / p1 - 1))
            dr.append(abs((p2 / c2) / (p1 / c1) - 1))
        if dc:
            out += ["## Reproducibility", "",
                    f"A second in-process pass ({len(dc)} models, results/repeat/): the medians moved by a median of "
                    f"{100 * med(dc):.1f}% (C++) and {100 * med(dp):.1f}% (Python), at most {100 * max(dc):.1f}% and "
                    f"{100 * max(dp):.1f}%; the Python/C++ ratios by a median of {100 * med(dr):.1f}%, at most "
                    f"{100 * max(dr):.1f}%.", ""]
    print("\n".join(out))
    keys = []
    for r in csv_rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(RES, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(csv_rows)


if __name__ == "__main__":
    main()
