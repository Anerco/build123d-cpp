"""Samples of a perf.data file in one shared library, by function (the nearest exported symbol
at or below each address: OCCT's libraries ship without .symtab, and this perf doesn't
resolve their .dynsym).

    python -I -B perf_by_function.py <perf.data> <library file> [top]
"""

import bisect
import collections
import os
import re
import subprocess
import sys


def symbols(lib):
    out = subprocess.run(["nm", "-D", "-C", "--defined-only", lib], capture_output=True, text=True).stdout
    table = []
    for line in out.splitlines():
        parts = line.split(" ", 2)
        if len(parts) == 3 and parts[1] in "TtWw":
            table.append((int(parts[0], 16), parts[2]))
    table.sort()
    return [a for a, _ in table], [n for _, n in table]


def main():
    data, lib = sys.argv[1], sys.argv[2]
    top = int(sys.argv[3]) if len(sys.argv) > 3 else 12
    addrs, names = symbols(lib)
    base = os.path.basename(lib)
    rep = subprocess.run(["perf", "report", "-i", data, "--no-children", "--sort", "dso,sym", "-n", "-q", "-g", "none"],
                         capture_output=True, text=True).stdout
    by = collections.Counter()
    for line in rep.splitlines():
        m = re.match(r"\s*[\d.]+%\s+(\d+)\s+(\S+)\s+\[\.\]\s+0x([0-9a-f]+)", line)
        if not m or m.group(2) != base:
            continue
        i = bisect.bisect_right(addrs, int(m.group(3), 16)) - 1
        name = names[i] if i >= 0 else "?"
        by[re.sub(r"\(.*", "", name)] += int(m.group(1))
    total = sum(by.values())
    print(f"{base}: {total} samples")
    for name, n in by.most_common(top):
        print(f"  {n:6d}  {name}")


if __name__ == "__main__":
    main()
