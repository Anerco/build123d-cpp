"""Compare the C++ examples with the same examples in Python build123d.

Each program in build/examples prints the objects it shows as JSON lines;
examples_py.py builds the same objects in Python.

Usage: python compare_examples.py <build/examples directory> [example ...]
(run with a Python environment where build123d is installed)
"""

import os
import sys
import tempfile

from compare import diff, run

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    examples_dir = os.path.abspath(sys.argv[1])  # the examples run in a temporary directory
    only = sys.argv[2:]
    py = run([sys.executable, os.path.join(HERE, "examples_py.py"), *only])
    examples = list(dict.fromkeys(row["example"] for row in py.values()))
    total = failures = 0
    with tempfile.TemporaryDirectory() as tmp:  # the examples write STEP files
        for example in examples:
            cpp = run([os.path.join(examples_dir, example)], cwd=tmp)
            names = [n for n, row in py.items() if row["example"] == example]
            names += [n for n in cpp if n not in names]
            for name in names:
                total += 1
                if name not in cpp or name not in py:
                    failures += 1
                    side = "cpp" if name not in cpp else "python"
                    print(f"MISSING  {example}/{name}: no {side} result")
                    continue
                problems = diff(cpp[name], py[name])
                if problems:
                    failures += 1
                    print(f"DIFF     {example}/{name}: " + "; ".join(problems))
                else:
                    print(f"ok       {example}/{name}")
    print(f"\n{total - failures}/{total} example objects match")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
