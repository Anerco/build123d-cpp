"""The OCP calls a Python build spends its OCCT time in, by function.

    python -I -B ocp_calls.py <model.py> <func> [runs] [top]

After 2 warm-up builds, monitors <runs> builds as occt_share.py does and prints the <top>
OCP callables by time per build, with their calls per build. Used to explain the models where
Python's time inside OCP exceeds the whole C++ build: the Python library does OCCT work there
that the port doesn't (copies, bounding boxes), rather than the same work more slowly.
"""

import collections
import sys
import time

sys.dont_write_bytecode = True
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from occt_share import Meter  # noqa: E402
from pybench import load  # noqa: E402


class ByFunction(Meter):
    def __init__(self):
        super().__init__()
        self.time = collections.Counter()
        self.count = collections.Counter()

    def close(self):
        f = self.pending
        name = getattr(f, "__qualname__", None) or getattr(f, "__name__", repr(f))
        name = name.replace("PyCapsule.", "")
        self.time[name] += time.perf_counter() - self.t0
        self.count[name] += 1
        super().close()


def main():
    path, func = sys.argv[1], sys.argv[2]
    runs = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    top = int(sys.argv[4]) if len(sys.argv) > 4 else 8
    module = load(path)
    build = getattr(module, "EXAMPLES", {}).get(func) or getattr(module, func)
    for _ in range(2):
        build()
    meter = ByFunction()
    mon = sys.monitoring
    mon.use_tool_id(mon.PROFILER_ID, "ocp_calls")
    mon.register_callback(mon.PROFILER_ID, mon.events.CALL, meter.on_call)
    mon.register_callback(mon.PROFILER_ID, mon.events.C_RETURN, meter.on_return)
    mon.register_callback(mon.PROFILER_ID, mon.events.C_RAISE, meter.on_return)
    mon.set_events(mon.PROFILER_ID, mon.events.CALL | mon.events.C_RETURN | mon.events.C_RAISE)
    for _ in range(runs):
        build()
        if meter.pending is not None:
            meter.close()
    mon.set_events(mon.PROFILER_ID, 0)
    total = sum(meter.time.values())
    print(f"{func if func != 'model' else path.rsplit('/', 1)[-1][:-3]}: {1000 * total / runs:.2f} ms/build in OCP")
    for name, t in meter.time.most_common(top):
        print(f"  {1000 * t / runs:8.2f} ms  {meter.count[name] / runs:7.0f} calls  {name}")


if __name__ == "__main__":
    main()
