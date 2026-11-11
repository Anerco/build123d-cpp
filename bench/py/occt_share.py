"""How much of a Python build is spent inside OCCT (calls into OCP) and how much in Python.

    python -I -B occt_share.py <model.py> <func> [runs]

After 2 warm-up builds, builds the model <runs> times with sys.monitoring (PEP 669) timing
every call to an OCP callable (functions, methods and constructors of OCP's types), outermost
calls only. Prints one JSON line: the builds' median wall time while monitored, and the median
time inside OCP. The monitoring slows the Python side down (every call raises an event), not
the time inside OCP, so 'ocp_ms' estimates the OCCT time of an unmonitored build; the rest
of an unmonitored build is Python's own.
"""

import json
import statistics
import sys
import time
import types

sys.dont_write_bytecode = True
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from pybench import load  # noqa: E402

mon = sys.monitoring
TOOL = mon.PROFILER_ID


def _ocp_module(x):
    return (getattr(x, "__module__", None) or "").startswith("OCP")


def is_ocp(c):
    """An OCP type (constructor), function, static method or method, bound or not. pybind11's
    functions have a PyCapsule as __self__, and OCP's module as __module__."""
    if _ocp_module(c):
        return True
    if isinstance(c, types.MethodType):
        return _ocp_module(c.__func__) or _ocp_module(type(c.__self__))
    return False


class Meter:
    """Times OCP calls, one at a time (OCP calls don't call back into Python here). A call ends
    at its C_RETURN/C_RAISE, matched by function (an event can get a pybind11 method bound or
    the function itself). Python 3.12 raises no C_RETURN for a call with *args, so a call still
    open at the next CALL event ends there: the bit of Python in between is counted as OCP, so
    the share of OCP is, if anything, overstated."""

    def __init__(self):
        self.pending = None
        self.t0 = 0.0
        self.total = 0.0
        self.calls = 0
        self.unmatched = 0

    def close(self):
        self.total += time.perf_counter() - self.t0
        self.calls += 1
        self.pending = None

    def on_call(self, code, offset, callable_, arg0):
        if self.pending is not None:
            self.close()
            self.unmatched += 1
        if is_ocp(callable_):
            self.pending = callable_.__func__ if isinstance(callable_, types.MethodType) else callable_
            self.t0 = time.perf_counter()

    def on_return(self, code, offset, callable_, arg0):
        if self.pending is None:
            return
        if isinstance(callable_, types.MethodType):
            callable_ = callable_.__func__
        if callable_ is self.pending:
            self.close()


def main():
    path, func = sys.argv[1], sys.argv[2]
    runs = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    module = load(path)
    build = getattr(module, "EXAMPLES", {}).get(func) or getattr(module, func)
    prepare = getattr(module, "prepare", None)
    for _ in range(2):
        if prepare:
            prepare()
        build()
    meter = Meter()
    mon.use_tool_id(TOOL, "occt_share")
    mon.register_callback(TOOL, mon.events.CALL, meter.on_call)
    mon.register_callback(TOOL, mon.events.C_RETURN, meter.on_return)
    mon.register_callback(TOOL, mon.events.C_RAISE, meter.on_return)
    walls, ocps, calls, unmatched = [], [], [], []
    result = None
    for _ in range(runs):
        if prepare:
            prepare()
        result = None
        meter.total, meter.calls, meter.pending, meter.unmatched = 0.0, 0, None, 0
        mon.set_events(TOOL, mon.events.CALL | mon.events.C_RETURN | mon.events.C_RAISE)
        t0 = time.perf_counter()
        result = build()
        if meter.pending is not None:
            meter.close()
        wall = time.perf_counter() - t0
        mon.set_events(TOOL, 0)
        walls.append(1000 * wall)
        ocps.append(1000 * meter.total)
        calls.append(meter.calls)
        unmatched.append(meter.unmatched)
    mon.free_tool_id(TOOL)
    print(json.dumps({"model": func if func != "model" else path.rsplit("/", 1)[-1][:-3],
                      "monitored_wall_ms": statistics.median(walls), "ocp_ms": statistics.median(ocps),
                      "ocp_calls": int(statistics.median(calls)),
                      "closed_at_next_call": int(statistics.median(unmatched)), "runs": runs}))


if __name__ == "__main__":
    main()
