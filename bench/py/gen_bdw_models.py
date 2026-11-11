# one-off: writes the C++ and Python model files for the bd_warehouse parts (kept for reference)
import os
MODELS = [
    # name, C++ include, C++ expression, Python import, Python expression
    ("gear_spur", "SpurGear(1.5, 31, 20, 4)", "from bd_warehouse.gear import SpurGear", "SpurGear(1.5, 31, 20, 4)"),
    ("gear_helical", "HelicalGear(2, 13, 20, 45, 10)", "from bd_warehouse.gear import HelicalGear", "HelicalGear(2, 13, 20, 45, 10)"),
    ("gear_worm", "WormGear(2, 2, 10, 30, 30, 12)", "from bd_warehouse.gear import WormGear", "WormGear(2, 2, 10, 30, 30, 12)"),
    ("thread_iso_external", "IsoThread(10, 1.5, 20)", "from bd_warehouse.thread import IsoThread", "IsoThread(10, 1.5, 20)"),
    ("thread_iso_internal", "IsoThread(10, 1.5, 20, false)", "from bd_warehouse.thread import IsoThread", "IsoThread(10, 1.5, 20, external=False)"),
    ("screw_shcs_simple", "SocketHeadCapScrew(\"M6-1\", 20)", "from bd_warehouse.fastener import SocketHeadCapScrew", "SocketHeadCapScrew(\"M6-1\", 20)"),
    ("screw_shcs_threaded", "SocketHeadCapScrew(\"M6-1\", 20, {.simple = false})", "from bd_warehouse.fastener import SocketHeadCapScrew", "SocketHeadCapScrew(\"M6-1\", 20, simple=False)"),
    ("nut_hex_simple", "HexNut(\"M6-1\")", "from bd_warehouse.fastener import HexNut", "HexNut(\"M6-1\")"),
    ("nut_hex_threaded", "HexNut(\"M6-1\", {.simple = false})", "from bd_warehouse.fastener import HexNut", "HexNut(\"M6-1\", simple=False)"),
    ("bearing_deep_groove", "SingleRowDeepGrooveBallBearing(\"M8-22-7\")", "from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing", "SingleRowDeepGrooveBallBearing(\"M8-22-7\")"),
    ("flange_weld_neck", "WeldNeckFlange(\"4\", 150)", "from bd_warehouse.flange import WeldNeckFlange", "WeldNeckFlange(\"4\", 150)"),
    ("sprocket", "Sprocket(32)", "from bd_warehouse.sprocket import Sprocket", "Sprocket(32)"),
    ("o_ring", "ORing(\"214\")", "from bd_warehouse.o_rings import ORing", "ORing(\"214\")"),
    ("openbuilds_cbeam_assembly", "CBeamAssembly(300.0)", "from bd_warehouse.open_builds import CBeamAssembly", "CBeamAssembly(300.0)"),
]
here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for name, cxx, pyimp, pyx in MODELS:
    with open(f"{here}/cpp/models/{name}.cpp", "w") as f:
        f.write(f"""#include <bd_warehouse/bd_warehouse.hpp>

#include "../bench.hpp"

using namespace bd_warehouse;

static build123d::Shape model() {{ return build123d::Shape({cxx}); }}

BENCH_MODEL("{name}", model)
""")
    with open(f"{here}/py/models/{name}.py", "w") as f:
        f.write(f"""{pyimp}


def model():
    return {pyx}
""")
print(len(MODELS))
