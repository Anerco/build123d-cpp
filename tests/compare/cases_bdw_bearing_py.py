"""Python twins of cases_bdw_bearing.cpp: bd_warehouse's bearing.py."""

import re

from build123d import Plane

from bd_warehouse.bearing import (
    Bearing,
    PressFitHole,
    SingleRowAngularContactBallBearing,
    SingleRowCappedDeepGrooveBallBearing,
    SingleRowCylindricalRollerBearing,
    SingleRowDeepGrooveBallBearing,
    SingleRowTaperedRollerBearing,
)
from bd_warehouse.fastener import (
    CounterSunkScrew,
    HeatSetNut,
    HexNut,
    PlainWasher,
    SetScrew,
    SocketHeadCapScrew,
    method_exists,
)

from bdw_util import error_of, loc_values, num
from values import V

CLASSES = {c.__name__: c for c in Bearing.__subclasses__()}

METHODS = ["cap", "cage", "roller", "_roller", "inner_race_section", "contact_angle", "bore_diameter", "nothing"]


def case_name(*parts):
    return "bdw_bearing_" + "_".join(re.sub(r"[^A-Za-z0-9]+", "_", str(p)) for p in parts).strip("_")


def bearing_values(b):
    out = [
        b.label, b.info, b.bearing_class, b.bearing_size, b.bearing_type, b.capped, b.is_metric,
        b.bore_diameter, b.outer_diameter, b.thickness, b.roller_count, b.roller_diameter,
        b.race_center_radius, b.length_offset(), b.hole_locations,
        [[k, num(v)] for k, v in b.bearing_dict.items()],
        [[k, num(v)] for k, v in b.clearance_hole_diameters.items()],
        loc_values(b.location), loc_values(b.joints["a"].relative_location),
        loc_values(b.joints["b"].relative_location),
        [[c.label, c.volume, c.material.material.name, loc_values(c.location)] for c in b.children],
        [b.method_exists(m) for m in METHODS],
    ]  # fmt: skip
    if isinstance(b, (SingleRowAngularContactBallBearing, SingleRowTaperedRollerBearing)):
        out.append(b.contact_angle)
    if isinstance(b, SingleRowTaperedRollerBearing):
        out += [b.taper_length, b.cage_edge.position_at(0), b.cage_edge.position_at(1)]
    return V(out)


CASES = {}

# every size of every type of every bearing
for cls_name, cls in CLASSES.items():
    for bearing_type in sorted(cls.types()):
        for size in cls.sizes(bearing_type):
            CASES[case_name(cls_name, bearing_type, size)] = lambda c=cls, s=size, t=bearing_type: c(s, t)
            CASES[case_name(cls_name, bearing_type, size, "values")] = (
                lambda c=cls, s=size, t=bearing_type: bearing_values(c(s, t))
            )

# the parts of a few bearings
PART_SPECS = [
    ("SingleRowDeepGrooveBallBearing", "M8-22-7"),
    ("SingleRowCappedDeepGrooveBallBearing", "M8-22-7"),
    ("SingleRowAngularContactBallBearing", "M10-30-9"),
    ("SingleRowAngularContactBallBearing", "M25-62-17"),
    ("SingleRowCylindricalRollerBearing", "M15-35-11"),
    ("SingleRowTaperedRollerBearing", "M25-52-19.25"),
    ("SingleRowTaperedRollerBearing", "M17-47-20.25"),
]
for cls_name, size in PART_SPECS:
    cls = CLASSES[cls_name]
    CASES[case_name(cls_name, size, "inner_race_section")] = lambda c=cls, s=size: c(s).inner_race_section()
    CASES[case_name(cls_name, size, "outer_race_section")] = lambda c=cls, s=size: c(s).outer_race_section()
    CASES[case_name(cls_name, size, "roller")] = lambda c=cls, s=size: c(s).roller()
    CASES[case_name(cls_name, size, "countersink_profile")] = lambda c=cls, s=size: c(s).countersink_profile()
    CASES[case_name(cls_name, size, "countersink_profile_interference")] = (
        lambda c=cls, s=size: c(s).countersink_profile(0.05)
    )
    CASES[case_name(cls_name, size, "make_bearing")] = lambda c=cls, s=size: c(s).make_bearing()
    CASES[case_name(cls_name, size, "default_parts")] = lambda c=cls, s=size: V(
        [
            c(s).default_race_center_radius(), c(s).default_roller_diameter(),
            c(s).default_inner_race_section().area, c(s).default_outer_race_section().area,
            c(s).default_countersink_profile(0.1).area, c(s).default_roller().volume, c(s).default_cap().volume,
        ]  # fmt: skip
    )
    if hasattr(cls, "cage"):
        CASES[case_name(cls_name, size, "cage")] = lambda c=cls, s=size: c(s).cage()
    if hasattr(cls, "cap"):
        CASES[case_name(cls_name, size, "cap")] = lambda c=cls, s=size: c(s).cap()

CASES.update(
    {
        # PressFitHole
        "bdw_bearing_press_fit_hole": lambda: PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7")),
        "bdw_bearing_press_fit_hole_interference": lambda: PressFitHole(
            SingleRowDeepGrooveBallBearing("M8-22-7"), interference=0.05, fit="Close", depth=20
        ),
        "bdw_bearing_press_fit_hole_loose": lambda: PressFitHole(
            SingleRowAngularContactBallBearing("M12-32-10"), fit="Loose", depth=5
        ),
        "bdw_bearing_press_fit_hole_tapered": lambda: PressFitHole(SingleRowTaperedRollerBearing("M25-52-19.25")),
        "bdw_bearing_press_fit_hole_capped": lambda: PressFitHole(
            SingleRowCappedDeepGrooveBallBearing("M5-10-4", "OpenBuilds"), 0.02
        ),
        "bdw_bearing_press_fit_hole_depth": lambda: V(
            [
                PressFitHole(SingleRowTaperedRollerBearing("M25-52-19.25")).hole_depth,
                PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7"), depth=12.5).hole_depth,
            ]
        ),
        "bdw_bearing_press_fit_hole_bad_fit": lambda: error_of(
            lambda: PressFitHole(SingleRowDeepGrooveBallBearing("M8-22-7"), fit="Snug")
        ),
        # class functions
        "bdw_bearing_types": lambda: V([[name, sorted(c.types())] for name, c in CLASSES.items()]),
        "bdw_bearing_sizes": lambda: V([[name, t, c.sizes(t)] for name, c in CLASSES.items() for t in sorted(c.types())]),
        "bdw_bearing_select_by_size": lambda: V(
            [
                [[c.__name__, sorted(t)] for c, t in Bearing.select_by_size(size).items()]
                for size in ["M8-22-7", "M5-10-4", "M17-47-14", "M25-52-15", "M25-52-19.25", "M99-1-1"]
            ]
        ),
        "bdw_bearing_clearance_tables": lambda: V(
            [
                len(Bearing.clearance_hole_drill_sizes),
                [[k, v] for k, v in Bearing.clearance_hole_drill_sizes["M8"].items()],
                [[k, num(v)] for k, v in Bearing.clearance_hole_data["M8"].items()],
            ]
        ),
        "bdw_bearing_method_exists_fastener": lambda: V(
            [
                method_exists(HexNut, "nut_profile"),
                method_exists(HexNut, "head_profile"),
                method_exists(SetScrew, "head_profile"),
                method_exists(SetScrew, "custom_make"),
                method_exists(SocketHeadCapScrew, "head_recess"),
                method_exists(SocketHeadCapScrew, "flange_profile"),
                method_exists(CounterSunkScrew, "countersink_profile"),
                method_exists(HeatSetNut, "knurled_cylinder_faces"),
                method_exists(PlainWasher, "washer_profile"),
                method_exists(PlainWasher, "nut_profile"),
                method_exists(SingleRowCappedDeepGrooveBallBearing, "cap"),
            ]
        ),
        # errors
        "bdw_bearing_bad_type": lambda: error_of(lambda: SingleRowDeepGrooveBallBearing("M8-22-7", "ABC")),
        "bdw_bearing_bad_size": lambda: error_of(lambda: SingleRowAngularContactBallBearing("M8-22-7")),
        "bdw_bearing_empty_size": lambda: error_of(lambda: SingleRowDeepGrooveBallBearing("  ")),
        "bdw_bearing_size_stripped": lambda: V(SingleRowDeepGrooveBallBearing(" M8-22-7 ").label),
        "bdw_bearing_tapered_fillet_fails": lambda: V(
            error_of(lambda: SingleRowTaperedRollerBearing("M32-53-14.5")).data[0]
        ),
    }
)
