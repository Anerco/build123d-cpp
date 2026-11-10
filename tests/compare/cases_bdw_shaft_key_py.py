"""Python twins of cases_bdw_shaft_key.cpp: bd_warehouse's shaft_key.py."""

from build123d import Align

from bd_warehouse.shaft_key import Keyway, ShaftKey

from bdw_util import error_of, loc_values, num
from values import V

KEYWAY_DIAMETERS = [6, 10, 12, 22, 30, 50]


def key_length(d):
    return 2.5 * d


def key_values(k):
    return V(
        [
            k.label, k.info, k.key_form, k.shaft_diameter, [[n, num(v)] for n, v in k.key_dict.items()], k.length,
            k.width, k.key_height, k.plan_area, list(k.color), loc_values(k.joints["a"].relative_location),
        ]
    )  # fmt: skip


def keyway_values(w):
    return V(
        [
            w.label, w.keyway_type, w.fit, w.nominal_width, w.length, w.keyway_depth, w.keyway_depth_tolerance,
            w.width_tolerance, w.min_width, w.max_width, w.width, w.key.label,
        ]
    )  # fmt: skip


CASES = {}
for d in ShaftKey.sizes():
    for form in ["A", "B"]:
        CASES[f"bdw_shaft_key_{d}_{form}"] = lambda d=d, f=form: ShaftKey(d, key_length(d), f)
        CASES[f"bdw_shaft_key_{d}_{form}_values"] = lambda d=d, f=form: key_values(ShaftKey(d, key_length(d), f))
for d in KEYWAY_DIAMETERS:
    for keyway_type in ["shaft", "hub"]:
        for fit in ["Tight", "Loose", "Sliding"]:
            for form in ["A", "B"]:
                name = f"bdw_shaft_key_keyway_{d}_{keyway_type}_{fit}_{form}"
                CASES[name] = lambda d=d, t=keyway_type, f=fit, k=form: Keyway(ShaftKey(d, key_length(d), k), t, f)
                CASES[name + "_values"] = lambda d=d, t=keyway_type, f=fit, k=form: keyway_values(
                    Keyway(ShaftKey(d, key_length(d), k), t, f)
                )
    CASES[f"bdw_shaft_key_broach_{d}"] = lambda d=d: Keyway(ShaftKey(d, key_length(d)), "hub").broach_profile()

CASES.update(
    {
        "bdw_shaft_key_sizes": lambda: V(ShaftKey.sizes()),
        "bdw_shaft_key_key_data": lambda: V([[k, [[n, num(v)] for n, v in row.items()]] for k, row in ShaftKey.key_data.items()]),
        "bdw_shaft_key_parameters": lambda: V([[n, num(v)] for n, v in ShaftKey.parameters(12.0).items()]),
        "bdw_shaft_key_select_by_size": lambda: V(
            [[[c.__name__, s] for c, s in ShaftKey.select_by_size(d).items()] for d in [12, 19, 50.0, 51]]
        ),
        "bdw_shaft_key_keyway_fits": lambda: V([[k, [[n, v] for n, v in row.items()]] for k, row in Keyway.keyway_fits.items()]),
        "bdw_shaft_key_fractional_length": lambda: key_values(ShaftKey(20, 25.5, "B")),
        "bdw_shaft_key_rotated": lambda: ShaftKey(20, 30, rotation=(0, 0, 90), align=Align.MIN),
        "bdw_shaft_key_keyway_rotated": lambda: Keyway(
            ShaftKey(20, 30), "hub", "Tight", rotation=(90, 0, 0), align=(Align.CENTER, Align.MIN, Align.MAX)
        ),
        "bdw_shaft_key_errors": lambda: V(
            [
                error_of(lambda: ShaftKey(12, 20, "C")).data,
                error_of(lambda: ShaftKey(12.5, 20)).data,
                error_of(lambda: ShaftKey(float("inf"), 20)).data,
                error_of(lambda: ShaftKey(0, 20)).data,
                error_of(lambda: ShaftKey(-6, 20)).data,
                error_of(lambda: ShaftKey(19, 20)).data,
                error_of(lambda: ShaftKey(12, 0)).data,
                error_of(lambda: ShaftKey(12, 3.5)).data,
                error_of(lambda: ShaftKey.parameters(51)).data,
                error_of(lambda: ShaftKey.select_by_size(1.5)).data,
                error_of(lambda: Keyway(ShaftKey(12, 20), "bore")).data,
                error_of(lambda: Keyway(ShaftKey(12, 20), "hub", "Press")).data,
                error_of(lambda: Keyway(ShaftKey(12, 20), "shaft").broach_profile()).data,
            ]
        ),
    }
)
