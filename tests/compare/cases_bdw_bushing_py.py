"""Python twins of cases_bdw_bushing.cpp: bd_warehouse's bushing.py."""

from build123d import Align

from bd_warehouse.bushing import HexFlangedEccentricBushing

from bdw_util import error_of, loc_values
from values import V

SPECS = [
    (5, 8, 6, 2.5, 13, 0.75),
    (5, 8, 6, 2.5, 13, 0),
    (8, 16, 10, 4, 19, 2.5),
    (3, 6, 3, 1.5, 6, 1),
    (12.7, 25.4, 20, 6, 32, 4.5),
]


def name(spec):
    return "bdw_bushing_" + "_".join(str(x).replace(".", "p") for x in spec)


def values(b):
    return V(
        [
            b.label, b.bore_diameter, b.body_diameter, b.body_length, b.flange_height, b.hex_size, b.eccentricity,
            b.overall_length, b.material.material.name, loc_values(b.location),
            loc_values(b.joints["outer_center"].relative_location),
            loc_values(b.joints["ecc_top_center"].relative_location),
        ]
    )  # fmt: skip


CASES = {}
for spec in SPECS:
    CASES[name(spec)] = lambda s=spec: HexFlangedEccentricBushing(*s)
    CASES[name(spec) + "_values"] = lambda s=spec: values(HexFlangedEccentricBushing(*s))

CASES.update(
    {
        "bdw_bushing_rotated": lambda: HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, rotation=(90, 0, 45)),
        "bdw_bushing_aligned": lambda: HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, align=Align.MIN),
        "bdw_bushing_aligned_values": lambda: values(
            HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, rotation=(0, 0, 30), align=(Align.CENTER, Align.MAX, Align.NONE))
        ),
        "bdw_bushing_align_none": lambda: HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 0.75, align=None),
        "bdw_bushing_errors": lambda: V(
            [
                error_of(lambda: HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, -0.5)).data,
                error_of(lambda: HexFlangedEccentricBushing(5, 8, 6, 2.5, 13, 1.5)).data,
                error_of(lambda: HexFlangedEccentricBushing(8, 8, 6, 2.5, 13, 0)).data,
                error_of(lambda: HexFlangedEccentricBushing(5, 8, 6, 2.5, 7.9, 0.5)).data,
            ]
        ),
    }
)
