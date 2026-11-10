"""Python twins of cases_bdw_sprocket.cpp: bd_warehouse's sprocket.py."""

from build123d import IN, Align

from bd_warehouse.sprocket import Sprocket

from bdw_util import error_of
from values import V

SPECS = {
    "z3": lambda: Sprocket(3),
    "z5": lambda: Sprocket(5),
    "z9": lambda: Sprocket(9),
    "z16": lambda: Sprocket(16),
    "z32": lambda: Sprocket(32),
    "z60": lambda: Sprocket(60),
    "z20_clearance": lambda: Sprocket(20, clearance=0.2),
    "z15_quarter_inch": lambda: Sprocket(15, chain_pitch=0.25 * IN, roller_diameter=0.13 * IN, thickness=0.11 * IN),
    "z25_small_rollers": lambda: Sprocket(25, roller_diameter=0.2 * IN),
    "z20_bolts_bore": lambda: Sprocket(
        20, bolt_circle_diameter=40, num_mount_bolts=4, mount_bolt_diameter=5, bore_diameter=20
    ),
    "z24_bore": lambda: Sprocket(24, bore_diameter=30),
    "z12_rotated": lambda: Sprocket(12, rotation=(90, 0, 0), align=Align.MIN),
    "z12_align_none": lambda: Sprocket(12, align=None),
}


def values(s):
    return V(
        [
            s.num_teeth, s.chain_pitch, s.roller_diameter, s.clearance, s.thickness, s.bolt_circle_diameter,
            s.num_mount_bolts, s.mount_bolt_diameter, s.bore_diameter, s.pitch_radius, s.outer_radius,
            s.pitch_circumference, s.plan.area, len(s.plan.edges()),
        ]
    )  # fmt: skip


CASES = {}
for name, make in SPECS.items():
    CASES[f"bdw_sprocket_{name}"] = make
    CASES[f"bdw_sprocket_{name}_values"] = lambda m=make: values(m())
for name in ["z5", "z32", "z20_clearance"]:
    CASES[f"bdw_sprocket_{name}_plan"] = lambda m=SPECS[name]: m().plan

CASES.update(
    {
        "bdw_sprocket_statics": lambda: V(
            [
                [Sprocket.sprocket_pitch_radius(n, p), Sprocket.sprocket_circumference(n, p)]
                for n in [3, 8, 32, 100]
                for p in [12.7, 6.35, 25.4]
            ]
        ),
        "bdw_sprocket_errors": lambda: V(
            [
                error_of(lambda: Sprocket(20, chain_pitch=10.0, roller_diameter=10.0)).data,
                error_of(lambda: Sprocket(20, chain_pitch=10.0, roller_diameter=12.5)).data,
                error_of(lambda: Sprocket(2)).data,
                error_of(lambda: Sprocket(-4)).data,
            ]
        ),
    }
)
