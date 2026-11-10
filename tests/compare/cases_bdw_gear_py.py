"""Python twins of cases_bdw_gear.cpp: bd_warehouse's gear.py."""

import math

from build123d import Align

from bd_warehouse.gear import (
    HelicalGear,
    InvoluteToothProfile,
    RackGear,
    RackGearPlan,
    SpurGear,
    SpurGearPlan,
    Worm,
    WormFlank,
    WormGear,
    WormWheel,
)

from bdw_util import error_of, loc_values
from values import V

WORM_ATTRIBUTES = (
    "module", "starts", "diametral_quotient", "flank_form", "normal_pressure_angle", "axial_pressure_angle",
    "transverse_pressure_angle", "addendum", "dedendum", "pitch_radius", "addendum_radius", "root_radius",
    "base_radius", "axial_pitch", "lead", "lead_angle", "base_lead_angle",
)  # fmt: skip


def finite(x):
    return None if isinstance(x, float) and math.isinf(x) else x


def attrs_of(obj, names):
    return [[n, finite(getattr(obj, n))] for n in names]


def material_name(obj):
    return obj.material.material.name if obj.material is not None else None


TOOTH_ATTRS = ["module", "tooth_count", "pitch_radius", "base_radius", "addendum", "addendum_radius", "dedendum", "root_radius"]
RACK_ATTRS = [
    "normal_module", "transverse_module", "normal_pressure_angle", "transverse_pressure_angle", "module",
    "module_system", "tooth_count", "pressure_angle", "helix_angle", "addendum", "dedendum", "root_fillet",
    "base_height", "pitch", "pitch_length", "tooth_height",
]  # fmt: skip
SPUR_ATTRS = ["pitch_radius", "base_radius", "addendum_radius", "root_radius"]
HELICAL_ATTRS = [
    "normal_module", "transverse_module", "normal_pressure_angle", "transverse_pressure_angle", "module",
    "module_system", "tooth_count", "pressure_angle", "helix_angle", "thickness", "pitch_radius", "base_radius",
    "addendum_radius", "root_radius", "twist_angle", "lead",
]  # fmt: skip

TOOTH_SPECS = {
    "m2_z12_14p5_fillet": lambda: InvoluteToothProfile(2, 12, 14.5, 0.5),
    "m2_z12_20": lambda: InvoluteToothProfile(2, 12, 20),
    "m1_z20_20": lambda: InvoluteToothProfile(1, 20, 20, root_fillet=0.2),
    "m1_z40_20_fillet_fails": lambda: InvoluteToothProfile(1, 40, 20, root_fillet=0.2),
    "m3_z8_25_add": lambda: InvoluteToothProfile(3, 8, 25, addendum=2.5, dedendum=3.5),
    "m0p5_z100_20": lambda: InvoluteToothProfile(0.5, 100, 20),
}
RACK_PLAN_SPECS = {
    "m2_z10_20": lambda: RackGearPlan(2, 10, 20),
    "m2_z10_20_fillet": lambda: RackGearPlan(2, 10, 20, root_fillet=0.3),
    "m1p5_z6_14p5_helix": lambda: RackGearPlan(1.5, 6, 14.5, helix_angle=20, base_height=5),
    "m2_z5_20_transverse": lambda: RackGearPlan(2, 5, 20, helix_angle=-15, module_system="transverse", addendum=1.5),
    "m2_z4_20_rotated": lambda: RackGearPlan(2, 4, 20, rotation=30, align=Align.CENTER),
}
RACK_SPECS = {
    "m2_z10_20_t8": lambda: RackGear(2, 10, 20, 8),
    "m2_z10_20_t8_helix": lambda: RackGear(2, 10, 20, 8, helix_angle=15),
    "m2_z10_20_t8_helix_neg": lambda: RackGear(2, 10, 20, 8, helix_angle=-25, module_system="transverse"),
    "m1_z20_20_t5_fillet": lambda: RackGear(1, 20, 20, 5, root_fillet=0.25, dedendum=1.4),
    "m2_z6_20_t6_rotated": lambda: RackGear(2, 6, 20, 6, rotation=(0, 0, 90), align=Align.MIN),
}
SPUR_SPECS = {
    "m2_z12_14p5_t5_fillet": lambda: SpurGear(2, 12, 14.5, 5, 0.5),
    "m1p5_z31_20_t4": lambda: SpurGear(1.5, 31, 20, 4),
    "m1_z20_20_t3_addendum": lambda: SpurGear(1, 20, 20, 3, addendum=0.8, dedendum=1.3),
    "m2_z15_20_t6_rotated": lambda: SpurGear(2, 15, 20, 6, rotation=(90, 0, 0), align=(Align.MIN, Align.CENTER, Align.MAX)),
}
HELICAL_SPECS = {
    "m2_z13_20_h45_t10": lambda: HelicalGear(2, 13, 20, 45, 10),
    "m2_z20_20_hm20_t8_transverse": lambda: HelicalGear(2, 20, 20, -20, 8, module_system="transverse"),
    "m1_z30_20_h0_t5": lambda: HelicalGear(1, 30, 20, 0, 5),
    "m1p5_z18_20_h15_t6_fillet": lambda: HelicalGear(1.5, 18, 20, 15, 6, root_fillet=0.3, addendum=1.2),
    "m2_z12_20_h30_t8_aligned": lambda: HelicalGear(2, 12, 20, 30, 8, align=Align.MIN),
}
FLANK_SPECS = {
    "zi_lower": lambda: WormFlank(2, 2, 10, 30),
    "zi_upper_left": lambda: WormFlank(2, 1, 10, 30, side="upper", hand="left"),
    "za_lower": lambda: WormFlank(2, 1, 10, 30, flank_form="ZA"),
    "za_upper": lambda: WormFlank(1.5, 3, 12, 20, flank_form="ZA", side="upper", pressure_angle=15),
    "zi_low_q": lambda: WormFlank(2, 1, 6, 25, pressure_angle=25),
    "zi_addendum": lambda: WormFlank(3, 2, 9, 40, addendum=2.5, dedendum=3.0),
}
WORM_SPECS = {
    "zi_m2_s2_q10_l30": lambda: Worm(2, 2, 10, 30),
    "za_m2_s1_q10_l30": lambda: Worm(2, 1, 10, 30, flank_form="ZA"),
    "zi_m2_s3_q12_l40_left": lambda: Worm(2, 3, 12, 40, hand="left"),
    "zi_m1_s1_q8_l15": lambda: Worm(1, 1, 8, 15),
    "zi_m1p5_s4_q10_l25_pa25": lambda: Worm(1.5, 4, 10, 25, pressure_angle=25),
    "za_m2_s2_q11_l12_left_rotated": lambda: Worm(2, 2, 11, 12, flank_form="ZA", hand="left", rotation=(90, 0, 0), align=Align.MAX),
}
WHEEL_SPECS = {
    "m2_z30_t12_s2": lambda: WormWheel(2, 30, 12, 2),
    "m2_z40_t10": lambda: WormWheel(2, 40, 10),
    "m1p5_z25_t8_left": lambda: WormWheel(1.5, 25, 8, starts=3, diametral_quotient=12, hand="left", root_fillet=0.2),
}
GEAR_SPECS = {
    "m2_s2_q10_z30": lambda: WormGear(2, 2, 10, 30, 30, 12),
    "m1p5_s1_q8_z20_za": lambda: WormGear(1.5, 1, 8, 20, 20, 8, flank_form="ZA"),
    "m2_s3_q12_z36_left": lambda: WormGear(2, 3, 12, 36, 40, 10, hand="left", root_fillet=0.1),
}

CASES = {}
for name, make in TOOTH_SPECS.items():
    CASES[f"bdw_gear_tooth_{name}"] = make
    CASES[f"bdw_gear_tooth_{name}_values"] = lambda m=make: V(attrs_of(m(), TOOTH_ATTRS))
for name, make in RACK_PLAN_SPECS.items():
    CASES[f"bdw_gear_rack_plan_{name}"] = make
    CASES[f"bdw_gear_rack_plan_{name}_values"] = lambda m=make: V(attrs_of(m(), RACK_ATTRS))
for name, make in RACK_SPECS.items():
    CASES[f"bdw_gear_rack_{name}"] = make
    CASES[f"bdw_gear_rack_{name}_values"] = lambda m=make: V(attrs_of(m(), RACK_ATTRS + ["thickness", "lateral_offset"]))
for name, make in SPUR_SPECS.items():
    CASES[f"bdw_gear_spur_{name}"] = make
    CASES[f"bdw_gear_spur_{name}_values"] = lambda m=make: V(attrs_of(m(), SPUR_ATTRS) + [material_name(m())])
for name, make in HELICAL_SPECS.items():
    CASES[f"bdw_gear_helical_{name}"] = make
    CASES[f"bdw_gear_helical_{name}_values"] = lambda m=make: V(attrs_of(m(), HELICAL_ATTRS) + [material_name(m())])
for name, make in FLANK_SPECS.items():
    CASES[f"bdw_gear_flank_{name}"] = make
    CASES[f"bdw_gear_flank_{name}_values"] = lambda m=make: V(
        attrs_of(m(), list(WORM_ATTRIBUTES) + ["axial_length", "hand", "side", "inner_radius"])
    )
for name, make in WORM_SPECS.items():
    CASES[f"bdw_gear_worm_{name}"] = make
    CASES[f"bdw_gear_worm_{name}_values"] = lambda m=make: V(
        attrs_of(m(), list(WORM_ATTRIBUTES) + ["length", "hand"]) + [material_name(m())]
    )
for name, make in WHEEL_SPECS.items():
    CASES[f"bdw_gear_wheel_{name}"] = make
    CASES[f"bdw_gear_wheel_{name}_values"] = lambda m=make: V(
        attrs_of(
            m(), HELICAL_ATTRS + ["starts", "diametral_quotient", "hand", "lead_angle", "axial_pressure_angle", "center_distance"]
        )
        + [material_name(m())]
    )
for name, make in GEAR_SPECS.items():
    CASES[f"bdw_gear_worm_gear_{name}"] = make
    CASES[f"bdw_gear_worm_gear_{name}_values"] = lambda m=make: V(
        [
            m().center_distance,
            m().ratio,
            [[c.label, c.volume, material_name(c), loc_values(c.location)] for c in m().children],
            [[n, loc_values(j.relative_location)] for n, j in m().joints.items()],
        ]
    )

CASES.update(
    {
        "bdw_gear_spur_plan": lambda: SpurGearPlan(2, 12, 14.5, 0.5),
        "bdw_gear_spur_plan_rotated": lambda: SpurGearPlan(1, 25, 20, rotation=10, align=Align.MIN),
        "bdw_gear_spur_plan_values": lambda: V(attrs_of(SpurGearPlan(1, 25, 20), SPUR_ATTRS)),
        "bdw_gear_errors": lambda: V(
            # (the message of a failed fillet is build123d's: only its kind is compared)
            [error_of(lambda: InvoluteToothProfile(2, 12, 20, root_fillet=5)).data[0]]
            + [
                error_of(f).data
                for f in [
                    lambda: InvoluteToothProfile(2, 12, 20, closed=True),
                    lambda: RackGearPlan(0, 10, 20),
                    lambda: RackGearPlan(2, 0, 20),
                    lambda: RackGearPlan(2, 10, 90),
                    lambda: RackGearPlan(2, 10, 20, helix_angle=90),
                    lambda: RackGearPlan(2, 10, 20, module_system="axial"),
                    lambda: RackGearPlan(2, 10, 20, root_fillet=-1),
                    lambda: RackGearPlan(2, 10, 20, addendum=0),
                    lambda: RackGearPlan(2, 10, 20, dedendum=-1),
                    lambda: RackGearPlan(2, 10, 20, base_height=0),
                    lambda: RackGearPlan(2, 10, 45, addendum=3),
                    lambda: RackGearPlan(2, 10, 45, dedendum=3),
                    lambda: RackGearPlan(2, 10, 20, root_fillet=3),
                    lambda: RackGear(2, 10, 20, 0),
                    lambda: HelicalGear(0, 13, 20, 45, 10),
                    lambda: HelicalGear(2, 0, 20, 45, 10),
                    lambda: HelicalGear(2, 13, 20, 45, 0),
                    lambda: HelicalGear(2, 13, -1, 45, 10),
                    lambda: HelicalGear(2, 13, 20, -90, 10),
                    lambda: HelicalGear(2, 13, 20, 45, 10, module_system="axial"),
                    lambda: WormFlank(2, 1, 10, 0),
                    lambda: WormFlank(2, 1, 10, 30, hand="up"),
                    lambda: WormFlank(2, 1, 10, 30, side="middle"),
                    lambda: WormFlank(0, 1, 10, 30),
                    lambda: WormFlank(2, 0, 10, 30),
                    lambda: WormFlank(2, 1, 0, 30),
                    lambda: WormFlank(2, 1, 10, 30, pressure_angle=0),
                    lambda: WormFlank(2, 1, 10, 30, flank_form="ZK"),
                    lambda: WormFlank(2, 1, 10, 30, addendum=0),
                    lambda: WormFlank(2, 1, 10, 30, dedendum=-2),
                    lambda: WormFlank(2, 1, 2, 30),
                    lambda: WormFlank(2, 1, 10, 30, pressure_angle=60, flank_form="ZA"),
                    lambda: Worm(2, 1, 10, -5),
                    lambda: Worm(2, 1, 10, 30, hand="up"),
                    lambda: WormWheel(2, 30, 12, hand="up"),
                    lambda: WormGear(2, 1, 10, 30, 30, 12, flank_form="ZK"),
                ]
            ]
        ),
    }
)
