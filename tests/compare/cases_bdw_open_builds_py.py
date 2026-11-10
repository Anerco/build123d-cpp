"""Python twins of cases_bdw_open_builds.cpp: bd_warehouse's open_builds.py."""

import re

from build123d import Align

import bd_warehouse.open_builds as ob
from bd_warehouse.open_builds import (
    AcmeAntiBacklashNutBlock8mm,
    AcmeAntiBacklashNutBlock8mmAssembly,
    AluminumSpacer,
    CBeamAssembly,
    CBeamCapped,
    CBeamEndAssembly,
    CBeamEndMount,
    CBeamGantryPlate,
    CBeamGantryPlateXLarge,
    CBeamLinearRail,
    CBeamLinearRailProfile,
    CBeamRiserPlate,
    EccentricSpacer,
    FlexibleCoupler,
    LBracket,
    LockCollar,
    MetricLeadScrew,
    RouterSpindleMount,
    ShimWasher,
    SpacerBlock,
    StepperMotor,
    TNut,
    VSlotLinearRail,
    VSlotLinearRailProfile,
    XLargeCBeamGantry,
    XtremeSolidVWheel,
    XtremeSolidVWheelAssembly,
)

from bdw_util import error_of
from values import V

SPACER_LENGTHS = ["3mm", "1/8in", "6mm", "1/4in", "9mm", "10mm", "13.2mm", "20mm", "35mm", "1-1/2in", "40mm"]
SHIM_TYPES = ["MiniVWheel", "10x5x1", "12x8x1", "SlotWasher", "FlatWasher"]
RAIL_SIZES = ["20x20", "20x40", "20x60", "20x80", "40x40"]
MOTOR_TYPES = ["Nema17", "Nema23", "Nema23HighTorque"]


def key(text):
    return re.sub(r"[^A-Za-z0-9]+", "_", str(text)).strip("_")


def error(build):
    """error_of, with Python's UnboundLocalError as the port's runtime_error."""
    try:
        return error_of(build).data
    except UnboundLocalError as e:
        return ["runtime_error", str(e)]


def frame(loc):
    """A location as its position and x and z directions (Euler angles of 180 degrees flip
    between -180 and 180 with the sign of rounding noise)."""
    return [loc.position, loc.x_axis.direction, loc.z_axis.direction]


def joint_values(shape):
    return [[name, type(j).__name__, frame(j.location)] for name, j in shape.joints.items()]


def material_values(shape):
    """The material's fields (its repr prints a thickness given as an int without ".0")."""
    m = shape.material
    if m is None:
        return None
    return [
        m.material.name, None if m.finish is None else repr(m.finish), None if m.color is None else repr(m.color),
        m.thickness_mm, m.opacity, m.roughness, list(m.scale), m.rotation,
    ]  # fmt: skip


def color_values(shape):
    return None if shape.color is None else list(shape.color)


def shape_values(shape):
    return [shape.label, material_values(shape), color_values(shape), frame(shape.location), joint_values(shape)]


def assembly_values(shape):
    children = [
        [c.label, c.volume, frame(c.location), material_values(c), color_values(c), len(c.children)]
        for c in shape.children
    ]
    return shape_values(shape) + [children]


def xlarge_plate_values(plate):
    return shape_values(plate) + [plate.thickness]


CASES = {}


def add(name, make, values=shape_values):
    CASES[f"bdw_open_builds_{name}"] = make
    CASES[f"bdw_open_builds_{name}_values"] = lambda: V(values(make()))


# ---- parts
add("acme_nut_block", lambda: AcmeAntiBacklashNutBlock8mm())
add(
    "acme_nut_block_rotated",
    lambda: AcmeAntiBacklashNutBlock8mm(rotation=(0, 0, 45), align=(Align.CENTER, Align.CENTER, Align.MIN)),
)
for length in SPACER_LENGTHS:
    add(f"aluminum_spacer_{key(length)}", lambda l=length: AluminumSpacer(l))
add("aluminum_spacer_aligned", lambda: AluminumSpacer("20mm", rotation=(90, 0, 0), align=Align.MIN))
add("cbeam_end_mount", lambda: CBeamEndMount())
add("cbeam_end_mount_align_none", lambda: CBeamEndMount(align=Align.NONE))
add("cbeam_end_mount_rotated", lambda: CBeamEndMount(rotation=(90, 0, -90), align=None))
add("cbeam_linear_rail_profile", lambda: CBeamLinearRailProfile())
add("cbeam_linear_rail_profile_rotated", lambda: CBeamLinearRailProfile(rotation=30, align=(Align.MIN, Align.MAX)))
add("cbeam_linear_rail", lambda: CBeamLinearRail(100.0))
add(
    "cbeam_linear_rail_rotated",
    lambda: CBeamLinearRail(250.5, rotation=(-90, 0, 0), align=(Align.MIN, Align.CENTER, Align.CENTER)),
)
add("cbeam_gantry_plate", lambda: CBeamGantryPlate())
add("cbeam_gantry_plate_xlarge", lambda: CBeamGantryPlateXLarge(), xlarge_plate_values)
add("cbeam_gantry_plate_xlarge_aligned", lambda: CBeamGantryPlateXLarge(rotation=(0, 0, 30), align=Align.MAX))
add("cbeam_riser_plate", lambda: CBeamRiserPlate())
add("eccentric_spacer_6mm", lambda: EccentricSpacer("6mm"))
add("eccentric_spacer_1_4in", lambda: EccentricSpacer("1/4in"))
add("eccentric_spacer_default", lambda: EccentricSpacer())
add("flexible_coupler_8mm", lambda: FlexibleCoupler("8mm"))
add("flexible_coupler_1_4in", lambda: FlexibleCoupler("1/4in", rotation=(0, 90, 0), align=Align.CENTER))
add("lbracket", lambda: LBracket())
add("lbracket_aligned", lambda: LBracket(rotation=(0, 0, 90), align=Align.CENTER))
add("router_spindle_mount", lambda: RouterSpindleMount())
add("router_spindle_mount_aligned", lambda: RouterSpindleMount("both", rotation=(0, 0, 180), align=Align.MIN))
for shim_type in SHIM_TYPES:
    add(f"shim_washer_{key(shim_type)}", lambda t=shim_type: ShimWasher(t))
add("spacer_block", lambda: SpacerBlock())
add("tnut", lambda: TNut())
add("tnut_aligned", lambda: TNut(rotation=(180, 0, 0), align=Align.MIN))
for size in RAIL_SIZES:
    add(f"vslot_profile_{size}", lambda s=size: VSlotLinearRailProfile(s))
    add(f"vslot_rail_{size}", lambda s=size: VSlotLinearRail(s, 50.0))
add("vslot_profile_default", lambda: VSlotLinearRailProfile())
add("vslot_profile_rotated", lambda: VSlotLinearRailProfile("40x40", rotation=45, align=(Align.MIN, Align.MIN)))
add("vslot_rail_rotated", lambda: VSlotLinearRail("20x40", 120.5, rotation=(0, 90, 0), align=None))
add("xtreme_solid_vwheel", lambda: XtremeSolidVWheel())
add("xtreme_solid_vwheel_align_none", lambda: XtremeSolidVWheel(align=None))

# ---- assemblies
add("acme_nut_block_assembly", lambda: AcmeAntiBacklashNutBlock8mmAssembly(), assembly_values)
add("cbeam_assembly", lambda: CBeamAssembly(300.0), assembly_values)
add("cbeam_assembly_gantry", lambda: CBeamAssembly(300.0, gantry_position=100.0), assembly_values)
add("cbeam_assembly_no_end_plate", lambda: CBeamAssembly(300.0, end_plate=False), assembly_values)
add(
    "cbeam_assembly_screw_length",
    lambda: CBeamAssembly(250.0, screw_length=200.0, end_plate=False),
    assembly_values,
)
add(
    "cbeam_assembly_thread",
    lambda: CBeamAssembly(120.0, gantry_position=40.0, simple=False),
    assembly_values,
)
add("cbeam_capped", lambda: CBeamCapped(200.0), assembly_values)
add("cbeam_end_assembly", lambda: CBeamEndAssembly(), assembly_values)
for size in ["5mm", "8mm", "1/4in"]:
    add(f"lock_collar_{key(size)}", lambda s=size: LockCollar(s), assembly_values)
add("metric_lead_screw", lambda: MetricLeadScrew(100.0), assembly_values)
add("metric_lead_screw_thread", lambda: MetricLeadScrew(30.0, simple=False), assembly_values)
add("xlarge_cbeam_gantry_4", lambda: XLargeCBeamGantry(4), assembly_values)
add("xlarge_cbeam_gantry_6", lambda: XLargeCBeamGantry(6), assembly_values)
add("xlarge_cbeam_gantry_4_60", lambda: XLargeCBeamGantry(4, 60.0), assembly_values)
add("xlarge_cbeam_gantry_default", lambda: XLargeCBeamGantry(), assembly_values)
add("xtreme_solid_vwheel_assembly_eccentric", lambda: XtremeSolidVWheelAssembly(True), assembly_values)
add("xtreme_solid_vwheel_assembly", lambda: XtremeSolidVWheelAssembly(False), assembly_values)
for motor_type in MOTOR_TYPES:
    add(f"stepper_motor_{motor_type}", lambda t=motor_type: StepperMotor(t), assembly_values)
add("stepper_motor_Nema17_30", lambda: StepperMotor("Nema17", 30.0), assembly_values)
add("stepper_motor_Nema23_120", lambda: StepperMotor("Nema23", 120.5), assembly_values)
add("stepper_motor_Nema23HighTorque_15", lambda: StepperMotor("Nema23HighTorque", 15.0), assembly_values)

CASES.update(
    {
        "bdw_open_builds_module_values": lambda: V(
            # (info would print the int length 20 * MM as "20"; C++ holds a double)
            [ob.CAVITY_RADIUS, ob.FILLET_RADIUS, ob.m5.label, ob.m5.volume, ob.m5.length]
        ),
        "bdw_open_builds_errors": lambda: V(
            [
                error(f)
                for f in [
                    lambda: AluminumSpacer("7mm"),
                    lambda: EccentricSpacer("5mm"),
                    lambda: FlexibleCoupler("5mm"),
                    lambda: LockCollar("6mm"),
                    lambda: ShimWasher("20x5x1"),
                    lambda: VSlotLinearRailProfile("30x30"),
                    lambda: VSlotLinearRail("20x30", 10.0),
                    lambda: XLargeCBeamGantry(5),
                    lambda: StepperMotor("Nema34"),
                    lambda: StepperMotor("Nema17", 0.0),
                    lambda: StepperMotor("Nema23", -5.5),
                    lambda: RouterSpindleMount("base"),
                    lambda: RouterSpindleMount("faceplate"),
                    lambda: RouterSpindleMount("top"),
                ]
            ]
        ),
    }
)
