"""Python twins of cases_bdw_fastener.cpp: bd_warehouse's fastener.py."""

import re

from build123d import IN, Align, Axis, Pos, Rotation

import bd_warehouse.fastener as f
from bd_warehouse.fastener import (
    ButtonHeadScrew,
    ButtonHeadWithCollarScrew,
    ChamferedWasher,
    CheeseHeadScrew,
    CheeseHeadWasher,
    ClearanceHole,
    CounterSunkScrew,
    DomedCapNut,
    HeatSetNut,
    HexHeadScrew,
    HexHeadWithFlangeScrew,
    HexNut,
    HexNutWithFlange,
    InsertHole,
    InternalToothLockWasher,
    LowProfileScrew,
    Nut,
    PanHeadScrew,
    PanHeadWithCollarScrew,
    PlainWasher,
    RaisedCheeseHeadScrew,
    RaisedCounterSunkOvalHeadScrew,
    Screw,
    SetScrew,
    ShoulderScrew,
    SocketHeadCapScrew,
    SquareNut,
    TapHole,
    ThreadedHole,
    UnchamferedHexagonNut,
    Washer,
)

from bdw_util import error_of, loc_values, num, table_values
from values import V

CLASSES = {c.__name__: c for c in Nut.__subclasses__() + Screw.__subclasses__() + Washer.__subclasses__()}


def case_name(*parts):
    return "bdw_fastener_" + "_".join(re.sub(r"[^A-Za-z0-9]+", "_", str(p)) for p in parts).strip("_")


# (class, type, size) of the nuts compared
NUT_SPECS = [
    ("DomedCapNut", "din1587", "M6-1"),
    ("DomedCapNut", "din1587", "M24-3"),
    ("HeatSetNut", "McMaster-Carr", "M3-0.5-Standard"),
    ("HeatSetNut", "McMaster-Carr", "M4-0.7-Short"),
    ("HeatSetNut", "Hilitchi", "M4-0.7-8"),
    ("HeatSetNut", "CNCKitchen", "M3-0.5-5.7"),
    ("HeatSetNut", "ruthex", "M2-0.4-4"),
    ("HeatSetNut", "AE-SamZhihui", "M1.4-0.3-H2-D2.3"),
    ("HexNut", "iso4032", "M1.6-0.35"),
    ("HexNut", "iso4032", "M6-1"),
    ("HexNut", "iso4032", "M64-6"),
    ("HexNut", "iso4033", "M10-1.5"),
    ("HexNut", "iso4035", "M8-1.25"),
    ("HexNutWithFlange", "din1665", "M5-0.8"),
    ("HexNutWithFlange", "din1665", "M20-1.5"),
    ("UnchamferedHexagonNut", "iso4036", "M2-0.4"),
    ("UnchamferedHexagonNut", "iso4036", "M10-1.5"),
    ("SquareNut", "din557", "M5-0.8"),
    ("SquareNut", "din557", "M16-2"),
]

# (class, type, size, length) of the screws compared
SCREW_SPECS = [
    ("ButtonHeadScrew", "iso7380_1", "M3-0.5", 8.0),
    ("ButtonHeadScrew", "iso7380_1", "M16-2", 40.0),
    ("ButtonHeadWithCollarScrew", "iso7380_2", "M5-0.8", 12.0),
    ("CheeseHeadScrew", "iso1207", "M3-0.5", 10.0),
    ("CheeseHeadScrew", "iso14580", "M4-0.7", 30.0),
    ("CheeseHeadScrew", "iso7048", "M5-0.8", 45.0),
    ("CounterSunkScrew", "iso10642", "M6-1", 20.0),
    ("CounterSunkScrew", "iso14581", "M3-0.5", 12.0),
    ("CounterSunkScrew", "iso14582", "M8-1.25", 30.0),
    ("CounterSunkScrew", "iso2009", "M5-0.8", 25.0),
    ("CounterSunkScrew", "iso7046", "M4-0.7", 50.0),
    ("HexHeadScrew", "din931", "M8-1.25", 40.0),
    ("HexHeadScrew", "iso4014", "M6-1", 30.0),
    ("HexHeadScrew", "iso4014", "M20-2.5", 220.0),
    ("HexHeadScrew", "iso4017", "M12-1.75", 25.0),
    ("HexHeadWithFlangeScrew", "din1662", "M6-1", 20.0),
    ("HexHeadWithFlangeScrew", "din1665", "M10-1.5", 30.0),
    ("PanHeadScrew", "iso14583", "M4-0.7", 10.0),
    ("PanHeadScrew", "iso1580", "M3-0.5", 45.0),
    ("PanHeadScrew", "asme_b_18.6.3", "#4-40", 0.5 * IN),
    ("PanHeadScrew", "asme_b_18.6.3", "1/4-20", 1.5 * IN),
    ("PanHeadWithCollarScrew", "din967", "M4-0.7", 12.0),
    ("RaisedCheeseHeadScrew", "iso7045", "M3-0.5", 8.0),
    ("RaisedCounterSunkOvalHeadScrew", "iso14584", "M5-0.8", 20.0),
    ("RaisedCounterSunkOvalHeadScrew", "iso2010", "M3-0.5", 12.0),
    ("RaisedCounterSunkOvalHeadScrew", "iso7047", "M6-1", 40.0),
    ("SetScrew", "iso4026", "M6-1", 10.0),
    ("SetScrew", "iso4026", "M3-0.5", 4.0),
    ("ShoulderScrew", "iso7379", "M6-1", 16.0),
    ("ShoulderScrew", "iso7379", "M10-1.5", 40.0),
    ("SocketHeadCapScrew", "iso4762", "M6-1", 20.0),
    ("SocketHeadCapScrew", "iso4762", "M3-0.5", 8.0),
    ("SocketHeadCapScrew", "iso4762", "M20-2.5", 100.0),
    ("SocketHeadCapScrew", "asme_b18.3", "#10-32", 0.75 * IN),
    ("SocketHeadCapScrew", "asme_b18.3", "1/4-20", 1 * IN),
    ("LowProfileScrew", "OpenBuilds", "M5-0.8", 8.0),
]

# (class, type, size) of the washers compared
WASHER_SPECS = [
    ("PlainWasher", "iso7089", "M6"),
    ("PlainWasher", "iso7091", "M10"),
    ("PlainWasher", "iso7093", "M3"),
    ("PlainWasher", "iso7094", "M8"),
    ("ChamferedWasher", "iso7090", "M6"),
    ("ChamferedWasher", "iso7090", "M20"),
    ("CheeseHeadWasher", "iso7092", "M4"),
    ("InternalToothLockWasher", "din6797", "M5"),
    ("InternalToothLockWasher", "asme_b18.21.1", "#10"),
    ("InternalToothLockWasher", "asme_b18.21.1", "1/4"),
]


def nut_values(n):
    return V(
        [
            n.label, n.info, n.nut_class, n.thread_size, n.thread_diameter, n.thread_pitch, n.is_metric,
            n.fastener_type, n.hand, n.simple, n.nut_size, n.nut_thickness, n.nut_diameter,
            [[k, num(v)] for k, v in n.nut_data.items()], loc_values(n.joints["a"].relative_location),
            loc_values(n.joints["b"].relative_location), n.material.material.name,
        ]
    )  # fmt: skip


def screw_values(s):
    return V(
        [
            s.label, s.info, s.screw_class, s.thread_size, s.thread_diameter, s.thread_pitch, s.is_metric,
            s.fastener_type, s.length, s.head_height, s.head_diameter, s.max_thread_length, s.thread_length,
            s.grip_length, s.thread_offset, s.nominal_lengths, s.min_hole_depth(), s.min_hole_depth(False),
            s.length_offset(), [[k, num(v)] for k, v in s.screw_data.items()],
            loc_values(s.joints["a"].relative_location), s.material.material.name,
        ]
    )  # fmt: skip


def washer_values(w):
    return V(
        [
            w.label, w.info, w.washer_class, w.thread_size, w.thread_diameter, w.washer_thickness,
            w.washer_diameter, [[k, num(v)] for k, v in w.washer_data.items()],
            loc_values(w.joints["b"].relative_location),
        ]
    )  # fmt: skip


def hole_tables(x):
    out = [
        [[k, v] for k, v in x.clearance_drill_sizes.items()],
        [[k, num(v)] for k, v in x.clearance_hole_diameters.items()],
    ]
    if not isinstance(x, Washer):
        out += [[[k, v] for k, v in x.tap_drill_sizes.items()], [[k, num(v)] for k, v in x.tap_hole_diameters.items()]]
    return V(out)


def select_values(result):
    return V([[cls.__name__, sorted(types)] for cls, types in result.items()])


def recess_values(result):
    face, depth = result
    return V([face.area, len(face.edges()), depth, [face.bounding_box().min, face.bounding_box().max]])


CASES = {}

for cls_name, fastener_type, size in NUT_SPECS:
    cls = CLASSES[cls_name]
    CASES[case_name(cls_name, fastener_type, size)] = lambda c=cls, t=fastener_type, s=size: c(s, t)
    CASES[case_name(cls_name, fastener_type, size, "values")] = lambda c=cls, t=fastener_type, s=size: nut_values(c(s, t))

for cls_name, fastener_type, size, length in SCREW_SPECS:
    cls = CLASSES[cls_name]
    CASES[case_name(cls_name, fastener_type, size, length)] = lambda c=cls, t=fastener_type, s=size, l=length: c(s, l, t)
    CASES[case_name(cls_name, fastener_type, size, length, "values")] = (
        lambda c=cls, t=fastener_type, s=size, l=length: screw_values(c(s, l, t))
    )

for cls_name, fastener_type, size in WASHER_SPECS:
    cls = CLASSES[cls_name]
    CASES[case_name(cls_name, fastener_type, size)] = lambda c=cls, t=fastener_type, s=size: c(s, t)
    CASES[case_name(cls_name, fastener_type, size, "values")] = lambda c=cls, t=fastener_type, s=size: washer_values(c(s, t))

CASES.update(
    {
        # threads, hands, rotations and alignments
        "bdw_fastener_hexnut_threaded": lambda: HexNut("M6-1", simple=False),
        "bdw_fastener_hexnut_threaded_left": lambda: HexNut("M8-1.25", "iso4035", hand="left", simple=False),
        "bdw_fastener_flange_nut_threaded": lambda: HexNutWithFlange("M8-1.25", simple=False),
        "bdw_fastener_heatset_threaded": lambda: HeatSetNut("M3-0.5-Standard", simple=False),
        "bdw_fastener_shcs_threaded": lambda: SocketHeadCapScrew("M4-0.7", 10, simple=False),
        "bdw_fastener_shcs_threaded_left": lambda: SocketHeadCapScrew("M5-0.8", 30, hand="left", simple=False),
        "bdw_fastener_countersunk_threaded": lambda: CounterSunkScrew("M5-0.8", 12, simple=False),
        "bdw_fastener_setscrew_threaded": lambda: SetScrew("M5-0.8", 8, simple=False),
        "bdw_fastener_shoulder_threaded": lambda: ShoulderScrew("M5-0.8", 10, simple=False),
        "bdw_fastener_hexnut_rotated": lambda: HexNut("M6-1", rotation=(90, 0, 30), align=Align.MIN),
        "bdw_fastener_hexnut_threaded_rotated": lambda: HexNut("M5-0.8", simple=False, rotation=(90, 0, 0)),
        "bdw_fastener_shcs_aligned": lambda: SocketHeadCapScrew("M6-1", 16, align=(Align.CENTER, Align.CENTER, Align.MIN)),
        "bdw_fastener_washer_rotated": lambda: PlainWasher("M6", "iso7089", rotation=(0, 45, 0), align=Align.MAX),
        "bdw_fastener_hexnut_threaded_children": lambda: V([[c.label, c.volume] for c in HexNut("M5-0.8", simple=False).children]),
        # profiles
        "bdw_fastener_hexnut_profile": lambda: HexNut("M6-1").nut_profile(),
        "bdw_fastener_hexnut_plan": lambda: HexNut("M6-1").nut_plan(),
        "bdw_fastener_hexnut_countersink": lambda: HexNut("M6-1").countersink_profile("Normal"),
        "bdw_fastener_flange_nut_flange_profile": lambda: HexNutWithFlange("M10-1.5").flange_profile(),
        "bdw_fastener_flange_nut_countersink": lambda: HexNutWithFlange("M10-1.5").countersink_profile("Close"),
        "bdw_fastener_domed_nut_countersink": lambda: DomedCapNut("M8-1.25").countersink_profile("Loose"),
        "bdw_fastener_square_nut_plan": lambda: SquareNut("M8-1.25").nut_plan(),
        "bdw_fastener_square_nut_countersink": lambda: SquareNut("M8-1.25").countersink_profile("Normal"),
        "bdw_fastener_heatset_countersink": lambda: HeatSetNut("M4-0.7-Standard").countersink_profile(0.2),
        "bdw_fastener_heatset_knurled_faces": lambda: V(
            [[x.area, len(x.edges())] for x in HeatSetNut.knurled_cylinder_faces(6, 2, 1.5, 3, 0.5, 9, 8, "left")]
        ),
        "bdw_fastener_heatset_fill_factor": lambda: V(HeatSetNut("M3-0.5-Short").fill_factor),
        "bdw_fastener_heatset_diameter": lambda: V(HeatSetNut("M4-0.7-8", "Hilitchi").nut_diameter),
        "bdw_fastener_shcs_head_profile": lambda: SocketHeadCapScrew("M6-1", 20).head_profile(),
        "bdw_fastener_shcs_countersink": lambda: SocketHeadCapScrew("M6-1", 20).countersink_profile("Close"),
        "bdw_fastener_shcs_head": lambda: SocketHeadCapScrew("M6-1", 20).make_head(),
        "bdw_fastener_shcs_shank": lambda: SocketHeadCapScrew("M8-1.25", 50).shank_profile(),
        "bdw_fastener_shcs_recess": lambda: SocketHeadCapScrew("M6-1", 20).head_recess()[0],
        "bdw_fastener_shcs_body_hole": lambda: SocketHeadCapScrew("M6-1", 20).make_body_hole(3.2, 12),
        "bdw_fastener_hexhead_plan": lambda: HexHeadScrew("M10-1.5", 30).head_plan(),
        "bdw_fastener_hexhead_countersink": lambda: HexHeadScrew("M10-1.5", 30).countersink_profile("Normal"),
        "bdw_fastener_flange_screw_flange": lambda: HexHeadWithFlangeScrew("M8-1.25", 30).flange_profile(),
        "bdw_fastener_flange_screw_countersink": lambda: HexHeadWithFlangeScrew("M8-1.25", 30).countersink_profile("Loose"),
        "bdw_fastener_countersunk_countersink": lambda: CounterSunkScrew("M6-1", 20).countersink_profile("Normal"),
        "bdw_fastener_oval_countersink": lambda: RaisedCounterSunkOvalHeadScrew("M5-0.8", 20).countersink_profile("Normal"),
        "bdw_fastener_collar_countersink": lambda: ButtonHeadWithCollarScrew("M6-1", 16).countersink_profile("Normal"),
        "bdw_fastener_shoulder_shank": lambda: ShoulderScrew("M8-1.25", 20).shank_profile(),
        "bdw_fastener_shoulder_countersink": lambda: ShoulderScrew("M8-1.25", 20).countersink_profile("Normal"),
        "bdw_fastener_shoulder_body_hole": lambda: ShoulderScrew("M8-1.25", 20).make_body_hole(3.4, 40, 8),
        "bdw_fastener_setscrew_countersink": lambda: V(SetScrew("M6-1", 10).countersink_profile("Normal") is None),
        "bdw_fastener_lowprofile_head": lambda: LowProfileScrew("M4-0.7", 10).head_profile(),
        "bdw_fastener_washer_profile": lambda: ChamferedWasher("M8").washer_profile(),
        "bdw_fastener_washer_countersink": lambda: PlainWasher("M8", "iso7093").countersink_profile("Loose"),
        "bdw_fastener_cheese_washer_profile": lambda: CheeseHeadWasher("M5").washer_profile(),
        # tables, types, sizes
        "bdw_fastener_hole_tables_nut": lambda: hole_tables(HexNut("M6-1")),
        "bdw_fastener_hole_tables_screw": lambda: hole_tables(SocketHeadCapScrew("#10-32", 10, "asme_b18.3")),
        "bdw_fastener_hole_tables_washer": lambda: hole_tables(PlainWasher("M6", "iso7089")),
        "bdw_fastener_washer_clearance_drill_sizes": lambda: error_of(lambda: PlainWasher("M6", "iso7089").clearance_drill_sizes),
        "bdw_fastener_types": lambda: V([[name, sorted(c.types())] for name, c in CLASSES.items()]),
        "bdw_fastener_sizes": lambda: V([[name, t, c.sizes(t)] for name, c in CLASSES.items() for t in sorted(c.types())]),
        "bdw_fastener_select_nut": lambda: select_values(Nut.select_by_size("M6-1")),
        "bdw_fastener_select_screw": lambda: select_values(Screw.select_by_size("M6-1")),
        "bdw_fastener_select_screw_imperial": lambda: select_values(Screw.select_by_size("1/4-20")),
        "bdw_fastener_select_washer": lambda: select_values(Washer.select_by_size("M8")),
        "bdw_fastener_nominal_lengths": lambda: V(
            [[k, v] for k, v in Screw.nominal_length_range.items()]
        ),
        # functions
        "bdw_fastener_polygon_diagonal": lambda: V([f.polygon_diagonal(10), f.polygon_diagonal(8, 4), f.polygon_diagonal(5, 8)]),
        "bdw_fastener_read_csv": lambda: V(table_values(f.read_fastener_parameters_from_csv("square_nut_parameters.csv"))),
        "bdw_fastener_read_csv_missing": lambda: V(f.read_fastener_parameters_from_csv("missing.csv")),
        "bdw_fastener_decode_imperial": lambda: V(
            [list(f.decode_imperial_size(s)) for s in ["#10-32", "1/4-20", "#0-80", "1 1/2-6", "3/8-24"]]
        ),
        "bdw_fastener_decode_imperial_bad": lambda: error_of(lambda: f.decode_imperial_size("#7-32")),
        "bdw_fastener_metric_str_to_float": lambda: V(
            [num(f.metric_str_to_float(s)) for s in ["12", "2.5", "1/2", "PH2", "T30", " 4", "1//2"]]
        ),
        "bdw_fastener_evaluate": lambda: V(
            [[k, num(v)] for k, v in f.evaluate_parameter_dict({"a": "1/4", "b": "#7", "c": "1 1/2"}, is_metric=False).items()]
        ),
        "bdw_fastener_evaluate_dict_of_dict": lambda: V(
            table_values(f.evaluate_parameter_dict_of_dict({"M3": {"x": "3", "y": "R1"}, "M4": {"x": "4.5"}}))
        ),
        "bdw_fastener_isolate": lambda: V(
            table_values(f.isolate_fastener_type("iso4033", f.read_fastener_parameters_from_csv("hex_nut_parameters.csv")))
        ),
        "bdw_fastener_isolate_bad": lambda: error_of(lambda: f.isolate_fastener_type("x", {"M3": {"a:b:c": "1"}})),
        "bdw_fastener_drill_sizes": lambda: V([[k, v] for k, v in f.read_drill_sizes().items()]),
        "bdw_fastener_lookup_drill": lambda: V(
            table_values(f.lookup_drill_diameters({"M3": {"Close": "3.2", "Loose": "#30"}, "#4": {"Close": "1/8", "Normal": "C"}}))
        ),
        "bdw_fastener_cross_recess": lambda: recess_values(f.cross_recess("PH2")),
        "bdw_fastener_cross_recess_bad": lambda: error_of(lambda: f.cross_recess("PH5")),
        "bdw_fastener_hex_recess": lambda: f.hex_recess(5),
        "bdw_fastener_hexalobular_recess": lambda: recess_values(f.hexalobular_recess("T20")),
        "bdw_fastener_hexalobular_recess_t100": lambda: recess_values(f.hexalobular_recess("T100")),
        "bdw_fastener_hexalobular_recess_bad": lambda: error_of(lambda: f.hexalobular_recess("T7")),
        "bdw_fastener_slot_recess": lambda: f.slot_recess(5, 1),
        "bdw_fastener_square_recess": lambda: recess_values(f.square_recess("R2")),
        "bdw_fastener_square_recess_bad": lambda: error_of(lambda: f.square_recess("R4")),
        # holes
        "bdw_fastener_clearance_hole": lambda: ClearanceHole(SocketHeadCapScrew("M6-1", 20), depth=15),
        "bdw_fastener_clearance_hole_close": lambda: ClearanceHole(SocketHeadCapScrew("M4-0.7", 16), "Close", 10),
        "bdw_fastener_clearance_hole_loose_flat": lambda: ClearanceHole(
            HexHeadScrew("M8-1.25", 30), "Loose", 12, counter_sunk=False
        ),
        "bdw_fastener_clearance_hole_countersunk": lambda: ClearanceHole(CounterSunkScrew("M5-0.8", 16), depth=8),
        "bdw_fastener_clearance_hole_oval": lambda: ClearanceHole(RaisedCounterSunkOvalHeadScrew("M4-0.7", 16), depth=8),
        "bdw_fastener_clearance_hole_nut": lambda: ClearanceHole(HexNut("M6-1"), depth=10),
        "bdw_fastener_clearance_hole_captive_hex": lambda: ClearanceHole(HexNut("M6-1"), depth=10, captive_nut=True),
        "bdw_fastener_clearance_hole_captive_domed": lambda: ClearanceHole(DomedCapNut("M8-1.25"), "Close", 10, captive_nut=True),
        "bdw_fastener_clearance_hole_captive_square": lambda: ClearanceHole(SquareNut("M6-1"), depth=10, captive_nut=True),
        "bdw_fastener_clearance_hole_shoulder": lambda: ClearanceHole(ShoulderScrew("M6-1", 16), depth=30),
        "bdw_fastener_clearance_hole_rotated": lambda: ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), depth=5, rotation=(0, 90, 0)),
        "bdw_fastener_clearance_hole_washer": lambda: ClearanceHole(PlainWasher("M6", "iso7089"), depth=5),
        "bdw_fastener_clearance_hole_attrs": lambda: V(ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), depth=5).hole_depth),
        "bdw_fastener_clearance_hole_no_depth": lambda: error_of(lambda: ClearanceHole(SocketHeadCapScrew("M3-0.5", 8))),
        "bdw_fastener_clearance_hole_heatset": lambda: error_of(lambda: ClearanceHole(HeatSetNut("M3-0.5-Short"), depth=5)),
        "bdw_fastener_clearance_hole_captive_bad": lambda: error_of(
            lambda: ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), depth=5, captive_nut=True)
        ),
        "bdw_fastener_clearance_hole_bad_fit": lambda: error_of(lambda: ClearanceHole(SocketHeadCapScrew("M3-0.5", 8), "Snug", 5)),
        "bdw_fastener_tap_hole": lambda: TapHole(SocketHeadCapScrew("M6-1", 20), depth=15),
        "bdw_fastener_tap_hole_hard": lambda: TapHole(HexHeadScrew("M10-1.5", 40), "Hard", "Close", 20, counter_sunk=False),
        "bdw_fastener_tap_hole_countersunk": lambda: TapHole(CounterSunkScrew("M4-0.7", 12), depth=10),
        "bdw_fastener_tap_hole_nut": lambda: TapHole(HexNut("M8-1.25"), depth=10),
        "bdw_fastener_tap_hole_bad_material": lambda: error_of(lambda: TapHole(HexNut("M8-1.25"), "Wood", depth=10)),
        "bdw_fastener_tap_hole_washer": lambda: error_of(lambda: TapHole(PlainWasher("M8", "iso7089"), depth=10)),
        "bdw_fastener_threaded_hole": lambda: ThreadedHole(SocketHeadCapScrew("M6-1", 20), depth=15),
        "bdw_fastener_threaded_hole_threaded": lambda: ThreadedHole(SocketHeadCapScrew("M4-0.7", 10), depth=8, simple=False),
        "bdw_fastener_threaded_hole_flat": lambda: ThreadedHole(HexHeadScrew("M8-1.25", 30), "Hard", depth=12, counter_sunk=False),
        "bdw_fastener_threaded_hole_shoulder": lambda: ThreadedHole(ShoulderScrew("M6-1", 16), depth=30),
        "bdw_fastener_threaded_hole_attrs": lambda: V(
            [
                (t := ThreadedHole(SocketHeadCapScrew("M4-0.7", 10), depth=8, simple=False)).hole_depth,
                t.thread.length,
                t.thread_locations,
                [[c.label, c.volume] for c in t.children],
                ThreadedHole(SocketHeadCapScrew("M4-0.7", 10), depth=8).thread is None,
            ]
        ),
        "bdw_fastener_threaded_hole_nut": lambda: error_of(lambda: ThreadedHole(HexNut("M8-1.25"), depth=10)),
        "bdw_fastener_threaded_hole_shallow": lambda: error_of(lambda: ThreadedHole(SocketHeadCapScrew("M6-1", 20), depth=4.0)),
        "bdw_fastener_threaded_hole_zero": lambda: error_of(lambda: ThreadedHole(SocketHeadCapScrew("M6-1", 20), depth=0)),
        "bdw_fastener_insert_hole": lambda: InsertHole(HeatSetNut("M3-0.5-Standard"), depth=10),
        "bdw_fastener_insert_hole_compensated": lambda: InsertHole(
            HeatSetNut("M4-0.7-8", "Hilitchi"), "Close", 12, manufacturing_compensation=0.2
        ),
        # errors
        "bdw_fastener_nut_bad_size_format": lambda: error_of(lambda: HexNut("M6")),
        "bdw_fastener_nut_bad_type": lambda: error_of(lambda: DomedCapNut("M6-1", "iso4032")),
        "bdw_fastener_nut_bad_hand": lambda: error_of(lambda: HexNut("M6-1", hand="up")),
        "bdw_fastener_nut_bad_size": lambda: error_of(lambda: SquareNut("M7-1")),
        "bdw_fastener_screw_bad_size_format": lambda: error_of(lambda: SocketHeadCapScrew("M6", 20)),
        "bdw_fastener_screw_bad_size": lambda: error_of(lambda: ButtonHeadScrew("M7-1", 20)),
        "bdw_fastener_screw_bad_type": lambda: error_of(lambda: ButtonHeadScrew("M6-1", 20, "iso4762")),
        "bdw_fastener_screw_too_short": lambda: error_of(lambda: CounterSunkScrew("M6-1", 3.0)),
        "bdw_fastener_shoulder_bad_length": lambda: error_of(lambda: ShoulderScrew("M6-1", 15.0)),
        "bdw_fastener_shoulder_bad_size": lambda: error_of(lambda: ShoulderScrew("M7-1", 15)),
        "bdw_fastener_washer_bad_size": lambda: error_of(lambda: PlainWasher("M7", "iso7089")),
        "bdw_fastener_washer_bad_type": lambda: error_of(lambda: ChamferedWasher("M6", "iso7089")),
        "bdw_fastener_no_tap_data": lambda: error_of(lambda: HexNut("M1.6-0.35").tap_hole_diameters),
    }
)
