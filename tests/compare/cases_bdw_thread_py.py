"""Python twins of cases_bdw_thread.cpp: bd_warehouse's thread.py."""

from build123d import IN, Align, Pos, Rotation

from bd_warehouse.thread import (
    AcmeThread,
    BSPPThread,
    IsoThread,
    MetricTrapezoidalThread,
    PlasticBottleThread,
    Thread,
    TrapezoidalThread,
    WhitworthThread,
    imperial_str_to_float,
    is_safe,
)

from bdw_util import attrs, children_values, error_of, num
from values import V


def thread_attrs(t):
    return attrs(
        t,
        [
            "external", "apex_radius", "apex_width", "root_radius", "root_width", "pitch", "length",
            "apex_offset", "interference", "right_hand", "end_finishes", "tooth_height", "taper",
            "simple", "manufacturing_compensation", "thread_loops",
        ],
    )  # fmt: skip


def thread_profile_values(t):
    p = t.thread_profile
    bb = p.bounding_box()
    return V([p.area, len(p.edges()), [bb.min.X, bb.min.Y, bb.max.X, bb.max.Y]])


def iso_attrs(t):
    names = [
        "major_diameter", "pitch", "length", "external", "thread_angle", "hand", "end_finishes",
        "interference", "simple", "manufacturing_compensation", "apex_radius", "root_radius",
    ]  # fmt: skip
    return V(attrs(t, names).data + [["h_parameter", t.h_parameter], ["min_radius", t.min_radius]])


def whitworth_attrs(t):
    return attrs(
        t,
        [
            "major_diameter", "pitch", "length", "external", "hand", "end_finishes", "interference",
            "simple", "manufacturing_compensation", "crest_truncation", "fundamental_triangle_height",
            "thread_height", "rounding_radius", "minor_diameter", "apex_radius", "root_radius",
            "crest_diameter", "tooth_height", "thread_loops", "thread_angle",
        ],
    )  # fmt: skip


def bspp_attrs(t):
    return V(
        whitworth_attrs(t).data
        + attrs(
            t,
            [
                "thread_size", "tpi", "tolerance_class", "standard", "basic_major_diameter",
                "basic_pitch_diameter", "basic_minor_diameter", "designation", "pitch_diameter_limits",
                "major_diameter_limits", "minor_diameter_limits",
            ],
        ).data
    )  # fmt: skip


def trapezoidal_attrs(t):
    return attrs(
        t,
        [
            "thread_size", "external", "length", "diameter", "thread_angle", "pitch", "lead",
            "apex_radius", "root_radius", "hand", "end_finishes", "interference",
            "manufacturing_compensation",
        ],
    )  # fmt: skip


def bottle_attrs(t):
    names = [
        "thread_size", "external", "bottle_type", "hand", "interference", "manufacturing_compensation",
        "style", "diameter", "finish", "major_diameter", "minor_diameter", "tpi", "root_width",
        "apex_width", "apex_offset", "apex_radius", "root_radius", "pitch", "starts", "lead", "length",
    ]  # fmt: skip
    out = attrs(t, names).data
    for name in ["bottle_major_diameter", "bottle_minor_diameter", "closure_minor_diameter"]:
        out.append([name, getattr(t, name, None)])
    return V(out)


def match_values(matches):
    return V(
        [
            [m.size, m.bottle_type, list(m.major_diameter), list(m.neck_diameter), m.pitch, m.min_turns, m.error]
            for m in matches
        ]
    )


CASES = {
    # helpers
    "bdw_thread_is_safe": lambda: V(
        [is_safe(s) for s in ["1/4", "1 1/2", "12.5", "", "M6", "12345678901", "1234567890", "3/8-16", ".5"]]
    ),
    "bdw_thread_imperial_str_to_float": lambda: V(
        [num(imperial_str_to_float(s)) for s in ["1/4", "1 1/2", "3", ".5", "M6", "#10", "1  1/2", "1//2", " 3/4 "]]
    ),
    "bdw_thread_imperial_zero_division": lambda: error_of(lambda: imperial_str_to_float("1/0")),
    "bdw_thread_imperial_syntax": lambda: V(num(imperial_str_to_float("1..2"))),
    # Thread
    "bdw_thread_thread_default": lambda: Thread(5, 0.5, 4, 1.5, 2, 4),
    "bdw_thread_thread_default_attrs": lambda: thread_attrs(Thread(5, 0.5, 4, 1.5, 2, 4)),
    "bdw_thread_thread_default_children": lambda: V(children_values(Thread(5, 0.5, 4, 1.5, 2, 4))),
    "bdw_thread_thread_profile": lambda: thread_profile_values(Thread(5, 0.5, 4, 1.5, 2, 4)),
    "bdw_thread_thread_profile_left": lambda: thread_profile_values(
        Thread(5, 0.5, 4, 1.5, 2, 4, apex_offset=0.3, hand="left", simple=True)
    ),
    "bdw_thread_thread_no_interference": lambda: Thread(5, 0.5, 4, 1.5, 2, 4, interference=0),
    "bdw_thread_thread_left": lambda: Thread(5, 0.5, 4, 1.5, 2, 5, hand="left", end_finishes=("fade", "fade")),
    "bdw_thread_thread_internal": lambda: Thread(4, 0.5, 5, 1.5, 2, 4, end_finishes=("square", "chamfer")),
    "bdw_thread_thread_offset": lambda: Thread(5, 0.4, 4, 1.6, 2, 3, apex_offset=0.3, end_finishes=("chamfer", "square")),
    "bdw_thread_thread_rotated": lambda: Thread(5, 0.5, 4, 1.5, 2, 4, rotation=(90, 0, 0), align=Align.MAX),
    "bdw_thread_thread_compensation": lambda: Thread(5, 0.5, 4, 1.5, 2, 4, manufacturing_compensation=0.2),
    "bdw_thread_thread_short_fade": lambda: Thread(5, 0.5, 4, 1.5, 2, 1, end_finishes=("fade", "fade")),
    "bdw_thread_thread_simple": lambda: thread_attrs(Thread(5, 0.5, 4, 1.5, 2, 4, simple=True)),
    "bdw_thread_thread_bad_finish": lambda: error_of(lambda: Thread(5, 0.5, 4, 1.5, 2, 4, end_finishes=("raw", "x"))),
    "bdw_thread_thread_taper": lambda: error_of(lambda: Thread(5, 0.5, 4, 1.5, 2, 4, taper_angle=5)),
    # IsoThread
    "bdw_thread_iso_m6": lambda: IsoThread(6, 1, 10),
    "bdw_thread_iso_m6_attrs": lambda: iso_attrs(IsoThread(6, 1, 10)),
    "bdw_thread_iso_m10_internal": lambda: IsoThread(10, 1.5, 8, external=False, end_finishes=("chamfer", "fade")),
    "bdw_thread_iso_m8_left": lambda: IsoThread(8, 1.25, 6, hand="left"),
    "bdw_thread_iso_raw": lambda: IsoThread(5, 0.8, 4, end_finishes=("raw", "raw")),
    "bdw_thread_iso_square": lambda: IsoThread(5, 0.8, 4, end_finishes=("square", "square")),
    "bdw_thread_iso_chamfer": lambda: IsoThread(12, 1.75, 7, end_finishes=("chamfer", "chamfer")),
    "bdw_thread_iso_fade": lambda: IsoThread(4, 0.7, 5, end_finishes=("fade", "fade")),
    "bdw_thread_iso_internal_square": lambda: IsoThread(6, 1, 5, external=False, end_finishes=("square", "square")),
    "bdw_thread_iso_rotated": lambda: IsoThread(6, 1, 5, rotation=(90, 0, 0), align=(Align.MIN, Align.CENTER, Align.MAX)),
    "bdw_thread_iso_no_interference": lambda: IsoThread(6, 1, 5, interference=0),
    "bdw_thread_iso_compensation": lambda: IsoThread(6, 1, 5, external=False, manufacturing_compensation=0.15),
    "bdw_thread_iso_simple": lambda: iso_attrs(IsoThread(20, 2.5, 30, simple=True)),
    "bdw_thread_iso_imperial": lambda: IsoThread(0.25 * IN, IN / 20, 0.5 * IN),
    "bdw_thread_iso_bad_hand": lambda: error_of(lambda: IsoThread(6, 1, 5, hand="up")),
    "bdw_thread_iso_bad_finish": lambda: error_of(lambda: IsoThread(6, 1, 5, end_finishes=("fade", "taper"))),
    # WhitworthThread, BSPPThread
    "bdw_thread_whitworth": lambda: WhitworthThread(20, 2, 6),
    "bdw_thread_whitworth_attrs": lambda: whitworth_attrs(WhitworthThread(20, 2, 6)),
    "bdw_thread_whitworth_internal": lambda: WhitworthThread(20, 2, 6, external=False, end_finishes=("fade", "fade")),
    "bdw_thread_whitworth_truncated": lambda: WhitworthThread(20, 2, 6, crest_truncation=0.3, end_finishes=("square", "chamfer")),
    "bdw_thread_whitworth_truncated_internal": lambda: WhitworthThread(
        20, 2, 6, external=False, crest_truncation=0.25, hand="left", end_finishes=("chamfer", "square")
    ),
    "bdw_thread_whitworth_profile": lambda: thread_profile_values(WhitworthThread(20, 2, 6, simple=True)),
    "bdw_thread_whitworth_profile_truncated": lambda: thread_profile_values(
        WhitworthThread(20, 2, 6, crest_truncation=0.4, external=False, interference=0, simple=True)
    ),
    "bdw_thread_whitworth_bad_truncation": lambda: error_of(lambda: WhitworthThread(20, 2, 6, crest_truncation=1.5)),
    "bdw_thread_whitworth_bad_hand": lambda: error_of(lambda: WhitworthThread(20, 2, 6, hand="LEFT")),
    "bdw_thread_bspp_sizes": lambda: V(BSPPThread.sizes()),
    "bdw_thread_bspp_normalize": lambda: V(
        [BSPPThread._normalize_size(s) for s in ["G1/2", "g 1/2", ' 1 1/4" ', "G  2   1/2", "1\u2033"]]
    ),
    "bdw_thread_bspp_half": lambda: BSPPThread("G1/2", 10),
    "bdw_thread_bspp_half_attrs": lambda: bspp_attrs(BSPPThread("G1/2", 10, simple=True)),
    "bdw_thread_bspp_internal": lambda: BSPPThread("1/4", 8, external=False, end_finishes=("chamfer", "fade")),
    "bdw_thread_bspp_internal_attrs": lambda: bspp_attrs(BSPPThread("1/4", 8, external=False, simple=True)),
    "bdw_thread_bspp_class_b": lambda: bspp_attrs(BSPPThread("G 1", 12, tolerance_class="b", hand="left", simple=True)),
    "bdw_thread_bspp_crest": lambda: BSPPThread("G1/8", 5, crest_diameter=9.6),
    "bdw_thread_bspp_crest_internal": lambda: bspp_attrs(
        BSPPThread("G3/8", 8, external=False, crest_diameter=15.0, simple=True)
    ),
    "bdw_thread_bspp_bad_size": lambda: error_of(lambda: BSPPThread("G7", 10)),
    "bdw_thread_bspp_bad_class": lambda: error_of(lambda: BSPPThread("G1/2", 10, tolerance_class="C")),
    "bdw_thread_bspp_internal_class": lambda: error_of(lambda: BSPPThread("G1/2", 10, external=False, tolerance_class="A")),
    "bdw_thread_bspp_bad_crest": lambda: error_of(lambda: BSPPThread("G1/2", 10, crest_diameter=30)),
    # TrapezoidalThread, AcmeThread, MetricTrapezoidalThread
    "bdw_thread_trapezoidal": lambda: TrapezoidalThread(12, 2, 30, 8),
    "bdw_thread_trapezoidal_attrs": lambda: trapezoidal_attrs(TrapezoidalThread(12, 2, 30, 8)),
    "bdw_thread_trapezoidal_starts": lambda: TrapezoidalThread(12, 2, 30, 8, starts=2, end_finishes=("square", "square")),
    "bdw_thread_trapezoidal_starts_children": lambda: V(
        children_values(TrapezoidalThread(10, 2, 30, 6, starts=3, end_finishes=("raw", "raw")))
    ),
    "bdw_thread_trapezoidal_internal": lambda: TrapezoidalThread(16, 4, 30, 10, external=False, hand="left", rotation=(0, 90, 0)),
    "bdw_thread_trapezoidal_bad_hand": lambda: error_of(lambda: TrapezoidalThread(12, 2, 30, 8, hand="x")),
    "bdw_thread_acme_sizes": lambda: V(AcmeThread.sizes()),
    "bdw_thread_acme_pitch": lambda: V([[k, v] for k, v in AcmeThread.acme_pitch.items()]),
    "bdw_thread_acme": lambda: AcmeThread("1/2", 20),
    "bdw_thread_acme_attrs": lambda: trapezoidal_attrs(AcmeThread("1 1/4", 10, external=False)),
    "bdw_thread_acme_internal": lambda: AcmeThread("3/8", 10, external=False, end_finishes=("chamfer", "square")),
    "bdw_thread_acme_bad_size": lambda: error_of(lambda: AcmeThread("5/32", 10)),
    "bdw_thread_acme_bad_text": lambda: error_of(lambda: AcmeThread("M8", 10)),
    "bdw_thread_metric_sizes": lambda: V(MetricTrapezoidalThread.sizes()),
    "bdw_thread_metric": lambda: MetricTrapezoidalThread("8x1.5", 15),
    "bdw_thread_metric_attrs": lambda: trapezoidal_attrs(MetricTrapezoidalThread("20x4", 15, hand="left")),
    "bdw_thread_metric_internal": lambda: MetricTrapezoidalThread("14x3", 9, external=False, end_finishes=("raw", "fade")),
    "bdw_thread_metric_bad_size": lambda: error_of(lambda: MetricTrapezoidalThread("8x2", 10)),
    # PlasticBottleThread
    "bdw_thread_bottle_types": lambda: V(list(PlasticBottleThread.bottle_types())),
    "bdw_thread_bottle_sizes": lambda: V(PlasticBottleThread.sizes()),
    "bdw_thread_bottle_sizes_pco": lambda: V(PlasticBottleThread.sizes("pco1881")),
    "bdw_thread_bottle_sizes_bad": lambda: error_of(lambda: PlasticBottleThread.sizes("x")),
    "bdw_thread_bottle_identify": lambda: match_values(PlasticBottleThread.identify(neck_diameter=30, pitch=4, turns=1.5)),
    "bdw_thread_bottle_identify_major": lambda: match_values(
        PlasticBottleThread.identify(major_diameter=38.2, tpi=6, style="L", tolerance=0.3)
    ),
    "bdw_thread_bottle_identify_pco": lambda: match_values(PlasticBottleThread.identify(major_diameter=27.4, pitch=2.7)),
    "bdw_thread_bottle_identify_turns": lambda: match_values(PlasticBottleThread.identify(neck_diameter=36, turns=1.0)),
    "bdw_thread_bottle_identify_errors": lambda: V(
        [
            error_of(lambda: PlasticBottleThread.identify()).data,
            error_of(lambda: PlasticBottleThread.identify(30, pitch=4, tpi=6)).data,
            error_of(lambda: PlasticBottleThread.identify(30, style="X")).data,
        ]
    ),
    "bdw_thread_bottle_m38": lambda: PlasticBottleThread("M38SP444"),
    "bdw_thread_bottle_m38_attrs": lambda: bottle_attrs(PlasticBottleThread("M38SP444", external=False, manufacturing_compensation=0.2)),
    "bdw_thread_bottle_m38_internal": lambda: PlasticBottleThread("M38SP444", external=False, manufacturing_compensation=0.2),
    "bdw_thread_bottle_l28": lambda: PlasticBottleThread("L28SP400", hand="left", interference=0.1),
    "bdw_thread_bottle_pco": lambda: PlasticBottleThread("28", bottle_type="pco1881"),
    "bdw_thread_bottle_pco_attrs": lambda: bottle_attrs(PlasticBottleThread("28mm", external=False, bottle_type="pco1881")),
    "bdw_thread_bottle_pco_internal": lambda: PlasticBottleThread("28MM", external=False, bottle_type="pco1881"),
    "bdw_thread_bottle_bad_size": lambda: error_of(lambda: PlasticBottleThread("X38SP444")),
    "bdw_thread_bottle_bad_finish": lambda: error_of(lambda: PlasticBottleThread("M38SP999")),
    "bdw_thread_bottle_bad_diameter": lambda: error_of(lambda: PlasticBottleThread("M37SP400")),
    "bdw_thread_bottle_bad_type": lambda: error_of(lambda: PlasticBottleThread("28", bottle_type="din")),
    "bdw_thread_bottle_bad_pco": lambda: error_of(lambda: PlasticBottleThread("38", bottle_type="pco1881")),
}
