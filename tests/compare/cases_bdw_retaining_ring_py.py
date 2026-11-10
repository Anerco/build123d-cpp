"""Python twins of cases_bdw_retaining_ring.cpp: bd_warehouse's retaining_ring.py."""

from build123d import Align

from bd_warehouse.retaining_ring import (
    ExternalSnapRing,
    InternalSnapRing,
    RetainingRing,
    RetainingRingGroove,
    read_retaining_ring_parameters_from_csv,
)

from bdw_util import error_of, loc_values
from values import V

GROOVE_SIZES = {
    "ExternalSnapRing": ["3", "9", "10", "40", "100", "300"],
    "InternalSnapRing": ["8", "11", "12", "30", "100", "300"],
}


def ring_values(r):
    return V(
        [
            r.label, r.info, r.ring_size, r.standard, [[k, v] for k, v in r.ring_dict.items()],
            list(r.color), loc_values(r.location), loc_values(r.joints["a"].relative_location),
            loc_values(r.joints["b"].relative_location),
        ]
    )  # fmt: skip


def groove_values(g):
    return V([g.label, g.groove_diameter, g.groove_width, g.clearance_diameter, g.ring.label])


def select_values(result):
    return [[c.__name__, t] for c, t in result.items()]


CASES = {}
for cls in [ExternalSnapRing, InternalSnapRing]:
    name = cls.__name__
    for size in cls.sizes():
        CASES[f"bdw_retaining_ring_{name}_{size}"] = lambda c=cls, s=size: c(s)
        CASES[f"bdw_retaining_ring_{name}_{size}_values"] = lambda c=cls, s=size: ring_values(c(s))
    for size in GROOVE_SIZES[name]:
        CASES[f"bdw_retaining_ring_{name}_{size}_groove"] = lambda c=cls, s=size: RetainingRingGroove(c(s))
        CASES[f"bdw_retaining_ring_{name}_{size}_groove_values"] = lambda c=cls, s=size: groove_values(
            RetainingRingGroove(c(s))
        )

CASES.update(
    {
        "bdw_retaining_ring_read_din471": lambda: V(
            [[k, [[k2, v2] for k2, v2 in row.items()]] for k, row in read_retaining_ring_parameters_from_csv("din471.csv").items()]
        ),
        "bdw_retaining_ring_read_din472": lambda: V(
            [[k, [[k2, v2] for k2, v2 in row.items()]] for k, row in read_retaining_ring_parameters_from_csv("din472.csv").items()]
        ),
        "bdw_retaining_ring_sizes": lambda: V([ExternalSnapRing.sizes(), InternalSnapRing.sizes()]),
        "bdw_retaining_ring_parameters": lambda: V(
            [[[k, v] for k, v in ExternalSnapRing.parameters(" 12 ").items()], [[k, v] for k, v in InternalSnapRing.parameters("47").items()]]
        ),
        "bdw_retaining_ring_select_by_size": lambda: V(
            [
                select_values(RetainingRing.select_by_size(s))
                for s in ["40", " 8 ", "3", "31", "500"]
            ]
            + [select_values(ExternalSnapRing.select_by_size(s)) for s in ["8", "31"]]
            + [select_values(InternalSnapRing.select_by_size(s)) for s in ["3", "31"]]
        ),
        "bdw_retaining_ring_size_stripped": lambda: V(ExternalSnapRing(" 40 ").label),
        "bdw_retaining_ring_rotated": lambda: ExternalSnapRing("40", rotation=(90, 0, 0), align=Align.MIN),
        "bdw_retaining_ring_rotated_values": lambda: ring_values(InternalSnapRing("30", rotation=(0, 30, 0), align=Align.MAX)),
        "bdw_retaining_ring_groove_rotated": lambda: RetainingRingGroove(
            InternalSnapRing("30"), rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)
        ),
        "bdw_retaining_ring_bad_external": lambda: error_of(lambda: ExternalSnapRing("2")),
        "bdw_retaining_ring_bad_internal": lambda: error_of(lambda: InternalSnapRing("3")),
        "bdw_retaining_ring_bad_parameters": lambda: error_of(lambda: InternalSnapRing.parameters("12.5")),
    }
)
