"""Python twins of cases_bdw_o_rings.cpp: bd_warehouse's o_rings.py."""

from build123d import Align

from bd_warehouse.o_rings import ORing, read_parameters_from_csv

from bdw_util import error_of, loc_values, num
from values import V

SHAPE_SIZES = ["001", "025", "116", "214", "325", "381", "425", "475"]
WIDTHS = [1.02, 1.27, 1.52, 1.78, 2.62, 3.53, 5.33, 6.99]


def ring_values(r):
    return [
        r.label, r.info, r.o_ring_size, r.o_ring_type, r.dash_number, r.inner_diameter,
        r.inner_diameter_tolerance, r.width, r.width_tolerance, r.major_radius, r.minor_radius, r.length,
        r.gland_width, r.gland_width_tol, r.gland_depth, r.gland_depth_tol, r.gland_radius, r.gland_radius_tol,
        r.gland_profile is None, list(r.color), loc_values(r.joints["a"].relative_location),
    ]  # fmt: skip


def gland_values(r):
    return [r.gland_width, r.gland_width_tol, r.gland_depth, r.gland_depth_tol, r.gland_radius, r.gland_radius_tol]


def neighbors(result):
    return V([[k, list(v)] for k, v in result.items()])


def number_table(table):
    return V([[k, [[k2, v2] for k2, v2 in row.items()]] for k, row in table.items()])


def ring_with_static(size, axial, rings):
    r = ORing(size)
    profile = r.static_gland_profile(axial, rings)
    return r, profile


def ring_with_dynamic(size, rings):
    r = ORing(size)
    profile = r.dynamic_gland_profile(rings)
    return r, profile


CASES = {
    "bdw_o_rings_all_sizes": lambda: V([ring_values(ORing(s)) for s in ORing.sizes()]),
    "bdw_o_rings_types": lambda: V(sorted(ORing.types())),
    "bdw_o_rings_sizes": lambda: V(ORing.sizes()),
    "bdw_o_rings_parameters": lambda: V([[k, num(v)] for k, v in ORing.parameters("-25").items()]),
    "bdw_o_rings_o_ring_parameters": lambda: V(
        [[t, [[s, [[k, num(v)] for k, v in p.items()]] for s, p in table.items()]] for t, table in ORing.o_ring_parameters.items()]
    ),
    "bdw_o_rings_width_families": lambda: V(
        [[t, [[w, [list(e) for e in entries]] for w, entries in fam.items()]] for t, fam in ORing.o_ring_width_families.items()]
    ),
    "bdw_o_rings_o_ring_data": lambda: V([[k, [[k2, v2] for k2, v2 in row.items()]] for k, row in ORing.o_ring_data.items()]),
    "bdw_o_rings_static_gland_data": lambda: number_table(ORing.o_ring_static_gland_data),
    "bdw_o_rings_dynamic_gland_data": lambda: number_table(ORing.o_ring_dynamic_gland_data),
    "bdw_o_rings_read_parameters_from_csv": lambda: number_table(read_parameters_from_csv("o-ring_dynamic_gland_parameters.csv")),
    "bdw_o_rings_normalize_size": lambda: V([ORing._normalize_size(s) for s in ["1", "-25", " 025 ", "-7", "475", "000"]]),
    "bdw_o_rings_normalize_size_errors": lambda: V(
        [error_of(lambda s=s: ORing._normalize_size(s)).data for s in ["", "1234", "--5", "2a", "-", "+5", "1.5"]]
    ),
    "bdw_o_rings_select_by_size": lambda: V(
        [[[c.__name__, t] for c, t in ORing.select_by_size(s).items()] for s in ["25", "-001", "475", "999"]]
    ),
    "bdw_o_rings_select_by_inner_diameter": lambda: V(
        [neighbors(ORing.select_by_inner_diameter(d)).data for d in [650, 20, 1.07, 24.99, 100, 2.9, 500]]
    ),
    "bdw_o_rings_select_by_inner_diameter_tolerance": lambda: V(
        [neighbors(ORing.select_by_inner_diameter(d, t)).data for d, t in [(20, 0.0), (20, 0.2), (37.47, 0.0), (8.1, 0.01)]]
    ),
    "bdw_o_rings_select_by_length": lambda: V(
        [neighbors(ORing.select_by_length(l)).data for l in [101.42, 10, 300.0, 2000, 6.5]]
    ),
    "bdw_o_rings_select_by_length_tolerance": lambda: V(
        [neighbors(ORing.select_by_length(l, t)).data for l, t in [(101.42, 0.0), (101.42, 0.5), (55.0, 0.01)]]
    ),
    "bdw_o_rings_select_errors": lambda: V(
        [
            error_of(lambda: ORing.select_by_inner_diameter(0)).data,
            error_of(lambda: ORing.select_by_inner_diameter(-3.0)).data,
            error_of(lambda: ORing.select_by_length(10, -0.1)).data,
            error_of(lambda: ORing.select_by_length(10, 0.05, "as568")).data,
            error_of(lambda: ORing.sizes("as568")).data,
            error_of(lambda: ORing.nominal_widths("as568")).data,
            error_of(lambda: ORing.parameters("26", "as568")).data,
            error_of(lambda: ORing.parameters("999")).data,
            error_of(lambda: ORing.parameters("abc")).data,
        ]
    ),
    "bdw_o_rings_nominal_widths": lambda: V(ORing.nominal_widths()),
    "bdw_o_rings_gland_width_for": lambda: V(
        [ORing.gland_width_for(w, a, n) for w in WIDTHS for a in ["static", "dynamic"] for n in [0, 1, 2] if not (w < 1.75 and n > 0)]
    ),
    "bdw_o_rings_gland_width_for_errors": lambda: V(
        [
            error_of(lambda: ORing.gland_width_for(1.78, "rotary")).data,
            error_of(lambda: ORing.gland_width_for(1.78, "static", 3)).data,
            error_of(lambda: ORing.gland_width_for(0.0)).data,
            error_of(lambda: ORing.gland_width_for(-1.5)).data,
            error_of(lambda: ORing.gland_width_for(float("inf"))).data,
            error_of(lambda: ORing.gland_width_for(float("nan"))).data,
            error_of(lambda: ORing.gland_width_for(1.8)).data,
            error_of(lambda: ORing.gland_width_for(1.02, "static", 1)).data,
            error_of(lambda: ORing.gland_width_for(1.27, "dynamic", 2)).data,
        ]
    ),
    # O-rings
    "bdw_o_rings_rotated": lambda: ORing("214", rotation=(90, 0, 0), align=(Align.MIN, Align.CENTER, Align.MAX)),
    "bdw_o_rings_rotated_values": lambda: V([loc_values(ORing("214", rotation=(0, 45, 0)).location)]),
    "bdw_o_rings_bad_size": lambda: error_of(lambda: ORing("1000")),
    "bdw_o_rings_bad_code": lambda: error_of(lambda: ORing("999")),
    "bdw_o_rings_bad_type": lambda: error_of(lambda: ORing("025", "as568")),
    # gland errors
    "bdw_o_rings_gland_errors": lambda: V(
        [
            error_of(lambda: ORing("001").static_gland_profile(True, 1)).data,
            error_of(lambda: ORing("001").dynamic_gland_profile(2)).data,
            error_of(lambda: ORing("025").static_gland_profile(False, 5)).data,
            error_of(lambda: ORing("025").dynamic_gland_profile(-1)).data,
        ]
    ),
}

for size in SHAPE_SIZES:
    CASES[f"bdw_o_rings_ring_{size}"] = lambda s=size: ORing(s)

for size in ["010", "025", "116", "214", "325", "381", "425", "475"]:
    for axial in [True, False]:
        for rings in [0, 1, 2]:
            name = f"bdw_o_rings_static_gland_{size}_{'axial' if axial else 'radial'}_{rings}"
            CASES[name] = lambda s=size, a=axial, n=rings: ring_with_static(s, a, n)[1]
            CASES[name + "_values"] = lambda s=size, a=axial, n=rings: V(gland_values(ring_with_static(s, a, n)[0]))
    for rings in [0, 1, 2]:
        name = f"bdw_o_rings_dynamic_gland_{size}_{rings}"
        CASES[name] = lambda s=size, n=rings: ring_with_dynamic(s, n)[1]
        CASES[name + "_values"] = lambda s=size, n=rings: V(gland_values(ring_with_dynamic(s, n)[0]))
