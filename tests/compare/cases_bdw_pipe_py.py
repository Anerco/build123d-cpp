"""Python twins of cases_bdw_pipe.cpp: bd_warehouse's pipe.py."""

from typing import get_args

from build123d import Align, Edge, JernArc, Line, Polyline, Wire

from bd_warehouse.pipe import Identifier, Material, Nps, Pipe, PipeSection

from bdw_util import error_of, loc_values
from values import V

MATERIALS = ["abs", "copper", "stainless"]


def valid_specs():
    """Every (nps, material, identifier) of the table that makes a section."""
    specs = []
    for nps in get_args(Nps):
        for material in MATERIALS:
            for identifier in get_args(Identifier):
                # (14 stainless 5S has a wall of 156 in in the table: its section fails, see below)
                if nps + material + identifier in PipeSection.pipe_data and nps + material + identifier != "14stainless5S":
                    specs.append((nps, material, identifier))
    return specs


def section_values(s):
    return [s.nps, s.sch, s.od, s.thickness, s.id, s.material is None, s.area, loc_values(s.location)]


def pipe_values(p):
    return V(
        [
            p.od, p.id, p.thickness, p.length, p.material.material.name, p.label,
            loc_values(p.joints["inlet"].relative_location), loc_values(p.joints["outlet"].relative_location),
        ]
    )  # fmt: skip


def bent_path():
    return Wire([Line((0, 0, 0), (0, 0, 50)), JernArc((0, 0, 50), (0, 0, 1), 40, 90), Line((40, 0, 90), (120, 0, 90))])


PATHS = {
    "line": lambda: Edge.make_line((0, 0, 0), (0, 0, 100)),
    "slanted": lambda: Line((10, 0, 0), (60, 30, 40)),
    "arc": lambda: JernArc((0, 0, 0), (1, 0, 0), 60, 120),
    "bent": bent_path,
    "polyline": lambda: Polyline((0, 0, 0), (100, 0, 0), (100, 80, 0)),
}

CASES = {
    "bdw_pipe_values": lambda: V([list(get_args(Nps)), list(get_args(Identifier)), list(get_args(Material))]),
    "bdw_pipe_data": lambda: V([[k, list(v)] for k, v in PipeSection.pipe_data.items()]),
    "bdw_pipe_all_sections": lambda: V([section_values(PipeSection(*spec)) for spec in valid_specs()]),
    "bdw_pipe_section_errors": lambda: V(
        [
            error_of(lambda: PipeSection("7", "copper", "K")).data,
            error_of(lambda: PipeSection("1", "copper", "Z")).data,
            error_of(lambda: PipeSection("1", "aluminum", "K")).data,
            error_of(lambda: PipeSection("1", "copper", "40")).data,
            error_of(lambda: PipeSection("1", "iron", "STD")).data,
            error_of(lambda: PipeSection("1", "pvc", "40")).data,
            error_of(lambda: PipeSection("1", "steel", "40")).data,
            error_of(lambda: PipeSection("1", "wood", "40")).data,
            error_of(lambda: Pipe("2", "steel", "80", Edge.make_line((0, 0, 0), (0, 0, 10)))).data,
        ]
    ),
    "bdw_pipe_section_bad_table_row": lambda: PipeSection("14", "stainless", "5S"),
    "bdw_pipe_section_align_min": lambda: PipeSection("2", "copper", "L", align=Align.MIN),
    "bdw_pipe_section_align_none": lambda: PipeSection("2", "copper", "L", align=None),
    "bdw_pipe_section_align_mixed": lambda: PipeSection("2", "copper", "L", align=(Align.MAX, Align.MIN)),
    "bdw_pipe_rotated": lambda: Pipe("1", "copper", "K", Edge.make_line((0, 0, 0), (0, 0, 100)), rotation=(90, 0, 0)),
    "bdw_pipe_aligned": lambda: Pipe("1", "copper", "K", bent_path(), align=Align.MIN),
    "bdw_pipe_aligned_values": lambda: pipe_values(
        Pipe("1", "copper", "K", bent_path(), rotation=(0, 0, 45), align=(Align.CENTER, Align.CENTER, Align.MIN))
    ),
}

for spec in [("1/8", "stainless", "10S"), ("1", "copper", "K"), ("4", "abs", "40"), ("24", "stainless", "10S"), ("1 1/2", "copper", "M")]:
    key = "_".join(x.replace(" ", "_").replace("/", "_") for x in spec)
    CASES[f"bdw_pipe_section_{key}"] = lambda s=spec: PipeSection(*s)

for path_name, path in PATHS.items():
    for spec in [("1", "copper", "K"), ("3", "stainless", "40S"), ("2", "abs", "40")]:
        key = path_name + "_" + "_".join(spec)
        CASES[f"bdw_pipe_{key}"] = lambda s=spec, p=path: Pipe(*s, p())
        CASES[f"bdw_pipe_{key}_values"] = lambda s=spec, p=path: pipe_values(Pipe(*s, p()))
