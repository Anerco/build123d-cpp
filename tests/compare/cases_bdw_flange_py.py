"""Python twins of cases_bdw_flange.cpp: bd_warehouse's flange.py."""

from typing import get_args

from build123d import Align, Plane, Rectangle

import bd_warehouse.flange as fl
from bd_warehouse.flange import (
    BlindFlange,
    FaceType,
    Flange,
    FlangeClass,
    LappedFlange,
    LappedFlangeStub,
    Nps,
    SlipOnFlange,
    SocketWeldFlange,
    WeldNeckFlange,
)

from bdw_util import error_of, loc_values, num
from values import V

CLASSES = {
    "BlindFlange": BlindFlange,
    "SlipOnFlange": SlipOnFlange,
    "SocketWeldFlange": SocketWeldFlange,
    "WeldNeckFlange": WeldNeckFlange,
}
SHAPE_FACES = ["Flat", "Raised", "Ring"]
ALL_FACES = list(get_args(FaceType)) + ["Bogus"]
STUB_SPECS = [("copper", "K"), ("stainless", "40S"), ("abs", "40")]
STUB_NPS = ["1", "2", "4", "8"]


def key(nps):
    return nps.replace(" ", "_").replace("/", "_")


def flange_values(f):
    out = [
        f.nps, getattr(f, "od", None), getattr(f, "thickness", None), getattr(f, "id", None),
        getattr(f, "flange_class", None), getattr(f, "face_type", None),
    ]  # fmt: skip
    out.append([[name, loc_values(j.relative_location)] for name, j in f.joints.items()])
    return out


def attempt(build):
    """The flange's values, or the error it raises."""
    made = []
    error = error_of(lambda: made.append(build()))
    if error.data is not None:
        return error.data
    return flange_values(made[0])


def table(t):
    return V([[k, [num(v) for v in row]] for k, row in t.items()])


def section_values(result):
    section, height = result
    if section is None:
        return [None, height]
    bb = section.bounding_box()
    return [section.area, len(section.faces()), [bb.min.X, bb.min.Y, bb.max.X, bb.max.Y], height]


CASES = {
    "bdw_flange_literals": lambda: V([list(get_args(Nps)), list(get_args(FaceType)), list(get_args(FlangeClass))]),
    "bdw_flange_nps_to_dn": lambda: V([[k, v] for k, v in fl.nps_to_dn.items()]),
    "bdw_flange_drilling_class_150": lambda: table(fl.drilling_class_150),
    "bdw_flange_drilling_class_300": lambda: table(fl.drilling_class_300),
    "bdw_flange_flange_data_class_150": lambda: table(fl.flange_data_class_150),
    "bdw_flange_flange_data_class_300": lambda: table(fl.flange_data_class_300),
    "bdw_flange_ring_joint_facings_class_150": lambda: table(fl.ring_joint_facings_class_150),
    "bdw_flange_ring_joint_facings_class_300": lambda: table(fl.ring_joint_facings_class_300),
    "bdw_flange_inputs_are_valid": lambda: V(
        [
            Flange.inputs_are_valid("2", 300, "Raised"),
            Flange.inputs_are_valid("24", 2500, None),
            error_of(lambda: Flange.inputs_are_valid("3 1/2", 150, "Raised")).data,
            error_of(lambda: Flange.inputs_are_valid("2", 125, "Raised")).data,
            error_of(lambda: Flange.inputs_are_valid("2", 150, "Smooth")).data,
        ]
    ),
    "bdw_flange_dimension_is_numeric": lambda: V(
        [Flange.dimension_is_numeric(v) for v in fl.flange_data_class_150["14"] + fl.drilling_class_150["1/2"]]
    ),
    "bdw_flange_require_numeric_dimensions": lambda: V(
        [
            error_of(lambda: Flange.require_numeric_dimensions(fl.flange_data_class_150["2"], {"a": 0, "b": 14}, "X")).data,
            error_of(
                lambda: Flange.require_numeric_dimensions(fl.flange_data_class_150["20"], {"a": 11, "b": 13, "c": 1}, "Y")
            ).data,
            error_of(lambda: Flange.require_numeric_dimensions(fl.flange_data_class_300["1"], {"a": 0, "b": 14}, "Z")).data,
        ]
    ),
    "bdw_flange_get_flange_data": lambda: V(
        [
            [[num(v) for v in part] for part in Flange.get_flange_data(nps, c, None)]
            for nps in get_args(Nps)
            for c in [150, 300]
        ]
        + [error_of(lambda: Flange.get_flange_data("2", 600, "Raised")).data, error_of(lambda: Flange.get_flange_data("7", 150, None)).data]
    ),
    "bdw_flange_get_face_section_data": lambda: V(
        [
            error_of(lambda n=nps, c=c, f=f: Flange.get_face_section_data(n, c, f)).data
            or section_values(Flange.get_face_section_data(nps, c, f))
            for nps in get_args(Nps)
            for c in [150, 300]
            for f in [None, "Flat", "Raised", "Ring", "Groove"]
        ]
    ),
    "bdw_flange_get_face_section_data_thickness": lambda: V(
        [
            error_of(lambda f=f, t=t: Flange.get_face_section_data("4", 150, f, t)).data
            or section_values(Flange.get_face_section_data("4", 150, f, t))
            for f in [None, "Flat", "Raised", "Ring", "Groove", "Tongue", "Male", "Female"]
            for t in [3.0, 7.0]
        ]
        + [error_of(lambda: Flange.get_face_section_data("4", 400, "Raised")).data]
    ),
    "bdw_flange_ring_face_section": lambda: Flange.get_face_section_data("4", 300, "Ring")[0],
    "bdw_flange_socket_weld_face_section_data": lambda: V(
        [
            error_of(lambda n=nps, c=c: SocketWeldFlange.get_face_section_data(n, c, "Raised")).data
            or section_values(SocketWeldFlange.get_face_section_data(nps, c, "Raised"))
            for nps in get_args(Nps)
            for c in [150, 300]
        ]
    ),
    "bdw_flange_weld_neck_face_section_data": lambda: V(
        [
            error_of(lambda n=nps, c=c: WeldNeckFlange.get_face_section_data(n, c, "Ring")).data
            or section_values(WeldNeckFlange.get_face_section_data(nps, c, "Ring"))
            for nps in get_args(Nps)
            for c in [150, 300]
        ]
    ),
    "bdw_flange_base": lambda: Flange(Plane.XZ * Rectangle(200, 20, align=(Align.MIN, Align.MIN)), 150, 6, 12),
    "bdw_flange_base_rotated": lambda: Flange(
        Plane.XZ * Rectangle(100, 15, align=(Align.MIN, Align.MIN)), 80, 4, 8, rotation=(90, 0, 0), align=Align.MAX
    ),
    "bdw_flange_blind_rotated": lambda: BlindFlange("4", 150, rotation=(0, 90, 0), align=None),
    "bdw_flange_weld_neck_aligned": lambda: WeldNeckFlange("6", 300, "Ring", align=(Align.MIN, Align.MAX, Align.CENTER)),
    "bdw_flange_lapped_values": lambda: V(
        [attempt(lambda n=nps, c=c: LappedFlange(n, c)) for nps in get_args(Nps) for c in [150, 300]]
        + [error_of(lambda: LappedFlange("2", 600)).data, error_of(lambda: LappedFlange("2 1/4", 150)).data]
    ),
    "bdw_flange_stub_values": lambda: V(
        [
            attempt(lambda n=nps, m=m, i=i, f=f: LappedFlangeStub(n, m, i, 50, f))
            for nps in STUB_NPS
            for m, i in STUB_SPECS
            for f in ["Flat", "Raised", "Ring", "Groove", "Male"]
        ]
        + [error_of(lambda: LappedFlangeStub("2", "steel", "40", 50)).data]
    ),
    "bdw_flange_stub_edges": lambda: V(
        [[e.length, str(e.geom_type)] for e in LappedFlangeStub("2", "copper", "K", 60).stub_edges]
    ),
    "bdw_flange_stub_weld_radius": lambda: V(LappedFlangeStub.weld_radius),
    "bdw_flange_stub_rotated": lambda: LappedFlangeStub("4", "stainless", "40S", 80, "Ring", rotation=(0, 0, 45), align=None),
}

for cls_name, cls in CLASSES.items():
    CASES[f"bdw_flange_{cls_name}_values"] = lambda c=cls: V(
        [attempt(lambda n=nps, k=k, f=f: c(n, k, f)) for nps in get_args(Nps) for k in [150, 300] for f in ALL_FACES]
    )
    for nps in get_args(Nps):
        for flange_class in [150, 300]:
            for face in SHAPE_FACES:
                CASES[f"bdw_flange_{cls_name}_{key(nps)}_{flange_class}_{face}"] = (
                    lambda c=cls, n=nps, k=flange_class, f=face: c(n, k, f)
                )

for nps in get_args(Nps):
    for flange_class in [150, 300]:
        CASES[f"bdw_flange_LappedFlange_{key(nps)}_{flange_class}"] = lambda n=nps, k=flange_class: LappedFlange(n, k)

for nps in STUB_NPS:
    for material, identifier in STUB_SPECS:
        for face in ["Raised", "Ring", "Groove"]:
            CASES[f"bdw_flange_LappedFlangeStub_{nps}_{material}_{identifier}_{face}"] = (
                lambda n=nps, m=material, i=identifier, f=face: LappedFlangeStub(n, m, i, 50, f)
            )
