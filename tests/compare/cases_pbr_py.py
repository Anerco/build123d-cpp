"""Python side of cases_pbr.cpp: PBR materials with bd_materials and threejs_materials."""

import os

import numpy as np
from PIL import Image

import threejs_materials
from values import V

ASSETS = os.path.join(os.path.dirname(threejs_materials.__file__), "pbr_properties", "_assets")


def pixel_fingerprint(img):
    a = np.asarray(img, dtype=np.int64).ravel()
    i = np.arange(a.size, dtype=np.int64)
    s1 = int(a.sum())
    s2 = int((a * (i % 65521 + 1)).sum())
    s3 = int((a * ((i * 7919) % 104729)).sum())
    return [img.mode, img.size[0], img.size[1], f"{s1}/{s2}/{s3}"]


def open_texture(path):
    """threejs_materials.utils._open_texture_image"""
    img = Image.open(path)
    if img.mode in ("1", "P"):
        img = img.convert("L") if len(img.getbands()) == 1 else img.convert("RGB")
    return img


def texture_files():
    out = []
    for d in os.listdir(ASSETS):
        if os.path.isdir(os.path.join(ASSETS, d)):
            out += [d + "/" + f for f in os.listdir(os.path.join(ASSETS, d)) if f.endswith(".png")]
    return sorted(out)


def pil_textures():
    out = []
    for name in texture_files():
        raw = Image.open(os.path.join(ASSETS, name))
        img = open_texture(os.path.join(ASSETS, name))
        px = img.convert("RGB").resize((1, 1), Image.Resampling.LANCZOS).getpixel((0, 0))
        out.append([name, raw.mode, pixel_fingerprint(img), list(px[:3])])
    return V(out)


def pil_resize():
    out = []
    l_img = open_texture(os.path.join(ASSETS, "birch/roughness.png"))
    rgb = open_texture(os.path.join(ASSETS, "walnut/color.png"))
    p = open_texture(os.path.join(ASSETS, "carbon_fiber/color.png"))
    for size in [(1024, 1024), (333, 777), (2048, 100), (7, 3000), (1, 1)]:
        out.append(pixel_fingerprint(l_img.resize(size, Image.Resampling.LANCZOS)))
        out.append(pixel_fingerprint(rgb.resize(size, Image.Resampling.LANCZOS)))
        out.append(pixel_fingerprint(p.resize(size, Image.Resampling.LANCZOS)))
    out.append(pixel_fingerprint(rgb.convert("L")))
    rgba = rgb.copy()
    rgba.putalpha(l_img.resize(rgb.size, Image.Resampling.LANCZOS))
    out.append(pixel_fingerprint(rgba))
    out.append(pixel_fingerprint(l_img.convert("RGB").getchannel(1)))
    out.append(pixel_fingerprint(rgb.convert("RGBA")))
    return V(out)


from pathlib import Path
import warnings

from threejs_materials import coats, glass, metal, paper, plastic, textile, wood  # noqa: E402

CATEGORIES = {"coats": coats, "glass": glass, "metal": metal, "paper": paper, "plastic": plastic,
              "textile": textile, "wood": wood}


def pbr_summary(p, with_color=True):
    r = repr(p)
    d = None
    if p.maps_dir is not None:
        d = Path(p.maps_dir).name
        r = r.replace(str(p.maps_dir), "<maps_dir>/" + d)
    out = {
        "id": p.id, "name": p.name, "source": p.source, "url": p.url, "license": p.license,
        "values": p.values.to_dict(), "maps": p.maps.to_dict(),
        "texture_repeat": list(p.texture_repeat) if p.texture_repeat is not None else None,
        "texture_rotation": p.texture_rotation, "normalize_uvs": p.normalize_uvs, "maps_dir": d, "repr": r,
    }
    if with_color:
        out["color"] = list(p.interpolate_color())
    return out


def outcome(f):
    try:
        return f()
    except (ValueError, TypeError, AttributeError, ZeroDivisionError, KeyError, IndexError):
        return "error"


def pbr_presets():
    out = []
    for category, mod in CATEGORIES.items():
        for name in mod.__all__:
            out.append([f"{category}.{name}", pbr_summary(getattr(mod, name)())])
    return V(out)


def pbr_preset_variants():
    out = []

    def add(category, name, **kwargs):
        out.append(outcome(lambda: pbr_summary(getattr(CATEGORIES[category], name)(**kwargs))))

    add("plastic", "plastic", color="red")
    add("plastic", "plastic", color="#336699", roughness=0.5)
    add("plastic", "acrylic", color=(0.2, 0.4, 0.6), roughness=0.1, thickness=3.0)
    add("glass", "glass", color="#ffeedd80", thickness=2.0)
    add("wood", "oak", color="orange", scale=(2.0, 3.0), rotation=30.0)
    add("wood", "walnut", roughness=0.4)
    add("metal", "aluminum_brushed", roughness=0.2, scale=(0.5, 0.5))
    add("metal", "steel_matte", rotation=90.0)
    add("plastic", "plastic_fdm", color="blue", scale=(6.4, 6.4), rotation=0.0)
    add("plastic", "plastic_fdm", scale=(3.2, 3.2))
    add("plastic", "plastic_fdm_skin", rotation=45.0)
    add("plastic", "plastic_fdm_skin", rotation=0.0)
    add("textile", "leather", color="saddlebrown")
    add("paper", "paper", color=(1.0, 0.9, 0.8, 0.5))
    add("coats", "chrome", roughness=0.25)
    add("coats", "coat_gloss", color="rgb(10%, 20%, 30%)")
    add("metal", "aluminum", color="red")
    add("metal", "titanium", thickness=1.0)
    add("metal", "unobtainium")
    add("plastic", "plastic", color=0xFF0000)
    add("plastic", "plastic", color="nocolor")
    add("plastic", "plastic", color=(1.0, 0.0))
    return V(out)


def pbr_overrides():
    out = []
    base = metal.aluminum()

    def add(**kwargs):
        out.append(outcome(lambda: pbr_summary(base.override(**kwargs), False)))

    add(color="#ff000080")
    add(color="#ff000080", opacity=0.9)
    add(color="hsl(120, 50%, 50%)", roughness=0.3, metalness=0.5)
    add(color="HSV(200, 40%, 80%)")
    add(color="#abc")
    add(color="#abcd")
    add(color="rgba(1, 2, 3, 4)")
    add(ior=1.45, transmission=0.5, transparent=True, alpha_test=0.25)
    add(clearcoat=1.0, clearcoat_roughness=0.1, sheen=0.5, sheen_color="red", sheen_roughness=0.3)
    add(anisotropy=0.5, anisotropy_rotation=1.2, specular_intensity=0.7, specular_color=(0.1, 0.2, 0.3))
    add(emissive="#808080", emissive_intensity=2.0, attenuation_color="teal", attenuation_distance=5.0)
    add(thickness=1.5, iridescence=0.8, iridescence_ior=1.3, iridescence_thickness_range=(100.0, 400.0),
        dispersion=0.2, normal_scale=(0.5, 0.5), displacement_scale=0.1, side=2)
    add()
    add(color=16711680)
    add(color=("red", 0.5))
    add(color="#ff0000zz")
    out.append(pbr_summary(base.override(roughness=0.3).override(color="gold").scale(2.0, 2.0), False))
    out.append(pbr_summary(wood.oak().scale(0.5, 0.25, False, 15.0), False))
    out.append(pbr_summary(wood.oak().scale(1.0, 1.0, True, 0.0).strip_maps(), False))
    return V(out)


def pbr_interpolate_color():
    out = []
    for category, name in [("wood", "oak"), ("plastic", "carbon_fiber"), ("textile", "leather"),
                           ("metal", "brass"), ("glass", "glass")]:
        p = getattr(CATEGORIES[category], name)()
        for color in [None, "red", "#80808080", (0.9, 0.5, 0.1), (2.0, 2.0, 2.0)]:
            out.append(list(p.interpolate_color(color)))
    return V(out)



# ---- bd_materials ------------------------------------------------------------------------

import dataclasses  # noqa: E402

from build123d import Color  # noqa: E402
from bd_materials import FinishedMaterial, applicability, registry  # noqa: E402
from bd_materials import finishes as fin  # noqa: E402
from bd_materials import processes as proc  # noqa: E402
from bd_materials.core import (  # noqa: E402
    ALLOWED_CATEGORIES, COATABLE_POLY, EVERYTHING, FERROUS, MECHANICAL_PROPERTY_UNITS, METAL,
    NOT_SUITABLE, PAINTABLE, PROPERTY_UNITS, THERMAL_PROPERTY_UNITS, Range, with_density)
from bd_materials.materials import ALL_MATERIALS, metals, plastics, resins  # noqa: E402
from bd_materials.materials import glass as bglass  # noqa: E402
from bd_materials.materials import paper as bpaper  # noqa: E402
from bd_materials.materials import textile as btextile  # noqa: E402
from bd_materials.materials import wood as bwood  # noqa: E402
from bd_materials.pbr import get_pbr_properties  # noqa: E402
from bd_materials.processes import Process  # noqa: E402


def material_summary(m):
    props = []
    for f in dataclasses.fields(m):
        if f.name in ("name", "density", "family", "transparent", "hardness_scale"):
            continue
        v = getattr(m, f.name)
        props.append([f.name, None] if v is None else [f.name, v.min, v.max])
    return {"name": m.name, "kind": type(m).__name__, "category": m.category, "family": m.family,
            "transparent": m.transparent, "density": m.density,
            "fields": [f.name for f in dataclasses.fields(m)],
            "hardness_scale": getattr(m, "hardness_scale", None), "props": props, "str": str(m)}


def fm_summary(fm, with_color=True):
    return {"repr": repr(fm), "str": str(fm), "name": fm.name, "pbr": pbr_summary(fm.pbr, with_color)}


def material_catalog():
    out = []
    for name in registry.material_names():
        fm = registry.resolve(name)
        out.append([name, registry.canonical_name(fm), material_summary(fm.material), fm_summary(fm)])
    return V(out)


def material_variants():
    out = []

    def add(f):
        out.append(outcome(lambda: fm_summary(f())))

    A = metals.aluminum
    every_finish = [
        fin.bead_blast(), fin.brushed(), fin.fine_sanding(), fin.smooth_machining(), fin.electropolish(),
        fin.anodize("black"), fin.chem_film(), fin.conductive_oxidation(), fin.black_oxide(), fin.passivate(),
        fin.pickle(), fin.dye("red"), fin.chrome(), fin.gold_plate(), fin.nickel_plate(), fin.silver_plate(),
        fin.tin_plate(), fin.pvd(), fin.zinc_plate(), fin.vacuum_plating("blue"), fin.powder_coat("red"),
        fin.spray_paint("green"), fin.electrophoresis(), fin.laser_engrave(), fin.etch(), fin.silkscreen()]
    for f in every_finish:
        add(lambda f=f: A(finish=f))
    for mat in [metals.stainless, metals.mild_steel, metals.titanium, metals.brass, metals.copper,
                metals.magnesium, metals.gold, metals.bronze, metals.tool_steel]:
        for f in [fin.brushed(), fin.bead_blast(), fin.anodize("red"), fin.pvd("gunmetal"), fin.black_oxide(),
                  fin.electrophoresis("navy")]:
            add(lambda mat=mat, f=f: mat(finish=f))
    # finish geometry and lists
    add(lambda: A(finish=fin.brushed((2.0, 2.0), 30.0)))
    add(lambda: A(finish=fin.bead_blast((0.5, 0.5))))
    add(lambda: A(finish=fin.fine_sanding((1.0, 3.0))))
    add(lambda: A(finish=fin.brushed((1.0, 1.0), 0.0)))
    add(lambda: A(finish=[fin.brushed(), fin.anodize("red")]))
    add(lambda: A(finish=[fin.anodize("blue"), fin.bead_blast((2.0, 2.0))]))
    add(lambda: A(finish=[fin.powder_coat("red"), fin.spray_paint("blue", fin.Sheen.MATTE)]))
    add(lambda: A(finish=[]))
    add(lambda: A(finish=[fin.laser_engrave()]))
    # finish colors
    for c in ["rose gold", "natural", " Champagne ", "#336699", "#33669980", 0x336699, -1, ("red", 0.5),
              (0.1, 0.2, 0.3), (0.1, 0.2, 0.3, 0.4), Color(0.2, 0.4, 0.6, 0.5), "gunmetal", "nocolor", None]:
        add(lambda c=c: A(finish=fin.anodize(c)))
    # the other surfaces
    add(lambda: A(finish=fin.pvd("rose gold")))
    add(lambda: metals.stainless(finish=[fin.brushed(), fin.pvd("gold")]))
    add(lambda: metals.stainless(finish=[fin.bead_blast(), fin.black_oxide()]))
    add(lambda: plastics.nylon(finish=fin.dye("blue")))
    add(lambda: bwood.oak(finish=fin.dye("red")))
    add(lambda: A(finish=fin.dye(None)))
    add(lambda: A(finish=[fin.brushed(), fin.dye("red")]))
    add(lambda: metals.mild_steel(finish=fin.zinc_plate("yellow")))
    add(lambda: metals.mild_steel(finish=fin.zinc_plate()))
    add(lambda: plastics.abs_(finish=fin.vacuum_plating("silver", fin.Sheen.MATTE)))
    add(lambda: A(finish=fin.powder_coat("red", None)))
    add(lambda: A(finish=fin.electrophoresis(0)))
    add(lambda: A(finish=fin.electrophoresis("#000")))
    add(lambda: A(finish=fin.electrophoresis("black")))
    add(lambda: metals.stainless(finish=fin.electrophoresis("red")))
    add(lambda: A(finish=[fin.brushed(), fin.electrophoresis()]))
    add(lambda: metals.copper(finish=fin.silkscreen()))
    # processes
    P = plastics.pla
    for p in [proc.fdm(), proc.fdm(0.1), proc.fdm(0.2, 90.0), proc.fdm(0.3, 0.0, 2.0), proc.fdm(0.2, 0.0, 0.5),
              proc.fdm_skin(), proc.fdm_skin(0.6, 0.0), proc.fdm_skin(0.4, 45.0), proc.sls(), proc.sls((2.0, 2.0)),
              proc.mjf(), proc.slm(), proc.vat(), proc.molded(), proc.machined(), proc.cast(), proc.wrought()]:
        add(lambda p=p: P(color="orange", process=p))
    for p in [proc.fdm(), proc.slm((3.0, 3.0)), proc.cast(), proc.fdm_skin()]:
        add(lambda p=p: A(process=p))
        add(lambda p=p: bwood.oak(process=p))
    add(lambda: plastics.cfrp(process=proc.fdm()))
    add(lambda: plastics.pc(process=proc.fdm(), opacity=0.5))
    add(lambda: P(process=proc.fdm(0.0)))
    add(lambda: P(process=proc.fdm(0.2, 0.0, 0.0)))
    # colors
    for c in ["orange", "ORANGE", "#FF8800", "#ff880080", 0xFF8800, ("red", 0.5), (0.1, 0.2, 0.3),
              (0.1, 0.2, 0.3, 0.4), Color(1.0, 0.5, 0.0), "natural", "clear", "chrome", "rgb(10%, 20%, 30%)",
              "hsl(120, 50%, 50%)", (1.0,), "nocolor"]:
        add(lambda c=c: P(color=c))
    add(lambda: plastics.cfrp(color="red"))
    add(lambda: plastics.petg(plastics.PETG.CF, color="#11223380"))
    # see-through
    add(lambda: plastics.pc())
    add(lambda: plastics.pc(opacity=0.5))
    add(lambda: plastics.pc(opacity=1.0))
    add(lambda: plastics.pmma(color="blue", thickness_mm=3.0, roughness=0.1))
    add(lambda: bglass.soda_lime())
    add(lambda: bglass.borosilicate(opacity=0.2, roughness=0.05, thickness_mm=5.0))
    add(lambda: bglass.soda_lime(color="#ccffcc", finish=fin.etch()))
    add(lambda: resins.transparent(opacity=0.8, color="gold"))
    # wood, paper, textile, resins
    add(lambda: bwood.oak())
    add(lambda: bwood.walnut(scale=(2.0, 2.0)))
    add(lambda: bwood.maple(rotation=90.0))
    add(lambda: bwood.mdf())
    add(lambda: bwood.osb(scale=(0.5, 0.5), rotation=45.0))
    add(lambda: bwood.hardwood(bwood.Hardwood.GENERIC))
    add(lambda: bwood.softwood(bwood.Softwood.PINE))
    add(lambda: bwood.engineered_wood(bwood.EngineeredWood.OSB, density=600.0))
    add(lambda: bwood.ash(finish=fin.fine_sanding((2.0, 2.0))))
    add(lambda: bwood.birch(finish=fin.spray_paint("white", fin.Sheen.MATTE)))
    add(lambda: bwood.beech(finish=fin.brushed(), scale=(3.0, 3.0)))
    add(lambda: bwood.american_cherry(finish=fin.brushed((2.0, 2.0)), scale=(3.0, 3.0)))
    add(lambda: bpaper.paper())
    add(lambda: bpaper.paper(color=None))
    add(lambda: bpaper.paper(color="yellow", scale=(2.0, 2.0)))
    add(lambda: bpaper.cardboard())
    add(lambda: bpaper.cardboard(rotation=90.0))
    add(lambda: bpaper.foamboard(color="black"))
    add(lambda: btextile.leather(color="saddlebrown"))
    add(lambda: btextile.felt())
    add(lambda: btextile.woven(scale=(3.0, 3.0), rotation=15.0))
    add(lambda: resins.standard(color="grey"))
    add(lambda: resins.tough())
    add(lambda: resins.ceramic(process=proc.vat()))
    # materials of your own
    add(lambda: metals.custom_metal("MyAlloy", 7000.0, family="aluminum", tensile_strength=300.0,
                                    hardness=Range(10.0, 20.0)))
    add(lambda: metals.custom_metal("Mg", 1800.0, family="magnesium", finish=fin.anodize("red")))
    add(lambda: metals.custom_metal("Gilded", 8000.0, pbr=metal.gold()))
    add(lambda: bwood.custom_wood("Teak", 650.0, family="teak"))
    add(lambda: bwood.custom_wood("Walnutish", 650.0, family="walnut", scale=(2.0, 2.0)))
    add(lambda: bpaper.custom_paper("Kraft", 700.0, family="corrugated_cardboard", color="tan"))
    add(lambda: btextile.custom_textile("Hide", 900.0, family="leather", color="brown"))
    add(lambda: btextile.custom_textile("Cloth", 300.0))
    add(lambda: plastics.custom_plastic("CF-Nylon", 1200.0, family="CFRP"))
    add(lambda: plastics.custom_plastic("Clear", 1200.0, transparent=True, opacity=0.5, thickness_mm=2.0))
    add(lambda: plastics.custom_plastic("Fdm", 1200.0, process=proc.fdm(0.1)))
    add(lambda: resins.custom_resin("R", 1100.0, color="teal"))
    add(lambda: bglass.custom_glass("G", 2400.0, opacity=0.3))
    add(lambda: bglass.custom_glass("G2", 2400.0, transparent=False))
    # FinishedMaterial itself
    base = metals.aluminum().material
    add(lambda: FinishedMaterial(base))
    add(lambda: FinishedMaterial(base, fin.brushed(), color="red", scale=(2.0, 2.0), rotation=10.0))
    add(lambda: FinishedMaterial(base, color="red", thickness_mm=1.0, opacity=0.5, roughness=0.2))
    add(lambda: FinishedMaterial(plastics.pla().material, color="red", thickness_mm=1.0, opacity=0.5,
                                 roughness=0.2, scale=(2.0, 3.0), rotation=45.0, process=proc.fdm()))
    add(lambda: FinishedMaterial(base, pbr=metal.copper()))
    add(lambda: FinishedMaterial(bwood.oak().material, scale=(0.5, 0.5)))
    add(lambda: FinishedMaterial(with_density(base, 1000.0)))
    add(lambda: FinishedMaterial(bglass.soda_lime().material, color="blue", opacity=0.3))

    def set_pbr():
        fm = metals.aluminum()
        fm.pbr = wood.oak()
        return fm

    def unset_pbr():
        fm = FinishedMaterial(base, pbr=metal.copper())
        fm.pbr = None
        return fm

    add(set_pbr)
    add(unset_pbr)
    out.append(outcome(lambda: pbr_summary(get_pbr_properties(base, [fin.brushed()], proc.fdm(), color="red",
                                                              scale=(2.0, 2.0)))))
    out.append(outcome(lambda: pbr_summary(get_pbr_properties(base, fin.brushed((1.0, 1.0), 0.0),
                                                              proc.slm((2.0, 2.0))))))
    out.append(outcome(lambda: pbr_summary(get_pbr_properties(base, pbr=wood.oak()))))

    def deprecated():
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            fm = plastics.pla(process=Process.FDM)
        return {"fm": fm_summary(fm), "warnings": [str(x.message) for x in w]}

    out.append(deprecated())
    return V(out)


def message(f):
    try:
        f()
        return "no error"
    except (ValueError, TypeError) as e:
        return str(e)
    except (IndexError, KeyError):
        return "IndexError/KeyError"
    except ZeroDivisionError:
        return "ZeroDivisionError"


def material_messages():
    out = []
    base = metals.aluminum().material
    out.append(message(lambda: FinishedMaterial(base, fin.brushed(), pbr=metal.copper())))
    out.append(message(lambda: FinishedMaterial(base, color="red", pbr=metal.copper())))
    out.append(message(lambda: FinishedMaterial(base, scale=(2.0, 1.0), pbr=metal.copper())))
    out.append(message(lambda: FinishedMaterial(base, rotation=1.0, pbr=metal.copper())))
    out.append(message(lambda: FinishedMaterial(base, process=proc.fdm(), pbr=metal.copper())))
    out.append(message(lambda: FinishedMaterial(base, fin.brushed(), process=proc.fdm())))
    out.append(message(lambda: FinishedMaterial(base, [], process=proc.fdm())))
    out.append(message(lambda: metals.aluminum(finish=fin.brushed(), process=proc.cast())))
    out.append(message(lambda: registry.factory("aluminum")(color="red")))
    out.append(message(lambda: registry.factory("Alu_G7075_T6")(thickness_mm=1.0)))
    out.append(message(lambda: registry.factory("paper")(opacity=0.5)))
    out.append(message(lambda: registry.factory("oak")(color="red")))
    out.append(message(lambda: registry.factory("abs")(scale=(1.0, 1.0))))
    for name in ["alumnium", "Aluminium", "stainles steel", "pla generic", "xyz", "Alu-G6061", "", "  oak  x",
                 "titanum", "petg-cf2", "a" * 250, "aluminum" * 30, "ToolSteel_D2", "HARDWOOD_OKA", "brass c360",
                 "Nylon PA 12", "steel"]:
        out.append(message(lambda name=name: registry.factory(name)))
    out.append(message(lambda: registry.canonical_name(metals.custom_metal("Mine", 7000.0))))
    out.append(message(lambda: registry.canonical_name(metals.custom_metal("Alu G6061 T6", 7000.0))))
    out.append(message(lambda: registry.canonical_name(metals.aluminum().material)))
    out.append(message(lambda: Range(2.0, 1.0)))
    out.append(message(lambda: plastics.pla(color=(1.0,)).pbr))
    out.append(message(lambda: applicability.typical_materials(fin.Finish("Custom"))))
    out.append(message(lambda: plastics.pla(process=proc.fdm(0.0)).pbr))
    return V(out)


def material_tables():
    tables = [("MECHANICAL_FINISHES", fin.MECHANICAL_FINISHES), ("CHEMICAL_FINISHES", fin.CHEMICAL_FINISHES),
              ("METAL_PLATING_FINISHES", fin.METAL_PLATING_FINISHES), ("COATING_FINISHES", fin.COATING_FINISHES),
              ("MARKING_FINISHES", fin.MARKING_FINISHES)]
    applied = [fin.bead_blast(), fin.brushed((2.0, 3.0), 45.0), fin.fine_sanding((0.5, 0.5)), fin.smooth_machining(),
               fin.electropolish(), fin.anodize("red"), fin.chem_film(), fin.conductive_oxidation(),
               fin.black_oxide(), fin.passivate(), fin.pickle(), fin.dye("blue"), fin.chrome(), fin.gold_plate(),
               fin.nickel_plate(), fin.silver_plate(), fin.tin_plate(), fin.pvd(), fin.pvd("gold"),
               fin.zinc_plate(), fin.vacuum_plating("red", fin.Sheen.MATTE), fin.powder_coat(None, None),
               fin.spray_paint("white"), fin.electrophoresis(), fin.laser_engrave(), fin.etch(), fin.silkscreen(),
               fin.anodize(0x123456), fin.anodize(("red", 0.5)), fin.anodize((0.1, 0.2, 0.3)),
               fin.anodize(Color(1.0, 0.0, 0.0)), fin.anodize("it's")]
    processes = [proc.fdm(), proc.fdm(0.1, 90.0, 2.0), proc.fdm_skin(), proc.fdm_skin(0.6, 0.0), proc.sls(),
                 proc.sls((2.0, 2.0)), proc.mjf(), proc.slm(), proc.vat(), proc.molded(), proc.machined(),
                 proc.cast(), proc.wrought()]
    modules = [("ALL_GLASSES", bglass), ("ALL_METALS", metals), ("ALL_PAPERS", bpaper), ("ALL_PLASTICS", plastics),
               ("ALL_RESINS", resins), ("ALL_TEXTILES", btextile), ("ALL_WOODS", bwood)]
    out = [
        [[name, [[k.name, k.value, v.name, v.notes, repr(v)] for k, v in table.items()]] for name, table in tables],
        [repr(f) for f in fin.ALL_FINISHES],
        [repr(f) for f in applied],
        [applied[0] == fin.bead_blast(), applied[1] == fin.brushed(), fin.Finish("Brushed") == applied[1].finish],
        [repr(p) for p in processes],
        [processes[0] == proc.fdm(), processes[0] == proc.fdm_skin()],
        [p.value for p in Process],
        [proc.AUTHORED_LAYER_HEIGHT_MM, proc.AUTHORED_LINE_WIDTH_MM, proc.AUTHORED_SKIN_ROTATION],
        [[m.name, [f.name for f in applicability.typical_finishes(m)]] for m in ALL_MATERIALS],
        [[k.name, sorted(v), [m.name for m in applicability.typical_materials(k)]]
         for k, v in applicability.TYPICAL_SUBSTRATES.items()],
        [[f.name, [m.name for m in applicability.typical_materials(f)]] for f in fin.ALL_FINISHES],
        [m.name for m in applicability.typical_materials(fin.brushed((2.0, 2.0)))],
        [f.name for f in applicability.typical_finishes(bwood.custom_wood("x", 1.0).material)],
        list(registry.material_names()),
        [m.name for m in ALL_MATERIALS],
        [[name, [m.name for m in getattr(mod, name)]] for name, mod in modules],
        [[g.value, m.name] for g, m in metals.ALU_MATERIALS.items()],
        [[g.value, m.name] for g, m in bwood.HARDWOOD_MATERIALS.items()],
        [[g.value, m.name] for g, m in plastics.NYLON_MATERIALS.items()],
        [list(x) for x in PROPERTY_UNITS.items()],
        [list(x) for x in MECHANICAL_PROPERTY_UNITS.items()],
        [list(x) for x in THERMAL_PROPERTY_UNITS.items()],
        [sorted(s) for s in (ALLOWED_CATEGORIES, FERROUS, METAL, PAINTABLE, COATABLE_POLY, EVERYTHING)],
        [NOT_SUITABLE.min, NOT_SUITABLE.max, Range(1.0, 3.0).value_at(0.25), Range(2.0, 2.0).value_at(0.5)],
        material_summary(with_density(metals.aluminum().material, 2000.0)),
        material_summary(with_density(metals.aluminum().material, None)),
        material_summary(metals.custom_metal("C", 1.0, tensile_strength=NOT_SUITABLE, yield_strength=5.0).material),
        str(bwood.custom_wood("W", 600.0).material),
        str(bpaper.custom_paper("P", 600.0, thickness=0.1).material),
    ]
    return V(out)


CASES = {
    "pbr_pil_textures": pil_textures,
    "pbr_pil_resize": pil_resize,
    "pbr_presets": pbr_presets,
    "pbr_preset_variants": pbr_preset_variants,
    "pbr_overrides": pbr_overrides,
    "pbr_interpolate_color": pbr_interpolate_color,
    "pbr_material_catalog": material_catalog,
    "pbr_material_variants": material_variants,
    "pbr_material_messages": material_messages,
    "pbr_material_tables": material_tables,
}
