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


CASES = {
    "pbr_pil_textures": pil_textures,
    "pbr_pil_resize": pil_resize,
    "pbr_presets": pbr_presets,
    "pbr_preset_variants": pbr_preset_variants,
    "pbr_overrides": pbr_overrides,
    "pbr_interpolate_color": pbr_interpolate_color,
}
