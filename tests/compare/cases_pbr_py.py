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


CASES = {
    "pbr_pil_textures": pil_textures,
    "pbr_pil_resize": pil_resize,
}
