"""Render STEP files written by the C++ examples into one picture (docs/examples.png).

Usage: python render_examples.py <directory with the examples' STEP files> <output.png>
Needs Python build123d (to read STEP) and matplotlib.
"""

import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from build123d import import_step  # noqa: E402

# file, title, elevation, azimuth
VIEWS = [
    ("tea_cup.step", "tea_cup", 20, -60),
    ("key_cap.step", "key_cap", 30, -60),
    ("lego.step", "lego", 35, -55),
    ("heat_exchanger.step", "heat_exchanger", 25, -40),
    ("canadian_flag.step", "canadian_flag", 15, -80),
    ("card_box.step", "custom_sketch_objects", 45, -70),
    ("projection.step", "projection", 20, -125),
    ("pegboard_j_hook.step", "pegboard_j_hook", 25, -60),
]
DEFAULT_COLOR = (0.55, 0.65, 0.8)
LIGHT = np.array([0.4, -0.6, 0.7]) / np.linalg.norm([0.4, -0.6, 0.7])


def leaf_parts(shape, color=None):
    """(shape placed in world coordinates, color) for each assembly leaf."""
    color = tuple(shape.color)[:3] if shape.color is not None else color
    if shape.children:
        for child in shape.children:
            yield from leaf_parts(child, color)
    else:
        # Assembly children are placed relative to their parents
        yield shape.located(shape.global_location), color or DEFAULT_COLOR


def draw(ax, shape, title, elev, azim):
    bbox = shape.bounding_box()
    tolerance = bbox.diagonal / 400
    points = []
    for part, color in leaf_parts(shape):
        vertices, triangles = part.tessellate(tolerance, 0.2)
        if not triangles:
            continue
        v = np.array([tuple(p) for p in vertices])
        tri = v[np.array(triangles)]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normals /= np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12
        shade = 0.35 + 0.65 * np.abs(normals @ LIGHT)
        colors = np.clip(np.array(color)[None, :] * shade[:, None], 0, 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=colors, edgecolors="none", linewidths=0))
        points.append(v)
    v = np.vstack(points)
    center = (v.min(0) + v.max(0)) / 2
    radius = (v.max(0) - v.min(0)).max() / 2
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev, azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=11)


def main():
    step_dir, output = sys.argv[1], sys.argv[2]
    fig = plt.figure(figsize=(16, 8.4), dpi=110)
    for i, (file_name, title, elev, azim) in enumerate(VIEWS):
        ax = fig.add_subplot(2, 4, i + 1, projection="3d")
        draw(ax, import_step(os.path.join(step_dir, file_name)), title, elev, azim)
    fig.subplots_adjust(left=0, right=1, top=0.95, bottom=0, wspace=0, hspace=0.05)
    fig.savefig(output, facecolor="white")


if __name__ == "__main__":
    main()
