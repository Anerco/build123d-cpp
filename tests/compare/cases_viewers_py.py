"""Python twins of cases_viewers.cpp (needs vtk in the Python environment)."""

import json
import os
import tempfile

from build123d import Box, Cylinder, Part, Rectangle, Sphere
from build123d.jupyter_tools import shape_to_html
from build123d.vtk_tools import to_vtk_poly_data

from values import V

OUT_DIR = os.path.join(tempfile.gettempdir(), "build123d_compare_viewers")


def r5(v):
    # matches std::round(v * 1e5) / 1e5 except on exact ties
    return round(float(v) * 1e5) / 1e5


def arrays(poly):
    from vtkmodules.util.numpy_support import vtk_to_numpy  # pylint: disable=import-outside-toplevel

    if poly is None or poly.GetNumberOfPoints() == 0:
        return [], [], [], []
    points = vtk_to_numpy(poly.GetPoints().GetData()).tolist()
    polys = vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).reshape(-1, 3).tolist()
    pn = poly.GetPointData().GetNormals()
    cn = poly.GetCellData().GetNormals()
    point_normals = vtk_to_numpy(pn).tolist() if pn is not None else []
    cell_normals = vtk_to_numpy(cn).tolist() if cn is not None else []
    return points, polys, point_normals, cell_normals


def summary(points, polys, point_normals, cell_normals):
    point_rows = []
    for i, p in enumerate(points):
        row = [r5(c) for c in p]
        if point_normals:
            row += [r5(c) for c in point_normals[i]]
        point_rows.append(row)
    tri_rows = []
    for t, tri in enumerate(polys):
        verts = sorted([r5(c) for c in points[i]] for i in tri)
        row = [c for v in verts for c in v]
        if cell_normals:
            row += [r5(c) for c in cell_normals[t]]
        tri_rows.append(row)
    return {
        "points": len(points),
        "polys": len(polys),
        "point_rows": sorted(point_rows),
        "tri_rows": sorted(tri_rows),
    }


def mesh_summary(shape, tolerance=None, angular_tolerance=None, normals=False):
    return summary(*arrays(to_vtk_poly_data(shape, tolerance, angular_tolerance, normals)))


def holed():
    return Box(4, 4, 2) - Cylinder(1, 2)


def empty():
    try:
        to_vtk_poly_data(Part())
    except ValueError:
        raise
    return V(None)


def vtkpoly_string():
    """Read the C++ .vtp files back with VTK; each must hold the C++ mesh, which the vtk_mesh
    cases compare with Python's."""
    from vtkmodules.vtkIOXML import vtkXMLPolyDataReader  # pylint: disable=import-outside-toplevel

    out = []
    for name in ("box", "holed", "face"):
        reader = vtkXMLPolyDataReader()
        reader.SetFileName(os.path.join(OUT_DIR, name + ".vtp"))
        reader.Update()
        out.append(summary(*arrays(reader.GetOutput())))
    return V(out)


def html():
    text = shape_to_html(Box(1, 2, 3)).data
    start = text.index("shape-")
    div_id = text[start : start + 14]
    skeleton = text.replace(div_id, "DIV")
    data_start = skeleton.index("const data = ") + 13
    data_end = skeleton.index(";\n", data_start)
    payload = json.loads(skeleton[data_start:data_end])
    skeleton = skeleton[:data_start] + "DATA" + skeleton[data_end:]
    item = payload[0]
    return V(
        {
            "skeleton": skeleton,
            "id_length": len(div_id),
            "keys": list(item.keys()),
            "color": json.dumps(item["color"]),
            "position": json.dumps(item["position"]),
            "orientation": json.dumps(item["orientation"]),
            "shape_starts": item["shape"][:21],
        }
    )


CASES = {
    "viewers_vtk_mesh_box": lambda: V(mesh_summary(Box(1, 2, 3), 1e-3, 0.1, True)),
    "viewers_vtk_mesh_holed": lambda: V(mesh_summary(holed(), 0.01, 0.3, True)),
    "viewers_vtk_mesh_sphere": lambda: V(mesh_summary(Sphere(1), 0.05, 0.5, True)),
    "viewers_vtk_mesh_no_normals": lambda: V(mesh_summary(Cylinder(1, 2), 0.05, 0.5)),
    "viewers_vtk_mesh_empty": empty,
    "viewers_vtkpoly_string": vtkpoly_string,
    "viewers_shape_to_html": html,
}
