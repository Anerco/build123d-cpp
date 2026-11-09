"""Write the files in tests/compare/data that cases_io.cpp reads: data written by Python
build123d, so the C++ side can be checked reading it. Run it with Python build123d 0.13.0:

    python tests/compare/make_io_fixtures.py
"""

import os

from build123d import *  # noqa: F403
from build123d.persistence import serialize_location, serialize_shape

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")


def write(name, data):
    with open(os.path.join(DATA, name), "wb") as f:
        f.write(data)


def persistence():
    shape = Pos(1, 2, 3) * (Box(5, 6, 7) - Cylinder(2, 7)) + Pos(0, 0, 5) * Torus(4, 1)
    write("persist_py_shape.bin", serialize_shape(shape.wrapped))
    meshed = Rot(10, 20, 30) * Sphere(3)
    meshed.mesh(0.5, 0.5)
    write("persist_py_meshed.bin", serialize_shape(meshed.wrapped))
    write("persist_py_location.bin", serialize_location(Location((4, -5, 6), (30, 60, -45)).wrapped))


def main():
    os.makedirs(DATA, exist_ok=True)
    persistence()


if __name__ == "__main__":
    main()
