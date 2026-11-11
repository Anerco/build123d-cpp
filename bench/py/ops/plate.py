"""The README's example."""
from build123d import *  # noqa: F403


def model():
    plate = Box(80, 60, 10) - GridLocations(60, 40, 2, 2) * Cylinder(3, 10)
    plate = fillet(plate.edges().filter_by(Axis.Z), 5)
    plate -= Pos(0, 0, 5) * CounterBoreHole(4, 7, 3, 10)
    return plate
