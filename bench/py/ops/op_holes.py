"""One boolean: a plate minus 225 cylinders."""
from build123d import Box, Cylinder, GridLocations


def model():
    return Box(200, 200, 10) - GridLocations(12, 12, 15, 15) * Cylinder(4, 10)
