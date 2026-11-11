"""Fillet every edge (120) of a block with nine pockets."""
from build123d import Box, GridLocations, Pos, fillet

block = Box(60, 60, 10) - GridLocations(18, 18, 3, 3) * (Pos(0, 0, 3) * Box(10, 10, 4))


def model():
    return fillet(block.edges(), 1)
