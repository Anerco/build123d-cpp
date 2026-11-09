"""Results of comparison cases that are not shapes (see case_registry.hpp for the C++ side).

A case returns V(data); data may hold numbers, bools, strings, None, lists, dicts, enums
(compared by name), and Vector/Vertex (compared as [X, Y, Z]).
"""

from enum import Enum

from build123d import Vector, Vertex


class V:
    """A case result compared value by value (numbers with a tolerance)."""

    def __init__(self, data):
        self.data = to_json(data)


def to_json(x):
    if isinstance(x, V):
        return x.data
    if isinstance(x, Vector):
        return [x.X, x.Y, x.Z]
    if isinstance(x, Vertex):
        return [x.X, x.Y, x.Z]
    if isinstance(x, Enum):
        return x.name
    if isinstance(x, dict):
        return {str(k): to_json(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_json(i) for i in x]
    if x is None or isinstance(x, (bool, int, float, str)):
        return x
    raise TypeError(f"Can't compare a {type(x).__name__}; convert it to numbers, strings or lists")
