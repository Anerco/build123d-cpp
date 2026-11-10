"""Helpers shared by the bd_warehouse comparison cases (cases_bdw_*_py.py)."""

from build123d import Location, Vector

from values import V


def error_of(build):
    """The error a call raises as [kind, message], kind named as the C++ exception it maps to."""
    try:
        build()
    except ZeroDivisionError as e:
        return V(["domain_error", str(e)])
    except (KeyError, IndexError) as e:
        return V(["out_of_range", str(e)])
    except (ValueError, AttributeError, TypeError) as e:
        return V(["invalid_argument", str(e)])
    except RuntimeError as e:
        return V(["runtime_error", str(e)])
    return V(None)


def loc_values(loc: Location):
    return [loc.position, loc.orientation]


def num(x):
    """A table value: numbers stay numbers, other values become their str."""
    if isinstance(x, (int, float)) and not isinstance(x, bool):
        return x
    return str(x)


def table_values(table):
    """A dict of dict of values, keys as strings, in order."""
    return [[str(k), [[str(k2), num(v2)] for k2, v2 in row.items()]] for k, row in table.items()]


def children_values(shape):
    """The labels and volumes of the children of an assembly, in order."""
    return [[c.label, c.volume] for c in shape.children]


def attrs(obj, names):
    """The listed attributes of an object, as [name, value] pairs."""
    out = []
    for name in names:
        value = getattr(obj, name)
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, Vector):
            value = [value.X, value.Y, value.Z]
        out.append([name, value])
    return V(out)
