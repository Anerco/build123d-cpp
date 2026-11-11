from bd_warehouse.thread import IsoThread


def model():
    return IsoThread(10, 1.5, 20)
