from __future__ import annotations

from typing import override

from units.base import Unit, UnitUnit
from units.time import Time, seconds


class Distance(Unit):
    """
    Stored raw in meters.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "meters":
                return self.meters()
            case _:
                assert False

    @override
    def withval(self, new: float) -> Distance:
        return Distance(new, self.unit)

    def meters(self) -> float:
        return self

    def inches(self) -> float:
        return self * 39.3701


def meters(val: float) -> Distance:
    return Distance(val, "meters")


def inches(val: float) -> Distance:
    return Distance(val / 39.3701, "inches")


DistanceTime = UnitUnit[Distance, Time]


def meter_seconds(val: float) -> DistanceTime:
    return DistanceTime(meters(val), seconds(1))
