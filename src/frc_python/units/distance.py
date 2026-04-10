from __future__ import annotations

from typing import override

from frc_python.units.base import Unit, UnitUnit
from frc_python.units.time import Time, seconds


class Distance(Unit):
    """
    Stored raw in meters.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "meters":
                return self.meters()
            case "inches":
                return self.inches()
            case "feet":
                return self.feet()
            case _:
                assert False

    @override
    def withval(self, new: float) -> Distance:
        return Distance(new, self.unit)

    @override
    def in_base(self) -> Distance:
        return Distance(self.meters(), "meters")

    def meters(self) -> float:
        return self.raw

    def inches(self) -> float:
        return self.raw * 39.3701

    def feet(self) -> float:
        return self.raw / 0.3048


def meters(val: float) -> Distance:
    return Distance(val, "meters")


def inches(val: float) -> Distance:
    return Distance(val / 39.3701, "inches")


def feet(val: float) -> Distance:
    return Distance(val * 0.3048, "feet")


DistanceTime = UnitUnit[Distance, Time]


def meter_seconds(val: float) -> DistanceTime:
    return DistanceTime(meters(val), seconds(1))
