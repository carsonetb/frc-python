from __future__ import annotations

from typing import override

from frc_python.units.base import Unit, UnitSquared


class Time(Unit):
    """
    Stored raw in units of seconds.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "seconds":
                return self.seconds()
            case _:
                assert False

    @override
    def withval(self, new: float) -> Time:
        return Time(new, self.unit)

    def seconds(self) -> float:
        return self.raw

    def minutes(self) -> float:
        return self.raw / 60.0


def seconds(val: float) -> Time:
    return Time(val, "seconds")


def minutes(val: float) -> Time:
    return Time(val * 60.0, "minutes")


TimeSquared = UnitSquared[Time]
