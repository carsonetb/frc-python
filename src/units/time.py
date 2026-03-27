from __future__ import annotations

from typing import override

from units.base import Unit, UnitSquared


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
        return self


def seconds(val: float) -> Time:
    return Time(val, "seconds")


TimeSquared = UnitSquared[Time]
