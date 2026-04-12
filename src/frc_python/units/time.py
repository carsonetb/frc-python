from __future__ import annotations

from typing import override

from frc_python.units.base import Unit, UnitSquared


class Time(Unit):
    """
    Basic unit of time, can be stored in:

    - Microseconds
    - Milliseconds
    - Seconds
    - Minutes

    Stored raw in units of seconds.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "microseconds":
                return self.microseconds
            case "milliseconds":
                return self.milliseconds
            case "seconds":
                return self.seconds
            case "minutes":
                return self.minutes
            case _:
                assert False

    @override
    def withval(self, new: float) -> Time:
        return Time(new, self.unit)

    @override
    def in_base(self) -> Time:
        return Time(self.seconds, "seconds")

    @property
    def microseconds(self) -> float:
        return self.raw * 1000000.0

    @property
    def milliseconds(self) -> float:
        return self.raw * 1000.0

    @property
    def seconds(self) -> float:
        return self.raw

    @property
    def minutes(self) -> float:
        return self.raw / 60.0


def microseconds(val: float) -> Time:
    return Time(val / 1000000.0, "microseconds")


def milliseconds(val: float) -> Time:
    return Time(val / 1000.0, "milliseconds")


def seconds(val: float) -> Time:
    return Time(val, "seconds")


def minutes(val: float) -> Time:
    return Time(val * 60.0, "minutes")


TimeSquared = UnitSquared[Time]
