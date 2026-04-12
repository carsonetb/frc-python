from __future__ import annotations

from math import pi
from typing import override

from wpimath.geometry import Rotation2d

from frc_python.units.base import Unit, UnitUnit
from frc_python.units.distance import Distance, meters
from frc_python.units.time import Time, seconds


class Angle(Unit):
    """
    Stored raw in radians.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "radians":
                return self.radians
            case "degrees":
                return self.degrees
            case "rotations":
                return self.rotations
            case _:
                assert False

    @override
    def withval(self, new: float) -> Angle:
        return Angle(new, self.unit)

    @override
    def in_base(self) -> Angle:
        return radians(self.radians)

    @property
    def radians(self) -> float:
        return self.raw

    @property
    def degrees(self) -> float:
        return self.raw * (180.0 / pi)

    @property
    def rotations(self) -> float:
        return self.raw / (2.0 * pi)

    def to_rotation2d(self) -> Rotation2d:
        return Rotation2d(self.raw)

    def to_linear(self, radius: Distance) -> Distance:
        return meters(self.radians * radius.meters)

    @staticmethod
    def from_linear(linear: Distance, radius: Distance) -> Angle:
        return radians(linear.meters / radius.meters)


def radians(val: float) -> Angle:
    return Angle(val, "radians")


def degrees(val: float) -> Angle:
    return Angle(val * (pi / 180.0), "degrees")


def rotations(val: float) -> Angle:
    return Angle(val * 2 * pi, "rotations")


AngleTime = UnitUnit[Angle, Time]
"""Stored raw in units of rad s"""


def radian_seconds(val: float) -> AngleTime:
    return AngleTime(radians(val), seconds(1))
