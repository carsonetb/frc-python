from __future__ import annotations

from math import pi
from typing import override

from wpimath.geometry import Rotation2d

from frc_python.units.base import Unit, UnitUnit
from frc_python.units.distance import Distance, meters
from frc_python.units.time import Time, seconds


class Angle(Unit):
    """
    SI derived unit for a plane angle, can be stored in:

    - Radians
    - Degrees
    - Rotations

    Stored raw in radians.

    The Angle unit is continuous, similar to Rotation2d.
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
        """
        Converts this Angle to a WPILib Rotation2d.
        """

        return Rotation2d(self.raw)

    def to_linear(self, radius: Distance) -> Distance:
        """
        The length of a spool (say, of string), with a specified radius, if it
        were unraveled by this angle.
        """

        return meters(self.radians * radius.meters)

    @staticmethod
    def from_linear(linear: Distance, radius: Distance) -> Angle:
        """
        The total angle of an unraveled spool (say, of string), with a
        specified radius, would have turned if it was unraveled by a specified
        distance. Inverse of to_linear.
        """

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
