from __future__ import annotations

from math import cos, pi, sin

from units.angle import Angle, degrees, radians, rotations
from units.base import UnitPerUnit
from units.distance import Distance, meters
from units.time import Time, seconds


class LinearVelocity(UnitPerUnit[Distance, Time]):
    """
    Linear velocity, stored raw in meters / second.
    """

    def get_vertical_component(self, angle: Angle) -> LinearVelocity:
        return LinearVelocity(
            meters(self.meters_per_second() * sin(angle.radians())), seconds(1)
        )

    def get_horizontal_component(self, angle: Angle) -> LinearVelocity:
        return LinearVelocity(
            meters(self.meters_per_second() * cos(angle.radians())), seconds(1)
        )

    def to_angular(self, radius: Distance) -> AngularVelocity:
        return radians_per_second(self.raw / radius.raw)

    def meters_per_second(self) -> float:
        return self.raw


class AngularVelocity(UnitPerUnit[Angle, Time]):
    """
    Angular velocity, stored raw in radians / second
    """

    def to_linear(self, radius: Distance) -> LinearVelocity:
        return meters_per_second(self.raw * radius.raw)

    def radians_per_second(self) -> float:
        return self.raw

    def degrees_per_second(self) -> float:
        return self.raw * (180.0 / pi)

    def rotations_per_second(self) -> float:
        return self.raw / (2.0 * pi)


def meters_per_second(val: float) -> LinearVelocity:
    return LinearVelocity(meters(val), seconds(1))


def radians_per_second(val: float) -> AngularVelocity:
    return AngularVelocity(radians(val), seconds(1))


def degrees_per_second(val: float) -> AngularVelocity:
    return AngularVelocity(degrees(val), seconds(1))


def rotations_per_second(val: float) -> AngularVelocity:
    return AngularVelocity(rotations(val), seconds(1))
