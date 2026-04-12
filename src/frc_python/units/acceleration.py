from frc_python.units.angle import Angle
from frc_python.units.base import UnitPerUnit
from frc_python.units.distance import Distance
from frc_python.units.time import Time, seconds
from frc_python.units.velocity import (
    AngularVelocity,
    LinearVelocity,
    meters_per_second,
    radians_per_second,
)


class LinearAcceleration(UnitPerUnit[LinearVelocity, Time]):
    """
    Linear acceleration (or velocity per time), stored raw in m/s/s
    """

    @property
    def per_second(self) -> LinearVelocity:
        return self.mulr(seconds(1))

    @property
    def per_second_squared(self) -> Distance:
        return self.per_second.per_second

    @property
    def meters_per_second_squared(self) -> float:
        return self.raw


class AngularAcceleration(UnitPerUnit[AngularVelocity, Time]):
    """
    Angular acceleration (or change in *change* in angle per time), stored raw in rad/s/s
    """

    @property
    def per_second(self) -> AngularVelocity:
        return self.mulr(seconds(1))

    @property
    def per_second_squared(self) -> Angle:
        return self.per_second.per_second

    @property
    def radians_per_second_squared(self) -> float:
        return self.raw


def meters_per_second_squared(val: float) -> LinearAcceleration:
    return LinearAcceleration(meters_per_second(val), seconds(1))


def radians_per_second_squared(val: float) -> AngularAcceleration:
    return AngularAcceleration(radians_per_second(val), seconds(1))
