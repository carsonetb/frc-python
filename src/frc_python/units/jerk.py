from frc_python.units.acceleration import (
    AngularAcceleration,
    LinearAcceleration,
    meters_per_second_squared,
    radians_per_second_squared,
)
from frc_python.units.angle import Angle
from frc_python.units.base import UnitPerUnit
from frc_python.units.distance import Distance
from frc_python.units.time import Time, seconds


class LinearJerk(UnitPerUnit[LinearAcceleration, Time]):
    """
    Linear jerk (or acceleration per time), stored raw in m/s/s/s
    """

    @property
    def per_second(self) -> LinearAcceleration:
        return self.mulr(seconds(1))

    @property
    def per_second_cubed(self) -> Distance:
        return self.per_second.per_second.per_second

    @property
    def meters_per_second_squared(self) -> float:
        return self.raw


class AngularJerk(UnitPerUnit[AngularAcceleration, Time]):
    """
    Angular jerk (or angular acceleration per time), stored raw in m/s/s/s
    """

    @property
    def per_second(self) -> AngularAcceleration:
        return self.mulr(seconds(1))

    @property
    def per_second_cubed(self) -> Angle:
        return self.per_second.per_second.per_second

    @property
    def radians_per_second_cubed(self) -> float:
        return self.raw


def meters_per_second_cubed(val: float) -> LinearJerk:
    return LinearJerk(meters_per_second_squared(val), seconds(1))


def radians_per_second_cubed(val: float) -> AngularJerk:
    return AngularJerk(radians_per_second_squared(val), seconds(1))
