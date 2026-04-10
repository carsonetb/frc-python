from frc_python.units.base import UnitPerUnit
from frc_python.units.time import Time, seconds
from frc_python.units.velocity import LinearVelocity, meters_per_second


class LinearAcceleration(UnitPerUnit[LinearVelocity, Time]):
    """
    Linear acceleration (or velocity per time), stored raw in m/s^2
    """

    def meters_per_second_squared(self) -> float:
        return self.raw


def meters_per_second_squared(val: float) -> LinearAcceleration:
    return LinearAcceleration(meters_per_second(val), seconds(1))
