from frc_python.units.acceleration import LinearAcceleration, meters_per_second_squared
from frc_python.units.base import UnitPerUnit
from frc_python.units.time import Time, seconds


class LinearJerk(UnitPerUnit[LinearAcceleration, Time]):
    """
    Linear acceleration (or velocity per time), stored raw in m/s^2
    """

    def meters_per_second_squared(self) -> float:
        return self.raw


def meters_per_second_cubed(val: float) -> LinearJerk:
    return LinearJerk(meters_per_second_squared(val), seconds(1))
