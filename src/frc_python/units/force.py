from frc_python.units.acceleration import LinearAcceleration, meters_per_second_squared
from frc_python.units.base import UnitUnit
from frc_python.units.mass import Mass, kilograms


class Force(UnitUnit[Mass, LinearAcceleration]):
    """SI derived unit of force, units newtons, or, kg*(m/s^2)"""

    def __init__(self, value: Mass, times: LinearAcceleration) -> None:
        super().__init__(value, times)
        self.unit = "newtons"

    @property
    def newtons(self) -> float:
        return self.raw


def newtons(val: float) -> Force:
    return Force(kilograms(1), meters_per_second_squared(val))
