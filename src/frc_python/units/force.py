from frc_python.units.acceleration import LinearAcceleration
from frc_python.units.base import PackedUnitUnit
from frc_python.units.mass import Mass

Newtons = PackedUnitUnit[Mass, LinearAcceleration]
"""Basic unit of force, units are kg*(m/s^2)"""


def newtons(val: float) -> Newtons:
    return Newtons(val, "newtons")
