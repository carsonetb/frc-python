from frc_python.units.base import UnitPerUnit, UnitSquared, UnitUnit
from frc_python.units.distance import Distance, meters
from frc_python.units.mass import Mass, kilograms
from frc_python.units.time import Time, seconds

Joules = UnitUnit[Mass, UnitPerUnit[UnitSquared[Distance], UnitSquared[Time]]]
"""Stored raw in units of kg * m^2/s^2"""
Power = UnitPerUnit[Joules, Time]
"""Stored raw in units of Joules per second, or (kg * m^2/s^3)"""


def joules(val: float) -> Joules:
    return Joules(
        kilograms(val), UnitPerUnit(UnitSquared(meters(1)), UnitSquared(seconds(1)))
    )


def power(val: float) -> Power:
    return Power(joules(val), seconds(1))
