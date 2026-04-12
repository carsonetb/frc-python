from __future__ import annotations

from typing import override

from frc_python.units.base import Unit, UnitSquared, UnitUnit
from frc_python.units.distance import Distance


class Mass(Unit):
    """
    Basic unit of mass, can be stored in:

    - Kilograms
    - Grams
    - Pounds

    Stored raw in kilograms.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "grams":
                return self.grams
            case "kilograms":
                return self.kilograms
            case "pounds":
                return self.pounds
            case _:
                assert False

    @override
    def withval(self, new: float) -> Mass:
        return Mass(new, self.unit)

    @override
    def in_base(self) -> Mass:
        return Mass(self.kilograms, "kilograms")

    @property
    def grams(self) -> float:
        return self.raw * 1000.0

    @property
    def kilograms(self) -> float:
        return self.raw

    @property
    def pounds(self) -> float:
        return self.raw / 0.453592


def grams(val: float) -> Mass:
    return Mass(val / 1000, "grams")


def kilograms(val: float) -> Mass:
    return Mass(val, "kilograms")


def pounds(val: float) -> Mass:
    return Mass(val * 0.453592, "pounds")


MomentOfInertia = UnitUnit[Mass, UnitSquared[Distance]]
"""Stored in raw units of kg*m^2"""


def kilogram_meters_squared(val: float) -> MomentOfInertia:
    return MomentOfInertia(kilograms(val), UnitSquared[Distance](Distance(1, "meters")))
