from __future__ import annotations

from typing import override

from units.base import Unit, UnitSquared, UnitUnit
from units.distance import Distance


class Mass(Unit):
    """
    Stored in raw units of kilograms.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "kilograms":
                return self.kilograms
            case _:
                assert False

    @override
    def withval(self, new: float) -> Mass:
        return Mass(new, self.unit)

    @property
    def kilograms(self) -> float:
        return self.raw


def kilograms(val: float) -> Mass:
    return Mass(val, "kilograms")


MomentOfInertia = UnitUnit[Mass, UnitSquared[Distance]]
"""Stored in raw units of kg*m^2"""


def kilogram_meters_squared(val: float) -> MomentOfInertia:
    return MomentOfInertia(kilograms(val), UnitSquared[Distance](Distance(1, "meters")))
