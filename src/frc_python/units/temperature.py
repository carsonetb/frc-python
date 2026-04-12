from __future__ import annotations

from typing import override

from frc_python.units.base import Unit


class Temperature(Unit):
    """
    Basic unit of thermodynamic temperature, stored raw in units of kelvin.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "kelvin":
                return self.kelvin
            case "celsius":
                return self.celsius
            case "fahrenheit":
                return self.fahrenheit
            case _:
                assert False

    @override
    def withval(self, new: float) -> Temperature:
        return Temperature(new, self.unit)

    @override
    def in_base(self) -> Temperature:
        return Temperature(self.kelvin, "kelvin")

    @property
    def kelvin(self) -> float:
        return self.raw

    @property
    def celsius(self) -> float:
        return self.raw - 273.15

    @property
    def fahrenheit(self) -> float:
        return (self.celsius * (9.0 / 5.0)) + 32.0


def kelvin(val: float) -> Temperature:
    return Temperature(val, "kelvin")


def celsius(val: float) -> Temperature:
    return Temperature(val + 273.15, "celsius")


def fahrenheit(val: float) -> Temperature:
    return Temperature((val - 32.0) * (5.0 / 9.0), "fahrenheit")
