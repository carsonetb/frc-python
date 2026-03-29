from __future__ import annotations

from typing import override

from frc_python.units.base import Unit


class Temperature(Unit):
    """
    Stored raw in units of fahrenheit.
    """

    @override
    def in_current(self) -> float:
        match self.unit:
            case "celsius":
                return self.celsius()
            case "fahrenheit":
                return self.fahrenheit()
            case _:
                assert False

    @override
    def withval(self, new: float) -> Temperature:
        return Temperature(new, self.unit)

    def celsius(self) -> float:
        return self

    def fahrenheit(self) -> float:
        return (self * (9.0 / 5.0)) + 32.0


def celsius(val: float) -> Temperature:
    return Temperature(val, "celsius")


def fahrenheit(val: float) -> Temperature:
    return Temperature((val - 32.0) * (5.0 / 9.0), "fahrenheit")
