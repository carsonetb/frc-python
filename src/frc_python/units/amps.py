from __future__ import annotations

from typing import override

from frc_python.units.base import Unit


class Amps(Unit):
    """
    Basic unit of amperage, can only be stored in amps.
    """

    @override
    def in_current(self) -> float:
        return self.raw

    @override
    def withval(self, new: float) -> Amps:
        return Amps(new, self.unit)

    def amps(self) -> float:
        return self.raw


def amps(val: float) -> Amps:
    return Amps(val, "amps")
