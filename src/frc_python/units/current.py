from __future__ import annotations

from typing import override

from frc_python.units.base import Unit


class Current(Unit):
    """
    Basic unit of current, can only be stored in amps.
    """

    @override
    def in_current(self) -> float:
        return self.raw

    @override
    def withval(self, new: float) -> Current:
        return Current(new, self.unit)

    @override
    def in_base(self) -> Current:
        return self

    def amps(self) -> float:
        return self.raw


def amps(val: float) -> Current:
    return Current(val, "amps")
