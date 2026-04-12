from __future__ import annotations

from abc import ABC, abstractmethod
from copy import deepcopy
from functools import singledispatchmethod
from typing import Self, cast, override

from pykit.autolog import autolog_output, autologgable_output


@autologgable_output
class Unit(ABC):
    """
    Base class for a unit. A unit stores it's raw value, which is
    either in or derived from standard international base units,
    and the actual unit it would be printed in as a string.

    Included SI units:

        - Time
        - Distance (length)
        - Mass
        - (electric) Current
        - (thermodynamic) Temperature

    Included SI derived units:

        - (plane) Angle
        - Frequency
        - Force
        - Energy (work, amount of heat)
        - Power
        - (electric) Charge
        - Voltage (electric potential difference) (with many other voltage-derived units)
        - (electrical) Resistance
        - (electrical) Conductance
        - Capacitance
        - Inductance
        - Magnetic Flux
        - Temperature

    Other derived units:

        - Linear Velocity
        - Angular Velocity
        - Linear Acceleration
        - Angular Acceleration
        - Linear Jerk
        - Angular Jerk
    """

    def __init__(self, value: float, unit: str) -> None:
        self.raw: float = value
        self.unit: str = unit

    @autolog_output("Value")
    def _log_value(self):
        return self.in_current()

    @autolog_output("Unit")
    def _log_unit(self):
        return self.unit

    @abstractmethod
    def in_current(self) -> float:
        """
        Gets this unit in the current unit it is stored in.
        """

        pass

    @abstractmethod
    def withval(self, new: float) -> Self:
        """
        Return this unit with a new value in the same unit.
        """

        pass

    @abstractmethod
    def in_base(self) -> Self:
        pass

    def muldim[U: Unit](self, other: U) -> UnitUnit[Self, U]:
        """
        Multiplies the units, preserving their types. For example,
        kilograms times meters per second would become kg * (m/s),
        or, Mass times Velocity would become UnitUnit[Mass, Velocity].
        """

        return UnitUnit(self, other)

    def divdim[U: Unit](self, other: U) -> UnitPerUnit[Self, U]:
        """
        Divides the units, preserving their types. For example, meters
        divided by seconds would become m/s, or, Distance divided by
        Time would become UnitPerUnit[Distance, Time]
        """
        return UnitPerUnit(self, other)

    def __float__(self) -> float:
        return self.raw

    @override
    def __str__(self) -> str:
        return f"{self.in_current()} {self.unit}"

    @override
    def __repr__(self) -> str:
        return f"Unit<current={self.in_current()}, unit={self.unit}>"

    def clamp(self, minimum: Self, maximum: Self) -> Self:
        return self.withval(
            max(minimum.in_current(), min(self.in_current(), maximum.in_current()))
        )

    def __neg__(self) -> Self:
        return self.withval(-self.in_current())

    def __add__(self, other: Self) -> Self:
        return self.withval(self.raw + other.raw)

    def __sub__(self, other: Self) -> Self:
        return self.withval(self.raw - other.raw)

    def __mul__(self, other: Self) -> Self:
        return self.withval(self.raw * other.raw)

    def mulratio(self, ratio: float) -> Self:
        return self.withval(self.raw * ratio)

    def __truediv__(self, other: Self) -> Self:
        return self.withval(self.raw / other.raw)

    def __lt__(self, other: Self) -> bool:
        return self.raw < other.raw

    def __gt__(self, other: Self) -> bool:
        return self.raw > other.raw

    def __le__(self, other: Self) -> bool:
        return self.raw <= other.raw

    def __ge__(self, other: Self) -> bool:
        return self.raw >= other.raw

    @override
    def __eq__(self, other: Unit | object) -> bool:
        if not isinstance(other, Unit):
            raise NotImplementedError()
        return self.raw == other.raw

    @override
    def __ne__(self, other: Unit | object) -> bool:
        if not isinstance(other, Unit):
            raise NotImplementedError()
        return self.raw != other.raw


class UnitPerUnit[L: Unit, R: Unit](Unit):
    """
    Represents a unit divided by another unit. For example, a very simple
    example of this is meters per second (m/s). If you travel 10 meters in
    2 seconds, then you are traveling at 5 m/s.
    """

    def __init__(self, value: L, per: R) -> None:
        super().__init__(value.raw / per.raw, f"({value.unit} / {per.unit})")
        self.value: L = value
        self.per: R = per

    @override
    def in_current(self) -> float:
        return self.value.in_current() / self.per.in_current()

    @override
    def withval(self, new: float) -> UnitPerUnit[L, R]:
        return UnitPerUnit(self.value.withval(new), self.per.withval(1))

    @override
    def in_base(self) -> UnitPerUnit[L, R]:
        return UnitPerUnit(self.value.in_base(), self.per.in_base())

    def mulr(self, other: R) -> L:
        """
        Used for a typed multiplication of the right-hand unit, this
        function is helpful for dimensional analysis. With the default `muldim`
        function if you multiply (m/s) by seconds, you will get (m/s)*s. Using
        this will yield only meters.

        Note that this converts the displayed unit to the SI base unit.
        """

        return self.value.withval(self.raw * other.raw).in_base()


class UnitUnit[L: Unit, R: Unit](Unit):
    """
    Represents the left unit multiplied by the right unit.
    """

    def __init__(self, value: L, times: R) -> None:
        super().__init__(value.raw * times.raw, f"({value.unit} * {times.unit})")
        self.value: L = value
        self.times: R = times

    @override
    def in_current(self) -> float:
        return self.value.in_current() * self.times.in_current()

    @override
    def withval(self, new: float) -> UnitUnit[L, R]:
        return UnitUnit(self.value.withval(new), self.times.withval(1))

    @override
    def in_base(self) -> UnitUnit[L, R]:
        return UnitUnit(self.value.in_base(), self.times.in_base())

    def divl(self, other: L) -> R:
        """See `UnitPerUnit.mulr` for a similar explanation."""
        return self.times.withval(self.raw / other.raw).in_base()

    def divr(self, other: R) -> L:
        """See `UnitPerUnit.mulr` for a similar explanation."""
        return self.value.withval(self.raw / other.raw).in_base()


class UnitExp[U: Unit](Unit):
    """
    Represents whatever unit is stored inside, raised to some
    exponent.
    """

    def __init__(self, value: U, exp: int) -> None:
        self.value = value
        self.exp = exp
        super().__init__(value.raw, f"{value.unit}^{exp}")

    @override
    def in_current(self) -> float:
        return self.value.in_current()

    @override
    def withval(self, new: float) -> UnitExp[U]:
        return UnitExp(self.value.withval(new), self.exp)

    @override
    def in_base(self) -> UnitExp[U]:
        return UnitExp(self.value.in_base(), self.exp)

    def div(self, other: UnitExp[U]) -> UnitExp[U]:
        """
        Divides the values and subtracts the exponents.
        """
        return UnitExp(self.value / other.value, self.exp - other.exp)

    def mul(self, other: UnitExp[U]) -> UnitExp[U]:
        """
        Multiplies the values and adds the exponents.
        """
        return UnitExp(self.value * other.value, self.exp + other.exp)


class UnitSquared[U: Unit](UnitExp[U]):
    """
    Represents whatever unit is stored inside of it, squared.
    """

    def __init__(self, value: U) -> None:
        super().__init__(value, 2)


class UnitCubed[U: Unit](UnitExp[U]):
    """
    Represents whatever unit is stored inside of it, cubed.
    """

    def __init__(self, value: U) -> None:
        super().__init__(value, 3)


class InverseUnit[U: Unit](UnitExp[U]):
    """
    Represents one divided by whatever unit is stored inside.
    """

    def __init__(self, value: U) -> None:
        super().__init__(value, -1)


class InverseUnitSquared[U: Unit](UnitExp[U]):
    """
    Represents one divided by the square of whatever unit is stored inside.
    """

    def __init__(self, value: U) -> None:
        super().__init__(value, -2)


class InverseUnitCubed[U: Unit](UnitExp[U]):
    """
    Represents one divided by the cube of whatever unit is stored inside.
    """

    def __init__(self, value: U) -> None:
        super().__init__(value, -3)
