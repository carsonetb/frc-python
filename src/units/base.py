from __future__ import annotations

from abc import ABC, abstractmethod
from copy import deepcopy
from functools import singledispatchmethod
from typing import Self, override

from pykit.autolog import autolog_output, autologgable_output
from pykit.logtable import LogTable


@autologgable_output
class Unit(ABC):
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
        pass

    @abstractmethod
    def withval(self, new: float) -> Self:
        pass

    def __float__(self) -> float:
        return self.raw

    @override
    def __str__(self) -> str:
        return f"{self.in_current()} {self.unit}"

    @override
    def __repr__(self) -> str:
        return f"{self.in_current()} {self.unit}"

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

    def muldim(self, other: R) -> L:
        return self.value.withval(self.raw * other.raw)


class UnitUnit[L: Unit, R: Unit](Unit):
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

    def div_leftdim(self, other: L) -> R:
        return self.times.withval(self.raw / other.raw)

    def div_rightdim(self, other: R) -> L:
        return self.value.withval(self.raw / other.raw)


class UnitSquared[U: Unit](Unit):
    def __init__(self, value: U) -> None:
        super().__init__(value.raw, f"{value.unit}^2")
        self.value: U = value

    @override
    def in_current(self) -> float:
        return self.value.in_current()

    @override
    def withval(self, new: float) -> UnitSquared[U]:
        return UnitSquared(self.value.withval(new))
