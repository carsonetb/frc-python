from __future__ import annotations

from typing import override

from frc_python.units.base import InverseUnit, UnitPerUnit, UnitSquared, UnitUnit
from frc_python.units.current import Current, amps
from frc_python.units.distance import Distance, meters
from frc_python.units.mass import Mass, kilograms
from frc_python.units.time import Time, seconds


class Frequency(InverseUnit[Time]):
    """Stored raw in units of s^(-1)"""

    def __init__(self, value: Time) -> None:
        super().__init__(value)
        self.unit = "hertz"


class Charge(UnitUnit[Time, Current]):
    """Stored raw in coulombs, or, s*A."""

    def __init__(self, value: Time, times: Current) -> None:
        super().__init__(value, times)
        self.unit = "coulombs"


class Energy(UnitUnit[Mass, UnitPerUnit[UnitSquared[Distance], UnitSquared[Time]]]):
    """Stored raw in units of kg * m^2/s^2"""

    def __init__(
        self, value: Mass, times: UnitPerUnit[UnitSquared[Distance], UnitSquared[Time]]
    ) -> None:
        super().__init__(value, times)
        self.unit = "joules"

    @override
    def in_current(self) -> float:
        return self.raw


class Power(UnitPerUnit[Energy, Time]):
    """Stored raw in units of Joules per second, or (kg * m^2/s^3)"""

    def __init__(self, value: Energy, per: Time) -> None:
        super().__init__(value, per)
        self.unit = "watts"

    @override
    def in_current(self) -> float:
        return self.raw


class Voltage(UnitPerUnit[Power, Current]):
    """
    Basic unit of voltage, can only be stored in voltage.
    Stored in units of power per current.
    """

    def __init__(self, value: Power, per: Current) -> None:
        super().__init__(value, per)
        self.unit = "voltage"

    @override
    def in_current(self) -> float:
        return self.raw

    @property
    def voltage(self) -> float:
        return self.raw


class Resistance(UnitPerUnit[Voltage, Current]):
    """
    Stored raw in ohms, Volts per ampere, or kg*m^2*s^(-3)*A^(-2).
    Resistance is the inverse of Conductance.
    """

    def __init__(self, value: Voltage, per: Current) -> None:
        super().__init__(value, per)
        self.unit = "ohms"

    @override
    def in_current(self) -> float:
        return self.raw


class Conductance(UnitPerUnit[Current, Voltage]):
    """
    Stored raw in siemens, amperes per Volt, or kg^(-1)*m^(-2)*s^3*A^2.
    Conductance is the inverse of Resistance.
    """

    def __init__(self, value: Current, per: Voltage) -> None:
        super().__init__(value, per)
        self.unit = "siemens"

    @override
    def in_current(self) -> float:
        return self.raw


class Capacitance(UnitPerUnit[Charge, Voltage]):
    """
    Stored raw in farads, coulombs per Volt, or kg^(-1)*m^(-2)*s^4*A^2
    """

    def __init__(self, value: Charge, per: Voltage) -> None:
        super().__init__(value, per)
        self.unit = "farads"

    @override
    def in_current(self) -> float:
        return self.raw


class MagneticFlux(UnitUnit[Voltage, Time]):
    """
    Stored raw in webers, Volt seconds, or kg*m^2*s^(-2)*A^(-1).
    I will admit I don't really know what this is.
    """

    def __init__(self, value: Voltage, times: Time) -> None:
        super().__init__(value, times)
        self.unit = "webers"

    @override
    def in_current(self) -> float:
        return self.raw


class Inductance(UnitPerUnit[MagneticFlux, Current]):
    """
    Stored raw in henries, webers per ampere, or kg*m^2*s^(-2)*A^(-2).
    """

    def __init__(self, value: MagneticFlux, per: Current) -> None:
        super().__init__(value, per)
        self.unit = "henries"

    @override
    def in_current(self) -> float:
        return self.raw


def hertz(val: float) -> Frequency:
    return Frequency(seconds(val))


def coulombs(val: float) -> Charge:
    return Charge(seconds(1), amps(val))


def joules(val: float) -> Energy:
    return Energy(
        kilograms(val), UnitPerUnit(UnitSquared(meters(1)), UnitSquared(seconds(1)))
    )


def watts(val: float) -> Power:
    return Power(joules(val), seconds(1))


def ohms(val: float) -> Resistance:
    return Resistance(voltage(val), amps(1))


def siemens(val: float) -> Conductance:
    return Conductance(amps(val), voltage(1))


def farads(val: float) -> Capacitance:
    return Capacitance(coulombs(val), voltage(1))


def webers(val: float) -> MagneticFlux:
    return MagneticFlux(voltage(val), seconds(1))


def henries(val: float) -> Inductance:
    return Inductance(webers(val), amps(1))


def voltage(val: float) -> Voltage:
    return Voltage(watts(val), amps(1))
