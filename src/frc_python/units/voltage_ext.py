from __future__ import annotations

from typing import override

from frc_python.units.angle import Angle, AngleTime, radian_seconds, radians
from frc_python.units.base import (
    UnitPerUnit,
    UnitUnit,
)
from frc_python.units.distance import Distance, DistanceTime, meter_seconds, meters
from frc_python.units.electrical import Voltage, voltage
from frc_python.units.time import Time, TimeSquared, seconds

VoltagePerDistance = UnitPerUnit[Voltage, Distance]
"""Stored raw in units of v/m"""
VoltagePerAngle = UnitPerUnit[Voltage, Angle]
"""Stored raw in units of v/rad"""
VoltagePerDistanceTime = UnitPerUnit[Voltage, DistanceTime]
"""Stored raw in units of v/(ms)"""
VoltagePerAngleTime = UnitPerUnit[Voltage, AngleTime]
"""Stored raw in units of v/(rad s)"""
VoltageTime = UnitUnit[Voltage, Time]
"""Stored raw in units of vs"""
VoltageTimeSquared = UnitUnit[Voltage, TimeSquared]
"""Stored raw in units of vs²"""
VoltageTimePerDistance = UnitPerUnit[VoltageTime, Distance]
"""Stored raw in units of (vs)/m"""
VoltageTimePerAngle = UnitPerUnit[VoltageTime, Angle]
"""Stored raw in units of (vs)/rad"""
VoltageTimeSquaredPerDistance = UnitPerUnit[VoltageTimeSquared, Distance]
"""Stored raw in units of (vs²)/m"""
VoltageTimeSquaredPerAngle = UnitPerUnit[VoltageTimeSquared, Angle]
"""Stored raw in units of (vs²)/rad"""


def volts_per_meter(val: float) -> VoltagePerDistance:
    return VoltagePerDistance(voltage(val), meters(1))


def volts_per_radian(val: float) -> VoltagePerAngle:
    return VoltagePerAngle(voltage(val), radians(1))


def volts_per_meter_second(val: float) -> VoltagePerDistanceTime:
    return VoltagePerDistanceTime(voltage(val), meter_seconds(1))


def volts_per_radian_second(val: float) -> VoltagePerAngleTime:
    return VoltagePerAngleTime(voltage(val), radian_seconds(1))


def volt_seconds(val: float) -> VoltageTime:
    return VoltageTime(voltage(val), seconds(1))


def volt_seconds_squared(val: float) -> VoltageTimeSquared:
    return VoltageTimeSquared(voltage(val), TimeSquared(seconds(1)))


def volt_seconds_per_meter(val: float) -> VoltageTimePerDistance:
    return VoltageTimePerDistance(volt_seconds(val), meters(1))


def volt_seconds_per_radian(val: float) -> VoltageTimePerAngle:
    return VoltageTimePerAngle(volt_seconds(val), radians(1))


def volt_seconds_squared_per_meter(val: float) -> VoltageTimeSquaredPerDistance:
    return VoltageTimeSquaredPerDistance(volt_seconds_squared(val), meters(1))


def volt_seconds_squared_per_radian(val: float) -> VoltageTimeSquaredPerAngle:
    return VoltageTimeSquaredPerAngle(volt_seconds_squared(val), radians(1))
