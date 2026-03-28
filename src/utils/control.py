from __future__ import annotations

from dataclasses import dataclass

from phoenix6.configs import Slot0Configs, SlotConfigs
from wpimath.controller import (
    PIDController,
    ProfiledPIDController,
    SimpleMotorFeedforwardMeters,
    SimpleMotorFeedforwardRadians,
)
from wpimath.trajectory import TrapezoidProfile

from units.voltage import (
    Voltage,
    VoltagePerAngle,
    VoltagePerAngleTime,
    VoltagePerDistance,
    VoltagePerDistanceTime,
    VoltageTimePerAngle,
    VoltageTimePerDistance,
    VoltageTimeSquaredPerAngle,
    VoltageTimeSquaredPerDistance,
    volt_seconds_per_meter,
    volt_seconds_per_radian,
    volt_seconds_squared_per_meter,
    volt_seconds_squared_per_radian,
    voltage,
    volts_per_meter,
    volts_per_meter_second,
    volts_per_radian,
    volts_per_radian_second,
)


@dataclass
class LinearMotorFFGains:
    s: Voltage = voltage(0)
    v: VoltageTimePerDistance = volt_seconds_per_meter(0)
    a: VoltageTimeSquaredPerDistance = volt_seconds_squared_per_meter(0)

    def slot_with(self, slot: Slot0Configs) -> Slot0Configs:
        return slot.with_k_s(self.s.raw).with_k_v(self.s.raw).with_k_a(self.a.raw)

    def to_feedforward(self) -> SimpleMotorFeedforwardMeters:
        return SimpleMotorFeedforwardMeters(float(self.s), float(self.v), float(self.a))

    @staticmethod
    def from_ff(ff: SimpleMotorFeedforwardMeters) -> LinearMotorFFGains:
        return LinearMotorFFGains(
            voltage(ff.getKs()),
            volt_seconds_per_meter(ff.getKv()),
            volt_seconds_squared_per_meter(ff.getKa()),
        )

    @staticmethod
    def from_slot0(slot: Slot0Configs) -> LinearMotorFFGains:
        return LinearMotorFFGains(
            voltage(slot.k_s),
            volt_seconds_per_meter(slot.k_v),
            volt_seconds_squared_per_meter(slot.k_a),
        )


@dataclass
class AngularMotorFFGains:
    s: Voltage = voltage(0)
    v: VoltageTimePerAngle = volt_seconds_per_radian(0)
    a: VoltageTimeSquaredPerAngle = volt_seconds_squared_per_radian(0)

    def to_feedforward(self) -> SimpleMotorFeedforwardRadians:
        return SimpleMotorFeedforwardRadians(
            float(self.s), float(self.v), float(self.a)
        )

    @staticmethod
    def from_ff(ff: SimpleMotorFeedforwardRadians) -> AngularMotorFFGains:
        return AngularMotorFFGains(
            voltage(ff.getKs()),
            volt_seconds_per_radian(ff.getKv()),
            volt_seconds_squared_per_radian(ff.getKa()),
        )

    @staticmethod
    def from_slot0(slot: Slot0Configs) -> AngularMotorFFGains:
        return AngularMotorFFGains(
            voltage(slot.k_s),
            volt_seconds_per_radian(slot.k_v),
            volt_seconds_squared_per_radian(slot.k_a),
        )


@dataclass
class LinearPIDGains:
    p: VoltagePerDistance = volts_per_meter(0)
    i: VoltagePerDistanceTime = volts_per_meter_second(0)
    d: VoltageTimePerDistance = volt_seconds_per_meter(0)

    def to_controller(self) -> PIDController:
        return PIDController(self.p, self.i, self.d)

    def to_profiled(
        self, constraints: TrapezoidProfile.Constraints
    ) -> ProfiledPIDController:
        return ProfiledPIDController(self.p, self.i, self.d, constraints)

    def slot_with(self, slot: Slot0Configs) -> Slot0Configs:
        return (
            slot.with_k_p(float(self.p)).with_k_i(float(self.i)).with_k_d(float(self.d))
        )

    @staticmethod
    def from_controller(pid: PIDController) -> LinearPIDGains:
        return LinearPIDGains(
            volts_per_meter(pid.getP()),
            volts_per_meter_second(pid.getI()),
            volt_seconds_per_meter(pid.getD()),
        )

    @staticmethod
    def from_slot0(slot: Slot0Configs) -> LinearPIDGains:
        return LinearPIDGains(
            volts_per_meter(slot.k_p),
            volts_per_meter_second(slot.k_i),
            volt_seconds_per_meter(slot.k_d),
        )


@dataclass
class AngularPIDGains:
    p: VoltagePerAngle = volts_per_radian(0)
    i: VoltagePerAngleTime = volts_per_radian_second(0)
    d: VoltageTimePerAngle = volt_seconds_per_radian(0)

    def to_controller(self) -> PIDController:
        return PIDController(self.p, self.i, self.d)

    def to_profiled(
        self, constraints: TrapezoidProfile.Constraints
    ) -> ProfiledPIDController:
        return ProfiledPIDController(self.p, self.i, self.d, constraints)

    def slot_with(self, slot: Slot0Configs) -> Slot0Configs:
        return (
            slot.with_k_p(float(self.p)).with_k_i(float(self.i)).with_k_d(float(self.d))
        )

    @staticmethod
    def from_controller(pid: PIDController) -> AngularPIDGains:
        return AngularPIDGains(
            volts_per_radian(pid.getP()),
            volts_per_radian_second(pid.getI()),
            volt_seconds_per_radian(pid.getD()),
        )

    @staticmethod
    def from_slot0(slot: Slot0Configs) -> AngularPIDGains:
        return AngularPIDGains(
            volts_per_radian(slot.k_p),
            volts_per_radian_second(slot.k_i),
            volt_seconds_per_radian(slot.k_d),
        )
