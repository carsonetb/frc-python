from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import override

from pykit.autolog import autolog

from frc_python.can import CTREDeviceID, SIMDeviceID
from frc_python.sim.common import SimKrakenX60, SimulationInfo
from frc_python.units.angle import Angle, degrees, rotations
from frc_python.units.velocity import AngularVelocity, radians_per_second, rotations_per_second
from frc_python.units.voltage import voltage


@autolog
@dataclass
class IntakeInputs:
    position: Angle = field(default_factory=lambda: degrees(0))
    velocity: AngularVelocity = field(default_factory=lambda: radians_per_second(0))


class IntakeIO(ABC):
    @abstractmethod
    def set_speed(self, to: float) -> None:
        pass

    @abstractmethod
    def update_inputs(self, inputs: IntakeInputs) -> None:
        pass


class IntakeIOReal(IntakeIO):
    def __init__(self) -> None:
        self.motor = CTREDeviceID.INTAKE_ROLLER.to_talonfx()

    @override
    def update_inputs(self, inputs: IntakeInputs) -> None:
        inputs.position = rotations(self.motor.get_position().value)
        inputs.velocity = rotations_per_second(self.motor.get_velocity().value)

    @override
    def set_speed(self, to: float) -> None:
        self.motor.set(to)


class IntakeIOSim(IntakeIO):
    def __init__(self, info: SimulationInfo) -> None:
        self.motor = SimKrakenX60(info, SIMDeviceID.INTAKE_ROLLER.value)

    @override
    def update_inputs(self, inputs: IntakeInputs) -> None:
        inputs.position = self.motor.angle
        inputs.velocity = self.motor.velocity

    @override
    def set_speed(self, to: float) -> None:
        self.motor.apply_voltage(voltage(to * 12))
