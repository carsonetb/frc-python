from enum import Enum

from phoenix6 import CANBus
from phoenix6.hardware import CANcoder, Pigeon2, TalonFX

from frc_python.sim.common import MotorID

_canivore_bus = CANBus("*")


class SIMDeviceID(Enum):
    INTAKE_ROLLER = MotorID("intake_roller_1_hinge", "intake_roller_1_motor")


class CTREDeviceID(Enum):
    FRONT_LEFT_DRIVE_MOTOR = (1, _canivore_bus)
    BACK_LEFT_DRIVE_MOTOR = (2, _canivore_bus)
    BACK_RIGHT_DRIVE_MOTOR = (3, _canivore_bus)
    FRONT_RIGHT_DRIVE_MOTOR = (4, _canivore_bus)

    FRONT_LEFT_TURN_MOTOR = (5, _canivore_bus)
    BACK_LEFT_TURN_MOTOR = (6, _canivore_bus)
    BACK_RIGHT_TURN_MOTOR = (7, _canivore_bus)
    FRONT_RIGHT_TURN_MOTOR = (8, _canivore_bus)

    FRONT_LEFT_TURN_ENCODER = (9, _canivore_bus)
    BACK_LEFT_TURN_ENCODER = (10, _canivore_bus)
    BACK_RIGHT_TURN_ENCODER = (11, _canivore_bus)
    FRONT_RIGHT_TURN_ENCODER = (12, _canivore_bus)

    PIGEON_GYRO = (20, _canivore_bus)

    INTAKE_ROLLER = (30, _canivore_bus)

    @property
    def num(self) -> int:
        return self.value[0]

    @property
    def bus(self) -> CANBus:
        return self.value[1]

    def to_cancoder(self) -> CANcoder:
        return CANcoder(self.num, self.bus)

    def to_talonfx(self) -> TalonFX:
        return TalonFX(self.num, self.bus)

    def to_pigeon2(self) -> Pigeon2:
        return Pigeon2(self.num, self.bus)
