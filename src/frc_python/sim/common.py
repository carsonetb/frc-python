from abc import ABC, ABCMeta, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol, override

from commands2 import Subsystem
from mujoco import MjData, MjModel, mj_name2id, mjtObj
from pykit.loggedrobot import LoggedRobot
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from frc_python.sim.xmlgen import Body, Model
from frc_python.units.angle import Angle, radians
from frc_python.units.force import Newtons, newtons
from frc_python.units.velocity import AngularVelocity, radians_per_second, rotations_per_minute
from frc_python.units.voltage import Voltage, voltage


@dataclass
class SimulationInfo:
    model: MjModel
    data: MjData


class SimulatableRobot(LoggedRobot):
    def __init__(self, info: SimulationInfo | None = None) -> None:
        super().__init__()


# Annoying multiple inheritance rules mean I can't inherit ABC, so this is what
# I do instead.
class SimulatableSubsystem(Subsystem):
    @staticmethod
    def build(model: Model, robot: Body) -> None:
        raise RuntimeError("Subsystem doesn't implement build.")


@dataclass
class MotorID:
    joint_name: str
    motor_name: str


class DriveModuleID(Enum):
    FRONT_LEFT = (
        MotorID("fl_swerve_module_turret_hinge", "fl_swerve_module_turret_motor"),
        MotorID("fl_swerve_module_wheel_hinge", "fl_swerve_module_wheel_motor"),
    )
    FRONT_RIGHT = (
        MotorID("fr_swerve_module_turret_hinge", "fr_swerve_module_turret_motor"),
        MotorID("fr_swerve_module_wheel_hinge", "fr_swerve_module_wheel_motor"),
    )
    BACK_LEFT = (
        MotorID("bl_swerve_module_turret_hinge", "bl_swerve_module_turret_motor"),
        MotorID("bl_swerve_module_wheel_hinge", "bl_swerve_module_wheel_motor"),
    )
    BACK_RIGHT = (
        MotorID("br_swerve_module_turret_hinge", "br_swerve_module_turret_motor"),
        MotorID("br_swerve_module_wheel_hinge", "br_swerve_module_wheel_motor"),
    )

    @property
    def steer(self) -> MotorID:
        return self.value[0]

    @property
    def drive(self) -> MotorID:
        return self.value[1]


class SimMotor(ABC):
    def __init__(self, info: SimulationInfo, id: MotorID, reversed: bool = False) -> None:
        self.direction = -1 if reversed else 1
        self.info = info
        self.joint_id = mj_name2id(self.info.model, mjtObj.mjOBJ_JOINT, id.joint_name)
        self.motor_id: int = mj_name2id(self.info.model, mjtObj.mjOBJ_ACTUATOR, id.motor_name)
        self.qpos_idx: int = self.info.model.jnt_qposadr[self.joint_id]
        self.qvel_idx: int = self.info.model.jnt_dofadr[self.joint_id]

    @property
    def angle(self) -> Angle:
        return radians(self.info.data.qpos[self.qpos_idx] * self.direction)

    @property
    def velocity(self) -> AngularVelocity:
        return radians_per_second(self.info.data.qvel[self.qvel_idx] * self.direction)


class SimKrakenX60(SimMotor):
    FREE_SPEED: AngularVelocity = rotations_per_minute(6000)
    STALL_TORQUE: Newtons = newtons(
        9.2
    )  # Importantly, this is with the Phoenix Pro license. This might also be a lot less because of current limits.
    NOMINAL_VOLTAGE: Voltage = voltage(12)

    STATOR_LIMIT_AMPS = 60.0
    KRAKEN_STALL_AMPS = 414.0
    MAX_TORQUE = STALL_TORQUE.raw * (STATOR_LIMIT_AMPS / KRAKEN_STALL_AMPS)

    def apply_voltage(self, voltage: Voltage) -> None:
        self.info.data.ctrl[self.motor_id] = self._voltage_to_torque(voltage, self.velocity) * self.direction

    # Returns a torque in newton meters, probably should be unit-ed in the future.
    def _voltage_to_torque(self, voltage: Voltage, velocity: AngularVelocity) -> float:
        v_norm = voltage.clamp(-self.NOMINAL_VOLTAGE, self.NOMINAL_VOLTAGE).voltage() / self.NOMINAL_VOLTAGE.voltage()
        omega_norm = velocity.radians_per_second() / self.FREE_SPEED.radians_per_second()

        torque = (v_norm - omega_norm) * self.STALL_TORQUE.raw

        return max(-self.MAX_TORQUE, min(self.MAX_TORQUE, torque))


class SimKrakenX44(SimMotor):
    FREE_SPEED: AngularVelocity = rotations_per_minute(7368)
    STALL_TORQUE: Newtons = newtons(
        5.01
    )  # Importantly, this is with the Phoenix Pro license. This might also be a lot less because of current limits.
    NOMINAL_VOLTAGE: Voltage = voltage(12)

    STATOR_LIMIT_AMPS = 60.0
    KRAKEN_STALL_AMPS = 414.0
    MAX_TORQUE = STALL_TORQUE.raw * (STATOR_LIMIT_AMPS / KRAKEN_STALL_AMPS)

    def apply_voltage(self, voltage: Voltage) -> None:
        self.info.data.ctrl[self.motor_id] = self._voltage_to_torque(voltage, self.velocity) * self.direction

    # Returns a torque in newton meters, probably should be unit-ed in the future.
    def _voltage_to_torque(self, voltage: Voltage, velocity: AngularVelocity) -> float:
        v_norm = voltage.clamp(-self.NOMINAL_VOLTAGE, self.NOMINAL_VOLTAGE).voltage() / self.NOMINAL_VOLTAGE.voltage()
        omega_norm = velocity.radians_per_second() / self.FREE_SPEED.radians_per_second()

        torque = (v_norm - omega_norm) * self.STALL_TORQUE.raw

        return max(-self.MAX_TORQUE, min(self.MAX_TORQUE, torque))


# class SimDrivetrain:
#     # TODO: Actually (rad/s)/rotation
#     ROT_ALIGN_PID = AngularPIDGains(
#         volts_per_radian(80), volts_per_radian_second(0), volt_seconds_per_radian(60)
#     )

#     def __init__(self, info: SimulationInfo) -> None:
#         self.io = Mk5nDrivetrainIOSim(info)
#         self.rot_align_controller = self.ROT_ALIGN_PID.to_controller()
#         self.rot_align_controller.enableContinuousInput(-0.5, 0.5)

#     def periodic(self) -> None:
#         self.io.periodic()

#     def drive(self, velocity: Vector2[LinearVelocity], omega: AngularVelocity) -> None:
#         self.io.goto_chassis_speeds(
#             ChassisSpeeds(
#                 velocity.x.meters_per_second(),
#                 velocity.y.meters_per_second(),
#                 -omega.radians_per_second(),
#             )
#         )

#     def drive_rot_align(
#         self, velocity: Vector2[LinearVelocity], target_omega: Angle
#     ) -> None:
#         self.drive(
#             velocity,
#             radians_per_second(
#                 self.rot_align_controller.calculate(
#                     self.io.angle.rotations(), target_omega.rotations()
#                 )
#             ),
#         )
