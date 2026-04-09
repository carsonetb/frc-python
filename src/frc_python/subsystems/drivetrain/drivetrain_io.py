from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import cast, override

from phoenix6 import BaseStatusSignal
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)

from frc_python.can import CTREDeviceID
from frc_python.subsystems.drivetrain.gyro import Gyro, GyroPigeon, SimGyro
from frc_python.subsystems.drivetrain.module import SimMk5nSwerveModule, SwerveModule
from frc_python.subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from frc_python.units.angle import Angle
from frc_python.units.distance import inches
from frc_python.units.velocity import (
    AngularVelocity,
    feet_per_second,
    radians_per_second,
)
from frc_python.utils.math import Vector2
from frc_python.utils.sim import DriveModuleID, SimulationInfo
from frc_python.utils.swerve import DrivetrainCorner, PerCorner


@autolog
@dataclass
class DrivetrainInputs:
    swerve_states_field_relative: PerCorner[SwerveModuleState] = field(
        default_factory=lambda: PerCorner[SwerveModuleState].generate(
            lambda corner: SwerveModuleState()
        )
    )
    swerve_states: PerCorner[SwerveModuleState] = field(
        default_factory=lambda: PerCorner[SwerveModuleState].generate(
            lambda corner: SwerveModuleState()
        )
    )
    swerve_positions: PerCorner[SwerveModulePosition] = field(
        default_factory=lambda: PerCorner[SwerveModulePosition].generate(
            lambda corner: SwerveModulePosition()
        )
    )
    gyro_rotation: Rotation2d = field(default_factory=lambda: Rotation2d())
    gyro_velocity: AngularVelocity = field(
        default_factory=lambda: radians_per_second(0)
    )
    gyro_connected: bool = field(default_factory=lambda: True)


class DrivetrainIO(ABC):
    FL_POS = Vector2(inches(10.875), inches(10.875))
    FR_POS = Vector2(inches(10.875), inches(-10.875))
    BL_POS = Vector2(inches(-10.875), inches(10.875))
    BR_POS = Vector2(inches(-10.875), inches(-10.875))

    TOP_SPEED = feet_per_second(19.2)

    @property
    @abstractmethod
    def gyro(self) -> Gyro:
        pass

    @property
    @abstractmethod
    def modules(self) -> PerCorner[SwerveModule]:
        pass

    @property
    def desired_states(self) -> PerCorner[SwerveModuleState]:
        return self.modules.map_items(lambda module: module.desired_state)

    @desired_states.setter
    def desired_states(self, value: PerCorner[SwerveModuleState]):
        for module, desired in self.modules.zip_with(value):
            module.desired_state = desired

    @desired_states.setter
    def desired_states(self, val: PerCorner[SwerveModuleState]) -> None:
        for target, module in zip(val.to_list(), self.modules.to_list()):
            module.desired_state = target

    @property
    def odometry_positions(self) -> PerCorner[list[SwerveModulePosition]]:
        return self.modules.map_items(lambda module: module.odometry_positions)

    @property
    def odometry_timestamps(self) -> list[float]:
        return self.modules[DrivetrainCorner.FRONT_LEFT].odometry_timestamps

    @property
    def odometry_yaw_positions(self) -> list[float]:
        return self.gyro.odometry_yaw_positions

    @property
    def valid_timestamps(self) -> int:
        return self.modules[DrivetrainCorner.FRONT_LEFT].valid_timestamps

    @property
    def signals(self) -> list[BaseStatusSignal]:
        signals: list[BaseStatusSignal] = []

        for module in self.modules.to_list():
            signals += module.signals

        signals += self.gyro.signals
        return signals

    @abstractmethod
    def goto_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        pass

    def periodic(self) -> None:
        pass

    def for_each_corner(
        self, inputs: DrivetrainInputs, i: int, module: SwerveModule
    ) -> None:
        module.periodic()
        inputs.swerve_states[i] = module.state
        inputs.swerve_positions[i] = module.position
        inputs.swerve_states_field_relative[i] = SwerveModuleState(
            module.state.speed,
            module.position.angle + self.gyro.yaw.to_rotation2d(),
        )

    def update_inputs(self, inputs: DrivetrainInputs) -> None:
        self.gyro.periodic()
        self.modules.for_each_corner_indexed(
            lambda corner, index, module: self.for_each_corner(inputs, index, module)
        )
        inputs.gyro_rotation = self.gyro.yaw.to_rotation2d()
        inputs.gyro_velocity = self.gyro.yaw_velocity
        inputs.gyro_connected = self.gyro.connected


class DrivetrainIOReal(DrivetrainIO):
    def __init__(
        self, modules: PerCorner[SwerveModule], odometry_thread: PhoenixOdometryThread
    ) -> None:
        self._modules: PerCorner[SwerveModule] = modules
        self._gyro: GyroPigeon = GyroPigeon(
            CTREDeviceID.PIGEON_GYRO.to_pigeon2(), odometry_thread
        )

    @property
    @override
    def gyro(self) -> Gyro:
        return self._gyro

    @property
    @override
    def modules(self) -> PerCorner[SwerveModule]:
        return self._modules

    @override
    def goto_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        return super().goto_chassis_speeds(speeds)


class Mk5nDrivetrainIOSim(DrivetrainIO):
    GYRO_NAME = "chassis_gyro"

    def __init__(self, info: SimulationInfo) -> None:
        self._modules = PerCorner(
            front_left=SimMk5nSwerveModule(info, DriveModuleID.FRONT_LEFT),
            front_right=SimMk5nSwerveModule(info, DriveModuleID.FRONT_RIGHT),
            back_left=SimMk5nSwerveModule(info, DriveModuleID.BACK_LEFT),
            back_right=SimMk5nSwerveModule(info, DriveModuleID.BACK_RIGHT),
        )
        self._gyro = SimGyro(info, self.GYRO_NAME)
        self.kinematics = SwerveDrive4Kinematics(
            self.FL_POS.to_translation2d(),
            self.FR_POS.to_translation2d(),
            self.BL_POS.to_translation2d(),
            self.BR_POS.to_translation2d(),
        )

    @property
    @override
    def gyro(self) -> Gyro:
        return self._gyro

    @property
    @override
    def modules(self) -> PerCorner[SwerveModule]:
        return self._modules.map_items(lambda module: cast(SwerveModule, module))

    @property
    def angle(self) -> Angle:
        return self._gyro.yaw

    @override
    def goto_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        # TODO: Thsi is sped
        if abs(speeds.vx) > 0.1 or abs(speeds.vy) > 0.1 or abs(speeds.omega) > 0.1:
            module_states = self.kinematics.toSwerveModuleStates(speeds)
            module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                module_states, self.TOP_SPEED.meters_per_second()
            )
        else:
            module_states = (
                SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                SwerveModuleState(0, Rotation2d.fromDegrees(90 + 45)),
                SwerveModuleState(0, Rotation2d.fromDegrees(180 - 45)),
                SwerveModuleState(0, Rotation2d.fromDegrees(270 - 45)),
            )

        for i, module in enumerate(self.modules.to_list()):
            module_states[i].optimize(module.state.angle)
            module.desired_state = module_states[i]

    @override
    def periodic(self) -> None:
        for module in self.modules:
            module.periodic()
        self.gyro.periodic()

    @override
    def update_inputs(self, inputs: DrivetrainInputs):
        for i, module in enumerate(self.modules.to_list()):
            module.periodic()
            inputs.swerve_states[i] = module.state
            inputs.swerve_positions[i] = module.position
            inputs.swerve_states_field_relative[i] = SwerveModuleState(
                module.state.speed,
                module.position.angle + self._gyro.yaw.to_rotation2d(),
            )

        inputs.gyro_rotation = self._gyro.yaw.to_rotation2d()
        inputs.gyro_velocity = self._gyro.yaw_velocity
        inputs.gyro_connected = True
