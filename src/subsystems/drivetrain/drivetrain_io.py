from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import override

from phoenix6 import BaseStatusSignal
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from can import CTREDeviceID
from subsystems.drivetrain.gyro import Gyro, GyroPigeon
from subsystems.drivetrain.module import SwerveModule
from subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from units.velocity import AngularVelocity, radians_per_second
from utils.swerve import DrivetrainCorner, PerCorner


@autolog
@dataclass
class DrivetrainInputs:
    swerve_states_field_relative: PerCorner[SwerveModuleState] = PerCorner[
        SwerveModuleState
    ].generate(lambda corner: SwerveModuleState())
    swerve_states: PerCorner[SwerveModuleState] = PerCorner[SwerveModuleState].generate(
        lambda corner: SwerveModuleState()
    )
    swerve_positions: PerCorner[SwerveModulePosition] = PerCorner[
        SwerveModulePosition
    ].generate(lambda corner: SwerveModulePosition())
    gyro_rotation: Rotation2d = Rotation2d()
    gyro_velocity: AngularVelocity = radians_per_second(0)
    gyro_connected: bool = True


class DrivetrainIO(ABC):
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

    def for_each_corner(
        self, inputs: DrivetrainInputs, i: int, module: SwerveModule
    ) -> None:
        module.periodic()
        inputs.swerve_states[i] = module.state
        inputs.swerve_positions[i] = module.position
        inputs.swerve_states_field_relative[i] = SwerveModuleState(
            module.state.speed,
            module.position.angle + self.gyro.rotation.to_rotation2d(),
        )

    def update_inputs(self, inputs: DrivetrainInputs) -> None:
        self.gyro.periodic()
        self.modules.for_each_corner_indexed(
            lambda corner, index, module: self.for_each_corner(inputs, index, module)
        )
        inputs.gyro_rotation = self.gyro.rotation.to_rotation2d()
        inputs.gyro_velocity = self.gyro.velocity
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
