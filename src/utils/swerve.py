from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from pykit.autolog import autolog
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState

from units.temperature import Temperature
from units.velocity import AngularVelocity, LinearVelocity, mps, radps


class DrivetrainCorner(Enum):
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4


@dataclass
class Corner:
    position: Pose2d
    magnet_offset: float


@autolog
@dataclass
class PerCorner[T]:
    front_left: T
    front_right: T
    back_left: T
    back_right: T

    def __getitem__(self, corner: DrivetrainCorner | int) -> T:
        if isinstance(corner, DrivetrainCorner):
            return self[corner.value]
        else:
            match corner:
                case 0:
                    return self.front_left
                case 1:
                    return self.front_right
                case 2:
                    return self.back_left
                case 3:
                    return self.back_right
                case _:
                    raise IndexError("PerCorner only has 4 corners.")

    def __setitem__(self, corner: DrivetrainCorner | int, value: T):
        if isinstance(corner, DrivetrainCorner):
            return self[corner.value]
        else:
            match corner:
                case 0:
                    self.front_left = value
                case 1:
                    self.front_right = value
                case 2:
                    self.back_left = value
                case 3:
                    self.back_right = value
                case _:
                    raise IndexError("PerCorner only has 4 corners.")

    def to_list(self) -> list[T]:
        return list(self.to_tuple())

    def to_tuple(self) -> tuple[T, T, T, T]:
        return (self.front_left, self.front_right, self.back_left, self.back_right)

    def for_each_corner_indexed(
        self, block: Callable[[DrivetrainCorner, int, T], None]
    ):
        block(DrivetrainCorner.FRONT_LEFT, 0, self.front_left)
        block(DrivetrainCorner.FRONT_RIGHT, 1, self.front_right)
        block(DrivetrainCorner.BACK_LEFT, 2, self.back_left)
        block(DrivetrainCorner.BACK_RIGHT, 3, self.back_right)

    def map_items[O](self, fun: Callable[[T], O]) -> PerCorner[O]:
        return PerCorner(
            fun(self.front_left),
            fun(self.front_right),
            fun(self.back_left),
            fun(self.back_right),
        )

    @staticmethod
    def generate(block: Callable[[DrivetrainCorner], T]) -> PerCorner[T]:
        return PerCorner(
            block(DrivetrainCorner.FRONT_LEFT),
            block(DrivetrainCorner.FRONT_RIGHT),
            block(DrivetrainCorner.BACK_LEFT),
            block(DrivetrainCorner.BACK_RIGHT),
        )

    @staticmethod
    def from_sequence(seq: Sequence[T]) -> PerCorner[T]:
        return PerCorner(seq[0], seq[1], seq[2], seq[3])


@autolog
@dataclass
class SwerveModuleTemp:
    driving_temp: Temperature
    turning_temp: Temperature


class SwerveDriveKinematicsExt(SwerveDrive4Kinematics):
    def to_corner_swerve_module_states(
        self, speeds: ChassisSpeeds
    ) -> PerCorner[SwerveModuleState]:
        return PerCorner[SwerveModuleState].from_sequence(
            self.toSwerveModuleStates(speeds)
        )

    def corner_states_to_speeds(
        self, states: PerCorner[SwerveModuleState]
    ) -> ChassisSpeeds:
        return self.toChassisSpeeds(states.to_tuple())


class SwerveModuleStateExt(SwerveModuleState):
    @property
    def speed_units(self) -> LinearVelocity:
        return mps(self.speed)

    @property
    def translation2d_per_second(self) -> Translation2d:
        return Translation2d(self.speed, self.angle)


class ChassisSpeedsExt(ChassisSpeeds):
    @property
    def angular_velocity(self) -> AngularVelocity:
        return radps(self.omega)

    @property
    def translation2d_per_second(self) -> Translation2d:
        return Translation2d(self.vx, self.vy)
