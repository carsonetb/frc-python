from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, sqrt

from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Translation2d,
    Translation3d,
)

from frc_python.units.base import Unit


def sign(x: float) -> int:
    if x > 0:
        return 1
    if x < 0:
        return -1
    else:
        return 0


def t2d_from_polar(magnitude: float, angle: float) -> Translation2d:
    return Translation2d(magnitude * cos(angle), magnitude * sin(angle))


def t2d_dot(left: Translation2d, right: Translation2d) -> float:
    return left.x * right.x + left.y * right.y


@dataclass
class Vector2[U: Unit]:
    x: U
    y: U

    def to_translation2d(self) -> Translation2d:
        """
        Converts this Vector2 to a Translation2d using the raw, probably SI or
        SI derived, value.
        """

        return Translation2d(self.x.raw, self.y.raw)

    def to_pose2d(self) -> Pose2d:
        """
        Returns this vector as a Pose2d with zero rotation, in the
        raw units, which is probably meters or m/s.
        """

        return Pose2d(self.to_translation2d(), Rotation2d())

    def normalized(self) -> Vector2[U]:
        return self / self.length

    @property
    def length(self) -> U:
        return self.x.withval(sqrt(self.x * self.x + self.y * self.y))

    def mulratio(self, other: float) -> Vector2[U]:
        return Vector2(self.x.mulratio(other), self.y.mulratio(other))

    def divratio(self, other: float) -> Vector2[U]:
        return self.mulratio(1 / other)

    def __add__(self, other: Vector2[U]) -> Vector2[U]:
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2[U]) -> Vector2[U]:
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other: Vector2[U] | U) -> Vector2[U]:
        if isinstance(other, Vector2):
            return Vector2(self.x * other.x, self.y * other.y)
        return Vector2(self.x * other, self.y * other)

    def __truediv__(self, other: Vector2[U] | U) -> Vector2[U]:
        if isinstance(other, Vector2):
            return Vector2(self.x / other.x, self.y / other.y)
        return Vector2(self.x / other, self.y / other)


@dataclass
class Vector3[U: Unit]:
    x: U
    y: U
    z: U

    def to_translation3d(self) -> Translation3d:
        """
        Converts this Vector3 to a Translation3d using the raw, probably SI or
        SI derived, value.
        """

        return Translation3d(self.x.raw, self.y.raw, self.z.raw)

    def to_pose3d(self) -> Pose3d:
        """
        Returns this vector as a Pose2d with zero rotation, in the
        raw units, which is probably meters or m/s.
        """

        return Pose3d(self.to_translation3d(), Rotation3d())

    def normalized(self) -> Vector3[U]:
        return self / self.length

    @property
    def length(self) -> U:
        return self.x.withval(sqrt(self.x * self.x + self.y * self.y + self.z + self.z))

    def mulratio(self, other: float) -> Vector3[U]:
        return Vector3(
            self.x.mulratio(other), self.y.mulratio(other), self.z.mulratio(other)
        )

    def divratio(self, other: float) -> Vector3[U]:
        return self.mulratio(1 / other)

    def __add__(self, other: Vector3[U]) -> Vector3[U]:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3[U]) -> Vector3[U]:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: Vector3[U] | U) -> Vector3[U]:
        if isinstance(other, Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        return Vector3(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other: Vector3[U] | U) -> Vector3[U]:
        if isinstance(other, Vector3):
            return Vector3(self.x / other.x, self.y / other.y, self.z / other.z)
        return Vector3(self.x / other, self.y / other, self.z / other)
