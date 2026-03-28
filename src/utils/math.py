from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, sqrt
from typing import override

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from units.base import Unit


def t2d_from_polar(magnitude: float, angle: float) -> Translation2d:
    return Translation2d(magnitude * cos(angle), magnitude * sin(angle))


def t2d_dot(left: Translation2d, right: Translation2d) -> float:
    return left.x * right.x + left.y * right.y


@dataclass
class Vector2[U: Unit]:
    x: U
    y: U

    def to_pose2d(self) -> Pose2d:
        """
        Returns this vector as a Pose2d with zero rotation, in the
        raw units, which is probably meters or m/s.
        """

        return Pose2d(Translation2d(self.x.raw, self.y.raw), Rotation2d())

    def normalized(self) -> Vector2[U]:
        return self / self.length()

    def length(self) -> U:
        return self.x.withval(sqrt(self.x * self.x + self.y * self.y))

    def __add__(self, other: Vector2[U]) -> Vector2[U]:
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2[U]) -> Vector2[U]:
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other: Vector2[U] | float) -> Vector2[U]:
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x, self.y + other.y)
        return Vector2(self.x + other, self.y + other)

    def __truediv__(self, other: Vector2[U] | float) -> Vector2[U]:
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x, self.y + other.y)
        return Vector2(self.x + other, self.y + other)


@dataclass
class Vector3[U: Unit]:
    x: U
    y: U
    z: U

    def normalized(self) -> Vector3[U]:
        return self / self.length()

    def length(self) -> U:
        return self.x.withval(sqrt(self.x * self.x + self.y * self.y + self.z + self.z))

    def __add__(self, other: Vector3[U]) -> Vector3[U]:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3[U]) -> Vector3[U]:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: Vector3[U] | float) -> Vector3[U]:
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        return Vector3(self.x + other, self.y + other, self.z + other)

    def __truediv__(self, other: Vector3[U] | float) -> Vector3[U]:
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        return Vector3(self.x + other, self.y + other, self.z + other)
