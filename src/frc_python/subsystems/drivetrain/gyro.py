from abc import ABC, abstractmethod
from copy import deepcopy
from multiprocessing.queues import Queue
from typing import override

from mujoco import mj_name2id, mjtObj
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.hardware import Pigeon2
from phoenix6.status_signal import StatusSignal
from phoenix6.units import degree

from frc_python.subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from frc_python.units.angle import Angle, degrees, radians
from frc_python.units.time import seconds
from frc_python.units.velocity import (
    AngularVelocity,
    degrees_per_second,
    radians_per_second,
)
from frc_python.utils.misc import TIMESTEP
from frc_python.utils.sim import SimulationInfo


class Gyro(ABC):
    @property
    @abstractmethod
    def yaw(self) -> Angle:
        pass

    @property
    @abstractmethod
    def yaw_velocity(self) -> AngularVelocity:
        pass

    @property
    @abstractmethod
    def connected(self) -> bool:
        pass

    @property
    @abstractmethod
    def odometry_yaw_positions(self) -> list[float]:
        pass

    @property
    @abstractmethod
    def odometry_yaw_timestamps(self) -> list[float]:
        pass

    @property
    def signals(self) -> list[BaseStatusSignal]:
        return []

    @abstractmethod
    def periodic(self) -> None:
        pass

    @abstractmethod
    def zero(self) -> None:
        pass


class GyroPigeon(Gyro):
    def __init__(self, pigeon: Pigeon2, odometry_thread: PhoenixOdometryThread) -> None:
        super().__init__()
        self.pigeon: Pigeon2 = pigeon
        self.yaw_signal: StatusSignal[degree] = pigeon.get_yaw()
        self.pitch_signal: StatusSignal[degree] = pigeon.get_pitch()
        self.roll_signal: StatusSignal[degree] = pigeon.get_roll()
        self.angular_velocity_signal: StatusSignal[float] = (
            pigeon.get_angular_velocity_z_world()
        )
        self.yaw_timestamp_queue: Queue[float] = odometry_thread.make_timestamp_queue()
        self.yaw_position_queue: Queue[float] = odometry_thread.register_signal(
            self.yaw_signal
        )
        self._odometry_yaw_timestamps: list[float] = []
        self._odometry_yaw_positions: list[float] = []

        _ = BaseStatusSignal.set_update_frequency_for_all(250.0, self.yaw_signal)
        _ = BaseStatusSignal.set_update_frequency_for_all(
            100.0, self.pitch_signal, self.roll_signal, self.angular_velocity_signal
        )

    @property
    @override
    def yaw(self) -> Angle:
        return degrees(self.yaw_signal.value_as_double)

    @property
    @override
    def yaw_velocity(self) -> AngularVelocity:
        return degrees_per_second(self.angular_velocity_signal.value_as_double)

    @property
    @override
    def connected(self) -> bool:
        return self.yaw_signal.status.is_ok()

    @property
    @override
    def odometry_yaw_timestamps(self) -> list[float]:
        return self._odometry_yaw_timestamps

    @property
    @override
    def odometry_yaw_positions(self) -> list[float]:
        return self._odometry_yaw_positions

    @property
    @override
    def signals(self) -> list[BaseStatusSignal]:
        return [
            self.yaw_signal,
            self.pitch_signal,
            self.roll_signal,
            self.angular_velocity_signal,
        ]

    @override
    def periodic(self) -> None:
        self._odometry_yaw_timestamps = []
        while not self.yaw_timestamp_queue.empty():
            self._odometry_yaw_timestamps.append(self.yaw_timestamp_queue.get())
        self._odometry_yaw_positions = []
        while not self.yaw_position_queue.empty():
            self._odometry_yaw_positions.append(self.yaw_position_queue.get())

    @override
    def zero(self) -> None:
        self.pigeon.set_yaw(0)


class SimGyro(Gyro):
    def __init__(self, info: SimulationInfo, name: str) -> None:
        self.info = info
        self.sensor_id: int = self.info.model.sensor_adr[
            mj_name2id(self.info.model, mjtObj.mjOBJ_SENSOR, name)
        ]
        self._yaw = radians(0)

    @property
    @override
    def connected(self) -> bool:
        return True

    @property
    @override
    def odometry_yaw_positions(self) -> list[float]:
        return []

    @property
    @override
    def odometry_yaw_timestamps(self) -> list[float]:
        return []

    @property
    @override
    def yaw(self) -> Angle:
        return self._yaw

    @property
    @override
    def yaw_velocity(self) -> AngularVelocity:
        return radians_per_second(self.info.data.sensordata[self.sensor_id + 2])  # pyright: ignore[reportAny]

    @override
    def periodic(self) -> None:
        self._yaw += self.yaw_velocity.muldim(TIMESTEP)

    @override
    def zero(self) -> None:
        self._yaw = radians(0)
