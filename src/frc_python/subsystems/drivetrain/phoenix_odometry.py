from logging import Logger
from multiprocessing import Process, Queue, RLock
from multiprocessing.synchronize import RLock as RLockType
from typing import override

from phoenix6 import BaseStatusSignal, StatusSignal
from phoenix6.status_code import StatusCode
from wpilib import RobotController

from units.angle import Angle


class PhoenixOdometryThread(Process):
    def __init__(self, odometry_lock: RLockType) -> None:
        super().__init__(name="PhoenixOdometry")

        self.timestamp_queues: list[Queue[float]] = []
        self.odometry_lock: RLockType = odometry_lock
        self.signals_lock: RLockType = RLock()
        self.phoenix_signals: list[BaseStatusSignal] = []
        self.phoenix_queues: list[Queue[float]] = []
        self.logger: Logger = Logger("PhoenixOdometryThread")

    @override
    def start(self) -> None:
        if len(self.timestamp_queues) > 0:
            super().start()

    @override
    def run(self) -> None:
        while True:
            self.signals_lock.acquire()
            if len(self.phoenix_signals) > 0:
                if (
                    BaseStatusSignal.wait_for_all(2.0 / 250.0, *self.phoenix_signals)
                    == StatusCode.RX_TIMEOUT
                ):
                    self.logger.warning(
                        "Took too long to receive all phoenix odometry signals."
                    )
            self.signals_lock.release()

            self.odometry_lock.acquire()
            timestamp = RobotController.getFPGATime() / 1e6
            latency = 0.0
            for signal in self.phoenix_signals:
                latency += signal.timestamp.get_latency()
            if len(self.phoenix_signals) > 0:
                timestamp -= latency / len(self.phoenix_signals)

            for signal, queue in zip(self.phoenix_signals, self.phoenix_queues):
                queue.put(signal.value_as_double)

            for queue in self.timestamp_queues:
                queue.put(timestamp)
            self.odometry_lock.release()

    def register_signal(self, signal: StatusSignal[float]) -> Queue[float]:
        queue = Queue[float](20)

        self.signals_lock.acquire()
        self.odometry_lock.acquire()

        self.phoenix_signals.append(signal)
        self.phoenix_queues.append(queue)

        self.signals_lock.release()
        self.odometry_lock.release()

        return queue

    def make_timestamp_queue(self) -> Queue[float]:
        queue = Queue[float](20)

        self.odometry_lock.acquire()
        self.timestamp_queues.append(queue)
        self.odometry_lock.release()

        return queue
