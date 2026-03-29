import multiprocessing
from enum import Enum, auto
from multiprocessing import RLock
from multiprocessing.synchronize import RLock as RLockType
from pathlib import Path
from typing import Final, override

from commands2 import Command, cmd
from commands2.commandscheduler import CommandScheduler
from phoenix6.canbus import CANBus
from phoenix6.signal_logger import SignalLogger
from phoenix6.status_code import StatusCode
from phoenix6.status_signal_collection import StatusSignalCollection
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter
from wpilib import Alert, DriverStation, PowerDistribution, Preferences

from bindings import configure_bindings
from dashboard import Auto, Dashboard
from subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread


class Model(Enum):
    SIMULATION = auto()
    COMPETITION = auto()


class Robot(LoggedRobot):
    odometry_lock: RLockType = RLock()
    phoenix_thread: PhoenixOdometryThread = PhoenixOdometryThread(odometry_lock)

    def __init__(self) -> None:
        multiprocessing.freeze_support()

        super().__init__()

        self.last_selected_auto: Auto | None = None
        self.auto_command: Command | None = None
        self.model: Model = Model.COMPETITION  # by default

        self.rio_can_bus: Final = CANBus("rio")
        self.canivore: Final = CANBus("*")

        self.status_signals: Final = StatusSignalCollection()

        self.dashboard: Final = Dashboard()

        if self.isSimulation():
            self.model = Model.SIMULATION
        else:
            key = Preferences.getString("Model", "competition")
            if key == "competition":
                self.model = Model.COMPETITION
            else:
                raise RuntimeError(f"Invalid model found in preferences: {key}")

        if (status := SignalLogger.enable_auto_logging(False)) != StatusCode.OK:
            self.logger.warning(f"Failed to disable auto logging ({status.name})")

        DriverStation.silenceJoystickConnectionWarning(self.model != Model.COMPETITION)

        self.configure_subsystems()
        configure_bindings()

    def configure_subsystems(self) -> None:
        pass

    def configure_pykit(self) -> None:
        Logger.recordMetadata("Model", self.model.name)

        if self.isReal():
            Logger.addDataReciever(NT4Publisher(True))
            Logger.addDataReciever(WPILOGWriter())
            if not Path("/U").exists():
                Alert(
                    "The Log USB drive is not connected to the roboRIO, so a match replay will not be saved. After inserted, robot code will need to be restarted.",
                    Alert.AlertType.kWarning,
                ).set(True)

            # Enables power distribution logging.
            _ = PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        else:
            # TODO: Sim logging to file
            Logger.addDataReciever(NT4Publisher(True))

        Logger.start()

    @override
    def robotPeriodic(self) -> None:
        if (status := self.status_signals.refresh_all()) != StatusCode.OK:
            self.logger.error(f"Failed to refresh status signals ({status.name})")

        CommandScheduler.getInstance().run()

    @override
    def disabledPeriodic(self) -> None:
        selected_auto = self.dashboard.auto_chooser.getSelected()
        horizontal_flip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        if self.last_selected_auto != selected_auto:
            self.last_selected_auto = selected_auto
            match selected_auto:
                case Auto.NONE:
                    self.auto_command = cmd.none()
                case _:
                    raise RuntimeError(
                        "Auto chooser returned a non-Auto object, logic error."
                    )

    @override
    def autonomousInit(self) -> None:
        return super().autonomousInit()

    @override
    def autonomousExit(self) -> None:
        return super().autonomousExit()
