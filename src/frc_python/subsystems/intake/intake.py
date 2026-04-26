from typing import override

from commands2 import Command, cmd
from pykit.logger import Logger

from frc_python.sim.common import SimulatableSubsystem, SimulationInfo
from frc_python.sim.xmlgen import Body, Inertial, Mesh, Model
from frc_python.subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from frc_python.subsystems.intake.intake_io import IntakeInputs, IntakeIOReal, IntakeIOSim
from frc_python.units.mass import kilograms
from frc_python.utils.misc import RobotModel


class Intake(SimulatableSubsystem):
    def __init__(self, odometry_thread: PhoenixOdometryThread, info: SimulationInfo | None, model: RobotModel) -> None:
        super().__init__()

        self.inputs = IntakeInputs()
        match model:
            case RobotModel.COMPETITION:
                self.io = IntakeIOReal()
            case RobotModel.SIMULATION:
                assert info is not None
                self.io = IntakeIOSim(info)

    @override
    def periodic(self) -> None:
        self.io.update_inputs(self.inputs)
        Logger.processInputs("Intake", self.inputs)  # pyright: ignore[reportUnknownMemberType]

    def intake(self) -> Command:
        return cmd.startEnd(lambda: self.io.set_speed(0.7), lambda: self.io.set_speed(0.0))

    def outtake(self) -> Command:
        return cmd.startEnd(lambda: self.io.set_speed(-0.5), lambda: self.io.set_speed(0.0))

    @override
    @staticmethod
    def build(model: Model, robot: Body) -> None:
        prefix = "frc_python/resources/subsystem/intake/"

        intake_roller_1_mesh = model.add_mesh("intake_roller_1", "frc_python/resources/subsystems/intake/IntakeRoller1.stl")

        intake_roller_1 = Body("intake_roller_1")
        intake_roller_1.inertials.append(Inertial(kilograms(2.5), diaginertia="0.06 0.06 0.06"))
        intake_roller_1.geoms.append(Mesh(intake_roller_1_mesh, pos="0 0.356 0.07"))
