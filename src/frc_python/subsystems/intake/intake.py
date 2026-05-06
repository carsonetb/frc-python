from typing import override

from commands2 import Command, cmd
from pykit.logger import Logger

from frc_python.sim.common import SimulatableSubsystem, SimulationInfo
from frc_python.sim.xmlgen import Body, Cylinder, DCMotor, Equality, Geom, Inertial, Joint, Mesh, Model
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
        return cmd.startEnd(lambda: self.io.set_speed(1), lambda: self.io.set_speed(0.0), self)

    def outtake(self) -> Command:
        return cmd.startEnd(lambda: self.io.set_speed(-0.5), lambda: self.io.set_speed(0.0), self)

    @override
    @staticmethod
    def build(model: Model, robot: Body) -> None:
        prefix = "frc_python/resources/subsystems/intake/"

        crescendo_offset = "0 0.356 0.07"

        roller_1_mesh = model.add_mesh("intake_roller_1", prefix + "IntakeRoller1.stl")

        roller_1 = Body("intake_roller_1", pos="0 0.25 0.026")
        roller_1.inertials.append(Inertial(kilograms(2.5), diaginertia="0.06 0.06 0.06"))
        roller_1.geoms.append(Mesh(roller_1_mesh, pos="0 0.606 0.044"))
        roller_1.geoms.append(Cylinder("0 0 0", "0.0125 0.211", euler="0 1.5707 0", friction="1.5 0.1 0.01"))
        roller_1_joint = Joint("intake_roller_1_hinge", Joint.Type.HINGE, Joint.Axis.X, armature=0.0015, damping=0.8)
        roller_1.joints.append(roller_1_joint)

        roller_2 = Body("intake_roller_2", pos="0 0.323 0.07")
        roller_2.inertials.append(Inertial(kilograms(2.5), diaginertia="0.06 0.06 0.06"))
        roller_2.geoms.append(Cylinder("0 0 0", "0.025 0.2", euler="0 1.5707 0", friction="1.5 0.1 0.01", type=Geom.Type.COLLISION))
        roller_2.geoms.append(Mesh(roller_1_mesh, pos="0 0.606 0.044"))
        roller_2_joint = Joint("intake_roller_2_hinge", Joint.Type.HINGE, Joint.Axis.NX, armature=0.0015, damping=0.8)
        roller_2.joints.append(roller_2_joint)

        model.equalities.append(Equality(roller_1_joint, roller_2_joint))
        model.motors.append(DCMotor.kraken_x60("intake_roller_1_motor", roller_1_joint, 2.0))

        robot.bodies.append(roller_1)
        robot.bodies.append(roller_2)
