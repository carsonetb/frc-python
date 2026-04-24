from __future__ import annotations

from math import pi, sqrt, tau
from typing import override

from commands2 import Command, cmd
from commands2.subsystem import Subsystem
from pykit.logger import Logger
from wpilib import Joystick, XboxController
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

from frc_python.can import CTREDeviceID
from frc_python.sim.common import SimulatableRobot, SimulatableSubsystem, SimulationInfo
from frc_python.sim.xmlgen import Body, Box, Gyro, Inertial, Mesh, Model, Site
from frc_python.subsystems.drivetrain.constants import (
    BL_ENCODER_OFFSET,
    BL_POS,
    BR_ENCODER_OFFSET,
    BR_POS,
    FL_ENCODER_OFFSET,
    FL_POS,
    FR_ENCODER_OFFSET,
    FR_POS,
)
from frc_python.subsystems.drivetrain.drivetrain_io import DrivetrainInputs, DrivetrainIO, DrivetrainIOReal, Mk5nDrivetrainIOSim
from frc_python.subsystems.drivetrain.module import DrivingTalon, Mk5nSwerveModule, SimMk5nSwerveModule, TurningTalon
from frc_python.subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from frc_python.units.mass import kilograms
from frc_python.units.time import Time
from frc_python.units.voltage import voltage
from frc_python.utils.math import sign
from frc_python.utils.misc import RobotModel
from frc_python.utils.swerve import Corner, PerCorner


class Drivetrain(SimulatableSubsystem):
    MODULE_POSITIONS = PerCorner(
        front_left=Corner(FL_POS.to_pose2d(), FL_ENCODER_OFFSET),
        front_right=Corner(FR_POS.to_pose2d(), FR_ENCODER_OFFSET),
        back_left=Corner(BL_POS.to_pose2d(), BL_ENCODER_OFFSET),
        back_right=Corner(BR_POS.to_pose2d(), BR_ENCODER_OFFSET),
    )

    MODULE_CAN_IDS: PerCorner[tuple[CTREDeviceID, CTREDeviceID, CTREDeviceID]] = PerCorner(
        (CTREDeviceID.FRONT_LEFT_DRIVE_MOTOR, CTREDeviceID.FRONT_LEFT_TURN_MOTOR, CTREDeviceID.FRONT_LEFT_TURN_ENCODER),
        (CTREDeviceID.FRONT_RIGHT_DRIVE_MOTOR, CTREDeviceID.FRONT_RIGHT_TURN_MOTOR, CTREDeviceID.FRONT_RIGHT_TURN_ENCODER),
        (CTREDeviceID.BACK_LEFT_DRIVE_MOTOR, CTREDeviceID.BACK_LEFT_TURN_MOTOR, CTREDeviceID.BACK_LEFT_TURN_ENCODER),
        (CTREDeviceID.BACK_RIGHT_DRIVE_MOTOR, CTREDeviceID.BACK_RIGHT_TURN_MOTOR, CTREDeviceID.BACK_RIGHT_TURN_ENCODER),
    )

    JOYSTICK_DEADBAND = 0.05
    INPUT_EXP = 1.7

    def __init__(self, odometry_thread: PhoenixOdometryThread, info: SimulationInfo | None, model: RobotModel, period: Time) -> None:
        super().__init__()

        self.period = period

        def corner_ids_to_swerve(items: tuple[Corner, tuple[CTREDeviceID, CTREDeviceID, CTREDeviceID]]) -> Mk5nSwerveModule:
            (corner, (drive_id, turn_id, encoder_id)) = items
            return Mk5nSwerveModule(
                DrivingTalon(drive_id, odometry_thread),
                TurningTalon(turn_id, encoder_id, corner.magnet_offset, odometry_thread),
                corner.position.rotation(),
                odometry_thread,
            )

        self.inputs = DrivetrainInputs()
        match model:
            case RobotModel.COMPETITION:
                self.io = DrivetrainIOReal(self.MODULE_POSITIONS.zip_with(self.MODULE_CAN_IDS).map_items(corner_ids_to_swerve), odometry_thread)
            case RobotModel.SIMULATION:
                assert info is not None
                self.io = Mk5nDrivetrainIOSim(info)
        self._desired_speeds = ChassisSpeeds(0, 0, 0)

    @property
    def desired_speeds(self) -> ChassisSpeeds:
        return self._desired_speeds

    @desired_speeds.setter
    def desired_speeds(self, val: ChassisSpeeds) -> None:
        self._desired_speeds = ChassisSpeeds.discretize(val, self.period.seconds())

    @override
    def periodic(self) -> None:
        # seems to break drivetrain, probably obvious why
        # self.io.periodic()
        self.io.gyro.periodic()

        if isinstance(self.io, DrivetrainIOReal):
            pass
        else:
            self.io.update_inputs(self.inputs)
            self.io.goto_chassis_speeds(self.desired_speeds)

        Logger.processInputs("Drivetrain", self.inputs)  # pyright: ignore[reportUnknownMemberType]

    def _calculate_input_curve(self, input: float) -> float:
        return sign(input) * pow(abs(input), self.INPUT_EXP)  # pyright: ignore[reportAny]

    def _is_in_deadzone(self, translation: Translation2d) -> bool:
        return abs(translation.x) < self.JOYSTICK_DEADBAND and abs(translation.y) < self.JOYSTICK_DEADBAND

    def _drive(self, translation: Translation2d, rotation: Translation2d) -> None:
        if self._is_in_deadzone(translation) and self._is_in_deadzone(rotation):
            self.desired_speeds = ChassisSpeeds(0, 0, 0)
        else:
            x = self._calculate_input_curve(translation.x)
            y = self._calculate_input_curve(translation.y)
            self.desired_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * DrivetrainIO.TOP_SPEED.meters_per_second(),
                y * DrivetrainIO.TOP_SPEED.meters_per_second(),
                rotation.x * tau * (1 + sqrt(abs(x) ** 2 + abs(y) ** 2) * 0.6) * 0.8,
                self.io.gyro.yaw.to_rotation2d(),
            )

    def drive_with_joysticks(self, translation: Joystick, rotation: Joystick) -> Command:
        return cmd.run(
            lambda: self._drive(Translation2d(-translation.getX(), -translation.getX()), Translation2d(-rotation.getY(), -rotation.getX())), self
        )  # TODO: Alliance relative

    def drive_with_controller(self, controller: XboxController) -> Command:
        return cmd.run(
            lambda: self._drive(Translation2d(controller.getLeftX(), controller.getLeftY()), Translation2d(controller.getRightY(), 0)), self
        )

    @override
    @staticmethod
    def build(model: Model, robot: Body) -> None:
        prefix = "frc_python/resources/subsystems/drivetrain/"

        fl_swerve_module = model.add_mesh("fl_swerve_module", prefix + "FLSwerveModule.stl")
        fr_swerve_module = model.add_mesh("fr_swerve_module", prefix + "FRSwerveModule.stl")
        swerve_module_turret = model.add_mesh("swerve_module_turret", prefix + "FLSwerveModuleTurret.stl")
        swerve_module_wheel = model.add_mesh("swerve_module_wheel", prefix + "FLSwerveModuleWheel.stl")
        drivetrain_frame = model.add_mesh("drivetrain_frame", prefix + "DriveTrainFrame.stl")
        bumpers = model.add_mesh("bumpers", prefix + "Bumpers.stl")

        frame = robot
        frame.inertials.append(Inertial(kilograms(40), diaginertia="5 5 4.06585208333"))

        frame.geoms.append(Box("0 -0.3935 0.106", "0.422 0.0255 0.057", friction="0.6 0.1 0.01", solimp="0.8 0.95 0.01", solref="0.02 1.5"))
        frame.geoms.append(Box("0 0.3935 0.106", "0.422 0.0255 0.057", friction="0.6 0.1 0.01", solimp="0.8 0.95 0.01", solref="0.02 1.5"))
        frame.geoms.append(Box("0.3965 0.0 0.106", "0.0255 0.368 0.057", friction="0.6 0.1 0.01", solimp="0.8 0.95 0.01", solref="0.02 1.5"))
        frame.geoms.append(Box("-0.3965 0.0 0.106", "0.0255 0.368 0.057", friction="0.6 0.1 0.01", solimp="0.8 0.95 0.01", solref="0.02 1.5"))

        frame.geoms.append(Mesh(bumpers, "bumper_blue", pos="0 0 0.02"))
        frame.geoms.append(Mesh(drivetrain_frame))
        frame.geoms.append(Mesh(fl_swerve_module))
        frame.geoms.append(Mesh(fr_swerve_module))
        frame.geoms.append(Mesh(fl_swerve_module, axisangle="0 0 1 3.1416"))
        frame.geoms.append(Mesh(fr_swerve_module, axisangle="0 0 1 3.1416"))

        imu_site = Site("imu_site")
        frame.sites.append(imu_site)
        model.gyros.append(Gyro("chassis_gyro", imu_site))

        frame.bodies.append(SimMk5nSwerveModule.build(model, "fl", "0.276225 0.276225 0", swerve_module_turret, swerve_module_wheel))
        frame.bodies.append(SimMk5nSwerveModule.build(model, "fr", "0.276225 -0.276225 0", swerve_module_turret, swerve_module_wheel))
        frame.bodies.append(SimMk5nSwerveModule.build(model, "bl", "-0.276225 0.276225 0", swerve_module_turret, swerve_module_wheel))
        frame.bodies.append(SimMk5nSwerveModule.build(model, "br", "-0.276225 -0.276225 0", swerve_module_turret, swerve_module_wheel))

        model.world.bodies.append(frame)
