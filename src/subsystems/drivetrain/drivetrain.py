from commands2.subsystem import Subsystem
from phoenix6.swerve import SwerveModuleConstantsFactory
from wpimath.geometry import Pose2d, Translation2d

from subsystems.drivetrain.constants import FL_ENCODER_OFFSET, FL_POS
from subsystems.drivetrain.drivetrain_io import DrivetrainIO, DrivetrainIOReal
from subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from utils.swerve import Corner, PerCorner


class Drivetrain(Subsystem):
    MODULE_POSITIONS = PerCorner(
        front_left=Corner(FL_POS.to_pose2d(), FL_ENCODER_OFFSET),
        front_right=Corner(FR_)
    )
    
    def __init__(self, odometry_thread: PhoenixOdometryThread) -> None:
        self.io: DrivetrainIO = DrivetrainIOReal(MODULE_PO)
