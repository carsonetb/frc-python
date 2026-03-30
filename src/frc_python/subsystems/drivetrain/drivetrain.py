from commands2.subsystem import Subsystem
from subsystems.drivetrain.constants import (
    BL_ENCODER_OFFSET,
    BL_POS,
    BR_ENCODER_OFFSET,
    BR_POS,
    FL_ENCODER_OFFSET,
    FL_POS,
    FR_ENCODER_OFFSET,
    FR_POS,
)

from frc_python.can import CTREDeviceID
from frc_python.subsystems.drivetrain.drivetrain_io import (
    DrivetrainIO,
    DrivetrainIOReal,
)
from frc_python.subsystems.drivetrain.module import (
    DrivingTalon,
    Mk5nSwerveModule,
    TurningTalon,
)
from frc_python.subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from frc_python.utils.swerve import Corner, PerCorner


class Drivetrain(Subsystem):
    MODULE_POSITIONS = PerCorner(
        front_left=Corner(FL_POS.to_pose2d(), FL_ENCODER_OFFSET),
        front_right=Corner(FR_POS.to_pose2d(), FR_ENCODER_OFFSET),
        back_left=Corner(BL_POS.to_pose2d(), BL_ENCODER_OFFSET),
        back_right=Corner(BR_POS.to_pose2d(), BR_ENCODER_OFFSET),
    )

    MODULE_CAN_IDS: PerCorner[tuple[CTREDeviceID, CTREDeviceID, CTREDeviceID]] = (
        PerCorner(
            (
                CTREDeviceID.FRONT_LEFT_DRIVE_MOTOR,
                CTREDeviceID.FRONT_LEFT_TURN_MOTOR,
                CTREDeviceID.FRONT_LEFT_TURN_ENCODER,
            ),
            (
                CTREDeviceID.FRONT_RIGHT_DRIVE_MOTOR,
                CTREDeviceID.FRONT_RIGHT_TURN_MOTOR,
                CTREDeviceID.FRONT_RIGHT_TURN_ENCODER,
            ),
            (
                CTREDeviceID.BACK_LEFT_DRIVE_MOTOR,
                CTREDeviceID.BACK_LEFT_TURN_MOTOR,
                CTREDeviceID.BACK_LEFT_TURN_ENCODER,
            ),
            (
                CTREDeviceID.BACK_RIGHT_DRIVE_MOTOR,
                CTREDeviceID.BACK_RIGHT_TURN_MOTOR,
                CTREDeviceID.BACK_RIGHT_TURN_ENCODER,
            ),
        )
    )

    def __init__(self, odometry_thread: PhoenixOdometryThread) -> None:
        super().__init__()

        def corner_ids_to_swerve(
            items: tuple[Corner, tuple[CTREDeviceID, CTREDeviceID, CTREDeviceID]],
        ) -> Mk5nSwerveModule:
            (corner, (drive_id, turn_id, encoder_id)) = items
            return Mk5nSwerveModule(
                DrivingTalon(drive_id, odometry_thread),
                TurningTalon(
                    turn_id, encoder_id, corner.magnet_offset, odometry_thread
                ),
                corner.position.rotation(),
                odometry_thread,
            )

        self.io: DrivetrainIO = DrivetrainIOReal(
            self.MODULE_POSITIONS.zip_with(self.MODULE_CAN_IDS).map_items(
                corner_ids_to_swerve
            ),
            odometry_thread,
        )
