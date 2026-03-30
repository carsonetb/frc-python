from dataclasses import dataclass, field
from enum import Enum

from mujoco import mj_name2id, mjtObj
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)

from frc_python.sim import SimulationInfo
from frc_python.units.angle import Angle, degrees, radians
from frc_python.units.distance import Distance, inches
from frc_python.units.force import Newtons, newtons
from frc_python.units.time import seconds
from frc_python.units.velocity import (
    AngularVelocity,
    LinearVelocity,
    degrees_per_second,
    feet_per_second,
    meters_per_second,
    radians_per_second,
    rotations_per_minute,
)
from frc_python.units.voltage import (
    Voltage,
    volt_seconds_per_meter,
    volt_seconds_per_radian,
    volt_seconds_squared_per_meter,
    voltage,
    volts_per_meter,
    volts_per_radian,
    volts_per_radian_second,
)
from frc_python.utils.control import AngularPIDGains, LinearMotorFFGains, LinearPIDGains
from frc_python.utils.math import Vector2
from frc_python.utils.swerve import PerCorner


@dataclass
class DrivetrainInputs:
    swerve_states_field_relative: PerCorner[SwerveModuleState] = field(
        default_factory=lambda: PerCorner[SwerveModuleState].generate(
            lambda corner: SwerveModuleState()
        )
    )
    swerve_states: PerCorner[SwerveModuleState] = field(
        default_factory=lambda: PerCorner[SwerveModuleState].generate(
            lambda corner: SwerveModuleState()
        )
    )
    swerve_positions: PerCorner[SwerveModulePosition] = field(
        default_factory=lambda: PerCorner[SwerveModulePosition].generate(
            lambda corner: SwerveModulePosition()
        )
    )
    gyro_rotation: Rotation2d = field(default_factory=lambda: Rotation2d())
    gyro_velocity: AngularVelocity = field(
        default_factory=lambda: radians_per_second(0)
    )
    gyro_connected: bool = field(default_factory=lambda: True)


@dataclass
class MotorID:
    joint_name: str
    motor_name: str


class DriveModuleID(Enum):
    FRONT_LEFT = (
        MotorID("fl_swerve_module_turret_hinge", "fl_swerve_module_turret_motor"),
        MotorID("fl_swerve_module_wheel_hinge", "fl_swerve_module_wheel_motor"),
    )
    FRONT_RIGHT = (
        MotorID("fr_swerve_module_turret_hinge", "fr_swerve_module_turret_motor"),
        MotorID("fr_swerve_module_wheel_hinge", "fr_swerve_module_wheel_motor"),
    )
    BACK_LEFT = (
        MotorID("bl_swerve_module_turret_hinge", "bl_swerve_module_turret_motor"),
        MotorID("bl_swerve_module_wheel_hinge", "bl_swerve_module_wheel_motor"),
    )
    BACK_RIGHT = (
        MotorID("br_swerve_module_turret_hinge", "br_swerve_module_turret_motor"),
        MotorID("br_swerve_module_wheel_hinge", "br_swerve_module_wheel_motor"),
    )

    @property
    def steer(self) -> MotorID:
        return self.value[0]

    @property
    def drive(self) -> MotorID:
        return self.value[1]


class SimGyro:
    def __init__(self, info: SimulationInfo, name: str) -> None:
        self.info = info
        self.sensor_id: int = self.info.model.sensor_adr[
            mj_name2id(self.info.model, mjtObj.mjOBJ_SENSOR, name)
        ]
        self.yaw = radians(0)

    @property
    def yaw_velocity(self) -> AngularVelocity:
        return radians_per_second(self.info.data.sensordata[self.sensor_id + 2])

    def periodic(self) -> None:
        self.yaw += self.yaw_velocity.muldim(seconds(0.001))

    def zero(self) -> None:
        self.yaw = radians(0)


class SimKrakenX60:
    FREE_SPEED: AngularVelocity = rotations_per_minute(6000)
    STALL_TORQUE: Newtons = newtons(
        1.8
    )  # Importantly, this is with the Phoenix Pro license. This might also be a lot less because of current limits.
    NOMINAL_VOLTAGE: Voltage = voltage(12)

    def __init__(self, info: SimulationInfo, id: MotorID, gear_ratio: float) -> None:
        self.info = info
        self.joint_id: int = mj_name2id(
            self.info.model, mjtObj.mjOBJ_JOINT, id.joint_name
        )
        self.motor_id: int = mj_name2id(
            self.info.model, mjtObj.mjOBJ_ACTUATOR, id.motor_name
        )
        self.qpos_idx: int = self.info.model.jnt_qposadr[self.joint_id]
        self.gear_ratio: float = gear_ratio

    @property
    def angle(self) -> Angle:
        return radians(self.info.data.qpos[self.qpos_idx])

    @property
    def velocity(self) -> AngularVelocity:
        return radians_per_second(self.info.data.qvel[self.joint_id])

    def apply_voltage(self, voltage: Voltage) -> None:
        self.info.data.ctrl[self.motor_id] = self._voltage_to_torque(
            voltage, self.velocity
        )

    # Returns a torque in newton meters, probably should be unit-ed in the future.
    def _voltage_to_torque(
        self, voltage: Voltage, joint_speed: AngularVelocity
    ) -> float:
        voltage = voltage.clamp(-self.NOMINAL_VOLTAGE, self.NOMINAL_VOLTAGE)
        shaft_speed = joint_speed.mulratio(self.gear_ratio)
        motor_torque = (
            (voltage / self.NOMINAL_VOLTAGE).voltage()
            - (shaft_speed.radians_per_second() / self.FREE_SPEED.radians_per_second())
        ) * self.STALL_TORQUE.raw
        return motor_torque * self.gear_ratio


class SimMk5nSwerveModule:
    DRIVE_GEAR_RATIO: float = 5.27
    STEER_GEAR_RATIO: float = 26.09
    STEER_GAINS: AngularPIDGains = AngularPIDGains(
        volts_per_radian(125), volts_per_radian_second(0), volt_seconds_per_radian(10)
    )

    DRIVE_PID: LinearPIDGains = LinearPIDGains(volts_per_meter(0.7433))
    DRIVE_FF: LinearMotorFFGains = LinearMotorFFGains(
        voltage(0.19991),
        volt_seconds_per_meter(0.64508),
        volt_seconds_squared_per_meter(0.07864),
    )

    STEER_OFFSET = degrees(90)

    WHEEL_RADIUS: Distance = inches(2)

    def __init__(self, info: SimulationInfo, id: DriveModuleID) -> None:
        self.steer_motor: SimKrakenX60 = SimKrakenX60(
            info, id.steer, self.STEER_GEAR_RATIO
        )
        self.drive_motor: SimKrakenX60 = SimKrakenX60(
            info, id.drive, self.DRIVE_GEAR_RATIO
        )
        self.steer_pid: PIDController = self.STEER_GAINS.to_controller()
        self.steer_pid.enableContinuousInput(-0.5, 0.5)
        self.drive_pid: PIDController = self.DRIVE_PID.to_controller()
        self.drive_ff: SimpleMotorFeedforwardMeters = self.DRIVE_FF.to_feedforward()
        self.desired_state = SwerveModuleState(0, Rotation2d())

    @property
    def state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_motor.velocity.to_linear(self.WHEEL_RADIUS).meters_per_second(),
            (self.steer_motor.angle - self.STEER_OFFSET).to_rotation2d(),
        )

    @property
    def position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_motor.angle.to_linear(self.WHEEL_RADIUS).meters(),
            (self.steer_motor.angle - self.STEER_OFFSET).to_rotation2d(),
        )

    def periodic(self) -> None:
        self.turn_to_angle(radians(self.desired_state.angle.radians()))
        self.go_to_speed(meters_per_second(self.desired_state.speed))

    def turn_to_angle(self, angle: Angle) -> None:
        self.steer_motor.apply_voltage(
            voltage(
                self.steer_pid.calculate(
                    self.steer_motor.angle.rotations(),
                    (angle + self.STEER_OFFSET).rotations(),
                )
            )
        )

    def go_to_speed(self, speed: LinearVelocity) -> None:
        self.drive_motor.apply_voltage(
            voltage(
                self.drive_ff.calculate(speed.meters_per_second())
                + self.drive_pid.calculate(self.state.speed, speed.meters_per_second())
            )
        )


class Mk5nDrivetrainIOSim:
    GYRO_NAME = "chassis_gyro"

    def __init__(self, info: SimulationInfo) -> None:
        self.modules = PerCorner(
            front_left=SimMk5nSwerveModule(info, DriveModuleID.FRONT_LEFT),
            front_right=SimMk5nSwerveModule(info, DriveModuleID.FRONT_RIGHT),
            back_left=SimMk5nSwerveModule(info, DriveModuleID.BACK_LEFT),
            back_right=SimMk5nSwerveModule(info, DriveModuleID.BACK_RIGHT),
        )
        self.gyro = SimGyro(info, self.GYRO_NAME)

    @property
    def angle(self) -> Angle:
        return self.gyro.yaw

    @property
    def desired_states(self) -> PerCorner[SwerveModuleState]:
        return self.modules.map_items(lambda module: module.desired_state)

    @desired_states.setter
    def desired_states(self, val: PerCorner[SwerveModuleState]) -> None:
        for module, state in self.modules.zip_with(val):
            module.desired_state = state

    def periodic(self) -> None:
        self.gyro.periodic()

    def update_inputs(self, inputs: DrivetrainInputs):
        for i, module in enumerate(self.modules.to_list()):
            module.periodic()
            inputs.swerve_states[i] = module.state
            inputs.swerve_positions[i] = module.position
            inputs.swerve_states_field_relative[i] = SwerveModuleState(
                module.state.speed,
                module.position.angle + self.gyro.yaw.to_rotation2d(),
            )

        inputs.gyro_rotation = self.gyro.yaw.to_rotation2d()
        inputs.gyro_velocity = self.gyro.yaw_velocity
        inputs.gyro_connected = True


class SimDrivetrain:
    FL_POS = Vector2(inches(10.875), inches(10.875))
    FR_POS = Vector2(inches(10.875), inches(-10.875))
    BL_POS = Vector2(inches(-10.875), inches(10.875))
    BR_POS = Vector2(inches(-10.875), inches(-10.875))

    TOP_SPEED = feet_per_second(19.2)

    # TODO: Actually (rad/s)/rotation
    ROT_ALIGN_PID = AngularPIDGains(
        volts_per_radian(80), volts_per_radian_second(0), volt_seconds_per_radian(60)
    )

    def __init__(self, info: SimulationInfo) -> None:
        self.io = Mk5nDrivetrainIOSim(info)
        self.kinematics = SwerveDrive4Kinematics(
            self.FL_POS.to_translation2d(),
            self.FR_POS.to_translation2d(),
            self.BL_POS.to_translation2d(),
            self.BR_POS.to_translation2d(),
        )
        self.rot_align_controller = self.ROT_ALIGN_PID.to_controller()
        self.rot_align_controller.enableContinuousInput(-0.5, 0.5)

    def periodic(self) -> None:
        self.io.periodic()

    def drive(self, velocity: Vector2[LinearVelocity], omega: AngularVelocity) -> None:
        chassis_speeds = ChassisSpeeds(
            velocity.x.meters_per_second(),
            velocity.y.meters_per_second(),
            -omega.radians_per_second(),
        )
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, self.TOP_SPEED.meters_per_second()
        )

        for i, module in enumerate(self.io.modules.to_list()):
            module_states[i].optimize(module.state.angle)
            module.turn_to_angle(radians(module_states[i].angle.radians()))
            module.go_to_speed(meters_per_second(module_states[i].speed))

    def drive_rot_align(
        self, velocity: Vector2[LinearVelocity], target_omega: Angle
    ) -> None:
        self.drive(
            velocity,
            radians_per_second(
                self.rot_align_controller.calculate(
                    self.io.angle.rotations(), target_omega.rotations()
                )
            ),
        )
