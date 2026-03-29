from dataclasses import dataclass
from importlib.resources import read_text
from math import sin
from time import sleep, time

import mujoco
from mujoco import MjData, MjModel, mj_name2id, mj_step
from mujoco.viewer import launch_passive
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.kinematics import SwerveModuleState

from frc_python.units.angle import Angle, degrees, radians
from frc_python.units.distance import Distance, inches
from frc_python.units.force import Newtons, newtons
from frc_python.units.velocity import (
    AngularVelocity,
    LinearVelocity,
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

model: MjModel = MjModel.from_xml_string(
    read_text("frc_python.resources.subsystems.drivetrain", "swerve.xml")
)
data: MjData = MjData(model)


@dataclass
class MotorID:
    joint_name: str
    motor_name: str


class SimKrakenX60:
    FREE_SPEED: AngularVelocity = rotations_per_minute(6000)
    STALL_TORQUE: Newtons = newtons(
        0.9
    )  # Importantly, this is with the Phoenix Pro license. This might also be a lot less because of current limits.
    NOMINAL_VOLTAGE: Voltage = voltage(12)

    def __init__(self, id: MotorID, gear_ratio: float) -> None:
        self.joint_id: int = mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, id.joint_name)
        self.motor_id: int = mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, id.motor_name
        )
        self.qpos_idx: int = model.jnt_qposadr[self.joint_id]
        self.gear_ratio: float = gear_ratio

    @property
    def angle(self) -> Angle:
        return radians(data.qpos[self.qpos_idx])

    @property
    def velocity(self) -> AngularVelocity:
        return radians_per_second(data.qvel[self.joint_id])

    def apply_voltage(self, voltage: Voltage) -> None:
        data.ctrl[self.motor_id] = self._voltage_to_torque(voltage, self.velocity)

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
        volts_per_radian(125), volts_per_radian_second(0), volt_seconds_per_radian(4)
    )

    DRIVE_PID: LinearPIDGains = LinearPIDGains(volts_per_meter(0.7433))
    DRIVE_FF: LinearMotorFFGains = LinearMotorFFGains(
        voltage(0.19991),
        volt_seconds_per_meter(0.64508),
        volt_seconds_squared_per_meter(0.07864),
    )

    WHEEL_RADIUS: Distance = inches(2)

    def __init__(self, steer_id: MotorID, drive_id: MotorID) -> None:
        self.steer_motor: SimKrakenX60 = SimKrakenX60(steer_id, self.STEER_GEAR_RATIO)
        self.drive_motor: SimKrakenX60 = SimKrakenX60(drive_id, self.DRIVE_GEAR_RATIO)
        self.steer_pid: PIDController = self.STEER_GAINS.to_controller()
        self.drive_pid: PIDController = self.DRIVE_PID.to_controller()
        self.drive_ff: SimpleMotorFeedforwardMeters = self.DRIVE_FF.to_feedforward()

    @property
    def state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_motor.velocity.to_linear(self.WHEEL_RADIUS).meters_per_second(),
            self.steer_motor.angle.to_rotation2d(),
        )

    def turn_to_angle(self, angle: Angle) -> None:
        self.steer_motor.apply_voltage(
            voltage(
                self.steer_pid.calculate(
                    self.steer_motor.angle.rotations(), angle.rotations()
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


with launch_passive(model, data) as viewer:
    start_time = time()

    fl_module = SimMk5nSwerveModule(
        MotorID("fl_swerve_module_turret_hinge", "fl_swerve_module_turret_motor"),
        MotorID("fl_swerve_module_wheel_hinge", "fl_swerve_module_wheel_motor"),
    )
    fr_module = SimMk5nSwerveModule(
        MotorID("fr_swerve_module_turret_hinge", "fr_swerve_module_turret_motor"),
        MotorID("fr_swerve_module_wheel_hinge", "fr_swerve_module_wheel_motor"),
    )
    bl_module = SimMk5nSwerveModule(
        MotorID("bl_swerve_module_turret_hinge", "bl_swerve_module_turret_motor"),
        MotorID("bl_swerve_module_wheel_hinge", "bl_swerve_module_wheel_motor"),
    )
    br_module = SimMk5nSwerveModule(
        MotorID("br_swerve_module_turret_hinge", "br_swerve_module_turret_motor"),
        MotorID("br_swerve_module_wheel_hinge", "br_swerve_module_wheel_motor"),
    )

    while viewer.is_running():
        step_start = time()
        t = time() - start_time

        # data.ctrl[0] = 0.0001
        # data.ctrl[1] = 0.001

        mj_step(model, data)

        fl_module.turn_to_angle(degrees(t * 100))
        fr_module.turn_to_angle(degrees(t * 100))
        bl_module.turn_to_angle(degrees(t * 100))
        br_module.turn_to_angle(degrees(t * 100))
        fl_module.go_to_speed(meters_per_second(2))
        fr_module.go_to_speed(meters_per_second(2))
        bl_module.go_to_speed(meters_per_second(2))
        br_module.go_to_speed(meters_per_second(2))

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time() - step_start)
        if time_until_next_step > 0:
            sleep(time_until_next_step)
