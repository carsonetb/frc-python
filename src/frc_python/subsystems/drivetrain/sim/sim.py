from importlib.resources import read_text
from math import sin
from time import sleep, time

from mujoco import MjData, MjModel, mj_step
from mujoco.viewer import launch_passive

from frc_python.units.angle import Angle, degrees, radians
from frc_python.units.force import Newtons, newtons
from frc_python.units.velocity import (
    AngularVelocity,
    radians_per_second,
    rotations_per_minute,
)
from frc_python.units.voltage import Voltage, voltage, volts_per_radian
from frc_python.utils.control import AngularPIDGains

model: MjModel = MjModel.from_xml_string(
    read_text("frc_python.resources.subsystems.drivetrain", "swerve.xml")
)
data: MjData = MjData(model)


class SimKrakenX60:
    FREE_SPEED: AngularVelocity = rotations_per_minute(6000)
    STALL_TORQUE: Newtons = newtons(
        9.36
    )  # Importantly, this is with the Phoenix Pro license. This might also be a lot less because of current limits.
    NOMINAL_VOLTAGE: Voltage = voltage(12)

    def __init__(self, index: int, gear_ratio: float) -> None:
        self.index: int = index
        self.gear_ratio: float = gear_ratio

    @property
    def angle(self) -> Angle:
        return radians(data.qpos[self.index])

    def apply_voltage(self, voltage: Voltage) -> None:
        data.ctrl[self.index] = self._voltage_to_torque(
            voltage, radians_per_second(data.qvel[self.index])
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
    STEER_GAINS: AngularPIDGains = AngularPIDGains(volts_per_radian(125))

    def __init__(self, steer_index: int, drive_index: int) -> None:
        self.steer_motor: SimKrakenX60 = SimKrakenX60(
            steer_index, self.STEER_GEAR_RATIO
        )
        self.drive_motor: SimKrakenX60 = SimKrakenX60(
            steer_index, self.DRIVE_GEAR_RATIO
        )
        self.drive_pid = self.STEER_GAINS.to_controller()

    def turn_to_angle(self, angle: Angle) -> None:
        self.steer_motor.apply_voltage(
            voltage(self.drive_pid.calculate(self.steer_motor.angle, angle))
        )


with launch_passive(model, data) as viewer:
    start_time = time()

    module = SimMk5nSwerveModule(0, 1)

    while viewer.is_running():
        step_start = time()
        t = time() - start_time

        # data.ctrl[0] = 0.0001
        # data.ctrl[1] = 0.001

        mj_step(model, data)

        module.turn_to_angle(degrees(sin(t * 5) * 100))

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time() - step_start)
        if time_until_next_step > 0:
            sleep(time_until_next_step)
