from __future__ import annotations

from abc import ABC, abstractmethod
from array import array
from multiprocessing import Queue
from typing import override

from phoenix6 import StatusSignal
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.configs import CANcoderConfiguration, CurrentLimitsConfigs, FeedbackConfigs, Slot0Configs, TalonFXConfiguration, TorqueCurrentConfigs
from phoenix6.controls import PositionVoltage, VelocityVoltage, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import FeedbackSensorSourceValue, NeutralModeValue
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from frc_python.can import CTREDeviceID
from frc_python.sim.common import DriveModuleID, SimKrakenX44, SimKrakenX60, SimulationInfo
from frc_python.sim.xmlgen import Body, DCMotor, Inertial, Joint, Mesh, MeshAsset, Model, Motor, Sphere
from frc_python.subsystems.drivetrain.phoenix_odometry import PhoenixOdometryThread
from frc_python.units.amps import Current, amps
from frc_python.units.angle import Angle, degrees, radians, rotations
from frc_python.units.distance import Distance, inches
from frc_python.units.mass import kilograms
from frc_python.units.temperature import Temperature, celsius
from frc_python.units.velocity import AngularVelocity, LinearVelocity, meters_per_second, rotations_per_second
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


class SwerveModule(ABC):
    @property
    @abstractmethod
    def state(self) -> SwerveModuleState:
        pass

    @property
    @abstractmethod
    def desired_state(self) -> SwerveModuleState:
        pass

    @desired_state.setter
    @abstractmethod
    def desired_state(self, val: SwerveModuleState) -> None:
        pass

    @property
    @abstractmethod
    def position(self) -> SwerveModulePosition:
        pass

    @property
    @abstractmethod
    def angular_drive_position(self) -> Angle:
        pass

    @property
    @abstractmethod
    def odometry_timestamps(self) -> list[float]:
        pass

    @property
    @abstractmethod
    def odometry_turn_positions(self) -> list[Angle]:
        pass

    @property
    @abstractmethod
    def odometry_drive_positions(self) -> list[float]:  # TODO: make this use units
        pass

    @property
    @abstractmethod
    def odometry_positions(self) -> list[SwerveModulePosition]:
        pass

    @property
    @abstractmethod
    def valid_timestamps(self) -> int:
        pass

    @property
    def signals(self) -> list[BaseStatusSignal]:
        return []

    @abstractmethod
    def periodic(self) -> None:
        pass

    @abstractmethod
    def characterize(self, voltage: Voltage, turning_angle: Angle | None) -> None:
        pass


class Mk5nSwerveModule(SwerveModule):
    MAX_QUEUE_SIZE: int = 100
    COUPLING_RATIO: float = 0.64

    def __init__(
        self, drive: SwerveDrivingMotor, turn: SwerveTurningMotor, chassis_angle: Rotation2d, odometry_thread: PhoenixOdometryThread
    ) -> None:
        self.drive: SwerveDrivingMotor = drive
        self.turn: SwerveTurningMotor = turn
        self.chassis_angle: Rotation2d = chassis_angle

        self.timestamp_queue: Queue[float] = odometry_thread.make_timestamp_queue()

        self._odometry_timestamps: list[float] = []
        self._odometry_positions: list[SwerveModulePosition] = []
        self._desired_state: SwerveModuleState = SwerveModuleState(0.0, -self.chassis_angle)

    @property
    @override
    def odometry_drive_positions(self) -> list[float]:
        return self.drive.odometry_drive_positions

    @property
    @override
    def odometry_turn_positions(self) -> list[Angle]:
        return self.turn.odometry_turn_positions

    @property
    @override
    def valid_timestamps(self) -> int:
        return self.timestamp_queue.qsize()

    @property
    @override
    def state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive.velocity.meters_per_second(),
            self.turn.position.to_rotation2d() + self.chassis_angle,  # ?
        )

    @property
    @override
    def position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive.position.meters(), self.turn.position.to_rotation2d() + self.chassis_angle)

    @property
    @override
    def angular_drive_position(self) -> Angle:
        return self.drive.angular_position

    @property
    @override
    def desired_state(self) -> SwerveModuleState:
        return SwerveModuleState(self._desired_state.speed, self._desired_state.angle + self.chassis_angle)

    @desired_state.setter
    @override
    def desired_state(self, val: SwerveModuleState) -> None:
        corrected = SwerveModuleState(val.speed, val.angle - self.chassis_angle)
        corrected.optimize(self.turn.position.to_rotation2d())
        corrected.cosineScale(self.turn.position.to_rotation2d())

        self.drive.velocity = meters_per_second(corrected.speed) + (self.turn.velocity.mulratio(self.COUPLING_RATIO)).to_linear(
            DrivingTalon.WHEEL_RADIUS
        )
        self.turn.position = radians(corrected.angle.radians())

        self._desired_state = val

    @property
    @override
    def signals(self) -> list[BaseStatusSignal]:
        return self.turn.signals + self.drive.signals

    @property
    @override
    def odometry_timestamps(self) -> list[float]:
        return self._odometry_timestamps

    @property
    @override
    def odometry_positions(self) -> list[SwerveModulePosition]:
        return self._odometry_positions

    @override
    def characterize(self, voltage: Voltage, turning_angle: Angle | None) -> None:
        self.drive.set_voltage(voltage)
        if turning_angle is not None:
            raise RuntimeError("Turning angle cannot be used right now.")

    @override
    def periodic(self) -> None:
        i = 0
        while not self.timestamp_queue.empty():
            self._odometry_timestamps[i] = self.timestamp_queue.get()
            i += 1

        self.drive.periodic()
        self.turn.periodic()

        for i in range(self.valid_timestamps):
            distance = (radians(self.odometry_drive_positions[i]) - (self.odometry_turn_positions[i].mulratio(self.COUPLING_RATIO))).to_linear(
                DrivingTalon.WHEEL_RADIUS
            )
            angle = self.odometry_turn_positions[i].to_rotation2d() + self.chassis_angle
            pos = self.odometry_positions[i]

            pos.distance = distance.meters()
            pos.angle = angle


class SwerveDrivingMotor(ABC):
    @property
    @abstractmethod
    def position(self) -> Distance:
        pass

    @property
    @abstractmethod
    def angular_position(self) -> Angle:
        pass

    @property
    @abstractmethod
    def velocity(self) -> LinearVelocity:
        pass

    @velocity.setter
    @abstractmethod
    def velocity(self, val: LinearVelocity) -> None:
        pass

    @property
    @abstractmethod
    def temperature(self) -> Temperature:
        pass

    @property
    @abstractmethod
    def odometry_drive_positions(self) -> list[float]:
        pass

    @property
    @abstractmethod
    def signals(self) -> list[BaseStatusSignal]:
        pass

    @abstractmethod
    def set_voltage(self, voltage: Voltage) -> None:
        pass

    @abstractmethod
    def periodic(self) -> None:
        pass


class DrivingTalon(SwerveDrivingMotor):
    CONFIG: Slot0Configs = Slot0Configs().with_k_p(0.7433).with_k_i(0.0).with_k_d(0.0).with_k_s(0.19991).with_k_v(0.63508).with_k_a(0.07864)
    CURRENT_LIMIT: Current = amps(80)
    GEAR_RATIO: float = 5.27
    WHEEL_RADIUS: Distance = inches(1.9225)

    def __init__(self, id: CTREDeviceID, odometry_thread: PhoenixOdometryThread) -> None:
        self.motor: TalonFX = id.to_talonfx()
        self.motor.configurator.apply(
            TalonFXConfiguration()
            .with_slot0(self.CONFIG)
            .with_current_limits(CurrentLimitsConfigs().with_stator_current_limit(self.CURRENT_LIMIT.amps()).with_stator_current_limit_enable(True))
            .with_torque_current(
                TorqueCurrentConfigs()
                .with_peak_forward_torque_current(self.CURRENT_LIMIT.amps())
                .with_peak_reverse_torque_current(-self.CURRENT_LIMIT.amps())
            )
            .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(self.GEAR_RATIO))
        )

        self.position_signal: StatusSignal[float] = self.motor.get_position()
        self.velocity_signal: StatusSignal[float] = self.motor.get_velocity()
        self.temperature_signal: StatusSignal[float] = self.motor.get_device_temp()

        BaseStatusSignal.set_update_frequency_for_all(250.0, self.position_signal)
        BaseStatusSignal.set_update_frequency_for_all(100.0, self.temperature_signal)
        self.motor.optimize_bus_utilization(0.0)

        self.velocity_control: VelocityVoltage = VelocityVoltage(0.0).with_enable_foc(True)
        self.voltage_control: VoltageOut = VoltageOut(0.0).with_enable_foc(True)

        self.position_queue: Queue[float] = odometry_thread.register_signal(self.position_signal)
        self._odometry_drive_positions: list[float] = []

    @property
    @override
    def position(self) -> Distance:
        return self.angular_position.to_linear(self.WHEEL_RADIUS)

    @property
    @override
    def angular_position(self) -> Angle:
        return rotations(self.position_signal.value_as_double)

    @property
    @override
    def velocity(self) -> LinearVelocity:
        return rotations_per_second(self.velocity_signal.value_as_double).to_linear(self.WHEEL_RADIUS)

    @velocity.setter
    @override
    def velocity(self, val: LinearVelocity) -> None:
        self.motor.set_control(self.velocity_control.with_velocity(val.to_angular(self.WHEEL_RADIUS).rotations_per_second()))

    @property
    @override
    def temperature(self) -> Temperature:
        return celsius(self.temperature_signal.value_as_double)

    @property
    @override
    def signals(self) -> list[BaseStatusSignal]:
        return [self.position_signal, self.velocity_signal, self.temperature_signal]

    @property
    @override
    def odometry_drive_positions(self) -> list[float]:
        return self._odometry_drive_positions

    @override
    def set_voltage(self, voltage: Voltage) -> None:
        self.motor.set_control(self.voltage_control.with_output(voltage.voltage()))

    @override
    def periodic(self) -> None:
        self._odometry_drive_positions = []
        while not self.position_queue.empty():
            self._odometry_drive_positions.append(self.position_queue.get())


class SwerveTurningMotor(ABC):
    @property
    @abstractmethod
    def position(self) -> Angle:
        pass

    @position.setter
    @abstractmethod
    def position(self, val: Angle) -> None:
        pass

    @property
    @abstractmethod
    def velocity(self) -> AngularVelocity:
        pass

    @property
    @abstractmethod
    def temperature(self) -> Temperature:
        pass

    @property
    @abstractmethod
    def odometry_turn_positions(self) -> list[Angle]:
        pass

    @property
    @abstractmethod
    def signals(self) -> list[BaseStatusSignal]:
        pass

    @abstractmethod
    def periodic(self) -> None:
        pass


class TurningTalon(SwerveTurningMotor):
    PID: AngularPIDGains = AngularPIDGains(volts_per_radian(125))
    GEAR_RATIO: float = 26.09

    def __init__(self, id: CTREDeviceID, encoder_id: CTREDeviceID, magnet_offset: Angle, odometry_thread: PhoenixOdometryThread) -> None:
        motor_config = TalonFXConfiguration()
        motor_config.slot0 = self.PID.slot_with(motor_config.slot0)
        motor_config.closed_loop_general.continuous_wrap = True
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motor_config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        motor_config.feedback.rotor_to_sensor_ratio = self.GEAR_RATIO
        motor_config.feedback.feedback_remote_sensor_id = encoder_id.num
        self.motor: TalonFX = id.to_talonfx()
        self.motor.configurator.apply(motor_config)

        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor.magnet_offset = magnet_offset.rotations()
        encoder_id.to_cancoder().configurator.apply(encoder_config)

        self.position_control: PositionVoltage = PositionVoltage(0.0).with_enable_foc(True)

        self.position_signal: StatusSignal[float] = self.motor.get_position()
        self.velocity_signal: StatusSignal[float] = self.motor.get_velocity()
        self.temperature_signal: StatusSignal[float] = self.motor.get_device_temp()

        BaseStatusSignal.set_update_frequency_for_all(250.0, self.motor.get_position())
        BaseStatusSignal.set_update_frequency_for_all(100.0, self.velocity_signal, self.temperature_signal)
        self.motor.optimize_bus_utilization(0.0)

        self.position_queue: Queue[float] = odometry_thread.register_signal(self.position_signal)
        self._odometry_turn_positions: list[Angle] = []

    @property
    @override
    def position(self) -> Angle:
        return rotations(self.position_signal.value_as_double)

    @position.setter
    @override
    def position(self, val: Angle) -> None:
        self.motor.set_control(self.position_control.with_position(val.rotations()))

    @property
    @override
    def velocity(self) -> AngularVelocity:
        return rotations_per_second(self.velocity_signal.value_as_double)

    @property
    @override
    def temperature(self) -> Temperature:
        return celsius(self.temperature_signal.value_as_double)

    @property
    @override
    def odometry_turn_positions(self) -> list[Angle]:
        return self._odometry_turn_positions

    @property
    @override
    def signals(self) -> list[BaseStatusSignal]:
        return [self.position_signal, self.velocity_signal, self.temperature_signal]

    @override
    def periodic(self) -> None:
        self._odometry_turn_positions = []
        while not self.position_queue.empty():
            self._odometry_turn_positions.append(rotations(self.position_queue.get()))


class SimMk5nSwerveModule(SwerveModule):
    DRIVE_GEAR_RATIO: float = 5.27
    STEER_GEAR_RATIO: float = 26.09
    STEER_GAINS: AngularPIDGains = AngularPIDGains(volts_per_radian(125), volts_per_radian_second(0), volt_seconds_per_radian(10))

    DRIVE_PID: LinearPIDGains = LinearPIDGains(volts_per_meter(0.7433))
    DRIVE_FF: LinearMotorFFGains = LinearMotorFFGains(voltage(0.19991), volt_seconds_per_meter(1.95), volt_seconds_squared_per_meter(0.07864))

    STEER_OFFSET = degrees(0)

    WHEEL_RADIUS: Distance = inches(2)

    def __init__(self, info: SimulationInfo, id: DriveModuleID, drive_reversed: bool = False, steer_reversed: bool = False) -> None:
        self.steer_motor = SimKrakenX44(info, id.steer, steer_reversed)
        self.drive_motor = SimKrakenX60(info, id.drive, drive_reversed)
        self.steer_pid: PIDController = self.STEER_GAINS.to_controller()
        self.steer_pid.enableContinuousInput(-0.5, 0.5)
        self.drive_pid: PIDController = self.DRIVE_PID.to_controller()
        self.drive_ff: SimpleMotorFeedforwardMeters = self.DRIVE_FF.to_feedforward()
        self._desired_state = SwerveModuleState(0, Rotation2d())

    @property
    @override
    def desired_state(self) -> SwerveModuleState:
        return self._desired_state

    @desired_state.setter
    @override
    def desired_state(self, val: SwerveModuleState) -> None:
        self._desired_state = val

    @property
    @override
    def angular_drive_position(self) -> Angle:
        return self.steer_motor.angle - self.STEER_OFFSET

    @property
    @override
    def odometry_timestamps(self) -> list[float]:
        return []

    @property
    @override
    def odometry_turn_positions(self) -> list[Angle]:
        return []

    @property
    @override
    def odometry_drive_positions(self) -> list[float]:
        return []

    @property
    @override
    def odometry_positions(self) -> list[SwerveModulePosition]:
        return []

    @property
    @override
    def valid_timestamps(self) -> int:
        return 0

    @override
    def characterize(self, voltage: Voltage, turning_angle: Angle | None) -> None:
        print("Tried to run characterize in simulation!")

    @property
    @override
    def state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_motor.velocity.to_linear(self.WHEEL_RADIUS).meters_per_second(), self.angular_drive_position.to_rotation2d()
        )

    @property
    @override
    def position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_motor.angle.to_linear(self.WHEEL_RADIUS).meters(), self.angular_drive_position.to_rotation2d())

    @override
    def periodic(self) -> None:
        self._turn_to_angle(radians(self.desired_state.angle.radians()))
        self._go_to_speed(meters_per_second(self.desired_state.speed))

    def _turn_to_angle(self, angle: Angle) -> None:
        self.steer_motor.apply_voltage(voltage(self.steer_pid.calculate(self.angular_drive_position.rotations(), angle.rotations())))

    def _go_to_speed(self, speed: LinearVelocity) -> None:
        self.drive_motor.apply_voltage(
            voltage(self.drive_ff.calculate(speed.meters_per_second()) + self.drive_pid.calculate(self.state.speed, speed.meters_per_second()))
        )

    @staticmethod
    def build(model: Model, side: str, pos: str, turret_mesh: MeshAsset, wheel_mesh: MeshAsset) -> Body:
        module = Body(f"{side}_swerve_module_turret", pos=pos)

        module.inertials.append(Inertial(kilograms(1.5), diaginertia="0.02 0.02 0.02"))
        module.joints.append(Joint(f"{side}_swerve_module_suspension", Joint.Type.SLIDE, Joint.Axis.Z, stiffness=50000, damping=1000, springref=0))
        turret_hinge = Joint(f"{side}_swerve_module_turret_hinge", Joint.Type.HINGE, Joint.Axis.Z, armature=0.0015, damping=0.8)
        module.joints.append(turret_hinge)
        module.geoms.append(Mesh(turret_mesh, pos="0.066675 0.066675 0"))

        wheel = Body(f"{side}_swerve_module_wheel", pos="0 0 0.0508")
        module.bodies.append(wheel)

        wheel.inertials.append(Inertial(kilograms(0.4), diaginertia="0.000702 0.000499 0.000499"))
        wheel_hinge = Joint(f"{side}_swerve_module_wheel_hinge", Joint.Type.HINGE, Joint.Axis.Y, actuatorfrcrange="-9.70 9.70", frictionloss="0.81")
        wheel.joints.append(wheel_hinge)
        wheel.geoms.append(Mesh(wheel_mesh, pos="0.066675 0.066675 -0.0508"))
        wheel.geoms.append(Sphere("0 0 0", 0.0508, f"{side}_swerve_module_wheel_collision", "2.255 0.001 0.01"))

        model.motors.append(DCMotor.kraken_x44(f"{side}_swerve_module_turret_motor", turret_hinge, 26.09))
        model.motors.append(DCMotor.kraken_x60(f"{side}_swerve_module_wheel_motor", wheel_hinge, 5.27))

        return module
