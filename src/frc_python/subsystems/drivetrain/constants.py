from phoenix6.configs import CANcoderConfiguration, CurrentLimitsConfigs, Slot0Configs, TalonFXConfiguration
from phoenix6.swerve import ClosedLoopOutputType, DriveMotorArrangement, SteerFeedbackType, SteerMotorArrangement, SwerveModuleConstantsFactory

from frc_python.can import CTREDeviceID
from frc_python.units.amps import amps
from frc_python.units.angle import rotations
from frc_python.units.distance import inches
from frc_python.units.mass import kilogram_meters_squared
from frc_python.units.velocity import feet_per_second
from frc_python.units.voltage import volt_seconds_per_meter, volt_seconds_squared_per_meter, voltage, volts_per_meter, volts_per_radian
from frc_python.utils.control import AngularPIDGains, LinearMotorFFGains, LinearPIDGains
from frc_python.utils.math import Vector2

STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.VOLTAGE
DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.VOLTAGE
DRIVE_MOTOR_TYPE = DriveMotorArrangement.TALON_FX_INTEGRATED
STEER_MOTOR_TYPE = SteerMotorArrangement.TALON_FX_INTEGRATED
STEER_FEEDBACK_TYPE = SteerFeedbackType.FUSED_CANCODER

STEER_GAINS = AngularPIDGains(volts_per_radian(125))
DRIVE_PID = LinearPIDGains(volts_per_meter(0.7433))
DRIVE_FF = LinearMotorFFGains(voltage(0.19991), volt_seconds_per_meter(0.64508), volt_seconds_squared_per_meter(0.07864))

SLIP_CURRENT = amps(80)
STEER_CURRENT_LIMIT = amps(60)

SPEED_AT_12_VOLTS = feet_per_second(19.2)

COUPLE_GEAR_RATIO = 0.0
DRIVE_GEAR_RATIO = 5.27
STEER_GEAR_RATIO = 26.09
WHEEL_RADIUS = inches(2)

INVERT_LEFT_SIDE = False
INVERT_RIGHT_SIDE = False

STEER_INERTIA = kilogram_meters_squared(0.01)
DRIVE_INERTIA = kilogram_meters_squared(0.01)

STEER_FRICTION_VOLTAGE = voltage(0.2)
DRIVE_FRICTION_VOLTAGE = voltage(0.2)

DRIVE_INITIAL_CONFIGS = TalonFXConfiguration()
STEER_INITIAL_CONFIGS = TalonFXConfiguration().with_current_limits(
    CurrentLimitsConfigs().with_stator_current_limit(STEER_CURRENT_LIMIT.amps()).with_stator_current_limit_enable(True)
)
ENCODER_INITIAL_CONFIGS = CANcoderConfiguration()

constant_creator = (
    SwerveModuleConstantsFactory[TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration]()
    .with_drive_motor_gear_ratio(DRIVE_GEAR_RATIO)
    .with_steer_motor_gear_ratio(STEER_GEAR_RATIO)
    .with_coupling_gear_ratio(COUPLE_GEAR_RATIO)
    .with_wheel_radius(WHEEL_RADIUS.meters())
    .with_steer_motor_gains(STEER_GAINS.slot_with(Slot0Configs()))
    .with_drive_motor_gains(DRIVE_PID.slot_with(DRIVE_FF.slot_with(Slot0Configs())))
    .with_steer_motor_closed_loop_output(STEER_CLOSED_LOOP_OUTPUT)
    .with_drive_motor_closed_loop_output(DRIVE_CLOSED_LOOP_OUTPUT)
    .with_slip_current(SLIP_CURRENT.amps())
    .with_speed_at12_volts(SPEED_AT_12_VOLTS.meters_per_second())
    .with_drive_motor_type(DRIVE_MOTOR_TYPE)
    .with_steer_motor_type(STEER_MOTOR_TYPE)
    .with_feedback_source(STEER_FEEDBACK_TYPE)
    .with_drive_motor_initial_configs(DRIVE_INITIAL_CONFIGS)
    .with_steer_motor_initial_configs(STEER_INITIAL_CONFIGS)
    .with_encoder_initial_configs(ENCODER_INITIAL_CONFIGS)
    .with_steer_inertia(STEER_INERTIA.raw)
    .with_drive_inertia(DRIVE_INERTIA.raw)
    .with_steer_friction_voltage(STEER_FRICTION_VOLTAGE.voltage())
    .with_drive_friction_voltage(DRIVE_FRICTION_VOLTAGE.voltage())
)

FL_ENCODER_OFFSET = rotations(-0.336181640625)
FL_STEER_INVERTED = False
FL_ENCODER_INVERTED = False
FL_POS = Vector2(inches(10.875), inches(10.875))

FR_ENCODER_OFFSET = rotations(-0.00341796875)
FR_STEER_INVERTED = False
FR_ENCODER_INVERTED = False
FR_POS = Vector2(inches(10.875), inches(-10.875))

BL_ENCODER_OFFSET = rotations(0.132080078125)
BL_STEER_INVERTED = False
BL_ENCODER_INVERTED = False
BL_POS = Vector2(inches(-10.875), inches(10.875))

BR_ENCODER_OFFSET = rotations(-0.210693359375)
BR_STEER_INVERTED = False
BR_ENCODER_INVERTED = False
BR_POS = Vector2(inches(-10.875), inches(-10.875))

FRONT_LEFT = constant_creator.create_module_constants(
    CTREDeviceID.FRONT_LEFT_TURN_MOTOR.num,
    CTREDeviceID.FRONT_LEFT_DRIVE_MOTOR.num,
    CTREDeviceID.FRONT_LEFT_TURN_ENCODER.num,
    FL_ENCODER_OFFSET.rotations(),
    FL_POS.x.meters(),
    FL_POS.y.meters(),
    INVERT_LEFT_SIDE,
    FL_STEER_INVERTED,
    FL_ENCODER_INVERTED,
)

FRONT_RIGHT = constant_creator.create_module_constants(
    CTREDeviceID.FRONT_RIGHT_TURN_MOTOR.num,
    CTREDeviceID.FRONT_RIGHT_DRIVE_MOTOR.num,
    CTREDeviceID.FRONT_RIGHT_TURN_ENCODER.num,
    FR_ENCODER_OFFSET.rotations(),
    FR_POS.x.meters(),
    FR_POS.y.meters(),
    INVERT_RIGHT_SIDE,
    FR_STEER_INVERTED,
    FR_ENCODER_INVERTED,
)

BACK_LEFT = constant_creator.create_module_constants(
    CTREDeviceID.BACK_LEFT_TURN_MOTOR.num,
    CTREDeviceID.BACK_LEFT_DRIVE_MOTOR.num,
    CTREDeviceID.BACK_LEFT_TURN_ENCODER.num,
    BL_ENCODER_OFFSET.rotations(),
    BL_POS.x.meters(),
    BL_POS.y.meters(),
    INVERT_LEFT_SIDE,
    BR_STEER_INVERTED,
    BR_ENCODER_INVERTED,
)

BACK_RIGHT = constant_creator.create_module_constants(
    CTREDeviceID.BACK_RIGHT_TURN_MOTOR.num,
    CTREDeviceID.BACK_RIGHT_DRIVE_MOTOR.num,
    CTREDeviceID.BACK_RIGHT_TURN_ENCODER.num,
    BR_ENCODER_OFFSET.rotations(),
    BR_POS.x.meters(),
    BR_POS.y.meters(),
    INVERT_RIGHT_SIDE,
    BR_STEER_INVERTED,
    BR_ENCODER_INVERTED,
)
