from commands2.button.commandjoystick import CommandJoystick
from commands2.button.commandxboxcontroller import CommandXboxController

left_joystick = CommandJoystick(0)
right_joystick = CommandJoystick(1)
controller = CommandXboxController(2)

dev_joystick = CommandJoystick(3)
dev_controller = CommandXboxController(4)


def configure_bindings() -> None:
    pass
