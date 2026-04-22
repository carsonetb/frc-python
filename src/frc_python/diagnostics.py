from typing import Final

from phoenix6.canbus import CANBus
from wpilib import Alert
from wpilib.interfaces import GenericHID

from robot import Robot


class Diagnostics:
    def __init__(self, robot: Robot) -> None:
        self.robot: Final = robot
        self.alerts: set[Alert] = set()
        self.previous_alerts: set[Alert] = set()

    def add_alert(self, text: str) -> None:
        self.alerts.add(Alert(text, Alert.AlertType.kError))

    def report_can(self, can_bus: CANBus) -> None:
        status = can_bus.get_status()

        if status.status.is_error():
            self.add_alert(f'The "{can_bus.name}" CAN bus has FAILED!')

        if status.rec > 0 or status.tec > 0:
            self.add_alert(f'Devices on the "{can_bus.name}" CAN bus are experiencing errors (REC={status.rec}, TEC={status.tec}).')

    def report_ds_peripheral(self, device: GenericHID, is_joystick: bool) -> None:
        if not device.isConnected():
            if is_joystick:
                self.add_alert("An XBox controller has disconnected.")
            else:
                self.add_alert("A joystick has disconnected.")

        device_type = device.getType()
        if (
            not (device_type == GenericHID.HIDType.kHIDJoystick or device_type == GenericHID.HIDType.kHIDFlight)
            if is_joystick
            else (device_type == GenericHID.HIDType.kHIDGamepad or device_type == GenericHID.HIDType.kXInputGamepad)
        ):
            self.add_alert("Check USB device order in the Driver Station! The connected devices are likely in the wrong order.")

    def periodic(self) -> None:
        self.alerts = set()

        if self.robot.status_signals.is_all_good():
            self.add_alert("A CAN refresh failed, outdated data is being received.")

    def send(self) -> None:
        for alert in self.previous_alerts:
            alert.set(False)
        self.previous_alerts = set()

        for alert in self.alerts:
            alert.set(True)
        self.previous_alerts |= self.alerts
