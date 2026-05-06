from enum import Enum

from wpilib import SendableChooser, SmartDashboard


class Auto(Enum):
    NONE = "None"

class Dashboard:
    def __init__(self) -> None:
        self.auto_chooser: SendableChooser = SendableChooser()
        for auto in Auto:
            if auto == Auto.NONE:
                self.auto_chooser.setDefaultOption(auto.value, auto)
            else:
                self.auto_chooser.addOption(auto.value, auto)
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)
