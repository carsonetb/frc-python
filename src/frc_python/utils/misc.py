from enum import Enum, auto

from frc_python.units.time import seconds

TIMESTEP = seconds(0.003)


class RobotModel(Enum):
    SIMULATION = auto()
    COMPETITION = auto()
