from enum import Enum, auto

from frc_python.units.time import seconds

TIMESTEP = seconds(0.001)


class Model(Enum):
    SIMULATION = auto()
    COMPETITION = auto()
