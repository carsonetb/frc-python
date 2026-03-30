from dataclasses import dataclass

from mujoco import MjData, MjModel


@dataclass
class SimulationInfo:
    model: MjModel
    data: MjData
