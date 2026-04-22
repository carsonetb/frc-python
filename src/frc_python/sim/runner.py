from dataclasses import dataclass
from time import sleep, time
from typing import Callable

import hal
import pygame
from importlib_resources import read_text
from mujoco import MjData, MjModel, mj_step
from mujoco.viewer import launch_passive
from pygame.joystick import JoystickType
from pykit.loggedrobot import LoggedRobot
from wpilib.simulation import DriverStationSim, XboxControllerSim

from frc_python.sim.builder import build_drivetrain, build_field, build_misc
from frc_python.sim.common import SimulatableRobot, SimulatableSubsystem, SimulationInfo
from frc_python.sim.field import CrescendoField, SimField
from frc_python.sim.xmlgen import Body, Inertial, Model
from frc_python.subsystems.drivetrain.drivetrain import Drivetrain
from frc_python.units.mass import kilograms
from frc_python.utils.misc import TIMESTEP

hal.initialize(500, 0)
from frc_python.robot import Robot

FLIGHT_STICKS = False


class Simulator:
    Field: type[SimField] | None = None
    buildables: list[type[SimulatableSubsystem]] = []
    xml: str | None = None

    @classmethod
    def build(cls, save: str | None = None) -> str:

        if cls.Field is None:
            print("Must set Simulator.Field.")
            return ""

        model = Model("FRC")

        robot = Body("robot", True)
        field = cls.Field()
        field.build(model)

        for Subsystem in cls.buildables:
            Subsystem.build(model, robot)

        xml = model.build()
        if save is not None:
            open(save, "w").write(xml)

        cls.xml = xml

        return xml

    @classmethod
    def simulate(cls, RobotClass: type[SimulatableRobot], flight_sticks: bool = False) -> int:
        print("-- General Simulator for FRC (pre-beta version) --")

        if cls.Field is None:
            print("Must set Simulator.Field.")
            return 1

        print(f"Field is {cls.Field.NAME}")

        if cls.xml is None:
            print("Must call `build` before `simulate`.")
            return 1

        pygame.init()
        pygame.joystick.init()

        if flight_sticks:
            if pygame.joystick.get_count() < 2:
                print("Not all joysticks connected, please plug them in.")
                return 1
            else:
                physical_controller = (pygame.joystick.Joystick(0), pygame.joystick.Joystick(1))
        else:
            if pygame.joystick.get_count() < 1:
                print("No controller found, please plug one in.")
                return 1
            else:
                physical_controller = pygame.joystick.Joystick(0)

        model = MjModel.from_xml_string(cls.xml)
        data = MjData(model)

        robot = RobotClass(SimulationInfo(model, data))
        robot.robotInit()
        robot.teleopInit()

        ds = DriverStationSim()
        controller = XboxControllerSim(0)

        tick = 0
        with launch_passive(model, data) as viewer:
            start_time = time()

            while viewer.is_running():
                tick += 1
                step_start = time()
                t = time() - start_time

                pygame.event.get()

                if isinstance(physical_controller, tuple):
                    (left, right) = physical_controller
                    controller.setLeftX(left.get_axis(0))
                    controller.setLeftY(-left.get_axis(1))
                    controller.setRightX(right.get_axis(1))
                    controller.setRightY(-right.get_axis(0))
                else:
                    controller.setLeftX(physical_controller.get_axis(0))
                    controller.setLeftY(-physical_controller.get_axis(1))
                    controller.setRightX(physical_controller.get_axis(2))
                    controller.setRightY(-physical_controller.get_axis(3))

                ds.setDsAttached(True)
                ds.setEnabled(True)
                ds.setAutonomous(False)
                ds.notifyNewData()

                robot.robotPeriodic()
                robot.teleopPeriodic()

                # data.ctrl[0] = 0.0001
                # data.ctrl[1] = 0.001

                # drivetrain.periodic()
                # # drivetrain.drive_rot_align(
                # #     Vector2(meters_per_second(0), meters_per_second(0)), degrees(90)
                # # )
                # drivetrain.drive_rot_align(
                #     Vector2(meters_per_second(sin(t)), meters_per_second(cos(t))),
                #     degrees(sin(t / 1.5) * 70),
                # )

                mj_step(model, data)

                if tick % 4 == 0:
                    viewer.sync(True)

                wait = TIMESTEP.seconds() - (time() - step_start)
                if wait > 0:
                    sleep(wait)

        return 0


def main():

    # xml_model = Model("field")
    # build_misc(xml_model)
    # build_field(xml_model)
    # build_drivetrain(xml_model)
    # xml = xml_model.build()
    # open("temp.xml", "w").write(xml)
    # # xml = open("temp.xml", "r").read()

    Simulator.Field = CrescendoField
    Simulator.buildables.append(Drivetrain)
    Simulator.build("temp.xml")
    Simulator.simulate(Robot)


if __name__ == "__main__":
    main()
