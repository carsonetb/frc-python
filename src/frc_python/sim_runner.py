from dataclasses import dataclass
from time import sleep, time

import hal
import pygame
from importlib_resources import read_text
from mujoco import MjData, MjModel, mj_step
from mujoco.viewer import launch_passive
from pygame.joystick import JoystickType
from wpilib.simulation import DriverStationSim, XboxControllerSim

from frc_python.units.time import seconds
from frc_python.utils.sim import SimulationInfo

hal.initialize(500, 0)
from frc_python.robot import Robot

TIMESTEP = seconds(0.002)

FLIGHT_STICKS = False


def main():
    pygame.init()
    pygame.joystick.init()

    if FLIGHT_STICKS:
        joystick_left: JoystickType | None = None
        joystick_right: JoystickType | None = None
        if pygame.joystick.get_count() >= 2:
            joystick_left = pygame.joystick.Joystick(0)
            joystick_right = pygame.joystick.Joystick(1)
        else:
            print("Not all joysticks connected, please plug them in.")
    else:
        joystick: JoystickType | None = None
        if pygame.joystick.get_count() >= 1:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        else:
            print("No controller found, please plug one in.")
            return

    model: MjModel = MjModel.from_xml_string(
        read_text("frc_python.resources", "world.xml")
    )
    data: MjData = MjData(model)

    robot = Robot(SimulationInfo(model, data))
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

            if FLIGHT_STICKS:
                controller.setLeftX(joystick_left.get_axis(0))
                controller.setLeftY(-joystick_left.get_axis(1))
                controller.setRightX(joystick_right.get_axis(1))
                controller.setRightY(-joystick_right.get_axis(0))
            else:
                controller.setLeftX(joystick.get_axis(0))
                controller.setLeftY(-joystick.get_axis(1))
                controller.setRightX(joystick.get_axis(2))
                controller.setRightY(-joystick.get_axis(3))

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

            if tick % 10 == 0:
                viewer.sync()

            # time_until_next_step = model.opt.timestep - (time() - step_start)
            # if time_until_next_step > 0:
            #     sleep(time_until_next_step)


if __name__ == "__main__":
    main()
