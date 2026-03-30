from time import time

import hal
import pygame
from importlib_resources import read_text
from mujoco import MjData, MjModel
from mujoco.viewer import launch_passive
from pygame.joystick import JoystickType

hal.initialize(500, 0)
from frc_python.robot import Robot


def main():
    pygame.init()
    pygame.joystick.init()

    joystick: JoystickType | None = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    else:
        print("No controller found, please plug one in.")
        return

    model: MjModel = MjModel.from_xml_string(
        read_text("frc_python.resources.subsystems.drivetrain", "swerve.xml")
    )
    data: MjData = MjData(model)

    with launch_passive(model, data) as viewer:
        start_time = time()

        drivetrain = SimDrivetrain()

        while viewer.is_running():
            step_start = time()
            t = time() - start_time

            pygame.event.pump()
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]

            # data.ctrl[0] = 0.0001
            # data.ctrl[1] = 0.001

            drivetrain.periodic()
            # drivetrain.drive_rot_align(
            #     Vector2(meters_per_second(0), meters_per_second(0)), degrees(90)
            # )
            drivetrain.drive_rot_align(
                Vector2(meters_per_second(sin(t)), meters_per_second(cos(t))),
                degrees(sin(t / 1.5) * 70),
            )

            mj_step(model, data)

            viewer.sync()

            time_until_next_step = model.opt.timestep - (time() - step_start)
            if time_until_next_step > 0:
                sleep(time_until_next_step)
