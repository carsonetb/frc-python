# FRC Python Simulator

This is the project for my implementation of the robot code for FRC team
3636 (Generals) in Python. More importantly it also contains the code for
the 3D FRC physics simulator I'm working on, also known as **General Sim**, 
which integrates into robot code. The physics simulator used is MuJoCo, 
an open source physics engine designed for realtime high-accuracy simulation. 

The tool is currently pre-beta---I am still developing it, and it is not 
stable. That said, here are the features I have currently implemented:

- Flexible Python wrapper for XML generation
- Simple field model with collisions (for the 2024 Crescendo season)
- Simple game piece with collisions (the Note)
- Realistic drivetrain simulation which has:
  - Accurate simulation on the swerve wheel level---the robot moves by 
    applying a voltage to the drive and turning motors, which make contact
    with the ground and push the robot in a physically accurate manner
  - Bumper contact dynamics
- Currently models are optimized so that faster than realtime simulation 
  can be achieved on even a laptop, even with a high number of game pieces

Here are the features I plan to implement in the future. If you're up to
the task, feel free to contribute any of these in the form of a pull request:

- Improve QOL and reduce boilerplate by adding more helper functions to the 
  model builder, especially in the realm of mesh loading
- Generic simulation examples and controllers for intakes, pivots, flywheels,
  and other common FRC mechanisms
- Separate simulation into a separate module from my robot code
- Write more documentation.

## Running

Testing out the simulator is relatively easy, although I haven't gotten the 
renderer to work on Mac so good luck if you're using one. 

Run `uv sync` to install all the required packages. 

Make sure to plug a controller in (or if you're using flight sticks set the 
`FLIGHT_STICKS` constant to `True` in `sim/runner.py`). To run, `cd` into 
the `src` directory and run `python -m frc_python.sim.runner`. Ideally a 
window with the simulation should pop up, but if not you can open an issue
and I will be happy to help.
