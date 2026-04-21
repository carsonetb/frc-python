from random import randint, random, randrange, uniform

from frc_python.sim.xmlgen import (
    Body,
    Box,
    Geom,
    Gyro,
    Inertial,
    Joint,
    Material,
    Mesh,
    MeshAsset,
    MeshDeformable,
    Model,
    Motor,
    Plugin,
    Site,
    Sphere,
    World,
)
from frc_python.units.distance import meters
from frc_python.units.mass import kilograms
from frc_python.utils.math import Vector3


def build_misc(model: Model) -> None:
    materials = model.asset.materials

    materials.append(Material("grey", 0.4, 0.4, 0.4))
    materials.append(Material("white", 0.8, 0.8, 0.8))
    materials.append(Material("bright_white", 1, 1, 1))
    materials.append(Material("black", 0.2, 0.2, 0.2))
    materials.append(Material("red", 0.9, 0, 0))
    materials.append(Material("blue", 0, 0, 0.9))
    materials.append(Material("bumper_blue", 0.2, 0.2, 0.8))
    materials.append(Material("orange", 1, 0.5, 0))


def build_field(model: Model) -> None:
    geoms = model.world.geoms
    meshes = model.asset.meshes

    audience_side_field_wall = MeshAsset(
        "audience_side_field_wall", "frc_python/resources/AudienceSideFieldWall.stl"
    )
    scoring_side_field_wall = MeshAsset(
        "scoring_side_field_wall", "frc_python/resources/ScoringSideFieldWall.stl"
    )
    red_alliance_driver_station = MeshAsset(
        "red_alliance_driver_station",
        "frc_python/resources/RedAllianceDriverStation.stl",
    )
    blue_alliance_driver_station = MeshAsset(
        "blue_alliance_driver_station",
        "frc_python/resources/BlueAllianceDriverStation.stl",
    )
    red_speaker = MeshAsset("red_speaker", "frc_python/resources/RedSpeaker.stl")
    red_source = MeshAsset("red_source", "frc_python/resources/RedSource.stl")
    red_stage = MeshAsset("red_stage", "frc_python/resources/RedStage.stl")
    red_amp = MeshAsset("red_amp", "frc_python/resources/RedAmp.stl")
    blue_speaker = MeshAsset("blue_speaker", "frc_python/resources/BlueSpeaker.stl")
    blue_source = MeshAsset("blue_source", "frc_python/resources/BlueSource.stl")
    blue_stage = MeshAsset("blue_stage", "frc_python/resources/BlueStage.stl")
    blue_amp = MeshAsset("blue_amp", "frc_python/resources/BlueAmp.stl")

    red_speaker_ll_wall_collision = MeshAsset(
        "red_speaker_lower_left_wall_collision",
        "frc_python/resources/collisions/RedSpeakerLowerLeftWall.stl",
    )
    red_speaker_lr_wall_collision = MeshAsset(
        "red_speaker_lower_right_wall_collision",
        "frc_python/resources/collisions/RedSpeakerLowerRightWall.stl",
    )
    red_source_wall_collision = MeshAsset(
        "red_source_wall_collision",
        "frc_python/resources/collisions/RedSourceWall.stl",
    )
    red_stage_collision_1 = MeshAsset(
        "red_stage_collision_1", "frc_python/resources/collisions/RedStage1.stl"
    )
    red_stage_collision_2 = MeshAsset(
        "red_stage_collision_2", "frc_python/resources/collisions/RedStage2.stl"
    )
    red_stage_collision_3 = MeshAsset(
        "red_stage_collision_3", "frc_python/resources/collisions/RedStage2.stl"
    )
    blue_speaker_ll_wall_collision = MeshAsset(
        "blue_speaker_lower_left_wall_collision",
        "frc_python/resources/collisions/BlueSpeakerLowerLeftWall.stl",
    )
    blue_speaker_lr_wall_collision = MeshAsset(
        "blue_speaker_lower_right_wall_collision",
        "frc_python/resources/collisions/BlueSpeakerLowerRightWall.stl",
    )
    blue_source_wall_collision = MeshAsset(
        "blue_source_wall_collision",
        "frc_python/resources/collisions/BlueSourceWall.stl",
    )
    blue_stage_collision_1 = MeshAsset(
        "blue_stage_collision_1",
        "frc_python/resources/collisions/BlueStage1.stl",
    )
    blue_stage_collision_2 = MeshAsset(
        "blue_stage_collision_2",
        "frc_python/resources/collisions/BlueStage2.stl",
    )
    blue_stage_collision_3 = MeshAsset(
        "blue_stage_collision_3",
        "frc_python/resources/collisions/BlueStage3.stl",
    )
    note = MeshAsset("note", "frc_python/resources/collisions/Note.stl")

    carpet = MeshAsset("carpet", "frc_python/resources/Carpet.stl")
    red_tape = MeshAsset("red_tape", "frc_python/resources/RedTape.stl")
    blue_tape = MeshAsset("blue_tape", "frc_python/resources/BlueTape.stl")
    white_tape = MeshAsset("white_tape", "frc_python/resources/WhiteTape.stl")

    meshes.append(audience_side_field_wall)
    meshes.append(scoring_side_field_wall)
    meshes.append(red_alliance_driver_station)
    meshes.append(blue_alliance_driver_station)
    meshes.append(red_speaker)
    meshes.append(red_source)
    meshes.append(red_stage)
    meshes.append(red_amp)
    meshes.append(blue_speaker)
    meshes.append(blue_source)
    meshes.append(blue_stage)
    meshes.append(blue_amp)

    meshes.append(red_speaker_ll_wall_collision)
    meshes.append(red_speaker_lr_wall_collision)
    meshes.append(red_source_wall_collision)
    meshes.append(red_stage_collision_1)
    meshes.append(red_stage_collision_2)
    meshes.append(red_stage_collision_3)
    meshes.append(blue_speaker_ll_wall_collision)
    meshes.append(blue_speaker_lr_wall_collision)
    meshes.append(blue_source_wall_collision)
    meshes.append(blue_stage_collision_1)
    meshes.append(blue_stage_collision_2)
    meshes.append(blue_stage_collision_3)

    meshes.append(carpet)
    meshes.append(red_tape)
    meshes.append(blue_tape)
    meshes.append(white_tape)
    meshes.append(note)

    geoms.append(Mesh(audience_side_field_wall))
    geoms.append(Mesh(scoring_side_field_wall))
    geoms.append(Mesh(red_alliance_driver_station))
    geoms.append(Mesh(blue_alliance_driver_station))
    geoms.append(Mesh(red_speaker))
    geoms.append(Mesh(red_source))
    geoms.append(Mesh(red_stage))
    geoms.append(Mesh(red_amp))
    geoms.append(Mesh(blue_speaker))
    geoms.append(Mesh(blue_source))
    geoms.append(Mesh(blue_stage))
    geoms.append(Mesh(blue_amp))
    geoms.append(Mesh(carpet))
    geoms.append(Mesh(red_tape))
    geoms.append(Mesh(blue_tape))
    geoms.append(Mesh(white_tape))

    geoms.append(
        Box(
            "0 -4.206 0.2615",
            "8.268 0.1 0.241",
            "audience_side_field_wall_collision",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "0 4.206 0.2615",
            "8.268 0.1 0.241",
            "scoring_side_field_wall_collision",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "8.2955 1.163 0.9955",
            "0.0245 1.84 1.981",
            "red_alliance_driver_station_collision_large",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "8.2975 -3.144 0.995",
            "0.0225 0.9375 0.99",
            "red_alliance_driver_station_collision_small",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "-8.2955 0.9375 0.9955",
            "0.0245 1.84 1.981",
            "blue_alliance_driver_station_collision_large",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "-8.2975 -3.144 0.995",
            "0.0225 0.9375 0.99",
            "blue_alliance_driver_station_collision_small",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "-7.355 -1.442 0.111",
            "0.01 0.521 0.1",
            "red_alliance_speaker_low_wall_flat",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )
    geoms.append(
        Box(
            "7.355 -1.442 0.111",
            "0.01 0.521 0.1",
            "blu_alliance_speaker_low_wall_flat",
            type=Geom.Type.COLLISION,
            friction="0.3 0.005 0.0001",
        )
    )

    geoms.append(Mesh(red_speaker_ll_wall_collision, type=Geom.Type.COLLISION))
    geoms.append(Mesh(red_speaker_lr_wall_collision, type=Geom.Type.COLLISION))
    geoms.append(Mesh(red_source_wall_collision, type=Geom.Type.COLLISION))
    geoms.append(Mesh(red_stage_collision_1, type=Geom.Type.COLLISION))
    geoms.append(Mesh(red_stage_collision_2, type=Geom.Type.COLLISION))
    geoms.append(Mesh(red_stage_collision_3, type=Geom.Type.COLLISION))
    geoms.append(Mesh(blue_speaker_ll_wall_collision, type=Geom.Type.COLLISION))
    geoms.append(Mesh(blue_speaker_lr_wall_collision, type=Geom.Type.COLLISION))
    geoms.append(Mesh(blue_source_wall_collision, type=Geom.Type.COLLISION))
    geoms.append(Mesh(blue_stage_collision_1, type=Geom.Type.COLLISION))
    geoms.append(Mesh(blue_stage_collision_2, type=Geom.Type.COLLISION))
    geoms.append(Mesh(blue_stage_collision_3, type=Geom.Type.COLLISION))

    for _ in range(30):
        model.world.bodies.append(
            _build_note(
                f"{uniform(0, 1000)}",
                f"{uniform(-4, 4)} {uniform(-4, 4)} {uniform(5, 10)}",
                note,
            )
        )


def build_drivetrain(model: Model) -> None:
    prefix = "frc_python/resources/subsystems/drivetrain/"
    meshes = model.asset.meshes

    fl_swerve_module = MeshAsset("fl_swerve_module", prefix + "FLSwerveModule.stl")
    fr_swerve_module = MeshAsset("fr_swerve_module", prefix + "FRSwerveModule.stl")
    swerve_module_turret = MeshAsset(
        "swerve_module_turret",
        prefix + "FLSwerveModuleTurret.stl",
    )
    swerve_module_wheel = MeshAsset(
        "swerve_module_wheel",
        prefix + "FLSwerveModuleWheel.stl",
    )
    drivetrain_frame = MeshAsset("drivetrain_frame", prefix + "DriveTrainFrame.stl")
    bumpers = MeshAsset("bumpers", prefix + "Bumpers.stl")

    meshes.append(fl_swerve_module)
    meshes.append(fr_swerve_module)
    meshes.append(swerve_module_turret)
    meshes.append(swerve_module_wheel)
    meshes.append(drivetrain_frame)
    meshes.append(bumpers)

    frame = Body("drive_train_frame", True)
    frame.inertials.append(Inertial(kilograms(50), diaginertia="5 5 5"))

    frame.geoms.append(
        Box(
            "0 -0.3935 0.086",
            "0.422 0.0255 0.057",
            friction="0.6 0.1 0.01",
            solimp="0.8 0.95 0.01",
            solref="0.02 1.5",
        )
    )
    frame.geoms.append(
        Box(
            "0 0.3935 0.086",
            "0.422 0.0255 0.057",
            friction="0.6 0.1 0.01",
            solimp="0.8 0.95 0.01",
            solref="0.02 1.5",
        )
    )
    frame.geoms.append(
        Box(
            "0.3965 0.0 0.086",
            "0.0255 0.368 0.057",
            friction="0.6 0.1 0.01",
            solimp="0.8 0.95 0.01",
            solref="0.02 1.5",
        )
    )
    frame.geoms.append(
        Box(
            "-0.3965 0.0 0.086",
            "0.0255 0.368 0.057",
            friction="0.6 0.1 0.01",
            solimp="0.8 0.95 0.01",
            solref="0.02 1.5",
        )
    )

    frame.geoms.append(Mesh(bumpers, "bumper_blue"))
    frame.geoms.append(Mesh(drivetrain_frame))
    frame.geoms.append(Mesh(fl_swerve_module))
    frame.geoms.append(Mesh(fr_swerve_module))
    frame.geoms.append(Mesh(fl_swerve_module, axisangle="0 0 1 3.1416"))
    frame.geoms.append(Mesh(fr_swerve_module, axisangle="0 0 1 3.1416"))

    imu_site = Site("imu_site")
    frame.sites.append(imu_site)
    model.gyros.append(Gyro("chassis_gyro", imu_site))

    frame.bodies.append(
        _build_module(
            model,
            "fl",
            "0.276225 0.276225 0",
            swerve_module_turret,
            swerve_module_wheel,
        )
    )
    frame.bodies.append(
        _build_module(
            model,
            "fr",
            "0.276225 -0.276225 0",
            swerve_module_turret,
            swerve_module_wheel,
        )
    )
    frame.bodies.append(
        _build_module(
            model,
            "bl",
            "-0.276225 0.276225 0",
            swerve_module_turret,
            swerve_module_wheel,
        )
    )
    frame.bodies.append(
        _build_module(
            model,
            "br",
            "-0.276225 -0.276225 0",
            swerve_module_turret,
            swerve_module_wheel,
        )
    )

    model.world.bodies.append(frame)


def _build_module(
    model: Model, side: str, pos: str, turret_mesh: MeshAsset, wheel_mesh: MeshAsset
) -> Body:
    module = Body(f"{side}_swerve_module_turret", pos=pos)

    module.inertials.append(Inertial(kilograms(1.5), diaginertia="0.02 0.02 0.02"))
    module.joints.append(
        Joint(
            f"{side}_swerve_module_suspension",
            Joint.Type.SLIDE,
            Joint.Axis.Z,
            stiffness=50000,
            damping=1000,
            springref=0,
        )
    )
    turret_hinge = Joint(
        f"{side}_swerve_module_turret_hinge",
        Joint.Type.HINGE,
        Joint.Axis.Z,
        armature=0.0015,
        damping=0.8,
    )
    module.joints.append(turret_hinge)
    module.geoms.append(Mesh(turret_mesh, pos="0.066675 0.066675 0"))

    wheel = Body(f"{side}_swerve_module_wheel", pos="0 0 0.0508")
    module.bodies.append(wheel)

    wheel.inertials.append(
        Inertial(kilograms(0.4), diaginertia="0.000702 0.000499 0.000499")
    )
    wheel_hinge = Joint(
        f"{side}_swerve_module_wheel_hinge",
        Joint.Type.HINGE,
        Joint.Axis.Y,
        damping=0.03,
    )
    wheel.joints.append(wheel_hinge)
    wheel.geoms.append(Mesh(wheel_mesh, pos="0.066675 0.066675 -0.0508"))
    wheel.geoms.append(
        Sphere(
            "0 0 0", 0.0508, f"{side}_swerve_module_wheel_collision", "2.255 0.001 0.01"
        )
    )

    model.motors.append(
        Motor(f"{side}_swerve_module_turret_motor", turret_hinge, 26.09)
    )
    model.motors.append(Motor(f"{side}_swerve_module_wheel_motor", wheel_hinge, 5.27))

    return module


def _build_note(name: str, pos: str, mesh: MeshAsset) -> Body:
    col = Mesh(
        mesh,
        type=Geom.Type.COLLISION,
        friction="1.2 0.005 0.0001",
        solref="0.04 1",
        solimp="0.8 0.99 0.001",
    )
    vis = Mesh(mesh, "orange", Geom.Type.VISUAL)
    out = Body(name, True, pos)
    out.geoms.append(col)
    out.geoms.append(vis)
    return out
