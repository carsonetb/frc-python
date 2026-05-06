from abc import ABC, abstractmethod
from random import uniform
from typing import override

from frc_python.sim.xmlgen import Body, Box, Geom, Mesh, MeshAsset, Model


class SimField(ABC):
    NAME = ""

    @abstractmethod
    def build(self, model: Model) -> None:
        pass


class CrescendoField(SimField):
    NAME = "Crescendo"

    def build_note(self, name: str, pos: str, mesh: MeshAsset) -> Body:
        col = Mesh(mesh, type=Geom.Type.COLLISION, friction="1.2 0.005 0.0001", solref="0.025 1", solimp="0.95 0.99 0.001")
        vis = Mesh(mesh, "orange", Geom.Type.VISUAL)
        out = Body(name, True, pos)
        out.geoms.append(col)
        out.geoms.append(vis)
        return out

    @override
    def build(self, model: Model) -> None:
        geoms = model.world.geoms
        meshes = model.asset.meshes

        audience_side_field_wall = MeshAsset("audience_side_field_wall", "frc_python/resources/AudienceSideFieldWall.stl")
        scoring_side_field_wall = MeshAsset("scoring_side_field_wall", "frc_python/resources/ScoringSideFieldWall.stl")
        red_alliance_driver_station = MeshAsset("red_alliance_driver_station", "frc_python/resources/RedAllianceDriverStation.stl")
        blue_alliance_driver_station = MeshAsset("blue_alliance_driver_station", "frc_python/resources/BlueAllianceDriverStation.stl")
        red_speaker = MeshAsset("red_speaker", "frc_python/resources/RedSpeaker.stl")
        red_source = MeshAsset("red_source", "frc_python/resources/RedSource.stl")
        red_stage = MeshAsset("red_stage", "frc_python/resources/RedStage.stl")
        red_amp = MeshAsset("red_amp", "frc_python/resources/RedAmp.stl")
        blue_speaker = MeshAsset("blue_speaker", "frc_python/resources/BlueSpeaker.stl")
        blue_source = MeshAsset("blue_source", "frc_python/resources/BlueSource.stl")
        blue_stage = MeshAsset("blue_stage", "frc_python/resources/BlueStage.stl")
        blue_amp = MeshAsset("blue_amp", "frc_python/resources/BlueAmp.stl")

        red_speaker_ll_wall_collision = MeshAsset(
            "red_speaker_lower_left_wall_collision", "frc_python/resources/collisions/RedSpeakerLowerLeftWall.stl"
        )
        red_speaker_lr_wall_collision = MeshAsset(
            "red_speaker_lower_right_wall_collision", "frc_python/resources/collisions/RedSpeakerLowerRightWall.stl"
        )
        red_source_wall_collision = MeshAsset("red_source_wall_collision", "frc_python/resources/collisions/RedSourceWall.stl")
        red_stage_collision_1 = MeshAsset("red_stage_collision_1", "frc_python/resources/collisions/RedStage1.stl")
        red_stage_collision_2 = MeshAsset("red_stage_collision_2", "frc_python/resources/collisions/RedStage2.stl")
        red_stage_collision_3 = MeshAsset("red_stage_collision_3", "frc_python/resources/collisions/RedStage2.stl")
        blue_speaker_ll_wall_collision = MeshAsset(
            "blue_speaker_lower_left_wall_collision", "frc_python/resources/collisions/BlueSpeakerLowerLeftWall.stl"
        )
        blue_speaker_lr_wall_collision = MeshAsset(
            "blue_speaker_lower_right_wall_collision", "frc_python/resources/collisions/BlueSpeakerLowerRightWall.stl"
        )
        blue_source_wall_collision = MeshAsset("blue_source_wall_collision", "frc_python/resources/collisions/BlueSourceWall.stl")
        blue_stage_collision_1 = MeshAsset("blue_stage_collision_1", "frc_python/resources/collisions/BlueStage1.stl")
        blue_stage_collision_2 = MeshAsset("blue_stage_collision_2", "frc_python/resources/collisions/BlueStage2.stl")
        blue_stage_collision_3 = MeshAsset("blue_stage_collision_3", "frc_python/resources/collisions/BlueStage3.stl")
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
        geoms.append(Mesh(carpet, friction="1.3 0.1 0.01"))
        geoms.append(Mesh(red_tape))
        geoms.append(Mesh(blue_tape))
        geoms.append(Mesh(white_tape))

        geoms.append(Box("0 -4.206 0.2615", "8.268 0.1 0.241", "audience_side_field_wall_collision", friction="0.3 0.005 0.0001"))
        geoms.append(Box("0 4.206 0.2615", "8.268 0.1 0.241", "scoring_side_field_wall_collision", friction="0.3 0.005 0.0001"))
        geoms.append(Box("8.2955 1.163 0.9955", "0.0245 1.84 1.981", "red_alliance_driver_station_collision_large", friction="0.3 0.005 0.0001"))
        geoms.append(Box("8.2975 -3.144 0.995", "0.0225 0.9375 0.99", "red_alliance_driver_station_collision_small", friction="0.3 0.005 0.0001"))
        geoms.append(Box("-8.2955 0.9375 0.9955", "0.0245 1.84 1.981", "blue_alliance_driver_station_collision_large", friction="0.3 0.005 0.0001"))
        geoms.append(Box("-8.2975 -3.144 0.995", "0.0225 0.9375 0.99", "blue_alliance_driver_station_collision_small", friction="0.3 0.005 0.0001"))
        geoms.append(Box("-7.355 -1.442 0.111", "0.01 0.521 0.1", "red_alliance_speaker_low_wall_flat", friction="0.3 0.005 0.0001"))
        geoms.append(Box("7.355 -1.442 0.111", "0.01 0.521 0.1", "blu_alliance_speaker_low_wall_flat", friction="0.3 0.005 0.0001"))

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
            model.world.bodies.append(self.build_note(f"{uniform(0, 1000)}", f"{uniform(-4, 4)} {uniform(-4, 4)} {uniform(5, 10)}", note))
