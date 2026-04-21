from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from typing import Protocol, override

from frc_python.units.distance import Distance
from frc_python.units.mass import Mass
from frc_python.utils.math import Vector3


class Stringable(Protocol):
    @override
    def __str__(self) -> str: ...


class Buildable(ABC):
    @abstractmethod
    def build(self, indentation: int = 0) -> str:
        pass


class LabelBuilder(Buildable):
    def __init__(self, name: str) -> None:
        self.name = name
        self.options: list[tuple[str, str]] = []
        self.children: list[Buildable] = []

    def add_optional(self, name: str, value: Stringable | None) -> None:
        if value is not None:
            self.add_option(name, value)

    def add_optionals(self, optionals: list[tuple[str, Stringable | None]]) -> None:
        for name, value in optionals:
            self.add_optional(name, value)

    def add_option(self, name: str, value: Stringable) -> None:
        self.options.append((name, str(value)))

    def add_options(self, options: list[tuple[str, Stringable]]) -> None:
        for name, value in options:
            self.add_option(name, value)

    def add_child(self, child: Buildable) -> None:
        self.children.append(child)

    def add_children(self, children: Sequence[Buildable]) -> None:
        self.children += children

    def with_optional(self, name: str, value: Stringable | None) -> LabelBuilder:
        self.add_optional(name, value)
        return self

    def with_optionals(
        self, optionals: list[tuple[str, Stringable | None]]
    ) -> LabelBuilder:
        self.add_optionals(optionals)
        return self

    def with_option(self, name: str, value: Stringable) -> LabelBuilder:
        self.add_option(name, value)
        return self

    def with_options(self, options: list[tuple[str, Stringable]]) -> LabelBuilder:
        self.add_options(options)
        return self

    def with_child(self, child: Buildable) -> LabelBuilder:
        self.add_child(child)
        return self

    def with_children(self, children: Sequence[Buildable]) -> LabelBuilder:
        self.add_children(children)
        return self

    @override
    def build(self, indentation: int = 0) -> str:
        out = f"{'  ' * indentation}<{self.name} "

        for name, value in self.options:
            out += f'{name}="{value}" '

        if len(self.children) == 0:
            out += "/>\n"
            return out
        else:
            out += ">\n"

        for child in self.children:
            out += child.build(indentation + 1)

        out += f"{'  ' * indentation}</{self.name}>\n"

        return out


class Material(Buildable):
    def __init__(self, name: str, r: float, g: float, b: float, a: float = 1.0) -> None:
        self.name = name
        self.rgba = f"{r} {g} {b} {a}"

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("material")
            .with_option("name", self.name)
            .with_option("rgba", self.rgba)
            .build(indentation)
        )


class MeshAsset(Buildable):
    def __init__(self, name: str, file: str) -> None:
        self.name = name
        self.file = file

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("mesh")
            .with_option("name", self.name)
            .with_option("file", self.file)
            .build(indentation)
        )


class Asset(Buildable):
    def __init__(self) -> None:
        self.materials: list[Material] = []
        self.meshes: list[MeshAsset] = []

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("asset")
            .with_child(
                LabelBuilder("texture").with_options(
                    [
                        ("name", "groundplane"),
                        ("type", "2d"),
                        ("builtin", "checker"),
                        ("rgb1", ".2 .3 .4"),
                        ("rgb2", ".1 .1 .1"),
                        ("width", 100),
                        ("height", 100),
                        ("mark", "cross"),
                        ("markrgb", ".8 .8 .8"),
                    ]
                )
            )
            .with_child(
                LabelBuilder("material").with_options(
                    [
                        ("name", "groundplane"),
                        ("texture", "groundplane"),
                        ("texrepeat", "5 5"),
                        ("texuniform", "true"),
                    ]
                )
            )
            .with_children(self.materials)
            .with_children(self.meshes)
            .build(indentation)
        )


class Geom(Buildable, ABC):
    class Type(Enum):
        VISUAL = "visual"
        COLLISION = "collision"

    def __init__(self, material: str = "white", type: Type = Type.VISUAL) -> None:
        self.material = material
        self.type = type


class Mesh(Geom):
    def __init__(
        self,
        mesh: MeshAsset,
        material: str = "white",
        type: Geom.Type = Geom.Type.VISUAL,
        axisangle: str | None = None,
        pos: str | None = None,
        friction: str | None = None,
        solref: str | None = None,
        solimp: str | None = None,
    ) -> None:
        super().__init__(material, type)
        self.mesh = mesh.name
        self.axisangle = axisangle
        self.pos = pos
        self.friction = friction
        self.solref = solref
        self.solimp = solimp

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("geom")
            .with_option("mesh", self.mesh)
            .with_option("class", self.type.value)
            .with_option("material", self.material)
            .with_optional("axisangle", self.axisangle)
            .with_optional("pos", self.pos)
            .with_optionals(
                [
                    ("friction", self.friction),
                    ("solref", self.solref),
                    ("solimp", self.solimp),
                ]
            )
            .build(indentation)
        )


class Box(Geom):
    def __init__(
        self,
        pos: str,
        size: str,
        name: str | None = None,
        friction: str | None = None,
        material: str = "white",
        type: Geom.Type = Geom.Type.COLLISION,
        solimp: str | None = None,
        solref: str | None = None,
    ) -> None:
        super().__init__(material, type)
        self.name = name
        self.pos = pos
        self.size = size
        self.friction = friction
        self.solimp = solimp
        self.solref = solref

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("geom")
            .with_optionals(
                [
                    ("name", self.name),
                    ("type", "box"),
                    ("pos", self.pos),
                    ("size", self.size),
                    ("class", self.type.value),
                    ("material", self.material),
                    ("friction", self.friction),
                    ("solimp", self.solimp),
                    ("solref", self.solref),
                ]
            )
            .build(indentation)
        )


class Sphere(Geom):
    def __init__(
        self,
        pos: str,
        radius: float,
        name: str | None = None,
        friction: str | None = None,
    ) -> None:
        super().__init__(type=Geom.Type.COLLISION)
        self.pos = pos
        self.name = name
        self.radius = radius
        self.friction = friction

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("geom")
            .with_optionals(
                [
                    ("name", self.name),
                    ("type", "sphere"),
                    ("pos", self.pos),
                    ("size", self.radius),
                    ("class", self.type.value),
                    ("material", self.material),
                    ("friction", self.friction),
                ]
            )
            .build(indentation)
        )


class MeshDeformable(Buildable):
    class DegOF(Enum):
        FULL = "full"
        RADIAL = "radial"
        TRILINEAR = "trilinear"
        QUADRATIC = "quadratic"

    def __init__(
        self,
        name: str,
        mass: Mass,
        file: str,
        pos: str | None = None,
        dof: DegOF = DegOF.TRILINEAR,
        friction: str | None = None,
        radius: float | None = None,
        young: float | None = None,
        poisson: float | None = None,
        damping: float | None = None,
        thickness: float | None = None,
        solref: str | None = None,
        solimp: str | None = None,
        contype: int | None = None,
        conaffinity: int | None = None,
    ) -> None:
        self.name = name
        self.file = file
        self.mass = mass.kilograms
        self.pos = pos
        self.dof = dof
        self.friction = friction
        self.radius = radius
        self.young = young
        self.poisson = poisson
        self.damping = damping
        self.thickness = thickness
        self.solref = solref
        self.solimp = solimp
        self.contype = contype
        self.conaffinity = conaffinity

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("flexcomp")
            .with_option("type", "gmsh")
            .with_optionals(
                [
                    ("name", self.name),
                    ("mass", self.mass),
                    ("dof", self.dof.value),
                    ("pos", self.pos),
                    ("file", self.file),
                    ("radius", self.radius),
                ]
            )
            .with_child(
                LabelBuilder("contact")
                .with_optionals(
                    [
                        ("friction", self.friction),
                        ("solref", self.solref),
                        ("solimp", self.solimp),
                        ("contype", self.contype),
                        ("conaffinity", self.conaffinity),
                    ]
                )
                .with_option("selfcollide", "none")
                .with_option("internal", "false")
            )
            .with_child(
                LabelBuilder("elasticity").with_optionals(
                    [
                        ("young", self.young),
                        ("poisson", self.poisson),
                        ("damping", self.damping),
                        ("thickness", self.thickness),
                    ]
                )
            )
            .with_child(
                LabelBuilder("edge")
                .with_option("equality", "true")
                .with_optional("damping", self.damping)
            )
            .build(indentation)
        )


class Inertial(Buildable):
    def __init__(
        self,
        mass: Mass,
        pos: Vector3[Distance] | None = None,
        diaginertia: str | None = None,
    ) -> None:
        self.pos = (
            "0 0 0"
            if pos is None
            else f"{pos.x.meters()} {pos.y.meters()} {pos.z.meters()}"
        )
        self.mass = mass.kilograms
        self.diaginertia = diaginertia

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("inertial")
            .with_option("pos", self.pos)
            .with_option("mass", self.mass)
            .with_optional("diaginertia", self.diaginertia)
            .build(indentation)
        )


class Site(Buildable):
    def __init__(self, name: str, pos: Vector3[Distance] | None = None) -> None:
        self.name = name
        self.pos = (
            "0 0 0"
            if pos is None
            else f"{pos.x.meters()} {pos.y.meters()} {pos.z.meters()}"
        )

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("site")
            .with_option("name", self.name)
            .with_option("pos", self.pos)
            .build(indentation)
        )


class Joint(Buildable):
    class Type(Enum):
        SLIDE = "slide"
        HINGE = "hinge"

    class Axis(Enum):
        X = "1 0 0"
        Y = "0 1 0"
        Z = "0 0 1"

    def __init__(
        self,
        name: str,
        type: Type,
        axis: Axis,
        pos: str | None = None,
        stiffness: float | None = None,
        damping: float | None = None,
        armature: float | None = None,
        springref: float | None = None,
    ) -> None:
        self.name = name
        self.type = type
        self.axis = axis
        self.pos = "0 0 0" if pos is None else pos
        self.stiffness = stiffness
        self.damping = damping
        self.armature = armature
        self.springref = springref

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("joint")
            .with_optionals(
                [
                    ("name", self.name),
                    ("type", self.type.value),
                    ("axis", self.axis.value),
                    ("stiffness", self.stiffness),
                    ("damping", self.damping),
                    ("armature", self.armature),
                    ("springref", self.springref),
                ]
            )
            .build(indentation)
        )


class Body(Buildable):
    def __init__(self, name: str, free: bool = False, pos: str | None = None) -> None:
        self.name = name
        self.free = free
        self.pos = pos
        self.inertials: list[Inertial] = []
        self.sites: list[Site] = []
        self.joints: list[Joint] = []
        self.geoms: list[Geom] = []
        self.bodies: list[Body] = []

    @override
    def build(self, indentation: int = 0) -> str:
        out = (
            LabelBuilder("body")
            .with_option("name", self.name)
            .with_optional("pos", self.pos)
        )
        if self.free:
            out.add_child(LabelBuilder("freejoint"))
        return (
            out.with_children(self.inertials)
            .with_children(self.sites)
            .with_children(self.joints)
            .with_children(self.bodies)
            .with_children(self.geoms)
            .build(indentation)
        )


class World(Buildable):
    def __init__(self) -> None:
        self.geoms: list[Geom] = []
        self.bodies: list[Body] = []
        self.deformables: list[MeshDeformable] = []

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("worldbody")
            .with_child(
                LabelBuilder("light").with_options(
                    [
                        ("pos", "1 -1 1.5"),
                        ("dir", "-1 1 -1"),
                        ("diffuse", "0.5 0.5 0.5"),
                        ("directional", "true"),
                    ]
                )
            )
            .with_child(
                LabelBuilder("geom").with_options(
                    [
                        ("name", "floor"),
                        ("size", "0 0 0.05"),
                        ("type", "plane"),
                        ("material", "groundplane"),
                        ("friction", "1.3 0.1 0.01"),
                    ]
                )
            )
            .with_children(self.geoms)
            .with_children(self.bodies)
            .with_children(self.deformables)
            .build(indentation)
        )


class Motor(Buildable):
    def __init__(self, name: str, joint: Joint, gear_ratio: float) -> None:
        self.name = name
        self.joint = joint
        self.gear_ratio = gear_ratio

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("motor")
            .with_options(
                [
                    ("name", self.name),
                    ("joint", self.joint.name),
                    ("gear", self.gear_ratio),
                ]
            )
            .build(indentation)
        )


class Gyro(Buildable):
    def __init__(self, name: str, site: Site) -> None:
        self.name = name
        self.site = site

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("gyro")
            .with_option("name", self.name)
            .with_option("site", self.site.name)
            .build(indentation)
        )


class Plugin(Buildable):
    def __init__(self, plugin: str) -> None:
        self.plugin = plugin

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("plugin").with_option("plugin", self.plugin).build(indentation)
        )


class Model(Buildable):
    class Integrator(Enum):
        EULER = "euler"
        IMPLICIT = "implicit"
        IMPLICITFAST = "implicitfast"
        RK4 = "RK4"

    class Cone(Enum):
        PYRAMIDAL = "pyramidal"
        ELLIPTIC = "elliptic"

    def __init__(self, name: str) -> None:
        self.name = name
        self.timestep = 0.001
        self.integrator = Model.Integrator.IMPLICITFAST
        self.cone = Model.Cone.ELLIPTIC

        self.air_density: float | None = None
        self.air_viscosity: float | None = None
        self.sdf_iterations: int | None = None
        self.sdf_initpoints: int | None = None

        self.plugins: list[Plugin] = []
        self.asset = Asset()
        self.world = World()
        self.motors: list[Motor] = []
        self.gyros: list[Gyro] = []

    @override
    def build(self, indentation: int = 0) -> str:
        return (
            LabelBuilder("mujoco")
            .with_option("model", self.name)
            .with_child(LabelBuilder("extension").with_children(self.plugins))
            .with_child(
                LabelBuilder("compiler")
                .with_option("angle", "radian")
                .with_option("coordinate", "local")
            )
            .with_child(
                LabelBuilder("option").with_optionals(
                    [
                        ("timestep", self.timestep),
                        ("integrator", self.integrator.value),
                        ("cone", self.cone.value),
                        ("density", self.air_density),
                        ("viscosity", self.air_viscosity),
                        ("sdf_iterations", self.sdf_iterations),
                        ("sdf_initpoints", self.sdf_initpoints),
                    ]
                )
            )
            .with_child(
                LabelBuilder("size")
                .with_option("njmax", 20000)
                .with_option("nconmax", 10000)
            )
            .with_child(
                LabelBuilder("default").with_children(
                    [
                        LabelBuilder("default")
                        .with_option("class", "visual")
                        .with_child(
                            LabelBuilder("geom").with_options(
                                [
                                    ("type", "mesh"),
                                    ("group", 2),
                                    ("contype", 0),
                                    ("conaffinity", 0),
                                ]
                            )
                        ),
                        LabelBuilder("default")
                        .with_option("class", "collision")
                        .with_child(
                            LabelBuilder("geom").with_options(
                                [
                                    ("type", "mesh"),
                                    ("group", 3),
                                    ("contype", 1),
                                    ("conaffinity", 1),
                                ]
                            )
                        ),
                    ]
                )
            )
            .with_child(self.asset)
            .with_child(self.world)
            .with_child(LabelBuilder("actuator").with_children(self.motors))
            .with_child(LabelBuilder("sensor").with_children(self.gyros))
            .build(indentation)
        )
