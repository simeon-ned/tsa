from dataclasses import dataclass, field
import casadi


@dataclass
class KinematicParameters:
    radius: float | casadi.SX = 0.001  # string radius
    length: float | casadi.SX = 0.2  # string length


@dataclass
class SpaceParameters:
    inertia: float | casadi.SX  # inertia
    damping: float | casadi.SX  # viscous damping
    friction: float | casadi.SX  # coulomb friction


@dataclass
class DynamicParameters:
    motor: SpaceParameters = field(
        default_factory=lambda: SpaceParameters(
            inertia=1e-6,  # motor inertia
            damping=0.1,  # motor viscous damping
            friction=0.05,  # motor coulomb friction
        )
    )
    load: SpaceParameters = field(
        default_factory=lambda: SpaceParameters(
            inertia=1.0,  # load inertia (equivalent to mass)
            damping=0.01,  # load viscous damping
            friction=0.1,  # load coulomb friction
        )
    )


@dataclass
class StiffnessParameters:
    transverse: float | casadi.SX = 1000.0  # transverse stiffness
    longitudinal: float | casadi.SX = 5000.0  # longitudinal stiffness


@dataclass
class Model:
    kinematic: KinematicParameters = field(default_factory=KinematicParameters)
    dynamic: DynamicParameters = field(default_factory=DynamicParameters)
    stiffness: StiffnessParameters = field(default_factory=StiffnessParameters)
