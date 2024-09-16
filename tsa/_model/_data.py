from dataclasses import dataclass, field
import casadi


# TODO: Should we mark space with ENUM value?
@dataclass
class SpaceData:
    position: float | casadi.SX = 0.0
    velocity: float | casadi.SX = 0.0
    acceleration: float | casadi.SX = 0.0
    force: float | casadi.SX = 0.0
    coriolis: float | casadi.SX = 0.0
    inertia: float | casadi.SX = 0.0
    nonlinear: float | casadi.SX = 0.0
    static: float | casadi.SX = 0.0
    jamming: float | casadi.SX = 0.0


@dataclass
class Data:
    motor: SpaceData = field(default_factory=SpaceData)
    load: SpaceData = field(default_factory=SpaceData)
