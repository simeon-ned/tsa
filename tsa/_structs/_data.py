from dataclasses import dataclass, field
import casadi
from enum import IntEnum


class Space(IntEnum):
    MOTOR = 1
    LOAD = 2


@dataclass
class SpaceData:
    """
    Data structure for storing state and dynamic information for a specific space (motor or load) in the twisted string actuator system.

    Attributes:
        position (float | casadi.SX): Position of the space. Defaults to 0.0.
        velocity (float | casadi.SX): Velocity of the space. Defaults to 0.0.
        acceleration (float | casadi.SX): Acceleration of the space. Defaults to 0.0.
        jacobian (float | casadi.SX): Jacobian element for the space. Defaults to 0.0.
        djacobian (float | casadi.SX): Time derivative of the Jacobian for the space. Defaults to 0.0.
        force (float | casadi.SX): Force applied to or generated by the space. Defaults to 0.0.
        coriolis (float | casadi.SX): Coriolis term for the space. Defaults to 0.0.
        inertia (float | casadi.SX): Inertia term for the space. Defaults to 0.0.
        nonlinear (float | casadi.SX): Nonlinear term for the space. Defaults to 0.0.
        static (float | casadi.SX): Static term for the space. Defaults to 0.0.
        jamming (float | casadi.SX): Jamming effect term for the space. Defaults to 0.0.
    """

    position: float | casadi.SX = 0.0
    velocity: float | casadi.SX = 0.0
    acceleration: float | casadi.SX = 0.0
    jacobian: float | casadi.SX = 0.0
    djacobian: float | casadi.SX = 0.0
    force: float | casadi.SX = 0.0
    coriolis: float | casadi.SX = 0.0
    inertia: float | casadi.SX = 0.0
    nonlinear: float | casadi.SX = 0.0
    static: float | casadi.SX = 0.0
    jamming: float | casadi.SX = 0.0


@dataclass
class Data:
    """
    Data structure for storing state and dynamic information for both motor and load spaces in the twisted string actuator system.

    Attributes:
        motor (SpaceData): Data for the motor space.
        load (SpaceData): Data for the load space.
    """

    motor: SpaceData = field(default_factory=SpaceData)
    load: SpaceData = field(default_factory=SpaceData)


# TODO: Should we mark space with ENUM value?
# Maybe it will be easier to separate kinematics and dynamics in data?
