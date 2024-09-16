from dataclasses import dataclass, field
import casadi


@dataclass
class KinematicParameters:
    """
    Kinematic parameters for the twisted string actuator.

    Attributes:
        radius (float | casadi.SX): String radius in meters. Defaults to 0.001.
        length (float | casadi.SX): String length in meters. Defaults to 0.2.
    """

    radius: float | casadi.SX = 0.001
    length: float | casadi.SX = 0.2


@dataclass
class SpaceParameters:
    """
    Parameters for a specific space (motor or load) in the twisted string actuator system.

    Attributes:
        inertia (float | casadi.SX): Inertia of the space.
        damping (float | casadi.SX): Viscous damping coefficient.
        friction (float | casadi.SX): Coulomb friction coefficient.
    """

    inertia: float | casadi.SX
    damping: float | casadi.SX
    friction: float | casadi.SX


@dataclass
class DynamicParameters:
    """
    Dynamic parameters for both motor and load spaces in the twisted string actuator system.

    Attributes:
        motor (SpaceParameters): Motor space parameters.
        load (SpaceParameters): Load space parameters.
    """

    motor: SpaceParameters = field(
        default_factory=lambda: SpaceParameters(
            inertia=1e-6,
            damping=0.1,
            friction=0.05,
        )
    )
    load: SpaceParameters = field(
        default_factory=lambda: SpaceParameters(
            inertia=1.0,
            damping=0.01,
            friction=0.1,
        )
    )


@dataclass
class StiffnessParameters:
    """
    Stiffness parameters for the twisted string actuator.

    Attributes:
        transverse (float | casadi.SX): Transverse stiffness coefficient. Defaults to 1000.0.
        longitudinal (float | casadi.SX): Longitudinal stiffness coefficient. Defaults to 5000.0.
    """

    transverse: float | casadi.SX = 1000.0
    longitudinal: float | casadi.SX = 5000.0


@dataclass
class Model:
    """
    Complete model of the twisted string actuator system.

    This class aggregates all parameters necessary to describe the kinematics,
    dynamics, and stiffness properties of a twisted string actuator.

    Attributes:
        kinematic (KinematicParameters): Kinematic parameters of the system.
        dynamic (DynamicParameters): Dynamic parameters of the system.
        stiffness (StiffnessParameters): Stiffness parameters of the system.
    """

    kinematic: KinematicParameters = field(default_factory=KinematicParameters)
    dynamic: DynamicParameters = field(default_factory=DynamicParameters)
    stiffness: StiffnessParameters = field(default_factory=StiffnessParameters)
