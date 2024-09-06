from dataclasses import dataclass


@dataclass
class Model:
    # kinematic parameters
    r: float  # string radius
    L: float  # string length

    # dynamic parameters
    mass: float  # load mass
    inertia: float  # motor inertia
    b_x: float  # load viscous friction
    tau_c: float  # motor coloumb friction

    b_th: float  # motor viscous friction
    F_c: float  # load coloumb friction

    Kr: float  # transverse stiffness
    Kl: float  # longitudal stiffness
