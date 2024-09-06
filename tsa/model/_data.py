from dataclasses import dataclass


@dataclass
class Data:
    theta: float  # motor angle
    x: float  # load position

    dtheta: float  # motor speed
    dx: float  # load speed
