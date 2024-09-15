from dataclasses import dataclass


@dataclass
class Data:
    theta: float = 0  # motor angle
    x: float = 0  # load position

    dtheta: float = 0  # motor speed
    dx: float = 0  # load speed
