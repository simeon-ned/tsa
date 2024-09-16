from dataclasses import dataclass

# TODO:
# Create class for SpaceData it will contain position, velocity, acceleration
#


@dataclass
class Data:
    theta: float = 0  # motor angle
    x: float = 0  # load position

    dtheta: float = 0  # motor speed
    dx: float = 0  # load speed
