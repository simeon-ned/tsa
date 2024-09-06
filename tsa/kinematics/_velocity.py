from ..model import Model, Data
from ._position import motor_angle


def jacobian(
    model: Model,
    data: Data,
    theta: float | None = None,
    x: float | None = None,
) -> float:
    """Calculate jacobian as function of motor angle"""

    # if both theta and x are provided, use them
    if theta is not None and x is not None:
        # mixed version of jacobian
        return theta * (model.r**2) / (model.L - model.x)

    # if only theta is provided, use it
    if theta is not None:
        # motor jacobian
        return theta * (model.r**2) / (model.L**2 - (theta * model.r) ** 2) ** 0.5

    # if only x is provided, use it
    if x is not None:
        # compute theta for motor and then compute motor jacobian
        theta = motor_angle(model, data, x)
        return jacobian(model, data, theta=theta)

    # if neither theta nor x are provided, use the data
    if data.theta is not None and data.x is not None:
        return jacobian(model, data, data.theta, data.x)


def contraction_speed(
    model: Model,
    data: Data,
    x: float | None = None,
    dtheta: float | None = None,
) -> float:
    """Calculate contraction speed as function of position and motor speed"""
    x = data.x if x is None else x
    dtheta = data.dtheta if dtheta is None else dtheta

    data.x = x
    data.dtheta = dtheta
    data.dx = jacobian(model, data, x=x) * dtheta
    return data.dx


def motor_speed(
    model: Model,
    data: Data,
    theta: float | None = None,
    dx: float | None = None,
):
    """Calculate motor speed as function of position and contraction speed"""
    theta = data.theta if theta is None else theta
    dx = data.dx if dx is None else dx

    data.theta = theta
    data.dx = dx
    data.dtheta = dx / jacobian(model, data, theta=theta)
    return data.dtheta
