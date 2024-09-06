from ..model import Model, Data


def position_constraint(model: Model, data: Data, theta: float | None = None, x: float | None = None) -> float:
    """Holonomic constraint on the motor angle and contraction:
    c = f(theta, x)"""

    theta = data.theta if theta is None else theta
    x = data.x if x is None else x
    c = theta**2 * model.r**2 + (model.L - x) ** 2 - model.L**2
    return c


def velocity_constraint(
    model: Model,
    data: Data,
    theta: float | None = None,
    dtheta: float | None = None,
    x: float | None = None,
    dx: float | None = None,
) -> float:
    """Velocity constraint on the motor and contraction states:
    dcdt = f(theta, dtheta, x, dx)"""
    theta = data.theta if theta is None else theta
    dtheta = data.dtheta if dtheta is None else dtheta
    x = data.x if x is None else x
    dx = data.dx if dx is None else dx
    dc = theta * model.r**2 * dtheta - (model.L - x) * dx
    return dc


def acceleration_constraint(
    model: Model,
    data: Data,
    thetas: tuple[float, float, float],
    xs: tuple[float, float, float],
) -> float:
    """Acceleration constraint between motor and contraction motions,"""
    theta, dtheta, ddtheta = thetas
    x, dx, ddx = xs

    # update data values
    data.theta = theta
    data.dtheta = dtheta
    data.x = x
    data.dx = dx

    ddc = theta * model.r**2 * ddtheta + model.r**2 * dtheta**2 - (model.L - x) * ddx + dx**2
    return ddc
