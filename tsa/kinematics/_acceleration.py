from ..model import Model, Data
from ._position import contraction, motor_angle
from ._velocity import contraction_speed, motor_speed, jacobian


def djacobian(
    model: Model,
    data: Data,
    theta: float | None = None,
    dtheta: float | None = None,
    x: float | None = None,
    dx: float | None = None,
) -> float:
    """Calculate time derevetive of the jacobian with respect to time
    as function of motor state"""
    if theta is not None and dtheta is not None and x is not None and dx is not None:
        return model.r**2 * dtheta / (model.L - x) + theta * model.r**2 * dx / (model.L - x) ** 2

    if theta is not None and dtheta is not None:
        x = contraction(model, data, theta)
        dx = contraction_speed(model, data, x, dtheta)
        return djacobian(model, data, theta, dtheta, x, dx)

    if x is not None and dx is not None:
        theta = motor_angle(model, data, x)
        dtheta = motor_speed(model, data, theta, dx)
        return djacobian(model, data, theta, dtheta, x, dx)

    if data.theta is not None and data.dtheta is not None and data.x is not None and data.dx is not None:
        return djacobian(model, data, data.theta, data.dtheta, data.x, data.dx)


def motor_acceleration(
    model: Model,
    data: Data,
    xs: tuple[float, float, float],
) -> float:
    """Calculate motor acceleration as function of position and contraction speed"""
    x, dx, ddx = xs

    data.x = x
    data.dx = dx

    J = jacobian(model, data, x=data.x)

    theta = motor_angle(model, data)
    dtheta = dx / J
    d2theta = (ddx - djacobian(model, data, theta=theta, dtheta=dtheta)) / J

    return d2theta
