from .._model import Model, Data
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
    """
    Calculate time derivative of the Jacobian with respect to time as a function of motor state.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. Defaults to None.
        dtheta (float | None, optional): The motor angular velocity in rad/s. Defaults to None.
        x (float | None, optional): The contraction in meters. Defaults to None.
        dx (float | None, optional): The contraction velocity in m/s. Defaults to None.

    Returns:
        float: The time derivative of the Jacobian.

    Note:
        This function does not update the data object. It uses the provided values or calculates
        missing values using other kinematic functions.
    """
    L, r = model.kinematic.length, model.kinematic.radius

    if theta is not None and dtheta is not None and x is not None and dx is not None:
        return r**2 * dtheta / (L - x) + theta * r**2 * dx / (L - x) ** 2

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

    raise ValueError("Insufficient data to calculate dJacobian")


def motor_acceleration(
    model: Model,
    data: Data,
    xs: tuple[float, float, float],
) -> float:
    """
    Calculate motor acceleration as a function of position and contraction kinematics.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        xs (tuple[float, float, float]): A tuple containing (x, dx, ddx).
            x: The contraction in meters.
            dx: The contraction velocity in m/s.
            ddx: The contraction acceleration in m/s^2.

    Returns:
        float: The calculated motor angular acceleration (d2theta) in rad/s^2.

    Note:
        This function updates data.x and data.dx with the input values.
        It does not update other fields of the data object.
    """
    x, dx, ddx = xs

    data.x = x
    data.dx = dx

    J = jacobian(model, data, x=data.x)

    theta = motor_angle(model, data)
    dtheta = dx / J
    d2theta = (ddx - djacobian(model, data, theta=theta, dtheta=dtheta, x=x, dx=dx)) / J

    return d2theta
