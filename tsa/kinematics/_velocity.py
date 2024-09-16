from .._structs import Model, Data
from ._position import motor_angle


def jacobian(
    model: Model,
    data: Data,
    theta: float | None = None,
    x: float | None = None,
) -> float:
    """
    Calculate the Jacobian as a function of motor angle or contraction.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. Defaults to None.
        x (float | None, optional): The contraction in meters. Defaults to None.

    Returns:
        float: The calculated Jacobian.

    Note:
        This function uses the provided theta and x if available, otherwise it uses the values stored in data.
        The function does not update the data object.
    """
    L, r = model.kinematic.length, model.kinematic.radius

    if theta is not None and x is not None:
        data.motor.jacobian = theta * (r**2) / (L - x)
        return data.motor.jacobian

    if theta is not None:
        data.motor.jacobian = theta * (r**2) / (L**2 - (theta * r) ** 2) ** 0.5
        return data.motor.jacobian

    if x is not None:
        theta = motor_angle(model, data, x)
        data.motor.jacobian = jacobian(model, data, theta=theta)
        return data.motor.jacobian

    if data.motor.position is not None and data.load.position is not None:
        return jacobian(model, data, data.motor.position, data.load.position)

    raise ValueError("Insufficient data to calculate Jacobian")


def contraction_speed(
    model: Model,
    data: Data,
    x: float | None = None,
    dtheta: float | None = None,
) -> float:
    """
    Calculate contraction speed as a function of position and motor speed.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        x (float | None, optional): The contraction in meters. If None, uses the value stored in data. Defaults to None.
        dtheta (float | None, optional): The motor angular velocity in rad/s. If None, uses the value stored in data. Defaults to None.

    Returns:
        float: The calculated contraction speed (dx) in m/s.

    Note:
        This function updates data.load.position, data.motor.velocity, and data.load.velocity with the input and calculated values respectively.
    """
    x = data.load.position if x is None else x
    dtheta = data.motor.velocity if dtheta is None else dtheta

    data.load.position = x
    data.motor.velocity = dtheta
    data.load.velocity = jacobian(model, data, x=x) * dtheta
    return data.load.velocity


def motor_speed(
    model: Model,
    data: Data,
    theta: float | None = None,
    dx: float | None = None,
) -> float:
    """
    Calculate motor speed as a function of position and contraction speed.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. If None, uses the value stored in data. Defaults to None.
        dx (float | None, optional): The contraction speed in m/s. If None, uses the value stored in data. Defaults to None.

    Returns:
        float: The calculated motor angular velocity (dtheta) in rad/s.

    Note:
        This function updates data.motor.position, data.load.velocity, and data.motor.velocity with the input and calculated values respectively.
    """
    theta = data.motor.position if theta is None else theta
    dx = data.load.velocity if dx is None else dx

    data.motor.position = theta
    data.load.velocity = dx
    data.motor.velocity = dx / jacobian(model, data, theta=theta)
    return data.motor.velocity
