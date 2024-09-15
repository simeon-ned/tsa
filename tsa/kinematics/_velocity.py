from ..model import Model, Data
from ._position import motor_angle


# TODO: Think if we need to add space as argument
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
        return theta * (r**2) / (L - x)

    if theta is not None:
        return theta * (r**2) / (L**2 - (theta * r) ** 2) ** 0.5

    if x is not None:
        theta = motor_angle(model, data, x)
        return jacobian(model, data, theta=theta)

    if data.theta is not None and data.x is not None:
        return jacobian(model, data, data.theta, data.x)

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
        This function updates data.x, data.dtheta, and data.dx with the input and calculated values respectively.
    """
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
        This function updates data.theta, data.dx, and data.dtheta with the input and calculated values respectively.
    """
    theta = data.theta if theta is None else theta
    dx = data.dx if dx is None else dx

    data.theta = theta
    data.dx = dx
    data.dtheta = dx / jacobian(model, data, theta=theta)
    return data.dtheta
