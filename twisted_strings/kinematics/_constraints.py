from .._structs import Model, Data


def position_constraint(model: Model, data: Data, theta: float | None = None, x: float | None = None) -> float:
    """
    Calculate the holonomic constraint on the motor angle and contraction.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. If None, uses the value stored in data. Defaults to None.
        x (float | None, optional): The contraction in meters. If None, uses the value stored in data. Defaults to None.

    Returns:
        float: The value of the constraint function c = f(theta, x).

    Note:
        This function does not update the data object.
    """
    theta = data.motor.position if theta is None else theta
    x = data.load.position if x is None else x
    L, r = model.kinematic.length, model.kinematic.radius
    c = theta**2 * r**2 + (L - x) ** 2 - L**2
    return c


def velocity_constraint(
    model: Model,
    data: Data,
    theta: float | None = None,
    dtheta: float | None = None,
    x: float | None = None,
    dx: float | None = None,
) -> float:
    """
    Calculate the velocity constraint on the motor and contraction states.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. If None, uses the value stored in data. Defaults to None.
        dtheta (float | None, optional): The motor angular velocity in rad/s. If None, uses the value stored in data. Defaults to None.
        x (float | None, optional): The contraction in meters. If None, uses the value stored in data. Defaults to None.
        dx (float | None, optional): The contraction velocity in m/s. If None, uses the value stored in data. Defaults to None.

    Returns:
        float: The value of the velocity constraint function dcdt = f(theta, dtheta, x, dx).

    Note:
        This function does not update the data object.
    """
    theta = data.motor.position if theta is None else theta
    dtheta = data.motor.velocity if dtheta is None else dtheta
    x = data.load.position if x is None else x
    dx = data.load.velocity if dx is None else dx
    L, r = model.kinematic.length, model.kinematic.radius
    dc = theta * r**2 * dtheta - (L - x) * dx
    return dc


def acceleration_constraint(
    model: Model,
    data: Data,
    thetas: tuple[float, float, float],
    xs: tuple[float, float, float],
) -> float:
    """
    Calculate the acceleration constraint between motor and contraction motions.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        thetas (tuple[float, float, float]): A tuple containing (theta, dtheta, ddtheta).
            theta: The motor angle in radians.
            dtheta: The motor angular velocity in rad/s.
            ddtheta: The motor angular acceleration in rad/s^2.
        xs (tuple[float, float, float]): A tuple containing (x, dx, ddx).
            x: The contraction in meters.
            dx: The contraction velocity in m/s.
            ddx: The contraction acceleration in m/s^2.

    Returns:
        float: The value of the acceleration constraint function.

    Note:
        This function updates data.motor.position, data.motor.velocity, data.load.position, and data.load.velocity with the input values.
    """
    theta, dtheta, ddtheta = thetas
    x, dx, ddx = xs

    # update data values
    data.motor.position = theta
    data.motor.velocity = dtheta
    data.load.position = x
    data.load.velocity = dx

    L, r = model.kinematic.length, model.kinematic.radius
    ddc = theta * r**2 * ddtheta + r**2 * dtheta**2 - (L - x) * ddx + dx**2
    return ddc
