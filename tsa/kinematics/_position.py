from .._model import Model, Data


def contraction(model: Model, data: Data, theta: float | None = None) -> float:
    """
    Calculate contraction as a function of motor angle.

    This function computes the contraction of the string based on the motor angle.
    It updates the data object with the calculated values.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. If None, uses the value stored in data. Defaults to None.

    Returns:
        float: The calculated contraction (x) in meters.

    Note:
        This function updates data.motor.position and data.load.position with the input and calculated values respectively.
    """
    theta = data.motor.position if theta is None else theta
    L, r = model.kinematic.length, model.kinematic.radius
    data.motor.position = theta
    data.load.position = L - (L**2 - (theta * r) ** 2) ** 0.5
    return data.load.position


def motor_angle(model: Model, data: Data, x: float | None = None) -> float:
    """
    Calculate motor angle as a function of contraction.

    This function computes the motor angle based on the contraction of the string.
    It updates the data object with the calculated values.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        x (float | None, optional): The contraction in meters. If None, uses the value stored in data. Defaults to None.

    Returns:
        float: The calculated motor angle (theta) in radians.

    Note:
        This function updates data.load.position and data.motor.position with the input and calculated values respectively.
    """
    x = data.load.position if x is None else x
    L, r = model.kinematic.length, model.kinematic.radius

    data.load.position = x
    data.motor.position = (L**2 - (L - x) ** 2) ** 0.5 / r
    return data.motor.position
