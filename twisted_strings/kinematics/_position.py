"""
Position-related kinematics calculations for Twisted String Actuators (TSA).

This module provides functions for computing the basic kinematic relationships
between motor angle and load position (contraction) in a TSA system. The main
functions are:
- contraction: Computes load position from motor angle
- motor_angle: Computes motor angle from load position

All functions update the provided Data object with computed values and maintain
consistency between motor and load spaces.
"""

from .._structs import Model, Data


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
    if model.kinematic.length <= 0 or model.kinematic.radius <= 0:
        raise ValueError("Invalid model parameters: length and radius must be positive.")

    theta = data.motor.position if theta is None else theta
    if theta < 0:
        raise ValueError("Invalid motor angle: theta must be non-negative.")

    L, r = model.kinematic.length, model.kinematic.radius
    data.motor.position = theta
    contraction = L - (L**2 - (theta * r) ** 2) ** 0.5

    if contraction < 0 or contraction > L:
        raise ValueError(f"Calculated contraction {contraction} is outside valid range [0, {L}].")

    data.load.position = contraction
    return contraction


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
    if x < 0:
        raise ValueError("Invalid contraction: x must be non-negative.")

    L, r = model.kinematic.length, model.kinematic.radius

    data.load.position = x
    data.motor.position = (L**2 - (L - x) ** 2) ** 0.5 / r
    return data.motor.position
