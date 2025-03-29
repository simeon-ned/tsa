"""
Kinematics Module for Twisted String Actuators

This module provides functions for calculating various kinematic components
of twisted string actuators, including position, velocity, and acceleration
transformations between motor and load spaces.

The models and calculations in this module are based on the research
presented in the following papers (and others):

- Gaponov I, Popov D, Ryu JH. Twisted string actuation systems: A study of
   the mathematical model and a comparison of twisted strings. IEEE/ASME
   Transactions on mechatronics. 2013 Sep 20;19(4):1331-42.

- Nedelchev S, Kirsanov D, Gaponov I. IMU-based Parameter Identification and
   Position Estimation in Twisted String Actuators. In2020 IEEE/RSJ International
   Conference on Intelligent Robots and Systems (IROS) 2020 Oct 24 (pp. 6311-6317). IEEE.
"""

from ._acceleration import djacobian, motor_acceleration
from ._position import contraction, motor_angle
from ._velocity import contraction_speed, jacobian, motor_speed
from ._constraints import position_constraint, velocity_constraint, acceleration_constraint
from .._structs import Model, Data


def compute_all(model: Model, data: Data, theta: float | None = None, x: float | None = None):
    """
    Compute all kinematic properties based on either motor angle or contraction.

    Args:
        model (Model): The model object containing kinematic parameters.
        data (Data): The data object to store and retrieve state variables.
        theta (float | None, optional): The motor angle in radians. Defaults to None.
        x (float | None, optional): The contraction in meters. Defaults to None.

    Note:
        Either theta or x must be provided. If both are provided, theta takes precedence.
        This function updates all relevant fields in the data object.
    """
    if theta is not None:
        contraction(model, data, theta)
    elif x is not None:
        motor_angle(model, data, x)
    else:
        raise ValueError("Either theta or x must be provided.")

    jacobian(model, data)
    djacobian(model, data)

    # Compute velocities and accelerations if data is available
    if data.motor.velocity is not None:
        contraction_speed(model, data)
    elif data.load.velocity is not None:
        motor_speed(model, data)

    #  if data.motor.acceleration is not None:
    #      motor_acceleration(model, data)

    # Compute constraints
    position_constraint(model, data)
    velocity_constraint(model, data)


#  acceleration_constraint(model, data)
