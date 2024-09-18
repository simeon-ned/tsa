from .._structs import Model, Data, Space
from ..kinematics import jacobian
from ._components import inertia, nonlinear, jamming
import casadi


def forward_dynamics(
    model: Model, data: Data, space: Space, torque: float | casadi.SX, include_jamming: bool = True
) -> float | casadi.SX:
    """
    Calculate the forward dynamics (acceleration) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the forward dynamics.
        torque (float | casadi.SX): The applied torque (for MOTOR space) or force (for LOAD space).
        include_jamming (bool, optional): Whether to include jamming effects. Defaults to True.

    Returns:
        float | casadi.SX: The acceleration in the chosen space.

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = inertia(model, data, space)
    n = nonlinear(model, data, space)

    if include_jamming:
        j = jamming(model, data, torque if space == Space.LOAD else jacobian(model, data) * torque)
    else:
        j = 0

    if space == Space.MOTOR:
        acceleration = (torque - n - j) / m
        data.motor.acceleration = acceleration
    elif space == Space.LOAD:
        acceleration = (torque - n - j) / m
        data.load.acceleration = acceleration
    else:
        raise ValueError(f"Invalid space: {space}")

    return acceleration


def inverse_dynamics(
    model: Model, data: Data, space: Space, acceleration: float | casadi.SX, include_jamming: bool = True
) -> float | casadi.SX:
    """
    Calculate the inverse dynamics (required torque/force) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the inverse dynamics.
        acceleration (float | casadi.SX): The desired acceleration in the chosen space.
        include_jamming (bool, optional): Whether to include jamming effects. Defaults to True.

    Returns:
        float | casadi.SX: The required torque (for MOTOR space) or force (for LOAD space).

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = inertia(model, data, space)
    n = nonlinear(model, data, space)

    if space == Space.MOTOR:
        torque = m * acceleration + n
        # Note: Jamming effect is not included here as it depends on the force,
        # which is not known in inverse dynamics calculation for the motor space
        data.motor.torque = torque
        return torque
    elif space == Space.LOAD:
        force = m * acceleration + n
        if include_jamming:
            j = jamming(model, data, force)
            force += j
        data.load.force = force
        return force
    else:
        raise ValueError(f"Invalid space: {space}")
