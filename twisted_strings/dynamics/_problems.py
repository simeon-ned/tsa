from .._structs import Model, Data, Space
from ..kinematics import jacobian
from ._components import inertia, nonlinear, jamming


# TODO: Provide arguments similarly to kinematics functions


def forward_dynamics(model: Model, data: Data, space: Space) -> float:
    """
    Calculate the forward dynamics (acceleration) for the chosen space.

    NOTE: Jamming is not taken in to account for now.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the forward dynamics.
        torque (float): The applied torque (for MOTOR space) or force (for LOAD space).
        include_jamming (bool, optional): Whether to include jamming effects. Defaults to True.

    Returns:
        float: The acceleration in the chosen space.

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = inertia(model, data, space)
    # this term takes in to account external forces
    n = nonlinear(model, data, space)

    if space == Space.MOTOR:
        acceleration = (data.load.force - n) / m
        data.motor.acceleration = acceleration
    elif space == Space.LOAD:
        acceleration = (data.load.force - n) / m
        data.load.acceleration = acceleration
    else:
        raise ValueError(f"Invalid space: {space}")

    return acceleration


def inverse_dynamics(model: Model, data: Data, space: Space, include_jamming: bool = True) -> float:
    """
    Calculate the inverse dynamics (required torque/force) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the inverse dynamics.
        acceleration (float): The desired acceleration in the chosen space.
        include_jamming (bool, optional): Whether to include jamming effects. Defaults to True.

    Returns:
        float: The required torque (for MOTOR space) or force (for LOAD space).

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = inertia(model, data, space)
    n = nonlinear(model, data, space)

    if space == Space.MOTOR:
        torque = m * data.motor.acceleration + n
        data.motor.force = torque
        return torque
    elif space == Space.LOAD:
        force = m * data.load.acceleration + n
        if include_jamming:
            jam = jamming(model, data, force)
            force += jam
        data.load.force = force
        return force
    else:
        raise ValueError(f"Invalid space: {space}")
