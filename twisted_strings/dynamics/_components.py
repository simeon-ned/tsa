from .._structs import Model, Data, Space
from ..kinematics import jacobian, djacobian


# TODO: Provide arguments similarly to kinematics functions


def inertia(model: Model, data: Data, space: Space) -> float:
    """
    Calculate the inertia term M(q) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate inertia.

    Returns:
        float: The inertia term M(q).

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = model.dynamic.load.inertia
    I = model.dynamic.motor.inertia

    if space == Space.MOTOR:
        J = jacobian(model, data)
        data.motor.jacobian = J
        D = m * J**2 + I
    elif space == Space.LOAD:
        J = jacobian(model, data)
        data.load.jacobian = J
        D = I * (J**-2)
    else:
        raise ValueError(f"Invalid space: {space}")

    if space == Space.MOTOR:
        data.motor.inertia = D
    else:
        data.load.inertia = D

    return D


def coriolis(model: Model, data: Data, space: Space) -> float:
    """
    Calculate the Coriolis term C(q, dq) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the Coriolis term.

    Returns:
        float: The Coriolis term C(q, dq).

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = model.dynamic.load.inertia
    I = model.dynamic.motor.inertia
    b_theta = model.dynamic.motor.damping
    b_x = model.dynamic.load.damping

    if space == Space.MOTOR:
        J = jacobian(model, data)
        dJdt = djacobian(model, data)
        data.motor.jacobian = J
        data.motor.djacobian = dJdt
        C = m * J * dJdt + J * b_x + b_theta
    elif space == Space.LOAD:
        J = jacobian(model, data)
        dJdt = djacobian(model, data)
        data.load.jacobian = J
        data.load.djacobian = dJdt
        C = I * (J**-2) * dJdt + b_x + b_theta * (J**-1)
    else:
        raise ValueError(f"Invalid space: {space}")

    if space == Space.MOTOR:
        data.motor.coriolis = C
    else:
        data.load.coriolis = C

    return C


def jamming(model: Model, data: Data, force: float) -> float:
    """
    Calculate the jamming effect.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        force (float): The applied force.

    Returns:
        float: The jamming effect.
    """
    L = model.kinematic.length
    r = model.kinematic.radius
    C_r = model.stiffness.transverse
    C_L = model.stiffness.longitudinal

    theta, X = data.motor.position, data.load.position

    dS_dtheta = (r / (L - X) ** 2) ** 2 * ((2 * L**2 - (r * theta) ** 2) * r * theta**3 * C_r - L**3 * theta * C_L)

    jamming_effect = dS_dtheta * (force**2)
    data.motor.jamming = jamming_effect
    data.load.jamming = jamming_effect

    return jamming_effect


def static(model: Model, data: Data, force: float = 0) -> float:
    """
    Calculate the static term.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        force (float, optional): The applied force. Defaults to 0.

    Returns:
        float: The static term.
    """
    J = jacobian(model, data)
    data.motor.jacobian = J
    static_term = J * force
    data.motor.static = static_term
    data.load.static = force
    return static_term


def nonlinear(model: Model, data: Data, space: Space) -> float:
    """
    Calculate the nonlinear term for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the nonlinear term.

    Returns:
        float: The nonlinear term.

    Raises:
        ValueError: If an invalid space is provided.
    """
    C = coriolis(model, data, space)
    G = static(model, data)

    if space == Space.MOTOR:
        nonlinear_term = C * data.motor.velocity + G
        data.motor.nonlinear = nonlinear_term
    elif space == Space.LOAD:
        nonlinear_term = C * data.load.velocity + G
        data.load.nonlinear = nonlinear_term
    else:
        raise ValueError(f"Invalid space: {space}")

    return nonlinear_term
