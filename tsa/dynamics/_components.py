from enum import IntEnum
from .._model import Model, Data
from ..kinematics import jacobian, djacobian
import casadi


class Space(IntEnum):
    MOTOR = 1
    LOAD = 2


def inertia(model: Model, data: Data, space: Space) -> float | casadi.SX:
    """
    Calculate the inertia term D(q) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate inertia.

    Returns:
        float | casadi.SX: The inertia term D(q).

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = model.dynamic.load.inertia
    I = model.dynamic.motor.inertia

    if space == Space.MOTOR:
        J = jacobian(model, data)
        D = m * J**2 + I
    elif space == Space.LOAD:
        J = jacobian(model, data)
        D = I * (J**-2)
    else:
        raise ValueError(f"Invalid space: {space}")

    return D


def coriolis(model: Model, data: Data, space: Space) -> float | casadi.SX:
    """
    Calculate the Coriolis term C(q, dq) for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the Coriolis term.

    Returns:
        float | casadi.SX: The Coriolis term C(q, dq).

    Raises:
        ValueError: If an invalid space is provided.
    """
    m = model.dynamic.load.inertia
    I = model.dynamic.motor.inertia
    b_theta = model.dynamic.motor.damping
    b_x = model.dynamic.load.damping

    J = jacobian(model, data)
    dJdt = djacobian(model, data)
    if space == Space.MOTOR:
        C = m * J * dJdt + J * b_x + b_theta
    elif space == Space.LOAD:
        C = I * (J**-2) * dJdt + b_x + b_theta * (J**-1)
    else:
        raise ValueError(f"Invalid space: {space}")

    return C


def jamming(model: Model, data: Data, force: float | casadi.SX) -> float | casadi.SX:
    """
    Calculate the jamming effect.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        force (float | casadi.SX): The applied force.

    Returns:
        float | casadi.SX: The jamming effect.
    """
    L = model.kinematic.length
    r = model.kinematic.radius
    C_r = model.stiffness.transverse
    C_L = model.stiffness.longitudinal

    theta, X = data.theta, data.x

    dS_dtheta = (r / (L - X) ** 2) ** 2 * ((2 * L**2 - (r * theta) ** 2) * r * theta**3 * C_r - L**3 * theta * C_L)

    return dS_dtheta * (force**2)


def static(model: Model, data: Data, force: float | casadi.SX = 0) -> float | casadi.SX:
    """
    Calculate the static term.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        force (float | casadi.SX, optional): The applied force. Defaults to 0.

    Returns:
        float | casadi.SX: The static term.
    """
    J = jacobian(model, data)
    return J * force


def nonlinear(model: Model, data: Data, space: Space) -> float | casadi.SX:
    """
    Calculate the nonlinear term for the chosen space.

    Args:
        model (Model): The model object containing system parameters.
        data (Data): The data object storing state variables.
        space (Space): The space enum (MOTOR or LOAD) for which to calculate the nonlinear term.

    Returns:
        float | casadi.SX: The nonlinear term.

    Raises:
        ValueError: If an invalid space is provided.
    """
    C = coriolis(model, data, space)
    G = static(model, data)

    if space == Space.MOTOR:
        return C * data.dtheta + G
    elif space == Space.LOAD:
        return C * data.dx + G
    else:
        raise ValueError(f"Invalid space: {space}")


# TODO: Implement the following functions:
# def forward_dynamics(model: Model, data: Data, space: Space, input: float | casadi.SX) -> tuple[float | casadi.SX, float | casadi.SX]:
#     """Calculate the forward dynamics (acceleration) for the given space and input."""
#     pass

# def inverse_dynamics(model: Model, data: Data, space: Space, acceleration: float | casadi.SX) -> float | casadi.SX:
#     """Calculate the inverse dynamics (required input) for the given space and desired acceleration."""
#     pass

# def energy(model: Model, data: Data) -> tuple[float | casadi.SX, float | casadi.SX]:
#     """Calculate the kinetic and potential energy of the system."""
#     pass

"""
TODO: Decide on implementing this function. It may already be implemented elsewhere.
def nonlinear_term_mixed(model: Model, data: Data) -> float | casadi.SX:
    L = model.kinematic.length
    r = model.kinematic.radius
    b_m = model.dynamic.motor.damping
    I = model.dynamic.motor.inertia
    m = model.dynamic.load.inertia
    b_x = model.dynamic.load.damping
    g = 9.81  # Gravity constant (you may want to add this to the Model class)
    
    theta, dtheta = data.theta, data.dtheta
    x, dx = data.x, data.dx
    
    J = jacobian(model, data)
    h_x = b_m*dtheta - I*(dtheta**2 * r**2 + dx**2)/((L-x)*J) + J*(b_x*dx + m*g)
    return h_x
"""
