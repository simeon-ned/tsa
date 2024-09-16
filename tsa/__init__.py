from ._model import Model, Data


# TODO:
# Create compute_all function
# Think about code gen features

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


# TODO: Decide on implementing this function. It may already be implemented elsewhere.
# def nonlinear_term_mixed(model: Model, data: Data) -> float | casadi.SX:
#     L = model.kinematic.length
#     r = model.kinematic.radius
#     b_m = model.dynamic.motor.damping
#     I = model.dynamic.motor.inertia
#     m = model.dynamic.load.inertia
#     b_x = model.dynamic.load.damping
#     g = 9.81  # Gravity constant (you may want to add this to the Model class)

#     theta, dtheta = data.theta, data.dtheta
#     x, dx = data.x, data.dx

#     J = jacobian(model, data)
#     h_x = b_m*dtheta - I*(dtheta**2 * r**2 + dx**2)/((L-x)*J) + J*(b_x*dx + m*g)
# return h_x

# TODO:
# Create regressor terms
