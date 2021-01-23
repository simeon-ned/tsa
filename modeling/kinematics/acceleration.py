from .jacobians import jacobian_load
from .jacobian_derivatives import dot_jacobian_load, dot_jacobian_motor
from .position import motor_angle
# from .velocity import motor_speed

def motor_acceleration(x, dx, ddx, L, r):
    J = jacobian_load(x, L, r)
    dtheta = dx/J
    theta = motor_angle(x, L, r)
    # dJ = dot_jacobian_load(x, dx, L, r) 
    dJ = dot_jacobian_motor(theta, dtheta, L, r)

    ddtheta = (ddx - dJ*dtheta)/J
    return ddtheta 