
from .jacobians import jacobian_load, jacobian_mixed, jacobian_motor 

jacobians = {"load": jacobian_load,
             "motor": jacobian_motor, 
             "both": jacobian_mixed}

# calculate load speed as functionf of position and 
def contraction_speed(pos, dtheta, L, r, space = 'motor',  jacobian = None):
    """Calculate contraction speed as function of position and motor speed"""
    if not jacobian:
        jacobian = jacobians[space]
        J = jacobian(pos, L, r) 
    else:
        J = jacobian(pos)

    dx = J*dtheta
    
    return dx

# calculate
def motor_speed(pos, dx, L, r, space = 'motor', jacobian = None):
    """Calculate motor speed as function of position and contraction speed"""
    if not jacobian:
        jacobian = jacobians[space]
        J = jacobian(pos, L, r) 
    else:
        J = jacobian(pos)

    dtheta = dx/J
    return dtheta