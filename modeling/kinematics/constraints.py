
# // Constraints //

# // Position constraint // 
def pos_constraint(theta, x, L, r):
    """Holonomic constraint on the motor angle and contraction: 
    c = f(theta, x)"""
    
    c = theta**2 * r **2 + (L- x)**2 - L**2
    return c


def vel_constraint(theta, dtheta, x, dx, L, r):
    """Velocity constraint on the motor and contraction states: 
    dcdt = f(theta, dtheta, x, dx)"""
    dc = theta*r**2*dtheta - (L-x)*dx 
    return dc 


def accel_constraint(motor_motion, load_motion, L, r):
    """Acceleration constraint between motor and contraction motions, """
    theta, dtheta, ddtheta = motor_motion
    x, dx, ddx = load_motion
    ddc = theta*r**2*ddtheta + r**2 * dtheta**2 - (L-x)*ddx + dx**2 
    return ddc