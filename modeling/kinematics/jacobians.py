from .position import motor_angle

# // Jacobians // 

def jacobian_motor(theta, L, r):
    """Calculate jacobian as function of motor angle"""
    J = theta * (r**2)/(L**2 - (theta*r)**2)**0.5
    return J


def jacobian_load(x, L, r):
    """Calculate jacobian as function of contraction"""
    # J = r*((2*L - x)*x)**0.5/(L-x)
    theta = motor_angle(x, L, r)
    J = jacobian_motor(theta, L, r)

    return J


def jacobian_mixed(theta, x, L, r):
    """Calculate jacobian as function of both contraction and motor angle"""
    J = theta*(r**2)/(L-x)
    return J


