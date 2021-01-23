from .position import motor_angle, contraction
from .jacobians import jacobian_load, jacobian_motor
from .velocity import motor_speed, contraction_speed

# // Jacobian Derevetivs // 

def dot_jacobian_mixed(theta, dtheta, x, dx, L, r):
    """Calculate time derevetive of the 'mixed' jacobian with respect to time
     as function of both motor and load states"""
    djac = r**2*dtheta/(L-x) + theta*r**2 * dx/(L-x)**2
    return djac

    
def dot_jacobian_motor(theta, dtheta, L, r):
    """Calculate time derevetive of the jacobian with respect to time
     as function of motor state"""
    x = contraction(theta, L, r)
    dx = contraction_speed(theta, dtheta, L, r, space = 'motor')
    
    djac = dot_jacobian_mixed(theta, dtheta, x, dx, L, r)
    return djac


def dot_jacobian_load(x, dx, L, r):
    """Calculate time derevetive of the jacobian with respect to time
     as function of motor state"""
    theta = motor_angle(x, L, r)
    dtheta = motor_speed(x, dx, L, r, space = 'load')
    
    djac = dot_jacobian_mixed(theta, dtheta, x, dx, L, r)
    return djac

