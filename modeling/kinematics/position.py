

def contraction(theta, L, r):
    """Calculate contraction as function of motor angle"""
    x = L - (L**2 - (theta*r)**2)**0.5
    return x


def motor_angle(x, L, r):
    """Calculate motor angle as function of contraction"""        
    theta = (L**2 - (L - x)**2)**0.5/r
    return theta

# // Speed //

# TODO: implement handle for different jacobians and function arguments