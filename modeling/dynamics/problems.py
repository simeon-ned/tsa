
# ///////////////////////////////////////////
# // Inverse Dynamics (Forces and Torques) // 
# //////////////////////////////////////////


# tension()
# inverse_dynamics(motion, force, space, params) - obt
# forward_dynamics(input, force, space, params)


# // Tension // 

def tension(load_motion, load_params, force = 0):
    """"""
    m, b_x = load_params
    x, dx, ddx = load_motion 

    if callable(force):
        F = force(x, dx)
    else:
        F = force

    T = m*ddx + b_x*dx + m*g + F
    return T

# // Motor Torque // 

def motor_torque(motor_motion, tension, motor_params):
    theta, dtheta, ddtheta = motor_motion
    I, b_theta = motor_params
    tau = I*ddtheta + b_theta * dtheta
    return None


# //////////////////////
# // Forward dynamics //
# //////////////////////
# // Acceleration
def ddtheta():
    return None

def ddx():
    return None

# // State Space // 
def state_space():
    return None
