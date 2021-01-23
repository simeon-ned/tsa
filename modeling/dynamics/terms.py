g = 9.81 

# TODO: Rwerite functions using kwargs
# TODO: move repeated misc code to ..misc

# //////////////////////
# // Inverse dynamics //
# //////////////////////
 
# // Dynamical terms (D, C, g, h) //

# // Inertia terms // 

# def handle_arg(x, arg, default):
#     if callable():
from ..kinematics.position import contraction, motor_angle
from ..kinematics.jacobians import jacobian_load, jacobian_motor
from ..kinematics.jacobian_derivatives import dot_jacobian_load, dot_jacobian_motor
from ...misc._misc import spaces 

def inertia_term_motor(pos, jacobian, inertial_params):
    """Calculate inertia reflected to the motor side, 
    in terms of strings jacobian"""
    m, I = inertial_params

    J = jacobian(pos)

    D = J**2 * m + I
    return D


def inertia_term_load(pos, jacobian, inertial_params):
    """Calculate inertia reflected to the load side,
    in terms of strings jacobian"""
    m, I = inertial_params
    J = jacobian(pos)
    
    return I/J + m*J


def inertia_term(pos, kinematic_params, inertial_params, space = 'motor'):
    """Calculate inertia term D(q) with respect to chosen space"""
    
    L, r = kinematic_params 
    # TODO: Make ir elegant and concise
    if space == 'motor':
        J = lambda pos : jacobian_motor(pos, L, r) 
        D = inertia_term_motor(pos, J, inertial_params)
    if space == 'load':
        J = lambda pos : jacobian_load(pos, L, r) 
        D = inertia_term_load(pos, J, inertial_params)

    # else:
    return D


# // Corioolis terms // 

def coriolis_term_motor(pos, vel, jacobian, djacobian, inertial_params, friction_params = (0,0)):
    """Calculate coriolis term C(q, dq) for motor side dynamics,
    such that contribution of this term will be equal to C(q,dq)*dtheta"""
    
    m, I = inertial_params
    b_theta, b_x = friction_params

    J, dJdt = jacobian(pos), djacobian(pos, vel)

    C = m*J*dJdt + J*b_x + b_theta 
    
    return C


def coriolis_term_load(pos, vel, jacobian, djacobian, inertial_params, friction_params = (0,0)):
    """Calculate coriolis term C(q, dq) for load side dynamics,
    such that contribution of this term will be equal to C(q,dq)*dx"""
    
    m, I = inertial_params
    b_theta, b_x = friction_params
    
    J, dJdt = jacobian(pos), djacobian(pos, vel)

    C = I* (J**(-2))* dJdt + b_x +b_theta*(J**-1)

    return C


def coriolis_term(pos, vel, kinematic_params, inertial_params, friction_params = (0,0), space = 'motor'):
    """Calculate coriolis term C(q, dq) for choosen space"""
    L, r = kinematic_params 

    if space == 'motor':
        J = lambda pos : jacobian_motor(pos, L, r) 
        dJdt = lambda pos, vel: dot_jacobian_motor(pos, vel, L, r)
        C = coriolis_term_motor(pos, vel, J, dJdt, inertial_params, friction_params)
    if space == 'load':
        J = lambda pos : jacobian_load(pos, L, r) 
        dJdt = lambda pos, vel: dot_jacobian_load(pos, vel, L, r)
        C = coriolis_term_load(pos, vel, J, dJdt, inertial_params, friction_params)

    # else:
    return C

# TODO: Implement string jamming
# def string_jamming(pos)

# // Nonlinear term // 

def jamming_term(pos, force, kinematic_params, compliance_params, space = 'motor'):
    L, r = kinematic_params
    C_r, C_L = compliance_params
    
    if space == 'motor':
        theta = pos
        X = contraction(theta, L, r)
    
    if space == 'load':
        X = pos
        theta = motor_angle(X, L, r)

    dS_dtheta = (r/(L - X)**2)**2 * ( (2*L**2 - (r*theta)**2) * r*theta**3 *C_r - L**3 * theta*C_L)

    return dS_dtheta*(force**2)


# // static term // 
def static_term(pos, force, kinematic_params, compliance_params = [0,0], space = 'motor', internal_force = False):
    L, r = kinematic_params
    if space == 'motor':
        J = jacobian_motor(pos, L, r)
    if space == 'load':
        J = jacobian_load(pos, L, r)
    
    if force is callable:
        vel = 0
        F = force(pos, vel)
    else:
        F = force

    G = J*F 
    
    # if internal_force:
    #     G+=jamming_term()

    return G


# TODO: Implement nonlinear terms: h(q,dq)


def nonlinear_term(pos, vel, force, kinematic_params, inertial_params, friction_params = (0,0), space = 'motor', internal_force  = False):
    # if space == 'motor':
    C = coriolis_term(pos, vel, kinematic_params, inertial_params, friction_params, space)
    G = static_term(pos, force, kinematic_params, space, internal_force=internal_force)
    h = C*vel + G
    return h


def nonlinear_term_mixed(theta, dtheta, x, dx, load_params, motor_params, string_params):
    """Calculate nonlinear term with respect to full state"""
    L, r = string_params
    b_m, I = motor_params  
    m, b_x = load_params
    J = 1
    h_x = b_m*dtheta - I*(dtheta**2 * r**2 + dx**2)/((L-x)*J) + J*(b_x*dx +m*g)
    return h_x

