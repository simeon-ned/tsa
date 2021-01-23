""" class <<Dynamics>> is a collection of TSA dynamics routines  

One may use this to solve forward and inverse dynamics problems, calculate energy functions, dynamical coefficients (D(q),C(q,dq),G(q), h(q,dq) terms) 
as well as regressor matrix Y(q,dq,ddq)

"""
# from .
# add space option:
#       Motor space 
#       Load space
from ..kinematics import Kinematics
# from ..kinematics.jacobians
# from ..kinematics.djacobians import *
from ..stiffness import Stiffness
from .terms import *
from .problems import *
from .energy import *

# TODO: rewrite functions such that we can provide arguments and use them separatly from class
# class Dynamics(Kinematics):

from .terms import *

class Dynamics():
    """Kinematics class is used to store all TSA kinematics related methods and provide user interface"""
    g = 9.81
    # // Default kinematic parameters //  
    
    # // Init function // 
    def __init__(self, kinematic_params = [None, None], inertia_params = [None, None], friction_params = (0,0),  space = 'motor'):

        
        # TODO: Move argument handler to the different routine
        if space != 'motor' or 'load' or 'full':
            self.space = 'motor'
        if None in kinematic_params:
            print('\nPlease set kinematic parameters <kinematic_params = [L, r]> \n')
            self.__del__()
        

        else:
            # String parameters
            self.kinematic_params = kinematic_params
            self.L, self.r = self.kinematic_params
            self.inertia_params = inertia_params
            self.friction_params = friction_params

            self.m, self.I = self.inertia_params
            self.b_theta, self.b_x = self.friction_params            # self.m = m
            # self.g = 9.81
            # Space to work with (ex calculating Jacobians):
            #       motor -> motor space 
            #       load -> load space
            #       both -> combination of load and motor spaces
            # Kinematics.__init__(self, L=self.L, r = self.r, space = self.space)
            self.space = space    
            self.force = lambda pos, vel: self.m*g

            # // Store external functions as methods of class //
            # self.jacobian = 
            # // Dynamic terms  // 
            self.inertia_term = lambda pos : inertia_term(pos, self.kinematic_params, self.inertia_params, self.space)
            self.coriolis_term = lambda pos, vel : coriolis_term(pos, vel, self.kinematic_params, self.inertia_params, self.friction_params, self.space)
            self.static_term = lambda pos: static_term(pos, self.force, self.kinematic_params, self.space)
            self.nonlinear_term = lambda pos, vel: nonlinear_term(pos, vel, self.m*g, self.kinematic_params, self.inertia_params, self.friction_params, self.space)
            self.dynamic_regressor = None
            # self.
            
            # // Inverse problems 
            self.torque = None
            self.tension = None
            
            # // Forward problems 
            self.ddx = None
            self.state = None
            
            # // Energy models // 
            self.kinetic_energy = None
            self.potential_energy = None
            self.dissipated_energy = None
            self.energy_regressor = None
            
    # // Class destructor // 
    def __del__(self):
        print("Dynamics TSA object was deleted from memory")    
    
    # // User interface // 
    def set_kinematics(self, L, r):
        """Set new kinematic parameters, L - length of the stringc, 
        r - effective radius"""
        self.L = L
        self.r = r
        self.kinematic_params = self.L, self.r


    # MOVE TO MISC ROUTINES
    def set_length(self, L):
        self.L = L
        self.kinematic_params = self.L, self.r


    def set_radii(self, L):
        self.L = L
        self.kinematic_params = self.L, self.r


    def set_inertia(self, inertia_params):
        self.m, self.I = inertia_params

    def set_viscous_friction(self, friction_params):
        self.b_theta, self.b_x = friction_params
    
            
    def set_force(self, force):
        """Set the function of external force F(q, dq) that act on the load"""
        self.force = force


# TODO: think about this way to wrapp the functions 

# from ...misc.wrapper import add_method
# 
# @add_method(Kinematics)
# def get_projection(theta, L, r):
#     proj = (1 - (theta*r/L)**2)**0.5
#     return proj