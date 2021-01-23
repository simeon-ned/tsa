"""TSA.Kinematics is a collection of TSA kinematics routines  

one may use this to calculate contraction, jacobians 
as well as kinematic constraints 
"""

from .constraints import *
from .position import *
from .jacobians import *
from .velocity import *
from .jacobian_derivatives import *
# add space option:
#       Motor space 
#       Load space

# TODO: rewrite functions such that we can provide arguments and use them separatly from class
class Kinematics:
    """Kinematics class is used to store all TSA kinematics related methods and provide user interface"""
    
    # // Default kinematic parameters //  
    
    # // Init function // 
    def __init__(self, L = None, r = None, space = 'motor'):

        
        if space != 'motor' or 'load' or 'full':
            self.space = 'motor'

        if L == None or r == None:
            print('\nPlease initialize kinematics class with both string radius and length!\n')
            self.__del__()
        
        else:
            # String parameters
            self.L = L
            self.r = r

            # Space to work with (ex calculating Jacobians):
            #       motor -> motor space 
            #       load -> load space
            #       both -> combination of load and motor spaces
            self.space = space    
   

            # // Store external functions as methods of class //
   
            # // Kinematic constraints // 
            self.pos_constraint = lambda theta, x : pos_constraint(theta, x, self.L, self.r)
            self.vel_constraint = lambda theta, dtheta, x, dx : vel_constraint(theta, dtheta, x, dx, self.L, self.r)
            self.accel_constraint = lambda motor_motion, load_motion : accel_constraint(motor_motion, load_motion, self.L, self.r)
            
            # // Transformation functions 
            self.contraction = lambda theta : contraction(theta, self.L, self.r)
            self.motor_angel = lambda x : motor_angle(x, self.L, self.r)
            
            # TODO: add "space" option (either motor or load)
            self.contraction_speed = lambda pos, dtheta: contraction_speed(pos, dtheta, self.L, self.r, space) # TODO: add "space" option (either motor or load)
            self.motor_speed = lambda pos, dx: motor_speed(pos, dx, self.L, self.r, space)
            # motor_speed()
            # TODO: implement accelerations
            # self.motor_accel 
            # self.contraction_accel: 

            # // Jacobians //
            self.jacobian_motor = lambda theta : jacobian_motor(theta, self.L, self.r) 
            self.jacobian_load = lambda x : jacobian_load(x, self.L, self.r) 
            self.jacobian_mixed = lambda theta, x : jacobian_mixed(theta, x, self.L, self.r)
            
            # // Jacobian Derevetivs // 
            self.dot_jacobian_motor = lambda theta, dtheta : dot_jacobian_motor(theta, dtheta, L, r)
            self.dot_jacobian_load = lambda x, dx: dot_jacobian_load(x, dx, L, r)
            self.dot_jacobian_mixed = lambda theta, dtheta, x, dx: dot_jacobian_mixed(theta, dtheta, x, dx, L, r)

            
            # self.acce
    # // Class destructor // 
    def __del__(self):
        print("TSA object was deleted from memory")    
    
    # // User interface // 
    def set_parameters(self, L, r):
        """Set new kinematic parameters, L - length of the stringc, 
        r - effective radius"""
        self.L = L
        self.r = r


# TODO: think about this way to wrapp the functions 

# from ...misc.wrapper import add_method
# 
# @add_method(Kinematics)
# def get_projection(theta, L, r):
#     proj = (1 - (theta*r/L)**2)**0.5
#     return proj