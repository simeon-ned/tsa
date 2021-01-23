from ...misc.wrapper import add_method 
# from .__init__ import Kinematics 
    # # //////////////////
    # # // Acceleration // 
    # # //////////////////


    # def motor_acceleration(self, theta, dtheta, L, r):
    #     return None


    # # def motor_acceleration_x(x, dx, ddx, string_params):
    # #     L, r = string_params
    # #     jacobian = tsa_jacobian_x(x, string_params)
    # #     dtheta = dx/jacobian
    # #     ddtheta = ddx/jacobian - (dtheta**2*r**2 + dx**2)/(jacobian*(L-x))
    # #     return ddtheta 

    # # ////////////
    # # // Other //
    # # ///////////

# @add_method(Kinematics)
# def get_projection(theta, L, r):
#     proj = (1 - (theta*r/L)**2)**0.5
#     return proj