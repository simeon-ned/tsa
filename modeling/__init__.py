from .kinematics import Kinematics
from .dynamics import Dynamics


class TSAModel(Dynamics, Kinematics):
    def __init__(self, kinematic_params = [None, None], inertia_params = [None, None], friction_params = (0,0),  space = 'motor'):
        self.L, self.r = kinematic_params
        # print(self.L, self.r)
        
        Kinematics.__init__(self, self.L, self.r, space)
        # if 
        # TODO: init dynamics only if dynamics parameters are specified
        Dynamics.__init__(self, kinematic_params, inertia_params, friction_params, space)