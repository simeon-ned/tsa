
from .modeling.dynamics import Dynamics
from .modeling.kinematics import Kinematics

class TSA(Kinematics, Dynamics):

    def __init__(self, kinematic_params, inertia_params, friction_params = (0,0), space = 'motor'):
        # handle for args
        pass 

    def __del__(self):
        print("TSA Object was deleted from memory")
        
