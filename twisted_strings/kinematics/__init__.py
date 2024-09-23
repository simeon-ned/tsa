"""
Kinematics Module for Twisted String Actuators

This module provides functions for calculating various kinematic components
of twisted string actuators, including position, velocity, and acceleration
transformations between motor and load spaces.

The models and calculations in this module are based on the research
presented in the following papers (and others):

- Gaponov I, Popov D, Ryu JH. Twisted string actuation systems: A study of
   the mathematical model and a comparison of twisted strings. IEEE/ASME
   Transactions on mechatronics. 2013 Sep 20;19(4):1331-42.

- Nedelchev S, Kirsanov D, Gaponov I. IMU-based Parameter Identification and
   Position Estimation in Twisted String Actuators. In2020 IEEE/RSJ International
   Conference on Intelligent Robots and Systems (IROS) 2020 Oct 24 (pp. 6311-6317). IEEE.
"""

from ._acceleration import djacobian, motor_acceleration
from ._position import contraction, motor_angle
from ._velocity import contraction_speed, jacobian, motor_speed
