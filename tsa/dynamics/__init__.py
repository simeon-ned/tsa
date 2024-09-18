"""
Dynamic Components for Twisted String Actuators

This module provides functions for calculating various dynamic effect and
components of twisted string actuators, including inertia, Coriolis effect,
jamming, static forces, and nonlinear terms.

The models and calculations in this module are based on the research
presented in the following papers:

- Nedelchev S, Gaponov I, Ryu JH. Accurate dynamic modeling of twisted
   string actuators accounting for string compliance and friction. IEEE
   Robotics and Automation Letters. 2020 Jan 30;5(2):3438-43.

- Nedelchev S, Skvortsova V, Guryev B, Gaponov I, Ryu JH. On Energy-Preserving
   Motion in Twisted String Actuators. IEEE Robotics and Automation Letters.
   2021 Jul 16;6(4):7406-12.
"""

from ._components import Space, coriolis, inertia, jamming, nonlinear, static
from ._problems import forward_dynamics, inverse_dynamics
# TODO: Add forward and inverse dynamics functions
