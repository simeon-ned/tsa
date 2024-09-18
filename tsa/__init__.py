"""
Twisted String Actuator (TSA) Modeling and Simulation Module

This module provides a comprehensive framework for modeling, simulating, and analyzing
twisted string actuators (TSAs). TSAs are a type of linear actuator that converts rotational
motion into linear motion through the twisting of strings or cables.

Key features:
- Kinematic and dynamic modeling of TSAs
- Symbolic and numerical computation support
- Extensible data structures for representing TSA states and parameters
- Functions for calculating contraction, jacobians, and other key properties

The module is designed to be flexible and can work with different computational backends,
including CasADi for autodifferentiation and optimization, as well as symbolic libraries
like SymPy for symbolic calculations.

Main components:
- Model: Represents the physical parameters and properties of a TSA
- Data: Stores the state variables and computed values for a TSA
- Kinematics: Functions for computing kinematic properties (e.g., contraction, jacobian)
- Dynamics: Functions for computing dynamic properties and simulating TSA behavior

This module is intended for researchers, engineers, and developers working on
robotic systems, soft actuators, or any application involving twisted string actuators.
It provides tools for design optimization, control system development, and performance analysis
of TSA-based systems.
"""

from ._structs import Data, Model


# TODO:
# Create compute_all function
# Think about code gen features
