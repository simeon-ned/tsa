# Twisted String Actuator Modeling and Simulation

<!-- TODO: ADD LOGO -->

## Overview

This repository contains a Python module for modeling, simulating, and analyzing [Twisted String Actuators](https://www.youtube.com/watch?v=YD1GXLHVjYc) (TSAs). TSAs are a type of linear actuator that converts rotational motion into linear motion through the twisting of strings or cables.

## Features

- Symbolic and numerical computation support
- Extensible data structures for representing TSA states and parameters
- Kinematic and dynamic modeling of TSAs, Functions for calculating contraction, jacobians, dynamical components and other key properties

## Installation

To install the TSA module, clone this repository and install the module in editable mode:

```bash
pip install twisted-strings
```

## Usage

Here's a basic example of how to use the TSA module:

```python
from tsa import Model, Data
from tsa.kinematics import contraction, jacobian

# Create a model and set parameters
model = Model()
model.kinematic.length = 0.2
model.kinematic.radius = 1e-3

# Create a data object and set state
data = Data()
data.motor.position = 100

# Calculate contraction and jacobian
x = contraction(model, data)
jac = jacobian(model, data)

print(f"Contraction: {x}")
print(f"Jacobian: {jac}")
```
By populating the data and module structures with more advanced data types one may build more profound models and calculations, i.e build symbolical and differentiable models with CasADi and JAX, as well as simple symbolical calculations with sympy.

For more detailed examples, please refer to the `examples` directory. 


<!-- ## Contributing

Contributions to this project are welcome! Please follow these steps:

1. Fork the repository
2. Create a new branch for your feature
3. Commit your changes
4. Push to your fork
5. Submit a pull request

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact

For questions or support, please open an issue in the GitHub repository or contact the maintainers directly. -->

<!-- ## Acknowledgments

- List any individuals or organizations that have contributed to or inspired this project
- Mention any relevant research papers or resources -->

## Citation

If you use this software in your research, please cite it as follows:

```bibtex
@software{tsa_modeling,
  author = {Nedelchev, Simeon and Kozlov, Lev},
  title = {Twisted String Actuator (TSA) Modeling and Simulation},
  year = {2024},
  url = {https://github.com/simeon-ned/tsa}
}
```
