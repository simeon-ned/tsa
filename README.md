# Twisted String Actuator Modeling and Simulation
<!-- TODO: -->
<!-- - Small buttons displaying PyPI status and other relevant information
- A distinctive logo for easy recognition
- A quick-start tutorial in Google Colab for rapid familiarization -->

## Overview

This repository houses a Python module for modeling, simulating, and analyzing [Twisted String Actuators](https://www.youtube.com/watch?v=YD1GXLHVjYc) (TSAs). TSAs are innovative linear actuators that transform rotational motion into linear motion by twisting strings or cables, offering unique advantages in various applications.

## Features

- Symbolic and numerical computation support
- Flexible data structures for comprehensive TSA state and parameter representation
- Kinematic and dynamic modeling of TSAs: functions for calculating contraction, Jacobians, dynamical components, and other critical properties

<!-- Please dive into quick tutorial to get you up to speed in no time <a href="https://colab.research.google.com/github/based-robotics/jaxadi/blob/master/examples/_demo.ipynb"><img src="https://colab.research.google.com/assets/colab-badge.svg" width="120" align="center"/></a> -->

## Installation

Get started quickly by installing the TSA module using pip:

```bash
pip install twisted-strings
```

## Usage

Here's a quick example to demonstrate the basics of the TSA module:

```python
from tsa import Model, Data
from tsa.kinematics import contraction, jacobian

# Initialize model and set parameters
model = Model()
model.kinematic.length = 0.2
model.kinematic.radius = 1e-3

# Create data object and set state
data = Data()
data.motor.position = 100

# Calculate contraction and Jacobian
x = contraction(model, data)
jac = jacobian(model, data)

print(f"Contraction: {x}")
print(f"Jacobian: {jac}")
```

By leveraging more advanced data types in the data and module structures, you can construct sophisticated models and perform complex calculations. This includes building symbolical and differentiable models with CasADi and JAX, as well as conducting straightforward symbolical calculations using sympy.

For more in-depth examples, explore the `examples` directory. Note that running some examples may require additional libraries such as sympy and CasADi. We recommend creating a dedicated environment for this project, the simplest way is to use the provided `tsa_environment.yml` file to create the environment:

```bash
conda env create -f environment.yml
conda activate tsa
```


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

<!-- ## Future Enhancements

Stay tuned for these exciting updates!

## Feedback and Contributions

We welcome your feedback and contributions to make this project even better. Feel free to open an issue or submit a pull request on our GitHub repository. -->
