from tsa import Model, Data
from tsa.kinematics import contraction, jacobian
import casadi as ca

model = Model()
model.kinematic.length = ca.SX.sym("L")
model.kinematic.radius = ca.SX.sym("r")
print(model)

data = Data()
data.motor.position = ca.SX.sym("theta")
print(data)

x = contraction(model, data)
print(x)
# This will also populate the data.load.position
print(x == data.load.position)

# generate function to test:

model.kinematic.length = 0.2
model.kinematic.radius = 1e-3
cas_jacobian = ca.jacobian(contraction(model, data), data.motor.position)
cas_jacobian_func = ca.Function("cas_jacobian", [data.motor.position], [cas_jacobian])
analytical_jacobian = jacobian(model, data)

# This will also populate the data.motor.jacobian
print(analytical_jacobian == data.motor.jacobian)

analytical_jacobian_func = ca.Function("analytical_jacobian", [data.motor.position], [analytical_jacobian])
print(analytical_jacobian_func(100))
print(cas_jacobian_func(100))
