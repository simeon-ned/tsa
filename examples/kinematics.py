from tsa import Model, Data
from tsa.kinematics import contraction, jacobian
import casadi as ca

model = Model()
model.kinematic.length = ca.SX.sym("L")
model.kinematic.radius = ca.SX.sym("r")

data = Data()
data.motor.position = ca.SX.sym("theta")

print(model)
print(data)
print(contraction(model, data))
# generate function to test:

model.kinematic.length = 0.2
model.kinematic.radius = 1e-3
cas_jacobian = ca.jacobian(contraction(model, data), data.motor.position)
cas_jacobian_func = ca.Function("cas_jacobian", [data.motor.position], [cas_jacobian])
analytical_jacobian = jacobian(model, data)
analytical_jacobian_func = ca.Function("analytical_jacobian", [data.motor.position], [analytical_jacobian])

print(analytical_jacobian_func(100))
print(cas_jacobian_func(100))
