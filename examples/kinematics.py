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
print(ca.jacobian(contraction(model, data), data.motor.position))
print(jacobian(model, data))
