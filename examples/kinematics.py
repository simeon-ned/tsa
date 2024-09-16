from tsa import Model, Data
from tsa.kinematics import contraction, jacobian
import casadi as ca

model = Model()
model.kinematic.length = ca.SX.sym("L")
model.kinematic.radius = ca.SX.sym("r")

data = Data(theta=ca.SX.sym("theta"), x=ca.SX.sym("x"))

print(model)
print(data)
print(contraction(model, data))
print(ca.jacobian(contraction(model, data), data.theta))
print(jacobian(model, data))
