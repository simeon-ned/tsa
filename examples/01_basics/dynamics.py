from twisted_strings import Model, Data
from twisted_strings.dynamics import inertia, coriolis, jamming, static, nonlinear, Space
import casadi as ca

model = Model()
model.kinematic.length = ca.SX.sym("L")
model.kinematic.radius = ca.SX.sym("r")
print(model)

data = Data()
data.motor.position = ca.SX.sym("theta")
data.motor.velocity = ca.SX.sym("dtheta")
inertia(model, data, Space.MOTOR)
nonlinear(model, data, Space.MOTOR)
print(data.motor.inertia)
print(data.motor.nonlinear)
