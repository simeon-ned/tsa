from tsa import Model, Data
from tsa.kinematics import contraction, jacobian

# Numerical modeling without CasADi

# Create model with numeric values
model = Model()
model.kinematic.length = 0.2
model.kinematic.radius = 1e-3

data = Data()
data.motor.position = 100

# Calculate numerical results using existing functions
x = contraction(model, data)
jac = jacobian(model, data)

print("Numerical Results:")
print("Contraction:", x)
print("Jacobian:", jac)

# In fact one function is called the results are stored in the data structure
print("\nData structure population:")
print("Load position:", data.load.position)
print("Motor jacobian:", data.motor.jacobian)
