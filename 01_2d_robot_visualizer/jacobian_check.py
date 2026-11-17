
import math

# Link lengths
L1 = 1.0
L2 = 1.0

# Initial joint angles (convert degrees to radians)
theta1 = math.radians(30)
theta2 = math.radians(45)

# Small joint movements, already in radians
dtheta1 = 0.001
dtheta2 = 0.002


# Forward Kinematics
def forward_kinematics(t1, t2):
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


# Calculate Jacobian
def calculate_jacobian(t1, t2):
    j11 = -L1 * math.sin(t1) - L2 * math.sin(t1 + t2)
    j12 = -L2 * math.sin(t1 + t2)

    j21 = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    j22 = L2 * math.cos(t1 + t2)

    return [[j11, j12], [j21, j22]]


# Initial end-effector position
x0, y0 = forward_kinematics(theta1, theta2)

# Jacobian prediction
J = calculate_jacobian(theta1, theta2)

dx_predicted = J[0][0] * dtheta1 + J[0][1] * dtheta2
dy_predicted = J[1][0] * dtheta1 + J[1][1] * dtheta2

# Apply joint movements and calculate actual position
new_theta1 = theta1 + dtheta1
new_theta2 = theta2 + dtheta2

x1, y1 = forward_kinematics(new_theta1, new_theta2)

dx_actual = x1 - x0
dy_actual = y1 - y0

# Display results
print("INITIAL POSITION")
print(f"x = {x0:.6f}, y = {y0:.6f}")

print("\nJACOBIAN PREDICTION")
print(f"dx = {dx_predicted:.6f}")
print(f"dy = {dy_predicted:.6f}")

print("\nACTUAL FK MOVEMENT")
print(f"dx = {dx_actual:.6f}")
print(f"dy = {dy_actual:.6f}")

print("\nCOMPARISON")
print(f"X difference = {abs(dx_actual - dx_predicted):.8f}")
print(f"Y difference = {abs(dy_actual - dy_predicted):.8f}")

