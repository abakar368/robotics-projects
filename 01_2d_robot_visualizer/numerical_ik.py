
import numpy as np
import math

L1 = 1.0
L2 = 1.0


def forward_kinematics(theta):
    t1, t2 = theta

    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)

    return np.array([x, y])


def jacobian(theta):
    t1, t2 = theta

    return np.array([
        [-L1 * math.sin(t1) - L2 * math.sin(t1 + t2),
         -L2 * math.sin(t1 + t2)],

        [L1 * math.cos(t1) + L2 * math.cos(t1 + t2),
         L2 * math.cos(t1 + t2)]
    ])


def numerical_ik(target, initial_theta, damping=0.05,
                 max_iterations=1000, tolerance=1e-5):

    theta = np.array(initial_theta, dtype=float)

    for i in range(max_iterations):

        # Current end-effector position
        current = forward_kinematics(theta)

        # Position error
        error = target - current

        if np.linalg.norm(error) < tolerance:
            print("Target reached!")
            print("Iterations:", i)
            return theta

        # Calculate Jacobian
        J = jacobian(theta)

        # Damped Least Squares
        delta_theta = (
            J.T @
            np.linalg.solve(
                J @ J.T + damping**2 * np.eye(2),
                error
            )
        )

        # Limit each iteration's joint movement
        step_norm = np.linalg.norm(delta_theta)

        if step_norm > 0.2:
            delta_theta *= 0.2 / step_norm

        # Update joint angles
        theta += delta_theta

    print("Maximum iterations reached.")
    return theta


# Desired end-effector position
target = np.array([1.2, 0.8])

# Initial joint angles (30 and 45 degrees)
initial_theta = np.radians([30, 45])

solution = numerical_ik(target, initial_theta)

print("\nFinal joint angles:")
print("Shoulder:", math.degrees(solution[0]))
print("Elbow:", math.degrees(solution[1]))

print("\nFinal end-effector position:")
print(forward_kinematics(solution))

print("\nTarget position:")
print(target)
