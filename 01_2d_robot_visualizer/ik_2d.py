import math


def inverse_kinematics(x, y, L1, L2):
    """
    Calculate both IK solutions for a planar 2-link robot.

    Returns:
        A list of (theta1, theta2) solutions in degrees.
    """

    # Step 1: Calculate the distance to the target
    r_squared = x**2 + y**2

    # Step 2: Check whether the target is reachable
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    r = math.sqrt(r_squared)

    if r > max_reach or r < min_reach:
        raise ValueError("Target is outside the reachable workspace")

    # Step 3: Calculate the elbow angle using the cosine rule
    cos_theta2 = (
        r_squared - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Protect against tiny floating-point errors
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2_positive = math.acos(cos_theta2)
    theta2_negative = -theta2_positive

    solutions = []

    # Step 4: Calculate the shoulder angle for both solutions
    for theta2 in [theta2_positive, theta2_negative]:

        phi = math.atan2(y, x)

        beta = math.atan2(
            L2 * math.sin(theta2),
            L1 + L2 * math.cos(theta2)
        )

        theta1 = phi - beta

        # Convert radians to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        solutions.append((theta1_deg, theta2_deg))

    return solutions

def forward_kinematics(theta1, theta2, L1, L2):

    print("Before conversion:", theta1, theta2)

    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)

    print("After conversion:", theta1, theta2)

    x = (
        L1 * math.cos(theta1)
        + L2 * math.cos(theta1 + theta2)
    )

    y = (
        L1 * math.sin(theta1)
        + L2 * math.sin(theta1 + theta2)
    )

    return x, y


# Test the function
solutions = inverse_kinematics(
    x=0.5,
    y=1.5,
    L1=1,
    L2=1
)

for i, (theta1, theta2) in enumerate(solutions):
    print(f"Solution {i + 1}:")
    print(f"  Shoulder: {theta1:.2f} degrees")
    print(f"  Elbow:    {theta2:.2f} degrees")

    x_fk, y_fk = forward_kinematics(
        theta1, theta2, 1, 1
    )

    print(f"  FK position: ({x_fk:.3f}, {y_fk:.3f})")

# print("Manual FK test:")

# print(forward_kinematics(45, 90, 1, 1))
# print(forward_kinematics(90, -90, 1, 1))

