
from ik_2d import inverse_kinematics
from visualizer import forward_kinematics, draw_robot


# Robot dimensions
L1 = 1.0
L2 = 1.0


# Ask the user for a target
x = float(input("Enter target X: "))
y = float(input("Enter target Y: "))


try:
    # Calculate IK solutions
    solutions = inverse_kinematics(x, y, L1, L2)

    for i, (theta1, theta2) in enumerate(solutions):

        print(f"\nSolution {i + 1}")
        print(f"Shoulder: {theta1:.2f} degrees")
        print(f"Elbow: {theta2:.2f} degrees")

        # Verify using FK
        elbow, end_effector = forward_kinematics(
            theta1, theta2, L1, L2
        )

        print(
            f"Calculated target: "
            f"({end_effector[0]:.3f}, "
            f"{end_effector[1]:.3f})"
        )

        # Display robot
        draw_robot(
            theta1,
            theta2,
            target=(x, y),
            L1=L1,
            L2=L2
        )

except ValueError as error:
    print("IK Error:", error)
