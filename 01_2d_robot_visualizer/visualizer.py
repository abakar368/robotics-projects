
import math
import matplotlib.pyplot as plt


def forward_kinematics(theta1, theta2, L1, L2):

    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)

    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return (x1, y1), (x2, y2)


def draw_robot(theta1, theta2, target, L1=1.0, L2=1.0):

    shoulder = (0, 0)

    elbow, end_effector = forward_kinematics(
        theta1, theta2, L1, L2
    )

    target_x, target_y = target

    plt.figure(figsize=(7, 7))

    # First link
    plt.plot(
        [shoulder[0], elbow[0]],
        [shoulder[1], elbow[1]],
        "o-", linewidth=5, markersize=10,
        label="Link 1"
    )

    # Second link
    plt.plot(
        [elbow[0], end_effector[0]],
        [elbow[1], end_effector[1]],
        "o-", linewidth=5, markersize=10,
        label="Link 2"
    )

    # Calculated end-effector
    plt.scatter(
        end_effector[0], end_effector[1],
        marker="x", s=150, label="End-effector"
    )

    # Requested target
    plt.scatter(
        target_x, target_y,
        marker="*", s=180, label="Target"
    )

    plt.xlim(-2.2, 2.2)
    plt.ylim(-2.2, 2.2)

    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")

    plt.title(
        f"2DOF Robot Arm: "
        f"Shoulder={theta1:.2f}°, Elbow={theta2:.2f}°"
    )

    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.show()
