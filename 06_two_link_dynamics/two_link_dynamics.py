
import numpy as np


# Robot parameters
M1 = 2.0       # Mass at elbow (kg)
M2 = 1.0       # Mass at endpoint (kg)
L1 = 0.4       # First link length (m)
L2 = 0.3       # Second link length (m)
G = 10.0       # Gravitational acceleration (m/s^2)


def mass_matrix(q):
    """
    Calculate the 2x2 inertia matrix M(q).

    q = [q1, q2] in radians
    """
    q1, q2 = q

    M11 = (
        (M1 + M2) * L1**2
        + M2 * L2**2
        + 2 * M2 * L1 * L2 * np.cos(q2)
    )

    M12 = M2 * (
        L2**2 + L1 * L2 * np.cos(q2)
    )

    M22 = M2 * L2**2

    return np.array([
        [M11, M12],
        [M12, M22]
    ])


def coriolis_centrifugal(q, dq):
    """
    Calculate the combined Coriolis and centrifugal
    joint torque vector.

    dq = [q1_dot, q2_dot]
    """
    q1, q2 = q
    dq1, dq2 = dq

    h = M2 * L1 * L2 * np.sin(q2)

    tau1 = -h * (2 * dq1 * dq2 + dq2**2)
    tau2 = h * dq1**2

    return np.array([tau1, tau2])


def gravity_vector(q):
    """
    Calculate gravity compensation torques.

    q1 is measured from horizontal.
    q2 is relative to link 1.
    """
    q1, q2 = q

    G1 = G * (
        (M1 + M2) * L1 * np.cos(q1)
        + M2 * L2 * np.cos(q1 + q2)
    )

    G2 = G * M2 * L2 * np.cos(q1 + q2)

    return np.array([G1, G2])


def inverse_dynamics(q, dq, ddq):
    """
    Calculate required joint torques:

    tau = M(q)ddq + C(q,dq)dq + G(q)
    """
    M = mass_matrix(q)
    C = coriolis_centrifugal(q, dq)
    gravity = gravity_vector(q)

    inertia_torque = M @ ddq

    total_torque = inertia_torque + C + gravity

    return {
        "mass_matrix": M,
        "inertia_torque": inertia_torque,
        "coriolis_centrifugal_torque": C,
        "gravity_torque": gravity,
        "total_torque": total_torque
    }


if __name__ == "__main__":

    q = np.array([0.0, 0.0])
    dq = np.array([2.0, 1.0])
    ddq = np.array([3.0, 2.0])

    result = inverse_dynamics(q, dq, ddq)

    print("\n--- TWO-LINK ROBOT DYNAMICS ---")

    for name, value in result.items():
        print(f"\n{name}:")
        print(value)
