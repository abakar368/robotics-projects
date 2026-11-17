import math

def rotation_z(angle_deg):
    """
    Rotation of matrix about the z-axis

    3x3 rotation
    """

    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)

    R = [
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ]

    return R


def rotation_x(angle_deg):

    angle = math.radians(angle_deg)

    c = math.cos(angle)
    s = math.sin(angle)

    return [
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c]
    ]


def rotation_y(angle_deg):

    angle = math.radians(angle_deg)

    c = math.cos(angle)
    s = math.sin(angle)

    return [
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ]



def multiply_matrix_vector(matrix, vector):
    """
    Multiply a 3x3 matrix by a 3D vector.
    """

    result = []

    for row in matrix:
        value = sum(
            row[i] * vector[i]
            for i in range(3)
        )

        result.append(value)

    return result
