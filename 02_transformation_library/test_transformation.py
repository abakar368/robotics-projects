from transformations import rotation_z, rotation_y, rotation_x, multiply_matrix_vector

R = rotation_z(90)

# point = [1, 0, 0]

# rotated_point = multiply_matrix_vector(R, point)

# print("Original point:", point)
# print("Rotated point:", rotated_point)

R = rotation_y(90)

point = [1, 0, 0]

result = multiply_matrix_vector(R, point)

print(result)
