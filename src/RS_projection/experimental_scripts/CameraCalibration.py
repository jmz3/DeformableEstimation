import numpy as np
def aruco_transformation_matrix(P_x, P_y, P_z, Q_x, Q_y, Q_z, Q_w):
    # First row of the rotation matrix
    r00 = 2 * (Q_w * Q_w + Q_x * Q_x) - 1
    r01 = 2 * (Q_x * Q_y - Q_w * Q_z)
    r02 = 2 * (Q_x * Q_z + Q_w * Q_y)
     
    # Second row of the rotation matrix
    r10 = 2 * (Q_x * Q_y + Q_w * Q_z)
    r11 = 2 * (Q_w * Q_w + Q_y * Q_y) - 1
    r12 = 2 * (Q_y * Q_z - Q_w * Q_x)
     
    # Third row of the rotation matrix
    r20 = 2 * (Q_x * Q_z - Q_w * Q_y)
    r21 = 2 * (Q_y * Q_z + Q_w * Q_x)
    r22 = 2 * (Q_w * Q_w + Q_z * Q_z) - 1

    rot_matrix = np.array([[r00, r01, r02, P_x],
                           [r10, r11, r12, P_y],
                           [r20, r21, r22, P_z], 
                           [0, 0, 0, 1]])
    return rot_matrix

if __name__ == "__main__":
    #initial four points in the order
    point1 = np.array([0.1850,   -0.0551,   -1.2404])
    point2 = np.array([0.2228,    0.0678,   -1.2119])
    point3 = np.array([0.1136,    0.1146,   -1.2725])
    point4 = np.array([0.0784,   -0.0092,   -1.2983])

    #calculate the center point 
    center_point = (point1 + point2 + point3 + point4) /4

    #compute the Z axis
    vector_c1 = point1 - center_point
    vector_c2 = point2 - center_point
    z_axis = np.cross(vector_c1, vector_c2)

    #compute the X axis
    midpoint_x = (point1 + point2) / 2
    x_axis = midpoint_x - center_point

    #compute the Y axis
    #midpoint_y = (point2 + point3) / 2
    #y_axis2 = midpoint_y - center_point
    y_axis = np.cross(z_axis, x_axis)
    
    #normalize X,Y,Z axis
    x = x_axis / np.linalg.norm(x_axis)
    y = y_axis / np.linalg.norm(y_axis)
    z = z_axis / np.linalg.norm(z_axis)
    print("x axis: ", x)
    print("y axis: ", y)
    print("y axis: ", z)
    print("cneter point: ", center_point)
    #construct transformation matrix
    matrix_NDI = np.array([[x[0], y[0], z[0], center_point[0]], [x[1], y[1], z[1], center_point[1]], [x[2], y[2], z[2], center_point[2]], [0, 0, 0, 1]])
    print("NDI transformation: ", matrix_NDI)
    print("determin: ", np.linalg.det(matrix_NDI))

    #find the transformation from NDI optical tracker to real sense
    P_x = 0.009141292423009872
    P_y = 0.032449230551719666
    P_z = 0.8945698738098145
    Q_x = -0.6617885447981704
    Q_y = 0.6834627280713695
    Q_z = -0.22208728048439078
    Q_w = -0.21352250738085848
    matrix_rs = aruco_transformation_matrix(P_x, P_y, P_z, Q_x, Q_y, Q_z, Q_w)
    print(matrix_rs)
    matrix_NDI_rs = np.matmul(matrix_NDI, np.linalg.inv(matrix_rs))
    matrix_rs_NDI = np.matmul(matrix_rs, np.linalg.inv(matrix_NDI))
    print("NDI to RS transformation: \n", matrix_NDI_rs)
    print("RS to NDI transformation: \n", matrix_rs_NDI)
    result = np.matmul(matrix_NDI_rs, matrix_rs_NDI)
    if np.linalg.det(result) <= 1.01:
        print('')
    else:
        print("error value")

    exam_point = np.array([0.07848, -0.06649, -1.11854])
    tool_tip = np.array([-0.1666, 0.0008, -0.0009])
    trans_matrix = aruco_transformation_matrix(0, 0, 0, 0.322731869732042, -0.4045399482696, 0.825181486569, 0.2264223591798)
    roation_matrix = np.array([[trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]], [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]], [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]]])
    new_tip = roation_matrix.dot(np.transpose(tool_tip)) + np.transpose(exam_point)
    new_exam_point = matrix_rs_NDI.dot(np.append(new_tip, 1))
    # point_RS = matrix_NDI_rs.dot(new_exam_point)
    print(new_exam_point)
    print(np.linalg.norm(new_exam_point[0:3]))
    print(np.linalg.norm([0.09817775338888168, -0.037160724401474, 0.9384751915931702]))