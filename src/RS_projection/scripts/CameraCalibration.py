import numpy as np
import rosbag
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from draw_points import interpolation

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

def GetData(msg):
    # print(len(msg.poses))
    for i in range(0, len(msg.poses)):
        point_list.append(
            [msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]
        )

    return point_list


if __name__ == "__main__":
    image = mpimg.imread("received_image.png")
    bag = rosbag.Bag("2023-08-22-11-58-36.bag", "r")

    tool_tip = np.array([-0.1666, 0.0008, -0.0009])

    translation1 = np.array([-0.1075, 0.04522, -0.9059])
    rotation_homo1 = aruco_transformation_matrix(0, 0, 0, 0.242124147035, -0.188318781027, 0.9473944836263, 0.09140911622869652)

    translation2 = np.array([-0.12205, -0.08675, -0.9263])
    rotation_homo2 = aruco_transformation_matrix(0, 0, 0, 0.2353244020524389, -0.18281895748060276, 0.9510986245298313, 0.08130843130838622)

    translation3 = np.array([-0.00881, -0.12501, -0.84154])
    rotation_homo3 = aruco_transformation_matrix(0, 0, 0, 0.2804195574, -0.1273089781942, 0.95126613807, 0.01580)

    translation4 = np.array([0.00067, -0.01303, -0.83334])
    rotation_homo4 = aruco_transformation_matrix(0, 0, 0,  0.256423093221537, -0.08500765571, 0.9612865725606135, -0.05430489064715076)

    P_x = -0.00015562823682557791
    P_y = 0.004592740908265114
    P_z = 0.8858861327171326
    Q_x = 0.6797390030145669
    Q_y = 0.6641538703448957
    Q_z = -0.222600551109178
    Q_w = 0.21749372159340236

    rotation1 = np.array([[rotation_homo1[0][0], rotation_homo1[0][1], rotation_homo1[0][2]], [rotation_homo1[1][0], rotation_homo1[1][1], rotation_homo1[1][2]], [rotation_homo1[2][0], rotation_homo1[2][1], rotation_homo1[2][2]]])
    rotation2 = np.array([[rotation_homo2[0][0], rotation_homo2[0][1], rotation_homo2[0][2]], [rotation_homo2[1][0], rotation_homo2[1][1], rotation_homo2[1][2]], [rotation_homo2[2][0], rotation_homo2[2][1], rotation_homo2[2][2]]])
    rotation3 = np.array([[rotation_homo3[0][0], rotation_homo3[0][1], rotation_homo3[0][2]], [rotation_homo3[1][0], rotation_homo3[1][1], rotation_homo3[1][2]], [rotation_homo3[2][0], rotation_homo3[2][1], rotation_homo3[2][2]]])
    rotation4 = np.array([[rotation_homo4[0][0], rotation_homo4[0][1], rotation_homo4[0][2]], [rotation_homo4[1][0], rotation_homo4[1][1], rotation_homo4[1][2]], [rotation_homo4[2][0], rotation_homo4[2][1], rotation_homo4[2][2]]])
    
    point1 = np.transpose(rotation1.dot(np.transpose(tool_tip)) + np.transpose(translation1))
    point2 = np.transpose(rotation2.dot(np.transpose(tool_tip)) + np.transpose(translation2))
    point3 = np.transpose(rotation3.dot(np.transpose(tool_tip)) + np.transpose(translation3))
    point4 = np.transpose(rotation4.dot(np.transpose(tool_tip)) + np.transpose(translation4))
    

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
    print("z axis: ", z)
    print("cneter point: ", center_point)
    #construct transformation matrix
    matrix_NDI = np.array([[x[0], y[0], z[0], center_point[0]], [x[1], y[1], z[1], center_point[1]], [x[2], y[2], z[2], center_point[2]], [0, 0, 0, 1]])
    #print("NDI transformation: ", matrix_NDI)
    print("determin: ", np.linalg.det(matrix_NDI))

    #find the transformation from NDI optical tracker to real sense
    matrix_rs = aruco_transformation_matrix(P_x, P_y, P_z, Q_x, Q_y, Q_z, Q_w)
    #print(matrix_rs)
    matrix_NDI_rs = np.matmul(matrix_NDI, np.linalg.inv(matrix_rs))
    matrix_rs_NDI = np.matmul(matrix_rs, np.linalg.inv(matrix_NDI))
    print("NDI to RS transformation: \n", matrix_NDI_rs)
    print("RS to NDI transformation: \n", matrix_rs_NDI)
    result = np.matmul(matrix_NDI_rs, matrix_rs_NDI)
    if np.linalg.det(result) <= 1.01:
        print('')
    else:
        print("error value")

    
    point_list = []
    matrix_rsc_im = np.array([[969.650634765625, 0.0, 634.0615234375, 0],
                              [0.0, 950.003662109375, 358.694549561, 0],
                              [0, 0, 1, 0],])
    
    # matrix_rsc_im = np.array([[916.43481445,   0,  630.85916934, 0],
    #                           [0.0,  911.37298584, 362.27576711, 0],
    #                           [0, 0, 1, 0]])
    max_iter = 0
    for topic, msg, t in bag.read_messages(topics="/NDI/measured_cp_array"):
        # if t.secs == 1691792439 and t.nsecs == 314266213:
        max_iter = max_iter + 1
        if max_iter == 110:
            point_list = GetData(msg)
            #print(point_list)

    point_list.append(center_point)
    list_pts = []
    for p in point_list:
        #print(p)
        image_points = matrix_rs_NDI.dot(np.transpose(np.append(np.array(p), 1)))
        #ew_point = np.linalg.inv(matrix_rsd_rsc).dot(image_points)
        #print(image_points)
        final_point = matrix_rsc_im.dot(image_points)
        pix = final_point[0:2] / final_point[2]
        print(pix)
        list_pts.append(pix)  # scale the points with z normalized to 1
    #print(list_pts)

    # sort the datapoints
    order = np.array([6, 2, 0, 3, 5, 8, 7, 4, 1, 9, 10])
    pts = np.array(list_pts)
    pts = pts[order]
    print(pts)

    plt.imshow(image)
    interpolation(pts, plt.gca())
    # plt.plot(640, 570, "og", markersize=10)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="o", color="red", s=250)
    plt.show()
    bag.close


    #Test Code
    pointed_mid_point = np.array([-0.11274, -0.09907, -0.95352])
    trans_matrix = aruco_transformation_matrix(0, 0, 0, 0.1031086346786171, -0.14071178369817097, 0.9821822513857408, 0.06990585416135148)
    roation_matrix = np.array([[trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]], [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]], [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]]])
    new_tip = roation_matrix.dot(np.transpose(tool_tip)) + np.transpose(pointed_mid_point)
    
    RS_mid_point = np.array([0.07969529926776886, 0.03305203467607498, 0.8687229752540588, 1])
    NDI_mid_point = matrix_rs_NDI.dot(np.append(new_tip, 1))
    vector_error = NDI_mid_point[0:3] - RS_mid_point[0:3]
    space_error = np.linalg.norm(vector_error)
    pixel_error = np.linalg.norm(matrix_rsc_im.dot(np.append(vector_error, 1)))
    print("NDI to image", matrix_rsc_im @ matrix_rs_NDI)
    print("space error", space_error)
    print("pixel error", pixel_error)
    #trans_matrix = aruco_transformation_matrix(0, 0, 0, 0.322731869732042, -0.4045399482696, 0.825181486569, 0.2264223591798)
    #roation_matrix = np.array([[trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]], [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]], [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]]])
    #new_tip = roation_matrix.dot(np.transpose(tool_tip)) + np.transpose(exam_point)
    #new_exam_point = matrix_rs_NDI.dot(np.append(new_tip, 1))
    # point_RS = matrix_NDI_rs.dot(new_exam_point)
    #print(new_exam_point)
    #print(np.linalg.norm(new_exam_point[0:3]))
    #print(np.linalg.norm([0.09817775338888168, -0.037160724401474, 0.9384751915931702]))