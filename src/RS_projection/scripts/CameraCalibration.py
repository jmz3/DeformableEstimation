import numpy as np
import rosbag
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

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
    image = mpimg.imread("./received_image.png")
    bag = rosbag.Bag("2023081.bag", "r")

    tool_tip = np.array([-0.1666, 0.0008, -0.0009])

    translation1 = np.array([0.06367,  -0.06774,  -1.12689])
    rotation_homo1 = aruco_transformation_matrix(0, 0, 0, 0.365935007, -0.11271078, 0.9237883738, 0.0017)

    translation2 = np.array([0.08778,   0.05318,  -1.11543])
    rotation_homo2 = aruco_transformation_matrix(0, 0, 0, 0.30462706122, -0.09470841332, 0.9475841776414, -0.017801581385)

    translation3 = np.array([-0.02501,  0.10163,  -1.18093])
    rotation_homo3 = aruco_transformation_matrix(0, 0, 0, 0.2868184645, -0.162610468386, 0.9440607758687, 0.0065)

    translation4 = np.array([-0.01749, -0.01068,  -1.16211])
    rotation_homo4 = aruco_transformation_matrix(0, 0, 0,  0.454628917592, -0.1181075124672533, 0.8811560477129289, 0.0541034413)

    P_x = 0.009141292423009872
    P_y = 0.032449230551719666
    P_z = 0.8945698738098145
    Q_x = -0.6617885447981704
    Q_y = 0.6834627280713695
    Q_z = -0.22208728048439078
    Q_w = -0.21352250738085848

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
    #print("NDI to RS transformation: \n", matrix_NDI_rs)
    print("RS to NDI transformation: \n", matrix_rs_NDI)
    result = np.matmul(matrix_NDI_rs, matrix_rs_NDI)
    if np.linalg.det(result) <= 1.01:
        print('')
    else:
        print("error value")

    
    point_list = []
    matrix_rsc_im = np.array([[961.650634765625, 0.0, 636.0615234375, 0],
                               [0.0, 952.003662109375, 357.694549561, 0],
                               [0, 0, 1, 0]])
    
    # matrix_rsc_im = np.array([[916.43481445,   0,  630.85916934, 0],
    #                           [0.0,  911.37298584, 362.27576711, 0],
    #                           [0, 0, 1, 0]])
    max_iter = 0
    for topic, msg, t in bag.read_messages(topics="/NDI/measured_cp_array"):
        # if t.secs == 1691792439 and t.nsecs == 314266213:
        max_iter = max_iter + 1
        if max_iter == 100:
            point_list = GetData(msg)
            #print(point_list)

    list_pts = []
    for p in point_list:
        image_points = matrix_rs_NDI.dot(np.transpose(np.append(np.array(p), 1)))
        #new_point = np.linalg.inv(matrix_rsd_rsc).dot(image_points)
        # print(new_point)
        final_point = matrix_rsc_im.dot(image_points)
        list_pts.append(
            final_point[0:2] / final_point[2]
        )  # scale the points with z normalized to 1
    #print(list_pts)

    
    pts = np.array(list_pts)
    plt.imshow(image)
    # plt.plot(640, 570, "og", markersize=10)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="o", color="red", s=250)
    plt.show()
    bag.close


    #Test Code
    #exam_point = np.array([0.07848, -0.06649, -1.11854])
    #tool_tip = np.array([-0.1666, 0.0008, -0.0009])
    #trans_matrix = aruco_transformation_matrix(0, 0, 0, 0.322731869732042, -0.4045399482696, 0.825181486569, 0.2264223591798)
    #roation_matrix = np.array([[trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]], [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]], [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]]])
    #new_tip = roation_matrix.dot(np.transpose(tool_tip)) + np.transpose(exam_point)
    #new_exam_point = matrix_rs_NDI.dot(np.append(new_tip, 1))
    # point_RS = matrix_NDI_rs.dot(new_exam_point)
    #print(new_exam_point)
    #print(np.linalg.norm(new_exam_point[0:3]))
    #print(np.linalg.norm([0.09817775338888168, -0.037160724401474, 0.9384751915931702]))