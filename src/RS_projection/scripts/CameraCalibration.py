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

    rot_matrix = np.array(
        [[r00, r01, r02, P_x], [r10, r11, r12, P_y], [r20, r21, r22, P_z], [0, 0, 0, 1]]
    )
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
    bag = rosbag.Bag("NDI-0822.bag", "r")

    tool_tip = np.array([-0.1666, 0.0008, -0.0009])

    # Four points on AR tag
    translation1 = np.array([-0.07525, 0.06745000000000001, -0.9732500000000001])
    rotation_homo1 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.2828229548425604,
        -0.09100738645923974,
        0.939076218518968,
        0.17281402615556735,
    )

    translation2 = np.array([-0.12145, -0.08704, -1.00166])
    rotation_homo2 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.2328018787187419,
        0.011100089578084343,
        0.9705078320298068,
        0.06160049711801762,
    )

    translation3 = np.array([-0.0156, -0.10297, -0.94208])
    rotation_homo3 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.19022294036765186,
        -0.09191108422601053,
        0.9695169210956975,
        0.12411496792652783,
    )

    translation4 = np.array([0.0175, 0.03338, -0.9270900000000001])
    rotation_homo4 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.20410700609322832,
        -0.09930340864800377,
        0.9626330429463084,
        0.14770507006354638,
    )

    # For initial and end direction
    translation_pts1 = np.array([-0.13961, -0.16998, -0.95708])
    rotation_pts1 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.3191169806663239,
        -0.03480185185580718,
        0.9443502502137564,
        0.07180382078295848,
    )

    translation_pts2 = np.array([-0.1728, -0.14736000000000002, -0.97584])
    rotation_pts2 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.30362846498473756,
        -0.1223114666259335,
        0.9365878045395478,
        0.12511172914884935,
    )

    translation_pts3 = np.array([0.0577, -0.08747, -0.85682])
    rotation_pts3 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.23060449567846533,
        0.03600070184052364,
        0.9708189262994542,
        -0.05510107420591258,
    )

    translation_pts4 = np.array([0.08763, -0.00782, -0.82348])
    rotation_pts4 = aruco_transformation_matrix(
        0,
        0,
        0,
        0.23271042333329686,
        -0.318414262094206,
        0.892439973281625,
        0.21910981414836847,
    )

    # Group 1
    P_x = 0.033024862408638
    P_y = 0.035306621342897415
    P_z = 0.8528280854225159
    Q_x = 0.012882721105573876
    Q_y = 0.9497527004101783
    Q_z = -0.30987708136470304
    Q_w = -0.04219049662404029

    rotation1 = np.array(
        [
            [rotation_homo1[0][0], rotation_homo1[0][1], rotation_homo1[0][2]],
            [rotation_homo1[1][0], rotation_homo1[1][1], rotation_homo1[1][2]],
            [rotation_homo1[2][0], rotation_homo1[2][1], rotation_homo1[2][2]],
        ]
    )
    rotation2 = np.array(
        [
            [rotation_homo2[0][0], rotation_homo2[0][1], rotation_homo2[0][2]],
            [rotation_homo2[1][0], rotation_homo2[1][1], rotation_homo2[1][2]],
            [rotation_homo2[2][0], rotation_homo2[2][1], rotation_homo2[2][2]],
        ]
    )
    rotation3 = np.array(
        [
            [rotation_homo3[0][0], rotation_homo3[0][1], rotation_homo3[0][2]],
            [rotation_homo3[1][0], rotation_homo3[1][1], rotation_homo3[1][2]],
            [rotation_homo3[2][0], rotation_homo3[2][1], rotation_homo3[2][2]],
        ]
    )
    rotation4 = np.array(
        [
            [rotation_homo4[0][0], rotation_homo4[0][1], rotation_homo4[0][2]],
            [rotation_homo4[1][0], rotation_homo4[1][1], rotation_homo4[1][2]],
            [rotation_homo4[2][0], rotation_homo4[2][1], rotation_homo4[2][2]],
        ]
    )

    R_pts1 = np.array(
        [
            [rotation_pts1[0][0], rotation_pts1[0][1], rotation_pts1[0][2]],
            [rotation_pts1[1][0], rotation_pts1[1][1], rotation_pts1[1][2]],
            [rotation_pts1[2][0], rotation_pts1[2][1], rotation_pts1[2][2]],
        ]
    )
    R_pts2 = np.array(
        [
            [rotation_pts2[0][0], rotation_pts2[0][1], rotation_pts2[0][2]],
            [rotation_pts2[1][0], rotation_pts2[1][1], rotation_pts2[1][2]],
            [rotation_pts2[2][0], rotation_pts2[2][1], rotation_pts2[2][2]],
        ]
    )
    R_pts3 = np.array(
        [
            [rotation_pts3[0][0], rotation_pts3[0][1], rotation_pts3[0][2]],
            [rotation_pts3[1][0], rotation_pts3[1][1], rotation_pts3[1][2]],
            [rotation_pts3[2][0], rotation_pts3[2][1], rotation_pts3[2][2]],
        ]
    )
    R_pts4 = np.array(
        [
            [rotation_pts4[0][0], rotation_pts4[0][1], rotation_pts4[0][2]],
            [rotation_pts4[1][0], rotation_pts4[1][1], rotation_pts4[1][2]],
            [rotation_pts4[2][0], rotation_pts4[2][1], rotation_pts4[2][2]],
        ]
    )

    point1 = np.transpose(
        rotation1.dot(np.transpose(tool_tip)) + np.transpose(translation1)
    )
    point2 = np.transpose(
        rotation2.dot(np.transpose(tool_tip)) + np.transpose(translation2)
    )
    point3 = np.transpose(
        rotation3.dot(np.transpose(tool_tip)) + np.transpose(translation3)
    )
    point4 = np.transpose(
        rotation4.dot(np.transpose(tool_tip)) + np.transpose(translation4)
    )

    start1 = np.transpose(
        R_pts1.dot(np.transpose(tool_tip)) + np.transpose(translation_pts1)
    )
    start2 = np.transpose(
        R_pts2.dot(np.transpose(tool_tip)) + np.transpose(translation_pts2)
    )
    end1 = np.transpose(
        R_pts3.dot(np.transpose(tool_tip)) + np.transpose(translation_pts3)
    )
    end2 = np.transpose(
        R_pts4.dot(np.transpose(tool_tip)) + np.transpose(translation_pts4)
    )
    # calculate the center point
    center_point = (point1 + point2 + point3 + point4) / 4

    vec1 = start1 - start2
    vec2 = end2 - end1

    print("inital direction: ", vec1)
    print("end direction: ", vec2)
    print("start1: ", start1)

    # compute the Z axis
    vector_c1 = point1 - center_point
    vector_c2 = point2 - center_point
    z_axis = np.cross(vector_c1, vector_c2)

    # compute the X axis
    midpoint_x = (point1 + point2) / 2
    x_axis = midpoint_x - center_point

    # compute the Y axis
    # midpoint_y = (point2 + point3) / 2
    # y_axis2 = midpoint_y - center_point
    y_axis = np.cross(z_axis, x_axis)

    # normalize X,Y,Z axis
    x = x_axis / np.linalg.norm(x_axis)
    y = y_axis / np.linalg.norm(y_axis)
    z = z_axis / np.linalg.norm(z_axis)
    print("x axis: ", x)
    print("y axis: ", y)
    print("z axis: ", z)
    print("cneter point: ", center_point)
    # construct transformation matrix
    matrix_NDI = np.array(
        [
            [x[0], y[0], z[0], center_point[0]],
            [x[1], y[1], z[1], center_point[1]],
            [x[2], y[2], z[2], center_point[2]],
            [0, 0, 0, 1],
        ]
    )
    # print("NDI transformation: ", matrix_NDI)
    print("determin: ", np.linalg.det(matrix_NDI))

    # find the transformation from NDI optical tracker to real sense
    matrix_rs = aruco_transformation_matrix(P_x, P_y, P_z, Q_x, Q_y, Q_z, Q_w)
    # print(matrix_rs)
    matrix_NDI_rs = np.matmul(matrix_NDI, np.linalg.inv(matrix_rs))
    matrix_rs_NDI = np.matmul(matrix_rs, np.linalg.inv(matrix_NDI))
    print("NDI to RS transformation: \n", matrix_NDI_rs)
    print("RS to NDI transformation: \n", matrix_rs_NDI)
    result = np.matmul(matrix_NDI_rs, matrix_rs_NDI)
    if np.linalg.det(result) <= 1.01:
        print("")
    else:
        print("error value")

    point_list = []
    matrix_rsc_im = np.array(
        [
            [945.650634765625, 0.0, 634.0615234375, 0],
            [0.0, 950.003662109375, 358.694549561, 0],
            [0, 0, 1, 0],
        ]
    )

    # matrix_rsc_im = np.array([[916.43481445,   0,  630.85916934, 0],
    #                           [0.0,  911.37298584, 362.27576711, 0],
    #                           [0, 0, 1, 0]])
    max_iter = 0
    for topic, msg, t in bag.read_messages(topics="/NDI/measured_cp_array"):
        # if t.secs == 1691792439 and t.nsecs == 314266213:
        max_iter = max_iter + 1
        if max_iter == 110:
            point_list = GetData(msg)
            # print(point_list)

    point_list.append(center_point)
    list_pts = []
    for p in point_list:
        # print(p)
        image_points = matrix_rs_NDI.dot(np.transpose(np.append(np.array(p), 1)))
        # ew_point = np.linalg.inv(matrix_rsd_rsc).dot(image_points)
        # print(image_points)
        final_point = matrix_rsc_im.dot(image_points)
        pix = final_point[0:2] / final_point[2]
        print(pix)
        list_pts.append(pix)  # scale the points with z normalized to 1
    # print(list_pts)

    # sort the datapoints
    order = np.array([6, 2, 0, 3, 5, 8, 7, 4, 1, 9, 10])
    pts = np.array(list_pts)
    pts = pts[order]
    print(pts)

    plt.imshow(image)
    interpolation(pts, plt.gca())
    # plt.plot(640, 570, "og", markersize=10)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="o", color="red", s=250)
    # plt.show()
    bag.close

    # Test Code
    pointed_mid_point = np.array([0.03843, 0.19363, -0.9206900000000001])
    trans_matrix = aruco_transformation_matrix(
        0,
        0,
        0,
        0.21841214186845903,
        -0.05970331899975735,
        0.9661537099776479,
        0.12360687149698506,
    )
    roation_matrix = np.array(
        [
            [trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]],
            [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]],
            [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]],
        ]
    )
    new_tip = roation_matrix.dot(np.transpose(tool_tip)) + np.transpose(
        pointed_mid_point
    )

    RS_mid_point = np.array(
        [0.08087549358606339, -0.15578018128871918, 0.9792232513427734, 1]
    )
    NDI_mid_point = matrix_rs_NDI.dot(np.append(new_tip, 1))
    vector_error = NDI_mid_point[0:3] - RS_mid_point[0:3]
    space_error = np.linalg.norm(vector_error)
    pixel_error = np.linalg.norm(matrix_rsc_im.dot(np.append(vector_error, 1)))
    print("NDI to image", matrix_rsc_im @ matrix_rs_NDI)
    print("space error", space_error)
    print("pixel error", pixel_error)
    # trans_matrix = aruco_transformation_matrix(0, 0, 0, 0.322731869732042, -0.4045399482696, 0.825181486569, 0.2264223591798)
    # roation_matrix = np.array([[trans_matrix[0][0], trans_matrix[0][1], trans_matrix[0][2]], [trans_matrix[1][0], trans_matrix[1][1], trans_matrix[1][2]], [trans_matrix[2][0], trans_matrix[2][1], trans_matrix[2][2]]])
    # new_tip = roation_matrix.dot(np.transpose(tool_tip)) + np.transpose(exam_point)
    # new_exam_point = matrix_rs_NDI.dot(np.append(new_tip, 1))
    # point_RS = matrix_NDI_rs.dot(new_exam_point)
    # print(new_exam_point)
    # print(np.linalg.norm(new_exam_point[0:3]))
    # print(np.linalg.norm([0.09817775338888168, -0.037160724401474, 0.9384751915931702]))
