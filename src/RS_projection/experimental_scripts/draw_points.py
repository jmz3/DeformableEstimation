import rosbag
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def GetData(msg):
    #print(len(msg.poses))
    for i in range(0, len(msg.poses)):
        point_list.append([msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]) 
    
    return point_list

def QuaToRot(P_x, P_y, P_z, Q_x, Q_y, Q_z, Q_w):
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
    bag  = rosbag.Bag('NDI_9points_0728.bag', 'r')
    point_list = []
    matrix_rsd_NDI = np.array([[0.82823066, -0.05809467,  0.55752979,  0.41930386], 
                              [0.36500011, -0.7000691,  -0.61638837, -1.00690144], 
                              [0.42525975,  0.71395135, -0.55682345,  0.54403331], 
                              [0, 0, 0, 1]])
    
    matrix_rsd_rsc = QuaToRot(-0.015, 0, 0, 0.002, -0.002, -0.001, 1.00)

    matrix_rsc_im = np.array([[931.650634765625, 0.0,  634.0615234375,  0], 
                              [0.0, 932.003662109375, 352.694549561, 0], 
                              [0,  0,  1,  0]])

    matrix_rsd_im = np.matmul(matrix_rsc_im, np.linalg.inv(matrix_rsd_rsc))
    matrix_rs_im = np.matmul(matrix_rsd_im, matrix_rsd_NDI)
    
    for topic, msg, t in bag.read_messages(topics = '/NDI/measured_cp_array'):
        if t.secs == 1690566201 and t.nsecs == 437192367:
            point_list = GetData(msg)
            print(point_list)
    
    list_pts = []
    for p in point_list:
        image_points = matrix_rsd_NDI.dot(np.transpose(np.append(np.array(p), 1)))
        #print(image_points)
        new_point = np.linalg.inv(matrix_rsd_rsc).dot(image_points)
        #print(new_point)
        final_point = matrix_rsc_im.dot(new_point)
        list_pts.append(final_point[0:2])
        #print(final_point)




    image = mpimg.imread("received_image.png")
    pts = np.array(list_pts)
    plt.imshow(image)
    plt.plot(640, 570, "og", markersize=10)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="o", color="green", s=200)
    plt.show()
    bag.close