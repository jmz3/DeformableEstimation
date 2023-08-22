import rosbag
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from scipy.interpolate import UnivariateSpline, Rbf


def GetData(msg):
    # print(len(msg.poses))
    for i in range(0, len(msg.poses)):
        point_list.append(
            [msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]
        )

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

    rot_matrix = np.array(
        [[r00, r01, r02, P_x], [r10, r11, r12, P_y], [r20, r21, r22, P_z], [0, 0, 0, 1]]
    )
    return rot_matrix


def interpolation(points, ax):
    """
    Polynomial regression via least squares
    """
    distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0) / distance[-1]

    print(distance)

    splines = [Rbf(distance, coords, k=3, s=0.2) for coords in points.T]

    alpha = np.linspace(0, 1, 75)
    points_fitted = np.vstack(s(alpha) for s in splines).T
    ax.plot(*points_fitted.T, "-r", label="fitted spline k=3, s=.2", lw=3.5)

    # model_samples = np.linspace(X.min(), X.max(), 100)

    # interp_values = interp_func(model_samples)
    # ax.plot(
    #     model_samples,
    #     interp_values,
    #     color="red",
    # )


if __name__ == "__main__":
    bag = rosbag.Bag("bag/2023081.bag", "r")
    point_list = []
    matrix_rsd_NDI = np.array(
        [
            [-0.31151697, -0.92261727, -0.22745229, -0.20253537],
            [-0.36595512, 0.3373835, -0.86732302, -1.01179961],
            [0.87694585, -0.18694851, -0.4427372, 0.21261319],
            [0, 0, 0, 1],
        ]
    )

    matrix_rsd_rsc = QuaToRot(-0.015, 0, 0, 0.002, -0.002, -0.001, 1.00)
    matrix_rsd_rsc = np.eye(4)

    matrix_rsc_im = np.array(
        [
            [931.650634765625, 0.0, 634.0615234375, 0],
            [0.0, 932.003662109375, 352.694549561, 0],
            [0, 0, 1, 0],
        ]
    )

    matrix_rsd_im = np.matmul(matrix_rsc_im, np.linalg.inv(matrix_rsd_rsc))
    matrix_rs_im = np.matmul(matrix_rsd_im, matrix_rsd_NDI)
    print(matrix_rs_im)
    max_iter = 0
    for topic, msg, t in bag.read_messages(topics="/NDI/measured_cp_array"):
        # if t.secs == 1691792439 and t.nsecs == 314266213:
        max_iter = max_iter + 1
        if max_iter == 100:
            point_list = GetData(msg)
            print(point_list)

    list_pts = []
    for p in point_list:
        image_points = matrix_rsd_NDI.dot(np.transpose(np.append(np.array(p), 1)))
        new_point = np.linalg.inv(matrix_rsd_rsc).dot(image_points)
        # print(new_point)
        final_point = matrix_rsc_im.dot(new_point)
        print(final_point)
        list_pts.append(
            final_point[0:2] / final_point[2]
        )  # scale the points with z normalized to 1

    # print(list_pts)

    # sort the datapoints
    order = np.array([6, 2, 0, 3, 5, 8, 7, 4, 1])
    pts = np.array(list_pts)
    pts = pts[order]
    print(pts)
    # plot the points on the image

    image = mpimg.imread("./received_image.png")

    plt.imshow(image)
    # plt.plot(640, 570, "og", markersize=10)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="o", color="green", s=30)

    # plot the line
    x = pts[:, 0]
    y = pts[:, 1]
    interpolation(pts, plt.gca())
    # linreg(x, y, plt.gca())

    plt.xlim(0, 1280)
    plt.ylim(720, 0)
    plt.show()
    bag.close
