#!/user/bin/env python
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib.animation import FuncAnimation


class CableRTViz:
    def __init__(self, T_RS_to_NDI: np.ndarray, T_inrinsic: np.ndarray):
        """
        Initialize ROS node

        Parameters
        ----------
        T_proj : np.array() (3x4) Transformation matrix from Polaris to Camera

        """

        rospy.init_node("RT_viz", anonymous=False)
        # rospy.loginfo("realtime_visualization node started")
        rospy.Subscriber("/DOSE/interpolated_pts", PoseArray, self.CB_interp)
        rospy.Subscriber("/camera/color/image_raw", Image, self.CB_image)
        # rospy.Subscriber("/camera/depth/color/points", PoseArray, self.CB_depth)
        self.pub_image_ = rospy.Publisher("/DOSE/overlay", Image, queue_size=10)
        self.rate = rospy.Rate(50)

        # Define the class variables
        self.point_list = None
        self.rgb_img = None
        self.depth_img = None
        self.T_RS_to_NDI = T_RS_to_NDI
        self.T_inrinsic = T_inrinsic

    def CB_interp(self, msg):
        self.point_list = []
        for pose in msg.poses:
            point = np.array(
                [
                    pose.position.x / 1000,
                    pose.position.y / 1000,
                    pose.position.z / 1000,
                    1,
                ]
            )
            # point = np.reshape(point, (4, 1))
            p1 = self.T_RS_to_NDI.dot(np.transpose(point))
            # print(p1)
            pixel_coord = self.T_inrinsic.dot(p1)
            self.point_list.append(pixel_coord[0:2] / pixel_coord[2])

        # print(self.point_list)
        # rospy.loginfo("Point List is Captured")

    def CB_image(self, msg):
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to a OpenCV image
            self.rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            # rospy.loginfo("Image is Captured")
        except CvBridgeError as e:
            rospy.logerr(e)

        pass

    def CB_depth(self, msg):
        try:
            depth_img = CvBridge().imgmsg_to_cv2(msg, "32FC1")
            depth_img = np.array(depth_img, dtype=np.float32)
            self.depth_img = cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)
            # rospy.loginfo("Depth Image is Captured")
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        while not rospy.is_shutdown():
            # print(len(self.point_list))
            # plt.ioff()
            # plt.show()
            if self.point_list is not None and self.rgb_img is not None:
                # plt.show()
                plt.ioff()
                # rospy.loginfo("Start to show image")
                pts = np.array(self.point_list, np.int32)
                # cv2.polylines(
                #     self.rgb_img,
                #     [pts],
                #     isClosed=False,
                #     color=(0, 255, 255),
                #     thickness=1,
                # )
                print(pts)
                # Publish the image to rostopics
                for p in pts:
                    cv2.circle(self.rgb_img, (p[0], p[1]), 10, (0, 0, 255), -1)

                self.pub_image_.publish(CvBridge().cv2_to_imgmsg(self.rgb_img, "bgr8"))

                # rgb_img = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2RGB)
                # # plt.clear()
                # plt.imshow(rgb_img)
                # rospy.loginfo("Image is shown")
                # plt.draw()
                # plt.pause(0.00000001)
                # plt.scatter(pts[:, 0], pts[:, 1], marker="o", color="red", s=250)
                # plt.show()
                # plt.ioff()
                # plt.show(block=True)

            self.rate.sleep()


if __name__ == "__main__":
    T_RS_to_NDI = np.array(
        [
            [-0.30795141, -0.93400759, -0.18109599, -0.19575269],
            [-0.35330252,  0.28900212, -0.88975002, -0.78584078],
            [ 0.8833704,  -0.2100181,  -0.41898585,  0.30760982],
            [ 0.,          0.,          0.,          1.        ]
        ]
    )

    # T_NDI_to_camera = np.array(
    #     [
    #         [-0.1006913, -0.31304906, 0.94438422, -0.75158],
    #         [-0.98297352, 0.17794253, -0.04582048, -0.00415],
    #         [-0.15370206, -0.9329184, -0.3256362, -0.66794],
    #         [
    #             0.0,
    #             0.0,
    #             0.0,
    #             1.0,
    #         ],
    #     ]
    # )

    T_inrinsic = np.array(
        [
            [945.650634765625, 0.0, 634.0615234375, 0],
            [0.0, 950.003662109375, 358.694549561, 0],
            [0, 0, 1, 0],
        ]
    )

    try:
        cable_rtviz = CableRTViz(T_RS_to_NDI=T_RS_to_NDI, T_inrinsic=T_inrinsic)
        cable_rtviz.run()
    except rospy.ROSInterruptException:
        pass
