#!/user/bin/env python
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CableRTViz:
    def __init__(self, T_proj: np.ndarray):
        """
        Initialize ROS node

        Parameters
        ----------
        T_proj : np.array() (3x4) Transformation matrix from Polaris to Camera

        """

        rospy.init_node("RT_viz", anonymous=False)
        rospy.loginfo("realtime_visualization node started")
        rospy.Subscriber("/sorted_pts", PoseArray, self.CB_interp)
        rospy.Subscriber("/camera/color/image_raw", Image, self.CB_image)
        # rospy.Subscriber("/camera/depth/color/points", PoseArray, self.CB_depth)
        self.rate = rospy.Rate(50)

        # Define the class variables
        self.point_list = None
        self.rgb_img = None
        self.depth_img = None
        self.T_proj = T_proj

    def CB_interp(self, msg):
        self.point_list = []
        for pose in msg.poses:
            point = np.array([pose.position.x, pose.position.y, pose.position.z, 1])
            point = np.reshape(point, (4, 1))
            pixel_coord = self.T_proj @ point
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
            rospy.loginfo("Depth Image is Captured")
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        while not rospy.is_shutdown():
            # print(len(self.point_list))
            if self.point_list is not None and self.rgb_img is not None:
                rospy.loginfo("Start to show image")
                pts = np.array(self.point_list, np.int32)
                pts = pts.reshape((-1, 1, 2))
                # cv2.polylines(
                #     self.rgb_img,
                #     [pts],
                #     isClosed=False,
                #     color=(0, 255, 255),
                #     thickness=1,
                # )
                rgb_img = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2RGB)
                plt.imshow(rgb_img)
                rospy.loginfo("Image is shown")
                plt.draw()
                plt.pause(0.001)
                plt.show()

            self.rate.sleep()


if __name__ == "__main__":
    T_NDI_to_camera = np.array(
        [
            [5.01162310e02, -9.82193900e02, -3.55510689e02, 1.35128111e02],
            [4.13477157e01, 1.52610502e02, -1.00307983e03, -6.38292381e02],
            [9.44384216e-01, -4.58204811e-02, -3.25636203e-01, 4.92087895e-01],
            [0, 0, 0, 1],
        ]
    )
    T_inrinsic = np.array(
        [
            [969.650634765625, 0.0, 634.0615234375, 0],
            [0.0, 950.003662109375, 358.694549561, 0],
            [0, 0, 1, 0],
        ]
    )

    T_proj = T_inrinsic @ T_NDI_to_camera
    try:
        cable_rtviz = CableRTViz(T_proj=T_proj)
        cable_rtviz.run()
    except rospy.ROSInterruptException:
        pass
