#!/user/bin/env python
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge, CvBridgeError


class CableRTViz:
    def __init__(self, T_proj: np.array()) -> None:
        """
        Initialize ROS node

        Parameters
        ----------
        T_proj : np.array() (3x4) Transformation matrix from Polaris to Camera

        """

        rospy.init_node("RT_viz", anonymous=False)
        rospy.loginfo("realtime_visualization node started")
        rospy.Subscriber("/interplated_pts", PoseArray, self.CB_interp)
        rospy.Subscriber("/camera/color/image_raw", PoseArray, self.CB_image)
        rospy.Subscriber("/camera/depth/color/points", PoseArray, self.CB_pointcloud)

        rospy.Rate(50)

        # Define the class variables
        self.point_list = None
        self.rgb_img = None
        self.point_cloud = None
        self.T_proj = T_proj

    def CB_interp(self, msg):
        self.point_list = []
        for pose in msg.poses:
            point = np.array([pose.position.x, pose.position.y, pose.position.z, 1])
            point = np.reshape(point,(4,1))
            pixel_coord = self.T_proj @ point
            self.point_list.append(
                pixel_coord[0] / pixel_coord[2], pixel_coord[1] / pixel_coord[2]
            )

        rospy.loginfo("Point List is Captured")

    def CB_image(self, msg):
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to a OpenCV image
            self.rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")

            # cv2.imshow("figure1", cv_image)
            # cv2.waitKey(0)

        except CvBridgeError as e:
            rospy.logerr(e)

        pass

    def CB_depth(self, msg):
        try:
            depth_img = CvBridge().imgmsg_to_cv2(msg, "32FC1")
            depth_img = np.array(depth_img, dtype=np.float32)
            depth_img = cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        while not rospy.is_shutdown():
            if self.point_list is not None:
                # cv2.imshow("show RGB Image", self.rgb_img)
                # cv2.waitKey(1)
                # cv2.imshow("show depth Image", self.depth_img)
                pts = np.array(self.point_list, np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(
                    self.rgb_img,
                    [pts],
                    isClosed=False,
                    color=(0, 255, 255),
                    thickness=1,
                )
            rospy.spin()


if __name__ == "__main__":
    T_NDI_to_camera = np.array(
        [[ 5.01162310e+02 -9.82193900e+02 -3.55510689e+02  1.35128111e+02],
         [ 4.13477157e+01  1.52610502e+02 -1.00307983e+03 -6.38292381e+02],
         [ 9.44384216e-01 -4.58204811e-02 -3.25636203e-01  4.92087895e-01]])
    try:
        cable_rtviz = CableRTViz(T_proj=T_NDI_to_camera)
        cable_rtviz.run()
    except rospy.ROSInterruptException:
        pass
