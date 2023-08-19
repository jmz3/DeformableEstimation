#!/user/bin/env python
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge, CvBridgeError


class CableRTViz:
    def __init__(self) -> None:
        # Initialize ROS node
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

    def CB_interp(self, msg):
        self.point_list = []
        for pose in msg.poses:
            self.point_list.append(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]
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
                cv2.imshow("show RGB Image", self.rgb_img)
                cv2.waitKey(1)
                cv2.imshow("show depth Image", self.depth_img)

                cv2.scatter(
                    self.rgb_img, self.point_list, 1, (0, 0, 255)
                )  # visualize the points on the image, have to be 2D points
            rospy.spin()


if __name__ == "__main__":
    try:
        cable_rtviz = CableRTViz()
        cable_rtviz.run()
    except rospy.ROSInterruptException:
        pass
