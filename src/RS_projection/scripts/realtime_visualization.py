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

    def CB_pointcloud(self, msg):
        pass

    def run(self):
        plt.plot(self.point_list)
        rospy.spin()


if __name__ == "__main__":
    try:
        cable_rtviz = CableRTViz()
        cable_rtviz.run()
    except rospy.ROSInterruptException:
        pass
