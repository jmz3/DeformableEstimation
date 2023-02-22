#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray

def catch_poses():
    pass

def plot_func():
    pass

if __name__ == "__main__":

    rospy.init_node('plot_node')
    rospy.loginfo("python node has started")
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        msg = "Hello"
        print(msg)
        rate.sleep()
        # rospy.spin()

    catch_poses()
    plot_func()

