#!usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from matplotlib import pyplot as plt;


def callback(data):
    rospy.loginfo('Pose are successfully catched in Pyhton!')

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Interp", PoseArray, callback)