#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped

class SortedSubscriber:
    def __init__(self) -> None:
        self.sorted_pointset = None
        self.free_end_pose = None
        self.sub_pointset = rospy.Subscriber("/Sorted", PoseArray, self.sorted_callback)
        self.sub_free_end = rospy.Subscriber("/NDI/PointerNew/measured_cp", TransformStamped,self.free_end_callback)
        pass

    def sorted_callback(self, msg):
        self.sorted_pointset = msg

    def free_end_callback(self, msg):
        self.free_end_pose = msg

