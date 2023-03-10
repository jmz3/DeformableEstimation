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
        for i in range(len(msg.poses)):
            self.sorted_pointset.poses[i].position.x = .001 * msg.poses[i].position.x
            self.sorted_pointset.poses[i].position.y = .001 * msg.poses[i].position.y
            self.sorted_pointset.poses[i].position.z = .001 * msg.poses[i].position.z


    def free_end_callback(self, msg):
        self.free_end_pose = msg

