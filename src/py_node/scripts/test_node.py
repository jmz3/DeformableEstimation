#!/usr/bin/env python
import rospy
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseArray

def plot_callback(cable_array):
    # rospy.loginfo("the length of the msg inside %d", len(cable_array.poses))
    del temp_list[:]
    for i in range(len(cable_array.poses)):
        temp_list.append(cable_array.poses[i].position.x)

def main():
    rospy.init_node("hello_node")
    rospy.loginfo('this node has been successfully initiated')
    rospy.Subscriber("/NDI/measured_cp_array",PoseArray,plot_callback)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        # rospy.loginfo("the length of the msg outside %d", len(cable_array.poses))
        rospy.loginfo( temp_list[0:-1])
        
        rate.sleep()

if __name__ == "__main__":
    cable_array = PoseArray()
    temp_list=[]
    main()