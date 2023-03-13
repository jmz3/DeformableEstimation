#!/usr/bin/env python3
from fabrik.fabrikSolver import FabrikSolver3D
import matplotlib.pyplot as plt
import numpy as np

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


class VirtualRobot():
    '''
    robot_pos is a class that contains the robot's current position and orientation
    fk is a function that uses the robot's joint angles to calculate the end effector position
    ik is a function that uses the end effector position to calculate the robot's joint angles

    '''
    def __init__(
            self, 
            seg_len: float, 
            seg_dir: list, 
            robot_base: list
            ) -> None:
        
        '''
        Param
        ----------------
        seg_len: a list of the robot's segment lengths
        seg_dir: a list of the robot's segment directions
        robot_base: a list of the robot's base position

        '''

        self.seg_len = seg_len # length of each segment
        self.seg_dir = seg_dir # direction of each segment
        self.robot = FabrikSolver3D(marginOfError=0.001)
        self.robot.basePoint = robot_base # base position of the robot
        
        self.robot_init()
        # self.sub_robot = rospy.Subscriber("/robot_pose", TransformStamped, self.robot_callback)
        pass

    def robot_init(self):
        '''
        Initialize the robot's position and orientation
        '''
        for i in range(len(self.seg_dir)):
            self.robot.addSegment(self.seg_len, 0.0, 0.0)
        

    def fk(self):
        pass

    def ik(self, target: list, returnFlag: bool = False) ->list:
        '''
        inverse kinematics implementation is done using the fabrik algorithm

        Param
        ----------------
        target:  is a list of the end effector's position
        returnFlag: if true, return the robot's joint positions

        Return
        ----------------
        a list of the robot's joint angles

        '''
        self.robot.compute(target[0], target[1], target[2])

        if returnFlag:
            x ,y, z = [],[],[]
            x.append(self.robot.basePoint[0])
            y.append(self.robot.basePoint[1])
            z.append(self.robot.basePoint[2])

            for segment in self.robot.segments:
                x.append(segment.point[0])
                y.append(segment.point[1])
                z.append(segment.point[2])

            return x, y, z
        
        else:
            return None


def distance(x,y,z):
    return np.sqrt(x**2 + y**2 + z**2)

def test_fab():
    arm = FabrikSolver3D()

    arm.addSegment(0.25, 0.0, 0.0)
    arm.addSegment(0.25, 0.0, 0.0)
    arm.addSegment(0.25, 0.0, 0.0)
    arm.addSegment(0.25, 0.0, 0.0)
    arm.basePoint = [0.1,0.1,0.1]
    # arm.plot()
    x ,y, z = [],[],[]
    x.append(arm.basePoint[0])
    y.append(arm.basePoint[1])
    z.append(arm.basePoint[2])
    for segment in arm.segments:
        x.append(segment.point[0])
        y.append(segment.point[1])
        z.append(segment.point[2])
        print(distance(x[-1] - x[-2], y[-1] - y[-2], z[-1] - z[-2]))
        # print(segment.point)

    

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(x, y, z, 'gray')
    x, y, z = [],[],[]
    x.append(arm.basePoint[0])
    y.append(arm.basePoint[1])
    z.append(arm.basePoint[2])

    arm.compute(0.1, 0.300, 0.50)
    for segment in arm.segments:
        x.append(segment.point[0])
        y.append(segment.point[1])
        z.append(segment.point[2])
        print(distance(x[-1] - x[-2], y[-1] - y[-2], z[-1] - z[-2]))
        # print(segment.point)

    ax.plot3D(x, y, z, 'red')
    arm.plot()
    

if __name__ == "__main__":
    # test_fab()
    robot = VirtualRobot(0.5, [0,0,0,0,0], [0.20,0.20,0.20])
    robot.ik([1.1,2.1,1.1])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # robot.robot.plot()

    x ,y, z = [],[],[]
    x.append(robot.robot.basePoint[0])
    y.append(robot.robot.basePoint[1])
    z.append(robot.robot.basePoint[2])
    for segment in robot.robot.segments:
        x.append(segment.point[0])
        y.append(segment.point[1])
        z.append(segment.point[2])
        print(distance(x[-1] - x[-2], y[-1] - y[-2], z[-1] - z[-2]))
        # print(segment.point)

    ax.plot3D(x, y, z, 'gray')

    plt.show()
    # ax.plot3D(x, y, z, 'red')
    # arm.plot()

