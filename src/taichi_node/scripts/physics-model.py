#!/usr/bin/env python3
# from __future__ import annotations
import os 
import sys
script_dir = os.path.dirname(__file__)
# mesh_dir = os.path.joint( script_dir)
sys.path.append(script_dir)


import taichi as ti
from taichiCubeImport import data
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped
from sorted_sub import SortedSubscriber

def initialize():

    
    for i in range(n):
        CurPos[i] = [ i * seg_len, 0.0, 0.0 ]
        OldPos[i] = [ i * seg_len, 0.0, 0.0 ]
        Vel[i] = [0.0, 0.0, 0.0]


    scale = 0.05
    for i in range(len(data)):
        cube1_vertex_origin[i] = [scale* data[i][0],scale * data[i][1],scale*data[i][2]]
        cube2_vertex_origin[i] = [scale* data[i][0] + 1,scale * data[i][1],scale*data[i][2]-0.5]
    
    canvas.set_background_color((0.3, 0.3, 0.4))
    camera.position(-2.5, -2.5, 2.5)
    camera.lookat(0, 0, 0)
    camera.up(0, 0, 1)

# def sorted_CB(msg):
#     global sorted_pointset
#     sorted_pointset= msg


# def free_end_CB(msg):
#     global free_end_pose
#     free_end_pose = msg
#     print("received free end position : " + str(free_end_pose.transform.translation.x))

@ti.kernel
def update_physics(x:float):
    # global sorted_pointset, free_end_pose
    print("value in taichi kernel: " + str(x))
    pass


if __name__ == "__main__":
        
    ti.init(arch=ti.cpu)  # Alternatively, ti.init(arch=ti.cpu)

    n =128
    seg_len = 2 / n

    #######################################################################
    ########## Definition of the simulation environment ###################
    #######################################################################
    # Define cable properties
    CurPos = ti.Vector.field(3, dtype=float, shape=n)
    OldPos = ti.Vector.field(3, dtype=float, shape=n) # 3 x n x 1 
    Vel = ti.Vector.field(3, dtype=float, shape= n)

    # Define end position
    cube1_vertex_origin = ti.Vector.field(3, dtype=float, shape= len(data))
    cube2_vertex_origin = ti.Vector.field(3, dtype=float, shape= len(data))
    cube1_vertex_current = ti.Vector.field(3, dtype=float, shape= len(data))
    cube2_vertex_current = ti.Vector.field(3, dtype=float, shape= len(data))

    # Define physics param
    gravity = ti.field(dtype=float,shape=(3))
    gravity = [0,-9.8,0]


    # Define the visualization process
    window = ti.ui.Window("Test for Drawing 3d-lines", (768, 768))
    canvas = window.get_canvas()
    scene = ti.ui.Scene()
    camera = ti.ui.Camera()

    # assign the initial value
    initialize()
    # sorted_pointset = None
    # free_end_pose = None

    # Initialize the ROS node
    rospy.init_node('taichi_node')
    sorted_sub_ = SortedSubscriber()
    rate = rospy.Rate(100)  # 100hz

    while not rospy.is_shutdown():
        
        if sorted_sub_.free_end_pose is not None:
            print("value in main loop: " + str(sorted_sub_.free_end_pose.transform.translation.x))
            # x = ti.float64(free_end_pose.transform.translation.x)
            x = 0.1
            update_physics(x)

            camera.track_user_inputs(window, movement_speed=.03, hold_key=ti.ui.SPACE)
            scene.set_camera(camera)
            scene.ambient_light((1, 1, 1))
            scene.point_light(pos=(0.5, 1.5, 1.5), color=(0.1, 0.91, 0.91))

            # Draw 3d-lines in the scene
            scene.lines(CurPos, color = (1, 1, 1), width = 0.01)
            canvas.scene(scene)
            window.show()

        rate.sleep()
