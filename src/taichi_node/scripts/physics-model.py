#!/usr/bin/env python3
from __future__ import annotations
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

    rospy.init_node('taichi_node')
    rospy.Subscriber("Sorted", PoseArray, sorted_CB)
    rospy.Subscriber("/NDI/PointerNew/measured_cp", TransformStamped, free_end_CB)
    rate = rospy.Rate(100)  # 10hz
    # assign the initial value
    initialize()