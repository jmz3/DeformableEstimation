#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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


@ti.kernel
def update_Cube1():

    offset_cube1 = ti.Vector([x[0], y[0], z[0]], float)
    for i in ti.grouped(cube1_vertex):
        cube1_vertex[i] = offset_cube1

@ti.kernel
def update_Cube2():
    print("free end position : ", free_end[0])
    print(offset[0])
    for i in ti.grouped(cube2_vertex):
        cube2_vertex[i] = offset


@ti.func
def euclidean_dist(point_a, point_b) -> float:

    d_square =  (point_a[0] - point_b[0]) * (point_a[0] - point_b[0]) +\
                (point_a[1] - point_b[1]) * (point_a[1] - point_b[1]) +\
                (point_a[2] - point_b[2]) * (point_a[2] - point_b[2])

    d = ti.math.sqrt(d_square)
    return d
    


@ti.kernel
def initialize_cable_points():

    for i in range(n):
        CurPos[i] = [ i * seg_len, 0.0, 0.0 ]
        OldPos[i] = [ i * seg_len, 0.0, 0.0 ]
        Vel[i] = [0.0, 0.0, 0.0]


@ti.kernel
def update_cable():

    # print("update cable")
    # Simulation
    for i in ti.grouped(CurPos):
        Vel[i] = CurPos[i] - OldPos[i]
        OldPos[i] = CurPos[i]
        G = ti.Vector([deltaTime*0,deltaTime*0.0,deltaTime*-9.8])
        CurPos[i] += (Vel[i] + G) * 0.97 # Gravity Term needs to be verifiedc
        # CurPos[i] += Vel[i]
    # print(OldPos)
    # print(Vel[1])


    # Constraints
    # Starting Point is Fixed
    loop_count = 0
    while loop_count < 100:
        loop_count += 1
        CurPos[0] = [0.0, 0.0, 0.0]
        CurPos[n-1] = cube2_vertex[1]
        for i in range(n-1):
            first_seg = CurPos[i]
            # print("first_seg is :",first_seg)

            if ti.math.isnan(first_seg[0]):
                ti.TaichiTypeError("first_seg is nan")
            
            eu_dist = euclidean_dist(CurPos[i],CurPos[i+1])
            # print("eu_dist is :",eu_dist)
            error = eu_dist - seg_len
            direction = (CurPos[i+1] - CurPos[i]) / eu_dist

            # first_seg += error * 0.5 * direction
            # second_seg -= error * 0.5 * direction

            CurPos[i] += error * 0.5 * direction
            CurPos[i+1] -= error * 0.5 * direction

            # test whether the distance is corrected
            eu_dist = euclidean_dist(CurPos[i],CurPos[i+1])
            # print(eu_dist)
 


@ti.kernel
def boundary_condition():
    pass
    #保持你的绳子的两个节点跟随fixed的固定运动
    #直接把x赋值为你希望他所在位置

@ti.kernel
def compute_force():
    pass

def sorted_CB(sorted_pointset):
    # rospy.loginfo("Successful")
    
    for i in range(len(sorted_pointset.poses)):
        x.append(0.001 * sorted_pointset.poses[i].position.x)
        y.append(0.001 * sorted_pointset.poses[i].position.y)
        z.append(0.001 * sorted_pointset.poses[i].position.z)

    


def free_end_CB(free_end_point):
    free_end[0] = free_end_point.transform.translation.x
    free_end[1] = free_end_point.transform.translation.y
    free_end[2] = free_end_point.transform.translation.z
    print("free end position : ", free_end)




if __name__=="__main__":

    ti.init(arch=ti.cpu)  # Alternatively, ti.init(arch=ti.cpu)

    n =128
    seg_len = 2 / n
    theta = ti.field(float, 1)

    CurPos = ti.Vector.field(3, dtype=float, shape=n)
    OldPos = ti.Vector.field(3, dtype=float, shape=n) # 3 x n x 1 
    Vel = ti.Vector.field(3, dtype=float, shape= n)
    gravity = ti.field(dtype=float,shape=(3))
    gravity = [0,-9.8,0]
    deltaTime = 0.0001
    force = ti.Vector.field(3, dtype=float, shape= n)
    dm = 0.1

    
    # you need to define more parameters here, such as stiffness,dt,length of spring .......


    cube1_vertex = ti.Vector.field(3, dtype=float, shape= len(data))
    cube2_vertex = ti.Vector.field(3, dtype=float, shape= len(data))
    scale = 0.05
    for i in range(len(data)):
        cube1_vertex[i] = [scale* data[i][0],scale * data[i][1],scale*data[i][2]]
        cube2_vertex[i] = [scale* data[i][0] + 1,scale * data[i][1],scale*data[i][2]-0.5]



    window = ti.ui.Window("Test for Drawing 3d-lines", (768, 768))
    canvas = window.get_canvas()
    canvas.set_background_color((0.3, 0.3, 0.4))
    scene = ti.ui.Scene()
    camera = ti.ui.Camera()
    camera.position(-2.5, -2.5, 2.5)
    camera.lookat(0, 0, 0)
    camera.up(0, 0, 1)

    initialize_cable_points()

    x, y, z = [], [], []
    free_end = np.array([0,0,0], dtype=float)
    offset = ti.Vector([free_end[0],free_end[1],free_end[2]])
    OpticalReading = ti.Vector.field(3, dtype=float, shape= 1)
    rospy.init_node('taichi_node')

    rospy.Subscriber("Sorted", PoseArray, sorted_CB)
    rospy.Subscriber("/NDI/PointerNew/measured_cp", TransformStamped, free_end_CB)
    rate = rospy.Rate(100)  # 10hz



    # for i in ti.grouped(OpticalReading):
    #     OpticalReading[i] = [x[i], y[i], z[i]]
    while not rospy.is_shutdown():  
        # print(x)
        update_cable()

        # constrain the cube when moving signal is received
        if rospy.get_param('/ReadingCatched') and len(x) != 0:
            update_Cube1()
            # correct or initialize the cable shape via optical reading
        else:
            # run purely on simulation
            pass


        # print("free end position in update func: ", free_end[0])
        offset = [free_end[0], free_end[1], free_end[2]]
        update_Cube2()


        camera.track_user_inputs(window, movement_speed=.03, hold_key=ti.ui.SPACE)
        scene.set_camera(camera)
        scene.ambient_light((1, 1, 1))
        scene.point_light(pos=(0.5, 1.5, 1.5), color=(0.1, 0.91, 0.91))
        scene.mesh(cube1_vertex)
        scene.mesh(cube2_vertex)

        # Draw 3d-lines in the scene
        scene.lines(CurPos, color = (1, 1, 1), width = 0.01)
        canvas.scene(scene)
        window.show()

        if(rospy.has_param("/marker_num")):
            num_of_markers = rospy.get_param("/marker_num")
            OpticalReading = ti.Vector.field(3,dtype=float,shape=num_of_markers)
            
            if len(x) != 0:
                for i in range(len(x)):
                    OpticalReading[i] = [x[i], y[i], z[i]]

                scene.particles(OpticalReading, color = (1, 1, 1), radius = 0.01)
            
            

        x.clear()
        y.clear()
        z.clear()
        rate.sleep()

