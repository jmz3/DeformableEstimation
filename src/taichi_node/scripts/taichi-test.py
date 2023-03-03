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
import rospy
from geometry_msgs.msg import PoseArray


@ti.kernel
def update_Cube1(x: np.ndarray, y: np.ndarray, z:np.ndarray):
    offset = ti.Vector([x[0],y[0],z[0]])
    for i in ti.grouped(cube1_vertex):
        cube1_vertex[i] = offset

@ti.kernel
def update_Cube2(x: np.ndarray, y: np.ndarray, z:np.ndarray):
    angle = theta[0]
    # angle += 0.01
    # theta[0] = angle
    print("updating cube2")
    # offset = ti.Vector([0.01 * ti.sin(angle),0.0,0.0])
    # for i in ti.grouped(cube2_vertex):
    #     cube2_vertex[i] += offset
    offset = ti.Vector([x[x.size()-1], y[y.size()-1], z[z.size()-1]])
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

    print("update cable")
    # Simulation
    for i in ti.grouped(CurPos):
        Vel[i] = CurPos[i] - OldPos[i]
        OldPos[i] = CurPos[i]
        G = ti.Vector([deltaTime*0,deltaTime*0.0,deltaTime*-9.8])
        CurPos[i] += (Vel[i] + G) * 0.97 # Gravity Term needs to be verifiedc
        # CurPos[i] += Vel[i]
    # print(OldPos)
    print(Vel[1])


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

def taichi_CB(sorted_pointset):
    # rospy.loginfo("Successful")
    temp_x, temp_y, temp_z = [],[],[]

    for i in range(len(sorted_pointset.poses)):
        temp_x.append(sorted_pointset.poses[i].position.x)
        temp_y.append(sorted_pointset.poses[i].position.y)
        temp_z.append(sorted_pointset.poses[i].position.z)

    x = np.array(temp_x)
    y = np.array(temp_y)
    z = np.array(temp_z)



if __name__=="__main__":

    ti.init(arch=ti.cpu)  # Alternatively, ti.init(arch=ti.cpu)

    n =128
    seg_len = 2 / n
    theta = ti.field(float, 1)

    CurPos = ti.Vector.field(3, dtype=float, shape=n)
    OldPos = ti.Vector.field(3, dtype=float, shape=n)
    Vel = ti.Vector.field(3, dtype=float, shape= n)
    gravity = ti.field(dtype=float,shape=(3))
    gravity = [0,-9.8,0];
    deltaTime = 0.0001
    force = ti.Vector.field(3, dtype=float, shape= n)
    dm = 0.1
    # you need to define more parameters here, such as stiffness,dt,length of spring .......


    #initialize two cubes


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
    camera.position(-5, -5, 5)
    camera.lookat(0, 0, 0)
    camera.up(0, 0, 1)

    initialize_cable_points()

    rospy.init_node('taichi_node')
    rospy.Subscriber("Sorted", PoseArray, taichi_CB)
    rate = rospy.Rate(100)  # 10hz
    x = np.zeros(10)
    y = np.zeros(10)
    z = np.zeros(10)

    while not rospy.is_shutdown():
        
        update_cable()
        optical_reading = ti.Matrix(x,y,z)
        update_Cube1(optical_reading)
        theta[0] += 0.01
        update_Cube2(optical_reading)
        # print(CurPos[1][1])

        camera.track_user_inputs(window, movement_speed=.03, hold_key=ti.ui.SPACE)
        scene.set_camera(camera)
        scene.ambient_light((1, 1, 1))
        scene.point_light(pos=(0.5, 1.5, 1.5), color=(0.1, 0.91, 0.91))
        scene.mesh(cube1_vertex)
        scene.mesh(cube2_vertex)

        # Draw 3d-lines in the scene
        scene.lines(CurPos, color = (1, 1, 1), width = 0.01)
        scene.particles(CurPos, color = (1, 1, 1), radius = 0.01)
        canvas.scene(scene)
        window.show()
