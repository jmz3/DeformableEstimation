#!usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.animation import FuncAnimation

import numpy as np
import pandas as pd


def updateFunc(frame):
    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(2, 7)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.grid(False)

    # Set tick values
    ax.set_xticks(np.arange(-5, 5.1, 5))
    ax.set_yticks(np.arange(-5, 5.1, 5))
    ax.set_zticks(np.arange(2, 8, 5))

    # Set the pillar color as black
    # ax.plot3D([0, 0], [0, 0], [0, 8], "k")

    ax.xaxis.pane.set_edgecolor("black")
    ax.yaxis.pane.set_edgecolor("black")
    ax.zaxis.pane.set_edgecolor("black")
    # ax.xaxis.pane.fill = False
    # ax.yaxis.pane.fill = False
    # ax.zaxis.pane.fill = False
    # ax.zaxis.pane.fill = False
    # ax.zaxis.pane.set_edgecolor("black")

    # Plot the cable, the selected points are the 6th, 7th and 8th
    idx = [0, 4, 8, 12, 16, 20, 23]
    ax.plot3D(
        cable_position[frame][" Position_X"].values[idx],
        cable_position[frame][" Position_Z"].values[idx] + 3,
        cable_position[frame][" Position_Y"].values[idx],
        "or",
    )

    # Plot the whole cable
    ax.plot3D(
        cable_position[frame][" Position_X"].values,
        cable_position[frame][" Position_Z"].values + 3,
        cable_position[frame][" Position_Y"].values,
        "b",
    )


def main():
    # Import the csv file
    keypoint_position = pd.read_csv("data/unity_simulation_position.csv")

    # Separate the segments
    global cable_position
    cable_position = [
        keypoint_position.loc[i * 24 : (i + 1) * 24 - 1]
        for i in range(int(len(keypoint_position) / 24))
    ]

    global fig, ax
    fig = plt.figure(figsize=(8, 8))

    # Set fontsize of the figure
    plt.rcParams.update({"font.size": 16})
    ax = fig.add_subplot(111, projection="3d")
    ax.azim = 125
    ax.elev = 20

    ani = FuncAnimation(fig, updateFunc, frames=len(cable_position), repeat=False)

    # ani.save("animation.gif", writer="imagemagick", fps=100)  # Save as GIF
    ani.save("unity_sim.mp4", writer="ffmpeg", fps=102)  # Save as MP4
    plt.show()


if __name__ == "__main__":
    main()
