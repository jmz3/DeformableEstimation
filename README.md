# DeformableEstimation

This repo is made for estimating the center of mass for a linear deformable object in the real-time. The whole package is built via ROS.


## Develop Log
### 2023 02 06
<p>Tried with ros interpolation library ecl. Not working. Deprecated packages and insufficient support of CMake guide and other usage.</p>

### 2023 02 19
<p>
Fixed matplotlib-cpp bugs.
You have to specify the figure index when you call plot3 func.</p>

### 2023 02 27
<p>
Tested on data recorded using rosbag.
Add Functionality for asking whether to include the start point in the Optical Reading Set.</p>

### 2023 03 03
Integrated ROS with taichi package. To avoid the conflicts between the anaconda env and ros default env, we can simply install taichi on the base python environment where the ros is installed.

Of course there is another more elegant way of doing this, which is install ros-noetic via conda install command. The introduction to the repo can be found [here](https://github.com/RoboStack/ros-noetic)