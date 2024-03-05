# Realtime Robust Shape Estimation of Deformable Linear Object

This repo is made for estimation the shape and center of mass for a linear deformable object in the real-time with ROS functionalities. This package is a part of the project of adaptive robotic TMS system developed by [BIGSS Lab](https://bigss.lcsr.jhu.edu/) and [VOR Lab](https://vorlab.jhu.edu/) at Johns Hopkins University.

## How to use
Clone this repo to the `root` (not `root/src`!) of your catkin workspace, and then run `catkin build` to build the package.

* Run the Sorting and Interpolation Node

This is the main node of the package. It will subscribe to the topics published by the optical tracker and the RealSense camera, and then publish the estimated center of mass and the shape of the deformable object.

```bash
rosrun interp_cable interp_cable_node
```

This node will subscribe to the topics published by the optical tracker and the RealSense camera, and then publish the estimated center of mass and the shape of the deformable object.

The predefined topics are:
- `/NDI/measured_cp_array`, with type of `geometry_msgs::PoseArray`for the optical tracker
- `/camera/color/image_raw` for the RealSense camera

If you want to run this node based on your device, you can remap the topics from your device to the predefined topics or modify the source code to subscribe to the topics you want.

* Run the calibration node

This node is used to calibrate the transformation between the optical tracker and the RealSense camera. It will subscribe to the topics published by the optical tracker and the RealSense camera, and then publish the transformation between the two frames.

```bash
chmod +x CameraCalibration.py
rosrun RS_projection CameraCalibration.py
```

## License
This repo is under the MIT License. See the [LICENSE](LICENSE) file for the full license text. 

## Citation
This work has been submitted to the [IEEE International Conference on Robotics and Automation (ICRA) 2024](https://2024.ieee-icra.org/). If you find this work helpful, please consider citing it:
<!-- 
```bibtex
@inproceedings{zhang2024realtime,
    title={Realtime Robust Shape Estimation of Deformable Linear Object},
    author={Zhang, Jiaming and others},
    booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
    year={2024}
}
``` -->