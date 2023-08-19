import rospy
import time
import pcl
import pcl.pcl_visualization
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points_list(
        data, field_names=("x", "y", "z"), skip_nans=True
    )
    pc_list = []
    for p in gen:
        pc_list.append([p[0], p[1], p[2]])

    p = pcl.PointCloud()
    p.from_list(pc_list)

    visual = pcl.pcl_visualization.PCLVisualizering()
    visual.AddPointCloud(p, b"cloud", 0)
    visual.SpinOnce(10000)
    visual.RemovePointCloud(b"cloud", 0)


if __name__ == "__main__":
    rospy.init_node("pcl_listener", anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback_pointcloud)
    rospy.spin()
