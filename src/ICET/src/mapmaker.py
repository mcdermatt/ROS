#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np

import std_msgs.msg as std_msgs
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from laser_geometry import LaserProjection

class MapMaker():
    """mapmaker node subscribes to /point_cloud topic and attemts to 
        register multiple point clouds to single map"""
    
    def __init__(self, scan_topic="point_cloud"):
        # self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.on_scan)
        self.laser_projector = LaserProjection()

        rospy.init_node('mapmaker', anonymous=True)
        self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan) #pc2 tutorial

    def on_scan(self, scan):
        # https://answers.ros.org/question/202787/using-pointcloud2-data-getting-xy-points-in-python/

        gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
        # self.xyz_generator = gen

        xyz = []
        for p in gen:
            xyz.append(p)
        self.xyz = np.array(xyz)

        rospy.loginfo("Got scan!")
        print(self.xyz[:10])

if __name__ == '__main__':
    m = MapMaker()

    while not rospy.is_shutdown():

        rospy.spin()