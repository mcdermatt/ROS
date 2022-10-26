#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from ICET.msg import Num #using custom message type

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

        self.etc_sub = rospy.Subscriber('lidar_info', Num, self.get_info)

        self.mapPub = rospy.Publisher('map', PointCloud2, queue_size = 10)

        self.map_xyz = np.array([[0., 0., 0.]])
        #TODO: make this 6DOF
        self.global_pose = np.array([0., 0., 0.])

        r = 10
        self.rate = rospy.Rate(r)

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        Args:
            points: Nx7 array of xyz positions (m) and rgba colors (0..1)
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyzrgba')]

        header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

    def update_map(self):
        """add new scan to HD map"""

        #Clear map if restart == True
        print("restart: ", self.scan_data.restart)
        if self.scan_data.restart == True:
            self.map_xyz = np.array([[0., 0., 0.]])
            self.global_pose = np.array([0., 0. ,0.])

        #get transformation to align new cloud with existing map
        # (temp -- just going to use the transform provided in custom <Num> msg)
        trans = self.scan_data.true_transform[:3]
        print("trans:", trans)
        self.global_pose = self.global_pose + trans

        # self.newscan_xyz = self.newscan_xyz[self.newscan_xyz[:,2] > -1.65] #remove ground plane
        self.newscan_xyz = self.newscan_xyz[np.random.choice(len(self.newscan_xyz), size = 10000)]  #downsample

        #add two clouds together to update Map
        self.map_xyz = np.append(self.map_xyz, self.newscan_xyz + self.global_pose, axis = 0)
        # print(self.map_xyz)

        #publish updated map
        #white PC
        self.mapPub.publish(self.point_cloud(self.map_xyz, 'map'))
        #color by z height
        # colors = np.zeros([np.shape(self.map_xyz)[0], 4])
        # colors[:,3] = self.map_xyz[:,2]
        # xyz_with_color = np.append(self.map_xyz, colors, axis = 1)
        # self.mapPub.publish(self.point_cloud(xyz_with_color, 'map')) 

    def get_info(self, data):
        """ Gets Lidar info from custom Num msg """

        # rospy.loginfo("got data!")
        self.scan_data = data
        print("frame idx:", data.frame)

    def on_scan(self, scan):
        # https://answers.ros.org/question/202787/using-pointcloud2-data-getting-xy-points-in-python/

        gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
        # self.xyz_generator = gen

        xyz = []
        for p in gen:
            xyz.append(p)
        self.newscan_xyz = np.array(xyz)
        # print("newscan_xyz", np.shape(self.newscan_xyz))

        # rospy.loginfo("Got scan!")
        # print(self.newscan_xyz[:10])

        self.update_map()

if __name__ == '__main__':
    m = MapMaker()

    while not rospy.is_shutdown():

        rospy.spin()